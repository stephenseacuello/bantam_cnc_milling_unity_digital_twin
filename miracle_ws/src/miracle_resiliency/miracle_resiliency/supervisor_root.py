"""
Root Supervisor Node.

Top-level supervisor implementing Erlang-style supervision trees.
Monitors all critical subsystems and orchestrates system-wide recovery.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
from enum import Enum
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import FleetHealth, NodeFailure, RecoveryRequest


class RestartStrategy(Enum):
    ONE_FOR_ONE = 'one_for_one'
    ONE_FOR_ALL = 'one_for_all'
    REST_FOR_ONE = 'rest_for_one'


@dataclass
class SupervisedChild:
    """A supervised child process/node."""
    name: str
    criticality: str
    restart_count: int = 0
    max_restarts: int = 5
    restart_window_sec: float = 300.0
    restart_timestamps: List[float] = field(default_factory=list)
    state: str = 'RUNNING'


class SupervisorRootNode(MiracleLifecycleNode):
    """Root supervisor with Erlang-style supervision strategies.

    Parameters:
        strategy (str): Restart strategy (one_for_one, one_for_all, rest_for_one).
        max_restarts (int): Max restarts within window.
        restart_window_sec (float): Time window for restart counting.

    Subscribed Topics:
        /miracle/resiliency/fleet_health (FleetHealth): Fleet health status.
        /miracle/resiliency/failures (NodeFailure): Node failure events.

    Published Topics:
        /miracle/resiliency/recovery_requests (RecoveryRequest): Recovery commands.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'supervisor_root',
            criticality=self.CRITICALITY_CRITICAL,
            **kwargs,
        )
        self._children: Dict[str, SupervisedChild] = {}
        self._children_lock = threading.Lock()
        self._health_sub = None
        self._failure_sub = None
        self._recovery_pub = None
        self._strategy = RestartStrategy.ONE_FOR_ONE

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure root supervisor."""
        params = self.declare_and_validate_parameters({
            'strategy': {
                'default': 'one_for_one',
                'type': str,
                'choices': ['one_for_one', 'one_for_all', 'rest_for_one'],
            },
            'max_restarts': {
                'default': 5,
                'type': int,
                'range': (1, 100),
            },
            'restart_window_sec': {
                'default': 300.0,
                'type': float,
                'range': (10.0, 3600.0),
            },
        })

        self._strategy = RestartStrategy(params['strategy'])

        self._failure_sub = self.create_subscription(
            NodeFailure,
            '/miracle/resiliency/failures',
            self._on_failure,
            QoSProfiles.alert(),
        )

        self._health_sub = self.create_subscription(
            FleetHealth,
            '/miracle/resiliency/fleet_health',
            self._on_fleet_health,
            QoSProfiles.state_data(),
        )

        self._recovery_pub = self.create_publisher(
            RecoveryRequest,
            '/miracle/resiliency/recovery_requests',
            QoSProfiles.alert(),
        )

        self.get_logger().info(
            f"Root supervisor configured (strategy={self._strategy.value})"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate supervision."""
        self.get_logger().info("Root supervisor activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate supervision."""
        return TransitionCallbackReturn.SUCCESS

    def _on_failure(self, msg: NodeFailure) -> None:
        """Handle node failure event."""
        now = self.get_clock().now().nanoseconds / 1e9

        with self._children_lock:
            if msg.node_name not in self._children:
                max_restarts = self.get_parameter('max_restarts').value
                window = self.get_parameter('restart_window_sec').value
                self._children[msg.node_name] = SupervisedChild(
                    name=msg.node_name,
                    criticality=msg.criticality,
                    max_restarts=max_restarts,
                    restart_window_sec=window,
                )

            child = self._children[msg.node_name]
            child.state = msg.state

            if msg.state != 'FAILED':
                return

            # Check restart budget
            child.restart_timestamps = [
                t for t in child.restart_timestamps
                if now - t < child.restart_window_sec
            ]

            if len(child.restart_timestamps) >= child.max_restarts:
                self.get_logger().error(
                    f"Node {msg.node_name} exceeded max restarts "
                    f"({child.max_restarts} in {child.restart_window_sec}s). "
                    f"Giving up."
                )
                child.state = 'ABANDONED'
                return

            child.restart_timestamps.append(now)
            child.restart_count += 1

        # Apply restart strategy
        if self._strategy == RestartStrategy.ONE_FOR_ONE:
            self._restart_node(msg.node_name, child)
        elif self._strategy == RestartStrategy.ONE_FOR_ALL:
            self._restart_all()
        elif self._strategy == RestartStrategy.REST_FOR_ONE:
            self._restart_rest(msg.node_name)

    def _restart_node(self, node_name: str, child: SupervisedChild) -> None:
        """Request restart of a single node."""
        msg = RecoveryRequest()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.failed_node = node_name
        msg.criticality = child.criticality
        msg.strategy = 'RESTART'
        msg.reason = f'Supervised restart (attempt {child.restart_count})'
        msg.attempt_number = child.restart_count

        self._recovery_pub.publish(msg)
        self.get_logger().info(
            f"Requesting restart of {node_name} (attempt {child.restart_count})"
        )

    def _restart_all(self) -> None:
        """Restart all supervised children."""
        with self._children_lock:
            for name, child in self._children.items():
                self._restart_node(name, child)

    def _restart_rest(self, failed_name: str) -> None:
        """Restart the failed node and all nodes registered after it."""
        with self._children_lock:
            found = False
            for name, child in self._children.items():
                if name == failed_name:
                    found = True
                if found:
                    self._restart_node(name, child)

    def _on_fleet_health(self, msg: FleetHealth) -> None:
        """Monitor overall fleet health."""
        if msg.health_score < 0.5:
            self.get_logger().warn(
                f"Fleet health critical: {msg.health_score:.2f} "
                f"({msg.failed_nodes} failed nodes)"
            )


def main(args=None):
    """Entry point for the root supervisor node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = SupervisorRootNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
