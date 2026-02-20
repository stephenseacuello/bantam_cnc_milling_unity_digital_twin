"""
Fleet-Wide Heartbeat Aggregator.

Monitors heartbeats from all nodes and detects failures
based on configurable criticality-based timeouts.
"""

from typing import Any, Dict
from dataclasses import dataclass, field
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import Heartbeat, FleetHealth, NodeFailure, RecoveryRequest


@dataclass
class NodeInfo:
    """Tracked information about a node."""
    node_name: str
    namespace: str
    criticality: str
    last_heartbeat_ns: int = 0
    lifecycle_state: str = 'unknown'
    dependencies: list = field(default_factory=list)
    consecutive_misses: int = 0
    state: str = 'HEALTHY'
    cpu_usage: float = 0.0
    memory_usage: float = 0.0


class HeartbeatAggregatorNode(MiracleLifecycleNode):
    """Aggregates heartbeats with criticality-based timeouts.

    Parameters:
        critical_timeout_sec (float): Timeout for CRITICAL nodes.
        high_timeout_sec (float): Timeout for HIGH nodes.
        medium_timeout_sec (float): Timeout for MEDIUM nodes.
        low_timeout_sec (float): Timeout for LOW nodes.
        check_interval_sec (float): Heartbeat check interval.
        health_publish_interval_sec (float): Fleet health publish rate.

    Subscribed Topics:
        /miracle/heartbeats (Heartbeat): Heartbeats from all nodes.

    Published Topics:
        /miracle/resiliency/fleet_health (FleetHealth): Fleet health.
        /miracle/resiliency/failures (NodeFailure): Failure events.
        /miracle/resiliency/recovery_requests (RecoveryRequest): Recovery triggers.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'heartbeat_aggregator',
            criticality=self.CRITICALITY_CRITICAL,
            **kwargs,
        )
        self._nodes: Dict[str, NodeInfo] = {}
        self._nodes_lock = threading.Lock()
        self._health_pub = None
        self._failure_pub = None
        self._recovery_pub = None
        self._heartbeat_sub = None
        self._check_timer = None
        self._health_timer = None
        self._timeouts: Dict[str, float] = {}

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure aggregator."""
        params = self.declare_and_validate_parameters({
            'critical_timeout_sec': {
                'default': 0.5, 'type': float, 'range': (0.1, 10.0),
            },
            'high_timeout_sec': {
                'default': 1.0, 'type': float, 'range': (0.1, 30.0),
            },
            'medium_timeout_sec': {
                'default': 5.0, 'type': float, 'range': (0.5, 60.0),
            },
            'low_timeout_sec': {
                'default': 30.0, 'type': float, 'range': (1.0, 300.0),
            },
            'check_interval_sec': {
                'default': 0.1, 'type': float, 'range': (0.01, 5.0),
            },
            'health_publish_interval_sec': {
                'default': 1.0, 'type': float, 'range': (0.1, 30.0),
            },
        })

        self._timeouts = {
            'CRITICAL': params['critical_timeout_sec'],
            'HIGH': params['high_timeout_sec'],
            'MEDIUM': params['medium_timeout_sec'],
            'LOW': params['low_timeout_sec'],
        }

        self._health_pub = self.create_publisher(
            FleetHealth, '/miracle/resiliency/fleet_health', QoSProfiles.state_data(),
        )
        self._failure_pub = self.create_publisher(
            NodeFailure, '/miracle/resiliency/failures', QoSProfiles.alert(),
        )
        self._recovery_pub = self.create_publisher(
            RecoveryRequest, '/miracle/resiliency/recovery_requests', QoSProfiles.alert(),
        )
        self._heartbeat_sub = self.create_subscription(
            Heartbeat, '/miracle/heartbeats', self._on_heartbeat, QoSProfiles.heartbeat(),
        )

        self.get_logger().info("Heartbeat aggregator configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate monitoring."""
        check = self.get_parameter('check_interval_sec').value
        health = self.get_parameter('health_publish_interval_sec').value

        self._check_timer = self.create_timer(
            check, self._check_heartbeats, callback_group=self.realtime_callback_group,
        )
        self._health_timer = self.create_timer(
            health, self._publish_fleet_health, callback_group=self.service_callback_group,
        )

        self.get_logger().info("Heartbeat monitoring activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Stop monitoring."""
        for timer in (self._check_timer, self._health_timer):
            if timer is not None:
                timer.cancel()
        self._check_timer = None
        self._health_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_heartbeat(self, msg: Heartbeat) -> None:
        """Process incoming heartbeat."""
        full_name = f"{msg.node_namespace}/{msg.node_name}"
        now_ns = self.get_clock().now().nanoseconds

        with self._nodes_lock:
            if full_name not in self._nodes:
                self._nodes[full_name] = NodeInfo(
                    node_name=msg.node_name,
                    namespace=msg.node_namespace,
                    criticality=msg.criticality,
                    last_heartbeat_ns=now_ns,
                    lifecycle_state=msg.lifecycle_state,
                    dependencies=list(msg.dependencies),
                )
                self.get_logger().info(
                    f"Discovered node: {full_name} ({msg.criticality})"
                )
            else:
                node = self._nodes[full_name]
                node.last_heartbeat_ns = now_ns
                node.lifecycle_state = msg.lifecycle_state
                node.cpu_usage = msg.cpu_usage
                node.memory_usage = msg.memory_usage
                node.consecutive_misses = 0
                if node.state != 'HEALTHY':
                    self.get_logger().info(f"Node recovered: {full_name}")
                    node.state = 'HEALTHY'

    def _check_heartbeats(self) -> None:
        """Check all nodes for missed heartbeats."""
        now_ns = self.get_clock().now().nanoseconds

        with self._nodes_lock:
            for full_name, node in self._nodes.items():
                timeout = self._timeouts.get(node.criticality, 5.0)
                elapsed = (now_ns - node.last_heartbeat_ns) / 1e9

                if elapsed > timeout:
                    node.consecutive_misses += 1

                    if node.state == 'HEALTHY':
                        node.state = 'DEGRADED'
                        self._publish_failure(full_name, node, 'DEGRADED', elapsed)
                        self.get_logger().warn(
                            f"Node degraded: {full_name} ({elapsed:.2f}s)"
                        )
                    elif node.state == 'DEGRADED' and node.consecutive_misses > 3:
                        node.state = 'FAILED'
                        self._publish_failure(full_name, node, 'FAILED', elapsed)
                        self._trigger_recovery(full_name, node)
                        self.get_logger().error(
                            f"Node failed: {full_name} "
                            f"({node.consecutive_misses} misses)"
                        )

    def _publish_failure(
        self, full_name: str, node: NodeInfo, state: str, elapsed: float
    ) -> None:
        """Publish node failure event."""
        msg = NodeFailure()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.node_name = full_name
        msg.criticality = node.criticality
        msg.state = state
        msg.time_since_last_heartbeat = elapsed
        msg.lifecycle_state = node.lifecycle_state
        self._failure_pub.publish(msg)

    def _trigger_recovery(self, full_name: str, node: NodeInfo) -> None:
        """Trigger recovery for failed node."""
        dependents = []
        with self._nodes_lock:
            for name, info in self._nodes.items():
                if full_name in info.dependencies:
                    dependents.append(name)

        strategy = {
            'CRITICAL': 'IMMEDIATE_RESTART',
            'HIGH': 'RESTART_WITH_DEPENDENCIES',
        }.get(node.criticality, 'DELAYED_RESTART')

        msg = RecoveryRequest()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.failed_node = full_name
        msg.criticality = node.criticality
        msg.dependents = dependents
        msg.strategy = strategy
        self._recovery_pub.publish(msg)

    def _publish_fleet_health(self) -> None:
        """Publish aggregated fleet health."""
        with self._nodes_lock:
            msg = FleetHealth()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.total_nodes = len(self._nodes)
            msg.healthy_nodes = sum(1 for n in self._nodes.values() if n.state == 'HEALTHY')
            msg.degraded_nodes = sum(1 for n in self._nodes.values() if n.state == 'DEGRADED')
            msg.failed_nodes = sum(1 for n in self._nodes.values() if n.state == 'FAILED')
            msg.critical_healthy = sum(
                1 for n in self._nodes.values()
                if n.criticality == 'CRITICAL' and n.state == 'HEALTHY'
            )
            msg.critical_total = sum(
                1 for n in self._nodes.values() if n.criticality == 'CRITICAL'
            )
            msg.failed_node_names = [
                name for name, n in self._nodes.items() if n.state == 'FAILED'
            ]
            msg.degraded_node_names = [
                name for name, n in self._nodes.items() if n.state == 'DEGRADED'
            ]
            msg.health_score = msg.healthy_nodes / max(msg.total_nodes, 1)

        self._health_pub.publish(msg)


def main(args=None):
    """Entry point for the heartbeat aggregator node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = HeartbeatAggregatorNode()
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
