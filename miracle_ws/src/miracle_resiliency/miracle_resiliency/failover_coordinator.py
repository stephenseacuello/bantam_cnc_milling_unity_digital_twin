"""
Failover Coordinator Node.

Manages hot/warm/cold standby nodes and coordinates failover
when primary nodes fail. Implements leader election for critical services.
"""

from typing import Any, Dict, Optional
from dataclasses import dataclass
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import NodeFailure, RecoveryRequest
from miracle_msgs.srv import TriggerFailover


@dataclass
class StandbyNode:
    """A standby node ready for failover."""
    node_name: str
    standby_type: str  # HOT, WARM, COLD
    primary_for: str
    ready: bool = False
    activated: bool = False


class FailoverCoordinatorNode(MiracleLifecycleNode):
    """Coordinates failover between primary and standby nodes.

    Parameters:
        failover_timeout_sec (float): Max time for failover completion.
        auto_failover (bool): Enable automatic failover.

    Subscribed Topics:
        /miracle/resiliency/failures (NodeFailure): Node failures.

    Service Servers:
        ~/trigger_failover (TriggerFailover): Manual failover trigger.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'failover_coordinator',
            criticality=self.CRITICALITY_CRITICAL,
            **kwargs,
        )
        self._standbys: Dict[str, StandbyNode] = {}
        self._failure_sub = None
        self._failover_srv = None
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure failover coordinator."""
        self.declare_and_validate_parameters({
            'failover_timeout_sec': {
                'default': 10.0, 'type': float, 'range': (1.0, 120.0),
            },
            'auto_failover': {'default': True, 'type': bool},
        })

        self._failure_sub = self.create_subscription(
            NodeFailure, '/miracle/resiliency/failures',
            self._on_failure, QoSProfiles.alert(),
        )

        self._failover_srv = self.create_service(
            TriggerFailover, 'trigger_failover',
            self._handle_failover, callback_group=self.service_callback_group,
        )

        self.get_logger().info("Failover coordinator configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("Failover coordinator activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_failure(self, msg: NodeFailure) -> None:
        """Handle node failure for potential failover."""
        if msg.state != 'FAILED':
            return

        auto = self.get_parameter('auto_failover').value
        if not auto:
            return

        with self._lock:
            for standby in self._standbys.values():
                if standby.primary_for == msg.node_name and not standby.activated:
                    self._execute_failover(msg.node_name, standby)
                    break

    def _execute_failover(self, failed: str, standby: StandbyNode) -> None:
        """Execute failover to standby node."""
        standby.activated = True
        self.get_logger().info(
            f"Failover: {failed} -> {standby.node_name} ({standby.standby_type})"
        )

    def _handle_failover(
        self, request: TriggerFailover.Request, response: TriggerFailover.Response,
    ) -> TriggerFailover.Response:
        """Handle manual failover request."""
        with self._lock:
            for standby in self._standbys.values():
                if standby.primary_for == request.failed_node and not standby.activated:
                    self._execute_failover(request.failed_node, standby)
                    response.success = True
                    response.backup_node = standby.node_name
                    response.message = 'Failover completed'
                    return response

        response.success = False
        response.backup_node = ''
        response.message = 'No standby available'
        return response

    def register_standby(
        self, standby_name: str, primary_name: str, standby_type: str = 'WARM'
    ) -> None:
        """Register a standby node."""
        with self._lock:
            self._standbys[standby_name] = StandbyNode(
                node_name=standby_name,
                standby_type=standby_type,
                primary_for=primary_name,
                ready=True,
            )
        self.get_logger().info(
            f"Standby registered: {standby_name} for {primary_name} ({standby_type})"
        )


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = FailoverCoordinatorNode()
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
