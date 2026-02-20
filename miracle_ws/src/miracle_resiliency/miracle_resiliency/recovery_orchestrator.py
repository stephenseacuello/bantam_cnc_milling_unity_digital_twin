"""
Recovery Orchestrator Node.

Orchestrates multi-step recovery sequences for failed nodes,
coordinating with lifecycle management and dependency ordering.
"""

from typing import Any, Dict, List
from dataclasses import dataclass
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import RecoveryRequest, NodeFailure


@dataclass
class RecoveryAction:
    """A recovery action in progress."""
    failed_node: str
    strategy: str
    attempt: int = 0
    status: str = 'PENDING'


class RecoveryOrchestratorNode(MiracleLifecycleNode):
    """Orchestrates recovery sequences.

    Parameters:
        max_concurrent_recoveries (int): Max simultaneous recoveries.
        recovery_timeout_sec (float): Recovery attempt timeout.

    Subscribed Topics:
        /miracle/resiliency/recovery_requests (RecoveryRequest): Recovery triggers.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'recovery_orchestrator',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._recovery_sub = None
        self._active_recoveries: Dict[str, RecoveryAction] = {}
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'max_concurrent_recoveries': {
                'default': 5, 'type': int, 'range': (1, 50),
            },
            'recovery_timeout_sec': {
                'default': 30.0, 'type': float, 'range': (5.0, 300.0),
            },
        })

        self._recovery_sub = self.create_subscription(
            RecoveryRequest, '/miracle/resiliency/recovery_requests',
            self._on_recovery_request, QoSProfiles.alert(),
        )

        self.get_logger().info("Recovery orchestrator configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("Recovery orchestrator activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_recovery_request(self, msg: RecoveryRequest) -> None:
        """Handle recovery request."""
        max_concurrent = self.get_parameter('max_concurrent_recoveries').value

        with self._lock:
            if len(self._active_recoveries) >= max_concurrent:
                self.get_logger().warn("Max concurrent recoveries reached")
                return

            if msg.failed_node in self._active_recoveries:
                self.get_logger().info(
                    f"Recovery already in progress for {msg.failed_node}"
                )
                return

            action = RecoveryAction(
                failed_node=msg.failed_node,
                strategy=msg.strategy,
                attempt=msg.attempt_number,
                status='IN_PROGRESS',
            )
            self._active_recoveries[msg.failed_node] = action

        self.get_logger().info(
            f"Starting recovery for {msg.failed_node} "
            f"(strategy={msg.strategy}, attempt={msg.attempt_number})"
        )
        self._execute_recovery(action)

    def _execute_recovery(self, action: RecoveryAction) -> None:
        """Execute recovery strategy."""
        if action.strategy == 'IMMEDIATE_RESTART':
            self._immediate_restart(action)
        elif action.strategy == 'RESTART_WITH_DEPENDENCIES':
            self._restart_with_deps(action)
        elif action.strategy == 'DELAYED_RESTART':
            self._delayed_restart(action)
        else:
            self.get_logger().warn(f"Unknown strategy: {action.strategy}")

        with self._lock:
            action.status = 'COMPLETED'
            self._active_recoveries.pop(action.failed_node, None)

    def _immediate_restart(self, action: RecoveryAction) -> None:
        """Immediate restart via lifecycle transition."""
        self.get_logger().info(f"Immediate restart: {action.failed_node}")
        # In production: use lifecycle service calls to restart node

    def _restart_with_deps(self, action: RecoveryAction) -> None:
        """Restart node and its dependents."""
        self.get_logger().info(
            f"Restart with dependencies: {action.failed_node}"
        )

    def _delayed_restart(self, action: RecoveryAction) -> None:
        """Delayed restart for non-critical nodes."""
        self.get_logger().info(f"Delayed restart: {action.failed_node}")


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = RecoveryOrchestratorNode()
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
