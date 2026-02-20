"""Self-Protector - proactive security posture management."""

from typing import Any
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import SecurityAlert


class SelfProtectorNode(MiracleLifecycleNode):
    """Self-protection with proactive security adaptation."""
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('self_protector', criticality=self.CRITICALITY_HIGH, **kwargs)
        self._alert_sub = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'threat_level': {'default': 'NORMAL', 'type': str,
                             'choices': ['LOW', 'NORMAL', 'ELEVATED', 'HIGH', 'CRITICAL']},
        })
        self._alert_sub = self.create_subscription(
            SecurityAlert, '/miracle/security/alerts',
            self._on_alert, QoSProfiles.alert(),
        )
        self.get_logger().info("Self-protector configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_alert(self, msg: SecurityAlert) -> None:
        if msg.severity in ('CRITICAL', 'HIGH'):
            self.get_logger().warn(f"Elevating security posture: {msg.description}")


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = SelfProtectorNode()
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
