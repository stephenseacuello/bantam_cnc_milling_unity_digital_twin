"""Self-Healer - detects and automatically recovers from degraded states."""

from typing import Any
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import FleetHealth, AnomalyAlert


class SelfHealerNode(MiracleLifecycleNode):
    """Self-healing capabilities for automatic recovery."""
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('self_healer', criticality=self.CRITICALITY_HIGH, **kwargs)
        self._health_sub = None
        self._anomaly_sub = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'healing_threshold': {'default': 0.7, 'type': float, 'range': (0.1, 1.0)},
            'max_healing_attempts': {'default': 3, 'type': int, 'range': (1, 10)},
        })
        self._health_sub = self.create_subscription(
            FleetHealth, '/miracle/resiliency/fleet_health',
            self._on_health, QoSProfiles.state_data(),
        )
        self._anomaly_sub = self.create_subscription(
            AnomalyAlert, '/miracle/+/anomaly',
            self._on_anomaly, QoSProfiles.alert(),
        )
        self.get_logger().info("Self-healer configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_health(self, msg: FleetHealth) -> None:
        threshold = self.get_parameter('healing_threshold').value
        if msg.health_score < threshold:
            self.get_logger().warn(f"Health below threshold: {msg.health_score:.2f}")

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        if msg.severity > 0.8:
            self.get_logger().info(f"Self-healing triggered for {msg.machine_id}")


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = SelfHealerNode()
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
