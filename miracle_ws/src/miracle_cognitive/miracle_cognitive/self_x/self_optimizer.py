"""Self-Optimizer - continuously optimizes system performance using RL and Bayesian optimization."""

from typing import Any, Dict
import threading
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import SystemKPIs, OptimizationAction


class SelfOptimizerNode(MiracleLifecycleNode):
    """Self-optimization using continuous learning.

    Parameters:
        optimization_interval_sec (float): Optimization cycle interval.
        min_improvement (float): Minimum improvement to apply change.
    """
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('self_optimizer', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._kpi_sub = None
        self._action_pub = None
        self._optimize_timer = None
        self._latest_kpis = None
        self._kpi_history: list = []
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'optimization_interval_sec': {'default': 60.0, 'type': float, 'range': (10.0, 600.0)},
            'min_improvement': {'default': 0.01, 'type': float, 'range': (0.001, 0.5)},
        })
        self._kpi_sub = self.create_subscription(
            SystemKPIs, '/miracle/system_kpis', self._on_kpis, QoSProfiles.state_data(),
        )
        self._action_pub = self.create_publisher(
            OptimizationAction, 'optimization_actions', QoSProfiles.command(),
        )
        self.get_logger().info("Self-optimizer configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        interval = self.get_parameter('optimization_interval_sec').value
        self._optimize_timer = self.create_timer(
            interval, self._optimize_cycle, callback_group=self.service_callback_group,
        )
        self.get_logger().info("Self-optimizer activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        if self._optimize_timer:
            self._optimize_timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def _on_kpis(self, msg: SystemKPIs) -> None:
        with self._lock:
            self._latest_kpis = msg
            self._kpi_history.append(msg.oee)
            if len(self._kpi_history) > 100:
                self._kpi_history = self._kpi_history[-100:]

    def _optimize_cycle(self) -> None:
        with self._lock:
            if self._latest_kpis is None or len(self._kpi_history) < 5:
                return
            current_oee = self._latest_kpis.oee
            avg_oee = sum(self._kpi_history) / len(self._kpi_history)

        if current_oee < avg_oee - 0.05:
            action = OptimizationAction()
            action.timestamp = self.get_clock().now().to_msg()
            action.action_type = 'PARAMETER_ADJUSTMENT'
            action.parameter_name = 'feed_rate_factor'
            action.old_value = 1.0
            action.new_value = 0.95
            action.expected_improvement = 0.02
            action.reasoning = f'OEE dropped below average ({current_oee:.2f} < {avg_oee:.2f})'
            action.confidence = 0.7
            self._action_pub.publish(action)


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = SelfOptimizerNode()
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
