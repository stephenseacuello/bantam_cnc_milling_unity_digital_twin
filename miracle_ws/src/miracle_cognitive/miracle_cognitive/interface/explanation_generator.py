"""Explanation Generator - generates human-readable explanations of AI decisions."""

from typing import Any
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode


class ExplanationGeneratorNode(MiracleLifecycleNode):
    """Generates explanations for autonomous decisions (XAI)."""
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('explanation_generator', criticality=self.CRITICALITY_LOW, **kwargs)

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'detail_level': {'default': 'medium', 'type': str,
                             'choices': ['brief', 'medium', 'detailed']},
        })
        self.get_logger().info("Explanation generator configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def explain_anomaly(self, anomaly_type: str, factors: list) -> str:
        return f"Detected {anomaly_type} based on: {', '.join(factors)}"

    def explain_optimization(self, action: str, reasoning: str) -> str:
        return f"Optimization '{action}': {reasoning}"


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = ExplanationGeneratorNode()
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
