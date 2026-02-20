"""Active Learning - identifies most informative samples for model improvement."""

from typing import Any
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode


class ActiveLearningNode(MiracleLifecycleNode):
    """Active learning for efficient data collection."""
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('active_learning', criticality=self.CRITICALITY_LOW, **kwargs)

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'uncertainty_threshold': {'default': 0.5, 'type': float, 'range': (0.0, 1.0)},
            'query_budget': {'default': 100, 'type': int, 'range': (1, 10000)},
        })
        self.get_logger().info("Active learning configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = ActiveLearningNode()
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
