"""Transfer Learning - transfers knowledge between machines with different configurations."""

from typing import Any
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode


class TransferLearningNode(MiracleLifecycleNode):
    """Cross-machine knowledge transfer."""
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('transfer_learning', criticality=self.CRITICALITY_LOW, **kwargs)

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'source_domain': {'default': '', 'type': str},
            'adaptation_epochs': {'default': 10, 'type': int, 'range': (1, 1000)},
        })
        self.get_logger().info("Transfer learning configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = TransferLearningNode()
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
