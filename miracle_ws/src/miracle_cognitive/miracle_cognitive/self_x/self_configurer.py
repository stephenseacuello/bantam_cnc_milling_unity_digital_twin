"""Self-Configurer - automatically configures system for new machines/jobs."""

from typing import Any
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode


class SelfConfigurerNode(MiracleLifecycleNode):
    """Self-configuration for plug-and-produce capabilities."""
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('self_configurer', criticality=self.CRITICALITY_MEDIUM, **kwargs)

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'auto_discover': {'default': True, 'type': bool},
            'config_template_path': {'default': '', 'type': str},
        })
        self.get_logger().info("Self-configurer configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = SelfConfigurerNode()
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
