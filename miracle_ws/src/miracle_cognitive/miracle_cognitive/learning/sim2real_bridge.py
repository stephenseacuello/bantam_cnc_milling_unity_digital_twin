"""Sim2Real Bridge - transfers learned policies from simulation to physical machines."""

from typing import Any
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode


class Sim2RealBridgeNode(MiracleLifecycleNode):
    """Bridges simulation-trained policies to real machines."""
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('sim2real_bridge', criticality=self.CRITICALITY_MEDIUM, **kwargs)

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'domain_randomization': {'default': True, 'type': bool},
            'adaptation_rate': {'default': 0.01, 'type': float, 'range': (0.001, 1.0)},
        })
        self.get_logger().info("Sim2Real bridge configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = Sim2RealBridgeNode()
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
