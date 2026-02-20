"""RL Environment - simulated manufacturing environment for RL training."""

from typing import Any
import numpy as np
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn


class RLEnvironmentNode(MiracleLifecycleNode):
    """Simulated CNC environment for RL training."""
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('rl_environment', criticality=self.CRITICALITY_LOW, **kwargs)

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'episode_length': {'default': 100, 'type': int, 'range': (10, 10000)},
        })
        self.get_logger().info("RL environment configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = RLEnvironmentNode()
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
