"""Coalition Former - forms agent coalitions for complex tasks requiring multiple agents."""

from typing import Any, Dict, List
import threading
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode


class CoalitionFormerNode(MiracleLifecycleNode):
    """Forms coalitions of agents for multi-machine tasks.

    Parameters:
        max_coalition_size (int): Max agents per coalition.
    """
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('coalition_former', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._coalitions: Dict[str, List[str]] = {}
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'max_coalition_size': {'default': 5, 'type': int, 'range': (2, 20)},
        })
        self.get_logger().info("Coalition former configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("Coalition former activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def form_coalition(self, task_id: str, agents: List[str]) -> str:
        max_size = self.get_parameter('max_coalition_size').value
        agents = agents[:max_size]
        with self._lock:
            self._coalitions[task_id] = agents
        self.get_logger().info(f"Coalition formed for {task_id}: {agents}")
        return task_id


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = CoalitionFormerNode()
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
