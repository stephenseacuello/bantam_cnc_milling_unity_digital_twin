"""Coalition Former - forms agent coalitions for complex tasks requiring multiple agents."""

from typing import Any, Dict, List
from dataclasses import dataclass, field
import threading
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode


@dataclass
class AgentCapability:
    """Describes an agent's capabilities and current status."""
    agent_id: str
    capabilities: List[str] = field(default_factory=list)
    availability: float = 1.0  # 0.0 = unavailable, 1.0 = fully available
    load: float = 0.0  # 0.0 = idle, 1.0 = fully loaded


class CoalitionFormerNode(MiracleLifecycleNode):
    """Forms coalitions of agents for multi-machine tasks.

    Parameters:
        max_coalition_size (int): Max agents per coalition.
    """
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('coalition_former', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._coalitions: Dict[str, List[str]] = {}
        self._agent_registry: Dict[str, AgentCapability] = {}
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

    def register_agent(self, agent_id: str, capabilities: List[str],
                        availability: float = 1.0, load: float = 0.0) -> None:
        """Register an agent with its capabilities and status."""
        with self._lock:
            self._agent_registry[agent_id] = AgentCapability(
                agent_id=agent_id,
                capabilities=capabilities,
                availability=availability,
                load=load,
            )
        self.get_logger().info(
            f"Agent registered: {agent_id} capabilities={capabilities}"
        )

    def update_agent_status(self, agent_id: str, availability: float,
                            load: float) -> None:
        """Update an agent's availability and load."""
        with self._lock:
            if agent_id in self._agent_registry:
                self._agent_registry[agent_id].availability = availability
                self._agent_registry[agent_id].load = load

    def form_coalition(self, task_id: str, agents: List[str],
                       required_capabilities: List[str] | None = None) -> str:
        """Form a coalition, filtering by capability match and sorting by availability.

        Args:
            task_id: Unique task identifier.
            agents: Candidate agent IDs.
            required_capabilities: If provided, only agents with matching
                capabilities are included.

        Returns:
            The task_id for the formed coalition.
        """
        max_size = self.get_parameter('max_coalition_size').value

        with self._lock:
            if required_capabilities:
                # Filter agents by capability match
                qualified = []
                for agent_id in agents:
                    entry = self._agent_registry.get(agent_id)
                    if entry is None:
                        continue
                    if any(cap in entry.capabilities
                           for cap in required_capabilities):
                        qualified.append(agent_id)
                # Sort by availability (descending), then by load (ascending)
                qualified.sort(
                    key=lambda aid: (
                        -self._agent_registry[aid].availability,
                        self._agent_registry[aid].load,
                    )
                )
                agents = qualified
            else:
                # Sort registered agents by availability if they are known
                def sort_key(aid: str):
                    entry = self._agent_registry.get(aid)
                    if entry:
                        return (-entry.availability, entry.load)
                    return (0.0, 0.0)
                agents = sorted(agents, key=sort_key)

            agents = agents[:max_size]
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
