"""
Agent Registry Node.

Manages registration and discovery of autonomous manufacturing agents.
Tracks capabilities, status, and availability.
"""

from typing import Any, Dict, List
from dataclasses import dataclass, field
import threading

from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import Heartbeat


@dataclass
class AgentInfo:
    """Registered agent information."""
    agent_id: str
    agent_type: str
    capabilities: List[str] = field(default_factory=list)
    status: str = 'IDLE'
    current_load: float = 0.0
    tasks_completed: int = 0


class AgentRegistryNode(MiracleLifecycleNode):
    """Registry for autonomous manufacturing agents.

    Parameters:
        max_agents (int): Maximum registered agents.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('agent_registry', criticality=self.CRITICALITY_HIGH, **kwargs)
        self._agents: Dict[str, AgentInfo] = {}
        self._lock = threading.Lock()
        self._heartbeat_sub = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'max_agents': {'default': 50, 'type': int, 'range': (1, 1000)},
        })
        self._heartbeat_sub = self.create_subscription(
            Heartbeat, '/miracle/heartbeats',
            self._on_heartbeat, QoSProfiles.heartbeat(),
        )
        self.get_logger().info("Agent registry configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("Agent registry activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_heartbeat(self, msg: Heartbeat) -> None:
        agent_id = f"{msg.node_namespace}/{msg.node_name}"
        with self._lock:
            if agent_id not in self._agents:
                self._agents[agent_id] = AgentInfo(
                    agent_id=agent_id, agent_type=msg.criticality,
                )

    def register_agent(self, agent_id: str, agent_type: str, capabilities: List[str]) -> bool:
        max_a = self.get_parameter('max_agents').value
        with self._lock:
            if len(self._agents) >= max_a:
                return False
            self._agents[agent_id] = AgentInfo(
                agent_id=agent_id, agent_type=agent_type, capabilities=capabilities,
            )
        return True

    def get_capable_agents(self, required_capabilities: List[str]) -> List[AgentInfo]:
        with self._lock:
            return [
                a for a in self._agents.values()
                if all(c in a.capabilities for c in required_capabilities)
            ]


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = AgentRegistryNode()
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
