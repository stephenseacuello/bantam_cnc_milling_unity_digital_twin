"""Consensus Protocol - implements distributed consensus for multi-agent decisions."""

from typing import Any, Dict, List
import threading
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode


class ConsensusProtocolNode(MiracleLifecycleNode):
    """Distributed consensus for multi-agent coordination.

    Parameters:
        consensus_timeout_sec (float): Timeout for consensus rounds.
        quorum_fraction (float): Fraction of agents needed for quorum.
    """
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('consensus_protocol', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._proposals: Dict[str, Dict[str, Any]] = {}
        self._votes: Dict[str, List] = {}
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'consensus_timeout_sec': {'default': 10.0, 'type': float, 'range': (1.0, 120.0)},
            'quorum_fraction': {'default': 0.67, 'type': float, 'range': (0.5, 1.0)},
        })
        self.get_logger().info("Consensus protocol configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("Consensus protocol activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def propose(self, proposal_id: str, value: Any, total_agents: int) -> None:
        with self._lock:
            self._proposals[proposal_id] = {'value': value, 'total': total_agents}
            self._votes[proposal_id] = []

    def vote(self, proposal_id: str, agent_id: str, accept: bool) -> None:
        with self._lock:
            if proposal_id in self._votes:
                self._votes[proposal_id].append((agent_id, accept))

    def check_consensus(self, proposal_id: str) -> bool:
        quorum = self.get_parameter('quorum_fraction').value
        with self._lock:
            if proposal_id not in self._proposals:
                return False
            total = self._proposals[proposal_id]['total']
            votes = self._votes.get(proposal_id, [])
            accepts = sum(1 for _, a in votes if a)
            return accepts >= total * quorum


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = ConsensusProtocolNode()
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
