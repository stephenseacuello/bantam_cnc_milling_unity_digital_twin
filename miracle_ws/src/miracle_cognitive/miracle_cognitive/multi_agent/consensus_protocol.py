"""Consensus Protocol - implements distributed consensus for multi-agent decisions."""

from typing import Any, Callable, Dict, List, Optional
import time
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
        self._proposal_timestamps: Dict[str, float] = {}
        self._timeout_timers: Dict[str, threading.Timer] = {}
        self._result_callbacks: Dict[str, Optional[Callable]] = {}
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

    def propose(self, proposal_id: str, value: Any, total_agents: int,
                on_result: Optional[Callable[[str, bool], None]] = None) -> None:
        """Submit a proposal for consensus voting.

        Args:
            proposal_id: Unique proposal identifier.
            value: The proposed value.
            total_agents: Total number of agents that should vote.
            on_result: Optional callback invoked with (proposal_id, reached_consensus)
                when consensus is reached or timeout expires.
        """
        timeout_sec = self.get_parameter('consensus_timeout_sec').value

        with self._lock:
            self._proposals[proposal_id] = {'value': value, 'total': total_agents}
            self._votes[proposal_id] = []
            self._proposal_timestamps[proposal_id] = time.monotonic()
            self._result_callbacks[proposal_id] = on_result

            # Start a timeout timer
            timer = threading.Timer(timeout_sec, self._on_timeout, args=[proposal_id])
            timer.daemon = True
            timer.start()
            self._timeout_timers[proposal_id] = timer

        self.get_logger().info(
            f"Proposal {proposal_id} submitted, timeout={timeout_sec}s"
        )

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

    def _on_timeout(self, proposal_id: str) -> None:
        """Handle consensus timeout for a proposal."""
        with self._lock:
            if proposal_id not in self._proposals:
                return

            reached = self._check_consensus_unlocked(proposal_id)
            elapsed = time.monotonic() - self._proposal_timestamps.get(proposal_id, 0)
            callback = self._result_callbacks.pop(proposal_id, None)

            # Clean up the timer reference
            self._timeout_timers.pop(proposal_id, None)

        if reached:
            self.get_logger().info(
                f"Proposal {proposal_id} reached consensus after {elapsed:.1f}s"
            )
        else:
            self.get_logger().warn(
                f"Proposal {proposal_id} TIMED OUT after {elapsed:.1f}s without consensus"
            )

        if callback is not None:
            try:
                callback(proposal_id, reached)
            except Exception as exc:
                self.get_logger().error(
                    f"Consensus result callback error: {exc}"
                )

    def _check_consensus_unlocked(self, proposal_id: str) -> bool:
        """Check consensus without acquiring lock (caller must hold lock)."""
        quorum = self.get_parameter('quorum_fraction').value
        if proposal_id not in self._proposals:
            return False
        total = self._proposals[proposal_id]['total']
        votes = self._votes.get(proposal_id, [])
        accepts = sum(1 for _, a in votes if a)
        return accepts >= total * quorum

    def get_proposal_age(self, proposal_id: str) -> float:
        """Return the elapsed time in seconds since a proposal was submitted."""
        with self._lock:
            ts = self._proposal_timestamps.get(proposal_id)
            if ts is None:
                return -1.0
            return time.monotonic() - ts


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
