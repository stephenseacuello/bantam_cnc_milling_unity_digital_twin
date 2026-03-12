"""Operator Feedback Processor — closed-loop learning from operator decisions.

Subscribes to operator feedback messages and adjusts:
- Causal inference link strengths based on action outcomes
- Explanation generator templates based on quality ratings
- Recommendation confidence based on acceptance/rejection rates
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
import time
import threading

from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import OperatorFeedback


@dataclass
class FeedbackRecord:
    """A single operator feedback event."""
    timestamp: float
    machine_id: str
    feedback_type: str
    anomaly_type: str
    rating: float
    action_taken: str
    reference_id: str


@dataclass
class RecommendationStats:
    """Statistics for a specific recommendation type."""
    anomaly_type: str
    total_shown: int = 0
    accepted: int = 0
    rejected: int = 0
    confirmed_effective: int = 0
    confirmed_ineffective: int = 0
    avg_rating: float = 0.0

    @property
    def acceptance_rate(self) -> float:
        total = self.accepted + self.rejected
        return self.accepted / total if total > 0 else 0.5

    @property
    def effectiveness_rate(self) -> float:
        total = self.confirmed_effective + self.confirmed_ineffective
        return self.confirmed_effective / total if total > 0 else 0.5


class FeedbackProcessorNode(MiracleLifecycleNode):
    """Processes operator feedback to improve system recommendations."""

    def __init__(self, **kwargs):
        super().__init__('feedback_processor', criticality=self.CRITICALITY_LOW, **kwargs)
        self._feedback_history: List[FeedbackRecord] = []
        self._feedback_lock = threading.Lock()
        self._recommendation_stats: Dict[str, RecommendationStats] = {}
        self._history_size = 10000

        # Callbacks for external integration
        self._causal_update_callback = None
        self._explanation_update_callback = None

    def _do_configure(self) -> TransitionCallbackReturn:
        params = self.declare_and_validate_parameters({
            'history_size': {'default': 10000, 'type': int, 'range': (100, 100000)},
        })
        self._history_size = params['history_size']

        self.create_subscription(
            OperatorFeedback,
            '/miracle/cognitive/operator_feedback',
            self._on_feedback,
            QoSProfiles.alert(),
        )

        self.get_logger().info("Feedback processor configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def set_causal_update_callback(self, callback):
        """Set callback for updating causal inference: callback(anomaly_type, action, effective: bool)"""
        self._causal_update_callback = callback

    def set_explanation_update_callback(self, callback):
        """Set callback for updating explanation quality: callback(anomaly_type, rating)"""
        self._explanation_update_callback = callback

    def _on_feedback(self, msg: OperatorFeedback) -> None:
        record = FeedbackRecord(
            timestamp=time.time(),
            machine_id=msg.machine_id,
            feedback_type=msg.feedback_type,
            anomaly_type=msg.anomaly_type,
            rating=msg.rating,
            action_taken=msg.action_taken,
            reference_id=msg.reference_id,
        )

        with self._feedback_lock:
            self._feedback_history.append(record)
            if len(self._feedback_history) > self._history_size:
                self._feedback_history = self._feedback_history[-self._history_size:]

        # Update stats
        stats = self._recommendation_stats.setdefault(
            msg.anomaly_type, RecommendationStats(anomaly_type=msg.anomaly_type)
        )

        if msg.feedback_type == 'RECOMMENDATION_ACCEPTED':
            stats.accepted += 1
            stats.total_shown += 1
        elif msg.feedback_type == 'RECOMMENDATION_REJECTED':
            stats.rejected += 1
            stats.total_shown += 1
        elif msg.feedback_type == 'ACTION_CONFIRMED':
            stats.confirmed_effective += 1
            if self._causal_update_callback:
                self._causal_update_callback(msg.anomaly_type, msg.action_taken, True)
        elif msg.feedback_type == 'ACTION_FAILED':
            stats.confirmed_ineffective += 1
            if self._causal_update_callback:
                self._causal_update_callback(msg.anomaly_type, msg.action_taken, False)
        elif msg.feedback_type == 'EXPLANATION_RATED':
            # Update rolling average
            n = stats.total_shown or 1
            stats.avg_rating = (stats.avg_rating * (n - 1) + msg.rating) / n
            if self._explanation_update_callback:
                self._explanation_update_callback(msg.anomaly_type, msg.rating)

        self.get_logger().info(
            f"Feedback: {msg.feedback_type} for {msg.anomaly_type} "
            f"(rating={msg.rating}, action={msg.action_taken})"
        )

    def get_recommendation_confidence(self, anomaly_type: str) -> float:
        """Get adjusted confidence for recommendations based on operator feedback.

        Returns 0.0-1.0 where higher means operators tend to accept and confirm effectiveness.
        """
        stats = self._recommendation_stats.get(anomaly_type)
        if stats is None:
            return 0.5  # neutral prior

        # Blend acceptance rate and effectiveness rate
        acc = stats.acceptance_rate
        eff = stats.effectiveness_rate
        # Weight effectiveness more heavily if we have confirmation data
        if stats.confirmed_effective + stats.confirmed_ineffective > 0:
            return 0.3 * acc + 0.7 * eff
        return acc

    @property
    def feedback_history(self) -> List[FeedbackRecord]:
        with self._feedback_lock:
            return list(self._feedback_history)

    @property
    def recommendation_stats(self) -> Dict[str, RecommendationStats]:
        return dict(self._recommendation_stats)

    def get_feedback_summary(self) -> str:
        if not self._recommendation_stats:
            return "No operator feedback yet."
        lines = ["Operator Feedback Summary:"]
        for atype, stats in sorted(self._recommendation_stats.items()):
            lines.append(
                f"  {atype}: shown={stats.total_shown}, "
                f"accept={stats.acceptance_rate:.0%}, "
                f"effective={stats.effectiveness_rate:.0%}, "
                f"avg_rating={stats.avg_rating:.1f}/5"
            )
        return '\n'.join(lines)


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = FeedbackProcessorNode()
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
