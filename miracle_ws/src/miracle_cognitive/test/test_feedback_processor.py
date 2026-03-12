"""Tests for the Operator Feedback Processor."""

import pytest
import time
import sys
import os
from unittest.mock import MagicMock

# Mock ROS2 dependencies before import.
# Use a real base class stub so that __new__ works on subclasses.


class _StubLifecycleNode:
    """Minimal stub replacing MiracleLifecycleNode for tests."""
    CRITICALITY_LOW = 'LOW'
    CRITICALITY_HIGH = 'HIGH'
    CRITICALITY_CRITICAL = 'CRITICAL'
    CRITICALITY_MEDIUM = 'MEDIUM'

    def __init__(self, *args, **kwargs):
        pass


_lifecycle_mod = MagicMock()
_lifecycle_mod.MiracleLifecycleNode = _StubLifecycleNode

_mock_modules = {
    'rclpy': MagicMock(),
    'rclpy.lifecycle': MagicMock(),
    'rclpy.node': MagicMock(),
    'rclpy.qos': MagicMock(),
    'rclpy.callback_groups': MagicMock(),
    'miracle_core.lifecycle_node_base': _lifecycle_mod,
    'miracle_core.qos_profiles': MagicMock(),
    'miracle_msgs': MagicMock(),
    'miracle_msgs.msg': MagicMock(),
}
for mod_name, mock_obj in _mock_modules.items():
    sys.modules.setdefault(mod_name, mock_obj)

# Ensure OperatorFeedback exists on whatever miracle_msgs.msg module is installed.
_msgs_mod = sys.modules['miracle_msgs.msg']
if not hasattr(_msgs_mod, 'OperatorFeedback') or not callable(getattr(_msgs_mod, 'OperatorFeedback', None)):
    _msgs_mod.OperatorFeedback = MagicMock()

# Ensure lifecycle_node_base has our stub.
sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = _StubLifecycleNode

# Force reimport
sys.modules.pop('miracle_cognitive.interface.feedback_processor', None)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_cognitive.interface.feedback_processor import (
    FeedbackRecord,
    RecommendationStats,
    FeedbackProcessorNode,
)


def _make_node():
    """Create a FeedbackProcessorNode with mocked super().__init__."""
    node = FeedbackProcessorNode.__new__(FeedbackProcessorNode)
    node._feedback_history = []
    node._feedback_lock = __import__('threading').Lock()
    node._recommendation_stats = {}
    node._history_size = 10000
    node._causal_update_callback = None
    node._explanation_update_callback = None
    # Mock logger
    node.get_logger = MagicMock()
    return node


def _make_feedback_msg(feedback_type, anomaly_type='vibration_anomaly',
                       rating=1.0, action_taken='', reference_id='ref-1',
                       machine_id='cnc1'):
    """Create a mock OperatorFeedback message."""
    msg = MagicMock()
    msg.machine_id = machine_id
    msg.feedback_type = feedback_type
    msg.anomaly_type = anomaly_type
    msg.rating = rating
    msg.action_taken = action_taken
    msg.reference_id = reference_id
    return msg


class TestRecommendationAccepted:
    def test_accepted_updates_count(self):
        node = _make_node()
        msg = _make_feedback_msg('RECOMMENDATION_ACCEPTED')
        node._on_feedback(msg)
        stats = node._recommendation_stats['vibration_anomaly']
        assert stats.accepted == 1
        assert stats.total_shown == 1

    def test_rejected_updates_count(self):
        node = _make_node()
        msg = _make_feedback_msg('RECOMMENDATION_REJECTED')
        node._on_feedback(msg)
        stats = node._recommendation_stats['vibration_anomaly']
        assert stats.rejected == 1
        assert stats.total_shown == 1


class TestActionConfirmed:
    def test_confirmed_updates_effectiveness(self):
        node = _make_node()
        msg = _make_feedback_msg('ACTION_CONFIRMED', action_taken='reduced feed')
        node._on_feedback(msg)
        stats = node._recommendation_stats['vibration_anomaly']
        assert stats.confirmed_effective == 1

    def test_failed_updates_ineffectiveness(self):
        node = _make_node()
        msg = _make_feedback_msg('ACTION_FAILED', action_taken='reduced feed')
        node._on_feedback(msg)
        stats = node._recommendation_stats['vibration_anomaly']
        assert stats.confirmed_ineffective == 1


class TestAcceptanceRate:
    def test_acceptance_rate_calculation(self):
        node = _make_node()
        # 3 accepted, 1 rejected
        for _ in range(3):
            node._on_feedback(_make_feedback_msg('RECOMMENDATION_ACCEPTED'))
        node._on_feedback(_make_feedback_msg('RECOMMENDATION_REJECTED'))
        stats = node._recommendation_stats['vibration_anomaly']
        assert stats.acceptance_rate == pytest.approx(0.75)

    def test_no_feedback_returns_default(self):
        stats = RecommendationStats(anomaly_type='test')
        assert stats.acceptance_rate == 0.5


class TestEffectivenessRate:
    def test_effectiveness_rate_calculation(self):
        node = _make_node()
        node._on_feedback(_make_feedback_msg('ACTION_CONFIRMED'))
        node._on_feedback(_make_feedback_msg('ACTION_CONFIRMED'))
        node._on_feedback(_make_feedback_msg('ACTION_FAILED'))
        stats = node._recommendation_stats['vibration_anomaly']
        assert stats.effectiveness_rate == pytest.approx(2.0 / 3.0)

    def test_no_confirmations_returns_default(self):
        stats = RecommendationStats(anomaly_type='test')
        assert stats.effectiveness_rate == 0.5


class TestRecommendationConfidence:
    def test_blends_acceptance_and_effectiveness(self):
        node = _make_node()
        # 2 accepted, 0 rejected => acceptance_rate = 1.0
        node._on_feedback(_make_feedback_msg('RECOMMENDATION_ACCEPTED'))
        node._on_feedback(_make_feedback_msg('RECOMMENDATION_ACCEPTED'))
        # 1 confirmed, 1 failed => effectiveness_rate = 0.5
        node._on_feedback(_make_feedback_msg('ACTION_CONFIRMED'))
        node._on_feedback(_make_feedback_msg('ACTION_FAILED'))
        confidence = node.get_recommendation_confidence('vibration_anomaly')
        # 0.3 * 1.0 + 0.7 * 0.5 = 0.65
        assert confidence == pytest.approx(0.65)

    def test_no_feedback_returns_neutral(self):
        node = _make_node()
        assert node.get_recommendation_confidence('unknown') == 0.5

    def test_acceptance_only_when_no_confirmation(self):
        node = _make_node()
        node._on_feedback(_make_feedback_msg('RECOMMENDATION_ACCEPTED'))
        node._on_feedback(_make_feedback_msg('RECOMMENDATION_REJECTED'))
        # acceptance_rate = 0.5, no confirmations so just use acceptance
        confidence = node.get_recommendation_confidence('vibration_anomaly')
        assert confidence == pytest.approx(0.5)


class TestFeedbackHistory:
    def test_history_stored(self):
        node = _make_node()
        node._on_feedback(_make_feedback_msg('RECOMMENDATION_ACCEPTED'))
        node._on_feedback(_make_feedback_msg('RECOMMENDATION_REJECTED'))
        assert len(node.feedback_history) == 2

    def test_history_trimmed(self):
        node = _make_node()
        node._history_size = 5
        for i in range(10):
            node._on_feedback(_make_feedback_msg('RECOMMENDATION_ACCEPTED'))
        assert len(node.feedback_history) == 5


class TestCallbacks:
    def test_causal_callback_invoked_on_confirmed(self):
        node = _make_node()
        callback = MagicMock()
        node.set_causal_update_callback(callback)
        msg = _make_feedback_msg('ACTION_CONFIRMED', action_taken='replaced tool')
        node._on_feedback(msg)
        callback.assert_called_once_with('vibration_anomaly', 'replaced tool', True)

    def test_causal_callback_invoked_on_failed(self):
        node = _make_node()
        callback = MagicMock()
        node.set_causal_update_callback(callback)
        msg = _make_feedback_msg('ACTION_FAILED', action_taken='reduced speed')
        node._on_feedback(msg)
        callback.assert_called_once_with('vibration_anomaly', 'reduced speed', False)

    def test_explanation_callback_invoked_on_rated(self):
        node = _make_node()
        callback = MagicMock()
        node.set_explanation_update_callback(callback)
        msg = _make_feedback_msg('EXPLANATION_RATED', rating=4.0)
        node._on_feedback(msg)
        callback.assert_called_once_with('vibration_anomaly', 4.0)


class TestFeedbackSummary:
    def test_empty_summary(self):
        node = _make_node()
        assert node.get_feedback_summary() == "No operator feedback yet."

    def test_summary_format(self):
        node = _make_node()
        node._on_feedback(_make_feedback_msg('RECOMMENDATION_ACCEPTED'))
        node._on_feedback(_make_feedback_msg('ACTION_CONFIRMED'))
        summary = node.get_feedback_summary()
        assert "Operator Feedback Summary:" in summary
        assert "vibration_anomaly" in summary
        assert "shown=" in summary
        assert "accept=" in summary
        assert "effective=" in summary
