"""Tests for adaptive controller inferred-action handling from cognitive reasoning."""

import pytest
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from unittest.mock import MagicMock
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Minimal stubs for ROS2 types so tests run without a ROS2 installation.
# ---------------------------------------------------------------------------

@dataclass
class _FakeTime:
    sec: int = 0
    nanosec: int = 0


@dataclass
class _FakeStabilityRecommendation:
    machine_id: str = ''
    current_rpm: float = 0.0
    recommended_rpm: float = 0.0
    current_depth: float = 0.0
    max_stable_depth: float = 0.0
    risk_level: str = 'LOW'
    stability_margin: float = 1.0
    recommendation: str = ''


@dataclass
class _FakeFeedOverride:
    timestamp: object = None
    feed_override_pct: float = 100.0
    spindle_override_pct: float = 100.0
    reason: str = ''
    confidence: float = 0.0
    revert_after_sec: float = 0.0

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = _FakeTime()


@dataclass
class _FakeMachineState:
    cutting_force_x: float = 0.0
    cutting_force_y: float = 0.0
    cutting_force_z: float = 0.0
    spindle_load: float = 0.0
    spindle_temp: float = 25.0


@dataclass
class _FakeAnomalyAlert:
    anomaly_type: str = ''
    severity: float = 0.0
    contributing_factors: list = None

    def __post_init__(self):
        if self.contributing_factors is None:
            self.contributing_factors = []


@dataclass
class _FakeInferredAction:
    timestamp: object = None
    machine_id: str = ''
    inference_rule: str = ''
    action_type: str = ''
    subject: str = ''
    predicate: str = ''
    object_value: str = ''
    confidence: float = 0.0
    reasoning: str = ''

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = _FakeTime()


# Stub miracle_msgs.msg
_miracle_msgs_mod = type(sys)('miracle_msgs')
_miracle_msgs_msg_mod = type(sys)('miracle_msgs.msg')
_miracle_msgs_msg_mod.StabilityRecommendation = _FakeStabilityRecommendation
_miracle_msgs_msg_mod.FeedOverride = _FakeFeedOverride
_miracle_msgs_msg_mod.MachineState = _FakeMachineState
_miracle_msgs_msg_mod.AnomalyAlert = _FakeAnomalyAlert
_miracle_msgs_msg_mod.InferredAction = _FakeInferredAction
sys.modules.setdefault('miracle_msgs', _miracle_msgs_mod)
sys.modules.setdefault('miracle_msgs.msg', _miracle_msgs_msg_mod)
_existing_msg_mod = sys.modules['miracle_msgs.msg']
_existing_msg_mod.StabilityRecommendation = _FakeStabilityRecommendation
_existing_msg_mod.FeedOverride = _FakeFeedOverride
_existing_msg_mod.MachineState = _FakeMachineState
_existing_msg_mod.AnomalyAlert = _FakeAnomalyAlert
_existing_msg_mod.InferredAction = _FakeInferredAction

# Stub rclpy modules
_rclpy_mod = type(sys)('rclpy')
_rclpy_lifecycle_mod = type(sys)('rclpy.lifecycle')


class _FakeTransitionCallbackReturn:
    SUCCESS = 0
    FAILURE = 1
    ERROR = 2


_rclpy_lifecycle_mod.TransitionCallbackReturn = _FakeTransitionCallbackReturn
sys.modules.setdefault('rclpy', _rclpy_mod)
sys.modules.setdefault('rclpy.lifecycle', _rclpy_lifecycle_mod)
sys.modules.setdefault('rclpy.executors', type(sys)('rclpy.executors'))

# Stub miracle_core modules
_miracle_core_mod = type(sys)('miracle_core')
_miracle_core_lifecycle_mod = type(sys)('miracle_core.lifecycle_node_base')
_miracle_core_qos_mod = type(sys)('miracle_core.qos_profiles')


class _FakeMiracleLifecycleNode:
    CRITICALITY_HIGH = 'HIGH'

    def __init__(self, name, **kwargs):
        self._name = name
        self.service_callback_group = None
        self._clock_nanoseconds = 1_000_000_000  # 1 second

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.nanoseconds = self._clock_nanoseconds
        clock.now.return_value.to_msg.return_value = _FakeTime()
        return clock

    def get_logger(self):
        return MagicMock()

    def declare_and_validate_parameters(self, params):
        return {k: v.get('default') for k, v in params.items()}

    def create_publisher(self, msg_type, topic, qos):
        return MagicMock()

    def create_subscription(self, msg_type, topic, callback, qos):
        return MagicMock()

    def create_multi_machine_subscriptions(self, *args, **kwargs):
        return MagicMock()

    def create_timer(self, interval, callback, **kwargs):
        return MagicMock()

    def get_parameter(self, name):
        param = MagicMock()
        param.value = 1.0
        return param


class _FakeQoSProfiles:
    @staticmethod
    def command():
        return MagicMock()

    @staticmethod
    def sensor_data():
        return MagicMock()

    @staticmethod
    def alert():
        return MagicMock()

    @staticmethod
    def state_data():
        return MagicMock()


_miracle_core_lifecycle_mod.MiracleLifecycleNode = _FakeMiracleLifecycleNode
_miracle_core_qos_mod.QoSProfiles = _FakeQoSProfiles
sys.modules.setdefault('miracle_core.lifecycle_node_base', _miracle_core_lifecycle_mod)
sys.modules.setdefault('miracle_core.qos_profiles', _miracle_core_qos_mod)
sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = _FakeMiracleLifecycleNode
sys.modules['miracle_core.qos_profiles'].QoSProfiles = _FakeQoSProfiles
sys.modules['rclpy.lifecycle'].TransitionCallbackReturn = _FakeTransitionCallbackReturn

# Force reimport so the controller picks up our fakes
sys.modules.pop('miracle_twin.adaptive_controller', None)

from miracle_twin.adaptive_controller import AdaptiveControllerNode


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_controller():
    """Create an AdaptiveControllerNode, configure it, and return (node, mock_publisher)."""
    node = AdaptiveControllerNode()
    node._do_configure()
    node._do_activate()
    pub = MagicMock()
    node._override_pub = pub
    return node, pub


def _make_inferred_action(
    action_type='REDUCE_FEED',
    confidence=0.9,
    machine_id='cnc1',
    inference_rule='test_rule',
    subject='cnc1',
    predicate='requires',
    object_value='FeedRateReduction',
    reasoning='test reasoning',
):
    return _FakeInferredAction(
        machine_id=machine_id,
        inference_rule=inference_rule,
        action_type=action_type,
        subject=subject,
        predicate=predicate,
        object_value=object_value,
        confidence=confidence,
        reasoning=reasoning,
    )


# ---------------------------------------------------------------------------
# Tests: REDUCE_FEED
# ---------------------------------------------------------------------------

class TestReduceFeed:
    """REDUCE_FEED with confidence > 0.7 should publish feed override at 70%."""

    def test_reduce_feed_triggers_70pct_override(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='REDUCE_FEED', confidence=0.9)
        node._on_inferred_action(msg)

        pub.publish.assert_called_once()
        override = pub.publish.call_args[0][0]
        assert override.feed_override_pct == 70.0
        assert override.spindle_override_pct == 100.0

    def test_reduce_feed_low_confidence_ignored(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='REDUCE_FEED', confidence=0.5)
        node._on_inferred_action(msg)

        pub.publish.assert_not_called()


# ---------------------------------------------------------------------------
# Tests: REDUCE_SPEED
# ---------------------------------------------------------------------------

class TestReduceSpeed:
    """REDUCE_SPEED with confidence > 0.7 should publish spindle override at 85%."""

    def test_reduce_speed_triggers_85pct_spindle(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='REDUCE_SPEED', confidence=0.9)
        node._on_inferred_action(msg)

        pub.publish.assert_called_once()
        override = pub.publish.call_args[0][0]
        assert override.feed_override_pct == 100.0
        assert override.spindle_override_pct == 85.0

    def test_reduce_speed_low_confidence_ignored(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='REDUCE_SPEED', confidence=0.6)
        node._on_inferred_action(msg)

        pub.publish.assert_not_called()


# ---------------------------------------------------------------------------
# Tests: TOOL_CHANGE
# ---------------------------------------------------------------------------

class TestToolChange:
    """TOOL_CHANGE with confidence > 0.8 should publish protective slowdown at 50%."""

    def test_tool_change_triggers_50pct_feed(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='TOOL_CHANGE', confidence=0.95)
        node._on_inferred_action(msg)

        pub.publish.assert_called_once()
        override = pub.publish.call_args[0][0]
        assert override.feed_override_pct == 50.0
        assert override.spindle_override_pct == 100.0

    def test_tool_change_below_threshold_ignored(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='TOOL_CHANGE', confidence=0.75)
        node._on_inferred_action(msg)

        pub.publish.assert_not_called()


# ---------------------------------------------------------------------------
# Tests: PAUSE (emergency stop)
# ---------------------------------------------------------------------------

class TestPause:
    """PAUSE with confidence > 0.9 should publish feed override at 0%."""

    def test_pause_triggers_emergency_stop(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='PAUSE', confidence=0.95)
        node._on_inferred_action(msg)

        pub.publish.assert_called_once()
        override = pub.publish.call_args[0][0]
        assert override.feed_override_pct == 0.0

    def test_pause_below_threshold_ignored(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='PAUSE', confidence=0.85)
        node._on_inferred_action(msg)

        pub.publish.assert_not_called()


# ---------------------------------------------------------------------------
# Tests: COOLANT_INCREASE (log only)
# ---------------------------------------------------------------------------

class TestCoolantIncrease:
    """COOLANT_INCREASE should only log a warning, not publish a FeedOverride."""

    def test_coolant_increase_does_not_publish(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='COOLANT_INCREASE', confidence=0.85)
        node._on_inferred_action(msg)

        pub.publish.assert_not_called()


# ---------------------------------------------------------------------------
# Tests: Deduplication
# ---------------------------------------------------------------------------

class TestDeduplication:
    """Same action_type within 30 seconds should be deduplicated."""

    def test_duplicate_within_30s_ignored(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='REDUCE_FEED', confidence=0.9)

        # First call should publish
        node._on_inferred_action(msg)
        assert pub.publish.call_count == 1

        # Second call at same time should be deduplicated
        node._on_inferred_action(msg)
        assert pub.publish.call_count == 1

    def test_different_action_types_not_deduplicated(self):
        node, pub = _make_controller()
        msg1 = _make_inferred_action(action_type='REDUCE_FEED', confidence=0.9)
        msg2 = _make_inferred_action(action_type='REDUCE_SPEED', confidence=0.9)

        node._on_inferred_action(msg1)
        node._on_inferred_action(msg2)
        assert pub.publish.call_count == 2

    def test_action_allowed_after_30s(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='REDUCE_FEED', confidence=0.9)

        # First call at t=1s
        node._on_inferred_action(msg)
        assert pub.publish.call_count == 1

        # Advance clock past 30s dedup window
        node._clock_nanoseconds = 32_000_000_000  # 32 seconds
        node._on_inferred_action(msg)
        assert pub.publish.call_count == 2


# ---------------------------------------------------------------------------
# Tests: Low confidence filtering
# ---------------------------------------------------------------------------

class TestLowConfidenceFiltering:
    """Actions with confidence below thresholds should not trigger overrides."""

    def test_reduce_feed_at_exactly_07_not_triggered(self):
        """Confidence must be strictly > 0.7."""
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='REDUCE_FEED', confidence=0.7)
        node._on_inferred_action(msg)
        pub.publish.assert_not_called()

    def test_reduce_speed_at_exactly_07_not_triggered(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='REDUCE_SPEED', confidence=0.7)
        node._on_inferred_action(msg)
        pub.publish.assert_not_called()

    def test_tool_change_at_exactly_08_not_triggered(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='TOOL_CHANGE', confidence=0.8)
        node._on_inferred_action(msg)
        pub.publish.assert_not_called()

    def test_pause_at_exactly_09_not_triggered(self):
        node, pub = _make_controller()
        msg = _make_inferred_action(action_type='PAUSE', confidence=0.9)
        node._on_inferred_action(msg)
        pub.publish.assert_not_called()
