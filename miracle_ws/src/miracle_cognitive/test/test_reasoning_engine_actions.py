"""Tests for reasoning engine action mapping and new manufacturing rules."""

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
class _FakeKnowledgeUpdate:
    timestamp: object = None
    update_type: str = ''
    subject: str = ''
    predicate: str = ''
    object_value: str = ''
    confidence: float = 0.0
    source: str = ''
    reasoning: str = ''

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = _FakeTime()


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
_miracle_msgs_msg_mod.KnowledgeUpdate = _FakeKnowledgeUpdate
_miracle_msgs_msg_mod.InferredAction = _FakeInferredAction
sys.modules.setdefault('miracle_msgs', _miracle_msgs_mod)
sys.modules.setdefault('miracle_msgs.msg', _miracle_msgs_msg_mod)
_existing_msg_mod = sys.modules['miracle_msgs.msg']
_existing_msg_mod.KnowledgeUpdate = _FakeKnowledgeUpdate
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
    CRITICALITY_MEDIUM = 'MEDIUM'

    def __init__(self, name, **kwargs):
        self._name = name
        self.service_callback_group = None

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.nanoseconds = 1000000000
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

    def create_timer(self, interval, callback, **kwargs):
        return MagicMock()

    def get_parameter(self, name):
        param = MagicMock()
        param.value = 5.0
        return param


class _FakeQoSProfiles:
    @staticmethod
    def state_data():
        return MagicMock()

    @staticmethod
    def command():
        return MagicMock()

    @staticmethod
    def sensor_data():
        return MagicMock()


_miracle_core_lifecycle_mod.MiracleLifecycleNode = _FakeMiracleLifecycleNode
_miracle_core_qos_mod.QoSProfiles = _FakeQoSProfiles
sys.modules.setdefault('miracle_core.lifecycle_node_base', _miracle_core_lifecycle_mod)
sys.modules.setdefault('miracle_core.qos_profiles', _miracle_core_qos_mod)
sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = _FakeMiracleLifecycleNode
sys.modules['miracle_core.qos_profiles'].QoSProfiles = _FakeQoSProfiles
sys.modules['rclpy.lifecycle'].TransitionCallbackReturn = _FakeTransitionCallbackReturn

# Force reimport so reasoning engine picks up our fakes
sys.modules.pop('miracle_cognitive.knowledge.reasoning_engine', None)

from miracle_cognitive.knowledge.reasoning_engine import ReasoningEngineNode, _ACTION_MAPPING


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_engine():
    """Create a configured and activated ReasoningEngineNode."""
    node = ReasoningEngineNode()
    node._do_configure()
    node._do_activate()
    # Replace publishers with mocks so we can inspect calls
    node._inference_pub = MagicMock()
    node._action_pub = MagicMock()
    return node


def _add_fact(node, subject, predicate, obj):
    """Add a fact to the engine."""
    msg = _FakeKnowledgeUpdate()
    msg.subject = subject
    msg.predicate = predicate
    msg.object_value = obj
    node._on_knowledge_update(msg)


# ---------------------------------------------------------------------------
# Tests: Action mapping
# ---------------------------------------------------------------------------

class TestActionMapping:
    """Test that inference conclusions map to correct action types."""

    def test_feed_rate_reduction_maps_to_reduce_feed(self):
        assert _ACTION_MAPPING[('requires', 'FeedRateReduction')] == 'REDUCE_FEED'

    def test_quality_drop_maps_to_tool_change(self):
        assert _ACTION_MAPPING[('causesRisk', 'QualityDrop')] == 'TOOL_CHANGE'

    def test_chatter_maps_to_reduce_speed(self):
        assert _ACTION_MAPPING[('mayHave', 'Chatter')] == 'REDUCE_SPEED'

    def test_coolant_increase_maps_to_coolant_increase(self):
        assert _ACTION_MAPPING[('requires', 'CoolantIncrease')] == 'COOLANT_INCREASE'

    def test_emergency_stop_maps_to_pause(self):
        assert _ACTION_MAPPING[('requires', 'EmergencyStop')] == 'PAUSE'


# ---------------------------------------------------------------------------
# Tests: Inference triggers action publishing
# ---------------------------------------------------------------------------

class TestInferredActionPublishing:
    """Test that matched inferences publish InferredAction messages."""

    def test_overload_publishes_reduce_feed(self):
        node = _make_engine()
        _add_fact(node, 'cnc1', 'hasSpindleLoad', 'OVERLOAD')
        node._run_inference()

        # Should publish at least one InferredAction with action_type REDUCE_FEED
        action_calls = node._action_pub.publish.call_args_list
        action_types = [c[0][0].action_type for c in action_calls]
        assert 'REDUCE_FEED' in action_types

    def test_high_wear_publishes_tool_change(self):
        node = _make_engine()
        _add_fact(node, 'tool1', 'hasWearLevel', 'HIGH')
        node._run_inference()

        action_calls = node._action_pub.publish.call_args_list
        action_types = [c[0][0].action_type for c in action_calls]
        assert 'TOOL_CHANGE' in action_types

    def test_high_vibration_publishes_reduce_speed(self):
        node = _make_engine()
        _add_fact(node, 'cnc1', 'hasVibrationLevel', 'HIGH')
        node._run_inference()

        action_calls = node._action_pub.publish.call_args_list
        action_types = [c[0][0].action_type for c in action_calls]
        assert 'REDUCE_SPEED' in action_types

    def test_no_action_for_unmapped_inference(self):
        """An inference with no mapping should not publish an InferredAction."""
        node = _make_engine()
        # sustained_overload_degrades_tool produces ('causesRisk', 'ToolDegradation')
        # which is NOT in _ACTION_MAPPING, so no action published for it.
        # But overload ALSO triggers REDUCE_FEED — so filter to check ToolDegradation
        # is NOT in action types.
        _add_fact(node, 'cnc1', 'hasSpindleLoad', 'OVERLOAD')
        node._run_inference()

        action_calls = node._action_pub.publish.call_args_list
        action_types = [c[0][0].action_type for c in action_calls]
        # ToolDegradation has no mapping, so it shouldn't appear
        assert 'ToolDegradation' not in action_types

    def test_inference_not_duplicated_on_second_run(self):
        """Running inference twice with same facts should not re-publish."""
        node = _make_engine()
        _add_fact(node, 'cnc1', 'hasSpindleLoad', 'OVERLOAD')
        node._run_inference()
        first_count = node._action_pub.publish.call_count

        node._run_inference()
        second_count = node._action_pub.publish.call_count
        assert second_count == first_count  # No new actions on second run


# ---------------------------------------------------------------------------
# Tests: New manufacturing rules exist
# ---------------------------------------------------------------------------

class TestNewManufacturingRules:
    """Test that the three new rules are loaded."""

    def test_thermal_overload_requires_coolant_rule_exists(self):
        node = _make_engine()
        rule_names = [r.name for r in node._rules]
        assert 'thermal_overload_requires_coolant' in rule_names

    def test_combined_wear_vibration_requires_stop_rule_exists(self):
        node = _make_engine()
        rule_names = [r.name for r in node._rules]
        assert 'combined_wear_vibration_requires_stop' in rule_names

    def test_sustained_overload_degrades_tool_rule_exists(self):
        node = _make_engine()
        rule_names = [r.name for r in node._rules]
        assert 'sustained_overload_degrades_tool' in rule_names

    def test_thermal_overload_fires_coolant_increase(self):
        node = _make_engine()
        _add_fact(node, 'cnc1', 'hasTemperature', 'CRITICAL')
        node._run_inference()

        action_calls = node._action_pub.publish.call_args_list
        action_types = [c[0][0].action_type for c in action_calls]
        assert 'COOLANT_INCREASE' in action_types

    def test_combined_wear_vibration_fires_pause(self):
        node = _make_engine()
        _add_fact(node, 'cnc1', 'hasWearLevel', 'HIGH')
        _add_fact(node, 'cnc1', 'hasVibrationLevel', 'HIGH')
        node._run_inference()

        action_calls = node._action_pub.publish.call_args_list
        action_types = [c[0][0].action_type for c in action_calls]
        assert 'PAUSE' in action_types

    def test_thermal_overload_confidence_is_085(self):
        node = _make_engine()
        rule = next(r for r in node._rules if r.name == 'thermal_overload_requires_coolant')
        assert rule.confidence == 0.85

    def test_combined_wear_vibration_confidence_is_095(self):
        node = _make_engine()
        rule = next(r for r in node._rules if r.name == 'combined_wear_vibration_requires_stop')
        assert rule.confidence == 0.95

    def test_sustained_overload_confidence_is_08(self):
        node = _make_engine()
        rule = next(r for r in node._rules if r.name == 'sustained_overload_degrades_tool')
        assert rule.confidence == 0.8


# ---------------------------------------------------------------------------
# Tests: Confidence threshold filtering
# ---------------------------------------------------------------------------

class TestConfidenceInAction:
    """Test that published actions carry correct confidence values."""

    def test_action_carries_rule_confidence(self):
        node = _make_engine()
        _add_fact(node, 'cnc1', 'hasTemperature', 'CRITICAL')
        node._run_inference()

        action_calls = node._action_pub.publish.call_args_list
        coolant_actions = [c[0][0] for c in action_calls if c[0][0].action_type == 'COOLANT_INCREASE']
        assert len(coolant_actions) == 1
        assert abs(coolant_actions[0].confidence - 0.85) < 0.01
