"""Tests for adaptive controller stability recommendation handling."""

import pytest
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from unittest.mock import MagicMock, patch, call
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Minimal stubs for ROS2 types so tests run without a ROS2 installation.
# Uses setdefault to avoid clobbering real modules if they are available.
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


# Stub out miracle_msgs.msg before importing the controller
_miracle_msgs_mod = type(sys)('miracle_msgs')
_miracle_msgs_msg_mod = type(sys)('miracle_msgs.msg')
_miracle_msgs_msg_mod.StabilityRecommendation = _FakeStabilityRecommendation
_miracle_msgs_msg_mod.FeedOverride = _FakeFeedOverride
_miracle_msgs_msg_mod.MachineState = _FakeMachineState
_miracle_msgs_msg_mod.AnomalyAlert = _FakeAnomalyAlert
sys.modules.setdefault('miracle_msgs', _miracle_msgs_mod)
sys.modules.setdefault('miracle_msgs.msg', _miracle_msgs_msg_mod)
# Ensure our fakes are set even if another test loaded miracle_msgs.msg first
_existing_msg_mod = sys.modules['miracle_msgs.msg']
_existing_msg_mod.StabilityRecommendation = _FakeStabilityRecommendation
_existing_msg_mod.FeedOverride = _FakeFeedOverride
_existing_msg_mod.MachineState = _FakeMachineState
_existing_msg_mod.AnomalyAlert = _FakeAnomalyAlert

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

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.nanoseconds = 1000000000
        clock.now.return_value.to_msg.return_value = _FakeTime()
        return clock

    def get_logger(self):
        logger = MagicMock()
        return logger

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


_miracle_core_lifecycle_mod.MiracleLifecycleNode = _FakeMiracleLifecycleNode
_miracle_core_qos_mod.QoSProfiles = _FakeQoSProfiles
sys.modules.setdefault('miracle_core.lifecycle_node_base', _miracle_core_lifecycle_mod)
sys.modules.setdefault('miracle_core.qos_profiles', _miracle_core_qos_mod)
# Ensure our fakes override any prior MagicMock stubs
sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = _FakeMiracleLifecycleNode
sys.modules['miracle_core.qos_profiles'].QoSProfiles = _FakeQoSProfiles
sys.modules['rclpy.lifecycle'].TransitionCallbackReturn = _FakeTransitionCallbackReturn

# Force reimport so the controller picks up our fakes
sys.modules.pop('miracle_twin.adaptive_controller', None)

from miracle_twin.adaptive_controller import AdaptiveControllerNode


# ---------------------------------------------------------------------------
# Helper to build a configured controller with mocked publisher
# ---------------------------------------------------------------------------

def _make_controller():
    """Create an AdaptiveControllerNode, configure it, and return (node, mock_publisher)."""
    node = AdaptiveControllerNode()
    node._do_configure()
    node._do_activate()
    pub = MagicMock()
    node._override_pub = pub
    return node, pub


def _make_recommendation(
    risk_level='HIGH',
    current_rpm=12000.0,
    recommended_rpm=10000.0,
    current_depth=2.0,
    max_stable_depth=1.8,
    stability_margin=0.1,
    recommendation='Test recommendation',
    machine_id='cnc1',
):
    return _FakeStabilityRecommendation(
        machine_id=machine_id,
        current_rpm=current_rpm,
        recommended_rpm=recommended_rpm,
        current_depth=current_depth,
        max_stable_depth=max_stable_depth,
        risk_level=risk_level,
        stability_margin=stability_margin,
        recommendation=recommendation,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestStabilityRecommendationHighRisk:
    """HIGH risk recommendations should trigger an immediate spindle override."""

    def test_high_risk_triggers_override_when_rpm_diff_exceeds_5pct(self):
        node, pub = _make_controller()
        msg = _make_recommendation(
            risk_level='HIGH',
            current_rpm=12000.0,
            recommended_rpm=10000.0,  # ~16.7% difference
        )
        node._on_stability_recommendation(msg)

        pub.publish.assert_called_once()
        override = pub.publish.call_args[0][0]
        assert override.feed_override_pct == 80.0
        # spindle_override_pct should reflect the RPM ratio
        expected_spindle = (10000.0 / 12000.0) * 100.0  # ~83.3%
        assert abs(override.spindle_override_pct - expected_spindle) < 0.1
        assert 'HIGH' in override.reason

    def test_high_risk_no_override_when_rpm_diff_under_5pct(self):
        node, pub = _make_controller()
        msg = _make_recommendation(
            risk_level='HIGH',
            current_rpm=10000.0,
            recommended_rpm=10400.0,  # 4% difference — under threshold
        )
        node._on_stability_recommendation(msg)

        pub.publish.assert_not_called()

    def test_high_risk_resets_medium_counter(self):
        node, pub = _make_controller()
        # First send two MEDIUM messages
        med_msg = _make_recommendation(risk_level='MEDIUM')
        node._on_stability_recommendation(med_msg)
        node._on_stability_recommendation(med_msg)
        assert node._consecutive_medium_count == 2

        # Then HIGH should reset
        high_msg = _make_recommendation(risk_level='HIGH')
        node._on_stability_recommendation(high_msg)
        assert node._consecutive_medium_count == 0

    def test_high_risk_spindle_clamped_to_80_minimum(self):
        node, pub = _make_controller()
        msg = _make_recommendation(
            risk_level='HIGH',
            current_rpm=20000.0,
            recommended_rpm=10000.0,  # 50% reduction would give 50% spindle
        )
        node._on_stability_recommendation(msg)

        pub.publish.assert_called_once()
        override = pub.publish.call_args[0][0]
        assert override.spindle_override_pct >= 80.0


class TestStabilityRecommendationMediumRisk:
    """MEDIUM risk should not trigger immediate override, only after 3 consecutive."""

    def test_single_medium_no_override(self):
        node, pub = _make_controller()
        msg = _make_recommendation(risk_level='MEDIUM')
        node._on_stability_recommendation(msg)

        pub.publish.assert_not_called()
        assert node._consecutive_medium_count == 1

    def test_two_medium_no_override(self):
        node, pub = _make_controller()
        msg = _make_recommendation(risk_level='MEDIUM')
        node._on_stability_recommendation(msg)
        node._on_stability_recommendation(msg)

        pub.publish.assert_not_called()
        assert node._consecutive_medium_count == 2

    def test_three_consecutive_medium_triggers_override(self):
        node, pub = _make_controller()
        msg = _make_recommendation(
            risk_level='MEDIUM',
            current_rpm=12000.0,
            recommended_rpm=10000.0,
        )
        node._on_stability_recommendation(msg)
        node._on_stability_recommendation(msg)
        node._on_stability_recommendation(msg)

        pub.publish.assert_called_once()
        override = pub.publish.call_args[0][0]
        assert override.feed_override_pct == 90.0
        assert 'MEDIUM' in override.reason
        assert 'persistent' in override.reason

    def test_medium_counter_resets_after_trigger(self):
        node, pub = _make_controller()
        msg = _make_recommendation(risk_level='MEDIUM')
        for _ in range(3):
            node._on_stability_recommendation(msg)

        # Counter should have reset after triggering
        assert node._consecutive_medium_count == 0

    def test_medium_interrupted_by_low_resets_counter(self):
        node, pub = _make_controller()
        med_msg = _make_recommendation(risk_level='MEDIUM')
        low_msg = _make_recommendation(risk_level='LOW')

        node._on_stability_recommendation(med_msg)
        node._on_stability_recommendation(med_msg)
        assert node._consecutive_medium_count == 2

        node._on_stability_recommendation(low_msg)
        assert node._consecutive_medium_count == 0

        # Now two more MEDIUM — should not trigger (only 2)
        node._on_stability_recommendation(med_msg)
        node._on_stability_recommendation(med_msg)
        pub.publish.assert_not_called()


class TestStabilityRecommendationLowRisk:
    """LOW risk should reset the consecutive medium counter and not publish."""

    def test_low_risk_no_override(self):
        node, pub = _make_controller()
        msg = _make_recommendation(risk_level='LOW')
        node._on_stability_recommendation(msg)

        pub.publish.assert_not_called()
        assert node._consecutive_medium_count == 0

    def test_low_risk_resets_medium_counter(self):
        node, pub = _make_controller()
        med_msg = _make_recommendation(risk_level='MEDIUM')
        low_msg = _make_recommendation(risk_level='LOW')

        node._on_stability_recommendation(med_msg)
        node._on_stability_recommendation(med_msg)
        assert node._consecutive_medium_count == 2

        node._on_stability_recommendation(low_msg)
        assert node._consecutive_medium_count == 0


class TestStabilityRecommendationConfidence:
    """Override confidence should derive from stability margin."""

    def test_confidence_is_inverse_of_margin(self):
        node, pub = _make_controller()
        msg = _make_recommendation(
            risk_level='HIGH',
            stability_margin=0.1,
            current_rpm=12000.0,
            recommended_rpm=10000.0,
        )
        node._on_stability_recommendation(msg)

        override = pub.publish.call_args[0][0]
        assert abs(override.confidence - 0.9) < 0.01

    def test_confidence_at_zero_margin(self):
        node, pub = _make_controller()
        msg = _make_recommendation(
            risk_level='HIGH',
            stability_margin=0.0,
            current_rpm=12000.0,
            recommended_rpm=10000.0,
        )
        node._on_stability_recommendation(msg)

        override = pub.publish.call_args[0][0]
        assert abs(override.confidence - 1.0) < 0.01
