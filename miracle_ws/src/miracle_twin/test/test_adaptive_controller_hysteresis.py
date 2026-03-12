"""Tests for adaptive controller hysteresis-based state machine."""

import pytest
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from unittest.mock import MagicMock, patch, call
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Minimal stubs for ROS2 types so tests run without a ROS2 installation.
# ---------------------------------------------------------------------------

@dataclass
class _FakeTime:
    sec: int = 0
    nanosec: int = 0


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
class _FakeStabilityRecommendation:
    machine_id: str = ''
    current_rpm: float = 0.0
    recommended_rpm: float = 0.0
    current_depth: float = 0.0
    max_stable_depth: float = 0.0
    risk_level: str = 'LOW'
    stability_margin: float = 1.0
    recommendation: str = ''


# Stub miracle_msgs.msg
_miracle_msgs_mod = type(sys)('miracle_msgs')
_miracle_msgs_msg_mod = type(sys)('miracle_msgs.msg')
_miracle_msgs_msg_mod.StabilityRecommendation = _FakeStabilityRecommendation
_miracle_msgs_msg_mod.FeedOverride = _FakeFeedOverride
_miracle_msgs_msg_mod.MachineState = _FakeMachineState
_miracle_msgs_msg_mod.AnomalyAlert = _FakeAnomalyAlert
sys.modules.setdefault('miracle_msgs', _miracle_msgs_mod)
sys.modules.setdefault('miracle_msgs.msg', _miracle_msgs_msg_mod)
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
        self._clock_nanoseconds = 1_000_000_000  # 1.0 second

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


_miracle_core_lifecycle_mod.MiracleLifecycleNode = _FakeMiracleLifecycleNode
_miracle_core_qos_mod.QoSProfiles = _FakeQoSProfiles
sys.modules.setdefault('miracle_core.lifecycle_node_base', _miracle_core_lifecycle_mod)
sys.modules.setdefault('miracle_core.qos_profiles', _miracle_core_qos_mod)
sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = _FakeMiracleLifecycleNode
sys.modules['miracle_core.qos_profiles'].QoSProfiles = _FakeQoSProfiles
sys.modules['rclpy.lifecycle'].TransitionCallbackReturn = _FakeTransitionCallbackReturn

# Force reimport so the controller picks up our fakes
sys.modules.pop('miracle_twin.adaptive_controller', None)

from miracle_twin.adaptive_controller import (
    AdaptiveControllerNode,
    ControllerState,
    FORCE_ACTIVATE_PCT,
    FORCE_DEACTIVATE_PCT,
    THERMAL_ACTIVATE_PCT,
    THERMAL_DEACTIVATE_PCT,
    WEAR_ACTIVATE_PCT,
    WEAR_DEACTIVATE_PCT,
)


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _make_controller(clock_sec=1.0):
    """Create a configured and activated controller with a mock publisher."""
    node = AdaptiveControllerNode()
    node._clock_nanoseconds = int(clock_sec * 1e9)
    node._do_configure()
    node._do_activate()
    pub = MagicMock()
    node._override_pub = pub
    # Set state entry time to 0 so initial transitions pass the debounce
    node._state_entry_time = 0.0
    return node, pub


def _set_clock(node, sec):
    """Advance the fake clock to the given time in seconds."""
    node._clock_nanoseconds = int(sec * 1e9)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestForceHysteresis:
    """Force-based hysteresis state transitions."""

    def test_force_at_82pct_activates_force_limited(self):
        """Force above 80% activation threshold enters FORCE_LIMITED."""
        node, pub = _make_controller(clock_sec=10.0)
        node._state.force_ratio = 0.82  # above 80% activate threshold

        node._compute_and_publish()

        assert node._controller_state == ControllerState.FORCE_LIMITED

    def test_force_drops_to_70pct_stays_force_limited(self):
        """Force at 70% is above 65% deactivation threshold: stays FORCE_LIMITED."""
        node, pub = _make_controller(clock_sec=10.0)
        node._state.force_ratio = 0.82
        node._compute_and_publish()
        assert node._controller_state == ControllerState.FORCE_LIMITED

        # Advance clock past debounce and drop force into hysteresis band
        _set_clock(node, 20.0)
        node._state.force_ratio = 0.70  # above 0.65 deactivate, below 0.80 activate
        node._compute_and_publish()

        # Should remain FORCE_LIMITED due to hysteresis
        assert node._controller_state == ControllerState.FORCE_LIMITED

    def test_force_drops_to_60pct_returns_to_normal(self):
        """Force below 65% deactivation threshold returns to NORMAL."""
        node, pub = _make_controller(clock_sec=10.0)
        node._state.force_ratio = 0.82
        node._compute_and_publish()
        assert node._controller_state == ControllerState.FORCE_LIMITED

        # Advance clock past debounce and drop force below deactivation threshold
        _set_clock(node, 20.0)
        node._state.force_ratio = 0.60  # below 0.65 deactivate
        node._compute_and_publish()

        assert node._controller_state == ControllerState.NORMAL


class TestMinStateDuration:
    """Debounce: minimum state duration prevents flapping."""

    def test_rapid_oscillation_no_flapping(self):
        """Rapid force oscillation around 80% doesn't cause state flapping."""
        node, pub = _make_controller(clock_sec=10.0)

        # Enter FORCE_LIMITED
        node._state.force_ratio = 0.85
        node._compute_and_publish()
        assert node._controller_state == ControllerState.FORCE_LIMITED

        # Only 2 seconds later, force drops below deactivation — should NOT transition
        _set_clock(node, 12.0)
        node._state.force_ratio = 0.50
        node._compute_and_publish()
        assert node._controller_state == ControllerState.FORCE_LIMITED  # debounce holds

    def test_min_duration_prevents_transition_within_5s(self):
        """Cannot leave a state before 5 seconds have elapsed."""
        node, pub = _make_controller(clock_sec=10.0)

        # Enter FORCE_LIMITED
        node._state.force_ratio = 0.85
        node._compute_and_publish()
        assert node._controller_state == ControllerState.FORCE_LIMITED
        entry_time = node._state_entry_time

        # At 14s (only 4s in state), try to go back to NORMAL
        _set_clock(node, 14.0)
        node._state.force_ratio = 0.50
        node._compute_and_publish()
        assert node._controller_state == ControllerState.FORCE_LIMITED

        # At 16s (6s since entry), should transition
        _set_clock(node, 16.0)
        node._state.force_ratio = 0.50
        node._compute_and_publish()
        assert node._controller_state == ControllerState.NORMAL


class TestStateHistory:
    """State transition recording."""

    def test_state_transition_logged_in_history(self):
        """Transitions are recorded in state_history."""
        node, pub = _make_controller(clock_sec=10.0)
        node._state.force_ratio = 0.85
        node._compute_and_publish()

        history = node.state_history
        assert len(history) >= 1
        ts, from_state, to_state = history[-1]
        assert from_state == ControllerState.NORMAL
        assert to_state == ControllerState.FORCE_LIMITED

    def test_state_history_records_correct_sequence(self):
        """Full sequence: NORMAL -> FORCE_LIMITED -> NORMAL recorded correctly."""
        node, pub = _make_controller(clock_sec=10.0)

        # NORMAL -> FORCE_LIMITED
        node._state.force_ratio = 0.85
        node._compute_and_publish()

        # FORCE_LIMITED -> NORMAL (after debounce)
        _set_clock(node, 20.0)
        node._state.force_ratio = 0.50
        node._compute_and_publish()

        history = node.state_history
        assert len(history) == 2
        assert history[0][1] == ControllerState.NORMAL
        assert history[0][2] == ControllerState.FORCE_LIMITED
        assert history[1][1] == ControllerState.FORCE_LIMITED
        assert history[1][2] == ControllerState.NORMAL


class TestSmoothRamping:
    """Override values ramp gradually rather than jumping."""

    def test_override_ramps_gradually_not_step_change(self):
        """Feed override doesn't jump to target in a single cycle."""
        node, pub = _make_controller(clock_sec=10.0)
        node._current_feed_override = 100.0

        # Force a large target reduction
        node._state.force_ratio = 0.95
        node._compute_and_publish()

        # The current override should have moved toward the target but not reached it
        # (ramp rate is 5% per cycle, target is well below 100%)
        assert node._current_feed_override < 100.0
        assert node._current_feed_override > node._target_feed_override

    def test_smooth_ramp_from_100_to_70_takes_multiple_cycles(self):
        """Ramping from 100% to 70% takes at least 6 cycles at 5% per cycle."""
        node, pub = _make_controller(clock_sec=10.0)
        node._current_feed_override = 100.0

        # Set force to push target to ~70%
        node._state.force_ratio = 0.95
        # With force_activate=0.80, excess=0.15, reduction=0.15/0.20=0.75
        # feed_pct = max(20, 100*(1 - 0.75*0.5)) = 100*0.625 = 62.5

        cycle_count = 0
        while node._current_feed_override > 63.0 and cycle_count < 100:
            _set_clock(node, 10.0 + cycle_count)
            # Keep state_entry_time far in the past to avoid debounce issues
            node._state_entry_time = 0.0
            node._compute_and_publish()
            cycle_count += 1

        # Should take multiple cycles (at 5% per cycle, ~7-8 cycles for 37.5% drop)
        assert cycle_count >= 6

    def test_recovery_ramps_back_to_100_smoothly(self):
        """When returning to NORMAL, override ramps back up gradually."""
        node, pub = _make_controller(clock_sec=10.0)
        node._current_feed_override = 70.0
        node._current_spindle_override = 100.0
        node._controller_state = ControllerState.FORCE_LIMITED
        node._state_entry_time = 0.0  # long ago

        # All values nominal — target should be 100%
        node._state.force_ratio = 0.50
        _set_clock(node, 20.0)
        node._compute_and_publish()

        # Should have ramped up by ramp_rate (5%) but not jumped to 100%
        assert node._current_feed_override == pytest.approx(75.0, abs=0.1)


class TestStatePriority:
    """State priority ordering."""

    def test_emergency_overrides_all_others(self):
        """EMERGENCY state transitions immediately regardless of debounce."""
        node, pub = _make_controller(clock_sec=10.0)

        # Enter FORCE_LIMITED first
        node._state.force_ratio = 0.85
        node._compute_and_publish()
        assert node._controller_state == ControllerState.FORCE_LIMITED

        # Now force at 100% capacity — EMERGENCY should happen immediately
        _set_clock(node, 11.0)  # only 1 second later (within debounce)
        node._state.force_ratio = 1.0
        node._compute_and_publish()

        assert node._controller_state == ControllerState.EMERGENCY

    def test_wear_limited_takes_priority_over_force_limited(self):
        """WEAR_LIMITED outranks FORCE_LIMITED when both are active."""
        node, pub = _make_controller(clock_sec=10.0)

        # Both force and wear above activation thresholds
        node._state.force_ratio = 0.85
        node._state.wear_ratio = 0.95  # above 90% wear activate
        node._compute_and_publish()

        assert node._controller_state == ControllerState.WEAR_LIMITED


class TestStateProperty:
    """External access to state and history via properties."""

    def test_state_property_returns_current_state(self):
        node, pub = _make_controller(clock_sec=10.0)
        assert node.state == ControllerState.NORMAL

        node._state.force_ratio = 0.85
        node._compute_and_publish()
        assert node.state == ControllerState.FORCE_LIMITED

    def test_state_history_property_returns_copy(self):
        """state_history property returns a copy, not a reference."""
        node, pub = _make_controller(clock_sec=10.0)
        node._state.force_ratio = 0.85
        node._compute_and_publish()

        history = node.state_history
        history.clear()  # mutate the returned copy
        assert len(node.state_history) == 1  # original unchanged
