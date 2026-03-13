"""Tests for preemptive (prediction-based) control in the adaptive controller."""

import sys
from dataclasses import dataclass
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Mock ROS2 / miracle_core / miracle_msgs before importing the module under test
# ---------------------------------------------------------------------------
for mod in ['rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.callback_groups',
            'rclpy.qos', 'rclpy.lifecycle',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
            'miracle_core.exceptions',
            'miracle_msgs', 'miracle_msgs.msg', 'miracle_msgs.srv',
            'miracle_msgs.action']:
    sys.modules.setdefault(mod, MagicMock())

sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = type('FakeNode', (), {
    'CRITICALITY_HIGH': 'HIGH',
    'CRITICALITY_MEDIUM': 'MEDIUM',
    '__init__': lambda self, *a, **kw: None,
    'get_logger': lambda self: MagicMock(),
    'create_publisher': lambda self, *a, **kw: MagicMock(),
    'create_subscription': lambda self, *a, **kw: MagicMock(),
    'create_timer': lambda self, *a, **kw: MagicMock(),
    'declare_and_validate_parameters': lambda self, specs: {k: MagicMock(value=v['default']) for k, v in specs.items()},
    'get_parameter': lambda self, name: MagicMock(value=0),
})

for _mod_name in ('miracle_msgs.action', 'miracle_msgs.srv', 'miracle_msgs.msg'):
    _mod = sys.modules[_mod_name]
    for _attr in ('RunPrediction', 'PredictionResult', 'AnomalyAlert', 'MachineState',
                  'FeedOverride', 'StabilityRecommendation', 'InferredAction', 'OperatorFeedback'):
        if not hasattr(_mod, _attr):
            setattr(_mod, _attr, MagicMock())

sys.modules.pop('miracle_twin.adaptive_controller', None)

from miracle_twin.adaptive_controller import (
    AdaptiveControllerNode,
    PreemptiveAction,
    ControllerState,
)


# ---------------------------------------------------------------------------
# Helper: lightweight AnomalyMarker stand-in
# ---------------------------------------------------------------------------
@dataclass
class FakeMarker:
    block_index: int
    gcode_line: str
    marker_type: str
    severity: float
    predicted_value: float
    threshold: float
    recommendation: str


def _make_controller() -> AdaptiveControllerNode:
    """Create an AdaptiveControllerNode with mocked ROS2 plumbing."""
    node = AdaptiveControllerNode()
    return node


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestPreemptiveFeedReduction:
    """Test preemptive feed reduction for various marker types."""

    def test_force_warning_reduces_feed(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=5, gcode_line='G1 X10', marker_type='FORCE_WARNING',
            severity=1.0, predicted_value=900, threshold=800, recommendation='Reduce feed',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        assert ctrl._current_preemptive is not None
        assert ctrl._current_preemptive.recommended_feed_pct == 85.0
        assert ctrl._current_preemptive.recommended_speed_pct == 100.0

    def test_force_critical_reduces_feed(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=3, gcode_line='G1 X10', marker_type='FORCE_CRITICAL',
            severity=1.0, predicted_value=1100, threshold=800, recommendation='Reduce feed',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        assert ctrl._current_preemptive is not None
        assert ctrl._current_preemptive.recommended_feed_pct == 70.0

    def test_thermal_warning_reduces_speed(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=2, gcode_line='G1 X10', marker_type='THERMAL_WARNING',
            severity=1.0, predicted_value=100, threshold=90, recommendation='Reduce speed',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        assert ctrl._current_preemptive is not None
        assert ctrl._current_preemptive.recommended_speed_pct == 90.0
        assert ctrl._current_preemptive.recommended_feed_pct == 100.0

    def test_thermal_critical_reduces_both(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=4, gcode_line='G1 X10', marker_type='THERMAL_CRITICAL',
            severity=1.0, predicted_value=120, threshold=90, recommendation='Reduce both',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        act = ctrl._current_preemptive
        assert act is not None
        assert act.recommended_speed_pct == 75.0
        assert act.recommended_feed_pct == 85.0

    def test_chatter_risk_override_no_stability_data(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=1, gcode_line='G1 X10', marker_type='CHATTER_RISK',
            severity=1.0, predicted_value=0.8, threshold=0.5, recommendation='Adjust RPM',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        act = ctrl._current_preemptive
        assert act is not None
        # No stability data -> 10% reduction -> 90%
        assert act.recommended_speed_pct == 90.0

    def test_chatter_risk_with_stability_data(self):
        ctrl = _make_controller()
        # Simulate stability recommendation
        rec = MagicMock()
        rec.recommended_rpm = 4500.0
        rec.current_rpm = 5000.0
        ctrl._last_stability_recommendation = rec

        marker = FakeMarker(
            block_index=1, gcode_line='G1 X10', marker_type='CHATTER_RISK',
            severity=1.0, predicted_value=0.8, threshold=0.5, recommendation='Adjust RPM',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        act = ctrl._current_preemptive
        assert act is not None
        # 4500/5000 = 90%
        assert act.recommended_speed_pct == 90.0

    def test_wear_accelerated_override(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=7, gcode_line='G1 X10', marker_type='WEAR_ACCELERATED',
            severity=1.0, predicted_value=0.85, threshold=0.7, recommendation='Reduce feed',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        assert ctrl._current_preemptive.recommended_feed_pct == 80.0

    def test_tool_end_of_life_override(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=2, gcode_line='G1 X10', marker_type='TOOL_END_OF_LIFE',
            severity=1.0, predicted_value=0.95, threshold=0.9, recommendation='Tool change',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        act = ctrl._current_preemptive
        assert act.recommended_feed_pct == 60.0
        assert 'tool change' in act.reason.lower()

    def test_surface_quality_warning_override(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=3, gcode_line='G1 X10', marker_type='SURFACE_QUALITY_WARNING',
            severity=1.0, predicted_value=1.2, threshold=0.8, recommendation='Reduce feed',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        assert ctrl._current_preemptive.recommended_feed_pct == 75.0


class TestPreemptiveHorizonAndFiltering:
    """Test that markers outside the horizon are ignored."""

    def test_markers_outside_horizon_are_ignored(self):
        ctrl = _make_controller()
        ctrl._preemptive_horizon = 5
        marker = FakeMarker(
            block_index=100, gcode_line='G1 X10', marker_type='FORCE_WARNING',
            severity=1.0, predicted_value=900, threshold=800, recommendation='Reduce feed',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        assert ctrl._current_preemptive is None

    def test_markers_in_past_are_ignored(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=5, gcode_line='G1 X10', marker_type='FORCE_WARNING',
            severity=1.0, predicted_value=900, threshold=800, recommendation='Reduce feed',
        )
        ctrl.handle_anomaly_markers([marker], current_block=10)
        assert ctrl._current_preemptive is None


class TestConfidenceScaling:
    """Test that confidence (severity) scales the override effect."""

    def test_low_confidence_produces_softer_override(self):
        ctrl = _make_controller()
        # severity=0.5 -> half the override effect
        marker = FakeMarker(
            block_index=3, gcode_line='G1 X10', marker_type='FORCE_WARNING',
            severity=0.5, predicted_value=900, threshold=800, recommendation='Reduce feed',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        act = ctrl._current_preemptive
        assert act is not None
        # FORCE_WARNING base feed = 85%, so reduction = 15%.
        # With confidence 0.5: scaled_feed = 100 - 15*0.5 = 92.5%
        assert act.recommended_feed_pct == pytest.approx(92.5, abs=0.1)

    def test_half_confidence_scaling_formula(self):
        """Verify: scaled = 100 - (100 - base) * confidence."""
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=3, gcode_line='G1 X10', marker_type='FORCE_CRITICAL',
            severity=0.5, predicted_value=1100, threshold=800, recommendation='',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        act = ctrl._current_preemptive
        # FORCE_CRITICAL base = 70%, reduction = 30%, half = 15%  -> 85%
        assert act.recommended_feed_pct == pytest.approx(85.0, abs=0.1)


class TestMergePreemptiveReactive:
    """Test merge logic: preemptive + reactive takes more conservative value."""

    def test_merge_takes_lower_feed(self):
        ctrl = _make_controller()
        # Set reactive targets
        ctrl._target_feed_override = 90.0
        ctrl._target_spindle_override = 100.0

        # Apply a preemptive override that is more conservative for feed
        marker = FakeMarker(
            block_index=2, gcode_line='G1 X10', marker_type='FORCE_CRITICAL',
            severity=1.0, predicted_value=1100, threshold=800, recommendation='',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        # After merge, feed target should be min(90, 70) = 70
        assert ctrl._target_feed_override == pytest.approx(70.0, abs=0.1)

    def test_merge_keeps_reactive_when_more_conservative(self):
        ctrl = _make_controller()
        # Reactive is already more aggressive
        ctrl._target_feed_override = 60.0
        ctrl._target_spindle_override = 100.0

        marker = FakeMarker(
            block_index=2, gcode_line='G1 X10', marker_type='FORCE_WARNING',
            severity=1.0, predicted_value=900, threshold=800, recommendation='',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        # 60 < 85, so reactive wins
        assert ctrl._target_feed_override == pytest.approx(60.0, abs=0.1)


class TestPreemptiveClearAndDisable:
    """Test clearing and disabling preemptive overrides."""

    def test_clear_preemptive_removes_active_override(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=3, gcode_line='G1 X10', marker_type='FORCE_WARNING',
            severity=1.0, predicted_value=900, threshold=800, recommendation='',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        assert ctrl._current_preemptive is not None
        ctrl.clear_preemptive()
        assert ctrl._current_preemptive is None

    def test_preemptive_disabled_ignores_markers(self):
        ctrl = _make_controller()
        ctrl._preemptive_enabled = False
        marker = FakeMarker(
            block_index=3, gcode_line='G1 X10', marker_type='FORCE_WARNING',
            severity=1.0, predicted_value=900, threshold=800, recommendation='',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        assert ctrl._current_preemptive is None
        assert len(ctrl._preemptive_actions) == 0


class TestMultipleMarkers:
    """Test that when multiple markers are present, the most severe wins."""

    def test_most_severe_marker_wins(self):
        ctrl = _make_controller()
        markers = [
            FakeMarker(block_index=2, gcode_line='G1 X10', marker_type='FORCE_WARNING',
                       severity=1.0, predicted_value=900, threshold=800, recommendation=''),
            FakeMarker(block_index=3, gcode_line='G1 X20', marker_type='FORCE_CRITICAL',
                       severity=1.0, predicted_value=1100, threshold=800, recommendation=''),
        ]
        ctrl.handle_anomaly_markers(markers, current_block=0)
        # FORCE_CRITICAL at 70% is more severe than FORCE_WARNING at 85%
        assert ctrl._current_preemptive.marker_type == 'FORCE_CRITICAL'
        assert ctrl._current_preemptive.recommended_feed_pct == 70.0

    def test_unknown_marker_type_is_skipped(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=3, gcode_line='G1 X10', marker_type='UNKNOWN_TYPE',
            severity=1.0, predicted_value=0, threshold=0, recommendation='',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        assert ctrl._current_preemptive is None


class TestPreemptiveStatus:
    """Test get_preemptive_status returns correct dashboard data."""

    def test_status_when_no_active_preemptive(self):
        ctrl = _make_controller()
        status = ctrl.get_preemptive_status()
        assert status['enabled'] is True
        assert status['active'] is False
        assert status['current_action'] is None
        assert status['history_count'] == 0
        assert status['horizon'] == 10
        assert status['min_confidence'] == 0.5

    def test_status_when_active_preemptive(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=5, gcode_line='G1 X10', marker_type='THERMAL_WARNING',
            severity=0.8, predicted_value=95, threshold=90, recommendation='',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        status = ctrl.get_preemptive_status()
        assert status['active'] is True
        assert status['current_action']['marker_type'] == 'THERMAL_WARNING'
        assert status['current_action']['blocks_ahead'] == 5
        assert status['history_count'] == 1


class TestPreemptiveActionHistory:
    """Test that preemptive actions are tracked in history."""

    def test_history_grows_with_each_call(self):
        ctrl = _make_controller()
        for i in range(3):
            marker = FakeMarker(
                block_index=i + 1, gcode_line=f'G1 X{i}', marker_type='FORCE_WARNING',
                severity=1.0, predicted_value=900, threshold=800, recommendation='',
            )
            ctrl.handle_anomaly_markers([marker], current_block=0)
        assert len(ctrl._preemptive_actions) == 3

    def test_history_entries_are_marked_applied(self):
        ctrl = _make_controller()
        marker = FakeMarker(
            block_index=2, gcode_line='G1 X10', marker_type='WEAR_ACCELERATED',
            severity=1.0, predicted_value=0.85, threshold=0.7, recommendation='',
        )
        ctrl.handle_anomaly_markers([marker], current_block=0)
        assert ctrl._preemptive_actions[-1].applied is True
