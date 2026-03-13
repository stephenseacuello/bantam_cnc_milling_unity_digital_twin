"""Tests for auto-calibration integration between block telemetry and prediction runner."""

import sys
from unittest.mock import MagicMock, patch

# Mock ROS2 and miracle dependencies before importing modules under test
for mod in ['rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.action.server',
            'rclpy.callback_groups',
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
# Ensure miracle_msgs submodules have required attributes
for _mod_name in ('miracle_msgs.action', 'miracle_msgs.srv', 'miracle_msgs.msg'):
    _mod = sys.modules[_mod_name]
    for _attr in ('RunPrediction', 'PredictionResult', 'AnomalyAlert', 'MachineState',
                  'FeedOverride', 'StabilityRecommendation', 'InferredAction',
                  'PHMPrediction'):
        if not hasattr(_mod, _attr):
            setattr(_mod, _attr, MagicMock())

import pytest

from miracle_twin.block_telemetry import (
    BlockTelemetryTracker,
    CalibrationTrigger,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_records(tracker, n, force_ratio=1.0, temp_ratio=1.0, power_ratio=1.0,
                  base_force=100.0, base_temp=50.0, base_power=500.0):
    """Insert *n* records where actual = predicted * ratio for each metric."""
    for i in range(n):
        predicted = {
            'force_n': base_force,
            'temp_c': base_temp,
            'power_w': base_power,
            'wear_mm': 0.01,
        }
        actual = {
            'force_n': base_force * force_ratio,
            'temp_c': base_temp * temp_ratio,
            'power_w': base_power * power_ratio,
            'wear_mm': 0.01,
        }
        tracker.record(i, f"G1 X{i}", predicted, actual)


def _make_oscillating_records(tracker, n, metric='force',
                              base_force=100.0, base_temp=50.0, base_power=500.0):
    """Insert records with alternating over/under-prediction to create low confidence."""
    for i in range(n):
        ratio = 1.15 if i % 2 == 0 else 0.85
        force_r = ratio if metric == 'force' else 1.0
        temp_r = ratio if metric == 'temp' else 1.0
        power_r = ratio if metric == 'power' else 1.0
        predicted = {
            'force_n': base_force,
            'temp_c': base_temp,
            'power_w': base_power,
            'wear_mm': 0.01,
        }
        actual = {
            'force_n': base_force * force_r,
            'temp_c': base_temp * temp_r,
            'power_w': base_power * power_r,
            'wear_mm': 0.01,
        }
        tracker.record(i, f"G1 X{i}", predicted, actual)


# ===========================================================================
# Tests for CalibrationTrigger dataclass
# ===========================================================================

class TestCalibrationTriggerDataclass:
    """Test CalibrationTrigger dataclass fields and defaults."""

    def test_all_fields_present(self):
        trigger = CalibrationTrigger(
            triggered=True,
            reason="force_drift",
            force_correction=1.05,
            temp_correction=0.97,
            power_correction=1.02,
            confidence=0.85,
            blocks_analyzed=50,
            drift_magnitude=0.08,
            suggested_adjustments={'force_scale': 1.05},
            blocks_since_last_calibration=100,
        )
        assert trigger.triggered is True
        assert trigger.reason == "force_drift"
        assert trigger.drift_magnitude == 0.08
        assert trigger.suggested_adjustments == {'force_scale': 1.05}
        assert trigger.blocks_since_last_calibration == 100

    def test_default_new_fields(self):
        trigger = CalibrationTrigger(
            triggered=False, reason="none",
            force_correction=1.0, temp_correction=1.0,
            power_correction=1.0, confidence=0.0, blocks_analyzed=0,
        )
        assert trigger.drift_magnitude == 0.0
        assert trigger.suggested_adjustments is None
        assert trigger.blocks_since_last_calibration == 0


# ===========================================================================
# Tests for no trigger with accurate predictions
# ===========================================================================

class TestNoTriggerAccuratePredictions:
    """Test no trigger when predictions are accurate."""

    def test_small_error_no_trigger_check(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.02, temp_ratio=0.99, power_ratio=1.01)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is False

    def test_exact_predictions_no_trigger_check(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.0, temp_ratio=1.0, power_ratio=1.0)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is False

    def test_small_error_no_trigger_legacy(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 30, force_ratio=1.02, temp_ratio=0.99, power_ratio=1.01)
        trigger = tracker.compute_calibration_trigger(min_blocks=30)
        assert trigger.triggered is False


# ===========================================================================
# Tests for force drift trigger at 8% threshold
# ===========================================================================

class TestForceDriftTrigger:
    """Test force drift triggers at >8% sustained over 30+ blocks."""

    def test_force_drift_10pct_triggers(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.10)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is True
        assert trigger.drift_magnitude >= 0.08

    def test_force_drift_at_7pct_no_force_reason(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.07)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        # 7% is below force threshold; may trigger cumulative but not force_drift alone
        if trigger.triggered and trigger.reason == "force_drift":
            # Should not happen -- 7% < 8%
            pytest.fail("7% force drift should not produce force_drift reason")


# ===========================================================================
# Tests for temp drift trigger at 10% threshold
# ===========================================================================

class TestTempDriftTrigger:
    """Test temperature drift triggers at >10% sustained."""

    def test_temp_drift_12pct_triggers(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, temp_ratio=1.12)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is True

    def test_temp_drift_at_9pct_no_temp_reason(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, temp_ratio=1.09)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        if trigger.triggered:
            assert trigger.reason != "temp_drift"


# ===========================================================================
# Tests for cumulative error trigger
# ===========================================================================

class TestCumulativeErrorTrigger:
    """Test cumulative absolute error threshold (avg abs error > 6%)."""

    def test_7pct_consistent_error_triggers_cumulative(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.07)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is True

    def test_3pct_error_no_cumulative_trigger(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.03)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is False


# ===========================================================================
# Tests for periodic trigger at 200 blocks
# ===========================================================================

class TestPeriodicTrigger:
    """Test periodic calibration at >200 blocks since last calibration."""

    def test_periodic_triggers_after_200_blocks(self):
        tracker = BlockTelemetryTracker()
        # Insert 201 blocks with small drift below all metric thresholds
        _make_records(tracker, 201, force_ratio=1.04, temp_ratio=1.03, power_ratio=1.02)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is True

    def test_no_periodic_at_150_blocks(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 150, force_ratio=1.02, temp_ratio=1.01, power_ratio=1.01)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is False


# ===========================================================================
# Tests for suggested adjustments computation
# ===========================================================================

class TestSuggestedAdjustments:
    """Test compute_suggested_adjustments returns correct scaling factors."""

    def test_adjustments_reflect_bias(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.10, temp_ratio=0.90, power_ratio=1.05)
        adj = tracker.compute_suggested_adjustments(window=50)
        assert abs(adj['force_scale'] - 1.10) < 0.02
        assert abs(adj['temp_scale'] - 0.90) < 0.02
        assert abs(adj['power_scale'] - 1.05) < 0.02

    def test_empty_tracker_returns_unity(self):
        tracker = BlockTelemetryTracker()
        adj = tracker.compute_suggested_adjustments(window=50)
        assert adj == {'force_scale': 1.0, 'temp_scale': 1.0, 'power_scale': 1.0}

    def test_trigger_includes_suggested_adjustments(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.12)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.suggested_adjustments is not None
        assert 'force_scale' in trigger.suggested_adjustments
        assert abs(trigger.suggested_adjustments['force_scale'] - 1.12) < 0.02


# ===========================================================================
# Tests for mark_calibrated
# ===========================================================================

class TestMarkCalibrated:
    """Test mark_calibrated resets counters."""

    def test_resets_blocks_since_calibration(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50)
        assert tracker._blocks_since_calibration == 50
        tracker.mark_calibrated()
        assert tracker._blocks_since_calibration == 0

    def test_resets_consecutive_drifting(self):
        tracker = BlockTelemetryTracker()
        tracker._consecutive_drifting = 10
        tracker.mark_calibrated()
        assert tracker._consecutive_drifting == 0

    def test_prevents_periodic_trigger_after_reset(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 201, force_ratio=1.04, temp_ratio=1.03, power_ratio=1.02)
        tracker.mark_calibrated()
        _make_records(tracker, 50, force_ratio=1.02)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is False


# ===========================================================================
# Tests for adjustment value clamping (0.8-1.2 range)
# ===========================================================================

class TestAdjustmentClamping:
    """Test correction/adjustment values stay within [0.8, 1.2]."""

    def test_extreme_underprediction_clamped(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.50)
        adj = tracker.compute_suggested_adjustments(window=50)
        assert adj['force_scale'] == pytest.approx(1.2)

    def test_extreme_overprediction_clamped(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=0.50)
        adj = tracker.compute_suggested_adjustments(window=50)
        assert adj['force_scale'] == pytest.approx(0.8)

    def test_trigger_corrections_within_range(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=2.0)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert 0.8 <= trigger.force_correction <= 1.2
        assert 0.8 <= trigger.temp_correction <= 1.2
        assert 0.8 <= trigger.power_correction <= 1.2


# ===========================================================================
# Tests for confidence calculation
# ===========================================================================

class TestConfidenceCalculation:
    """Test confidence scoring."""

    def test_perfect_consistency_high_confidence(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.10)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.confidence > 0.9

    def test_oscillating_low_confidence(self):
        tracker = BlockTelemetryTracker()
        _make_oscillating_records(tracker, 50, metric='force')
        trigger = tracker.check_calibration_needed(min_blocks=30)
        # Oscillating means high MAD => low confidence
        assert trigger.confidence < 0.5 or not trigger.triggered


# ===========================================================================
# Tests for multiple drift types simultaneously
# ===========================================================================

class TestMultipleDriftTypes:
    """Test multiple metrics drifting at once."""

    def test_force_and_temp_drift_multi_metric(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.12, temp_ratio=1.15)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is True
        assert trigger.reason == "multi_metric"

    def test_all_three_metrics_drift(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.10, temp_ratio=0.85, power_ratio=1.15)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is True
        assert trigger.reason == "multi_metric"
        adj = trigger.suggested_adjustments
        assert abs(adj['force_scale'] - 1.0) > 0.05
        assert abs(adj['temp_scale'] - 1.0) > 0.05
        assert abs(adj['power_scale'] - 1.0) > 0.05


# ===========================================================================
# Tests for drift below threshold (no trigger)
# ===========================================================================

class TestDriftBelowThreshold:
    """Test that small drift does not trigger calibration."""

    def test_5pct_force_no_trigger(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.05)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is False

    def test_3pct_all_metrics_no_trigger(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.03, temp_ratio=0.97, power_ratio=1.03)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is False


# ===========================================================================
# Tests for empty tracker
# ===========================================================================

class TestEmptyTracker:
    """Test empty tracker returns safe no-trigger result."""

    def test_empty_check_calibration_needed(self):
        tracker = BlockTelemetryTracker()
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.triggered is False
        assert trigger.reason == "insufficient_data"
        assert trigger.blocks_analyzed == 0

    def test_empty_suggested_adjustments(self):
        tracker = BlockTelemetryTracker()
        adj = tracker.compute_suggested_adjustments()
        assert adj['force_scale'] == 1.0
        assert adj['temp_scale'] == 1.0
        assert adj['power_scale'] == 1.0


# ===========================================================================
# Tests for drift_magnitude field
# ===========================================================================

class TestDriftMagnitude:
    """Test drift_magnitude reflects actual drift."""

    def test_drift_magnitude_matches_max_bias(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.12, temp_ratio=1.05, power_ratio=1.03)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.drift_magnitude == pytest.approx(0.12, abs=0.02)

    def test_no_trigger_drift_magnitude_zero(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.0)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.drift_magnitude == 0.0


# ===========================================================================
# Tests for blocks_since_last_calibration tracking
# ===========================================================================

class TestBlocksSinceCalibration:
    """Test blocks_since_last_calibration in trigger."""

    def test_trigger_reports_correct_count(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 50, force_ratio=1.12)
        trigger = tracker.check_calibration_needed(min_blocks=30)
        assert trigger.blocks_since_last_calibration == 50

    def test_after_mark_calibrated_count_resets(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 100, force_ratio=1.12)
        tracker.mark_calibrated()
        _make_records(tracker, 20, force_ratio=1.0)
        assert tracker._blocks_since_calibration == 20


# ===========================================================================
# Legacy compute_calibration_trigger still works
# ===========================================================================

class TestLegacyComputeCalibrationTrigger:
    """Test existing compute_calibration_trigger with new fields."""

    def test_legacy_trigger_has_new_fields(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 30, force_ratio=1.10)
        trigger = tracker.compute_calibration_trigger(min_blocks=30)
        assert trigger.triggered is True
        assert hasattr(trigger, 'drift_magnitude')
        assert hasattr(trigger, 'suggested_adjustments')
        assert hasattr(trigger, 'blocks_since_last_calibration')

    def test_legacy_trigger_adjustments_populated(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 30, force_ratio=1.10)
        trigger = tracker.compute_calibration_trigger(min_blocks=30)
        assert trigger.suggested_adjustments is not None
        assert 'force_scale' in trigger.suggested_adjustments

    def test_legacy_history_max_10(self):
        tracker = BlockTelemetryTracker(max_history=2000)
        for j in range(12):
            for i in range(30):
                predicted = {'force_n': 100.0, 'temp_c': 50.0, 'power_w': 500.0, 'wear_mm': 0.01}
                actual = {'force_n': 110.0, 'temp_c': 50.0, 'power_w': 500.0, 'wear_mm': 0.01}
                tracker.record(j * 30 + i, f"G1 X{i}", predicted, actual)
            tracker.compute_calibration_trigger(min_blocks=30)
        history = tracker.get_calibration_history()
        assert len(history) == 10

    def test_legacy_reset_clears_history(self):
        tracker = BlockTelemetryTracker()
        _make_records(tracker, 30, force_ratio=1.10)
        tracker.compute_calibration_trigger(min_blocks=30)
        assert len(tracker.get_calibration_history()) == 1
        tracker.reset()
        assert len(tracker.get_calibration_history()) == 0


# ===========================================================================
# PredictionRunner integration tests
# ===========================================================================

class TestPredictionRunnerAutoCal:
    """Test auto-calibration integration in PredictionRunner._check_drift."""

    def _make_runner(self):
        from miracle_twin.prediction_runner import PredictionRunnerNode
        runner = PredictionRunnerNode()
        return runner

    def test_auto_calibration_enabled_flag_default(self):
        runner = self._make_runner()
        assert runner._auto_calibration_enabled is True

    def test_auto_calibration_disabled_skips(self):
        runner = self._make_runner()
        runner._auto_calibration_enabled = False
        proxy = MagicMock()
        runner._cutting_sim_proxy = proxy
        _make_records(runner._block_telemetry, 100, force_ratio=1.15)
        runner._blocks_since_last_calibration = 200
        runner._check_drift()
        proxy.scale_force_coefficients.assert_not_called()

    def test_cooldown_prevents_rapid_recalibration(self):
        runner = self._make_runner()
        proxy = MagicMock()
        runner._cutting_sim_proxy = proxy
        _make_records(runner._block_telemetry, 100, force_ratio=1.15)
        runner._blocks_since_last_calibration = 50
        runner._calibration_cooldown_blocks = 100
        runner._check_drift()
        proxy.scale_force_coefficients.assert_not_called()

    def test_correction_applied_to_proxy(self):
        runner = self._make_runner()
        proxy = MagicMock()
        runner._cutting_sim_proxy = proxy
        _make_records(runner._block_telemetry, 100, force_ratio=1.10)
        runner._blocks_since_last_calibration = 200
        runner._calibration_cooldown_blocks = 100
        runner._check_drift()
        proxy.scale_force_coefficients.assert_called_once()
        call_arg = proxy.scale_force_coefficients.call_args[0][0]
        assert abs(call_arg - 1.10) < 0.02

    def test_cooldown_resets_after_calibration(self):
        runner = self._make_runner()
        proxy = MagicMock()
        runner._cutting_sim_proxy = proxy
        _make_records(runner._block_telemetry, 100, force_ratio=1.10)
        runner._blocks_since_last_calibration = 200
        runner._calibration_cooldown_blocks = 100
        runner._check_drift()
        assert runner._blocks_since_last_calibration == 0

    def test_mark_calibrated_called_on_telemetry(self):
        runner = self._make_runner()
        proxy = MagicMock()
        runner._cutting_sim_proxy = proxy
        _make_records(runner._block_telemetry, 100, force_ratio=1.12)
        runner._blocks_since_last_calibration = 200
        runner._calibration_cooldown_blocks = 100
        assert runner._block_telemetry._blocks_since_calibration == 100
        runner._check_drift()
        assert runner._block_telemetry._blocks_since_calibration == 0

    def test_no_proxy_no_crash(self):
        runner = self._make_runner()
        runner._cutting_sim_proxy = None
        _make_records(runner._block_telemetry, 100, force_ratio=1.15)
        runner._blocks_since_last_calibration = 200
        runner._check_drift()

    def test_multi_metric_applies_all_corrections(self):
        runner = self._make_runner()
        proxy = MagicMock()
        runner._cutting_sim_proxy = proxy
        _make_records(runner._block_telemetry, 100,
                      force_ratio=1.10, temp_ratio=0.85, power_ratio=1.12)
        runner._blocks_since_last_calibration = 200
        runner._calibration_cooldown_blocks = 100
        runner._check_drift()
        proxy.scale_force_coefficients.assert_called_once()
        proxy.scale_thermal_factor.assert_called_once()
        proxy.scale_edge_coefficients.assert_called_once()
