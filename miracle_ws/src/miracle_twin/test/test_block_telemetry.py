"""Tests for block_telemetry module -- actual vs predicted drift detection."""

import csv
import os
import sys
import tempfile
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Mock ROS 2 and miracle_core/miracle_msgs before importing project modules
# ---------------------------------------------------------------------------
for mod in [
    'rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.callback_groups',
    'rclpy.qos', 'rclpy.lifecycle',
    'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
    'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
    'miracle_core.exceptions',
    'miracle_msgs', 'miracle_msgs.msg', 'miracle_msgs.srv',
    'miracle_msgs.action',
]:
    sys.modules.setdefault(mod, MagicMock())

sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = type(
    'FakeNode', (), {
        'CRITICALITY_HIGH': 'HIGH',
        'CRITICALITY_MEDIUM': 'MEDIUM',
        '__init__': lambda self, *a, **kw: None,
        'get_logger': lambda self: MagicMock(),
        'create_publisher': lambda self, *a, **kw: MagicMock(),
        'create_subscription': lambda self, *a, **kw: MagicMock(),
        'create_timer': lambda self, *a, **kw: MagicMock(),
        'declare_and_validate_parameters': lambda self, specs: {
            k: MagicMock(value=v['default']) for k, v in specs.items()
        },
        'get_parameter': lambda self, name: MagicMock(value=0),
    },
)

from miracle_twin.block_telemetry import (
    BlockTelemetryRecord,
    BlockTelemetryTracker,
    DriftReport,
    _error_pct,
    _linear_slope,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_predicted(force=100.0, temp=50.0, power=200.0, wear=0.05):
    return {'force_n': force, 'temp_c': temp, 'power_w': power, 'wear_mm': wear}


def _make_actual(force=100.0, temp=50.0, power=200.0, wear=0.05):
    return {'force_n': force, 'temp_c': temp, 'power_w': power, 'wear_mm': wear}


# ===========================================================================
# Tests
# ===========================================================================


class TestRecordCreation:
    """Test that records are created correctly."""

    def test_basic_record(self):
        tracker = BlockTelemetryTracker()
        rec = tracker.record(0, 'G1 X10', _make_predicted(), _make_actual())
        assert isinstance(rec, BlockTelemetryRecord)
        assert rec.block_index == 0
        assert rec.gcode_line == 'G1 X10'

    def test_record_stores_predicted_and_actual(self):
        tracker = BlockTelemetryTracker()
        rec = tracker.record(
            1, 'G1 X20',
            _make_predicted(force=100.0),
            _make_actual(force=120.0),
        )
        assert rec.predicted_force_n == 100.0
        assert rec.actual_force_n == 120.0

    def test_record_has_timestamp(self):
        tracker = BlockTelemetryTracker()
        rec = tracker.record(0, 'G1 X5', _make_predicted(), _make_actual())
        assert rec.timestamp > 0


class TestErrorPercentage:
    """Test error percentage calculations."""

    def test_positive_error(self):
        """Actual > predicted gives positive error %."""
        pct = _error_pct(actual=120.0, predicted=100.0)
        assert pct == pytest.approx(20.0)

    def test_negative_error(self):
        """Actual < predicted gives negative error %."""
        pct = _error_pct(actual=80.0, predicted=100.0)
        assert pct == pytest.approx(-20.0)

    def test_zero_predicted(self):
        """Zero predicted returns 0 to avoid division by zero."""
        assert _error_pct(actual=50.0, predicted=0.0) == 0.0

    def test_exact_match(self):
        pct = _error_pct(actual=100.0, predicted=100.0)
        assert pct == pytest.approx(0.0)

    def test_record_error_fields(self):
        """Error fields on the record match the helper function."""
        tracker = BlockTelemetryTracker()
        rec = tracker.record(
            0, 'G1',
            _make_predicted(force=100.0, temp=50.0, power=200.0),
            _make_actual(force=110.0, temp=55.0, power=220.0),
        )
        assert rec.force_error_pct == pytest.approx(10.0)
        assert rec.temp_error_pct == pytest.approx(10.0)
        assert rec.power_error_pct == pytest.approx(10.0)


class TestDriftDetection:
    """Test drift detection with known linear drift patterns."""

    def test_drift_with_linearly_increasing_error(self):
        """A linearly increasing force error should trigger drift detection."""
        tracker = BlockTelemetryTracker(drift_threshold=5.0)
        for i in range(100):
            # Force error grows: 0%, 1%, 2%, ... 99%
            pred_force = 100.0
            actual_force = 100.0 + i  # error grows linearly
            tracker.record(
                i, f'G1 X{i}',
                _make_predicted(force=pred_force),
                _make_actual(force=actual_force),
            )
        report = tracker.get_drift_report(window=100)
        assert report.is_drifting is True
        assert report.force_drift_trend > 0  # positive = underpredicts

    def test_no_drift_for_accurate_predictions(self):
        """Constant small error should NOT trigger drift."""
        tracker = BlockTelemetryTracker(drift_threshold=10.0)
        for i in range(100):
            tracker.record(
                i, f'G1 X{i}',
                _make_predicted(force=100.0, temp=50.0),
                _make_actual(force=102.0, temp=51.0),  # constant 2% error
            )
        report = tracker.get_drift_report(window=100)
        assert report.is_drifting is False
        # Trend should be near zero since error is constant
        assert abs(report.force_drift_trend) < 1.0

    def test_temperature_drift_triggers(self):
        """Drift in temperature alone should set is_drifting."""
        tracker = BlockTelemetryTracker(drift_threshold=5.0)
        for i in range(100):
            tracker.record(
                i, f'G1 X{i}',
                _make_predicted(force=100.0, temp=50.0),
                _make_actual(force=100.0, temp=50.0 + i * 0.5),
            )
        report = tracker.get_drift_report(window=100)
        assert report.is_drifting is True
        assert report.temp_drift_trend > 0


class TestWorstBlocks:
    """Test worst_blocks ranking."""

    def test_worst_blocks_by_force(self):
        tracker = BlockTelemetryTracker()
        # Insert blocks with varying error
        for i in range(10):
            tracker.record(
                i, f'G1 X{i}',
                _make_predicted(force=100.0),
                _make_actual(force=100.0 + i * 10),
            )
        worst = tracker.get_worst_blocks(metric='force', top_n=3)
        assert len(worst) == 3
        # Worst block should have largest absolute error
        assert abs(worst[0].force_error_pct) >= abs(worst[1].force_error_pct)
        assert abs(worst[1].force_error_pct) >= abs(worst[2].force_error_pct)

    def test_worst_blocks_by_temp(self):
        tracker = BlockTelemetryTracker()
        for i in range(10):
            tracker.record(
                i, f'G1 X{i}',
                _make_predicted(temp=50.0),
                _make_actual(temp=50.0 + i * 5),
            )
        worst = tracker.get_worst_blocks(metric='temp', top_n=2)
        assert len(worst) == 2
        assert abs(worst[0].temp_error_pct) >= abs(worst[1].temp_error_pct)

    def test_worst_blocks_by_power(self):
        tracker = BlockTelemetryTracker()
        for i in range(10):
            tracker.record(
                i, f'G1 X{i}',
                _make_predicted(power=200.0),
                _make_actual(power=200.0 + i * 20),
            )
        worst = tracker.get_worst_blocks(metric='power', top_n=1)
        assert len(worst) == 1


class TestMaxHistoryCap:
    """Test that max_history limits storage."""

    def test_cap_at_max_history(self):
        tracker = BlockTelemetryTracker(max_history=10)
        for i in range(25):
            tracker.record(i, f'G1 X{i}', _make_predicted(), _make_actual())
        history = tracker.get_block_history(last_n=100)
        assert len(history) == 10
        # Oldest kept should be block_index 15
        assert history[0].block_index == 15


class TestExportCSV:
    """Test CSV export format."""

    def test_export_creates_valid_csv(self):
        tracker = BlockTelemetryTracker()
        for i in range(5):
            tracker.record(
                i, f'G1 X{i}',
                _make_predicted(force=100.0),
                _make_actual(force=110.0),
            )
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.csv', delete=False
        ) as tmp:
            path = tmp.name

        try:
            tracker.export_csv(path)
            with open(path, newline='') as fh:
                reader = csv.reader(fh)
                rows = list(reader)
            # Header + 5 data rows
            assert len(rows) == 6
            assert rows[0][0] == 'block_index'
            assert 'force_error_pct' in rows[0]
        finally:
            os.unlink(path)

    def test_export_empty_tracker(self):
        tracker = BlockTelemetryTracker()
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.csv', delete=False
        ) as tmp:
            path = tmp.name
        try:
            tracker.export_csv(path)
            with open(path, newline='') as fh:
                reader = csv.reader(fh)
                rows = list(reader)
            # Header only, no data rows
            assert len(rows) == 1
        finally:
            os.unlink(path)


class TestReset:
    """Test that reset clears history."""

    def test_reset_clears_records(self):
        tracker = BlockTelemetryTracker()
        for i in range(10):
            tracker.record(i, f'G1 X{i}', _make_predicted(), _make_actual())
        assert len(tracker.get_block_history()) == 10
        tracker.reset()
        assert len(tracker.get_block_history()) == 0

    def test_reset_clears_calibration_counter(self):
        tracker = BlockTelemetryTracker()
        for i in range(10):
            tracker.record(i, f'G1', _make_predicted(), _make_actual())
        tracker.reset()
        report = tracker.get_drift_report()
        assert report.blocks_since_calibration == 0


class TestRecalibrationSuggested:
    """Test recalibration_suggested logic."""

    def test_recalibration_after_prolonged_drift(self):
        """Recalibration should be suggested after >50 consecutive drift checks."""
        tracker = BlockTelemetryTracker(drift_threshold=1.0)
        # Generate strongly drifting data for many blocks
        for i in range(200):
            tracker.record(
                i, f'G1 X{i}',
                _make_predicted(force=100.0),
                _make_actual(force=100.0 + i * 2),  # strong linear drift
            )
            # Call get_drift_report to advance the consecutive counter
            tracker.get_drift_report(window=50)

        report = tracker.get_drift_report(window=50)
        assert report.is_drifting is True
        assert report.recalibration_suggested is True

    def test_no_recalibration_for_brief_drift(self):
        """Brief drift should NOT suggest recalibration."""
        tracker = BlockTelemetryTracker(drift_threshold=1.0)
        for i in range(30):
            tracker.record(
                i, f'G1 X{i}',
                _make_predicted(force=100.0),
                _make_actual(force=100.0 + i * 2),
            )
            tracker.get_drift_report(window=30)
        report = tracker.get_drift_report(window=30)
        # Only ~30 checks, not >50
        assert report.recalibration_suggested is False


class TestDriftThresholdSensitivity:
    """Test that drift_threshold parameter controls sensitivity."""

    def test_high_threshold_ignores_moderate_drift(self):
        tracker = BlockTelemetryTracker(drift_threshold=100.0)
        for i in range(100):
            tracker.record(
                i, f'G1 X{i}',
                _make_predicted(force=100.0),
                _make_actual(force=100.0 + i * 0.5),
            )
        report = tracker.get_drift_report(window=100)
        assert report.is_drifting is False

    def test_low_threshold_catches_small_drift(self):
        tracker = BlockTelemetryTracker(drift_threshold=0.5)
        for i in range(100):
            tracker.record(
                i, f'G1 X{i}',
                _make_predicted(force=100.0),
                _make_actual(force=100.0 + i * 0.2),
            )
        report = tracker.get_drift_report(window=100)
        assert report.is_drifting is True


class TestEmptyTracker:
    """Test safe defaults when tracker is empty."""

    def test_drift_report_on_empty(self):
        tracker = BlockTelemetryTracker()
        report = tracker.get_drift_report()
        assert report.window_size == 0
        assert report.mean_force_error_pct == 0.0
        assert report.is_drifting is False
        assert report.recalibration_suggested is False

    def test_block_history_on_empty(self):
        tracker = BlockTelemetryTracker()
        assert tracker.get_block_history() == []

    def test_worst_blocks_on_empty(self):
        tracker = BlockTelemetryTracker()
        assert tracker.get_worst_blocks() == []


class TestWindowParameter:
    """Test window parameter for drift report."""

    def test_small_window(self):
        tracker = BlockTelemetryTracker()
        for i in range(100):
            tracker.record(i, f'G1', _make_predicted(), _make_actual())
        report = tracker.get_drift_report(window=10)
        assert report.window_size == 10

    def test_window_larger_than_history(self):
        tracker = BlockTelemetryTracker()
        for i in range(5):
            tracker.record(i, f'G1', _make_predicted(), _make_actual())
        report = tracker.get_drift_report(window=100)
        assert report.window_size == 5

    def test_different_windows_different_stats(self):
        """Narrow window on recent drifting data should differ from full history."""
        tracker = BlockTelemetryTracker(drift_threshold=5.0)
        # First 50 blocks: accurate
        for i in range(50):
            tracker.record(
                i, f'G1',
                _make_predicted(force=100.0),
                _make_actual(force=100.0),
            )
        # Next 50 blocks: drifting
        for i in range(50, 100):
            tracker.record(
                i, f'G1',
                _make_predicted(force=100.0),
                _make_actual(force=100.0 + (i - 50) * 2),
            )
        report_full = tracker.get_drift_report(window=100)
        report_recent = tracker.get_drift_report(window=50)
        # The recent window should show stronger drift
        assert abs(report_recent.force_drift_trend) > abs(report_full.force_drift_trend)


class TestLinearSlope:
    """Test the _linear_slope helper."""

    def test_flat(self):
        assert _linear_slope([5.0, 5.0, 5.0, 5.0]) == pytest.approx(0.0)

    def test_positive_slope(self):
        # 0, 1, 2, 3 -> slope = 1 per index, * 100 = 100 per 100 blocks
        slope = _linear_slope([0.0, 1.0, 2.0, 3.0])
        assert slope == pytest.approx(100.0)

    def test_single_value(self):
        assert _linear_slope([42.0]) == 0.0

    def test_empty(self):
        assert _linear_slope([]) == 0.0
