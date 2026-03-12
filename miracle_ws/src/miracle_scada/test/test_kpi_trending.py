"""Tests for OEE trending, history tracking, and degradation detection.

Uses the same mock pattern as test_kpi_calculator.py — pure functions and
dataclasses are tested without any ROS2 dependency.
"""

import logging
import pytest
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# ---------------------------------------------------------------------------
# Import pure functions and dataclasses from the kpi_calculator module.
# We mirror the same pattern as test_kpi_calculator.py for isolation.
# ---------------------------------------------------------------------------

from miracle_scada.kpi_calculator import (
    OEESnapshot,
    OEETrendAnalysis,
    _MAX_OEE_HISTORY,
    compute_moving_average,
    compute_linear_regression_slope,
    _classify_trend,
    _analyze_trends,
    compute_real_mtbf,
    compute_real_mttr,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_snapshots(
    values: List[float],
    start_time: float = 0.0,
    interval: float = 30.0,
    machine_id: str = 'fleet',
) -> List[OEESnapshot]:
    """Create a list of OEESnapshot with the given OEE values.

    All four metrics (oee, availability, performance, quality) are set to the
    same value for simplicity unless overridden.
    """
    snapshots = []
    for i, v in enumerate(values):
        snapshots.append(OEESnapshot(
            timestamp=start_time + i * interval,
            oee=v,
            availability=v,
            performance=v,
            quality=v,
            machine_id=machine_id,
        ))
    return snapshots


def _make_snapshots_custom(
    entries: List[dict],
    start_time: float = 0.0,
    interval: float = 30.0,
) -> List[OEESnapshot]:
    """Create snapshots with per-metric control."""
    snapshots = []
    for i, e in enumerate(entries):
        snapshots.append(OEESnapshot(
            timestamp=start_time + i * interval,
            oee=e.get('oee', 0.8),
            availability=e.get('availability', 0.9),
            performance=e.get('performance', 0.85),
            quality=e.get('quality', 0.95),
            machine_id=e.get('machine_id', 'fleet'),
        ))
    return snapshots


class _FakeHistory:
    """Mimics the history storage behaviour of KPICalculatorNode without ROS2."""

    def __init__(self) -> None:
        self._oee_history: List[OEESnapshot] = []
        self._machine_oee_history: Dict[str, List[OEESnapshot]] = {}
        self._degradation_threshold: float = -0.02
        self._improving_threshold: float = 0.01

    def append_snapshot(self, snapshot: OEESnapshot) -> None:
        if snapshot.machine_id == 'fleet':
            self._oee_history.append(snapshot)
            if len(self._oee_history) > _MAX_OEE_HISTORY:
                self._oee_history = self._oee_history[-_MAX_OEE_HISTORY:]
        else:
            hist = self._machine_oee_history.setdefault(
                snapshot.machine_id, [],
            )
            hist.append(snapshot)
            if len(hist) > _MAX_OEE_HISTORY:
                self._machine_oee_history[snapshot.machine_id] = \
                    hist[-_MAX_OEE_HISTORY:]

    @property
    def fleet_history(self) -> List[OEESnapshot]:
        return list(self._oee_history)

    def machine_history(self, machine_id: str) -> List[OEESnapshot]:
        return list(self._machine_oee_history.get(machine_id, []))

    def get_oee_history(self, hours: float = 24.0, now: float = 0.0) -> List[OEESnapshot]:
        cutoff = now - hours * 3600.0
        return [s for s in self._oee_history if s.timestamp >= cutoff]

    def get_trend_analysis(self) -> Dict[str, OEETrendAnalysis]:
        return _analyze_trends(
            self._oee_history,
            degradation_threshold=self._degradation_threshold,
            improving_threshold=self._improving_threshold,
        )


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def history():
    return _FakeHistory()


# ---------------------------------------------------------------------------
# Tests — Snapshot Storage
# ---------------------------------------------------------------------------

class TestSnapshotStorage:
    """Test that snapshots are stored and capped correctly."""

    def test_snapshot_stored_on_publish(self, history):
        """Each append should add a snapshot to the fleet history."""
        snap = OEESnapshot(
            timestamp=100.0, oee=0.85, availability=0.9,
            performance=0.9, quality=0.95,
        )
        history.append_snapshot(snap)
        assert len(history.fleet_history) == 1
        assert history.fleet_history[0].oee == 0.85

    def test_history_capped_at_max(self, history):
        """History should never exceed _MAX_OEE_HISTORY entries."""
        for i in range(_MAX_OEE_HISTORY + 200):
            snap = OEESnapshot(
                timestamp=float(i * 30),
                oee=0.8,
                availability=0.9,
                performance=0.85,
                quality=0.95,
            )
            history.append_snapshot(snap)

        assert len(history.fleet_history) == _MAX_OEE_HISTORY
        # Oldest entries should have been trimmed; first entry timestamp
        # should correspond to the 200th snapshot (index 200).
        assert history.fleet_history[0].timestamp == 200 * 30.0

    def test_per_machine_snapshots_separate(self, history):
        """Per-machine snapshots are tracked independently from fleet."""
        fleet_snap = OEESnapshot(
            timestamp=1.0, oee=0.80, availability=0.9,
            performance=0.85, quality=0.95, machine_id='fleet',
        )
        cnc1_snap = OEESnapshot(
            timestamp=1.0, oee=0.75, availability=0.85,
            performance=0.82, quality=0.93, machine_id='cnc1',
        )
        cnc2_snap = OEESnapshot(
            timestamp=1.0, oee=0.90, availability=0.95,
            performance=0.92, quality=0.98, machine_id='cnc2',
        )
        history.append_snapshot(fleet_snap)
        history.append_snapshot(cnc1_snap)
        history.append_snapshot(cnc2_snap)

        assert len(history.fleet_history) == 1
        assert len(history.machine_history('cnc1')) == 1
        assert len(history.machine_history('cnc2')) == 1
        assert history.machine_history('cnc1')[0].oee == 0.75
        assert history.machine_history('cnc2')[0].oee == 0.90

    def test_per_machine_history_capped(self, history):
        """Per-machine history should also be capped at _MAX_OEE_HISTORY."""
        for i in range(_MAX_OEE_HISTORY + 50):
            snap = OEESnapshot(
                timestamp=float(i * 30),
                oee=0.8, availability=0.9, performance=0.85, quality=0.95,
                machine_id='cnc1',
            )
            history.append_snapshot(snap)

        assert len(history.machine_history('cnc1')) == _MAX_OEE_HISTORY


# ---------------------------------------------------------------------------
# Tests — Trend Detection
# ---------------------------------------------------------------------------

class TestTrendDetection:
    """Test that trend direction is correctly classified."""

    def test_improving_trend_detected(self):
        """OEE increasing linearly over 1 hour should be classified as improving."""
        # 120 samples at 30s interval = 1 hour, rising from 0.70 to 0.90
        values = [0.70 + 0.20 * i / 119 for i in range(120)]
        snapshots = _make_snapshots(values, start_time=0.0, interval=30.0)

        trends = _analyze_trends(snapshots)
        assert trends['oee'].trend_direction == 'improving'
        assert trends['oee'].slope_per_hour > 0.01

    def test_degrading_trend_detected(self):
        """OEE decreasing linearly over 1 hour should be classified as degrading."""
        # 120 samples at 30s interval = 1 hour, falling from 0.90 to 0.70
        values = [0.90 - 0.20 * i / 119 for i in range(120)]
        snapshots = _make_snapshots(values, start_time=0.0, interval=30.0)

        trends = _analyze_trends(snapshots)
        assert trends['oee'].trend_direction == 'degrading'
        assert trends['oee'].slope_per_hour < -0.02

    def test_stable_trend_when_constant(self):
        """Constant OEE should be classified as stable."""
        values = [0.85] * 120
        snapshots = _make_snapshots(values, start_time=0.0, interval=30.0)

        trends = _analyze_trends(snapshots)
        assert trends['oee'].trend_direction == 'stable'
        assert abs(trends['oee'].slope_per_hour) < 0.001

    def test_slope_calculation_accuracy(self):
        """Linear increase of 0.10 over 1 hour should give slope ~0.10/hr."""
        # 121 samples from t=0 to t=3600 (1 hour), values from 0.80 to 0.90
        values = [0.80 + 0.10 * i / 120 for i in range(121)]
        snapshots = _make_snapshots(values, start_time=0.0, interval=30.0)

        slope = compute_linear_regression_slope(
            snapshots, 'oee', window_seconds=3700.0,
        )
        # Should be very close to 0.10 per hour
        assert abs(slope - 0.10) < 0.005

    def test_classify_trend_thresholds(self):
        """Verify classification against explicit thresholds."""
        assert _classify_trend(0.05) == 'improving'
        assert _classify_trend(0.01) == 'improving'
        assert _classify_trend(0.005) == 'stable'
        assert _classify_trend(0.0) == 'stable'
        assert _classify_trend(-0.01) == 'stable'
        assert _classify_trend(-0.02) == 'degrading'
        assert _classify_trend(-0.05) == 'degrading'

    def test_trend_analysis_all_metrics(self):
        """Trend analysis should return results for all four OEE metrics."""
        values = [0.85] * 10
        snapshots = _make_snapshots(values, start_time=0.0, interval=30.0)
        trends = _analyze_trends(snapshots)

        assert set(trends.keys()) == {'oee', 'availability', 'performance', 'quality'}
        for metric, analysis in trends.items():
            assert analysis.metric == metric
            assert analysis.samples == 10

    def test_trend_empty_history(self):
        """Trend analysis with no history should return stable with zero values."""
        trends = _analyze_trends([])
        for metric in ('oee', 'availability', 'performance', 'quality'):
            assert trends[metric].trend_direction == 'stable'
            assert trends[metric].samples == 0
            assert trends[metric].current_value == 0.0


# ---------------------------------------------------------------------------
# Tests — Moving Average
# ---------------------------------------------------------------------------

class TestMovingAverage:
    """Test moving average computation."""

    def test_moving_average_1h_window(self):
        """Moving average over 1 hour window should average only those samples."""
        # Create 2 hours of data: first hour at 0.70, second hour at 0.90
        values_h1 = [0.70] * 120  # first hour
        values_h2 = [0.90] * 120  # second hour
        snapshots = _make_snapshots(
            values_h1 + values_h2, start_time=0.0, interval=30.0,
        )
        # 1h window from the end: last 120 samples (all 0.90)
        avg = compute_moving_average(
            snapshots, 'oee', window_seconds=3600.0,
        )
        assert abs(avg - 0.90) < 0.01

    def test_moving_average_8h_window(self):
        """Moving average over 8 hours should include all available data if < 8h."""
        # 1 hour of data with known mean
        values = [0.80 + 0.001 * i for i in range(120)]
        snapshots = _make_snapshots(values, start_time=0.0, interval=30.0)

        avg_8h = compute_moving_average(
            snapshots, 'oee', window_seconds=28800.0,
        )
        expected = sum(values) / len(values)
        assert abs(avg_8h - expected) < 0.001

    def test_moving_average_empty(self):
        """Moving average with no data should return 0.0."""
        assert compute_moving_average([], 'oee', 3600.0) == 0.0

    def test_moving_average_single_sample(self):
        """Moving average with one sample should return that sample's value."""
        snapshots = _make_snapshots([0.85], start_time=100.0)
        avg = compute_moving_average(snapshots, 'oee', 3600.0)
        assert abs(avg - 0.85) < 1e-9


# ---------------------------------------------------------------------------
# Tests — Degradation Warning
# ---------------------------------------------------------------------------

class TestDegradationWarning:
    """Test that degradation warnings are triggered correctly."""

    def test_degradation_warning_logged(self, history, caplog):
        """When trend is degrading, a warning should appear in analysis results."""
        # Build a declining OEE history over 1 hour
        values = [0.90 - 0.25 * i / 119 for i in range(120)]
        for i, v in enumerate(values):
            snap = OEESnapshot(
                timestamp=float(i * 30),
                oee=v, availability=v, performance=v, quality=v,
            )
            history.append_snapshot(snap)

        trends = history.get_trend_analysis()
        assert trends['oee'].trend_direction == 'degrading'
        assert trends['oee'].slope_per_hour < history._degradation_threshold

    def test_no_degradation_for_stable(self, history):
        """Stable trend should not trigger degradation."""
        values = [0.85] * 120
        for i, v in enumerate(values):
            snap = OEESnapshot(
                timestamp=float(i * 30),
                oee=v, availability=v, performance=v, quality=v,
            )
            history.append_snapshot(snap)

        trends = history.get_trend_analysis()
        assert trends['oee'].trend_direction == 'stable'


# ---------------------------------------------------------------------------
# Tests — MTBF / MTTR Real Tracking
# ---------------------------------------------------------------------------

class TestRealMTBFMTTR:
    """Test MTBF and MTTR computation from actual failure events."""

    def test_mtbf_from_failure_events(self):
        """MTBF = total_uptime / failure_count."""
        failure_timestamps = [1000.0, 5000.0, 9000.0]
        total_uptime = 9000.0  # seconds of uptime
        mtbf = compute_real_mtbf(failure_timestamps, total_uptime)
        assert abs(mtbf - 3000.0) < 1e-9

    def test_mtbf_single_failure(self):
        """Single failure should give MTBF = total_uptime."""
        mtbf = compute_real_mtbf([500.0], 3600.0)
        assert abs(mtbf - 3600.0) < 1e-9

    def test_mtbf_no_failures(self):
        """No failures should return 0 (undefined)."""
        assert compute_real_mtbf([], 10000.0) == 0.0

    def test_mttr_from_repair_durations(self):
        """MTTR = sum(repair_durations) / count."""
        repair_durations = [120.0, 180.0, 60.0]
        mttr = compute_real_mttr(repair_durations)
        # (120 + 180 + 60) / 3 = 120
        assert abs(mttr - 120.0) < 1e-9

    def test_mttr_single_repair(self):
        """Single repair duration should equal MTTR."""
        mttr = compute_real_mttr([300.0])
        assert abs(mttr - 300.0) < 1e-9

    def test_mttr_no_repairs(self):
        """No repairs should return 0."""
        assert compute_real_mttr([]) == 0.0

    def test_mtbf_many_failures(self):
        """MTBF across many failures should be accurate."""
        timestamps = [float(i * 100) for i in range(1, 11)]  # 10 failures
        total_uptime = 8000.0
        mtbf = compute_real_mtbf(timestamps, total_uptime)
        assert abs(mtbf - 800.0) < 1e-9


# ---------------------------------------------------------------------------
# Tests — Integration-style (history + trends combined)
# ---------------------------------------------------------------------------

class TestHistoryTrendIntegration:
    """Integration tests combining history storage with trend analysis."""

    def test_full_cycle_store_and_analyze(self, history):
        """Store snapshots then run trend analysis end-to-end."""
        # Slightly improving OEE
        for i in range(60):
            snap = OEESnapshot(
                timestamp=float(i * 30),
                oee=0.80 + 0.002 * i,
                availability=0.90,
                performance=0.85 + 0.001 * i,
                quality=0.95,
            )
            history.append_snapshot(snap)

        trends = history.get_trend_analysis()
        assert trends['oee'].trend_direction == 'improving'
        assert trends['oee'].samples == 60
        assert trends['oee'].current_value == pytest.approx(0.80 + 0.002 * 59, abs=1e-6)

    def test_min_max_tracked(self):
        """Min and max values should be tracked across the full history."""
        values = [0.70, 0.75, 0.80, 0.85, 0.90, 0.95]
        snapshots = _make_snapshots(values, start_time=0.0, interval=30.0)
        trends = _analyze_trends(snapshots)

        assert trends['oee'].min_value == 0.70
        assert trends['oee'].max_value == 0.95

    def test_get_oee_history_time_filter(self, history):
        """get_oee_history should filter by time window."""
        # Add 48 hours of data at 30s intervals
        for i in range(200):
            snap = OEESnapshot(
                timestamp=float(i * 30),
                oee=0.85, availability=0.9,
                performance=0.85, quality=0.95,
            )
            history.append_snapshot(snap)

        # Total span is 200*30 = 6000s ~ 1.67 hours
        # Request last 1 hour from t=6000: cutoff = 6000-3600 = 2400
        # Samples from t=2400 onward: i >= 80 → 120 samples
        filtered = history.get_oee_history(hours=1.0, now=5970.0)
        assert len(filtered) > 0
        assert all(s.timestamp >= 5970.0 - 3600.0 for s in filtered)
