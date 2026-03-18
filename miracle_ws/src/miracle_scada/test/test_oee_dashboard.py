"""Tests for OEEDashboardProvider — multi-machine OEE dashboard aggregation.

Uses the same mock pattern as test_capability_profiler.py — ROS2 modules are
stubbed so the pure dataclasses and classes can be tested without a ROS2
installation.
"""

import sys
from unittest.mock import MagicMock

# Mock ROS2 and miracle_core sub-module dependencies before importing
for _mod in ['rclpy', 'rclpy.node', 'rclpy.callback_groups', 'rclpy.qos', 'rclpy.lifecycle',
             'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
             'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
             'miracle_core.exceptions',
             'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(_mod, MagicMock())

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

sys.modules.pop('miracle_scada.kpi_calculator', None)

import pytest

from miracle_scada.kpi_calculator import (
    DashboardOEESnapshot,
    OEETrend,
    DashboardSummary,
    OEEDashboardProvider,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_snapshot(
    machine_id: str = 'cnc_01',
    timestamp: float = 1000.0,
    availability: float = 0.90,
    performance: float = 0.85,
    quality: float = 0.98,
    oee: float | None = None,
    good_parts: int = 98,
    total_parts: int = 100,
    planned_time_min: float = 480.0,
    actual_run_time_min: float = 432.0,
) -> DashboardOEESnapshot:
    if oee is None:
        oee = availability * performance * quality
    return DashboardOEESnapshot(
        machine_id=machine_id,
        timestamp=timestamp,
        availability=availability,
        performance=performance,
        quality=quality,
        oee=oee,
        good_parts=good_parts,
        total_parts=total_parts,
        planned_time_min=planned_time_min,
        actual_run_time_min=actual_run_time_min,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestRecordAndSummary:
    """Test basic recording and dashboard summary generation."""

    def test_record_snapshot_stores_data(self):
        provider = OEEDashboardProvider()
        snap = _make_snapshot()
        provider.record_snapshot(snap)
        summary = provider.get_dashboard_summary()

        assert 'cnc_01' in summary.machines
        assert summary.machines['cnc_01'] is snap

    def test_summary_weighted_oee(self):
        """Overall OEE should be the weighted average by planned_time_min."""
        provider = OEEDashboardProvider()
        # Machine A: OEE 0.80, planned 600 min
        provider.record_snapshot(_make_snapshot(
            machine_id='A', oee=0.80, planned_time_min=600.0,
        ))
        # Machine B: OEE 0.60, planned 200 min
        provider.record_snapshot(_make_snapshot(
            machine_id='B', oee=0.60, planned_time_min=200.0,
        ))

        summary = provider.get_dashboard_summary()
        # Expected: (0.80*600 + 0.60*200) / 800 = 600/800 = 0.75
        expected = (0.80 * 600.0 + 0.60 * 200.0) / 800.0
        assert abs(summary.overall_oee - expected) < 1e-9

    def test_summary_best_and_worst(self):
        provider = OEEDashboardProvider()
        provider.record_snapshot(_make_snapshot(machine_id='A', oee=0.90))
        provider.record_snapshot(_make_snapshot(machine_id='B', oee=0.50))
        provider.record_snapshot(_make_snapshot(machine_id='C', oee=0.70))

        summary = provider.get_dashboard_summary()
        assert summary.best_machine == 'A'
        assert summary.worst_machine == 'B'

    def test_empty_summary(self):
        provider = OEEDashboardProvider()
        summary = provider.get_dashboard_summary()
        assert summary.overall_oee == 0.0
        assert summary.best_machine == ''
        assert summary.worst_machine == ''
        assert summary.machines == {}


class TestTrend:
    """Test trend retrieval and direction classification."""

    def test_trend_returns_correct_points(self):
        provider = OEEDashboardProvider()
        for i in range(10):
            provider.record_snapshot(_make_snapshot(
                machine_id='cnc_01', timestamp=float(i), oee=0.80,
            ))

        trend = provider.get_trend('cnc_01', num_points=5)
        assert len(trend.timestamps) == 5
        assert trend.timestamps == [5.0, 6.0, 7.0, 8.0, 9.0]

    def test_trend_improving(self):
        provider = OEEDashboardProvider()
        for i in range(20):
            provider.record_snapshot(_make_snapshot(
                machine_id='cnc_01',
                timestamp=float(i),
                oee=0.50 + i * 0.02,  # rising steadily
            ))

        trend = provider.get_trend('cnc_01', num_points=20)
        assert trend.trend_direction == 'improving'

    def test_trend_declining(self):
        provider = OEEDashboardProvider()
        for i in range(20):
            provider.record_snapshot(_make_snapshot(
                machine_id='cnc_01',
                timestamp=float(i),
                oee=0.90 - i * 0.02,  # falling steadily
            ))

        trend = provider.get_trend('cnc_01', num_points=20)
        assert trend.trend_direction == 'declining'

    def test_trend_stable(self):
        provider = OEEDashboardProvider()
        for i in range(20):
            provider.record_snapshot(_make_snapshot(
                machine_id='cnc_01',
                timestamp=float(i),
                oee=0.80,
            ))

        trend = provider.get_trend('cnc_01', num_points=20)
        assert trend.trend_direction == 'stable'

    def test_trend_unknown_machine(self):
        provider = OEEDashboardProvider()
        trend = provider.get_trend('nonexistent')
        assert trend.timestamps == []
        assert trend.trend_direction == 'stable'


class TestAlerts:
    """Test alert generation against OEE / availability / quality thresholds."""

    def test_oee_alert(self):
        provider = OEEDashboardProvider()
        provider.record_snapshot(_make_snapshot(
            machine_id='cnc_01', oee=0.60, availability=0.90, quality=0.99,
        ))
        alerts = provider.get_alerts()
        assert any('OEE' in a and 'cnc_01' in a for a in alerts)

    def test_availability_alert(self):
        provider = OEEDashboardProvider()
        provider.record_snapshot(_make_snapshot(
            machine_id='cnc_01', oee=0.70, availability=0.75, quality=0.99,
        ))
        alerts = provider.get_alerts()
        assert any('Availability' in a and 'cnc_01' in a for a in alerts)

    def test_quality_alert(self):
        provider = OEEDashboardProvider()
        provider.record_snapshot(_make_snapshot(
            machine_id='cnc_01', oee=0.70, availability=0.90, quality=0.90,
        ))
        alerts = provider.get_alerts()
        assert any('Quality' in a and 'cnc_01' in a for a in alerts)

    def test_no_alerts_when_healthy(self):
        provider = OEEDashboardProvider()
        provider.record_snapshot(_make_snapshot(
            machine_id='cnc_01', oee=0.85, availability=0.95, quality=0.99,
        ))
        alerts = provider.get_alerts()
        assert alerts == []


class TestCompareMachines:
    """Test machine ranking."""

    def test_ranking_order(self):
        provider = OEEDashboardProvider()
        provider.record_snapshot(_make_snapshot(machine_id='A', oee=0.60))
        provider.record_snapshot(_make_snapshot(machine_id='B', oee=0.90))
        provider.record_snapshot(_make_snapshot(machine_id='C', oee=0.75))

        ranking = provider.compare_machines()
        assert [m for m, _ in ranking] == ['B', 'C', 'A']


class TestShiftComparison:
    """Test shift-level OEE comparison."""

    def test_shift_averages(self):
        provider = OEEDashboardProvider()
        # Shift 1: timestamps 0-100
        for i in range(5):
            provider.record_snapshot(_make_snapshot(
                machine_id='cnc_01', timestamp=float(i * 10), oee=0.80,
                availability=0.90, performance=0.90, quality=0.99,
            ))
        # Shift 2: timestamps 100-200
        for i in range(5):
            provider.record_snapshot(_make_snapshot(
                machine_id='cnc_01', timestamp=100.0 + i * 10, oee=0.60,
                availability=0.75, performance=0.80, quality=0.95,
            ))

        shifts = [(0.0, 100.0), (100.0, 200.0)]
        results = provider.get_shift_comparison('cnc_01', shifts)

        assert len(results) == 2
        assert abs(results[0]['avg_oee'] - 0.80) < 1e-9
        assert abs(results[1]['avg_oee'] - 0.60) < 1e-9
        assert results[0]['sample_count'] == 5
        assert results[1]['sample_count'] == 5

    def test_shift_no_data(self):
        provider = OEEDashboardProvider()
        results = provider.get_shift_comparison('cnc_01', [(0.0, 100.0)])
        assert results[0]['sample_count'] == 0
        assert results[0]['avg_oee'] == 0.0
