"""Tests for the AlarmAnalyticsEngine.

Uses the same ROS2 mock pattern as test_capability_profiler.py so that the
module can be imported without a live ROS2 installation.
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

# Force reimport so the mock is picked up
sys.modules.pop('miracle_scada.alert_correlator', None)

import pytest
from typing import List

from miracle_scada.alert_correlator import (
    AlarmHistoryEntry,
    AlarmTrendAnalysis,
    AlarmAnalyticsSummary,
    AlarmAnalyticsEngine,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _entry(
    alarm_id: str = 'A001',
    alarm_type: str = 'overtemp',
    machine_id: str = 'cnc_1',
    severity: float = 0.8,
    timestamp: float = 1000.0,
    duration_sec: float = 30.0,
    acknowledged: bool = True,
    root_cause: str = 'spindle_overload',
) -> AlarmHistoryEntry:
    return AlarmHistoryEntry(
        alarm_id=alarm_id,
        alarm_type=alarm_type,
        machine_id=machine_id,
        severity=severity,
        timestamp=timestamp,
        duration_sec=duration_sec,
        acknowledged=acknowledged,
        root_cause=root_cause,
    )


def _populated_engine() -> AlarmAnalyticsEngine:
    """Return an engine pre-loaded with a small but diverse set of alarms."""
    engine = AlarmAnalyticsEngine()
    entries = [
        _entry('A001', 'overtemp',    'cnc_1', 0.9, 1000, 20, True,  'spindle_overload'),
        _entry('A002', 'overtemp',    'cnc_1', 0.8, 1050, 25, True,  'spindle_overload'),
        _entry('A003', 'vibration',   'cnc_1', 0.7, 1100, 15, True,  'bearing_wear'),
        _entry('A004', 'overtemp',    'cnc_2', 0.6, 1200, 40, True,  'coolant_failure'),
        _entry('A005', 'collision',   'cnc_2', 0.95, 1300, 0,  False, 'path_error'),
        _entry('A006', 'vibration',   'cnc_3', 0.5, 1400, 10, True,  'unbalanced_tool'),
        _entry('A007', 'overtemp',    'cnc_1', 0.85, 2000, 30, True,  'spindle_overload'),
        _entry('A008', 'overtemp',    'cnc_1', 0.88, 2050, 35, True,  'spindle_overload'),
    ]
    for e in entries:
        engine.record_alarm(e)
    return engine


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestAlarmHistoryEntry:
    """Test the AlarmHistoryEntry dataclass."""

    def test_dataclass_fields(self):
        entry = _entry()
        assert entry.alarm_id == 'A001'
        assert entry.alarm_type == 'overtemp'
        assert entry.machine_id == 'cnc_1'
        assert entry.severity == 0.8
        assert entry.timestamp == 1000.0
        assert entry.duration_sec == 30.0
        assert entry.acknowledged is True
        assert entry.root_cause == 'spindle_overload'


class TestRecordAlarm:
    """Test recording alarms into the engine."""

    def test_record_single(self):
        engine = AlarmAnalyticsEngine()
        engine.record_alarm(_entry())
        summary = engine.get_summary(0, 9999)
        assert summary.total_alarms == 1

    def test_record_multiple(self):
        engine = _populated_engine()
        summary = engine.get_summary(0, 9999)
        assert summary.total_alarms == 8


class TestGetSummary:
    """Test the get_summary analytical query."""

    def test_empty_range(self):
        engine = _populated_engine()
        summary = engine.get_summary(5000, 6000)
        assert summary.total_alarms == 0
        assert summary.unique_types == 0
        assert summary.alarm_rate_per_hour == 0.0

    def test_full_range_counts(self):
        engine = _populated_engine()
        summary = engine.get_summary(0, 9999)
        assert summary.total_alarms == 8
        assert summary.unique_types == 3  # overtemp, vibration, collision

    def test_top_alarm_types_ordering(self):
        engine = _populated_engine()
        summary = engine.get_summary(0, 9999)
        # overtemp appears 5 times, vibration 2, collision 1
        assert summary.top_alarm_types[0] == ('overtemp', 5)

    def test_top_machines_ordering(self):
        engine = _populated_engine()
        summary = engine.get_summary(0, 9999)
        # cnc_1 has 5 alarms (A001, A002, A003, A007, A008)
        assert summary.top_machines[0] == ('cnc_1', 5)

    def test_avg_response_time(self):
        engine = _populated_engine()
        summary = engine.get_summary(0, 9999)
        # 7 acknowledged alarms with durations: 20, 25, 15, 40, 10, 30, 35
        expected = (20 + 25 + 15 + 40 + 10 + 30 + 35) / 7.0
        assert abs(summary.avg_response_time_sec - expected) < 0.01

    def test_alarm_rate_per_hour(self):
        engine = _populated_engine()
        summary = engine.get_summary(1000, 2050)
        span_hours = (2050 - 1000) / 3600.0
        expected_rate = 8 / span_hours
        assert abs(summary.alarm_rate_per_hour - expected_rate) < 0.01

    def test_repeat_alarm_pct(self):
        engine = _populated_engine()
        summary = engine.get_summary(0, 9999)
        # Pairs with >1 occurrence: (overtemp, cnc_1)=4, so repeat_count=4
        # total=8, pct = 4/8*100 = 50.0
        assert summary.repeat_alarm_pct > 0.0


class TestGetTrend:
    """Test the trend analysis query."""

    def test_no_matching_type(self):
        engine = _populated_engine()
        trend = engine.get_trend('nonexistent', periods=3, period_duration_sec=500)
        assert trend.alarm_type == 'nonexistent'
        assert trend.count_trend == [0, 0, 0]
        assert trend.increasing is False

    def test_trend_increasing_flag(self):
        engine = AlarmAnalyticsEngine()
        # Create an increasing pattern: 1 alarm early, 5 alarms late
        engine.record_alarm(_entry(alarm_id='E1', timestamp=100))
        for i in range(5):
            engine.record_alarm(_entry(alarm_id=f'L{i}', timestamp=900 + i * 10))
        trend = engine.get_trend('overtemp', periods=4, period_duration_sec=250)
        assert trend.increasing is True

    def test_trend_period_labels(self):
        engine = _populated_engine()
        trend = engine.get_trend('overtemp', periods=3)
        assert len(trend.period_labels) == 3
        assert trend.period_labels == ['P0', 'P1', 'P2']


class TestGetRepeatAlarms:
    """Test repeat alarm detection."""

    def test_finds_repeat_group(self):
        engine = _populated_engine()
        # overtemp on cnc_1 at 1000, 1050 — within a 100s window
        groups = engine.get_repeat_alarms(window_sec=100)
        overtemp_cnc1 = [
            g for g in groups
            if g[0].alarm_type == 'overtemp' and g[0].machine_id == 'cnc_1'
        ]
        assert len(overtemp_cnc1) >= 1
        assert len(overtemp_cnc1[0]) >= 2

    def test_no_repeats_tiny_window(self):
        engine = AlarmAnalyticsEngine()
        engine.record_alarm(_entry(alarm_id='X1', timestamp=1000))
        engine.record_alarm(_entry(alarm_id='X2', timestamp=5000))
        groups = engine.get_repeat_alarms(window_sec=1)
        assert groups == []


class TestGetMTTR:
    """Test mean-time-to-respond calculation."""

    def test_mttr_all_types(self):
        engine = _populated_engine()
        mttr = engine.get_mttr()
        # 7 acknowledged alarms
        expected = (20 + 25 + 15 + 40 + 10 + 30 + 35) / 7.0
        assert abs(mttr - expected) < 0.01

    def test_mttr_specific_type(self):
        engine = _populated_engine()
        mttr = engine.get_mttr('overtemp')
        # overtemp acknowledged: durations 20, 25, 40, 30, 35
        expected = (20 + 25 + 40 + 30 + 35) / 5.0
        assert abs(mttr - expected) < 0.01

    def test_mttr_no_acknowledged(self):
        engine = AlarmAnalyticsEngine()
        engine.record_alarm(_entry(acknowledged=False))
        assert engine.get_mttr() == 0.0


class TestGetAlarmHeatmap:
    """Test alarm heatmap generation."""

    def test_heatmap_dimensions(self):
        engine = _populated_engine()
        machines = ['cnc_1', 'cnc_2', 'cnc_3']
        types = ['overtemp', 'vibration', 'collision']
        heatmap = engine.get_alarm_heatmap(machines, types)
        assert len(heatmap) == 3
        assert all(len(row) == 3 for row in heatmap)

    def test_heatmap_values(self):
        engine = _populated_engine()
        machines = ['cnc_1', 'cnc_2', 'cnc_3']
        types = ['overtemp', 'vibration', 'collision']
        heatmap = engine.get_alarm_heatmap(machines, types)
        # cnc_1: overtemp=4, vibration=1, collision=0
        assert heatmap[0] == [4, 1, 0]
        # cnc_2: overtemp=1, vibration=0, collision=1
        assert heatmap[1] == [1, 0, 1]
        # cnc_3: overtemp=0, vibration=1, collision=0
        assert heatmap[2] == [0, 1, 0]

    def test_heatmap_unknown_machine(self):
        engine = _populated_engine()
        heatmap = engine.get_alarm_heatmap(['unknown_machine'], ['overtemp'])
        assert heatmap == [[0]]
