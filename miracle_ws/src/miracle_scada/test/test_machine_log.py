"""Tests for MachineLogAnalyzer.

ROS2 modules are stubbed so the pure dataclasses and classes can be tested
without a ROS2 installation.
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

# Clear cached module so the new mock is picked up
sys.modules.pop('miracle_scada.alert_correlator', None)

import pytest
from typing import List

from miracle_scada.alert_correlator import (
    LogEntry,
    LogPattern,
    LogAnalysisReport,
    MachineLogAnalyzer,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_entry(
    timestamp: float = 0.0,
    level: str = 'INFO',
    source: str = 'spindle',
    message: str = 'status ok',
    machine_id: str = 'cnc_1',
    code: str = 'S000',
) -> LogEntry:
    return LogEntry(
        timestamp=timestamp,
        level=level,
        source=source,
        message=message,
        machine_id=machine_id,
        code=code,
    )


def _populate_analyzer(analyzer: MachineLogAnalyzer) -> None:
    """Add a mixture of log entries to *analyzer* for reuse across tests."""
    entries = [
        _make_entry(1.0, 'INFO', 'spindle', 'Spindle started at 3000 RPM', 'cnc_1', 'S001'),
        _make_entry(2.0, 'WARNING', 'coolant', 'Coolant level low', 'cnc_1', 'C001'),
        _make_entry(3.0, 'ERROR', 'spindle', 'Spindle overtemp 85.3C', 'cnc_1', 'S100'),
        _make_entry(4.0, 'ERROR', 'spindle', 'Spindle overtemp 92.1C', 'cnc_1', 'S100'),
        _make_entry(5.0, 'INFO', 'axis', 'Homing complete', 'cnc_2', 'A001'),
        _make_entry(6.0, 'CRITICAL', 'spindle', 'Spindle stall detected', 'cnc_1', 'S200'),
        _make_entry(7.0, 'ERROR', 'axis', 'Following error on X', 'cnc_2', 'A100'),
        _make_entry(8.0, 'INFO', 'coolant', 'Coolant refilled', 'cnc_1', 'C002'),
        _make_entry(9.0, 'ERROR', 'spindle', 'Spindle overtemp 78.0C', 'cnc_2', 'S100'),
        _make_entry(10.0, 'INFO', 'spindle', 'Spindle started at 5000 RPM', 'cnc_2', 'S001'),
    ]
    for e in entries:
        analyzer.ingest(e)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestIngestAndBasicStorage:
    """Test that ingested entries are stored and retrievable."""

    def test_ingest_single_entry(self):
        analyzer = MachineLogAnalyzer()
        entry = _make_entry(1.0)
        analyzer.ingest(entry)
        results = analyzer.search()
        assert len(results) == 1
        assert results[0] is entry

    def test_ingest_multiple_entries(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        results = analyzer.search()
        assert len(results) == 10


class TestAnalyze:
    """Test the analyze() report generation."""

    def test_analyze_full_range(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        report = analyzer.analyze(0.0, 11.0)

        assert report.total_entries == 10
        assert report.by_level.get('ERROR', 0) == 4
        assert report.by_level.get('CRITICAL', 0) == 1
        assert report.by_level.get('INFO', 0) == 4
        assert report.by_level.get('WARNING', 0) == 1
        assert report.time_range_sec == 11.0
        # 5 errors+criticals in 11 seconds -> error_rate_per_hour > 0
        assert report.error_rate_per_hour > 0

    def test_analyze_sub_range(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        report = analyzer.analyze(3.0, 6.0)
        # entries at t=3,4,5,6
        assert report.total_entries == 4

    def test_analyze_top_errors(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        report = analyzer.analyze(0.0, 11.0)
        # top_errors should be a list of (message, count)
        assert isinstance(report.top_errors, list)
        assert all(isinstance(t, tuple) and len(t) == 2 for t in report.top_errors)


class TestFindPatterns:
    """Test pattern detection via message normalisation."""

    def test_patterns_detected(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        patterns = analyzer.find_patterns(min_frequency=2)
        # "Spindle overtemp <N>C" should appear 3 times (entries at t=3,4,9)
        overtemp = [p for p in patterns if 'overtemp' in p.description.lower()]
        assert len(overtemp) == 1
        assert overtemp[0].frequency == 3

    def test_patterns_min_frequency_filter(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        # Requiring freq >= 5 should return no patterns from the small dataset
        patterns = analyzer.find_patterns(min_frequency=5)
        assert len(patterns) == 0

    def test_pattern_timestamps(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        patterns = analyzer.find_patterns(min_frequency=2)
        for p in patterns:
            assert p.first_seen <= p.last_seen
            assert p.first_seen > 0


class TestGetErrorTimeline:
    """Test error timeline bucketing."""

    def test_timeline_buckets(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        timeline = analyzer.get_error_timeline('cnc_1', granularity_sec=2.0)
        # cnc_1 errors at t=3,4 and critical at t=6
        assert len(timeline) >= 1
        total_errors = sum(count for _, count in timeline)
        assert total_errors == 3  # 2 errors + 1 critical

    def test_timeline_empty_for_unknown_machine(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        timeline = analyzer.get_error_timeline('cnc_99')
        assert timeline == []


class TestSearch:
    """Test keyword / level / machine_id search."""

    def test_search_by_keyword(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        results = analyzer.search(keyword='overtemp')
        assert len(results) == 3

    def test_search_by_level(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        results = analyzer.search(level='ERROR')
        assert len(results) == 4

    def test_search_by_machine(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        results = analyzer.search(machine_id='cnc_2')
        assert len(results) == 4

    def test_search_combined_filters(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        results = analyzer.search(keyword='overtemp', level='ERROR', machine_id='cnc_1')
        assert len(results) == 2


class TestGetEntriesAround:
    """Test the context window retrieval."""

    def test_entries_around(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        # window of 4 sec centred on t=5 -> [3, 7]
        results = analyzer.get_entries_around(5.0, window_sec=4.0)
        timestamps = [e.timestamp for e in results]
        assert all(3.0 <= t <= 7.0 for t in timestamps)
        assert len(results) == 5  # entries at 3,4,5,6,7

    def test_entries_around_narrow_window(self):
        analyzer = MachineLogAnalyzer()
        _populate_analyzer(analyzer)
        results = analyzer.get_entries_around(1.0, window_sec=0.5)
        assert len(results) == 1
        assert results[0].timestamp == 1.0
