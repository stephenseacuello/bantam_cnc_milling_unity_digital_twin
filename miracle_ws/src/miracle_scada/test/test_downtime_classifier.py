"""Tests for the DowntimeClassifier downtime-event tracking system.

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

import math
import pytest

from miracle_scada.kpi_calculator import (
    DowntimeEvent,
    DowntimeSummary,
    DowntimeClassifier,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_event(
    event_id: str = 'evt_1',
    machine_id: str = 'cnc_1',
    start_time: float = 0.0,
    end_time: float = None,
    category: str = 'unplanned',
    reason: str = 'spindle fault',
    resolved: bool = False,
) -> DowntimeEvent:
    return DowntimeEvent(
        event_id=event_id,
        machine_id=machine_id,
        start_time=start_time,
        end_time=end_time,
        category=category,
        reason=reason,
        resolved=resolved,
    )


def _classifier_with_events():
    """Return a classifier pre-loaded with several events for testing."""
    clf = DowntimeClassifier()
    # Event 1: unplanned, cnc_1, 10 min (0 -> 600s)
    clf.record_event(_make_event(
        event_id='e1', machine_id='cnc_1',
        start_time=0.0, end_time=600.0,
        category='unplanned', reason='spindle fault', resolved=True,
    ))
    # Event 2: planned, cnc_1, 30 min (1000 -> 2800s)
    clf.record_event(_make_event(
        event_id='e2', machine_id='cnc_1',
        start_time=1000.0, end_time=2800.0,
        category='planned', reason='scheduled PM', resolved=True,
    ))
    # Event 3: changeover, cnc_2, 15 min (500 -> 1400s)
    clf.record_event(_make_event(
        event_id='e3', machine_id='cnc_2',
        start_time=500.0, end_time=1400.0,
        category='changeover', reason='tool change', resolved=True,
    ))
    # Event 4: unplanned, cnc_2, 5 min (2000 -> 2300s)
    clf.record_event(_make_event(
        event_id='e4', machine_id='cnc_2',
        start_time=2000.0, end_time=2300.0,
        category='unplanned', reason='spindle fault', resolved=True,
    ))
    # Event 5: open event (not resolved), cnc_1
    clf.record_event(_make_event(
        event_id='e5', machine_id='cnc_1',
        start_time=3000.0, end_time=None,
        category='maintenance', reason='bearing inspection', resolved=False,
    ))
    return clf


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestDowntimeEventDataclass:
    """Test DowntimeEvent dataclass behaviour."""

    def test_duration_min_closed_event(self):
        ev = _make_event(start_time=0.0, end_time=600.0)
        assert ev.duration_min() == pytest.approx(10.0)

    def test_duration_min_open_event(self):
        ev = _make_event(start_time=0.0, end_time=None)
        assert ev.duration_min() == 0.0


class TestRecordEvent:
    """Test event recording and validation."""

    def test_record_valid_event(self):
        clf = DowntimeClassifier()
        ev = _make_event(category='planned')
        clf.record_event(ev)
        assert len(clf.get_events_by_machine('cnc_1')) == 1

    def test_record_invalid_category_raises(self):
        clf = DowntimeClassifier()
        ev = _make_event(category='bogus')
        with pytest.raises(ValueError, match='Invalid category'):
            clf.record_event(ev)

    def test_record_duplicate_id_raises(self):
        clf = DowntimeClassifier()
        ev1 = _make_event(event_id='dup')
        ev2 = _make_event(event_id='dup')
        clf.record_event(ev1)
        with pytest.raises(ValueError, match='already exists'):
            clf.record_event(ev2)


class TestCloseEvent:
    """Test closing / resolving events."""

    def test_close_event_marks_resolved(self):
        clf = DowntimeClassifier()
        ev = _make_event(event_id='c1', start_time=0.0)
        clf.record_event(ev)
        clf.close_event('c1', end_time=300.0)
        closed = clf.get_events_by_machine('cnc_1')
        assert closed[0].resolved is True
        assert closed[0].end_time == 300.0
        assert closed[0].duration_min() == pytest.approx(5.0)

    def test_close_nonexistent_event_raises(self):
        clf = DowntimeClassifier()
        with pytest.raises(KeyError, match='not found'):
            clf.close_event('missing', end_time=100.0)


class TestGetSummary:
    """Test summary generation over a time window."""

    def test_summary_covers_full_range(self):
        clf = _classifier_with_events()
        summary = clf.get_summary(0.0, 3000.0)
        # e1 = 10 min, e2 = 30 min, e3 = 15 min, e4 = 5 min
        assert summary.total_downtime_min == pytest.approx(60.0)
        assert summary.planned_min == pytest.approx(30.0)
        assert summary.unplanned_min == pytest.approx(30.0)  # 10 + 15 + 5
        assert summary.event_count == 4  # open event excluded

    def test_summary_clips_to_window(self):
        clf = _classifier_with_events()
        # Window 0-1000s: e1 fully inside (10 min), e3 partially (500-1000 = 8.33 min)
        summary = clf.get_summary(0.0, 1000.0)
        assert summary.total_downtime_min == pytest.approx(10.0 + 500.0 / 60.0)
        assert summary.event_count == 2

    def test_summary_mttr(self):
        clf = _classifier_with_events()
        summary = clf.get_summary(0.0, 3000.0)
        # MTTR = mean of full durations of resolved events
        # e1=10, e2=30, e3=15, e4=5  => mean = 60/4 = 15
        assert summary.mttr_min == pytest.approx(15.0)

    def test_summary_by_machine(self):
        clf = _classifier_with_events()
        summary = clf.get_summary(0.0, 3000.0)
        # cnc_1: e1(10) + e2(30) = 40,  cnc_2: e3(15) + e4(5) = 20
        assert summary.by_machine['cnc_1'] == pytest.approx(40.0)
        assert summary.by_machine['cnc_2'] == pytest.approx(20.0)

    def test_summary_top_reasons(self):
        clf = _classifier_with_events()
        summary = clf.get_summary(0.0, 3000.0)
        # 'spindle fault': 2 events, 15 min total;  'scheduled PM': 1, 30 min;
        # 'tool change': 1, 15 min
        # Sorted by total duration: scheduled PM (30), spindle fault (15), tool change (15)
        assert summary.top_reasons[0][0] == 'scheduled PM'
        assert summary.top_reasons[0][2] == pytest.approx(30.0)


class TestGetOpenEvents:
    """Test retrieval of unresolved events."""

    def test_returns_only_open(self):
        clf = _classifier_with_events()
        opens = clf.get_open_events()
        assert len(opens) == 1
        assert opens[0].event_id == 'e5'


class TestParetoReasons:
    """Test Pareto analysis of downtime reasons."""

    def test_pareto_top_n(self):
        clf = _classifier_with_events()
        pareto = clf.get_pareto_reasons(top_n=2)
        assert len(pareto) == 2
        # Top reason by total duration
        assert pareto[0][0] == 'scheduled PM'
        assert pareto[0][2] == pytest.approx(30.0)

    def test_pareto_all(self):
        clf = _classifier_with_events()
        pareto = clf.get_pareto_reasons(top_n=100)
        assert len(pareto) == 4  # 4 distinct reasons


class TestCalculateAvailability:
    """Test OEE availability calculation."""

    def test_full_availability(self):
        clf = DowntimeClassifier()
        avail = clf.calculate_availability('cnc_1', 480.0, 0.0, 28800.0)
        assert avail == pytest.approx(1.0)

    def test_partial_availability(self):
        clf = _classifier_with_events()
        # cnc_1 downtime in [0, 3000]: 40 min.  Planned = 50 min.
        # availability = (50 - 40) / 50 = 0.2
        avail = clf.calculate_availability('cnc_1', 50.0, 0.0, 3000.0)
        assert avail == pytest.approx(0.2)

    def test_zero_planned_returns_one(self):
        clf = _classifier_with_events()
        avail = clf.calculate_availability('cnc_1', 0.0, 0.0, 3000.0)
        assert avail == pytest.approx(1.0)

    def test_availability_clamped_to_zero(self):
        clf = _classifier_with_events()
        # Planned = 10 min but downtime = 40 min → clamped to 0
        avail = clf.calculate_availability('cnc_1', 10.0, 0.0, 3000.0)
        assert avail == pytest.approx(0.0)
