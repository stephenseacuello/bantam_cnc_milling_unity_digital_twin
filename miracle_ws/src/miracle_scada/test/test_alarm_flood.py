"""Tests for AlarmFloodDetector — alarm rate limiting and flood detection.

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

# Force reimport of the module under test
sys.modules.pop('miracle_scada.alarm_manager', None)

import pytest

from miracle_scada.alarm_manager import (
    AlarmFloodDetector,
    AlarmFloodEvent,
    AlarmSuppression,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_detector(**kwargs):
    """Create an AlarmFloodDetector with sensible test defaults."""
    defaults = dict(flood_threshold=10, window_sec=60.0, cooldown_sec=30.0)
    defaults.update(kwargs)
    return AlarmFloodDetector(**defaults)


def _send_alarms(detector, count, alarm_type='THERMAL', severity=0.5,
                 start_time=100.0, interval=1.0, id_prefix='a'):
    """Send *count* alarms to *detector* and return list of (alarm_id, forwarded)."""
    results = []
    for i in range(count):
        aid = f'{id_prefix}_{i}'
        t = start_time + i * interval
        forwarded = detector.record_alarm(aid, severity, alarm_type, t)
        results.append((aid, forwarded))
    return results


# ---------------------------------------------------------------------------
# Normal alarm rate (below threshold) — all forwarded
# ---------------------------------------------------------------------------

class TestNormalRate:
    def test_all_forwarded_below_threshold(self):
        """Alarms arriving below the flood threshold are all forwarded."""
        detector = _make_detector(flood_threshold=10, window_sec=60.0)
        # Send 5 alarms in 60 seconds — rate = 5 per minute, below threshold
        results = _send_alarms(detector, 5, start_time=100.0, interval=12.0)
        assert all(forwarded for _, forwarded in results)

    def test_no_flood_event_below_threshold(self):
        """No flood event is created when rate stays below threshold."""
        detector = _make_detector(flood_threshold=10, window_sec=60.0)
        _send_alarms(detector, 5, start_time=100.0, interval=12.0)
        assert detector.get_flood_status() is None

    def test_suppression_log_empty(self):
        """Suppression log is empty when no flood occurs."""
        detector = _make_detector(flood_threshold=10, window_sec=60.0)
        _send_alarms(detector, 5, start_time=100.0, interval=12.0)
        assert detector.get_suppression_log() == []


# ---------------------------------------------------------------------------
# Flood detection when rate exceeds threshold
# ---------------------------------------------------------------------------

class TestFloodDetection:
    def test_flood_detected_at_threshold(self):
        """A flood event is created once the rate hits the threshold."""
        detector = _make_detector(flood_threshold=10, window_sec=60.0)
        # 10 alarms in 5 seconds => rate = 10 * (60/60) = 10/min at the window level,
        # but since they all land within the 60-second window the count is 10
        # and rate = 10 * 60/60 = 10.  That equals threshold.
        _send_alarms(detector, 10, start_time=100.0, interval=0.5)
        flood = detector.get_flood_status()
        assert flood is not None
        assert flood.is_active is True

    def test_flood_event_tracks_alarm_count(self):
        """The alarm_count field reflects how many alarms arrived during the flood."""
        detector = _make_detector(flood_threshold=5, window_sec=60.0)
        _send_alarms(detector, 8, start_time=100.0, interval=0.5)
        flood = detector.get_flood_status()
        assert flood is not None
        # First 5 trigger the flood; alarms 6-8 happen during flood
        assert flood.alarm_count >= 3


# ---------------------------------------------------------------------------
# Duplicate suppression during flood
# ---------------------------------------------------------------------------

class TestDuplicateSuppression:
    def test_duplicate_type_suppressed(self):
        """Duplicate alarm_types are suppressed during a flood."""
        detector = _make_detector(flood_threshold=5, window_sec=60.0)
        # Trigger flood with THERMAL alarms
        _send_alarms(detector, 5, alarm_type='THERMAL', start_time=100.0, interval=0.1)
        # Now send more THERMAL alarms — should be suppressed
        results = _send_alarms(detector, 3, alarm_type='THERMAL', severity=0.5,
                               start_time=100.6, interval=0.1, id_prefix='dup')
        suppressed = [aid for aid, fwd in results if not fwd]
        assert len(suppressed) > 0

    def test_different_type_forwarded_during_flood(self):
        """A new alarm_type is forwarded even during a flood."""
        detector = _make_detector(flood_threshold=5, window_sec=60.0)
        # Trigger flood with THERMAL
        _send_alarms(detector, 6, alarm_type='THERMAL', start_time=100.0, interval=0.1)
        # New type SAFETY should be forwarded
        forwarded = detector.record_alarm('safety_1', 0.9, 'SAFETY', 100.7)
        assert forwarded is True


# ---------------------------------------------------------------------------
# Highest severity alarm passes through flood
# ---------------------------------------------------------------------------

class TestHighestSeverityPassthrough:
    def test_higher_severity_supersedes(self):
        """A higher-severity alarm of the same type passes through."""
        detector = _make_detector(flood_threshold=5, window_sec=60.0)
        # Trigger flood
        _send_alarms(detector, 5, alarm_type='THERMAL', severity=0.3,
                     start_time=100.0, interval=0.1)
        # Send a higher-severity alarm of the same type
        forwarded = detector.record_alarm('high_sev', 0.9, 'THERMAL', 100.6)
        assert forwarded is True

    def test_lower_severity_suppressed(self):
        """A lower-severity alarm of the same type is suppressed."""
        detector = _make_detector(flood_threshold=5, window_sec=60.0)
        _send_alarms(detector, 5, alarm_type='THERMAL', severity=0.8,
                     start_time=100.0, interval=0.1)
        forwarded = detector.record_alarm('low_sev', 0.2, 'THERMAL', 100.6)
        assert forwarded is False


# ---------------------------------------------------------------------------
# Flood event start/end tracking
# ---------------------------------------------------------------------------

class TestFloodEventLifecycle:
    def test_flood_start_time_recorded(self):
        """start_time matches the moment the flood threshold was reached."""
        detector = _make_detector(flood_threshold=5, window_sec=60.0)
        _send_alarms(detector, 5, start_time=200.0, interval=0.1)
        flood = detector.get_flood_status()
        assert flood is not None
        assert flood.start_time == pytest.approx(200.4, abs=0.01)

    def test_flood_end_time_set_when_subsided(self):
        """end_time and is_active are updated once the flood subsides."""
        detector = _make_detector(flood_threshold=5, window_sec=60.0, cooldown_sec=0.0)
        # Trigger flood: 5 alarms in quick succession within the 60s window
        _send_alarms(detector, 6, start_time=100.0, interval=0.1)
        flood = detector.get_flood_status()
        assert flood is not None and flood.is_active is True

        # Send an alarm well outside the window (>60s later) so old arrivals
        # fall out and count drops below threshold
        forwarded = detector.record_alarm('late', 0.5, 'THERMAL', 200.0)
        flood = detector.get_flood_status()
        assert flood is not None
        assert flood.is_active is False
        assert flood.end_time == pytest.approx(200.0, abs=0.01)

    def test_unique_alarm_types_tracked(self):
        """unique_alarm_types accumulates distinct types during a flood."""
        detector = _make_detector(flood_threshold=4, window_sec=60.0)
        # Send 2 THERMAL + 2 SAFETY = 4 alarms to trigger flood on 4th alarm
        detector.record_alarm('th_0', 0.5, 'THERMAL', 100.0)
        detector.record_alarm('th_1', 0.5, 'THERMAL', 100.1)
        detector.record_alarm('sf_0', 0.5, 'SAFETY', 100.2)
        # 4th alarm triggers flood — the alarm_type on this one is captured
        detector.record_alarm('sf_1', 0.5, 'SAFETY', 100.3)
        # 5th and 6th arrive during flood and add to unique_alarm_types
        detector.record_alarm('th_2', 0.5, 'THERMAL', 100.4)
        detector.record_alarm('sf_2', 0.5, 'SAFETY', 100.5)
        flood = detector.get_flood_status()
        assert flood is not None
        assert 'THERMAL' in flood.unique_alarm_types
        assert 'SAFETY' in flood.unique_alarm_types


# ---------------------------------------------------------------------------
# Suppression log records
# ---------------------------------------------------------------------------

class TestSuppressionLog:
    def test_suppression_entries_created(self):
        """Each suppressed alarm creates an AlarmSuppression entry."""
        detector = _make_detector(flood_threshold=5, window_sec=60.0)
        _send_alarms(detector, 5, alarm_type='THERMAL', severity=0.5,
                     start_time=100.0, interval=0.1)
        # These should be suppressed (duplicates)
        _send_alarms(detector, 3, alarm_type='THERMAL', severity=0.3,
                     start_time=100.6, interval=0.1, id_prefix='dup')
        log = detector.get_suppression_log()
        assert len(log) > 0
        assert all(isinstance(entry, AlarmSuppression) for entry in log)

    def test_suppression_fields_populated(self):
        """AlarmSuppression entries have correct field values."""
        detector = _make_detector(flood_threshold=5, window_sec=60.0)
        _send_alarms(detector, 5, alarm_type='TOOL', severity=0.7,
                     start_time=100.0, interval=0.1)
        detector.record_alarm('suppressed_1', 0.3, 'TOOL', 100.6)
        log = detector.get_suppression_log()
        assert len(log) >= 1
        entry = log[0]
        assert entry.alarm_id == 'suppressed_1'
        assert entry.original_severity == pytest.approx(0.3)
        assert entry.suppressed_at == pytest.approx(100.6)
        assert 'TOOL' in entry.reason
        assert entry.would_have_notified is True

    def test_suppression_log_is_copy(self):
        """get_suppression_log returns a copy, not the internal list."""
        detector = _make_detector(flood_threshold=5, window_sec=60.0)
        log1 = detector.get_suppression_log()
        log1.append(AlarmSuppression('fake', 'test', 0.0, 0.0))
        log2 = detector.get_suppression_log()
        assert len(log2) == 0


# ---------------------------------------------------------------------------
# Alarm rate calculation
# ---------------------------------------------------------------------------

class TestAlarmRateCalculation:
    def test_rate_zero_initially(self):
        """Rate is zero with no alarms recorded."""
        detector = _make_detector()
        assert detector.alarm_rate_per_minute() == 0.0

    def test_rate_reflects_arrivals(self):
        """Rate is positive and proportional to alarm count."""
        detector = _make_detector(flood_threshold=100, window_sec=60.0)
        # 6 alarms from t=100 to t=125
        _send_alarms(detector, 6, start_time=100.0, interval=5.0)
        rate = detector.alarm_rate_per_minute(current_time=125.0)
        # 6 alarms over 25s actual span => 6*(60/25) = 14.4/min
        assert rate > 0
        # More alarms -> higher rate
        _send_alarms(detector, 6, start_time=126.0, interval=1.0, id_prefix='b')
        rate2 = detector.alarm_rate_per_minute(current_time=132.0)
        assert rate2 > rate

    def test_rate_drops_after_window_expires(self):
        """Old arrivals fall out of the sliding window."""
        detector = _make_detector(flood_threshold=100, window_sec=10.0)
        _send_alarms(detector, 5, start_time=100.0, interval=1.0)
        # At t=115, the 10-second window is [105, 115) — none of the
        # arrivals (100..104) are inside, so rate should be 0.
        rate = detector.alarm_rate_per_minute(current_time=115.0)
        assert rate == 0.0


# ---------------------------------------------------------------------------
# Cooldown period after flood ends
# ---------------------------------------------------------------------------

class TestCooldownPeriod:
    def test_flood_does_not_retrigger_during_cooldown(self):
        """A new burst during cooldown does not create a second flood event."""
        detector = _make_detector(flood_threshold=5, window_sec=60.0, cooldown_sec=30.0)
        # Trigger a flood
        _send_alarms(detector, 6, start_time=100.0, interval=0.1)
        # Let the window slide so flood ends (send alarm >60s later)
        detector.record_alarm('end', 0.5, 'THERMAL', 200.0)
        flood_after_end = detector.get_flood_status()
        assert flood_after_end is not None and flood_after_end.is_active is False

        # During cooldown (200.0 + 5 < 200.0 + 30), send another burst
        results = _send_alarms(detector, 6, start_time=205.0, interval=0.1,
                               id_prefix='cd')
        # The detector should not create a new active flood during cooldown
        flood_during_cd = detector.get_flood_status()
        # The flood object is the same ended one — no new active flood
        assert flood_during_cd.is_active is False

    def test_normal_operation_resumes_after_cooldown(self):
        """After cooldown expires, alarms are forwarded normally."""
        detector = _make_detector(flood_threshold=5, window_sec=60.0, cooldown_sec=10.0)
        # Trigger flood
        _send_alarms(detector, 6, start_time=100.0, interval=0.1)
        # End flood (>60s later so old arrivals fall out)
        detector.record_alarm('end', 0.5, 'X', 200.0)
        # After cooldown (200 + 10 = 210), send a normal alarm
        forwarded = detector.record_alarm('normal_1', 0.5, 'THERMAL', 220.0)
        assert forwarded is True
