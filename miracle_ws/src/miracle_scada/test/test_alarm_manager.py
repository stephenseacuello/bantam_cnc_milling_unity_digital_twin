"""Tests for ISA-18.2 alarm management logic.

Extracts the core alarm state machine, shelving, priority calculation,
flood detection, and deadband (hysteresis) logic from AlarmManagerNode
and tests them without any ROS2 dependency.
"""

import time
import pytest
from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, List, Optional


# ---------------------------------------------------------------------------
# Extracted helpers (mirror/extend the logic in alarm_manager.py)
# ---------------------------------------------------------------------------

class AlarmState(Enum):
    """ISA-18.2 alarm states (matches alarm_manager.py)."""
    NORMAL = 'NORMAL'
    UNACKNOWLEDGED = 'UNACKNOWLEDGED'
    ACKNOWLEDGED = 'ACKNOWLEDGED'
    SHELVED = 'SHELVED'
    SUPPRESSED = 'SUPPRESSED'


@dataclass
class _Alarm:
    """Alarm record (matches the Alarm dataclass in alarm_manager.py)."""
    alarm_id: str
    source: str
    severity: float
    message: str
    state: AlarmState = AlarmState.UNACKNOWLEDGED
    timestamp: float = 0.0
    acknowledged_by: Optional[str] = None
    escalation_level: int = 0
    shelved_until: Optional[float] = None


class _AlarmManager:
    """Extracted alarm management logic from AlarmManagerNode.

    Covers ISA-18.2 state transitions, shelving, escalation,
    alarm flood detection, and deadband.
    """

    def __init__(self, max_active=1000, escalation_timeout=300.0, history_size=10000):
        self.active_alarms: Dict[str, _Alarm] = {}
        self.alarm_history: List[_Alarm] = []
        self.max_active = max_active
        self.escalation_timeout = escalation_timeout
        self.history_size = history_size

    def raise_alarm(self, alarm_id, source, severity, message, timestamp=0.0):
        """Raise a new alarm (mirrors _on_anomaly)."""
        if len(self.active_alarms) >= self.max_active:
            return False
        alarm = _Alarm(
            alarm_id=alarm_id,
            source=source,
            severity=severity,
            message=message,
            timestamp=timestamp,
        )
        self.active_alarms[alarm_id] = alarm
        return True

    def acknowledge(self, alarm_id, user):
        """Acknowledge an alarm (mirrors acknowledge_alarm)."""
        if alarm_id in self.active_alarms:
            alarm = self.active_alarms[alarm_id]
            if alarm.state == AlarmState.SHELVED:
                return False  # Cannot ack a shelved alarm
            alarm.state = AlarmState.ACKNOWLEDGED
            alarm.acknowledged_by = user
            return True
        return False

    def clear(self, alarm_id):
        """Clear/resolve an alarm (mirrors clear_alarm)."""
        if alarm_id in self.active_alarms:
            alarm = self.active_alarms.pop(alarm_id)
            alarm.state = AlarmState.NORMAL
            self.alarm_history.append(alarm)
            if len(self.alarm_history) > self.history_size:
                self.alarm_history = self.alarm_history[-self.history_size:]
            return True
        return False

    def shelve(self, alarm_id, duration_sec, current_time=0.0):
        """Shelve (temporarily suppress) an alarm.

        Args:
            alarm_id: The alarm to shelve.
            duration_sec: How long to shelve (seconds).
            current_time: Current epoch time.

        Returns:
            True if shelved successfully.
        """
        if alarm_id in self.active_alarms:
            alarm = self.active_alarms[alarm_id]
            alarm.state = AlarmState.SHELVED
            alarm.shelved_until = current_time + duration_sec
            return True
        return False

    def unshelve_expired(self, current_time):
        """Unshelve alarms whose shelve duration has expired."""
        for alarm in self.active_alarms.values():
            if (alarm.state == AlarmState.SHELVED
                    and alarm.shelved_until is not None
                    and current_time >= alarm.shelved_until):
                alarm.state = AlarmState.UNACKNOWLEDGED
                alarm.shelved_until = None

    def check_escalations(self, current_time):
        """Check for alarms needing escalation (mirrors _check_escalations)."""
        escalated = []
        for alarm in self.active_alarms.values():
            if alarm.state == AlarmState.UNACKNOWLEDGED:
                elapsed = current_time - alarm.timestamp
                if elapsed > self.escalation_timeout:
                    alarm.escalation_level += 1
                    escalated.append(alarm.alarm_id)
        return escalated

    def active_count(self):
        """Return the number of active alarms."""
        return len(self.active_alarms)


class _AlarmPriorityCalculator:
    """Calculates alarm priority from severity and context.

    Priority levels (ISA-18.2 inspired):
      4 = CRITICAL (severity >= 0.9)
      3 = HIGH     (severity >= 0.7)
      2 = MEDIUM   (severity >= 0.4)
      1 = LOW      (severity < 0.4)
    """

    @staticmethod
    def priority_level(severity):
        """Map a 0-1 severity to an integer priority level."""
        if severity >= 0.9:
            return 4
        if severity >= 0.7:
            return 3
        if severity >= 0.4:
            return 2
        return 1

    @staticmethod
    def priority_label(level):
        """Convert a priority level to a human-readable label."""
        labels = {4: 'CRITICAL', 3: 'HIGH', 2: 'MEDIUM', 1: 'LOW'}
        return labels.get(level, 'UNKNOWN')


class _AlarmFloodDetector:
    """Detects alarm floods -- too many alarms in a short period.

    An alarm flood is typically defined as > N alarms within T seconds.
    """

    def __init__(self, threshold_count=10, window_sec=60.0):
        self.threshold_count = threshold_count
        self.window_sec = window_sec
        self._timestamps: List[float] = []

    def record_alarm(self, timestamp):
        """Record that an alarm was raised at the given time."""
        self._timestamps.append(timestamp)

    def is_flooding(self, current_time):
        """Check whether the alarm rate exceeds the flood threshold."""
        cutoff = current_time - self.window_sec
        recent = [t for t in self._timestamps if t >= cutoff]
        self._timestamps = recent  # prune old entries
        return len(recent) >= self.threshold_count

    def recent_count(self, current_time):
        """Count alarms within the current window."""
        cutoff = current_time - self.window_sec
        return sum(1 for t in self._timestamps if t >= cutoff)


class _AlarmDeadband:
    """Implements alarm deadband (hysteresis) to prevent chattering.

    An alarm triggers when value >= setpoint.
    It clears only when value <= setpoint - deadband.
    """

    def __init__(self, setpoint, deadband):
        self.setpoint = setpoint
        self.deadband = deadband
        self.active = False

    def evaluate(self, value):
        """Evaluate the current value against the deadband.

        Returns:
            True if alarm should be active.
        """
        if not self.active:
            if value >= self.setpoint:
                self.active = True
        else:
            if value <= self.setpoint - self.deadband:
                self.active = False
        return self.active


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def manager():
    """Return a fresh alarm manager with default settings."""
    return _AlarmManager(max_active=100, escalation_timeout=300.0, history_size=50)


@pytest.fixture
def priority():
    """Return an alarm priority calculator."""
    return _AlarmPriorityCalculator()


@pytest.fixture
def flood_detector():
    """Return an alarm flood detector with low thresholds for testing."""
    return _AlarmFloodDetector(threshold_count=5, window_sec=10.0)


@pytest.fixture
def deadband():
    """Return a deadband with setpoint=100 and deadband=5."""
    return _AlarmDeadband(setpoint=100.0, deadband=5.0)


# ---------------------------------------------------------------------------
# Test classes
# ---------------------------------------------------------------------------

class TestAlarmStateTransitions:
    """Test ISA-18.2 alarm state transitions."""

    def test_new_alarm_is_unacknowledged(self, manager):
        """A newly raised alarm should be in UNACKNOWLEDGED state."""
        manager.raise_alarm('a1', 'cnc1', 0.8, 'high temp')
        assert manager.active_alarms['a1'].state == AlarmState.UNACKNOWLEDGED

    def test_acknowledge_transitions_to_acknowledged(self, manager):
        """Acknowledging should move alarm to ACKNOWLEDGED state."""
        manager.raise_alarm('a1', 'cnc1', 0.8, 'high temp')
        result = manager.acknowledge('a1', 'operator1')
        assert result is True
        assert manager.active_alarms['a1'].state == AlarmState.ACKNOWLEDGED
        assert manager.active_alarms['a1'].acknowledged_by == 'operator1'

    def test_clear_transitions_to_normal_and_removes(self, manager):
        """Clearing should move alarm to history with NORMAL state."""
        manager.raise_alarm('a1', 'cnc1', 0.8, 'high temp')
        manager.acknowledge('a1', 'operator1')
        result = manager.clear('a1')
        assert result is True
        assert 'a1' not in manager.active_alarms
        assert len(manager.alarm_history) == 1
        assert manager.alarm_history[0].state == AlarmState.NORMAL

    def test_full_lifecycle(self, manager):
        """NORMAL -> UNACK -> ACK -> NORMAL (cleared)."""
        manager.raise_alarm('a1', 'cnc1', 0.5, 'minor issue')
        assert manager.active_alarms['a1'].state == AlarmState.UNACKNOWLEDGED

        manager.acknowledge('a1', 'user')
        assert manager.active_alarms['a1'].state == AlarmState.ACKNOWLEDGED

        manager.clear('a1')
        assert 'a1' not in manager.active_alarms
        assert manager.alarm_history[-1].state == AlarmState.NORMAL

    def test_acknowledge_nonexistent_alarm(self, manager):
        """Acknowledging a non-existent alarm should return False."""
        assert manager.acknowledge('missing', 'user') is False

    def test_clear_nonexistent_alarm(self, manager):
        """Clearing a non-existent alarm should return False."""
        assert manager.clear('missing') is False

    def test_max_active_alarms_enforced(self, manager):
        """Should not allow more alarms than max_active."""
        mgr = _AlarmManager(max_active=3)
        mgr.raise_alarm('a1', 's', 0.5, 'm')
        mgr.raise_alarm('a2', 's', 0.5, 'm')
        mgr.raise_alarm('a3', 's', 0.5, 'm')
        result = mgr.raise_alarm('a4', 's', 0.5, 'm')
        assert result is False
        assert mgr.active_count() == 3

    def test_history_size_trimmed(self):
        """History should be trimmed to history_size."""
        mgr = _AlarmManager(max_active=1000, history_size=3)
        for i in range(5):
            aid = f'a{i}'
            mgr.raise_alarm(aid, 's', 0.5, 'm')
            mgr.clear(aid)
        assert len(mgr.alarm_history) == 3


class TestAlarmShelving:
    """Test alarm shelving (temporary suppression)."""

    def test_shelve_sets_state(self, manager):
        """Shelving should set state to SHELVED."""
        manager.raise_alarm('a1', 'cnc1', 0.7, 'vibration')
        result = manager.shelve('a1', duration_sec=60.0, current_time=1000.0)
        assert result is True
        assert manager.active_alarms['a1'].state == AlarmState.SHELVED
        assert manager.active_alarms['a1'].shelved_until == 1060.0

    def test_shelved_alarm_cannot_be_acknowledged(self, manager):
        """A shelved alarm should not be acknowledgeable."""
        manager.raise_alarm('a1', 'cnc1', 0.7, 'vibration')
        manager.shelve('a1', duration_sec=60.0, current_time=1000.0)
        result = manager.acknowledge('a1', 'user')
        assert result is False
        assert manager.active_alarms['a1'].state == AlarmState.SHELVED

    def test_unshelve_after_expiry(self, manager):
        """An expired shelved alarm should return to UNACKNOWLEDGED."""
        manager.raise_alarm('a1', 'cnc1', 0.7, 'vibration')
        manager.shelve('a1', duration_sec=60.0, current_time=1000.0)
        manager.unshelve_expired(current_time=1061.0)
        assert manager.active_alarms['a1'].state == AlarmState.UNACKNOWLEDGED

    def test_shelve_not_expired_yet(self, manager):
        """A shelved alarm should remain SHELVED before expiry."""
        manager.raise_alarm('a1', 'cnc1', 0.7, 'vibration')
        manager.shelve('a1', duration_sec=60.0, current_time=1000.0)
        manager.unshelve_expired(current_time=1050.0)
        assert manager.active_alarms['a1'].state == AlarmState.SHELVED

    def test_shelve_nonexistent_alarm(self, manager):
        """Shelving a non-existent alarm should return False."""
        assert manager.shelve('missing', 30.0) is False


class TestAlarmPriorityCalculation:
    """Test alarm priority level calculation."""

    def test_critical_priority(self, priority):
        """Severity >= 0.9 should map to CRITICAL (level 4)."""
        assert priority.priority_level(0.95) == 4
        assert priority.priority_level(1.0) == 4
        assert priority.priority_level(0.9) == 4

    def test_high_priority(self, priority):
        """Severity in [0.7, 0.9) should map to HIGH (level 3)."""
        assert priority.priority_level(0.7) == 3
        assert priority.priority_level(0.85) == 3

    def test_medium_priority(self, priority):
        """Severity in [0.4, 0.7) should map to MEDIUM (level 2)."""
        assert priority.priority_level(0.4) == 2
        assert priority.priority_level(0.65) == 2

    def test_low_priority(self, priority):
        """Severity < 0.4 should map to LOW (level 1)."""
        assert priority.priority_level(0.3) == 1
        assert priority.priority_level(0.0) == 1

    def test_boundary_at_0_9(self, priority):
        """Boundary: 0.9 is CRITICAL, 0.89 is HIGH."""
        assert priority.priority_level(0.9) == 4
        assert priority.priority_level(0.89) == 3

    def test_priority_labels(self, priority):
        """Priority labels should match expected strings."""
        assert priority.priority_label(4) == 'CRITICAL'
        assert priority.priority_label(3) == 'HIGH'
        assert priority.priority_label(2) == 'MEDIUM'
        assert priority.priority_label(1) == 'LOW'
        assert priority.priority_label(0) == 'UNKNOWN'


class TestAlarmFloodDetection:
    """Test alarm flood detection (too many alarms in short period)."""

    def test_no_flood_below_threshold(self, flood_detector):
        """Below threshold count should not be considered flooding."""
        for i in range(4):
            flood_detector.record_alarm(float(i))
        assert flood_detector.is_flooding(5.0) is False

    def test_flood_at_threshold(self, flood_detector):
        """Exactly at threshold should be considered flooding."""
        for i in range(5):
            flood_detector.record_alarm(float(i))
        assert flood_detector.is_flooding(5.0) is True

    def test_flood_above_threshold(self, flood_detector):
        """Well above threshold should be flooding."""
        for i in range(10):
            flood_detector.record_alarm(float(i))
        assert flood_detector.is_flooding(5.0) is True

    def test_old_alarms_expire_outside_window(self, flood_detector):
        """Alarms outside the time window should not count."""
        # Record 5 alarms at t=0..4, then check at t=100 (window=10s)
        for i in range(5):
            flood_detector.record_alarm(float(i))
        assert flood_detector.is_flooding(100.0) is False

    def test_recent_count_accuracy(self, flood_detector):
        """recent_count should return only alarms within the window."""
        flood_detector.record_alarm(0.0)
        flood_detector.record_alarm(5.0)
        flood_detector.record_alarm(10.0)
        flood_detector.record_alarm(15.0)
        # Window is 10s, checking at t=18 -> only t=10 and t=15 are in window
        assert flood_detector.recent_count(18.0) == 2

    def test_empty_detector_not_flooding(self, flood_detector):
        """An empty detector should not report flooding."""
        assert flood_detector.is_flooding(0.0) is False


class TestAlarmDeadband:
    """Test alarm deadband (hysteresis to prevent chattering)."""

    def test_alarm_activates_at_setpoint(self, deadband):
        """Alarm should activate when value reaches setpoint."""
        assert deadband.evaluate(100.0) is True
        assert deadband.active is True

    def test_alarm_stays_active_above_setpoint(self, deadband):
        """Alarm should remain active while value is above setpoint."""
        deadband.evaluate(105.0)
        assert deadband.evaluate(101.0) is True

    def test_alarm_stays_active_in_deadband_zone(self, deadband):
        """Value between (setpoint - deadband) and setpoint should stay active."""
        deadband.evaluate(100.0)  # activate
        assert deadband.evaluate(96.0) is True  # 96 > 100-5=95

    def test_alarm_clears_at_or_below_deadband(self, deadband):
        """Alarm should clear when value reaches setpoint - deadband."""
        deadband.evaluate(100.0)  # activate
        # At exactly setpoint - deadband (95.0), the condition
        # value <= setpoint - deadband is True, so alarm clears.
        assert deadband.evaluate(95.0) is False
        # Well below should also remain cleared
        assert deadband.evaluate(90.0) is False

    def test_no_chattering(self, deadband):
        """Value oscillating near setpoint should not cause rapid on/off."""
        deadband.evaluate(100.0)  # activate
        results = []
        for v in [99.0, 98.0, 97.0, 96.5, 96.0, 98.0, 100.0, 96.0]:
            results.append(deadband.evaluate(v))
        # Should stay active throughout since no value drops below 95
        assert all(results), "Deadband should prevent chattering near setpoint"

    def test_below_setpoint_stays_inactive(self, deadband):
        """Values below setpoint should not activate the alarm."""
        assert deadband.evaluate(99.9) is False
        assert deadband.evaluate(50.0) is False

    def test_reactivation_after_clear(self, deadband):
        """After clearing, alarm should re-activate at setpoint."""
        deadband.evaluate(100.0)  # activate
        deadband.evaluate(94.0)   # clear (below 95)
        assert deadband.active is False
        deadband.evaluate(100.0)  # re-activate
        assert deadband.active is True


class TestEscalation:
    """Test alarm escalation logic."""

    def test_no_escalation_before_timeout(self, manager):
        """Alarm should not escalate before timeout elapses."""
        manager.raise_alarm('a1', 'cnc1', 0.8, 'temp', timestamp=1000.0)
        escalated = manager.check_escalations(current_time=1100.0)
        assert escalated == []
        assert manager.active_alarms['a1'].escalation_level == 0

    def test_escalation_after_timeout(self, manager):
        """Alarm should escalate after timeout elapses."""
        manager.raise_alarm('a1', 'cnc1', 0.8, 'temp', timestamp=1000.0)
        escalated = manager.check_escalations(current_time=1301.0)
        assert 'a1' in escalated
        assert manager.active_alarms['a1'].escalation_level == 1

    def test_multiple_escalations(self, manager):
        """Each check after timeout should increment escalation level."""
        manager.raise_alarm('a1', 'cnc1', 0.8, 'temp', timestamp=0.0)
        manager.check_escalations(current_time=301.0)
        manager.check_escalations(current_time=302.0)
        assert manager.active_alarms['a1'].escalation_level == 2

    def test_acknowledged_alarm_not_escalated(self, manager):
        """Acknowledged alarms should not be escalated."""
        manager.raise_alarm('a1', 'cnc1', 0.8, 'temp', timestamp=0.0)
        manager.acknowledge('a1', 'user')
        escalated = manager.check_escalations(current_time=500.0)
        assert escalated == []
        assert manager.active_alarms['a1'].escalation_level == 0
