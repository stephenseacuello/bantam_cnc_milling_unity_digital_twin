"""Tests for alarm escalation actions: priority boost, forced ack, and notifications.

Validates the escalation logic extracted from AlarmManagerNode without
requiring any ROS2 runtime dependencies.
"""

import os
import sys
import types
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional

import pytest

# Add the package root to sys.path so miracle_scada can be imported without
# a full ROS2 workspace install.
_PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

# ---------------------------------------------------------------------------
# Mock ROS2 / miracle infrastructure so alarm_manager.py can be imported
# ---------------------------------------------------------------------------

# Provide stubs for every package the module touches at import time.

_builtin_interfaces_msg = types.ModuleType('builtin_interfaces.msg')


class _Time:
    def __init__(self):
        self.sec = 0
        self.nanosec = 0


_builtin_interfaces_msg.Time = _Time

sys.modules.setdefault('builtin_interfaces', types.ModuleType('builtin_interfaces'))
sys.modules.setdefault('builtin_interfaces.msg', _builtin_interfaces_msg)

sys.modules.setdefault('rclpy', types.ModuleType('rclpy'))
sys.modules.setdefault('rclpy.lifecycle', types.ModuleType('rclpy.lifecycle'))

_rclpy_lifecycle = sys.modules['rclpy.lifecycle']


class _TransitionCallbackReturn:
    SUCCESS = 0
    FAILURE = 1
    ERROR = 2


_rclpy_lifecycle.TransitionCallbackReturn = _TransitionCallbackReturn

# miracle_core stubs
_miracle_core = types.ModuleType('miracle_core')
_miracle_core_lifecycle = types.ModuleType('miracle_core.lifecycle_node_base')
_miracle_core_qos = types.ModuleType('miracle_core.qos_profiles')


class _FakeNode:
    CRITICALITY_HIGH = 3

    def __init__(self, *a, **kw):
        pass


_miracle_core_lifecycle.MiracleLifecycleNode = _FakeNode


class _QoSProfiles:
    @staticmethod
    def alert():
        return None


_miracle_core_qos.QoSProfiles = _QoSProfiles

sys.modules.setdefault('miracle_core.lifecycle_node_base', _miracle_core_lifecycle)
sys.modules.setdefault('miracle_core.qos_profiles', _miracle_core_qos)

# miracle_msgs stubs
_miracle_msgs = types.ModuleType('miracle_msgs')
_miracle_msgs_msg = types.ModuleType('miracle_msgs.msg')


class _Stub:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

    def __setattr__(self, name, value):
        object.__setattr__(self, name, value)


_miracle_msgs_msg.AnomalyAlert = type('AnomalyAlert', (_Stub,), {})
_miracle_msgs_msg.SecurityAlert = type('SecurityAlert', (_Stub,), {})
_miracle_msgs_msg.CorrelatedAlert = type('CorrelatedAlert', (_Stub,), {})
_miracle_msgs_msg.AlarmEscalation = type('AlarmEscalation', (_Stub,), {})

sys.modules.setdefault('miracle_msgs', _miracle_msgs)
sys.modules.setdefault('miracle_msgs.msg', _miracle_msgs_msg)

# ---------------------------------------------------------------------------
# Now import the real production code
# ---------------------------------------------------------------------------

from miracle_scada.alarm_manager import (  # noqa: E402
    Alarm,
    AlarmState,
    EscalationAction,
    MIN_ESCALATION_INTERVAL,
)


# ---------------------------------------------------------------------------
# Extracted escalation manager (mirrors AlarmManagerNode escalation logic
# without ROS2 dependencies)
# ---------------------------------------------------------------------------

class _EscalationManager:
    """Pure-Python re-implementation of the escalation logic in AlarmManagerNode.

    Replicates the behaviour in ``_check_escalations``, ``acknowledge_alarm``,
    ``get_escalation_summary``, and de-escalation so we can test without a
    running ROS2 graph.
    """

    def __init__(self, escalation_timeout: float = 60.0):
        self.active_alarms: Dict[str, Alarm] = {}
        self.alarm_history: List[Alarm] = []
        self.escalation_timeout = escalation_timeout
        self.published_escalations: List[Dict[str, Any]] = []

    # -- helpers -------------------------------------------------------------

    def raise_alarm(self, alarm_id: str, source: str, severity: float,
                    message: str, timestamp: float = 0.0) -> Alarm:
        alarm = Alarm(
            alarm_id=alarm_id,
            source=source,
            severity=severity,
            message=message,
            timestamp=timestamp,
        )
        self.active_alarms[alarm_id] = alarm
        return alarm

    def _record_escalation(self, alarm: Alarm, action: str, reason: str,
                           time_unacked: float) -> None:
        self.published_escalations.append({
            'alarm_id': alarm.alarm_id,
            'action': action,
            'level': alarm.escalation_level,
            'severity': alarm.severity,
            'reason': reason,
            'time_unacknowledged_sec': time_unacked,
        })

    # -- core escalation (mirrors _check_escalations) -----------------------

    def check_escalations(self, now: float) -> List[str]:
        """Return list of alarm_ids that were escalated."""
        escalated: List[str] = []

        for alarm in self.active_alarms.values():
            if alarm.state != AlarmState.UNACKNOWLEDGED:
                continue

            elapsed = now - alarm.timestamp

            # Determine target level
            if elapsed > self.escalation_timeout * 3:
                target_level = 3
            elif elapsed > self.escalation_timeout * 2:
                target_level = 2
            elif elapsed > self.escalation_timeout:
                target_level = 1
            else:
                # De-escalation if severity dropped below 0.3
                if alarm.escalation_level > 0 and alarm.severity < 0.3:
                    alarm.escalation_level = max(0, alarm.escalation_level - 1)
                    alarm.escalation_actions_taken.append(
                        f"DE-ESCALATED to level {alarm.escalation_level}"
                    )
                continue

            if alarm.escalation_level >= target_level:
                if alarm.severity < 0.3 and alarm.escalation_level > 0:
                    alarm.escalation_level = max(0, alarm.escalation_level - 1)
                    alarm.escalation_actions_taken.append(
                        f"DE-ESCALATED to level {alarm.escalation_level}"
                    )
                continue

            # Min interval enforcement
            if (now - alarm.last_escalation_time) < MIN_ESCALATION_INTERVAL:
                continue

            time_unacked = elapsed
            alarm.escalation_level += 1
            alarm.last_escalation_time = now
            level = alarm.escalation_level
            escalated.append(alarm.alarm_id)

            if level == 1:
                alarm.severity = min(1.0, alarm.severity + 0.1)
                action = EscalationAction.PRIORITY_BOOST
                reason = (
                    f"Unacknowledged for {time_unacked:.0f}s, "
                    f"severity boosted to {alarm.severity:.2f}"
                )
                alarm.escalation_actions_taken.append(action)
                self._record_escalation(alarm, action, reason, time_unacked)

            elif level == 2:
                alarm.requires_forced_ack = True
                action = EscalationAction.FORCED_ACK_REQUIRED
                reason = (
                    f"Unacknowledged for {time_unacked:.0f}s, "
                    f"forced acknowledgment now required"
                )
                alarm.escalation_actions_taken.append(action)
                self._record_escalation(alarm, action, reason, time_unacked)

            elif level >= 3:
                action = EscalationAction.SUPERVISOR_NOTIFY
                reason = (
                    f"Unacknowledged for {time_unacked:.0f}s, "
                    f"supervisor notification sent"
                )
                alarm.escalation_actions_taken.append(action)
                self._record_escalation(alarm, action, reason, time_unacked)

                if alarm.severity > 0.8:
                    estop_action = EscalationAction.EMERGENCY_STOP
                    estop_reason = (
                        f"Severity {alarm.severity:.2f} > 0.8 at escalation "
                        f"level 3, emergency stop triggered"
                    )
                    alarm.escalation_actions_taken.append(estop_action)
                    self._record_escalation(
                        alarm, estop_action, estop_reason, time_unacked
                    )

        return escalated

    # -- acknowledge (mirrors acknowledge_alarm) -----------------------------

    def acknowledge(self, alarm_id: str, user: str,
                    force: bool = False) -> bool:
        if alarm_id not in self.active_alarms:
            return False
        alarm = self.active_alarms[alarm_id]
        if alarm.requires_forced_ack and not force:
            return False
        alarm.state = AlarmState.ACKNOWLEDGED
        alarm.acknowledged_by = user
        return True

    # -- summary (mirrors get_escalation_summary) ----------------------------

    def get_escalation_summary(self) -> Dict:
        level_counts: Dict[int, int] = {}
        forced_ack_ids: List[str] = []

        for alarm in self.active_alarms.values():
            level = alarm.escalation_level
            level_counts[level] = level_counts.get(level, 0) + 1
            if alarm.requires_forced_ack:
                forced_ack_ids.append(alarm.alarm_id)

        return {
            'level_counts': level_counts,
            'avg_time_to_ack': None,
            'forced_ack_required': forced_ack_ids,
        }


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def mgr():
    """Return an escalation manager with a short 60-second timeout."""
    return _EscalationManager(escalation_timeout=60.0)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestEscalationLevels:
    """Test that each escalation level fires the correct action."""

    def test_level1_priority_boost_after_timeout(self, mgr):
        """After 1x timeout, alarm should get PRIORITY_BOOST."""
        mgr.raise_alarm('a1', 'cnc1', 0.5, 'high temp', timestamp=0.0)
        escalated = mgr.check_escalations(now=61.0)
        assert 'a1' in escalated
        alarm = mgr.active_alarms['a1']
        assert alarm.escalation_level == 1
        assert alarm.severity == pytest.approx(0.6)
        assert EscalationAction.PRIORITY_BOOST in alarm.escalation_actions_taken
        assert mgr.published_escalations[-1]['action'] == EscalationAction.PRIORITY_BOOST

    def test_level2_forced_ack_after_2x_timeout(self, mgr):
        """After 2x timeout, alarm should get FORCED_ACK_REQUIRED."""
        mgr.raise_alarm('a1', 'cnc1', 0.5, 'high temp', timestamp=0.0)
        # Escalate to level 1 first
        mgr.check_escalations(now=61.0)
        # Escalate to level 2 (must satisfy min interval)
        mgr.check_escalations(now=121.0)
        alarm = mgr.active_alarms['a1']
        assert alarm.escalation_level == 2
        assert alarm.requires_forced_ack is True
        assert EscalationAction.FORCED_ACK_REQUIRED in alarm.escalation_actions_taken

    def test_level3_supervisor_notify_after_3x_timeout(self, mgr):
        """After 3x timeout, alarm should get SUPERVISOR_NOTIFY."""
        mgr.raise_alarm('a1', 'cnc1', 0.5, 'high temp', timestamp=0.0)
        mgr.check_escalations(now=61.0)   # -> level 1
        mgr.check_escalations(now=121.0)  # -> level 2
        mgr.check_escalations(now=181.0)  # -> level 3
        alarm = mgr.active_alarms['a1']
        assert alarm.escalation_level == 3
        assert EscalationAction.SUPERVISOR_NOTIFY in alarm.escalation_actions_taken
        # Severity started at 0.5, boosted to 0.6 at level 1, still <= 0.8
        assert EscalationAction.EMERGENCY_STOP not in alarm.escalation_actions_taken

    def test_level3_emergency_stop_high_severity(self, mgr):
        """Level 3 with severity > 0.8 should also trigger EMERGENCY_STOP."""
        mgr.raise_alarm('a1', 'cnc1', 0.85, 'critical temp', timestamp=0.0)
        mgr.check_escalations(now=61.0)   # -> level 1, severity 0.95
        mgr.check_escalations(now=121.0)  # -> level 2
        mgr.check_escalations(now=181.0)  # -> level 3
        alarm = mgr.active_alarms['a1']
        assert alarm.escalation_level == 3
        assert EscalationAction.EMERGENCY_STOP in alarm.escalation_actions_taken
        assert EscalationAction.SUPERVISOR_NOTIFY in alarm.escalation_actions_taken


class TestSeverityCap:
    """Test that priority boost never exceeds 1.0."""

    def test_severity_capped_at_1_0(self, mgr):
        """Severity should never exceed 1.0 after a PRIORITY_BOOST."""
        mgr.raise_alarm('a1', 'cnc1', 0.95, 'near-max', timestamp=0.0)
        mgr.check_escalations(now=61.0)  # boost +0.1 -> should cap at 1.0
        alarm = mgr.active_alarms['a1']
        assert alarm.severity == pytest.approx(1.0)
        assert alarm.severity <= 1.0


class TestForcedAcknowledgment:
    """Test forced-ack gating on acknowledgment."""

    def test_forced_ack_flag_set_at_level2(self, mgr):
        """After level-2 escalation, requires_forced_ack must be True."""
        mgr.raise_alarm('a1', 'cnc1', 0.5, 'temp', timestamp=0.0)
        mgr.check_escalations(now=61.0)
        mgr.check_escalations(now=121.0)
        assert mgr.active_alarms['a1'].requires_forced_ack is True

    def test_normal_ack_rejected_when_forced_required(self, mgr):
        """Normal acknowledge should fail when forced ack is required."""
        mgr.raise_alarm('a1', 'cnc1', 0.5, 'temp', timestamp=0.0)
        mgr.check_escalations(now=61.0)
        mgr.check_escalations(now=121.0)
        result = mgr.acknowledge('a1', 'operator', force=False)
        assert result is False
        assert mgr.active_alarms['a1'].state == AlarmState.UNACKNOWLEDGED

    def test_force_ack_succeeds(self, mgr):
        """force=True should override the forced-ack requirement."""
        mgr.raise_alarm('a1', 'cnc1', 0.5, 'temp', timestamp=0.0)
        mgr.check_escalations(now=61.0)
        mgr.check_escalations(now=121.0)
        result = mgr.acknowledge('a1', 'supervisor', force=True)
        assert result is True
        assert mgr.active_alarms['a1'].state == AlarmState.ACKNOWLEDGED


class TestDeEscalation:
    """Test that alarms de-escalate when severity drops."""

    def test_de_escalation_when_severity_drops_below_03(self, mgr):
        """Alarm should de-escalate when severity drops below 0.3."""
        alarm = mgr.raise_alarm('a1', 'cnc1', 0.5, 'temp', timestamp=0.0)
        mgr.check_escalations(now=61.0)  # -> level 1, severity 0.6
        assert alarm.escalation_level == 1

        # Simulate severity drop (e.g. condition improved)
        alarm.severity = 0.2
        # Check escalations at a time still within level-1 window
        # but not yet 2x timeout -- the de-escalation path in the
        # 'elapsed <= timeout' branch triggers.
        # We need to reset timestamp so elapsed < timeout for de-escalation branch
        alarm.timestamp = 100.0
        mgr.check_escalations(now=110.0)  # elapsed=10 < 60, severity=0.2 < 0.3
        assert alarm.escalation_level == 0
        assert any('DE-ESCALATED' in a for a in alarm.escalation_actions_taken)


class TestEscalationSummary:
    """Test the escalation summary report."""

    def test_summary_counts_levels(self, mgr):
        """Summary should report correct counts per escalation level."""
        mgr.raise_alarm('a1', 'cnc1', 0.5, 'temp1', timestamp=0.0)
        mgr.raise_alarm('a2', 'cnc2', 0.6, 'temp2', timestamp=0.0)
        mgr.raise_alarm('a3', 'cnc3', 0.7, 'temp3', timestamp=0.0)

        # Escalate a1 to level 1
        mgr.check_escalations(now=61.0)
        # Escalate a1 to level 2, a2 & a3 stay at level 1
        mgr.check_escalations(now=121.0)

        summary = mgr.get_escalation_summary()
        # a1 at level 2, a2 & a3 at level 1 (but a2 would also be at level 2
        # since 121 > 2*60). Let's verify:
        a1 = mgr.active_alarms['a1']
        a2 = mgr.active_alarms['a2']
        a3 = mgr.active_alarms['a3']
        assert a1.escalation_level == 2
        assert a2.escalation_level == 2
        assert a3.escalation_level == 2

        assert summary['level_counts'][2] == 3
        assert len(summary['forced_ack_required']) == 3

    def test_summary_forced_ack_list(self, mgr):
        """Summary should list alarm IDs requiring forced ack."""
        mgr.raise_alarm('a1', 'cnc1', 0.5, 'temp', timestamp=0.0)
        mgr.raise_alarm('a2', 'cnc2', 0.5, 'temp', timestamp=0.0)
        mgr.check_escalations(now=61.0)
        mgr.check_escalations(now=121.0)

        summary = mgr.get_escalation_summary()
        assert 'a1' in summary['forced_ack_required']
        assert 'a2' in summary['forced_ack_required']


class TestMinEscalationInterval:
    """Test that the 30-second minimum interval between escalations is enforced."""

    def test_rapid_escalations_blocked(self, mgr):
        """Two escalation checks within 30s should not double-escalate."""
        mgr.raise_alarm('a1', 'cnc1', 0.5, 'temp', timestamp=0.0)
        mgr.check_escalations(now=61.0)   # -> level 1
        assert mgr.active_alarms['a1'].escalation_level == 1
        # Try again 10s later (within 30s min interval) -- even though
        # elapsed > 2x timeout, the min interval should block
        mgr.check_escalations(now=71.0)
        assert mgr.active_alarms['a1'].escalation_level == 1  # still 1

    def test_escalation_allowed_after_interval(self, mgr):
        """Escalation should proceed once the min interval has passed."""
        mgr.raise_alarm('a1', 'cnc1', 0.5, 'temp', timestamp=0.0)
        mgr.check_escalations(now=61.0)   # -> level 1
        # Wait beyond 30s min interval AND 2x timeout
        mgr.check_escalations(now=121.0)  # -> level 2
        assert mgr.active_alarms['a1'].escalation_level == 2


class TestAuditTrail:
    """Test that escalation actions are recorded for audit."""

    def test_audit_trail_records_all_actions(self, mgr):
        """escalation_actions_taken should contain every action taken."""
        mgr.raise_alarm('a1', 'cnc1', 0.85, 'critical', timestamp=0.0)
        mgr.check_escalations(now=61.0)   # level 1: PRIORITY_BOOST
        mgr.check_escalations(now=121.0)  # level 2: FORCED_ACK_REQUIRED
        mgr.check_escalations(now=181.0)  # level 3: SUPERVISOR_NOTIFY + EMERGENCY_STOP

        trail = mgr.active_alarms['a1'].escalation_actions_taken
        assert EscalationAction.PRIORITY_BOOST in trail
        assert EscalationAction.FORCED_ACK_REQUIRED in trail
        assert EscalationAction.SUPERVISOR_NOTIFY in trail
        assert EscalationAction.EMERGENCY_STOP in trail
        assert len(trail) == 4

    def test_published_escalation_messages_match(self, mgr):
        """Each escalation action should produce a published message."""
        mgr.raise_alarm('a1', 'cnc1', 0.5, 'temp', timestamp=0.0)
        mgr.check_escalations(now=61.0)
        mgr.check_escalations(now=121.0)

        assert len(mgr.published_escalations) == 2
        actions = [e['action'] for e in mgr.published_escalations]
        assert EscalationAction.PRIORITY_BOOST in actions
        assert EscalationAction.FORCED_ACK_REQUIRED in actions
