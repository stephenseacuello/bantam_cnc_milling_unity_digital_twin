"""Tests for the configurable EscalationEngine.

Validates escalation policies, duplicate suppression, time-based escalation,
acknowledgment, auto-acknowledge, and history tracking without ROS2 runtime.
"""

import os
import sys
import types
from typing import Any

import pytest

# ---------------------------------------------------------------------------
# Add package root so miracle_scada can be imported without colcon install
# ---------------------------------------------------------------------------
_PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

# ---------------------------------------------------------------------------
# Mock ROS2 / miracle infrastructure
# ---------------------------------------------------------------------------

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

sys.modules.setdefault('miracle_core', _miracle_core)
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
# Now import production code
# ---------------------------------------------------------------------------

from miracle_scada.alarm_manager import (  # noqa: E402
    Alarm,
    AlarmState,
    EscalationEngine,
    EscalationActionResult,
    EscalationLevel,
    EscalationPolicy,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_alarm(alarm_id='alarm-1', source='cnc1', severity=0.5,
                message='test alarm', alarm_type='SAFETY',
                timestamp=1000.0) -> Alarm:
    """Create a test alarm."""
    return Alarm(
        alarm_id=alarm_id,
        source=source,
        severity=severity,
        message=message,
        alarm_type=alarm_type,
        timestamp=timestamp,
    )


# ---------------------------------------------------------------------------
# Tests: Default policies
# ---------------------------------------------------------------------------

class TestDefaultPolicies:
    """Verify that the engine ships with sensible default policies."""

    def test_engine_has_default_policies(self):
        engine = EscalationEngine()
        assert len(engine._policies) >= 6  # 5 types + fallback

    def test_safety_policy_exists(self):
        engine = EscalationEngine()
        assert 'default_safety' in engine._policies

    def test_quality_policy_exists(self):
        engine = EscalationEngine()
        assert 'default_quality' in engine._policies

    def test_tool_policy_exists(self):
        engine = EscalationEngine()
        assert 'default_tool' in engine._policies

    def test_thermal_policy_exists(self):
        engine = EscalationEngine()
        assert 'default_thermal' in engine._policies

    def test_communication_policy_exists(self):
        engine = EscalationEngine()
        assert 'default_communication' in engine._policies

    def test_fallback_policy_exists(self):
        engine = EscalationEngine()
        assert 'default_fallback' in engine._policies

    def test_safety_policy_has_four_levels(self):
        engine = EscalationEngine()
        policy = engine._policies['default_safety']
        assert len(policy.levels) == 4

    def test_communication_policy_auto_ack(self):
        engine = EscalationEngine()
        policy = engine._policies['default_communication']
        assert policy.auto_acknowledge_after_sec == 600.0


# ---------------------------------------------------------------------------
# Tests: Custom policy registration
# ---------------------------------------------------------------------------

class TestPolicyRegistration:
    """Verify custom policies can be registered and override defaults."""

    def test_register_custom_policy(self):
        engine = EscalationEngine()
        custom = EscalationPolicy(
            policy_id='custom_vibration',
            name='Vibration Policy',
            alarm_types=['VIBRATION'],
            levels=[EscalationLevel(1, 0.0, ['operator'], 'notify', 'vib {alarm_id}')],
        )
        engine.register_policy(custom)
        assert 'custom_vibration' in engine._policies

    def test_register_replaces_existing(self):
        engine = EscalationEngine()
        original_name = engine._policies['default_safety'].name
        replacement = EscalationPolicy(
            policy_id='default_safety',
            name='Replaced Safety',
            alarm_types=['SAFETY'],
            levels=[],
        )
        engine.register_policy(replacement)
        assert engine._policies['default_safety'].name == 'Replaced Safety'

    def test_custom_policy_matched_on_process(self):
        engine = EscalationEngine()
        custom = EscalationPolicy(
            policy_id='custom_pressure',
            name='Pressure Policy',
            alarm_types=['PRESSURE'],
            levels=[EscalationLevel(1, 0.0, ['operator'], 'page', 'pressure {alarm_id}')],
        )
        engine.register_policy(custom)
        alarm = _make_alarm(alarm_type='PRESSURE')
        result = engine.process_alarm(alarm)
        assert result.action_type == EscalationEngine.PAGE


# ---------------------------------------------------------------------------
# Tests: Alarm processing -> correct initial action
# ---------------------------------------------------------------------------

class TestAlarmProcessing:
    """Verify process_alarm returns correct initial actions."""

    def test_safety_alarm_initial_notify(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_type='SAFETY')
        result = engine.process_alarm(alarm)
        assert result.action_type == EscalationEngine.NOTIFY
        assert result.level == 1
        assert 'operator' in result.notify_roles

    def test_alarm_id_in_result(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='xyz-123', alarm_type='SAFETY')
        result = engine.process_alarm(alarm)
        assert result.alarm_id == 'xyz-123'

    def test_message_rendered(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='abc', alarm_type='SAFETY', message='spindle fault')
        result = engine.process_alarm(alarm)
        assert 'abc' in result.message
        assert 'spindle fault' in result.message

    def test_alarm_tracked_as_active(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY')
        engine.process_alarm(alarm)
        assert 'a1' in engine._active


# ---------------------------------------------------------------------------
# Tests: Duplicate suppression
# ---------------------------------------------------------------------------

class TestDuplicateSuppression:
    """Verify duplicate alarms within suppress window are suppressed."""

    def test_duplicate_suppressed_within_window(self):
        engine = EscalationEngine()
        a1 = _make_alarm(alarm_id='a1', alarm_type='SAFETY', source='cnc1', timestamp=100.0)
        a2 = _make_alarm(alarm_id='a2', alarm_type='SAFETY', source='cnc1', timestamp=110.0)
        engine.process_alarm(a1)
        result = engine.process_alarm(a2)
        assert result.action_type == EscalationEngine.SUPPRESS

    def test_not_suppressed_after_window(self):
        engine = EscalationEngine()
        # Safety suppress window is 30s
        a1 = _make_alarm(alarm_id='a1', alarm_type='SAFETY', source='cnc1', timestamp=100.0)
        a2 = _make_alarm(alarm_id='a2', alarm_type='SAFETY', source='cnc1', timestamp=200.0)
        engine.process_alarm(a1)
        result = engine.process_alarm(a2)
        assert result.action_type != EscalationEngine.SUPPRESS

    def test_different_source_not_suppressed(self):
        engine = EscalationEngine()
        a1 = _make_alarm(alarm_id='a1', alarm_type='SAFETY', source='cnc1', timestamp=100.0)
        a2 = _make_alarm(alarm_id='a2', alarm_type='SAFETY', source='cnc2', timestamp=105.0)
        engine.process_alarm(a1)
        result = engine.process_alarm(a2)
        assert result.action_type != EscalationEngine.SUPPRESS

    def test_different_type_not_suppressed(self):
        engine = EscalationEngine()
        a1 = _make_alarm(alarm_id='a1', alarm_type='SAFETY', source='cnc1', timestamp=100.0)
        a2 = _make_alarm(alarm_id='a2', alarm_type='TOOL', source='cnc1', timestamp=105.0)
        engine.process_alarm(a1)
        result = engine.process_alarm(a2)
        assert result.action_type != EscalationEngine.SUPPRESS


# ---------------------------------------------------------------------------
# Tests: Time-based escalation (level 1 -> 2 -> 3)
# ---------------------------------------------------------------------------

class TestTimeBasedEscalation:
    """Verify alarms escalate through levels based on elapsed time."""

    def test_no_escalation_before_delay(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        actions = engine.update_escalation_timers(1030.0)  # 30s < 60s level 2
        assert len(actions) == 0

    def test_escalate_to_level_2(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        actions = engine.update_escalation_timers(1061.0)  # >60s
        assert len(actions) == 1
        assert actions[0].level == 2
        assert actions[0].alarm_id == 'a1'
        assert actions[0].action_type == EscalationEngine.PAGE

    def test_escalate_to_level_3(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        # First escalation to level 2
        engine.update_escalation_timers(1061.0)
        # Then to level 3 (delay_sec=180)
        actions = engine.update_escalation_timers(1181.0)
        assert len(actions) == 1
        assert actions[0].level == 3
        assert actions[0].action_type == EscalationEngine.AUTO_STOP

    def test_escalate_to_level_4_emergency(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        engine.update_escalation_timers(1061.0)  # -> level 2
        engine.update_escalation_timers(1181.0)  # -> level 3
        actions = engine.update_escalation_timers(1301.0)  # -> level 4 (300s)
        assert len(actions) == 1
        assert actions[0].level == 4
        assert actions[0].action_type == EscalationEngine.LOCKOUT

    def test_no_escalation_past_max_level(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        engine.update_escalation_timers(1061.0)  # 2
        engine.update_escalation_timers(1181.0)  # 3
        engine.update_escalation_timers(1301.0)  # 4
        actions = engine.update_escalation_timers(1500.0)  # nothing more
        assert len(actions) == 0

    def test_multiple_levels_in_one_update(self):
        """If enough time passes, multiple levels should fire at once."""
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        # Jump to 200s -- should trigger level 2 (60s) and level 3 (180s)
        actions = engine.update_escalation_timers(1200.0)
        levels = [a.level for a in actions]
        assert 2 in levels
        assert 3 in levels


# ---------------------------------------------------------------------------
# Tests: Acknowledgment stops escalation
# ---------------------------------------------------------------------------

class TestAcknowledgment:
    """Verify acknowledge_alarm stops further escalation."""

    def test_acknowledge_returns_true(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY')
        engine.process_alarm(alarm)
        assert engine.acknowledge_alarm('a1', 'op1') is True

    def test_acknowledge_stops_escalation(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        engine.acknowledge_alarm('a1', 'op1')
        actions = engine.update_escalation_timers(2000.0)
        assert len(actions) == 0

    def test_acknowledge_unknown_alarm_returns_false(self):
        engine = EscalationEngine()
        assert engine.acknowledge_alarm('nonexistent', 'op1') is False

    def test_double_acknowledge_returns_false(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY')
        engine.process_alarm(alarm)
        engine.acknowledge_alarm('a1', 'op1')
        assert engine.acknowledge_alarm('a1', 'op2') is False

    def test_acknowledged_not_in_active_escalations(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        engine.acknowledge_alarm('a1', 'op1')
        active = engine.get_active_escalations()
        alarm_ids = [a[0] for a in active]
        assert 'a1' not in alarm_ids


# ---------------------------------------------------------------------------
# Tests: Auto-acknowledge after timeout
# ---------------------------------------------------------------------------

class TestAutoAcknowledge:
    """Verify alarms are auto-acknowledged based on policy setting."""

    def test_auto_ack_communication_alarm(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='c1', alarm_type='COMMUNICATION', timestamp=1000.0)
        engine.process_alarm(alarm)
        actions = engine.update_escalation_timers(1601.0)  # >600s
        ack_actions = [a for a in actions if a.action_type == EscalationEngine.ACKNOWLEDGE]
        assert len(ack_actions) == 1
        assert ack_actions[0].alarm_id == 'c1'

    def test_no_auto_ack_before_timeout(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='c1', alarm_type='COMMUNICATION', timestamp=1000.0)
        engine.process_alarm(alarm)
        actions = engine.update_escalation_timers(1100.0)  # only 100s
        ack_actions = [a for a in actions if a.action_type == EscalationEngine.ACKNOWLEDGE]
        assert len(ack_actions) == 0

    def test_no_auto_ack_for_safety(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='s1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        actions = engine.update_escalation_timers(5000.0)
        ack_actions = [a for a in actions if a.action_type == EscalationEngine.ACKNOWLEDGE]
        assert len(ack_actions) == 0

    def test_auto_ack_stops_further_escalation(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='c1', alarm_type='COMMUNICATION', timestamp=1000.0)
        engine.process_alarm(alarm)
        engine.update_escalation_timers(1601.0)  # triggers auto-ack
        further = engine.update_escalation_timers(2000.0)
        assert len(further) == 0


# ---------------------------------------------------------------------------
# Tests: Active escalations listing
# ---------------------------------------------------------------------------

class TestActiveEscalations:
    """Verify get_active_escalations returns correct info."""

    def test_single_active_alarm(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        active = engine.get_active_escalations()
        assert len(active) == 1
        assert active[0][0] == 'a1'
        assert active[0][1] == 1  # current level

    def test_next_escalation_in_provided(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        active = engine.get_active_escalations()
        # next_escalation_in should be a float (time until level 2)
        assert active[0][3] is not None
        assert active[0][3] >= 0

    def test_no_next_at_max_level(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        engine.update_escalation_timers(1061.0)
        engine.update_escalation_timers(1181.0)
        engine.update_escalation_timers(1301.0)  # level 4 = max
        active = engine.get_active_escalations()
        assert active[0][3] is None  # no further escalation


# ---------------------------------------------------------------------------
# Tests: Escalation history tracking
# ---------------------------------------------------------------------------

class TestEscalationHistory:
    """Verify get_escalation_history returns correct records."""

    def test_initial_history_entry(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        history = engine.get_escalation_history('a1')
        assert len(history) == 1
        assert history[0][0] == 1  # level
        assert history[0][2] == EscalationEngine.NOTIFY  # action

    def test_history_grows_with_escalation(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        engine.update_escalation_timers(1061.0)
        history = engine.get_escalation_history('a1')
        assert len(history) == 2

    def test_history_includes_ack(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        engine.acknowledge_alarm('a1', 'op1')
        history = engine.get_escalation_history('a1')
        actions = [h[2] for h in history]
        assert EscalationEngine.ACKNOWLEDGE in actions

    def test_history_unknown_alarm_empty(self):
        engine = EscalationEngine()
        assert engine.get_escalation_history('nonexistent') == []


# ---------------------------------------------------------------------------
# Tests: Multiple concurrent alarms
# ---------------------------------------------------------------------------

class TestMultipleConcurrentAlarms:
    """Verify engine handles multiple alarms independently."""

    def test_two_alarms_independent_escalation(self):
        engine = EscalationEngine()
        a1 = _make_alarm(alarm_id='a1', alarm_type='SAFETY', source='cnc1', timestamp=1000.0)
        a2 = _make_alarm(alarm_id='a2', alarm_type='TOOL', source='cnc2', timestamp=1050.0)
        engine.process_alarm(a1)
        engine.process_alarm(a2)

        actions = engine.update_escalation_timers(1061.0)
        # Only a1 should escalate (SAFETY level 2 at 60s), a2 only 11s old
        ids = [a.alarm_id for a in actions]
        assert 'a1' in ids
        assert 'a2' not in ids

    def test_ack_one_leaves_other_active(self):
        engine = EscalationEngine()
        a1 = _make_alarm(alarm_id='a1', alarm_type='SAFETY', source='cnc1', timestamp=1000.0)
        a2 = _make_alarm(alarm_id='a2', alarm_type='QUALITY', source='cnc2', timestamp=1000.0)
        engine.process_alarm(a1)
        engine.process_alarm(a2)
        engine.acknowledge_alarm('a1', 'op1')

        active = engine.get_active_escalations()
        ids = [a[0] for a in active]
        assert 'a1' not in ids
        assert 'a2' in ids


# ---------------------------------------------------------------------------
# Tests: Unknown alarm type -> default policy
# ---------------------------------------------------------------------------

class TestUnknownAlarmType:
    """Verify unknown alarm types fall back to default policy."""

    def test_unknown_type_uses_fallback(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_type='COMPLETELY_UNKNOWN')
        result = engine.process_alarm(alarm)
        assert result.action_type == EscalationEngine.NOTIFY
        assert result.level == 1

    def test_unknown_type_tracked(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='u1', alarm_type='UNKNOWN_TYPE')
        engine.process_alarm(alarm)
        assert 'u1' in engine._active


# ---------------------------------------------------------------------------
# Tests: Emergency level actions
# ---------------------------------------------------------------------------

class TestEmergencyActions:
    """Verify emergency-level actions produce correct results."""

    def test_safety_level_4_lockout(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='a1', alarm_type='SAFETY', timestamp=1000.0)
        engine.process_alarm(alarm)
        engine.update_escalation_timers(1061.0)
        engine.update_escalation_timers(1181.0)
        actions = engine.update_escalation_timers(1301.0)
        assert len(actions) == 1
        assert actions[0].action_type == EscalationEngine.LOCKOUT
        assert 'emergency_team' in actions[0].notify_roles

    def test_thermal_level_4_lockout(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='t1', alarm_type='THERMAL', source='cnc3',
                            timestamp=1000.0)
        engine.process_alarm(alarm)
        engine.update_escalation_timers(1061.0)   # level 2
        engine.update_escalation_timers(1181.0)   # level 3
        actions = engine.update_escalation_timers(1301.0)   # level 4
        assert len(actions) == 1
        assert actions[0].action_type == EscalationEngine.LOCKOUT

    def test_tool_level_3_auto_stop(self):
        engine = EscalationEngine()
        alarm = _make_alarm(alarm_id='tl1', alarm_type='TOOL', source='cnc1',
                            timestamp=1000.0)
        engine.process_alarm(alarm)
        engine.update_escalation_timers(1091.0)  # level 2 (90s)
        actions = engine.update_escalation_timers(1241.0)  # level 3 (240s)
        assert len(actions) == 1
        assert actions[0].action_type == EscalationEngine.AUTO_STOP


# ---------------------------------------------------------------------------
# Tests: EscalationActionResult dataclass
# ---------------------------------------------------------------------------

class TestEscalationActionResultDataclass:
    """Verify the EscalationActionResult dataclass fields."""

    def test_default_values(self):
        result = EscalationActionResult(action_type='NOTIFY', alarm_id='x')
        assert result.level == 1
        assert result.notify_roles == []
        assert result.message == ''

    def test_all_fields(self):
        result = EscalationActionResult(
            action_type='PAGE',
            alarm_id='a1',
            level=2,
            notify_roles=['supervisor'],
            message='test message',
        )
        assert result.action_type == 'PAGE'
        assert result.alarm_id == 'a1'
        assert result.level == 2
        assert result.notify_roles == ['supervisor']
        assert result.message == 'test message'


# ---------------------------------------------------------------------------
# Tests: EscalationPolicy and EscalationLevel dataclasses
# ---------------------------------------------------------------------------

class TestDataclasses:
    """Verify dataclass defaults and construction."""

    def test_escalation_level_defaults(self):
        el = EscalationLevel(level=1, delay_sec=30.0)
        assert el.notify_roles == []
        assert el.action == 'notify'

    def test_escalation_policy_defaults(self):
        ep = EscalationPolicy(policy_id='p1', name='Test')
        assert ep.alarm_types == []
        assert ep.levels == []
        assert ep.auto_acknowledge_after_sec == 0.0
        assert ep.suppress_duplicate_window_sec == 60.0

    def test_escalation_policy_custom_suppress_window(self):
        ep = EscalationPolicy(
            policy_id='p2', name='Test2',
            suppress_duplicate_window_sec=120.0,
        )
        assert ep.suppress_duplicate_window_sec == 120.0
