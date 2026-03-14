"""Tests for the multi-channel NotificationDispatcher.

Validates notification dispatching, severity filtering, quiet hours,
rate limiting, channel suppression, acknowledgment, history, and
per-channel formatting without any ROS2 runtime dependency.
"""

import os
import sys
import types
from typing import Any

import pytest

# ---------------------------------------------------------------------------
# Add package root so miracle_scada is importable
# ---------------------------------------------------------------------------
_PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

# ---------------------------------------------------------------------------
# Mock ROS2 / miracle infrastructure (same pattern as other SCADA tests)
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
# Import production code
# ---------------------------------------------------------------------------

from miracle_scada.alarm_manager import (  # noqa: E402
    NotificationChannel,
    NotificationDispatcher,
    Notification,
    SEVERITY_ORDER,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def dispatcher():
    """Return a fresh NotificationDispatcher with default channels."""
    return NotificationDispatcher()


@pytest.fixture
def multi_dispatcher():
    """Return a dispatcher with DASHBOARD, EMAIL, SMS, AUDIO channels."""
    d = NotificationDispatcher()
    d.register_channel(NotificationChannel(
        channel_type='EMAIL', enabled=True, min_severity='WARNING',
        rate_limit_per_min=10,
    ))
    d.register_channel(NotificationChannel(
        channel_type='SMS', enabled=True, min_severity='CRITICAL',
        rate_limit_per_min=5,
    ))
    d.register_channel(NotificationChannel(
        channel_type='AUDIO', enabled=True, min_severity='WARNING',
        rate_limit_per_min=20,
    ))
    return d


# ---------------------------------------------------------------------------
# Tests: Dashboard always dispatched
# ---------------------------------------------------------------------------

class TestDashboardAlwaysDispatched:

    def test_info_dispatched_to_dashboard(self, dispatcher):
        result = dispatcher.dispatch('A1', 'INFO', 'Low temp', 'Temp is low',
                                     current_time=1000.0, current_hour=12)
        assert len(result) == 1
        assert result[0].channel == 'DASHBOARD'
        assert result[0].delivered is True

    def test_warning_dispatched_to_dashboard(self, dispatcher):
        result = dispatcher.dispatch('A2', 'WARNING', 'High temp', 'Temp high',
                                     current_time=1000.0, current_hour=12)
        assert len(result) == 1
        assert result[0].channel == 'DASHBOARD'

    def test_critical_dispatched_to_dashboard(self, dispatcher):
        result = dispatcher.dispatch('A3', 'CRITICAL', 'Overtemp', 'Meltdown',
                                     current_time=1000.0, current_hour=12)
        assert len(result) == 1
        assert result[0].channel == 'DASHBOARD'


# ---------------------------------------------------------------------------
# Tests: Severity filtering
# ---------------------------------------------------------------------------

class TestSeverityFiltering:

    def test_info_not_sent_to_sms(self, multi_dispatcher):
        """INFO severity should not reach SMS (min_severity=CRITICAL)."""
        result = multi_dispatcher.dispatch('A1', 'INFO', 'Test', 'Body',
                                           current_time=1000.0, current_hour=12)
        channels = {n.channel for n in result}
        assert 'SMS' not in channels
        assert 'DASHBOARD' in channels

    def test_info_not_sent_to_email(self, multi_dispatcher):
        """INFO severity should not reach EMAIL (min_severity=WARNING)."""
        result = multi_dispatcher.dispatch('A1', 'INFO', 'Test', 'Body',
                                           current_time=1000.0, current_hour=12)
        channels = {n.channel for n in result}
        assert 'EMAIL' not in channels

    def test_warning_sent_to_email_not_sms(self, multi_dispatcher):
        result = multi_dispatcher.dispatch('A1', 'WARNING', 'Warn', 'Body',
                                           current_time=1000.0, current_hour=12)
        channels = {n.channel for n in result}
        assert 'EMAIL' in channels
        assert 'SMS' not in channels

    def test_critical_sent_to_all(self, multi_dispatcher):
        result = multi_dispatcher.dispatch('A1', 'CRITICAL', 'Crit', 'Body',
                                           current_time=1000.0, current_hour=12)
        channels = {n.channel for n in result}
        assert 'DASHBOARD' in channels
        assert 'EMAIL' in channels
        assert 'SMS' in channels
        assert 'AUDIO' in channels


# ---------------------------------------------------------------------------
# Tests: Quiet hours suppression
# ---------------------------------------------------------------------------

class TestQuietHours:

    def test_quiet_hours_suppress(self, dispatcher):
        dispatcher.set_quiet_hours('DASHBOARD', 22, 6)
        result = dispatcher.dispatch('A1', 'CRITICAL', 'Test', 'Body',
                                     current_time=1000.0, current_hour=23)
        assert len(result) == 0

    def test_quiet_hours_allow_outside(self, dispatcher):
        dispatcher.set_quiet_hours('DASHBOARD', 22, 6)
        result = dispatcher.dispatch('A1', 'CRITICAL', 'Test', 'Body',
                                     current_time=1000.0, current_hour=12)
        assert len(result) == 1

    def test_quiet_hours_wrap_midnight_early_morning(self, dispatcher):
        dispatcher.set_quiet_hours('DASHBOARD', 22, 6)
        result = dispatcher.dispatch('A1', 'CRITICAL', 'Test', 'Body',
                                     current_time=1000.0, current_hour=3)
        assert len(result) == 0

    def test_quiet_hours_same_day_range(self, dispatcher):
        """Quiet hours 8-17 should suppress at hour 10."""
        dispatcher.set_quiet_hours('DASHBOARD', 8, 17)
        result = dispatcher.dispatch('A1', 'CRITICAL', 'Test', 'Body',
                                     current_time=1000.0, current_hour=10)
        assert len(result) == 0

    def test_quiet_hours_same_day_allow_outside(self, dispatcher):
        dispatcher.set_quiet_hours('DASHBOARD', 8, 17)
        result = dispatcher.dispatch('A1', 'CRITICAL', 'Test', 'Body',
                                     current_time=1000.0, current_hour=18)
        assert len(result) == 1


# ---------------------------------------------------------------------------
# Tests: Rate limiting
# ---------------------------------------------------------------------------

class TestRateLimiting:

    def test_rate_limit_blocks_excess(self, dispatcher):
        """Dashboard has rate_limit_per_min=60; a channel with limit 3 should block."""
        d = NotificationDispatcher()
        d.register_channel(NotificationChannel(
            channel_type='DASHBOARD', enabled=True, min_severity='INFO',
            rate_limit_per_min=3,
        ))
        results = []
        for i in range(5):
            r = d.dispatch(f'A{i}', 'INFO', 'Test', 'Body',
                           current_time=1000.0, current_hour=12)
            results.extend(r)
        assert len(results) == 3

    def test_rate_limit_resets_after_window(self, dispatcher):
        d = NotificationDispatcher()
        d.register_channel(NotificationChannel(
            channel_type='DASHBOARD', enabled=True, min_severity='INFO',
            rate_limit_per_min=2,
        ))
        r1 = d.dispatch('A1', 'INFO', 'T', 'B', current_time=1000.0, current_hour=12)
        r2 = d.dispatch('A2', 'INFO', 'T', 'B', current_time=1000.5, current_hour=12)
        r3 = d.dispatch('A3', 'INFO', 'T', 'B', current_time=1001.0, current_hour=12)
        assert len(r1) == 1
        assert len(r2) == 1
        assert len(r3) == 0  # rate limited

        # After 60s window, old entries expire
        r4 = d.dispatch('A4', 'INFO', 'T', 'B', current_time=1061.0, current_hour=12)
        assert len(r4) == 1


# ---------------------------------------------------------------------------
# Tests: Multi-channel dispatch
# ---------------------------------------------------------------------------

class TestMultiChannelDispatch:

    def test_multiple_recipients(self, multi_dispatcher):
        result = multi_dispatcher.dispatch(
            'A1', 'CRITICAL', 'Alert', 'Body',
            recipients=['operator', 'supervisor'],
            current_time=1000.0, current_hour=12,
        )
        # 4 channels x 2 recipients = 8 notifications
        assert len(result) == 8

    def test_each_recipient_gets_each_channel(self, multi_dispatcher):
        result = multi_dispatcher.dispatch(
            'A1', 'CRITICAL', 'Alert', 'Body',
            recipients=['op1', 'op2'],
            current_time=1000.0, current_hour=12,
        )
        op1_channels = {n.channel for n in result if n.recipient == 'op1'}
        op2_channels = {n.channel for n in result if n.recipient == 'op2'}
        assert op1_channels == {'DASHBOARD', 'EMAIL', 'SMS', 'AUDIO'}
        assert op2_channels == {'DASHBOARD', 'EMAIL', 'SMS', 'AUDIO'}


# ---------------------------------------------------------------------------
# Tests: Notification history
# ---------------------------------------------------------------------------

class TestNotificationHistory:

    def test_history_returns_dispatched(self, dispatcher):
        dispatcher.dispatch('A1', 'INFO', 'T', 'B', current_time=1000.0, current_hour=12)
        dispatcher.dispatch('A2', 'INFO', 'T', 'B', current_time=1001.0, current_hour=12)
        history = dispatcher.get_notification_history()
        assert len(history) == 2

    def test_history_filter_by_alarm_id(self, dispatcher):
        dispatcher.dispatch('A1', 'INFO', 'T', 'B', current_time=1000.0, current_hour=12)
        dispatcher.dispatch('A2', 'INFO', 'T', 'B', current_time=1001.0, current_hour=12)
        history = dispatcher.get_notification_history(alarm_id='A1')
        assert len(history) == 1
        assert history[0].alarm_id == 'A1'

    def test_history_filter_by_channel(self, multi_dispatcher):
        multi_dispatcher.dispatch('A1', 'CRITICAL', 'T', 'B',
                                  current_time=1000.0, current_hour=12)
        history = multi_dispatcher.get_notification_history(channel='SMS')
        assert all(n.channel == 'SMS' for n in history)

    def test_history_last_n(self, dispatcher):
        for i in range(10):
            dispatcher.dispatch(f'A{i}', 'INFO', 'T', 'B',
                                current_time=1000.0 + i, current_hour=12)
        history = dispatcher.get_notification_history(last_n=3)
        assert len(history) == 3


# ---------------------------------------------------------------------------
# Tests: Acknowledge notification
# ---------------------------------------------------------------------------

class TestAcknowledgeNotification:

    def test_acknowledge_existing(self, dispatcher):
        result = dispatcher.dispatch('A1', 'INFO', 'T', 'B',
                                     current_time=1000.0, current_hour=12)
        nid = result[0].notification_id
        assert dispatcher.acknowledge_notification(nid) is True
        # Verify it's acknowledged
        history = dispatcher.get_notification_history()
        assert history[0].acknowledged is True

    def test_acknowledge_nonexistent(self, dispatcher):
        assert dispatcher.acknowledge_notification('nonexistent') is False

    def test_acknowledge_idempotent(self, dispatcher):
        result = dispatcher.dispatch('A1', 'INFO', 'T', 'B',
                                     current_time=1000.0, current_hour=12)
        nid = result[0].notification_id
        assert dispatcher.acknowledge_notification(nid) is True
        assert dispatcher.acknowledge_notification(nid) is True  # already acked, still returns True


# ---------------------------------------------------------------------------
# Tests: Unacknowledged listing
# ---------------------------------------------------------------------------

class TestUnacknowledged:

    def test_all_unacknowledged_initially(self, dispatcher):
        dispatcher.dispatch('A1', 'INFO', 'T', 'B', current_time=1000.0, current_hour=12)
        dispatcher.dispatch('A2', 'INFO', 'T', 'B', current_time=1001.0, current_hour=12)
        unacked = dispatcher.get_unacknowledged()
        assert len(unacked) == 2

    def test_unacknowledged_decreases_after_ack(self, dispatcher):
        r1 = dispatcher.dispatch('A1', 'INFO', 'T', 'B', current_time=1000.0, current_hour=12)
        dispatcher.dispatch('A2', 'INFO', 'T', 'B', current_time=1001.0, current_hour=12)
        dispatcher.acknowledge_notification(r1[0].notification_id)
        unacked = dispatcher.get_unacknowledged()
        assert len(unacked) == 1

    def test_unacknowledged_filter_by_channel(self, multi_dispatcher):
        multi_dispatcher.dispatch('A1', 'CRITICAL', 'T', 'B',
                                  current_time=1000.0, current_hour=12)
        unacked_sms = multi_dispatcher.get_unacknowledged(channel='SMS')
        assert all(n.channel == 'SMS' for n in unacked_sms)
        assert len(unacked_sms) >= 1


# ---------------------------------------------------------------------------
# Tests: Delivery stats
# ---------------------------------------------------------------------------

class TestDeliveryStats:

    def test_stats_empty(self, dispatcher):
        stats = dispatcher.get_delivery_stats()
        assert stats['total_sent'] == 0
        assert stats['by_channel'] == {}
        assert stats['by_severity'] == {}
        assert stats['failed_count'] == 0

    def test_stats_after_dispatches(self, multi_dispatcher):
        multi_dispatcher.dispatch('A1', 'CRITICAL', 'T', 'B',
                                  current_time=1000.0, current_hour=12)
        multi_dispatcher.dispatch('A2', 'WARNING', 'T', 'B',
                                  current_time=1001.0, current_hour=12)
        stats = multi_dispatcher.get_delivery_stats()
        assert stats['total_sent'] > 0
        assert 'DASHBOARD' in stats['by_channel']
        assert 'CRITICAL' in stats['by_severity']
        assert stats['failed_count'] == 0

    def test_stats_by_channel_counts(self, multi_dispatcher):
        multi_dispatcher.dispatch('A1', 'CRITICAL', 'T', 'B',
                                  current_time=1000.0, current_hour=12)
        stats = multi_dispatcher.get_delivery_stats()
        # CRITICAL should go to all 4 channels
        assert stats['by_channel']['DASHBOARD'] == 1
        assert stats['by_channel']['EMAIL'] == 1
        assert stats['by_channel']['SMS'] == 1
        assert stats['by_channel']['AUDIO'] == 1


# ---------------------------------------------------------------------------
# Tests: Channel suppression
# ---------------------------------------------------------------------------

class TestChannelSuppression:

    def test_suppress_channel(self, dispatcher):
        dispatcher.suppress_channel('DASHBOARD', 120.0, current_time=1000.0)
        result = dispatcher.dispatch('A1', 'CRITICAL', 'T', 'B',
                                     current_time=1050.0, current_hour=12)
        assert len(result) == 0

    def test_suppress_expires(self, dispatcher):
        dispatcher.suppress_channel('DASHBOARD', 60.0, current_time=1000.0)
        result = dispatcher.dispatch('A1', 'CRITICAL', 'T', 'B',
                                     current_time=1061.0, current_hour=12)
        assert len(result) == 1

    def test_suppress_one_channel_others_work(self, multi_dispatcher):
        multi_dispatcher.suppress_channel('EMAIL', 120.0, current_time=1000.0)
        result = multi_dispatcher.dispatch('A1', 'CRITICAL', 'T', 'B',
                                           current_time=1050.0, current_hour=12)
        channels = {n.channel for n in result}
        assert 'EMAIL' not in channels
        assert 'DASHBOARD' in channels


# ---------------------------------------------------------------------------
# Tests: Format per channel type
# ---------------------------------------------------------------------------

class TestFormatNotification:

    def test_dashboard_format_short(self, dispatcher):
        subj, body = dispatcher.format_notification(
            {'alarm_id': 'A1', 'severity': 'WARNING',
             'subject': 'High temp', 'body': 'Temperature exceeded threshold'},
            'DASHBOARD',
        )
        assert '[WARNING]' in subj
        assert len(body) <= 200

    def test_email_format_full_detail(self, dispatcher):
        subj, body = dispatcher.format_notification(
            {'alarm_id': 'A1', 'severity': 'CRITICAL',
             'subject': 'Overtemp', 'body': 'Machine on fire'},
            'EMAIL',
        )
        assert 'A1' in subj
        assert 'Alarm ID: A1' in body
        assert 'Severity: CRITICAL' in body

    def test_sms_truncation(self, dispatcher):
        long_body = 'A' * 300
        subj, body = dispatcher.format_notification(
            {'alarm_id': 'A1', 'severity': 'CRITICAL',
             'subject': 'Alert', 'body': long_body},
            'SMS',
        )
        assert len(subj) <= 160
        assert subj.endswith('...')

    def test_sms_short_not_truncated(self, dispatcher):
        subj, body = dispatcher.format_notification(
            {'alarm_id': 'A1', 'severity': 'CRITICAL',
             'subject': 'Ok', 'body': 'Fine'},
            'SMS',
        )
        assert not subj.endswith('...')
        assert len(subj) <= 160

    def test_audio_format_spoken(self, dispatcher):
        subj, body = dispatcher.format_notification(
            {'alarm_id': 'A1', 'severity': 'CRITICAL',
             'subject': 'Overtemp', 'body': 'Machine hot'},
            'AUDIO',
        )
        assert 'Attention' in body
        assert 'CRITICAL' in body

    def test_generic_channel_format(self, dispatcher):
        subj, body = dispatcher.format_notification(
            {'alarm_id': 'A1', 'severity': 'INFO',
             'subject': 'Status', 'body': 'All good'},
            'MQTT',
        )
        assert '[INFO]' in subj


# ---------------------------------------------------------------------------
# Tests: Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:

    def test_no_channels_registered(self):
        d = NotificationDispatcher()
        # Remove the default dashboard channel
        d._channels.clear()
        result = d.dispatch('A1', 'CRITICAL', 'T', 'B',
                            current_time=1000.0, current_hour=12)
        assert len(result) == 0

    def test_all_channels_disabled(self):
        d = NotificationDispatcher()
        d.register_channel(NotificationChannel(
            channel_type='DASHBOARD', enabled=False,
        ))
        result = d.dispatch('A1', 'CRITICAL', 'T', 'B',
                            current_time=1000.0, current_hour=12)
        assert len(result) == 0

    def test_unknown_severity(self, dispatcher):
        """Unknown severity treated as lowest (0)."""
        result = dispatcher.dispatch('A1', 'UNKNOWN', 'T', 'B',
                                     current_time=1000.0, current_hour=12)
        # DASHBOARD min_severity=INFO (level 0), unknown severity = 0 => dispatched
        assert len(result) == 1

    def test_empty_recipients(self, dispatcher):
        result = dispatcher.dispatch('A1', 'INFO', 'T', 'B',
                                     recipients=[],
                                     current_time=1000.0, current_hour=12)
        assert len(result) == 0

    def test_default_recipients(self, dispatcher):
        result = dispatcher.dispatch('A1', 'INFO', 'T', 'B',
                                     current_time=1000.0, current_hour=12)
        assert result[0].recipient == 'operator'

    def test_notification_fields_populated(self, dispatcher):
        result = dispatcher.dispatch('A1', 'WARNING', 'Subject', 'Body text',
                                     current_time=1000.0, current_hour=12)
        n = result[0]
        assert n.alarm_id == 'A1'
        assert n.severity == 'WARNING'
        assert n.timestamp == 1000.0
        assert n.delivery_attempts == 1
        assert len(n.notification_id) > 0

    def test_set_quiet_hours_unknown_channel(self, dispatcher):
        """Setting quiet hours on non-existent channel should not raise."""
        dispatcher.set_quiet_hours('NONEXISTENT', 22, 6)  # no error

    def test_register_channel_replaces_existing(self, dispatcher):
        dispatcher.register_channel(NotificationChannel(
            channel_type='DASHBOARD', enabled=True, min_severity='CRITICAL',
        ))
        result = dispatcher.dispatch('A1', 'INFO', 'T', 'B',
                                     current_time=1000.0, current_hour=12)
        assert len(result) == 0  # INFO < CRITICAL
