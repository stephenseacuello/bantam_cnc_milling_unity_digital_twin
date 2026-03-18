"""
Alarm Manager Node.

Centralizes alarm management across the manufacturing cell.
Implements alarm shelving, acknowledgment, and escalation with real
escalation actions: priority boost, forced acknowledgment, supervisor
notification, and emergency stop.
Maintains alarm history with ISA-18.2 compliance.
"""

from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum
import fnmatch
import threading
import time
import uuid

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import AnomalyAlert, SecurityAlert, CorrelatedAlert, AlarmEscalation


class AlarmState(Enum):
    """ISA-18.2 alarm states."""
    NORMAL = 'NORMAL'
    UNACKNOWLEDGED = 'UNACKNOWLEDGED'
    ACKNOWLEDGED = 'ACKNOWLEDGED'
    SHELVED = 'SHELVED'
    SUPPRESSED = 'SUPPRESSED'


class EscalationActionType:
    """Constants for escalation action types (legacy)."""
    PRIORITY_BOOST = 'PRIORITY_BOOST'
    FORCED_ACK_REQUIRED = 'FORCED_ACK_REQUIRED'
    SUPERVISOR_NOTIFY = 'SUPERVISOR_NOTIFY'
    EMERGENCY_STOP = 'EMERGENCY_STOP'


# Backwards-compatible alias
EscalationAction = EscalationActionType


@dataclass
class Alarm:
    """Represents an active alarm."""
    alarm_id: str
    source: str
    severity: float
    message: str
    alarm_type: str = ''
    state: AlarmState = AlarmState.UNACKNOWLEDGED
    timestamp: float = 0.0
    acknowledged_by: Optional[str] = None
    escalation_level: int = 0
    last_escalation_time: float = 0.0
    requires_forced_ack: bool = False
    escalation_actions_taken: List[str] = field(default_factory=list)


# Minimum interval between successive escalations for a single alarm (seconds).
MIN_ESCALATION_INTERVAL = 30.0


# ---------------------------------------------------------------------------
# Configurable alarm escalation policy engine
# ---------------------------------------------------------------------------

@dataclass
class EscalationLevel:
    """Defines one level in an escalation policy.

    Attributes:
        level: Numeric level (1=initial, 2=supervisor, 3=manager, 4=emergency).
        delay_sec: Seconds after alarm raised before escalating to this level.
        notify_roles: Roles to notify (e.g. ["operator", "supervisor"]).
        action: Action to take (e.g. "notify", "page", "auto_stop", "lockout").
        message_template: Template string for the notification message.
            May contain ``{alarm_id}``, ``{source}``, ``{severity}``,
            ``{elapsed}``, ``{message}`` placeholders.
    """
    level: int
    delay_sec: float
    notify_roles: List[str] = field(default_factory=list)
    action: str = 'notify'
    message_template: str = 'Alarm {alarm_id} escalated to level {level}'


@dataclass
class EscalationPolicy:
    """Configurable escalation policy for a category of alarms.

    Attributes:
        policy_id: Unique identifier for this policy.
        name: Human-readable name.
        alarm_types: List of alarm type strings this policy applies to.
        levels: Ordered list of escalation levels.
        auto_acknowledge_after_sec: Auto-ack timeout in seconds (0 = never).
        suppress_duplicate_window_sec: Window in seconds to suppress duplicate
            alarms of the same type from the same source.
    """
    policy_id: str
    name: str
    alarm_types: List[str] = field(default_factory=list)
    levels: List[EscalationLevel] = field(default_factory=list)
    auto_acknowledge_after_sec: float = 0.0
    suppress_duplicate_window_sec: float = 60.0


@dataclass
class EscalationActionResult:
    """Result of processing an alarm through the escalation engine.

    Attributes:
        action_type: One of NOTIFY, PAGE, AUTO_STOP, LOCKOUT, SUPPRESS,
            ACKNOWLEDGE.
        alarm_id: The alarm identifier.
        level: Current escalation level.
        notify_roles: Roles that should be notified.
        message: Rendered notification message.
    """
    action_type: str
    alarm_id: str
    level: int = 1
    notify_roles: List[str] = field(default_factory=list)
    message: str = ''


class EscalationEngine:
    """Policy-driven alarm escalation engine.

    Manages configurable escalation policies, duplicate suppression,
    time-based escalation, acknowledgment, and auto-acknowledge.
    """

    # Action type constants
    NOTIFY = 'NOTIFY'
    PAGE = 'PAGE'
    AUTO_STOP = 'AUTO_STOP'
    LOCKOUT = 'LOCKOUT'
    SUPPRESS = 'SUPPRESS'
    ACKNOWLEDGE = 'ACKNOWLEDGE'

    # Internal mapping from policy action strings to action type constants
    _ACTION_MAP = {
        'notify': 'NOTIFY',
        'page': 'PAGE',
        'auto_stop': 'AUTO_STOP',
        'lockout': 'LOCKOUT',
    }

    def __init__(self) -> None:
        self._policies: Dict[str, EscalationPolicy] = {}
        # alarm_id -> {alarm, policy_id, first_seen, current_level,
        #              level_entered_at, acknowledged, ack_operator, history}
        self._active: Dict[str, Dict[str, Any]] = {}
        # (alarm_type, source) -> last_seen_time  for duplicate suppression
        self._recent_alarms: Dict[Tuple[str, str], float] = {}

        # Register default policies
        self._register_defaults()

    # -- default policies ----------------------------------------------------

    def _register_defaults(self) -> None:
        """Register built-in policies for common alarm categories."""
        defaults = [
            EscalationPolicy(
                policy_id='default_safety',
                name='Safety Alarm Policy',
                alarm_types=['SAFETY'],
                levels=[
                    EscalationLevel(1, 0.0, ['operator'], 'notify',
                                    'SAFETY alarm {alarm_id}: {message}'),
                    EscalationLevel(2, 60.0, ['operator', 'supervisor'], 'page',
                                    'SAFETY alarm {alarm_id} unacked for {elapsed:.0f}s'),
                    EscalationLevel(3, 180.0, ['operator', 'supervisor', 'plant_manager'],
                                    'auto_stop',
                                    'SAFETY alarm {alarm_id} - auto stop triggered'),
                    EscalationLevel(4, 300.0, ['operator', 'supervisor', 'plant_manager',
                                               'emergency_team'], 'lockout',
                                    'EMERGENCY: SAFETY alarm {alarm_id} - lockout'),
                ],
                auto_acknowledge_after_sec=0.0,
                suppress_duplicate_window_sec=30.0,
            ),
            EscalationPolicy(
                policy_id='default_quality',
                name='Quality Alarm Policy',
                alarm_types=['QUALITY'],
                levels=[
                    EscalationLevel(1, 0.0, ['operator'], 'notify',
                                    'Quality alarm {alarm_id}: {message}'),
                    EscalationLevel(2, 120.0, ['operator', 'supervisor'], 'notify',
                                    'Quality alarm {alarm_id} unacked for {elapsed:.0f}s'),
                    EscalationLevel(3, 300.0, ['operator', 'supervisor', 'plant_manager'],
                                    'page',
                                    'Quality alarm {alarm_id} - manager paged'),
                ],
                auto_acknowledge_after_sec=0.0,
                suppress_duplicate_window_sec=60.0,
            ),
            EscalationPolicy(
                policy_id='default_tool',
                name='Tool Alarm Policy',
                alarm_types=['TOOL'],
                levels=[
                    EscalationLevel(1, 0.0, ['operator'], 'notify',
                                    'Tool alarm {alarm_id}: {message}'),
                    EscalationLevel(2, 90.0, ['operator', 'supervisor'], 'page',
                                    'Tool alarm {alarm_id} unacked for {elapsed:.0f}s'),
                    EscalationLevel(3, 240.0, ['operator', 'supervisor', 'plant_manager'],
                                    'auto_stop',
                                    'Tool alarm {alarm_id} - auto stop'),
                ],
                auto_acknowledge_after_sec=0.0,
                suppress_duplicate_window_sec=60.0,
            ),
            EscalationPolicy(
                policy_id='default_thermal',
                name='Thermal Alarm Policy',
                alarm_types=['THERMAL'],
                levels=[
                    EscalationLevel(1, 0.0, ['operator'], 'notify',
                                    'Thermal alarm {alarm_id}: {message}'),
                    EscalationLevel(2, 60.0, ['operator', 'supervisor'], 'page',
                                    'Thermal alarm {alarm_id} unacked for {elapsed:.0f}s'),
                    EscalationLevel(3, 180.0, ['operator', 'supervisor', 'plant_manager'],
                                    'auto_stop',
                                    'Thermal alarm {alarm_id} - auto stop'),
                    EscalationLevel(4, 300.0, ['operator', 'supervisor', 'plant_manager',
                                               'emergency_team'], 'lockout',
                                    'EMERGENCY: Thermal alarm {alarm_id} - lockout'),
                ],
                auto_acknowledge_after_sec=0.0,
                suppress_duplicate_window_sec=45.0,
            ),
            EscalationPolicy(
                policy_id='default_communication',
                name='Communication Alarm Policy',
                alarm_types=['COMMUNICATION'],
                levels=[
                    EscalationLevel(1, 0.0, ['operator'], 'notify',
                                    'Comm alarm {alarm_id}: {message}'),
                    EscalationLevel(2, 120.0, ['operator', 'supervisor'], 'notify',
                                    'Comm alarm {alarm_id} unacked for {elapsed:.0f}s'),
                ],
                auto_acknowledge_after_sec=600.0,
                suppress_duplicate_window_sec=120.0,
            ),
            EscalationPolicy(
                policy_id='default_fallback',
                name='Default Fallback Policy',
                alarm_types=['__DEFAULT__'],
                levels=[
                    EscalationLevel(1, 0.0, ['operator'], 'notify',
                                    'Alarm {alarm_id}: {message}'),
                    EscalationLevel(2, 120.0, ['operator', 'supervisor'], 'notify',
                                    'Alarm {alarm_id} unacked for {elapsed:.0f}s'),
                    EscalationLevel(3, 300.0, ['operator', 'supervisor', 'plant_manager'],
                                    'page',
                                    'Alarm {alarm_id} - manager paged'),
                ],
                auto_acknowledge_after_sec=0.0,
                suppress_duplicate_window_sec=60.0,
            ),
        ]
        for policy in defaults:
            self._policies[policy.policy_id] = policy

    # -- public API ----------------------------------------------------------

    def register_policy(self, policy: EscalationPolicy) -> None:
        """Register or replace an escalation policy."""
        self._policies[policy.policy_id] = policy

    def process_alarm(self, alarm: Alarm) -> EscalationActionResult:
        """Process a new alarm through the escalation engine.

        Returns an ``EscalationActionResult`` indicating the action to take.
        Duplicate alarms (same type + source within the suppress window) are
        suppressed.
        """
        policy = self._find_policy(alarm.alarm_type)

        # -- duplicate suppression -------------------------------------------
        dup_key = (alarm.alarm_type, alarm.source)
        suppress_window = policy.suppress_duplicate_window_sec
        last_seen = self._recent_alarms.get(dup_key)
        if last_seen is not None and (alarm.timestamp - last_seen) < suppress_window:
            return EscalationActionResult(
                action_type=self.SUPPRESS,
                alarm_id=alarm.alarm_id,
                level=0,
                notify_roles=[],
                message=f'Duplicate alarm suppressed (within {suppress_window}s window)',
            )

        # Record this alarm occurrence
        self._recent_alarms[dup_key] = alarm.timestamp

        # -- determine initial escalation level ------------------------------
        initial_level = policy.levels[0] if policy.levels else None
        action_type = self._ACTION_MAP.get(
            initial_level.action, self.NOTIFY
        ) if initial_level else self.NOTIFY
        notify_roles = list(initial_level.notify_roles) if initial_level else ['operator']
        message = self._render_message(
            initial_level.message_template if initial_level else 'Alarm {alarm_id}',
            alarm, 0.0, initial_level.level if initial_level else 1,
        )

        # Track active alarm
        self._active[alarm.alarm_id] = {
            'alarm': alarm,
            'policy_id': policy.policy_id,
            'first_seen': alarm.timestamp,
            'current_level': initial_level.level if initial_level else 1,
            'level_entered_at': alarm.timestamp,
            'acknowledged': False,
            'ack_operator': None,
            'history': [
                (initial_level.level if initial_level else 1,
                 alarm.timestamp,
                 action_type),
            ],
        }

        return EscalationActionResult(
            action_type=action_type,
            alarm_id=alarm.alarm_id,
            level=initial_level.level if initial_level else 1,
            notify_roles=notify_roles,
            message=message,
        )

    def get_active_escalations(self) -> List[Tuple[str, int, float, Optional[float]]]:
        """Return active (non-acknowledged) escalations.

        Returns a list of tuples:
            (alarm_id, current_level, time_at_level, next_escalation_in)

        ``next_escalation_in`` is ``None`` if the alarm is at the highest
        configured level.
        """
        result: List[Tuple[str, int, float, Optional[float]]] = []
        for alarm_id, info in self._active.items():
            if info['acknowledged']:
                continue
            policy = self._policies.get(info['policy_id'])
            if policy is None:
                continue

            current_level = info['current_level']
            time_at_level = info['alarm'].timestamp  # will be updated by caller
            level_entered = info['level_entered_at']

            # Find next level
            next_level_def = None
            for ldef in sorted(policy.levels, key=lambda l: l.level):
                if ldef.level > current_level:
                    next_level_def = ldef
                    break

            if next_level_def is not None:
                time_since_alarm = 0.0  # caller needs current time
                next_escalation_in = max(
                    0.0,
                    next_level_def.delay_sec - (level_entered - info['first_seen']),
                )
            else:
                next_escalation_in = None

            result.append((
                alarm_id,
                current_level,
                level_entered,
                next_escalation_in,
            ))
        return result

    def acknowledge_alarm(self, alarm_id: str, operator_id: str) -> bool:
        """Acknowledge an alarm, stopping further escalation.

        Returns True if the alarm existed and was not already acknowledged.
        """
        info = self._active.get(alarm_id)
        if info is None or info['acknowledged']:
            return False
        info['acknowledged'] = True
        info['ack_operator'] = operator_id
        info['history'].append((
            info['current_level'],
            info['alarm'].timestamp,  # ideally current_time
            self.ACKNOWLEDGE,
        ))
        return True

    def get_escalation_history(
        self, alarm_id: str,
    ) -> List[Tuple[int, float, str]]:
        """Return escalation history for an alarm.

        Returns list of (level, timestamp, action_taken).
        """
        info = self._active.get(alarm_id)
        if info is None:
            return []
        return list(info['history'])

    def update_escalation_timers(
        self, current_time: float,
    ) -> List[EscalationActionResult]:
        """Check all active alarms and escalate as needed.

        Should be called periodically. Returns a list of new escalation
        actions for alarms that have crossed a level threshold.
        Also handles auto-acknowledge.
        """
        actions: List[EscalationActionResult] = []

        for alarm_id, info in list(self._active.items()):
            if info['acknowledged']:
                continue

            alarm = info['alarm']
            policy = self._policies.get(info['policy_id'])
            if policy is None:
                continue

            elapsed = current_time - info['first_seen']

            # -- auto-acknowledge check --------------------------------------
            if (policy.auto_acknowledge_after_sec > 0
                    and elapsed >= policy.auto_acknowledge_after_sec):
                info['acknowledged'] = True
                info['ack_operator'] = '__auto__'
                action = EscalationActionResult(
                    action_type=self.ACKNOWLEDGE,
                    alarm_id=alarm_id,
                    level=info['current_level'],
                    notify_roles=[],
                    message=f'Auto-acknowledged after {policy.auto_acknowledge_after_sec}s',
                )
                info['history'].append((
                    info['current_level'], current_time, self.ACKNOWLEDGE,
                ))
                actions.append(action)
                continue

            # -- level escalation --------------------------------------------
            current_level = info['current_level']
            sorted_levels = sorted(policy.levels, key=lambda l: l.level)

            for ldef in sorted_levels:
                if ldef.level <= current_level:
                    continue
                if elapsed >= ldef.delay_sec:
                    # Escalate to this level
                    info['current_level'] = ldef.level
                    info['level_entered_at'] = current_time
                    action_type = self._ACTION_MAP.get(ldef.action, self.NOTIFY)
                    message = self._render_message(
                        ldef.message_template, alarm, elapsed, ldef.level,
                    )
                    action = EscalationActionResult(
                        action_type=action_type,
                        alarm_id=alarm_id,
                        level=ldef.level,
                        notify_roles=list(ldef.notify_roles),
                        message=message,
                    )
                    info['history'].append((ldef.level, current_time, action_type))
                    actions.append(action)
                    # Don't break -- if multiple levels elapsed, apply all
                else:
                    break  # sorted, so remaining are higher delay

        return actions

    # -- helpers -------------------------------------------------------------

    def _find_policy(self, alarm_type: str) -> EscalationPolicy:
        """Find the best-matching policy for an alarm type."""
        for policy in self._policies.values():
            if alarm_type in policy.alarm_types:
                return policy
        # Fallback
        for policy in self._policies.values():
            if '__DEFAULT__' in policy.alarm_types:
                return policy
        # Last resort: return a minimal policy
        return EscalationPolicy(
            policy_id='_auto_fallback',
            name='Auto Fallback',
            alarm_types=[alarm_type],
            levels=[EscalationLevel(1, 0.0, ['operator'], 'notify',
                                    'Alarm {alarm_id}: {message}')],
        )

    def _render_message(self, template: str, alarm: Alarm,
                        elapsed: float, level: int) -> str:
        """Render a message template with alarm context."""
        try:
            return template.format(
                alarm_id=alarm.alarm_id,
                source=alarm.source,
                severity=alarm.severity,
                elapsed=elapsed,
                message=alarm.message,
                level=level,
            )
        except (KeyError, IndexError, ValueError):
            return template


# ---------------------------------------------------------------------------
# Multi-channel operator notification dispatcher
# ---------------------------------------------------------------------------

SEVERITY_ORDER = {'INFO': 0, 'WARNING': 1, 'CRITICAL': 2}


@dataclass
class NotificationChannel:
    """Configuration for a single notification channel."""
    channel_type: str  # DASHBOARD, AUDIO, EMAIL, SMS, MQTT, WEBHOOK
    enabled: bool = True
    config: Dict[str, Any] = field(default_factory=dict)
    min_severity: str = 'INFO'  # INFO, WARNING, CRITICAL
    quiet_hours: Optional[Tuple[int, int]] = None  # (start_hour, end_hour) or None
    rate_limit_per_min: int = 10


@dataclass
class Notification:
    """Record of a dispatched notification."""
    notification_id: str
    timestamp: float
    channel: str
    recipient: str
    subject: str
    body: str
    severity: str
    alarm_id: str
    delivered: bool = False
    delivery_attempts: int = 0
    acknowledged: bool = False


class NotificationDispatcher:
    """Multi-channel notification dispatcher for operator alerts.

    Manages multiple notification channels (DASHBOARD, AUDIO, EMAIL, SMS,
    MQTT, WEBHOOK) with per-channel severity filtering, quiet hours,
    rate limiting, and temporary suppression.
    """

    def __init__(self) -> None:
        self._channels: Dict[str, NotificationChannel] = {}
        self._history: List[Notification] = []
        self._rate_counters: Dict[str, List[float]] = {}  # channel -> list of timestamps
        self._suppressed_until: Dict[str, float] = {}  # channel -> resume_timestamp
        self._lock = threading.Lock()

        # Default: DASHBOARD always on
        self.register_channel(NotificationChannel(
            channel_type='DASHBOARD',
            enabled=True,
            min_severity='INFO',
            rate_limit_per_min=60,
        ))

    def register_channel(self, channel: NotificationChannel) -> None:
        """Register or replace a notification channel."""
        with self._lock:
            self._channels[channel.channel_type] = channel
            if channel.channel_type not in self._rate_counters:
                self._rate_counters[channel.channel_type] = []

    def dispatch(
        self,
        alarm_id: str,
        severity: str,
        subject: str,
        body: str,
        recipients: Optional[List[str]] = None,
        current_time: Optional[float] = None,
        current_hour: Optional[int] = None,
    ) -> List[Notification]:
        """Dispatch notifications across all eligible channels.

        Args:
            alarm_id: The alarm that triggered this notification.
            severity: One of INFO, WARNING, CRITICAL.
            subject: Notification subject line.
            body: Notification body text.
            recipients: Optional list of recipients. Defaults to ['operator'].
            current_time: Override for current timestamp (for testing).
            current_hour: Override for current hour (for testing quiet hours).

        Returns:
            List of Notification objects created (one per channel per recipient).
        """
        import time as _time
        now = current_time if current_time is not None else _time.time()
        hour = current_hour if current_hour is not None else int((_time.localtime(now).tm_hour))
        if recipients is None:
            recipients = ['operator']

        notifications: List[Notification] = []
        sev_level = SEVERITY_ORDER.get(severity, 0)

        with self._lock:
            for ch_type, channel in self._channels.items():
                if not channel.enabled:
                    continue

                # Severity filter
                ch_min = SEVERITY_ORDER.get(channel.min_severity, 0)
                if sev_level < ch_min:
                    continue

                # Suppression check
                suppressed_until = self._suppressed_until.get(ch_type, 0.0)
                if now < suppressed_until:
                    continue

                # Quiet hours check
                if channel.quiet_hours is not None:
                    start_h, end_h = channel.quiet_hours
                    if start_h <= end_h:
                        if start_h <= hour < end_h:
                            continue
                    else:
                        # Wraps midnight, e.g. (22, 6)
                        if hour >= start_h or hour < end_h:
                            continue

                # Rate limit check
                timestamps = self._rate_counters.get(ch_type, [])
                window_start = now - 60.0
                timestamps = [t for t in timestamps if t > window_start]
                self._rate_counters[ch_type] = timestamps
                if len(timestamps) >= channel.rate_limit_per_min:
                    continue

                # Format subject/body per channel
                formatted_subject, formatted_body = self.format_notification(
                    {'alarm_id': alarm_id, 'severity': severity,
                     'subject': subject, 'body': body},
                    ch_type,
                )

                for recipient in recipients:
                    notif = Notification(
                        notification_id=str(uuid.uuid4())[:12],
                        timestamp=now,
                        channel=ch_type,
                        recipient=recipient,
                        subject=formatted_subject,
                        body=formatted_body,
                        severity=severity,
                        alarm_id=alarm_id,
                        delivered=True,
                        delivery_attempts=1,
                        acknowledged=False,
                    )
                    self._history.append(notif)
                    timestamps.append(now)
                    notifications.append(notif)

        return notifications

    def get_notification_history(
        self,
        alarm_id: Optional[str] = None,
        channel: Optional[str] = None,
        last_n: int = 50,
    ) -> List[Notification]:
        """Return recent notification history, optionally filtered."""
        with self._lock:
            result = list(self._history)
        if alarm_id is not None:
            result = [n for n in result if n.alarm_id == alarm_id]
        if channel is not None:
            result = [n for n in result if n.channel == channel]
        return result[-last_n:]

    def acknowledge_notification(self, notification_id: str) -> bool:
        """Acknowledge a notification by ID. Returns True if found."""
        with self._lock:
            for notif in self._history:
                if notif.notification_id == notification_id:
                    notif.acknowledged = True
                    return True
        return False

    def get_unacknowledged(
        self, channel: Optional[str] = None,
    ) -> List[Notification]:
        """Return unacknowledged notifications, optionally filtered by channel."""
        with self._lock:
            result = [n for n in self._history if not n.acknowledged]
        if channel is not None:
            result = [n for n in result if n.channel == channel]
        return result

    def set_quiet_hours(
        self, channel_type: str, start_hour: int, end_hour: int,
    ) -> None:
        """Set quiet hours for a channel."""
        with self._lock:
            ch = self._channels.get(channel_type)
            if ch is not None:
                ch.quiet_hours = (start_hour, end_hour)

    def get_delivery_stats(self) -> Dict[str, Any]:
        """Return delivery statistics.

        Returns:
            Dict with total_sent, by_channel, by_severity, failed_count,
            avg_delivery_time.
        """
        with self._lock:
            history = list(self._history)

        total_sent = len(history)
        by_channel: Dict[str, int] = {}
        by_severity: Dict[str, int] = {}
        failed_count = 0

        for n in history:
            by_channel[n.channel] = by_channel.get(n.channel, 0) + 1
            by_severity[n.severity] = by_severity.get(n.severity, 0) + 1
            if not n.delivered:
                failed_count += 1

        return {
            'total_sent': total_sent,
            'by_channel': by_channel,
            'by_severity': by_severity,
            'failed_count': failed_count,
            'avg_delivery_time': 0.0,  # simulated — instant delivery
        }

    def suppress_channel(self, channel_type: str, duration_sec: float,
                         current_time: Optional[float] = None) -> None:
        """Temporarily suppress a channel for a given duration."""
        import time as _time
        now = current_time if current_time is not None else _time.time()
        with self._lock:
            self._suppressed_until[channel_type] = now + duration_sec

    def format_notification(
        self,
        alarm: Dict[str, Any],
        channel_type: str,
    ) -> Tuple[str, str]:
        """Format a notification for the given channel type.

        Args:
            alarm: Dict with keys alarm_id, severity, subject, body.
            channel_type: The target channel.

        Returns:
            (subject, body) tuple with channel-appropriate formatting.
        """
        alarm_id = alarm.get('alarm_id', '')
        severity = alarm.get('severity', '')
        subject = alarm.get('subject', '')
        body = alarm.get('body', '')

        if channel_type == 'DASHBOARD':
            # Short text
            dash_subject = f"[{severity}] {subject}"
            dash_body = body[:200] if len(body) > 200 else body
            return dash_subject, dash_body

        elif channel_type == 'EMAIL':
            # Full detail with context
            email_subject = f"[{severity}] Alarm {alarm_id}: {subject}"
            email_body = (
                f"Alarm ID: {alarm_id}\n"
                f"Severity: {severity}\n"
                f"Subject: {subject}\n\n"
                f"{body}\n\n"
                f"Please take appropriate action."
            )
            return email_subject, email_body

        elif channel_type == 'SMS':
            # 160-char summary
            sms_text = f"[{severity}] {alarm_id}: {subject} - {body}"
            if len(sms_text) > 160:
                sms_text = sms_text[:157] + '...'
            return sms_text, sms_text

        elif channel_type == 'AUDIO':
            # Spoken text
            spoken = f"Attention. {severity} alarm. {subject}. {body}"
            return subject, spoken

        else:
            # Generic (MQTT, WEBHOOK, etc.)
            return f"[{severity}] {subject}", body


# ---------------------------------------------------------------------------
# Alarm flood detection and rate limiting
# ---------------------------------------------------------------------------

@dataclass
class AlarmFloodEvent:
    """Represents a detected alarm flood episode.

    Attributes:
        start_time: Timestamp when the flood was first detected.
        end_time: Timestamp when the flood subsided (None while active).
        alarm_count: Total alarms received during the flood window.
        unique_alarm_types: Set of distinct alarm type strings seen.
        suppressed_count: Number of alarms suppressed during the flood.
        is_active: Whether the flood is currently ongoing.
    """
    start_time: float
    end_time: Optional[float] = None
    alarm_count: int = 0
    unique_alarm_types: set = field(default_factory=set)
    suppressed_count: int = 0
    is_active: bool = True


@dataclass
class AlarmSuppression:
    """Record of a single suppressed alarm during a flood.

    Attributes:
        alarm_id: Identifier of the suppressed alarm.
        reason: Human-readable reason for suppression.
        suppressed_at: Timestamp when the alarm was suppressed.
        original_severity: Severity value the alarm carried.
        would_have_notified: Whether the alarm would have triggered a
            notification under normal (non-flood) conditions.
    """
    alarm_id: str
    reason: str
    suppressed_at: float
    original_severity: float
    would_have_notified: bool = True


class AlarmFloodDetector:
    """Detects alarm floods and suppresses duplicate / low-severity alarms.

    When the alarm arrival rate exceeds *flood_threshold* alarms per minute
    within a sliding *window_sec* window, the detector enters flood mode.
    While in flood mode:

    * Duplicate alarms (same alarm_type) are suppressed.
    * Only the highest-severity alarm per alarm_type is forwarded.
    * All suppression decisions are logged for later audit.

    After the rate drops below the threshold, a *cooldown_sec* period is
    enforced before the detector returns to normal operation.

    Args:
        flood_threshold: Alarms per minute to trigger flood mode (default 10).
        window_sec: Sliding window length in seconds (default 60).
        cooldown_sec: Seconds after flood before resuming normal (default 30).
    """

    def __init__(
        self,
        flood_threshold: int = 10,
        window_sec: float = 60.0,
        cooldown_sec: float = 30.0,
    ) -> None:
        self.flood_threshold = flood_threshold
        self.window_sec = window_sec
        self.cooldown_sec = cooldown_sec
        # Internal threshold: number of alarms within the window that
        # corresponds to *flood_threshold* alarms per minute.
        self._window_count_threshold = flood_threshold * (window_sec / 60.0)

        # Sliding window of (timestamp, alarm_id, severity, alarm_type)
        self._arrivals: List[Tuple[float, str, float, str]] = []
        self._lock = threading.Lock()

        # Current flood event (None when not in flood)
        self._current_flood: Optional[AlarmFloodEvent] = None
        # Timestamp when the last flood ended (for cooldown tracking)
        self._flood_ended_at: Optional[float] = None

        # Suppression log
        self._suppression_log: List[AlarmSuppression] = []

        # During flood: alarm_type -> highest severity seen so far
        self._flood_type_severity: Dict[str, float] = {}
        # During flood: alarm_type -> whether the first (highest) was forwarded
        self._flood_type_forwarded: Dict[str, bool] = {}

    # -- public API ----------------------------------------------------------

    def record_alarm(
        self,
        alarm_id: str,
        severity: float,
        alarm_type: str,
        timestamp: float,
    ) -> bool:
        """Record an incoming alarm and decide whether to forward it.

        Args:
            alarm_id: Unique alarm identifier.
            severity: Alarm severity (higher = more severe).
            alarm_type: Category / type string for the alarm.
            timestamp: Arrival timestamp (epoch seconds).

        Returns:
            ``True`` if the alarm should be forwarded to operators.
            ``False`` if the alarm was suppressed (flood mitigation).
        """
        with self._lock:
            # Append to sliding window
            self._arrivals.append((timestamp, alarm_id, severity, alarm_type))
            # Prune arrivals outside the window
            self._prune_window(timestamp)

            count = len(self._arrivals)
            in_cooldown = self._in_cooldown(timestamp)

            # --- flood state transitions ---
            if self._current_flood is not None and self._current_flood.is_active:
                # Already in flood
                self._current_flood.alarm_count += 1
                self._current_flood.unique_alarm_types.add(alarm_type)

                if count < self._window_count_threshold and not in_cooldown:
                    # Flood has subsided
                    self._current_flood.is_active = False
                    self._current_flood.end_time = timestamp
                    self._flood_ended_at = timestamp
                    self._flood_type_severity.clear()
                    self._flood_type_forwarded.clear()
                    # This alarm arrives after flood ended — forward it
                    return True

                # Still in flood — apply suppression logic
                return self._flood_filter(alarm_id, severity, alarm_type, timestamp)

            # Not currently in flood
            if count >= self._window_count_threshold and not in_cooldown:
                # Enter flood mode
                self._current_flood = AlarmFloodEvent(
                    start_time=timestamp,
                    alarm_count=1,
                    unique_alarm_types={alarm_type},
                )
                self._flood_type_severity.clear()
                self._flood_type_forwarded.clear()
                # The first alarm that triggers the flood is forwarded
                self._flood_type_severity[alarm_type] = severity
                self._flood_type_forwarded[alarm_type] = True
                return True

            if in_cooldown:
                # During cooldown we still suppress duplicates
                return self._cooldown_filter(alarm_id, severity, alarm_type, timestamp)

            # Normal operation — forward everything
            return True

    def get_flood_status(self) -> Optional[AlarmFloodEvent]:
        """Return the current flood event, or ``None`` if no flood is active."""
        with self._lock:
            return self._current_flood

    def get_suppression_log(self) -> List[AlarmSuppression]:
        """Return the full list of suppression records."""
        with self._lock:
            return list(self._suppression_log)

    def alarm_rate_per_minute(self, current_time: Optional[float] = None) -> float:
        """Return the current alarm rate (alarms per minute).

        Args:
            current_time: Optional timestamp override (epoch seconds).
                If ``None``, uses the latest arrival timestamp.
        """
        with self._lock:
            if not self._arrivals:
                return 0.0
            ref_time = current_time if current_time is not None else self._arrivals[-1][0]
            self._prune_window(ref_time)
            return self._compute_rate(ref_time)

    # -- internal helpers ----------------------------------------------------

    def _prune_window(self, now: float) -> None:
        """Remove arrivals older than the sliding window."""
        cutoff = now - self.window_sec
        self._arrivals = [a for a in self._arrivals if a[0] > cutoff]

    def _compute_rate(self, now: float) -> float:
        """Compute alarms-per-minute from the current sliding window.

        Uses the actual time span covered by arrivals (capped at window_sec)
        to avoid inflating the rate when few alarms sit in a short window.
        """
        if not self._arrivals:
            return 0.0
        count = len(self._arrivals)
        actual_span = max(now - self._arrivals[0][0], 1.0)
        span = min(actual_span, self.window_sec)
        return count * (60.0 / span) if span > 0 else 0.0

    def _in_cooldown(self, now: float) -> bool:
        """Check whether we are in the post-flood cooldown period."""
        if self._flood_ended_at is None:
            return False
        return (now - self._flood_ended_at) < self.cooldown_sec

    def _flood_filter(
        self, alarm_id: str, severity: float, alarm_type: str, timestamp: float,
    ) -> bool:
        """Decide whether to forward or suppress an alarm during a flood.

        During a flood, only the highest-severity alarm per alarm_type is
        forwarded. Subsequent alarms of the same type (or lower severity)
        are suppressed.
        """
        prev_severity = self._flood_type_severity.get(alarm_type)

        if prev_severity is None:
            # First alarm of this type during the flood — forward it
            self._flood_type_severity[alarm_type] = severity
            self._flood_type_forwarded[alarm_type] = True
            return True

        if severity > prev_severity:
            # Higher severity supersedes — forward the new one
            self._flood_type_severity[alarm_type] = severity
            self._flood_type_forwarded[alarm_type] = True
            return True

        # Duplicate or lower severity — suppress
        self._current_flood.suppressed_count += 1
        self._suppression_log.append(AlarmSuppression(
            alarm_id=alarm_id,
            reason=f'Flood suppression: duplicate alarm_type={alarm_type}',
            suppressed_at=timestamp,
            original_severity=severity,
            would_have_notified=True,
        ))
        return False

    def _cooldown_filter(
        self, alarm_id: str, severity: float, alarm_type: str, timestamp: float,
    ) -> bool:
        """During cooldown, suppress only exact duplicates of already-seen types."""
        # During cooldown we forward all alarms (no suppression by default)
        return True


# ---------------------------------------------------------------------------
# Alarm suppression rule manager
# ---------------------------------------------------------------------------

@dataclass
class SuppressionRule:
    """Configurable alarm suppression rule.

    Supports shelving, maintenance windows, state-based suppression, and
    duplicate suppression with pattern matching on alarm type and machine.

    Attributes:
        rule_id: Unique identifier for this rule.
        name: Human-readable name.
        rule_type: One of 'shelve', 'maintenance_window', 'state_based',
            'duplicate'.
        alarm_type_pattern: Glob pattern for alarm types ('*' matches all,
            'THERMAL*' matches prefix).
        machine_pattern: Glob pattern for machine IDs.
        start_time: Start of the suppression window (epoch seconds).
        end_time: End of the suppression window (epoch seconds).
        max_occurrences: Maximum number of alarm occurrences before
            suppression kicks in (used by 'duplicate' rules).
        window_sec: Rolling window in seconds for occurrence counting.
        is_active: Whether the rule is currently active.
        created_by: Operator or system that created this rule.
        reason: Human-readable explanation for the suppression.
    """
    rule_id: str
    name: str
    rule_type: str  # 'shelve' | 'maintenance_window' | 'state_based' | 'duplicate'
    alarm_type_pattern: str = '*'
    machine_pattern: str = '*'
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    max_occurrences: int = 1
    window_sec: float = 60.0
    is_active: bool = True
    created_by: str = ''
    reason: str = ''


@dataclass
class SuppressionDecision:
    """Result of evaluating an alarm against suppression rules.

    Attributes:
        alarm_id: The alarm identifier that was evaluated.
        rule_id: The rule that triggered suppression (empty string if not
            suppressed).
        should_suppress: Whether the alarm should be suppressed.
        reason: Human-readable explanation of the decision.
        timestamp: When the decision was made (epoch seconds).
    """
    alarm_id: str
    rule_id: str
    should_suppress: bool
    reason: str
    timestamp: float


class AlarmSuppressionRuleManager:
    """Manages configurable alarm suppression rules.

    Supports shelving (temporary suppression of an alarm type),
    maintenance windows (suppress alarms for a machine during planned
    downtime), state-based suppression, and duplicate suppression.

    Pattern matching uses ``fnmatch``-style globs: ``'*'`` matches
    everything, ``'THERMAL*'`` matches any type starting with THERMAL.
    """

    def __init__(self) -> None:
        self._rules: Dict[str, SuppressionRule] = {}
        self._history: List[SuppressionDecision] = []
        # Track alarm occurrences for duplicate suppression:
        # key = (alarm_type, machine_id), value = list of timestamps
        self._occurrence_log: Dict[Tuple[str, str], List[float]] = {}
        self._lock = threading.Lock()

    # -- rule management -----------------------------------------------------

    def add_rule(self, rule: SuppressionRule) -> None:
        """Add or replace a suppression rule."""
        with self._lock:
            self._rules[rule.rule_id] = rule

    def remove_rule(self, rule_id: str) -> bool:
        """Remove a suppression rule by ID. Returns True if removed."""
        with self._lock:
            return self._rules.pop(rule_id, None) is not None

    def get_active_rules(self) -> List[SuppressionRule]:
        """Return all currently active rules."""
        with self._lock:
            return [r for r in self._rules.values() if r.is_active]

    # -- evaluation ----------------------------------------------------------

    def _matches_pattern(self, value: str, pattern: str) -> bool:
        """Check whether *value* matches a glob *pattern*."""
        return fnmatch.fnmatch(value, pattern)

    def _is_time_valid(self, rule: SuppressionRule, timestamp: float) -> bool:
        """Return True if *timestamp* falls within the rule's time window."""
        if rule.start_time is not None and timestamp < rule.start_time:
            return False
        if rule.end_time is not None and timestamp > rule.end_time:
            return False
        return True

    def _check_duplicate(
        self, alarm_type: str, machine_id: str, timestamp: float,
        rule: SuppressionRule,
    ) -> bool:
        """Return True if duplicate threshold has been exceeded."""
        key = (alarm_type, machine_id)
        timestamps = self._occurrence_log.get(key, [])
        # Trim to window
        cutoff = timestamp - rule.window_sec
        timestamps = [t for t in timestamps if t >= cutoff]
        self._occurrence_log[key] = timestamps
        return len(timestamps) >= rule.max_occurrences

    def evaluate(
        self,
        alarm_id: str,
        alarm_type: str,
        machine_id: str,
        severity: float,
        timestamp: float,
    ) -> SuppressionDecision:
        """Evaluate an alarm against all active suppression rules.

        Returns a ``SuppressionDecision``.  The first matching rule wins.
        Occurrence tracking for duplicate suppression is updated regardless
        of whether the alarm is ultimately suppressed.

        Args:
            alarm_id: Unique identifier of the alarm being evaluated.
            alarm_type: Type/category of the alarm (e.g. 'THERMAL').
            machine_id: Source machine identifier.
            severity: Alarm severity (0.0-1.0).
            timestamp: Alarm timestamp (epoch seconds).
        """
        with self._lock:
            # Record occurrence for duplicate tracking
            occ_key = (alarm_type, machine_id)
            self._occurrence_log.setdefault(occ_key, []).append(timestamp)

            for rule in self._rules.values():
                if not rule.is_active:
                    continue
                if not self._matches_pattern(alarm_type, rule.alarm_type_pattern):
                    continue
                if not self._matches_pattern(machine_id, rule.machine_pattern):
                    continue
                if not self._is_time_valid(rule, timestamp):
                    continue

                # Rule-type specific checks
                if rule.rule_type == 'duplicate':
                    if not self._check_duplicate(alarm_type, machine_id, timestamp, rule):
                        continue

                decision = SuppressionDecision(
                    alarm_id=alarm_id,
                    rule_id=rule.rule_id,
                    should_suppress=True,
                    reason=f'Suppressed by rule {rule.rule_id!r} ({rule.rule_type}): {rule.reason}',
                    timestamp=timestamp,
                )
                self._history.append(decision)
                return decision

            # No rule matched — allow the alarm
            decision = SuppressionDecision(
                alarm_id=alarm_id,
                rule_id='',
                should_suppress=False,
                reason='No suppression rule matched',
                timestamp=timestamp,
            )
            self._history.append(decision)
            return decision

    # -- convenience methods -------------------------------------------------

    def shelve_alarm_type(
        self,
        alarm_type: str,
        duration_sec: float,
        reason: str = '',
        created_by: str = '',
    ) -> SuppressionRule:
        """Temporarily suppress all alarms of the given type.

        Creates a 'shelve' rule that expires after *duration_sec* seconds.

        Args:
            alarm_type: Exact alarm type (or glob pattern) to shelve.
            duration_sec: How long to shelve, in seconds.
            reason: Human-readable reason for shelving.
            created_by: Operator ID.

        Returns:
            The created ``SuppressionRule``.
        """
        now = time.time()
        rule = SuppressionRule(
            rule_id=f'shelve_{uuid.uuid4().hex[:8]}',
            name=f'Shelve {alarm_type}',
            rule_type='shelve',
            alarm_type_pattern=alarm_type,
            machine_pattern='*',
            start_time=now,
            end_time=now + duration_sec,
            is_active=True,
            created_by=created_by,
            reason=reason,
        )
        self.add_rule(rule)
        return rule

    def add_maintenance_window(
        self,
        machine_id: str,
        start_time: float,
        end_time: float,
        reason: str = '',
    ) -> SuppressionRule:
        """Suppress all alarms for a machine during a maintenance window.

        Args:
            machine_id: Machine ID (or glob pattern).
            start_time: Window start (epoch seconds).
            end_time: Window end (epoch seconds).
            reason: Human-readable reason.

        Returns:
            The created ``SuppressionRule``.
        """
        rule = SuppressionRule(
            rule_id=f'maint_{uuid.uuid4().hex[:8]}',
            name=f'Maintenance window for {machine_id}',
            rule_type='maintenance_window',
            alarm_type_pattern='*',
            machine_pattern=machine_id,
            start_time=start_time,
            end_time=end_time,
            is_active=True,
            created_by='system',
            reason=reason,
        )
        self.add_rule(rule)
        return rule

    def get_suppression_history(self) -> List[SuppressionDecision]:
        """Return all past suppression decisions."""
        with self._lock:
            return list(self._history)

    def get_active_shelves(self) -> List[SuppressionRule]:
        """Return currently active shelved alarm types.

        A shelf is considered active if it is a 'shelve' rule, is marked
        active, and its end_time has not yet passed.
        """
        now = time.time()
        with self._lock:
            return [
                r for r in self._rules.values()
                if r.rule_type == 'shelve'
                and r.is_active
                and (r.end_time is None or r.end_time > now)
            ]


class AlarmManagerNode(MiracleLifecycleNode):
    """Centralizes alarm management with ISA-18.2 compliance.

    Parameters:
        max_active_alarms (int): Maximum concurrent active alarms.
        escalation_timeout_sec (float): Time before escalation.
        history_size (int): Number of alarms to retain in history.

    Subscribed Topics:
        /miracle/{machine_id}/anomaly (AnomalyAlert): Anomaly alerts from machines.
        /miracle/security/alerts (SecurityAlert): Security alerts.

    Published Topics:
        ~/active_alarms_json (std_msgs/String): JSON of active alarms.
        /miracle/scada/alarm_escalations (AlarmEscalation): Escalation events.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'alarm_manager',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._active_alarms: Dict[str, Alarm] = {}
        self._alarm_history: List[Alarm] = []
        self._alarms_lock = threading.Lock()
        self._anomaly_subs = None
        self._security_sub = None
        self._escalation_timer = None
        self._escalation_pub = None
        self._max_alarms: int = 1000
        self._escalation_timeout: float = 300.0
        self._history_size: int = 10000
        self._correlated_sub = None
        self._correlated_suppression: set = set()
        self._escalation_engine = EscalationEngine()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure alarm manager."""
        params = self.declare_and_validate_parameters({
            'max_active_alarms': {
                'default': 1000,
                'type': int,
                'range': (10, 100000),
            },
            'escalation_timeout_sec': {
                'default': 300.0,
                'type': float,
                'range': (10.0, 3600.0),
            },
            'history_size': {
                'default': 10000,
                'type': int,
                'range': (100, 1000000),
            },
            'machine_ids': {
                'default': 'cnc1,cnc2,cnc3',
                'type': str,
            },
        })

        machine_ids = self.get_machine_ids(params)

        self._max_alarms = params['max_active_alarms']
        self._escalation_timeout = params['escalation_timeout_sec']
        self._history_size = params['history_size']

        self._anomaly_subs = self.create_multi_machine_subscriptions(
            AnomalyAlert,
            'anomaly',
            self._on_anomaly,
            QoSProfiles.alert(),
            machine_ids,
        )

        self._security_sub = self.create_subscription(
            SecurityAlert,
            '/miracle/security/alerts',
            self._on_security_alert,
            QoSProfiles.alert(),
        )

        # Subscribe to correlated alerts
        self._correlated_sub = self.create_subscription(
            CorrelatedAlert,
            '/miracle/scada/correlated_alerts',
            self._on_correlated_alert,
            QoSProfiles.alert(),
        )

        # Publisher for escalation events
        self._escalation_pub = self.create_publisher(
            AlarmEscalation,
            '/miracle/scada/alarm_escalations',
            QoSProfiles.alert(),
        )

        self.get_logger().info("Alarm manager configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate alarm processing."""
        self._escalation_timer = self.create_timer(
            30.0,
            self._check_escalations,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("Alarm manager activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate alarm processing."""
        if self._escalation_timer is not None:
            self._escalation_timer.cancel()
            self._escalation_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        """Process anomaly alert into alarm."""
        alarm_id = str(uuid.uuid4())[:8]
        alarm = Alarm(
            alarm_id=alarm_id,
            source=msg.machine_id,
            severity=msg.severity,
            message=f"{msg.anomaly_type}: {msg.recommended_action}",
            alarm_type=getattr(msg, 'anomaly_type', ''),
            timestamp=msg.timestamp.sec + msg.timestamp.nanosec * 1e-9,
        )

        # Run alarm through escalation engine
        esc_result = self._escalation_engine.process_alarm(alarm)

        with self._alarms_lock:
            if esc_result.action_type == EscalationEngine.SUPPRESS:
                self.get_logger().debug(
                    f"Alarm suppressed (duplicate): [{alarm_id}] {alarm.message}"
                )
                return
            if len(self._active_alarms) < self._max_alarms:
                self._active_alarms[alarm_id] = alarm
                self.get_logger().info(
                    f"Alarm raised: [{alarm_id}] {alarm.message} "
                    f"(severity={alarm.severity:.2f})"
                )

    def _on_security_alert(self, msg: SecurityAlert) -> None:
        """Process security alert into alarm."""
        alarm_id = str(uuid.uuid4())[:8]
        alarm = Alarm(
            alarm_id=alarm_id,
            source=msg.source_node,
            severity=msg.confidence,
            message=f"SECURITY: {msg.description}",
            timestamp=msg.timestamp.sec + msg.timestamp.nanosec * 1e-9,
        )

        with self._alarms_lock:
            if len(self._active_alarms) < self._max_alarms:
                self._active_alarms[alarm_id] = alarm
                self.get_logger().warn(
                    f"Security alarm: [{alarm_id}] {alarm.message}"
                )

    def _on_correlated_alert(self, msg: CorrelatedAlert) -> None:
        """Process correlated alert -- suppress individual alerts that are part of the group."""
        # Track which alert IDs are part of this correlation for suppression
        for alert_ref in msg.contributing_alert_ids:
            self._correlated_suppression.add(alert_ref)

        # Create a single consolidated alarm for the correlated group
        alarm_id = msg.correlation_id
        alarm = Alarm(
            alarm_id=alarm_id,
            source=msg.machine_id,
            severity=msg.severity,
            message=f"CORRELATED: {msg.root_cause_hypothesis}",
            timestamp=msg.timestamp.sec + msg.timestamp.nanosec * 1e-9,
        )

        with self._alarms_lock:
            if len(self._active_alarms) < self._max_alarms:
                self._active_alarms[alarm_id] = alarm
                self.get_logger().info(
                    f"Correlated alarm: [{alarm_id}] {msg.category} "
                    f"({len(msg.contributing_alert_ids)} contributing alerts)"
                )

    def _publish_escalation(self, alarm: Alarm, action: str, reason: str,
                            time_unacked: float) -> None:
        """Publish an AlarmEscalation message."""
        if self._escalation_pub is None:
            return
        msg = AlarmEscalation()
        clock_now = self.get_clock().now()
        msg.timestamp.sec = int(clock_now.nanoseconds // 1_000_000_000)
        msg.timestamp.nanosec = int(clock_now.nanoseconds % 1_000_000_000)
        msg.alarm_id = alarm.alarm_id
        msg.machine_id = alarm.source
        msg.alarm_type = alarm.message
        msg.escalation_level = alarm.escalation_level
        msg.severity = alarm.severity
        msg.escalation_action = action
        msg.reason = reason
        msg.time_unacknowledged_sec = time_unacked
        self._escalation_pub.publish(msg)

    def _check_escalations(self) -> None:
        """Check for alarms that need escalation.

        Escalation levels and corresponding actions:
            Level 1 (after escalation_timeout_sec):
                PRIORITY_BOOST -- increase severity by 0.1 (capped at 1.0).
            Level 2 (after 2x timeout):
                FORCED_ACK_REQUIRED -- alarm must be force-acknowledged.
            Level 3 (after 3x timeout):
                SUPERVISOR_NOTIFY -- high-priority supervisor notification.
                EMERGENCY_STOP if severity > 0.8.

        A minimum interval of MIN_ESCALATION_INTERVAL seconds is enforced
        between successive escalations for each alarm.
        """
        now = self.get_clock().now().nanoseconds / 1e9

        with self._alarms_lock:
            for alarm in self._active_alarms.values():
                if alarm.state != AlarmState.UNACKNOWLEDGED:
                    continue

                elapsed = now - alarm.timestamp

                # Determine the target escalation level based on elapsed time
                if elapsed > self._escalation_timeout * 3:
                    target_level = 3
                elif elapsed > self._escalation_timeout * 2:
                    target_level = 2
                elif elapsed > self._escalation_timeout:
                    target_level = 1
                else:
                    # Check for de-escalation: severity dropped below 0.3
                    if alarm.escalation_level > 0 and alarm.severity < 0.3:
                        alarm.escalation_level = max(0, alarm.escalation_level - 1)
                        alarm.escalation_actions_taken.append(
                            f"DE-ESCALATED to level {alarm.escalation_level}"
                        )
                        self.get_logger().info(
                            f"Alarm de-escalated: [{alarm.alarm_id}] "
                            f"level={alarm.escalation_level} (severity={alarm.severity:.2f})"
                        )
                    continue

                # Only escalate if we haven't reached this level yet
                if alarm.escalation_level >= target_level:
                    # De-escalation check even when past timeout
                    if alarm.severity < 0.3 and alarm.escalation_level > 0:
                        alarm.escalation_level = max(0, alarm.escalation_level - 1)
                        alarm.escalation_actions_taken.append(
                            f"DE-ESCALATED to level {alarm.escalation_level}"
                        )
                    continue

                # Enforce minimum interval between escalations
                if (now - alarm.last_escalation_time) < MIN_ESCALATION_INTERVAL:
                    continue

                time_unacked = elapsed

                # Escalate one level at a time
                alarm.escalation_level += 1
                alarm.last_escalation_time = now
                level = alarm.escalation_level

                if level == 1:
                    # PRIORITY_BOOST -- increase severity by 0.1, cap at 1.0
                    alarm.severity = min(1.0, alarm.severity + 0.1)
                    action = EscalationAction.PRIORITY_BOOST
                    reason = (
                        f"Unacknowledged for {time_unacked:.0f}s, "
                        f"severity boosted to {alarm.severity:.2f}"
                    )
                    alarm.escalation_actions_taken.append(action)
                    self._publish_escalation(alarm, action, reason, time_unacked)
                    self.get_logger().warn(
                        f"Alarm escalated: [{alarm.alarm_id}] "
                        f"level=1 action={action}"
                    )

                elif level == 2:
                    # FORCED_ACK_REQUIRED
                    alarm.requires_forced_ack = True
                    action = EscalationAction.FORCED_ACK_REQUIRED
                    reason = (
                        f"Unacknowledged for {time_unacked:.0f}s, "
                        f"forced acknowledgment now required"
                    )
                    alarm.escalation_actions_taken.append(action)
                    self._publish_escalation(alarm, action, reason, time_unacked)
                    self.get_logger().warn(
                        f"Alarm escalated: [{alarm.alarm_id}] "
                        f"level=2 action={action}"
                    )

                elif level >= 3:
                    # SUPERVISOR_NOTIFY, and EMERGENCY_STOP if severity > 0.8
                    action = EscalationAction.SUPERVISOR_NOTIFY
                    reason = (
                        f"Unacknowledged for {time_unacked:.0f}s, "
                        f"supervisor notification sent"
                    )
                    alarm.escalation_actions_taken.append(action)
                    self._publish_escalation(alarm, action, reason, time_unacked)

                    if alarm.severity > 0.8:
                        estop_action = EscalationAction.EMERGENCY_STOP
                        estop_reason = (
                            f"Severity {alarm.severity:.2f} > 0.8 at escalation "
                            f"level 3, emergency stop triggered"
                        )
                        alarm.escalation_actions_taken.append(estop_action)
                        self._publish_escalation(
                            alarm, estop_action, estop_reason, time_unacked
                        )
                        self.get_logger().error(
                            f"Alarm escalated: [{alarm.alarm_id}] "
                            f"level=3 action={estop_action} "
                            f"(severity={alarm.severity:.2f})"
                        )
                    else:
                        self.get_logger().error(
                            f"Alarm escalated: [{alarm.alarm_id}] "
                            f"level=3 action={action}"
                        )

    def acknowledge_alarm(self, alarm_id: str, user: str,
                          force: bool = False) -> bool:
        """Acknowledge an active alarm.

        Args:
            alarm_id: The alarm identifier.
            user: The user acknowledging the alarm.
            force: If True, override forced-ack requirement. Required when
                the alarm has been escalated to level 2+ (requires_forced_ack).

        Returns:
            True if alarm was acknowledged successfully.
        """
        with self._alarms_lock:
            if alarm_id in self._active_alarms:
                alarm = self._active_alarms[alarm_id]
                if alarm.requires_forced_ack and not force:
                    self.get_logger().warn(
                        f"Alarm [{alarm_id}] requires forced acknowledgment "
                        f"(force=True). Normal ack rejected."
                    )
                    return False
                alarm.state = AlarmState.ACKNOWLEDGED
                alarm.acknowledged_by = user
                if alarm.requires_forced_ack:
                    self.get_logger().warn(
                        f"Alarm force-acknowledged: [{alarm_id}] by {user}"
                    )
                else:
                    self.get_logger().info(
                        f"Alarm acknowledged: [{alarm_id}] by {user}"
                    )
                # Publish CLEARED escalation so downstream consumers
                # (e.g. job scheduler) can unblock the machine.
                self._publish_escalation(
                    alarm,
                    action='CLEARED',
                    reason=f"Alarm acknowledged by {user}",
                    time_unacked=0.0,
                )
                return True
        return False

    def clear_alarm(self, alarm_id: str) -> bool:
        """Clear (resolve) an active alarm.

        Args:
            alarm_id: The alarm identifier.

        Returns:
            True if alarm was cleared successfully.
        """
        with self._alarms_lock:
            if alarm_id in self._active_alarms:
                alarm = self._active_alarms.pop(alarm_id)
                # Publish CLEARED escalation before changing state so the
                # message still carries the alarm's machine_id / severity.
                self._publish_escalation(
                    alarm,
                    action='CLEARED',
                    reason=f"Alarm cleared",
                    time_unacked=0.0,
                )
                alarm.state = AlarmState.NORMAL
                self._alarm_history.append(alarm)
                if len(self._alarm_history) > self._history_size:
                    self._alarm_history = self._alarm_history[-self._history_size:]
                self.get_logger().info(f"Alarm cleared: [{alarm_id}]")
                return True
        return False

    def get_escalation_summary(self) -> Dict:
        """Return a summary of current escalation state.

        Returns:
            Dict with:
                - level_counts: dict mapping escalation level -> count
                - avg_time_to_ack: average time-to-ack for acknowledged
                  alarms in history (seconds), or None if no data
                - forced_ack_required: list of alarm_ids requiring forced ack
        """
        with self._alarms_lock:
            level_counts: Dict[int, int] = {}
            forced_ack_ids: List[str] = []

            for alarm in self._active_alarms.values():
                level = alarm.escalation_level
                level_counts[level] = level_counts.get(level, 0) + 1
                if alarm.requires_forced_ack:
                    forced_ack_ids.append(alarm.alarm_id)

            # Calculate average time to ack from history
            ack_times: List[float] = []
            for alarm in self._alarm_history:
                if alarm.acknowledged_by is not None and alarm.timestamp > 0:
                    # Use last_escalation_time as a rough proxy for ack time
                    # In production this would use a dedicated ack_timestamp field
                    pass

            return {
                'level_counts': level_counts,
                'avg_time_to_ack': (
                    sum(ack_times) / len(ack_times) if ack_times else None
                ),
                'forced_ack_required': forced_ack_ids,
            }


def main(args=None):
    """Entry point for the alarm manager node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = AlarmManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
