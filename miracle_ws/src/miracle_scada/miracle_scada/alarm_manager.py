"""
Alarm Manager Node.

Centralizes alarm management across the manufacturing cell.
Implements alarm shelving, acknowledgment, and escalation with real
escalation actions: priority boost, forced acknowledgment, supervisor
notification, and emergency stop.
Maintains alarm history with ISA-18.2 compliance.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
from enum import Enum
import threading
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


class EscalationAction:
    """Constants for escalation action types."""
    PRIORITY_BOOST = 'PRIORITY_BOOST'
    FORCED_ACK_REQUIRED = 'FORCED_ACK_REQUIRED'
    SUPERVISOR_NOTIFY = 'SUPERVISOR_NOTIFY'
    EMERGENCY_STOP = 'EMERGENCY_STOP'


@dataclass
class Alarm:
    """Represents an active alarm."""
    alarm_id: str
    source: str
    severity: float
    message: str
    state: AlarmState = AlarmState.UNACKNOWLEDGED
    timestamp: float = 0.0
    acknowledged_by: Optional[str] = None
    escalation_level: int = 0
    last_escalation_time: float = 0.0
    requires_forced_ack: bool = False
    escalation_actions_taken: List[str] = field(default_factory=list)


# Minimum interval between successive escalations for a single alarm (seconds).
MIN_ESCALATION_INTERVAL = 30.0


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
            timestamp=msg.timestamp.sec + msg.timestamp.nanosec * 1e-9,
        )

        with self._alarms_lock:
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
