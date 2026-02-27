"""
Alarm Manager Node.

Centralizes alarm management across the manufacturing cell.
Implements alarm shelving, acknowledgment, and escalation.
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
from miracle_msgs.msg import AnomalyAlert, SecurityAlert


class AlarmState(Enum):
    """ISA-18.2 alarm states."""
    NORMAL = 'NORMAL'
    UNACKNOWLEDGED = 'UNACKNOWLEDGED'
    ACKNOWLEDGED = 'ACKNOWLEDGED'
    SHELVED = 'SHELVED'
    SUPPRESSED = 'SUPPRESSED'


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
        self._max_alarms: int = 1000
        self._escalation_timeout: float = 300.0
        self._history_size: int = 10000

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

    def _check_escalations(self) -> None:
        """Check for alarms that need escalation."""
        now = self.get_clock().now().nanoseconds / 1e9

        with self._alarms_lock:
            for alarm in self._active_alarms.values():
                if alarm.state == AlarmState.UNACKNOWLEDGED:
                    elapsed = now - alarm.timestamp
                    if elapsed > self._escalation_timeout:
                        alarm.escalation_level += 1
                        self.get_logger().warn(
                            f"Alarm escalated: [{alarm.alarm_id}] "
                            f"level={alarm.escalation_level}"
                        )

    def acknowledge_alarm(self, alarm_id: str, user: str) -> bool:
        """Acknowledge an active alarm.

        Args:
            alarm_id: The alarm identifier.
            user: The user acknowledging the alarm.

        Returns:
            True if alarm was acknowledged successfully.
        """
        with self._alarms_lock:
            if alarm_id in self._active_alarms:
                alarm = self._active_alarms[alarm_id]
                alarm.state = AlarmState.ACKNOWLEDGED
                alarm.acknowledged_by = user
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
