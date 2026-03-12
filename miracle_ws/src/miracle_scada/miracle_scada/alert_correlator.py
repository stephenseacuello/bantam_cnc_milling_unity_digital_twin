"""
Alert Correlation Engine.

Groups related alerts by machine_id within a sliding time window and
publishes CorrelatedAlert messages when YAML-configurable correlation rules match.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
import threading
import time
import os
import yaml

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import AnomalyAlert, SecurityAlert, CorrelatedAlert


@dataclass
class AlertEntry:
    """An alert within the correlation window."""
    alert_type: str
    machine_id: str
    severity: float
    message: str
    timestamp: float
    source: str = ''


@dataclass
class CorrelationRule:
    """A YAML-loaded correlation rule."""
    name: str
    required_types: List[str]
    time_window_sec: float
    root_cause: str
    recommended_actions: List[str]
    min_confidence: float = 0.7


class AlertCorrelatorNode(MiracleLifecycleNode):
    """Correlates related alerts from anomaly and security topics.

    Parameters:
        correlation_window_sec (float): Default time window for grouping alerts.
        rules_file (str): Path to YAML correlation rules file.
        machine_ids (str): Comma-separated machine IDs to monitor.

    Subscribed Topics:
        /miracle/{machine_id}/anomaly (AnomalyAlert): Per-machine anomaly alerts.
        /miracle/security/alerts (SecurityAlert): System-wide security alerts.

    Published Topics:
        /miracle/scada/correlated_alerts (CorrelatedAlert): Correlated alert bundles.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'alert_correlator',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._alert_buffer: Dict[str, List[AlertEntry]] = {}
        self._buffer_lock = threading.Lock()
        self._rules: List[CorrelationRule] = []
        self._correlation_window: float = 30.0
        self._correlated_pub = None
        self._check_timer = None
        self._correlation_counter: int = 0
        self._suppressed_ids: set = set()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure alert correlator."""
        params = self.declare_and_validate_parameters({
            'correlation_window_sec': {
                'default': 30.0,
                'type': float,
                'range': (5.0, 300.0),
                'description': 'Default time window for grouping alerts',
            },
            'rules_file': {
                'default': '',
                'type': str,
                'description': 'Path to YAML correlation rules file',
            },
            'machine_ids': {
                'default': 'cnc1,cnc2,cnc3',
                'type': str,
            },
        })

        self._correlation_window = params['correlation_window_sec']
        machine_ids = self.get_machine_ids(params)

        # Load correlation rules
        rules_file = params['rules_file']
        if not rules_file:
            rules_file = os.path.join(
                os.path.dirname(__file__), '..', 'config', 'correlation_rules.yaml'
            )
        self._load_rules(rules_file)

        # Subscribe to anomaly alerts per machine
        self.create_multi_machine_subscriptions(
            AnomalyAlert,
            'anomaly',
            self._on_anomaly,
            QoSProfiles.alert(),
            machine_ids,
        )

        # Subscribe to security alerts
        self.create_subscription(
            SecurityAlert,
            '/miracle/security/alerts',
            self._on_security_alert,
            QoSProfiles.alert(),
        )

        # Publisher for correlated alerts
        self._correlated_pub = self.create_publisher(
            CorrelatedAlert,
            '/miracle/scada/correlated_alerts',
            QoSProfiles.alert(),
        )

        self.get_logger().info(
            f"Alert correlator configured with {len(self._rules)} rules, "
            f"window={self._correlation_window}s"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate correlation checking."""
        self._check_timer = self.create_timer(
            5.0,
            self._check_correlations,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("Alert correlator activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate correlation checking."""
        if self._check_timer is not None:
            self._check_timer.cancel()
            self._check_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _load_rules(self, path: str) -> None:
        """Load correlation rules from YAML file."""
        try:
            if os.path.isfile(path):
                with open(path, 'r') as f:
                    data = yaml.safe_load(f) or {}
                for rule_data in data.get('rules', []):
                    rule = CorrelationRule(
                        name=rule_data['name'],
                        required_types=rule_data['required_types'],
                        time_window_sec=rule_data.get('time_window_sec', self._correlation_window),
                        root_cause=rule_data['root_cause'],
                        recommended_actions=rule_data.get('recommended_actions', []),
                        min_confidence=rule_data.get('min_confidence', 0.7),
                    )
                    self._rules.append(rule)
                self.get_logger().info(f"Loaded {len(self._rules)} correlation rules from {path}")
            else:
                self.get_logger().warn(f"Rules file not found: {path}, using defaults")
                self._load_default_rules()
        except Exception as e:
            self.get_logger().error(f"Failed to load rules: {e}")
            self._load_default_rules()

    def _load_default_rules(self) -> None:
        """Load built-in default correlation rules."""
        self._rules = [
            CorrelationRule(
                name='tool_degradation_chain',
                required_types=['vibration_anomaly', 'force_anomaly'],
                time_window_sec=30.0,
                root_cause='Progressive tool wear causing vibration and force increase',
                recommended_actions=['Inspect tool wear', 'Reduce feed rate', 'Schedule tool change'],
            ),
            CorrelationRule(
                name='thermal_cascade',
                required_types=['thermal_anomaly', 'dimensional_drift'],
                time_window_sec=60.0,
                root_cause='Thermal expansion causing dimensional accuracy loss',
                recommended_actions=['Activate coolant', 'Reduce cutting speed', 'Wait for thermal equilibrium'],
            ),
            CorrelationRule(
                name='security_breach_chain',
                required_types=['unauthorized_access', 'parameter_tampering'],
                time_window_sec=10.0,
                root_cause='Coordinated security breach attempt',
                recommended_actions=['Isolate machine', 'Notify security team', 'Audit access logs'],
                min_confidence=0.9,
            ),
        ]

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        """Buffer incoming anomaly alert."""
        entry = AlertEntry(
            alert_type=msg.anomaly_type,
            machine_id=msg.machine_id,
            severity=msg.severity,
            message=msg.recommended_action,
            timestamp=msg.timestamp.sec + msg.timestamp.nanosec * 1e-9,
            source='anomaly',
        )
        self._add_to_buffer(entry)

    def _on_security_alert(self, msg: SecurityAlert) -> None:
        """Buffer incoming security alert."""
        entry = AlertEntry(
            alert_type=msg.alert_type,
            machine_id=msg.source_node,
            severity=msg.confidence,
            message=msg.description,
            timestamp=msg.timestamp.sec + msg.timestamp.nanosec * 1e-9,
            source='security',
        )
        self._add_to_buffer(entry)

    def _add_to_buffer(self, entry: AlertEntry) -> None:
        """Add alert entry to the per-machine buffer."""
        with self._buffer_lock:
            if entry.machine_id not in self._alert_buffer:
                self._alert_buffer[entry.machine_id] = []
            self._alert_buffer[entry.machine_id].append(entry)

    def _check_correlations(self) -> None:
        """Periodically check alert buffer for matching correlation rules."""
        now = time.time()

        with self._buffer_lock:
            for machine_id, alerts in list(self._alert_buffer.items()):
                # Prune expired alerts
                alerts[:] = [a for a in alerts if now - a.timestamp < self._correlation_window * 2]

                # Check each rule
                for rule in self._rules:
                    self._try_correlate(machine_id, alerts, rule, now)

    def _try_correlate(
        self, machine_id: str, alerts: List[AlertEntry],
        rule: CorrelationRule, now: float
    ) -> None:
        """Try to match a correlation rule against buffered alerts."""
        # Get alerts within the rule's time window
        window_alerts = [
            a for a in alerts
            if now - a.timestamp <= rule.time_window_sec
        ]

        # Check if all required types are present
        present_types = {a.alert_type for a in window_alerts}
        if not all(rt in present_types for rt in rule.required_types):
            return

        # Build a dedup key to avoid re-firing the same correlation
        matching = [a for a in window_alerts if a.alert_type in rule.required_types]
        dedup_key = f"{machine_id}:{rule.name}:{int(now // rule.time_window_sec)}"
        if dedup_key in self._suppressed_ids:
            return
        self._suppressed_ids.add(dedup_key)

        # Compute confidence from contributing alerts
        avg_severity = sum(a.severity for a in matching) / len(matching)
        confidence = min(1.0, avg_severity * (len(matching) / len(rule.required_types)))

        if confidence < rule.min_confidence:
            return

        # Publish correlated alert
        self._correlation_counter += 1
        msg = CorrelatedAlert()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.correlation_id = f"CORR-{self._correlation_counter:06d}"
        msg.category = rule.name
        msg.severity = max(a.severity for a in matching)
        msg.root_cause_hypothesis = rule.root_cause
        msg.confidence = confidence
        msg.contributing_alert_ids = [f"{a.alert_type}@{a.timestamp:.0f}" for a in matching]
        msg.recommended_actions = rule.recommended_actions
        msg.machine_id = machine_id

        self._correlated_pub.publish(msg)
        self.get_logger().info(
            f"Correlated alert: [{msg.correlation_id}] {rule.name} "
            f"on {machine_id} (confidence={confidence:.2f})"
        )

    @property
    def suppressed_count(self) -> int:
        """Number of suppressed duplicate correlations."""
        return len(self._suppressed_ids)


def main(args=None):
    """Entry point for the alert correlator node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = AlertCorrelatorNode()
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
