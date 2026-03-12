"""
Alert Correlation Engine.

Groups related alerts by machine_id within a sliding time window and
publishes CorrelatedAlert messages when YAML-configurable correlation rules match.

Optionally integrates with the causal inference engine to boost correlation
confidence when supporting causal evidence exists.
"""

from typing import Any, Dict, List, Optional, Tuple
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


@dataclass
class FleetCorrelationRule:
    """A YAML-loaded fleet-wide correlation rule."""
    name: str
    description: str
    min_machines: int
    time_window_sec: float
    root_cause: str
    recommended_actions: List[str]
    min_confidence: float = 0.8


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
        self._fleet_rules: List[FleetCorrelationRule] = []
        self._correlation_window: float = 30.0
        self._fleet_window: float = 60.0
        self._correlated_pub = None
        self._check_timer = None
        self._correlation_counter: int = 0
        self._suppressed_ids: set = set()

        # Fleet-wide correlation state
        self._fleet_buffer: Dict[str, List[AlertEntry]] = {}  # keyed by anomaly_type
        self._fleet_suppressed_ids: set = set()
        self._fleet_alert_count: int = 0

        # Optional causal inference integration
        self._causal_links: Dict[str, List[Tuple[str, float]]] = {}  # effect -> [(cause, strength)]

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
            f"Alert correlator configured with {len(self._rules)} per-machine rules, "
            f"{len(self._fleet_rules)} fleet rules, window={self._correlation_window}s"
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
                for fleet_data in data.get('fleet_rules', []):
                    fleet_rule = FleetCorrelationRule(
                        name=fleet_data['name'],
                        description=fleet_data.get('description', ''),
                        min_machines=fleet_data.get('min_machines', 2),
                        time_window_sec=fleet_data.get('time_window_sec', self._fleet_window),
                        root_cause=fleet_data['root_cause'],
                        recommended_actions=fleet_data.get('recommended_actions', []),
                        min_confidence=fleet_data.get('min_confidence', 0.8),
                    )
                    self._fleet_rules.append(fleet_rule)
                self.get_logger().info(
                    f"Loaded {len(self._rules)} correlation rules and "
                    f"{len(self._fleet_rules)} fleet rules from {path}"
                )
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
        self._fleet_rules = [
            FleetCorrelationRule(
                name='fleet_same_anomaly',
                description='Same anomaly type detected across multiple machines',
                min_machines=2,
                time_window_sec=60.0,
                root_cause='Shared environmental or supply factor',
                recommended_actions=[
                    'Check shared coolant supply',
                    'Verify power quality across facility',
                    'Inspect common material batch',
                ],
                min_confidence=0.8,
            ),
            FleetCorrelationRule(
                name='fleet_cascading_failure',
                description='Multiple different anomalies across fleet within window',
                min_machines=3,
                time_window_sec=120.0,
                root_cause='Potential facility-wide issue',
                recommended_actions=[
                    'Check HVAC and ambient temperature',
                    'Verify facility power supply stability',
                    'Review recent maintenance schedule',
                ],
                min_confidence=0.7,
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
        """Add alert entry to the per-machine and fleet-wide buffers."""
        with self._buffer_lock:
            if entry.machine_id not in self._alert_buffer:
                self._alert_buffer[entry.machine_id] = []
            self._alert_buffer[entry.machine_id].append(entry)

            # Also add to fleet buffer keyed by anomaly_type
            if entry.alert_type not in self._fleet_buffer:
                self._fleet_buffer[entry.alert_type] = []
            self._fleet_buffer[entry.alert_type].append(entry)

    def _check_correlations(self) -> None:
        """Periodically check alert buffer for matching correlation rules."""
        now = time.time()

        with self._buffer_lock:
            # Per-machine correlation
            for machine_id, alerts in list(self._alert_buffer.items()):
                # Prune expired alerts
                alerts[:] = [a for a in alerts if now - a.timestamp < self._correlation_window * 2]

                # Check each rule
                for rule in self._rules:
                    self._try_correlate(machine_id, alerts, rule, now)

            # Fleet-wide correlation
            self._check_fleet_correlation(now)

    def _check_fleet_correlation(self, now: float) -> None:
        """Check fleet-wide buffers for cross-machine anomaly patterns.

        Must be called while holding ``_buffer_lock``.
        """
        for fleet_rule in self._fleet_rules:
            if fleet_rule.name == 'fleet_same_anomaly':
                self._check_fleet_same_anomaly(fleet_rule, now)
            elif fleet_rule.name == 'fleet_cascading_failure':
                self._check_fleet_cascading_failure(fleet_rule, now)

    def _check_fleet_same_anomaly(
        self, rule: FleetCorrelationRule, now: float
    ) -> None:
        """Detect same anomaly type appearing on multiple machines."""
        for anomaly_type, entries in list(self._fleet_buffer.items()):
            # Prune expired entries
            entries[:] = [e for e in entries if now - e.timestamp <= rule.time_window_sec]

            # Gather distinct machines within window
            machines: Dict[str, AlertEntry] = {}
            for entry in entries:
                if entry.machine_id not in machines or entry.severity > machines[entry.machine_id].severity:
                    machines[entry.machine_id] = entry

            if len(machines) < rule.min_machines:
                continue

            # Dedup: don't re-fire same fleet correlation
            dedup_key = f"fleet:{anomaly_type}:{int(now // rule.time_window_sec)}"
            if dedup_key in self._fleet_suppressed_ids:
                continue

            # Confidence from average severity
            matching = list(machines.values())
            avg_severity = sum(e.severity for e in matching) / len(matching)
            confidence = min(1.0, avg_severity * (len(matching) / rule.min_machines))
            if confidence < rule.min_confidence:
                continue

            self._fleet_suppressed_ids.add(dedup_key)
            machine_ids = sorted(machines.keys())
            n = len(machine_ids)

            self._correlation_counter += 1
            self._fleet_alert_count += 1

            msg = CorrelatedAlert()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.correlation_id = f"FLEET-{self._correlation_counter:06d}"
            msg.category = rule.name
            msg.severity = max(e.severity for e in matching)
            msg.root_cause_hypothesis = (
                f"Fleet-wide {anomaly_type} detected on {n} machines"
            )
            msg.confidence = confidence
            msg.contributing_alert_ids = [
                f"{e.alert_type}@{e.machine_id}@{e.timestamp:.0f}" for e in matching
            ]
            msg.recommended_actions = [
                "Investigate shared resource (coolant, power, material batch)",
                "Check environmental conditions",
                "Review recent maintenance across fleet",
            ]
            msg.machine_id = ",".join(machine_ids)

            self._correlated_pub.publish(msg)
            self.get_logger().info(
                f"Fleet alert: [{msg.correlation_id}] {anomaly_type} "
                f"on {machine_ids} (confidence={confidence:.2f})"
            )

    def _check_fleet_cascading_failure(
        self, rule: FleetCorrelationRule, now: float
    ) -> None:
        """Detect different anomaly types across multiple machines (cascading failure)."""
        # Collect all alerts across the fleet within the time window
        all_recent: List[AlertEntry] = []
        for entries in self._fleet_buffer.values():
            for e in entries:
                if now - e.timestamp <= rule.time_window_sec:
                    all_recent.append(e)

        # Need distinct machines
        machines = {e.machine_id for e in all_recent}
        anomaly_types = {e.alert_type for e in all_recent}

        if len(machines) < rule.min_machines:
            return
        if len(anomaly_types) < 2:
            # Need different anomaly types for cascading failure
            return

        dedup_key = f"fleet_cascade:{int(now // rule.time_window_sec)}"
        if dedup_key in self._fleet_suppressed_ids:
            return

        avg_severity = sum(e.severity for e in all_recent) / len(all_recent)
        confidence = min(1.0, avg_severity * (len(machines) / rule.min_machines))
        if confidence < rule.min_confidence:
            return

        self._fleet_suppressed_ids.add(dedup_key)
        machine_ids = sorted(machines)

        self._correlation_counter += 1
        self._fleet_alert_count += 1

        msg = CorrelatedAlert()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.correlation_id = f"FLEET-{self._correlation_counter:06d}"
        msg.category = rule.name
        msg.severity = max(e.severity for e in all_recent)
        msg.root_cause_hypothesis = (
            f"Fleet cascading failure: {len(anomaly_types)} anomaly types "
            f"across {len(machines)} machines"
        )
        msg.confidence = confidence
        msg.contributing_alert_ids = [
            f"{e.alert_type}@{e.machine_id}@{e.timestamp:.0f}" for e in all_recent
        ]
        msg.recommended_actions = rule.recommended_actions
        msg.machine_id = ",".join(machine_ids)

        self._correlated_pub.publish(msg)
        self.get_logger().info(
            f"Fleet cascade alert: [{msg.correlation_id}] "
            f"{len(anomaly_types)} types on {machine_ids} (confidence={confidence:.2f})"
        )

    def set_causal_links(self, causal_links: Dict[str, List[Tuple[str, float]]]) -> None:
        """Provide causal link data for enrichment.

        Args:
            causal_links: Mapping of effect -> [(cause, effective_strength), ...].
        """
        self._causal_links = causal_links

    def _enrich_with_causal_context(
        self,
        root_cause: str,
        alert_types: List[str],
        base_confidence: float,
    ) -> Tuple[float, float]:
        """Enrich a correlation with causal graph evidence.

        Looks up each alert type in the causal links graph.  If a causal link
        exists whose cause matches the root_cause (or a substring thereof), the
        link strength is used to compute a confidence boost.

        Returns:
            (adjusted_confidence, causal_confidence_boost)
        """
        if not self._causal_links:
            return base_confidence, 0.0

        max_causal_strength = 0.0
        for atype in alert_types:
            for effect_key, cause_list in self._causal_links.items():
                if effect_key != atype:
                    continue
                for cause, strength in cause_list:
                    if cause in root_cause or root_cause in cause:
                        max_causal_strength = max(max_causal_strength, strength)

        if max_causal_strength <= 0.0:
            return base_confidence, 0.0

        causal_confidence_boost = max_causal_strength * 0.2
        adjusted = min(1.0, base_confidence * (1.0 + causal_confidence_boost))
        return adjusted, causal_confidence_boost

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

        # Enrich with causal context if available
        alert_types = [a.alert_type for a in matching]
        confidence, causal_boost = self._enrich_with_causal_context(
            rule.root_cause, alert_types, confidence,
        )

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
        boost_info = f", causal_boost={causal_boost:.3f}" if causal_boost > 0 else ""
        self.get_logger().info(
            f"Correlated alert: [{msg.correlation_id}] {rule.name} "
            f"on {machine_id} (confidence={confidence:.2f}{boost_info})"
        )

    @property
    def suppressed_count(self) -> int:
        """Number of suppressed duplicate correlations."""
        return len(self._suppressed_ids)

    @property
    def fleet_alert_count(self) -> int:
        """Number of fleet-wide correlated alerts fired."""
        return self._fleet_alert_count


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
