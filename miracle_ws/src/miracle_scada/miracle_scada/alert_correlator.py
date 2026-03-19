"""
Alert Correlation Engine.

Groups related alerts by machine_id within a sliding time window and
publishes CorrelatedAlert messages when YAML-configurable correlation rules match.

Optionally integrates with the causal inference engine to boost correlation
confidence when supporting causal evidence exists.
"""

from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field, asdict
import json
import threading
import time
import os
import uuid
import yaml

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import AnomalyAlert, SecurityAlert, CorrelatedAlert


@dataclass
class GCodeContext:
    """G-code execution context for a machine at a point in time."""
    program_name: str
    block_index: int
    gcode_line: str
    operation_type: str  # "LINEAR_CUT", "ARC_CUT", "DRILL", "RAPID", "TOOL_CHANGE"
    feed_rate: float
    spindle_rpm: float
    depth_of_cut: float
    tool_id: str
    elapsed_program_pct: float  # 0-100

    def to_json(self) -> str:
        """Serialize to JSON string."""
        return json.dumps(asdict(self))

    @classmethod
    def from_json(cls, json_str: str) -> 'GCodeContext':
        """Deserialize from JSON string."""
        return cls(**json.loads(json_str))


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


@dataclass
class AnomalyPattern:
    """A learned anomaly pattern from historical correlated alerts."""
    pattern_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    name: str = ''
    description: str = ''
    signature: dict = field(default_factory=lambda: {
        'alert_types': [],
        'temporal_order': [],
        'time_window_sec': 0.0,
        'min_count': 1,
    })
    first_seen: float = 0.0
    last_seen: float = 0.0
    occurrence_count: int = 0
    associated_root_cause: str = ''
    recommended_action: str = ''
    confidence: float = 0.0
    machine_ids: list = field(default_factory=list)

    def to_dict(self) -> dict:
        """Serialize to a plain dictionary."""
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> 'AnomalyPattern':
        """Deserialize from a dictionary."""
        return cls(**d)


class AnomalyPatternLibrary:
    """Library of learned anomaly patterns with JSONL persistence.

    Stores anomaly signatures extracted from correlated alerts and matches
    incoming alert sequences against known patterns.
    """

    SIMILARITY_THRESHOLD = 0.70

    def __init__(self, persistence_path: Optional[str] = None) -> None:
        self._patterns: List[AnomalyPattern] = []
        self._persistence_path = persistence_path
        if persistence_path and os.path.isfile(persistence_path):
            self.load()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def record_pattern(self, correlated_alert: Any) -> AnomalyPattern:
        """Extract a signature from a correlated alert and record it.

        If a sufficiently similar pattern already exists (>70 % overlap),
        the existing pattern is updated; otherwise a new one is created.

        Args:
            correlated_alert: An object (or dict) with attributes
                ``category``, ``contributing_alert_ids``,
                ``root_cause_hypothesis``, ``recommended_actions``,
                ``machine_id``, ``confidence``.

        Returns:
            The created or updated ``AnomalyPattern``.
        """
        sig = self._extract_signature(correlated_alert)
        now = time.time()

        # Try to find an existing similar pattern
        best_match: Optional[AnomalyPattern] = None
        best_score = 0.0
        for pattern in self._patterns:
            score = self._compute_signature_similarity(sig, pattern.signature)
            if score > best_score:
                best_score = score
                best_match = pattern

        machine_id = self._get_attr(correlated_alert, 'machine_id', '')

        if best_match is not None and best_score >= self.SIMILARITY_THRESHOLD:
            # Update existing pattern
            best_match.last_seen = now
            best_match.occurrence_count += 1
            best_match.confidence = min(
                1.0, 0.5 + 0.05 * best_match.occurrence_count
            )
            if machine_id and machine_id not in best_match.machine_ids:
                best_match.machine_ids.append(machine_id)
            self._auto_save()
            return best_match

        # Create new pattern
        root_cause = self._get_attr(correlated_alert, 'root_cause_hypothesis', '')
        actions = self._get_attr(correlated_alert, 'recommended_actions', [])
        category = self._get_attr(correlated_alert, 'category', '')

        pattern = AnomalyPattern(
            name=category,
            description=f"Auto-learned pattern from {category}",
            signature=sig,
            first_seen=now,
            last_seen=now,
            occurrence_count=1,
            associated_root_cause=root_cause,
            recommended_action=actions[0] if actions else '',
            confidence=0.5,
            machine_ids=[machine_id] if machine_id else [],
        )
        self._patterns.append(pattern)
        self._auto_save()
        return pattern

    def match_pattern(
        self, alert_sequence: List[Any]
    ) -> List[Tuple[AnomalyPattern, float]]:
        """Match an alert sequence against the pattern library.

        Args:
            alert_sequence: List of ``AlertEntry``-like objects with
                ``alert_type`` and ``timestamp`` attributes.

        Returns:
            List of ``(AnomalyPattern, match_score)`` tuples sorted
            descending by score.  Only patterns with score > 0 are
            returned.
        """
        if not alert_sequence or not self._patterns:
            return []

        query_sig = self._build_signature_from_alerts(alert_sequence)
        results: List[Tuple[AnomalyPattern, float]] = []
        for pattern in self._patterns:
            score = self._compute_signature_similarity(query_sig, pattern.signature)
            if score > 0.0:
                results.append((pattern, score))

        results.sort(key=lambda x: x[1], reverse=True)
        return results

    def get_top_patterns(self, n: int = 10) -> List[AnomalyPattern]:
        """Return the *n* most frequently occurring patterns."""
        return sorted(
            self._patterns, key=lambda p: p.occurrence_count, reverse=True
        )[:n]

    def get_pattern_for_machine(self, machine_id: str) -> List[AnomalyPattern]:
        """Return patterns that have been seen on *machine_id*."""
        return [p for p in self._patterns if machine_id in p.machine_ids]

    def merge_patterns(self, pattern_id_1: str, pattern_id_2: str) -> Optional[AnomalyPattern]:
        """Merge two patterns into one, removing the second.

        The merged pattern retains the id of the first pattern.

        Returns:
            The merged ``AnomalyPattern``, or ``None`` if either id is
            not found.
        """
        p1 = self._find_by_id(pattern_id_1)
        p2 = self._find_by_id(pattern_id_2)
        if p1 is None or p2 is None:
            return None

        # Merge fields
        p1.occurrence_count += p2.occurrence_count
        p1.first_seen = min(p1.first_seen, p2.first_seen)
        p1.last_seen = max(p1.last_seen, p2.last_seen)
        p1.confidence = min(1.0, 0.5 + 0.05 * p1.occurrence_count)
        for mid in p2.machine_ids:
            if mid not in p1.machine_ids:
                p1.machine_ids.append(mid)

        # Merge signature alert_types (union)
        merged_types = list(dict.fromkeys(
            p1.signature.get('alert_types', []) + p2.signature.get('alert_types', [])
        ))
        p1.signature['alert_types'] = merged_types

        # Remove p2
        self._patterns = [p for p in self._patterns if p.pattern_id != pattern_id_2]
        self._auto_save()
        return p1

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save(self) -> None:
        """Persist patterns to JSONL file."""
        if not self._persistence_path:
            return
        with open(self._persistence_path, 'w') as fh:
            for pattern in self._patterns:
                fh.write(json.dumps(pattern.to_dict()) + '\n')

    def load(self) -> None:
        """Load patterns from JSONL file."""
        if not self._persistence_path or not os.path.isfile(self._persistence_path):
            return
        loaded: List[AnomalyPattern] = []
        with open(self._persistence_path, 'r') as fh:
            for line in fh:
                line = line.strip()
                if line:
                    loaded.append(AnomalyPattern.from_dict(json.loads(line)))
        self._patterns = loaded

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @property
    def patterns(self) -> List[AnomalyPattern]:
        """Read-only access to the pattern list."""
        return list(self._patterns)

    def _auto_save(self) -> None:
        """Persist if a persistence path is configured."""
        if self._persistence_path:
            self.save()

    def _find_by_id(self, pattern_id: str) -> Optional[AnomalyPattern]:
        for p in self._patterns:
            if p.pattern_id == pattern_id:
                return p
        return None

    @staticmethod
    def _get_attr(obj: Any, name: str, default: Any = None) -> Any:
        """Get attribute from object or dict."""
        if isinstance(obj, dict):
            return obj.get(name, default)
        return getattr(obj, name, default)

    def _extract_signature(self, correlated_alert: Any) -> dict:
        """Build a signature dict from a correlated alert."""
        contributing = self._get_attr(correlated_alert, 'contributing_alert_ids', [])
        alert_types: List[str] = []
        for cid in contributing:
            if '@' in cid:
                alert_types.append(cid.split('@')[0])
            else:
                alert_types.append(cid)

        # Deduplicate while preserving order
        seen: set = set()
        unique_types: List[str] = []
        for t in alert_types:
            if t not in seen:
                seen.add(t)
                unique_types.append(t)

        return {
            'alert_types': unique_types,
            'temporal_order': unique_types,  # preserved insertion order
            'time_window_sec': 30.0,
            'min_count': len(unique_types),
        }

    @staticmethod
    def _build_signature_from_alerts(alerts: List[Any]) -> dict:
        """Build a query signature from a list of AlertEntry-like objects."""
        sorted_alerts = sorted(alerts, key=lambda a: getattr(a, 'timestamp', 0))
        seen: set = set()
        types: List[str] = []
        for a in sorted_alerts:
            t = getattr(a, 'alert_type', '')
            if t and t not in seen:
                seen.add(t)
                types.append(t)

        if len(sorted_alerts) >= 2:
            window = (
                getattr(sorted_alerts[-1], 'timestamp', 0)
                - getattr(sorted_alerts[0], 'timestamp', 0)
            )
        else:
            window = 0.0

        return {
            'alert_types': types,
            'temporal_order': types,
            'time_window_sec': window,
            'min_count': len(types),
        }

    @staticmethod
    def _compute_signature_similarity(sig1: dict, sig2: dict) -> float:
        """Compute similarity between two signatures (0-1).

        Components:
        - alert_type overlap (Jaccard) — weight 0.5
        - temporal_order match — weight 0.3
        - time_window proximity — weight 0.2
        """
        types1 = set(sig1.get('alert_types', []))
        types2 = set(sig2.get('alert_types', []))
        if not types1 and not types2:
            type_score = 1.0
        elif not types1 or not types2:
            type_score = 0.0
        else:
            type_score = len(types1 & types2) / len(types1 | types2)

        # Temporal order: ratio of matching prefix
        order1 = sig1.get('temporal_order', [])
        order2 = sig2.get('temporal_order', [])
        if not order1 and not order2:
            order_score = 1.0
        elif not order1 or not order2:
            order_score = 0.0
        else:
            matches = sum(
                1 for a, b in zip(order1, order2) if a == b
            )
            order_score = matches / max(len(order1), len(order2))

        # Time window proximity
        w1 = sig1.get('time_window_sec', 0.0)
        w2 = sig2.get('time_window_sec', 0.0)
        max_w = max(abs(w1), abs(w2), 1.0)
        window_score = 1.0 - min(abs(w1 - w2) / max_w, 1.0)

        return 0.5 * type_score + 0.3 * order_score + 0.2 * window_score


@dataclass
class AlarmCorrelationRule:
    """A user-defined correlation rule for linking related alarms."""
    rule_id: str
    name: str
    conditions: List[Dict[str, Any]]
    time_window_sec: float
    min_matches: int
    action: str  # 'group' | 'suppress_secondary' | 'escalate'
    priority: int
    description: str = ''


@dataclass
class AlarmEvent:
    """An alarm event submitted for correlation evaluation."""
    alarm_id: str
    alarm_type: str
    source: str
    severity: float
    timestamp: float
    properties: Dict[str, Any] = field(default_factory=dict)


@dataclass
class CorrelationMatch:
    """Result of a successful correlation rule match."""
    rule_id: str
    matched_alarms: List[str]
    root_alarm_id: str
    action: str
    confidence: float
    timestamp: float


class AlarmCorrelationRuleEngine:
    """Evaluates user-defined correlation rules to link related alarms.

    Maintains a set of correlation rules and a sliding window of recent
    alarms.  When a new alarm is submitted via ``submit_alarm``, all rules
    are evaluated against the recent alarm history.  Matches are recorded
    and can be retrieved via ``get_correlation_history``.

    Three built-in rules are registered on construction:

    * **THERMAL_CASCADE** -- temperature + vibration within 30 s
    * **TOOL_FAILURE_CHAIN** -- force + wear + quality within 60 s
    * **POWER_SEQUENCE** -- power + spindle + feed within 10 s
    """

    def __init__(self) -> None:
        self._rules: Dict[str, AlarmCorrelationRule] = {}
        self._recent_alarms: List[AlarmEvent] = []
        self._correlation_history: List[CorrelationMatch] = []
        self._lock = threading.Lock()
        self._register_builtin_rules()

    # ------------------------------------------------------------------
    # Built-in rules
    # ------------------------------------------------------------------

    def _register_builtin_rules(self) -> None:
        """Register the three default correlation rules."""
        self.add_rule(AlarmCorrelationRule(
            rule_id='THERMAL_CASCADE',
            name='Thermal Cascade',
            conditions=[
                {'field': 'alarm_type', 'operator': 'eq', 'value': 'temperature'},
                {'field': 'alarm_type', 'operator': 'eq', 'value': 'vibration'},
            ],
            time_window_sec=30.0,
            min_matches=2,
            action='group',
            priority=10,
            description='Temperature and vibration alarms within 30s indicate thermal cascade',
        ))

        self.add_rule(AlarmCorrelationRule(
            rule_id='TOOL_FAILURE_CHAIN',
            name='Tool Failure Chain',
            conditions=[
                {'field': 'alarm_type', 'operator': 'eq', 'value': 'force'},
                {'field': 'alarm_type', 'operator': 'eq', 'value': 'wear'},
                {'field': 'alarm_type', 'operator': 'eq', 'value': 'quality'},
            ],
            time_window_sec=60.0,
            min_matches=3,
            action='escalate',
            priority=20,
            description='Force, wear, and quality alarms within 60s indicate tool failure chain',
        ))

        self.add_rule(AlarmCorrelationRule(
            rule_id='POWER_SEQUENCE',
            name='Power Sequence',
            conditions=[
                {'field': 'alarm_type', 'operator': 'eq', 'value': 'power'},
                {'field': 'alarm_type', 'operator': 'eq', 'value': 'spindle'},
                {'field': 'alarm_type', 'operator': 'eq', 'value': 'feed'},
            ],
            time_window_sec=10.0,
            min_matches=3,
            action='suppress_secondary',
            priority=30,
            description='Power, spindle, and feed alarms within 10s indicate power sequence',
        ))

    # ------------------------------------------------------------------
    # Rule management
    # ------------------------------------------------------------------

    def add_rule(self, rule: AlarmCorrelationRule) -> None:
        """Add or replace a correlation rule.

        Args:
            rule: The correlation rule to register.
        """
        with self._lock:
            self._rules[rule.rule_id] = rule

    def remove_rule(self, rule_id: str) -> bool:
        """Remove a correlation rule by id.

        Args:
            rule_id: Identifier of the rule to remove.

        Returns:
            ``True`` if the rule was found and removed, ``False`` otherwise.
        """
        with self._lock:
            if rule_id in self._rules:
                del self._rules[rule_id]
                return True
            return False

    def get_rules(self) -> List[AlarmCorrelationRule]:
        """Return all registered correlation rules, sorted by priority descending."""
        with self._lock:
            return sorted(self._rules.values(), key=lambda r: r.priority, reverse=True)

    # ------------------------------------------------------------------
    # Alarm submission & evaluation
    # ------------------------------------------------------------------

    def submit_alarm(self, alarm: AlarmEvent) -> List[CorrelationMatch]:
        """Submit an alarm for correlation evaluation.

        The alarm is added to the recent-alarm buffer and all rules are
        evaluated against the updated buffer.

        Args:
            alarm: The alarm event to evaluate.

        Returns:
            A list of ``CorrelationMatch`` objects for any rules that
            matched.
        """
        with self._lock:
            self._recent_alarms.append(alarm)
            matches = self._evaluate_rules_locked(alarm)
            self._correlation_history.extend(matches)
            return list(matches)

    def evaluate_rules(self, alarm: AlarmEvent) -> List[CorrelationMatch]:
        """Check all rules against recent alarms within their time windows.

        Unlike ``submit_alarm`` this does **not** add the alarm to the
        buffer or record matches in the history.

        Args:
            alarm: The alarm event to evaluate against.

        Returns:
            A list of ``CorrelationMatch`` objects for matching rules.
        """
        with self._lock:
            return self._evaluate_rules_locked(alarm)

    def _evaluate_rules_locked(self, alarm: AlarmEvent) -> List[CorrelationMatch]:
        """Evaluate all rules (must be called while holding ``_lock``)."""
        matches: List[CorrelationMatch] = []
        # Sort rules by priority descending so higher-priority rules are evaluated first
        sorted_rules = sorted(self._rules.values(), key=lambda r: r.priority, reverse=True)

        for rule in sorted_rules:
            match = self._evaluate_single_rule(rule, alarm)
            if match is not None:
                matches.append(match)

        return matches

    def _evaluate_single_rule(
        self, rule: AlarmCorrelationRule, trigger_alarm: AlarmEvent,
    ) -> Optional[CorrelationMatch]:
        """Evaluate a single rule against the recent alarm buffer.

        Returns a ``CorrelationMatch`` if the rule conditions are
        satisfied, otherwise ``None``.
        """
        cutoff = trigger_alarm.timestamp - rule.time_window_sec
        window_alarms = [
            a for a in self._recent_alarms
            if a.timestamp >= cutoff and a.timestamp <= trigger_alarm.timestamp
        ]

        if len(window_alarms) < rule.min_matches:
            return None

        # For each condition, find at least one alarm that satisfies it
        satisfied_conditions: List[List[AlarmEvent]] = []
        for condition in rule.conditions:
            matching_alarms = [
                a for a in window_alarms
                if self._check_condition(a, condition)
            ]
            if not matching_alarms:
                return None
            satisfied_conditions.append(matching_alarms)

        # Collect unique alarm ids from all matched conditions
        matched_alarm_ids: List[str] = []
        seen_ids: set = set()
        for alarm_group in satisfied_conditions:
            for a in alarm_group:
                if a.alarm_id not in seen_ids:
                    seen_ids.add(a.alarm_id)
                    matched_alarm_ids.append(a.alarm_id)

        if len(matched_alarm_ids) < rule.min_matches:
            return None

        # Determine root alarm (earliest in the window)
        matched_alarms_objs = [
            a for a in window_alarms if a.alarm_id in seen_ids
        ]
        root_alarm = min(matched_alarms_objs, key=lambda a: a.timestamp)

        # Confidence: based on ratio of satisfied conditions, average severity,
        # and how many alarms matched relative to min_matches
        condition_ratio = len(rule.conditions) / max(len(rule.conditions), 1)
        avg_severity = sum(a.severity for a in matched_alarms_objs) / len(matched_alarms_objs)
        count_ratio = min(len(matched_alarm_ids) / max(rule.min_matches, 1), 2.0)
        confidence = min(1.0, condition_ratio * avg_severity * count_ratio)

        return CorrelationMatch(
            rule_id=rule.rule_id,
            matched_alarms=matched_alarm_ids,
            root_alarm_id=root_alarm.alarm_id,
            action=rule.action,
            confidence=confidence,
            timestamp=trigger_alarm.timestamp,
        )

    @staticmethod
    def _check_condition(alarm: AlarmEvent, condition: Dict[str, Any]) -> bool:
        """Check whether an alarm satisfies a single condition dict.

        Supported operators: ``eq``, ``ne``, ``gt``, ``lt``, ``contains``.

        The *field* key is looked up first on the ``AlarmEvent`` attributes
        and then in ``alarm.properties``.
        """
        field_name = condition.get('field', '')
        operator = condition.get('operator', '')
        expected = condition.get('value')

        # Resolve field value from alarm attributes or properties
        if hasattr(alarm, field_name) and field_name != 'properties':
            actual = getattr(alarm, field_name)
        else:
            actual = alarm.properties.get(field_name)

        if actual is None:
            return False

        if operator == 'eq':
            return actual == expected
        elif operator == 'ne':
            return actual != expected
        elif operator == 'gt':
            return actual > expected
        elif operator == 'lt':
            return actual < expected
        elif operator == 'contains':
            return expected in actual
        return False

    # ------------------------------------------------------------------
    # History
    # ------------------------------------------------------------------

    def get_correlation_history(self) -> List[CorrelationMatch]:
        """Return all past correlation matches."""
        with self._lock:
            return list(self._correlation_history)


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

        # G-code context tracking per machine
        self._machine_gcode_context: Dict[str, GCodeContext] = {}

        # Block anomaly history: machine_id -> block_index -> count
        self._block_anomaly_history: Dict[str, Dict[int, int]] = {}
        self._block_anomaly_history_cap: int = 1000

        # Learned anomaly pattern library
        self._pattern_library = AnomalyPatternLibrary()

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

        # Enrich with G-code context
        confidence, gcode_context_json, blocks_until = self._enrich_alert_with_gcode(
            None, machine_id, confidence,
        )

        # Check pattern library for known signatures and boost confidence
        pattern_matches = self._pattern_library.match_pattern(matching)
        if pattern_matches and pattern_matches[0][1] >= AnomalyPatternLibrary.SIMILARITY_THRESHOLD:
            confidence = min(1.0, confidence + 0.1)

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

        # Attach G-code context fields
        if gcode_context_json is not None:
            msg.gcode_context_json = gcode_context_json
        if blocks_until is not None:
            msg.blocks_until_predicted_issue = blocks_until

        self._correlated_pub.publish(msg)

        # Record pattern in anomaly pattern library
        self._pattern_library.record_pattern(msg)

        boost_info = f", causal_boost={causal_boost:.3f}" if causal_boost > 0 else ""
        gcode_info = f", gcode_block={self._machine_gcode_context[machine_id].block_index}" if machine_id in self._machine_gcode_context else ""
        self.get_logger().info(
            f"Correlated alert: [{msg.correlation_id}] {rule.name} "
            f"on {machine_id} (confidence={confidence:.2f}{boost_info}{gcode_info})"
        )

    def update_gcode_context(self, machine_id: str, context: GCodeContext) -> None:
        """Update the current G-code execution context for a machine.

        Args:
            machine_id: The machine whose context is being updated.
            context: The current G-code execution context.
        """
        self._machine_gcode_context[machine_id] = context

    def get_gcode_context(self, machine_id: str) -> Optional[GCodeContext]:
        """Return the current G-code context for a machine, or None."""
        return self._machine_gcode_context.get(machine_id)

    def _record_block_anomaly(self, machine_id: str, block_index: int) -> None:
        """Record that an anomaly occurred at a specific G-code block."""
        if machine_id not in self._block_anomaly_history:
            self._block_anomaly_history[machine_id] = {}
        block_map = self._block_anomaly_history[machine_id]
        block_map[block_index] = block_map.get(block_index, 0) + 1
        # Cap the number of tracked blocks per machine
        if len(block_map) > self._block_anomaly_history_cap:
            # Remove the entry with the lowest count
            min_block = min(block_map, key=block_map.get)  # type: ignore[arg-type]
            del block_map[min_block]

    def _correlate_with_gcode_patterns(
        self, machine_id: str, block_index: int, base_confidence: float,
    ) -> Tuple[float, bool]:
        """Check if a G-code block has recurring anomalies and boost confidence.

        Args:
            machine_id: The machine to check.
            block_index: The G-code block index.
            base_confidence: The current confidence value.

        Returns:
            (adjusted_confidence, is_recurring) tuple.
        """
        block_map = self._block_anomaly_history.get(machine_id, {})
        count = block_map.get(block_index, 0)
        if count > 2:
            return min(1.0, base_confidence + 0.15), True
        return base_confidence, False

    def _enrich_alert_with_gcode(
        self, msg: Any, machine_id: str, confidence: float,
    ) -> Tuple[float, Optional[str], Optional[int]]:
        """Attach G-code context to a correlated alert message.

        Returns:
            (adjusted_confidence, gcode_context_json, blocks_until_predicted_issue)
        """
        ctx = self._machine_gcode_context.get(machine_id)
        gcode_context_json: Optional[str] = None
        blocks_until_predicted_issue: Optional[int] = None

        if ctx is not None:
            gcode_context_json = ctx.to_json()
            self._record_block_anomaly(machine_id, ctx.block_index)
            confidence, is_recurring = self._correlate_with_gcode_patterns(
                machine_id, ctx.block_index, confidence,
            )
            # Estimate blocks until predicted issue (placeholder heuristic)
            block_map = self._block_anomaly_history.get(machine_id, {})
            # Look ahead for blocks with known issues
            lookahead_blocks = 10
            for future_block in range(ctx.block_index + 1, ctx.block_index + lookahead_blocks + 1):
                if block_map.get(future_block, 0) > 2:
                    blocks_until_predicted_issue = future_block - ctx.block_index
                    break

        return confidence, gcode_context_json, blocks_until_predicted_issue

    @property
    def suppressed_count(self) -> int:
        """Number of suppressed duplicate correlations."""
        return len(self._suppressed_ids)

    @property
    def fleet_alert_count(self) -> int:
        """Number of fleet-wide correlated alerts fired."""
        return self._fleet_alert_count


# ---------------------------------------------------------------------------
# Alarm Analytics Engine
# ---------------------------------------------------------------------------


@dataclass
class AlarmHistoryEntry:
    """A single historical alarm record."""
    alarm_id: str
    alarm_type: str
    machine_id: str
    severity: float
    timestamp: float
    duration_sec: float
    acknowledged: bool
    root_cause: str


@dataclass
class AlarmTrendAnalysis:
    """Trend analysis for a specific alarm type over time periods."""
    alarm_type: str
    count_trend: List[int]
    avg_severity_trend: List[float]
    period_labels: List[str]
    increasing: bool


@dataclass
class AlarmAnalyticsSummary:
    """Summary statistics for alarm history over a time range."""
    total_alarms: int
    unique_types: int
    avg_response_time_sec: float
    top_alarm_types: List[Tuple[str, int]]
    top_machines: List[Tuple[str, int]]
    alarm_rate_per_hour: float
    repeat_alarm_pct: float


class AlarmAnalyticsEngine:
    """Historical alarm analytics and pattern mining engine.

    Stores alarm history entries and provides analytical queries including
    summaries, trend analysis, repeat-alarm detection, mean-time-to-respond,
    and alarm heatmaps.
    """

    def __init__(self) -> None:
        self._history: List[AlarmHistoryEntry] = []
        self._lock = threading.Lock()

    # -- recording ---------------------------------------------------------

    def record_alarm(self, entry: AlarmHistoryEntry) -> None:
        """Store an alarm history entry."""
        with self._lock:
            self._history.append(entry)

    # -- summary -----------------------------------------------------------

    def get_summary(
        self, start_time: float, end_time: float
    ) -> AlarmAnalyticsSummary:
        """Generate an :class:`AlarmAnalyticsSummary` for *[start_time, end_time]*."""
        with self._lock:
            entries = [
                e for e in self._history
                if start_time <= e.timestamp <= end_time
            ]

        total = len(entries)
        if total == 0:
            return AlarmAnalyticsSummary(
                total_alarms=0,
                unique_types=0,
                avg_response_time_sec=0.0,
                top_alarm_types=[],
                top_machines=[],
                alarm_rate_per_hour=0.0,
                repeat_alarm_pct=0.0,
            )

        # Unique types
        type_counts: Dict[str, int] = {}
        machine_counts: Dict[str, int] = {}
        for e in entries:
            type_counts[e.alarm_type] = type_counts.get(e.alarm_type, 0) + 1
            machine_counts[e.machine_id] = machine_counts.get(e.machine_id, 0) + 1

        unique_types = len(type_counts)

        # Average response time (duration_sec for acknowledged alarms)
        ack_durations = [e.duration_sec for e in entries if e.acknowledged]
        avg_response = (
            sum(ack_durations) / len(ack_durations) if ack_durations else 0.0
        )

        # Top alarm types / machines (sorted descending by count)
        top_alarm_types = sorted(type_counts.items(), key=lambda x: x[1], reverse=True)
        top_machines = sorted(machine_counts.items(), key=lambda x: x[1], reverse=True)

        # Rate
        span_hours = max((end_time - start_time) / 3600.0, 1e-9)
        alarm_rate = total / span_hours

        # Repeat alarm percentage: alarms whose (alarm_type, machine_id) pair
        # appears more than once in the window.
        pair_counts: Dict[Tuple[str, str], int] = {}
        for e in entries:
            key = (e.alarm_type, e.machine_id)
            pair_counts[key] = pair_counts.get(key, 0) + 1
        repeat_count = sum(c for c in pair_counts.values() if c > 1)
        repeat_pct = (repeat_count / total) * 100.0 if total else 0.0

        return AlarmAnalyticsSummary(
            total_alarms=total,
            unique_types=unique_types,
            avg_response_time_sec=avg_response,
            top_alarm_types=top_alarm_types,
            top_machines=top_machines,
            alarm_rate_per_hour=alarm_rate,
            repeat_alarm_pct=repeat_pct,
        )

    # -- trend analysis ----------------------------------------------------

    def get_trend(
        self,
        alarm_type: str,
        periods: int = 5,
        period_duration_sec: float = 3600.0,
    ) -> AlarmTrendAnalysis:
        """Analyse alarm count and severity trend over *periods* windows.

        Windows are defined backwards from the most recent alarm of *alarm_type*
        (or from the latest alarm overall when none of the requested type exist).
        """
        with self._lock:
            typed = [e for e in self._history if e.alarm_type == alarm_type]

        if not typed:
            return AlarmTrendAnalysis(
                alarm_type=alarm_type,
                count_trend=[0] * periods,
                avg_severity_trend=[0.0] * periods,
                period_labels=[f"P{i}" for i in range(periods)],
                increasing=False,
            )

        latest_ts = max(e.timestamp for e in typed)

        count_trend: List[int] = []
        severity_trend: List[float] = []
        labels: List[str] = []

        for i in range(periods - 1, -1, -1):
            p_start = latest_ts - (i + 1) * period_duration_sec
            p_end = latest_ts - i * period_duration_sec
            bucket = [e for e in typed if p_start <= e.timestamp < p_end]
            count_trend.append(len(bucket))
            if bucket:
                severity_trend.append(
                    sum(e.severity for e in bucket) / len(bucket)
                )
            else:
                severity_trend.append(0.0)
            labels.append(f"P{periods - i - 1}")

        # Determine if increasing (simple linear check on counts)
        increasing = False
        if len(count_trend) >= 2:
            first_half = count_trend[: len(count_trend) // 2]
            second_half = count_trend[len(count_trend) // 2:]
            if sum(second_half) > sum(first_half):
                increasing = True

        return AlarmTrendAnalysis(
            alarm_type=alarm_type,
            count_trend=count_trend,
            avg_severity_trend=severity_trend,
            period_labels=labels,
            increasing=increasing,
        )

    # -- repeat alarms -----------------------------------------------------

    def get_repeat_alarms(
        self, window_sec: float = 300.0
    ) -> List[List[AlarmHistoryEntry]]:
        """Find groups of alarms with the same type on the same machine within *window_sec*.

        Returns a list of groups, each group being a list of related entries.
        """
        with self._lock:
            entries = list(self._history)

        # Group by (alarm_type, machine_id)
        groups: Dict[Tuple[str, str], List[AlarmHistoryEntry]] = {}
        for e in entries:
            key = (e.alarm_type, e.machine_id)
            groups.setdefault(key, []).append(e)

        result: List[List[AlarmHistoryEntry]] = []
        for _key, group in groups.items():
            if len(group) < 2:
                continue
            # Sort by timestamp
            group.sort(key=lambda x: x.timestamp)
            # Find clusters within window_sec
            cluster: List[AlarmHistoryEntry] = [group[0]]
            for i in range(1, len(group)):
                if group[i].timestamp - cluster[0].timestamp <= window_sec:
                    cluster.append(group[i])
                else:
                    if len(cluster) >= 2:
                        result.append(cluster)
                    cluster = [group[i]]
            if len(cluster) >= 2:
                result.append(cluster)

        return result

    # -- MTTR (mean time to respond / acknowledge) -------------------------

    def get_mttr(self, alarm_type: Optional[str] = None) -> float:
        """Mean time to respond (duration_sec for acknowledged alarms).

        If *alarm_type* is given, restrict to that type.  Returns 0.0 when
        no acknowledged alarms exist.
        """
        with self._lock:
            entries = list(self._history)

        if alarm_type is not None:
            entries = [e for e in entries if e.alarm_type == alarm_type]

        ack = [e.duration_sec for e in entries if e.acknowledged]
        return sum(ack) / len(ack) if ack else 0.0

    # -- heatmap -----------------------------------------------------------

    def get_alarm_heatmap(
        self,
        machines: List[str],
        alarm_types: List[str],
    ) -> List[List[int]]:
        """Build a matrix of alarm counts — rows=machines, cols=alarm_types.

        Returns a list-of-lists where ``result[i][j]`` is the alarm count for
        ``machines[i]`` and ``alarm_types[j]``.
        """
        with self._lock:
            entries = list(self._history)

        # Pre-compute counts
        pair_counts: Dict[Tuple[str, str], int] = {}
        for e in entries:
            key = (e.machine_id, e.alarm_type)
            pair_counts[key] = pair_counts.get(key, 0) + 1

        return [
            [pair_counts.get((m, t), 0) for t in alarm_types]
            for m in machines
        ]


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
