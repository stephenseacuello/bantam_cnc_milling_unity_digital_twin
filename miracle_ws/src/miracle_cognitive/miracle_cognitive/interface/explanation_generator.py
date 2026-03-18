"""
Explanation Generator — Explainable AI (XAI) for MIRACLE decisions.

Generates human-readable, multi-level explanations for every autonomous
decision: anomaly detection, adaptive overrides, and predictive actions.

Levels:
  1. Summary  — one-line plain-English description
  2. Detail   — analysis with feature contributions and thresholds
  3. Counterfactual — "what-if" alternative scenario
"""

from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field
import json
import math
import time
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import AnomalyAlert, Explanation

try:
    from miracle_cognitive.knowledge.causal_inference import CausalLink
except ImportError:
    # Graceful fallback when ROS2 deps are mocked (e.g. in tests)
    CausalLink = None  # type: ignore[misc,assignment]


@dataclass
class EvidenceItem:
    """A single piece of evidence for or against a root cause hypothesis."""
    source: str  # 'sensor', 'model', 'history', 'operator'
    description: str
    strength: float  # 0-1, how strongly this evidence bears on the cause
    supports: bool  # True = supports the hypothesis, False = contradicts it
    timestamp: float


@dataclass
class RootCauseCandidate:
    """A ranked root cause hypothesis with supporting/contradicting evidence."""
    cause_id: str
    description: str
    probability: float  # 0-1
    evidence: List['EvidenceItem'] = field(default_factory=list)
    supporting_count: int = 0
    contradicting_count: int = 0
    net_score: float = 0.0  # supporting - contradicting, weighted by strength
    mechanism: str = ''  # causal chain explanation
    verification_steps: List[str] = field(default_factory=list)


class RootCauseAnalyzer:
    """Analyzes anomaly data to produce ranked root cause hypotheses.

    Uses cause templates for common CNC failure modes, matches incoming
    anomaly evidence against them, and applies Bayesian-style scoring.
    """

    # Each template maps a cause_id to:
    #   prior        – base probability before evidence
    #   indicators   – dict of anomaly_data field -> expected condition
    #   mechanism    – human-readable causal chain
    #   verification – steps an operator can take to confirm
    CAUSE_TEMPLATES: Dict[str, dict] = {
        'TOOL_WEAR': {
            'prior': 0.15,
            'description': 'Progressive tool wear causing degraded cutting performance',
            'indicators': {
                'force_trend': 'increasing',
                'vibration_trend': 'increasing',
                'tool_wear_vb': 'high',
            },
            'mechanism': (
                'Flank wear increases cutting edge radius -> higher specific cutting '
                'force -> elevated vibration from ploughing -> surface finish degradation'
            ),
            'verification': [
                'Measure flank wear VB under toolmakers microscope',
                'Compare cutting forces at start vs current values',
                'Inspect chip morphology for signs of built-up edge',
                'Check surface roughness Ra on most recent cut',
            ],
        },
        'CHATTER': {
            'prior': 0.10,
            'description': 'Regenerative chatter causing periodic vibration',
            'indicators': {
                'vibration_pattern': 'periodic',
                'frequency_type': 'non_harmonic',
                'vibration_amplitude': 'high',
            },
            'mechanism': (
                'Undulations on previously cut surface modulate chip thickness -> '
                'variable cutting force at chatter frequency -> phase shift causes '
                'regenerative growth -> self-excited vibration at non-tooth-passing frequency'
            ),
            'verification': [
                'Perform FFT analysis on vibration signal',
                'Check if dominant frequency is a non-integer multiple of tooth passing frequency',
                'Inspect workpiece surface for chatter marks (regular waviness)',
                'Tap-test the tool assembly to find natural frequencies',
            ],
        },
        'THERMAL_DRIFT': {
            'prior': 0.10,
            'description': 'Thermal expansion causing gradual positional drift',
            'indicators': {
                'position_drift': 'gradual',
                'temperature_correlation': 'positive',
                'temperature_trend': 'increasing',
            },
            'mechanism': (
                'Heat from cutting and spindle bearings -> thermal expansion of '
                'spindle housing and column -> gradual shift in tool center point -> '
                'dimensional errors that grow over time'
            ),
            'verification': [
                'Measure part dimensions at start vs end of operation',
                'Monitor spindle temperature with contact thermometer',
                'Check dimensional drift direction against thermal growth model',
                'Compare error magnitude with thermal expansion coefficient',
            ],
        },
        'COOLANT_FAILURE': {
            'prior': 0.05,
            'description': 'Coolant system failure causing thermal damage',
            'indicators': {
                'temperature_spike': 'sudden',
                'coolant_flow': 'low',
                'tool_wear_rate': 'accelerating',
            },
            'mechanism': (
                'Loss of coolant flow -> inadequate heat removal from cutting zone -> '
                'rapid temperature rise at tool-chip interface -> accelerated diffusion '
                'wear and potential thermal cracking of tool'
            ),
            'verification': [
                'Check coolant pump operation and flow rate',
                'Inspect coolant nozzle for blockage',
                'Verify coolant concentration and condition',
                'Examine tool for thermal discoloration',
            ],
        },
        'SPINDLE_BEARING': {
            'prior': 0.05,
            'description': 'Spindle bearing degradation causing runout',
            'indicators': {
                'vibration_frequency': 'high',
                'runout': 'increasing',
                'vibration_pattern': 'periodic',
            },
            'mechanism': (
                'Bearing surface degradation -> increased radial play -> higher '
                'runout at tool tip -> high-frequency vibration at bearing defect '
                'frequencies (BPFO/BPFI) -> poor surface finish and dimensional errors'
            ),
            'verification': [
                'Measure spindle runout with dial indicator at tool tip',
                'Perform vibration spectrum analysis for bearing defect frequencies',
                'Check spindle bearing preload',
                'Listen for abnormal spindle noise at various RPMs',
            ],
        },
        'MATERIAL_DEFECT': {
            'prior': 0.05,
            'description': 'Material inclusion or hard spot causing force spike',
            'indicators': {
                'force_spike': 'sudden',
                'location_specific': 'true',
                'force_pattern': 'localized',
            },
            'mechanism': (
                'Hard inclusion or void in workpiece material -> sudden change in '
                'specific cutting force when tool encounters defect -> force spike '
                'at specific workpiece coordinates -> potential tool damage'
            ),
            'verification': [
                'Note exact workpiece coordinates of the force spike',
                'Inspect workpiece material at that location',
                'Check material certification for composition',
                'Run a light finishing pass over the area to confirm',
            ],
        },
        'PROGRAMMING_ERROR': {
            'prior': 0.08,
            'description': 'G-code programming error causing consistent anomaly',
            'indicators': {
                'block_consistency': 'true',
                'gcode_block': 'repeating',
                'anomaly_recurrence': 'same_location',
            },
            'mechanism': (
                'Incorrect feed rate, speed, or toolpath in specific G-code block -> '
                'anomaly occurs at same program location on every cycle -> consistent '
                'and repeatable error pattern'
            ),
            'verification': [
                'Identify the G-code block number where anomaly occurs',
                'Review the G-code for that block and surrounding blocks',
                'Verify feed rate and spindle speed are appropriate for the operation',
                'Run the program in single-block mode through the affected section',
            ],
        },
        'FIXTURING_ISSUE': {
            'prior': 0.07,
            'description': 'Workholding problem causing intermittent vibration',
            'indicators': {
                'vibration_pattern': 'intermittent',
                'position_variation': 'high',
                'vibration_amplitude': 'variable',
            },
            'mechanism': (
                'Insufficient clamping force or worn fixture -> workpiece micro-movement '
                'under cutting forces -> intermittent vibration when cutting forces '
                'exceed friction -> position variation between parts'
            ),
            'verification': [
                'Check fixture clamping force with torque wrench',
                'Inspect fixture contact surfaces for wear',
                'Verify workpiece seating with feeler gauge',
                'Try increasing clamping pressure and re-running',
            ],
        },
    }

    def __init__(self) -> None:
        # Operator-supplied evidence keyed by cause_id
        self._operator_evidence: Dict[str, List[EvidenceItem]] = {}

    def analyze_root_cause(
        self, anomaly_data: dict, history: list,
    ) -> List[RootCauseCandidate]:
        """Produce a ranked list of root cause candidates for an anomaly.

        Args:
            anomaly_data: Dict with sensor readings and anomaly metadata.
                          Keys may include force_trend, vibration_trend,
                          vibration_pattern, temperature_spike, etc.
            history: List of past ExplanationRecord objects.

        Returns:
            List of RootCauseCandidate sorted by probability (descending).
        """
        candidates: List[RootCauseCandidate] = []

        for cause_id, template in self.CAUSE_TEMPLATES.items():
            evidence_items: List[EvidenceItem] = []
            now = time.time()

            # --- Collect evidence from anomaly data fields ---
            for indicator_field, expected_value in template['indicators'].items():
                actual = anomaly_data.get(indicator_field)
                if actual is not None:
                    matches = self._value_matches(actual, expected_value)
                    strength = self._compute_evidence_strength(
                        actual, expected_value,
                    )
                    evidence_items.append(EvidenceItem(
                        source='sensor',
                        description=(
                            f'{indicator_field} is {actual} '
                            f'(expected {expected_value} for {cause_id})'
                        ),
                        strength=strength,
                        supports=matches,
                        timestamp=now,
                    ))

            # --- History evidence: recurring cause boosts probability ---
            history_count = self._count_history_matches(cause_id, history)
            if history_count > 0:
                hist_strength = min(1.0, history_count * 0.15)
                evidence_items.append(EvidenceItem(
                    source='history',
                    description=(
                        f'{cause_id} has occurred {history_count} times '
                        f'in recent history'
                    ),
                    strength=hist_strength,
                    supports=True,
                    timestamp=now,
                ))

            # --- Operator-supplied evidence ---
            operator_ev = self._operator_evidence.get(cause_id, [])
            evidence_items.extend(operator_ev)

            # --- Score the candidate ---
            supporting = [e for e in evidence_items if e.supports]
            contradicting = [e for e in evidence_items if not e.supports]
            sup_score = sum(e.strength for e in supporting)
            con_score = sum(e.strength for e in contradicting)
            net = sup_score - con_score

            # Bayesian-style update: prior × likelihood
            prior = template['prior']
            # Likelihood is a sigmoid-like function of the net evidence score
            likelihood = 1.0 / (1.0 + math.exp(-2.0 * net))
            probability = prior * likelihood
            # Normalize will happen after all candidates are scored

            candidates.append(RootCauseCandidate(
                cause_id=cause_id,
                description=template['description'],
                probability=probability,
                evidence=evidence_items,
                supporting_count=len(supporting),
                contradicting_count=len(contradicting),
                net_score=net,
                mechanism=template['mechanism'],
                verification_steps=list(template['verification']),
            ))

        # Normalize probabilities so they sum to 1
        total_prob = sum(c.probability for c in candidates)
        if total_prob > 0:
            for c in candidates:
                c.probability = c.probability / total_prob

        # Sort by probability descending
        candidates.sort(key=lambda c: c.probability, reverse=True)
        return candidates

    def add_operator_evidence(
        self, cause_id: str, evidence: 'EvidenceItem',
    ) -> None:
        """Add operator-supplied evidence for a specific cause."""
        self._operator_evidence.setdefault(cause_id, []).append(evidence)

    def get_verification_plan(self, cause_id: str) -> List[str]:
        """Return verification steps for a given cause."""
        template = self.CAUSE_TEMPLATES.get(cause_id)
        if template is None:
            return []
        return list(template['verification'])

    # -- Private helpers ---------------------------------------------------

    @staticmethod
    def _value_matches(actual: Any, expected: str) -> bool:
        """Check whether an actual anomaly data value matches the expected condition."""
        if isinstance(actual, str):
            return actual.lower() == expected.lower()
        if isinstance(actual, (int, float)):
            # Numeric: 'high' means > 0.7, 'low' means < 0.3, etc.
            thresholds = {
                'high': lambda v: v > 0.7,
                'low': lambda v: v < 0.3,
                'increasing': lambda v: v > 0,
                'accelerating': lambda v: v > 0.5,
            }
            fn = thresholds.get(expected.lower())
            if fn is not None:
                return fn(actual)
        # Boolean-like
        if isinstance(actual, bool):
            return actual == (expected.lower() == 'true')
        return str(actual).lower() == expected.lower()

    @staticmethod
    def _compute_evidence_strength(actual: Any, expected: str) -> float:
        """Compute how strongly the actual value bears on the hypothesis."""
        if isinstance(actual, (int, float)):
            # Strength proportional to magnitude for numeric values
            return min(1.0, max(0.1, abs(float(actual))))
        # For string/bool matches, moderate fixed strength
        return 0.6

    @staticmethod
    def _count_history_matches(cause_id: str, history: list) -> int:
        """Count how many historical records relate to this cause template."""
        # Map cause_ids to the anomaly types they are most associated with
        cause_to_anomaly = {
            'TOOL_WEAR': 'tool_wear_anomaly',
            'CHATTER': 'vibration_anomaly',
            'THERMAL_DRIFT': 'thermal_anomaly',
            'COOLANT_FAILURE': 'thermal_anomaly',
            'SPINDLE_BEARING': 'vibration_anomaly',
            'MATERIAL_DEFECT': 'force_anomaly',
            'PROGRAMMING_ERROR': None,  # matches any repeating anomaly
            'FIXTURING_ISSUE': 'vibration_anomaly',
        }
        target_type = cause_to_anomaly.get(cause_id)
        count = 0
        for record in history:
            rec_type = getattr(record, 'anomaly_type', None)
            if target_type is None:
                # PROGRAMMING_ERROR: count any record
                count += 1
            elif rec_type == target_type:
                count += 1
        return count


@dataclass
class FeatureContribution:
    """A single feature's contribution to a decision."""
    feature_name: str
    value: float
    contribution_pct: float
    threshold: Optional[float] = None
    direction: str = 'above'
    confidence_interval: float = 0.0  # 0-1, lower means more confident


@dataclass
class ExplanationRecord:
    """Internal record of a generated explanation."""
    timestamp: float
    anomaly_type: str
    machine_id: str
    severity: float
    summary: str
    detail: str
    counterfactual: str
    features: List[FeatureContribution]


class ExplanationGeneratorNode(MiracleLifecycleNode):
    """Generates multi-level explanations for autonomous AI decisions.

    Parameters:
        detail_level (str): Default detail level ('brief', 'medium', 'detailed').
        history_size (int): Number of past explanations to retain.
        machine_ids (str): Comma-separated machine IDs to monitor.

    Subscribed Topics:
        /miracle/{machine_id}/anomaly (AnomalyAlert)

    Published Topics:
        /miracle/cognitive/explanations (Explanation)
    """

    _ANOMALY_TEMPLATES = {
        'vibration_anomaly': {
            'summary': 'Abnormal vibration detected on {machine_id} ({severity_pct}% severity)',
            'detail_prefix': 'Vibration levels exceeded normal operating envelope.',
            'counterfactual': 'If vibration remained below {threshold}, no alert would trigger. '
                            'Consider reducing spindle speed by ~{reduction}% or checking tool runout.',
            'typical_features': ['rms_acceleration', 'peak_frequency', 'spindle_speed', 'depth_of_cut'],
        },
        'force_anomaly': {
            'summary': 'Cutting force anomaly on {machine_id} ({severity_pct}% severity)',
            'detail_prefix': 'Cutting forces deviated from predicted Altintas model values.',
            'counterfactual': 'If feed rate were reduced by {reduction}%, forces would return to '
                            'expected range. Alternatively, check for tool wear or material hardness variation.',
            'typical_features': ['force_fx', 'force_fy', 'force_fz', 'feed_rate', 'tool_wear_vb'],
        },
        'thermal_anomaly': {
            'summary': 'Thermal anomaly detected on {machine_id} ({severity_pct}% severity)',
            'detail_prefix': 'Temperature exceeded thermal model predictions.',
            'counterfactual': 'With {reduction}% more coolant flow or {reduction}% lower cutting speed, '
                            'temperature would stay within limits.',
            'typical_features': ['tool_temperature', 'spindle_temperature', 'coolant_flow', 'cutting_speed'],
        },
        'tool_wear_anomaly': {
            'summary': 'Accelerated tool wear on {machine_id} ({severity_pct}% severity)',
            'detail_prefix': 'Tool wear rate exceeds Taylor model predictions for current conditions.',
            'counterfactual': 'Reducing cutting speed by {reduction}% would extend tool life by '
                            'approximately {life_extension} minutes based on Taylor equation.',
            'typical_features': ['wear_vb', 'cutting_speed', 'feed_per_tooth', 'depth_of_cut'],
        },
        'surface_quality_anomaly': {
            'summary': 'Surface finish degradation on {machine_id} ({severity_pct}% severity)',
            'detail_prefix': 'Surface roughness (Ra) exceeded acceptable limits.',
            'counterfactual': 'A sharper tool (VB < 0.15mm) or {reduction}% lower feed rate '
                            'would produce acceptable surface finish.',
            'typical_features': ['surface_ra', 'tool_wear_vb', 'feed_rate', 'spindle_speed'],
        },
    }

    _DEFAULT_TEMPLATE = {
        'summary': 'Anomaly "{anomaly_type}" on {machine_id} ({severity_pct}% severity)',
        'detail_prefix': 'An anomaly was detected outside normal parameters.',
        'counterfactual': 'Adjusting operating parameters may prevent recurrence.',
        'typical_features': ['sensor_reading', 'threshold'],
    }

    # ---- Causal-inference integration ------------------------------------

    # Maps anomaly_type strings to causal graph node names.
    _ANOMALY_TO_EFFECTS: Dict[str, str] = {
        'vibration_anomaly': 'Vibration',
        'force_anomaly': 'ToolWear',
        'thermal_anomaly': 'ThermalExpansion',
        'tool_wear_anomaly': 'ToolWear',
        'surface_quality_anomaly': 'SurfaceRoughnessIncrease',
    }

    # Maps root-cause node names to actionable counterfactual text.
    _CAUSAL_COUNTERFACTUALS: Dict[str, str] = {
        'HighFeedRate': 'Reducing the feed rate would lower tool stress and prevent downstream wear effects.',
        'HighSpindleSpeed': 'Lowering spindle speed would reduce thermal expansion and improve dimensional accuracy.',
        'ImproperToolpath': 'Correcting the toolpath strategy would eliminate chatter and improve surface finish.',
        'WornBearing': 'Inspecting and replacing the worn bearing would eliminate the vibration source.',
        'LowCoolant': 'Restoring adequate coolant flow would prevent thermal damage to the workpiece.',
        'ToolWear': 'Replacing the worn tool would restore cutting performance and surface quality.',
    }

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'explanation_generator',
            criticality=self.CRITICALITY_LOW,
            **kwargs,
        )
        self._history: List[ExplanationRecord] = []
        self._history_lock = threading.Lock()
        self._history_size: int = 500
        self._detail_level: str = 'medium'
        self._explanation_pub = None

        # Operator feedback integration
        self._feedback_confidence: Dict[str, float] = {}
        self._feedback_ratings: Dict[str, List[float]] = {}
        self._feedback_detail_overrides: Dict[str, str] = {}

        # Root cause analyzer for evidence-based ranking
        self._root_cause_analyzer = RootCauseAnalyzer()

        # G-code context per machine for enriching explanations
        self._gcode_contexts: Dict[str, Dict[str, Any]] = {}

        # Causal links mirroring causal_inference._init_causal_model().
        # Stored as effect -> (cause, strength) for backward tracing.
        self._causal_links: Dict[str, Tuple[str, float]] = {
            'ToolWear': ('HighFeedRate', 0.7),
            'SurfaceRoughnessIncrease': ('ToolWear', 0.8),
            'ThermalExpansion': ('HighSpindleSpeed', 0.6),
            'Chatter': ('ImproperToolpath', 0.75),
            'Vibration': ('WornBearing', 0.85),
            'ThermalDamage': ('LowCoolant', 0.9),
        }

    def _do_configure(self) -> TransitionCallbackReturn:
        params = self.declare_and_validate_parameters({
            'detail_level': {
                'default': 'medium', 'type': str,
                'choices': ['brief', 'medium', 'detailed'],
            },
            'history_size': {
                'default': 500, 'type': int, 'range': (10, 10000),
            },
            'machine_ids': {
                'default': 'cnc1,cnc2,cnc3', 'type': str,
            },
        })

        self._detail_level = params['detail_level']
        self._history_size = params['history_size']
        machine_ids = self.get_machine_ids(params)

        self.create_multi_machine_subscriptions(
            AnomalyAlert, 'anomaly', self._on_anomaly,
            QoSProfiles.alert(), machine_ids,
        )

        self._explanation_pub = self.create_publisher(
            Explanation, '/miracle/cognitive/explanations',
            QoSProfiles.alert(),
        )

        self.get_logger().info(
            f"Explanation generator configured (level={self._detail_level}, "
            f"history={self._history_size})"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("Explanation generator activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        explanation = self.generate_explanation(
            anomaly_type=msg.anomaly_type,
            machine_id=msg.machine_id,
            severity=msg.severity,
            contributing_factors=msg.contributing_factors if hasattr(msg, 'contributing_factors') else [],
            recommended_action=msg.recommended_action,
        )

        exp_msg = Explanation()
        exp_msg.timestamp = self.get_clock().now().to_msg()
        exp_msg.summary = explanation.summary
        exp_msg.detail = explanation.detail
        exp_msg.counterfactual = explanation.counterfactual
        exp_msg.contributing_features = [f.feature_name for f in explanation.features]
        exp_msg.feature_contributions = [f.contribution_pct for f in explanation.features]
        exp_msg.model_confidence = msg.severity
        # Use the average confidence interval from feature contributions
        if explanation.features:
            exp_msg.confidence_interval = sum(
                f.confidence_interval for f in explanation.features
            ) / len(explanation.features)
        self._explanation_pub.publish(exp_msg)

    def generate_explanation(
        self, anomaly_type: str, machine_id: str, severity: float,
        contributing_factors: Optional[List[str]] = None,
        recommended_action: str = '',
    ) -> ExplanationRecord:
        template = self._ANOMALY_TEMPLATES.get(anomaly_type, self._DEFAULT_TEMPLATE)
        severity_pct = int(severity * 100)

        # Compute reduction based on how far severity exceeds normal threshold
        if severity > 0.5:
            reduction = min(50, max(5, int((severity - 0.5) / 0.5 * 40)))
        else:
            reduction = max(5, int(severity * 20))

        life_extension = int((1.0 - severity) * 45)

        fmt_kwargs = {
            'machine_id': machine_id,
            'severity_pct': severity_pct,
            'anomaly_type': anomaly_type,
            'threshold': f'{severity * 0.8:.2f}',
            'reduction': reduction,
            'life_extension': life_extension,
        }

        summary = template['summary'].format(**fmt_kwargs)
        counterfactual = template['counterfactual'].format(**fmt_kwargs)

        # Enhance counterfactual with causal root-cause insight
        causal_result = self._build_causal_chain(anomaly_type, severity)
        if causal_result is not None:
            chain, confidence = causal_result
            root_cause = chain[0]
            causal_cf = self._CAUSAL_COUNTERFACTUALS.get(root_cause)
            if causal_cf:
                counterfactual += f' Causal insight ({confidence:.0%} confidence): {causal_cf}'

        # Add temporal context from historical resolution data
        temporal_context = self._get_temporal_resolution_context(
            machine_id, anomaly_type, reduction
        )
        if temporal_context:
            counterfactual += f' {temporal_context}'

        # Enrich counterfactual with G-code context
        gcode_ctx = self._gcode_contexts.get(machine_id)
        if gcode_ctx:
            block_idx = gcode_ctx.get('block_index', 0)
            feed = gcode_ctx.get('feed_rate', 0)
            # Estimate force reduction from feed rate reduction
            force_reduction_estimate = min(40, max(5, int(reduction * 0.8)))
            counterfactual += (
                f' Reducing feed rate for blocks {block_idx}-{block_idx + 5} '
                f'would have reduced force by ~{force_reduction_estimate}%.'
            )

        features = self._build_feature_contributions(
            anomaly_type, severity, contributing_factors, template,
            recommended_action=recommended_action,
        )

        # Adjust feature confidence intervals based on operator feedback
        feedback_conf = self._feedback_confidence.get(anomaly_type)
        if feedback_conf is not None:
            for f in features:
                # High feedback confidence (operators rate highly) reduces CI
                # Low feedback confidence (poor ratings) increases CI
                adjustment = (1.0 - feedback_conf) * 0.2
                f.confidence_interval = round(
                    min(1.0, f.confidence_interval + adjustment), 3
                )

        # Apply detail level override from feedback
        effective_detail_level = self._feedback_detail_overrides.get(
            anomaly_type, self._detail_level
        )

        detail = self._build_detail(
            template['detail_prefix'], features, severity,
            recommended_action, machine_id, anomaly_type
        )

        record = ExplanationRecord(
            timestamp=time.time(),
            anomaly_type=anomaly_type,
            machine_id=machine_id,
            severity=severity,
            summary=summary,
            detail=detail,
            counterfactual=counterfactual,
            features=features,
        )

        with self._history_lock:
            self._history.append(record)
            if len(self._history) > self._history_size:
                self._history = self._history[-self._history_size:]

        return record

    # Mapping from recommended_action keywords to top contributing feature
    _ACTION_TO_TOP_FEATURE = {
        'reduce feed': 'feed_rate',
        'lower feed': 'feed_rate',
        'replace tool': 'tool_wear_vb',
        'change tool': 'tool_wear_vb',
        'reduce speed': 'cutting_speed',
        'lower speed': 'cutting_speed',
    }

    def _build_feature_contributions(
        self, anomaly_type: str, severity: float,
        contributing_factors: Optional[List[str]], template: dict,
        recommended_action: str = '',
    ) -> List[FeatureContribution]:
        features = []
        factor_names = list(contributing_factors or template.get('typical_features', []))
        if not factor_names:
            return features

        # Derive top contributor from recommended_action if available
        if recommended_action:
            action_lower = recommended_action.lower()
            for action_key, feature_name in self._ACTION_TO_TOP_FEATURE.items():
                if action_key in action_lower:
                    # Move the matching feature to front, or insert it
                    if feature_name in factor_names:
                        factor_names.remove(feature_name)
                    factor_names.insert(0, feature_name)
                    break

        # Compute variance-based confidence from history
        severity_variance = self._compute_severity_variance(anomaly_type)

        total = len(factor_names)
        raw_pcts = []
        for i, name in enumerate(factor_names):
            if i == 0:
                # Primary factor: severity-weighted contribution
                pct = severity * 50.0
            elif i == 1:
                # Secondary factor: inverse-severity blend
                pct = (1.0 - severity) * 30.0 + severity * 25.0
            else:
                # Remaining: diminishing geometric series (each gets 60% of previous)
                if not raw_pcts:
                    pct = 10.0
                else:
                    prev = raw_pcts[-1] if len(raw_pcts) >= 2 else raw_pcts[-1]
                    pct = max(2.0, prev * 0.6)
            raw_pcts.append(pct)

        # Confidence interval: higher variance in history = lower confidence
        ci = min(1.0, severity_variance * 2.0) if severity_variance > 0 else 0.1

        for i, name in enumerate(factor_names):
            features.append(FeatureContribution(
                feature_name=name,
                value=severity * (1.0 - i * 0.15),
                contribution_pct=round(raw_pcts[i], 1),
                threshold=severity * 0.7 if i == 0 else None,
                direction='above',
                confidence_interval=round(ci, 3),
            ))

        # Normalize to 100%
        total_pct = sum(f.contribution_pct for f in features)
        if total_pct > 0:
            for f in features:
                f.contribution_pct = round(f.contribution_pct / total_pct * 100, 1)

        features.sort(key=lambda f: f.contribution_pct, reverse=True)
        return features

    def _compute_severity_variance(self, anomaly_type: str) -> float:
        """Compute variance of severity for historical records of the same anomaly type."""
        with self._history_lock:
            severities = [
                r.severity for r in self._history
                if r.anomaly_type == anomaly_type
            ]
        if len(severities) < 2:
            return 0.0
        mean = sum(severities) / len(severities)
        variance = sum((s - mean) ** 2 for s in severities) / len(severities)
        return variance

    # ---- Causal chain helpers --------------------------------------------

    def _build_causal_chain(
        self, anomaly_type: str, severity: float,
    ) -> Optional[Tuple[List[str], float]]:
        """Trace backward through causal links to find the root cause chain.

        Returns ``(chain, confidence)`` where *chain* is a list of node names
        from root cause to the observed effect (e.g.
        ``['HighFeedRate', 'ToolWear', 'SurfaceRoughnessIncrease']``) and
        *confidence* is the product of the individual link strengths.

        Returns ``None`` when the anomaly type has no mapping in the causal
        graph.
        """
        effect = self._ANOMALY_TO_EFFECTS.get(anomaly_type)
        if effect is None:
            return None

        chain: List[str] = [effect]
        confidence: float = 1.0

        current = effect
        visited = {current}
        while current in self._causal_links:
            cause, strength = self._causal_links[current]
            if cause in visited:
                break  # prevent cycles
            confidence *= strength
            chain.insert(0, cause)
            visited.add(cause)
            current = cause

        if len(chain) < 2:
            return None

        return chain, confidence

    def _generate_detail_explanation(
        self, anomaly_type: str, machine_id: str, severity: float,
        anomaly_data: Optional[dict] = None,
    ) -> str:
        """Generate root-cause-ranked detail section using the RootCauseAnalyzer."""
        if anomaly_data is None:
            anomaly_data = {}

        with self._history_lock:
            history = list(self._history)

        candidates = self._root_cause_analyzer.analyze_root_cause(
            anomaly_data, history,
        )

        if not candidates:
            return ''

        lines = ['\nRoot cause analysis (ranked by probability):']
        for i, candidate in enumerate(candidates[:5], 1):
            lines.append(
                f'  {i}. [{candidate.cause_id}] {candidate.description} '
                f'(probability: {candidate.probability:.1%})'
            )
            lines.append(
                f'     Evidence: {candidate.supporting_count} supporting, '
                f'{candidate.contradicting_count} contradicting '
                f'(net score: {candidate.net_score:+.2f})'
            )
            lines.append(f'     Mechanism: {candidate.mechanism}')

        # Top candidate verification steps
        top = candidates[0]
        lines.append(f'\nVerification plan for top cause ({top.cause_id}):')
        for step in top.verification_steps:
            lines.append(f'  - {step}')

        return '\n'.join(lines)

    def _build_detail(
        self, prefix: str, features: List[FeatureContribution],
        severity: float, recommended_action: str,
        machine_id: str, anomaly_type: str,
        anomaly_data: Optional[dict] = None,
    ) -> str:
        lines = [prefix]

        if features:
            lines.append('\nContributing factors (ranked by importance):')
            for i, f in enumerate(features, 1):
                threshold_str = f' (threshold: {f.threshold:.3f})' if f.threshold else ''
                ci_str = f', CI=+/-{f.confidence_interval:.1%}' if f.confidence_interval > 0 else ''
                lines.append(
                    f'  {i}. {f.feature_name}: {f.contribution_pct:.1f}% contribution '
                    f'(value={f.value:.3f}{threshold_str}{ci_str})'
                )

        # Causal analysis section
        causal_result = self._build_causal_chain(anomaly_type, severity)
        if causal_result is not None:
            chain, confidence = causal_result
            chain_str = ' \u2192 '.join(chain)
            root_cause = chain[0]
            lines.append('\nCausal analysis:')
            lines.append(f'  Root cause hypothesis: {root_cause} (confidence: {confidence:.0%})')
            lines.append(f'  Causal chain: {chain_str}')
            # Show individual link strengths
            link_details = []
            for i in range(len(chain) - 1):
                src, dst = chain[i], chain[i + 1]
                if dst in self._causal_links:
                    _, strength = self._causal_links[dst]
                    link_details.append(f'{src}\u2192{dst} ({strength:.0%})')
            if link_details:
                lines.append(f'  Link strengths: {", ".join(link_details)}')

        # Root cause ranking section
        root_cause_detail = self._generate_detail_explanation(
            anomaly_type, machine_id, severity, anomaly_data,
        )
        if root_cause_detail:
            lines.append(root_cause_detail)

        # G-code execution context
        gcode_ctx = self._gcode_contexts.get(machine_id)
        if gcode_ctx:
            lines.append(f'\nG-code execution context:')
            lines.append(
                f'  During execution of: {gcode_ctx.get("gcode_line", "N/A")} '
                f'(block {gcode_ctx.get("block_index", "?")})'
            )
            lines.append(
                f'  Operation: {gcode_ctx.get("operation_type", "UNKNOWN")} '
                f'at {gcode_ctx.get("feed_rate", 0):.0f} mm/min, '
                f'{gcode_ctx.get("spindle_rpm", 0):.0f} RPM'
            )
            lines.append(
                f'  Program progress: {gcode_ctx.get("elapsed_program_pct", 0):.1f}%'
            )

        context = self._get_historical_context(machine_id, anomaly_type)
        if context:
            lines.append(f'\nHistorical context: {context}')

        if recommended_action:
            lines.append(f'\nRecommended action: {recommended_action}')

        return '\n'.join(lines)

    def _get_historical_context(self, machine_id: str, anomaly_type: str) -> str:
        with self._history_lock:
            recent = [
                r for r in self._history
                if r.machine_id == machine_id and r.anomaly_type == anomaly_type
            ]

        if not recent:
            return 'First occurrence of this anomaly type on this machine.'

        count = len(recent)
        if count == 1:
            elapsed = time.time() - recent[0].timestamp
            return f'Previously seen once ({elapsed:.0f}s ago).'

        avg_severity = sum(r.severity for r in recent) / count
        latest = recent[-1]
        elapsed = time.time() - latest.timestamp
        return (
            f'Seen {count} times on this machine. '
            f'Average severity: {avg_severity:.0%}. '
            f'Last occurrence: {elapsed:.0f}s ago.'
        )

    def _get_temporal_resolution_context(
        self, machine_id: str, anomaly_type: str, reduction: int,
    ) -> str:
        """Build temporal context about past resolutions for counterfactuals."""
        with self._history_lock:
            past = [
                r for r in self._history
                if r.machine_id == machine_id and r.anomaly_type == anomaly_type
            ]

        if len(past) < 2:
            return ''

        # Estimate resolution rate: count how often severity decreased after occurrence
        resolved_count = 0
        for i in range(1, len(past)):
            if past[i].severity < past[i - 1].severity:
                resolved_count += 1

        total_transitions = len(past) - 1
        resolution_pct = int(resolved_count / total_transitions * 100)

        return (
            f'Based on the last {len(past)} occurrences, reducing by '
            f'{reduction}% resolved the issue {resolution_pct}% of the time.'
        )

    def set_gcode_context(self, machine_id: str, context: Dict[str, Any]) -> None:
        """Set the current G-code execution context for a machine.

        Args:
            machine_id: Machine identifier.
            context: Dict with keys: program_name, block_index, gcode_line,
                     operation_type, feed_rate, spindle_rpm, depth_of_cut,
                     tool_id, elapsed_program_pct.
        """
        self._gcode_contexts[machine_id] = context

    def get_gcode_context(self, machine_id: str) -> Optional[Dict[str, Any]]:
        """Return the current G-code context for a machine, or None."""
        return self._gcode_contexts.get(machine_id)

    def explain_anomaly(self, anomaly_type: str, factors: list) -> str:
        record = self.generate_explanation(
            anomaly_type=anomaly_type, machine_id='unknown',
            severity=0.5, contributing_factors=factors,
        )
        return record.summary

    def explain_optimization(self, action: str, reasoning: str) -> str:
        return f"Optimization '{action}': {reasoning}"

    def update_from_feedback(self, anomaly_type: str, rating: float) -> None:
        """Update explanation quality tracking from operator feedback.

        Args:
            anomaly_type: The anomaly type the feedback relates to.
            rating: Operator rating on a 1-5 scale.
        """
        ratings = self._feedback_ratings.setdefault(anomaly_type, [])
        ratings.append(rating)
        # Keep last 100 ratings per anomaly type
        if len(ratings) > 100:
            self._feedback_ratings[anomaly_type] = ratings[-100:]

        avg = sum(self._feedback_ratings[anomaly_type]) / len(self._feedback_ratings[anomaly_type])
        self._feedback_confidence[anomaly_type] = avg / 5.0  # normalize to 0-1

        # Adjust detail level override based on feedback quality
        if avg < 2.0:
            self._feedback_detail_overrides[anomaly_type] = 'brief'
        elif avg > 4.0:
            self._feedback_detail_overrides[anomaly_type] = 'detailed'
        else:
            self._feedback_detail_overrides.pop(anomaly_type, None)

    @property
    def history(self) -> List[ExplanationRecord]:
        with self._history_lock:
            return list(self._history)


@dataclass
class HypothesisTest:
    """Result of a statistical hypothesis test for manufacturing process analysis."""
    test_name: str
    null_hypothesis: str
    alternative_hypothesis: str
    test_statistic: float
    p_value: float
    reject_null: bool
    confidence_level: float
    sample_size: int
    conclusion: str


class HypothesisTestEngine:
    """Statistical hypothesis testing engine for manufacturing process analysis.

    Performs common statistical tests (t-tests, F-test, chi-squared) using
    pure-Python implementations — no scipy dependency required.  Approximations
    for the t-distribution and chi-squared CDF are based on well-known series
    expansions and the regularised incomplete beta / gamma functions.
    """

    # ------------------------------------------------------------------
    # Mathematical helpers (pure Python, no scipy)
    # ------------------------------------------------------------------

    @staticmethod
    def _mean(data: List[float]) -> float:
        """Arithmetic mean."""
        return sum(data) / len(data)

    @staticmethod
    def _variance(data: List[float], ddof: int = 1) -> float:
        """Sample variance with *ddof* degrees-of-freedom correction."""
        n = len(data)
        if n <= ddof:
            return 0.0
        m = sum(data) / n
        return sum((x - m) ** 2 for x in data) / (n - ddof)

    @staticmethod
    def _std(data: List[float], ddof: int = 1) -> float:
        """Sample standard deviation."""
        return math.sqrt(HypothesisTestEngine._variance(data, ddof))

    # ---- Gamma / Beta helpers for CDF approximations -------------------

    @staticmethod
    def _ln_gamma(z: float) -> float:
        """Lanczos approximation of ln(Gamma(z)) for z > 0."""
        if z <= 0:
            return 0.0
        # Lanczos g=7, n=9 coefficients
        coeffs = [
            0.99999999999980993,
            676.5203681218851,
            -1259.1392167224028,
            771.32342877765313,
            -176.61502916214059,
            12.507343278686905,
            -0.13857109526572012,
            9.9843695780195716e-6,
            1.5056327351493116e-7,
        ]
        g = 7
        z_shifted = z - 1.0
        x = coeffs[0]
        for i in range(1, g + 2):
            x += coeffs[i] / (z_shifted + i)
        t = z_shifted + g + 0.5
        return (0.5 * math.log(2.0 * math.pi)
                + (z_shifted + 0.5) * math.log(t)
                - t
                + math.log(x))

    @staticmethod
    def _beta_cf(a: float, b: float, x: float) -> float:
        """Evaluate the continued fraction for the incomplete beta function.

        Numerical Recipes method (betacf).  Returns the CF value that should
        be multiplied by the front factor to yield I_x(a,b).
        """
        max_iter = 200
        eps = 3e-14
        tiny = 1e-30

        qab = a + b
        qap = a + 1.0
        qam = a - 1.0

        # First step of Lentz's method
        c = 1.0
        d = 1.0 - qab * x / qap
        if abs(d) < tiny:
            d = tiny
        d = 1.0 / d
        h = d

        for m in range(1, max_iter + 1):
            m2 = 2 * m

            # Even coefficient: d_{2m} = m(b-m)x / ((a+2m-1)(a+2m))
            aa = m * (b - m) * x / ((qam + m2) * (a + m2))
            d = 1.0 + aa * d
            if abs(d) < tiny:
                d = tiny
            c = 1.0 + aa / c
            if abs(c) < tiny:
                c = tiny
            d = 1.0 / d
            h *= d * c

            # Odd coefficient: d_{2m+1} = -(a+m)(a+b+m)x / ((a+2m)(a+2m+1))
            aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
            d = 1.0 + aa * d
            if abs(d) < tiny:
                d = tiny
            c = 1.0 + aa / c
            if abs(c) < tiny:
                c = tiny
            d = 1.0 / d
            delta = d * c
            h *= delta

            if abs(delta - 1.0) < eps:
                break

        return h

    @staticmethod
    def _regularised_incomplete_beta(a: float, b: float, x: float) -> float:
        """Regularised incomplete beta function I_x(a, b).

        Used to compute CDF of the t-distribution and F-distribution.
        """
        if x < 0.0 or x > 1.0:
            return 0.0
        if x == 0.0:
            return 0.0
        if x == 1.0:
            return 1.0

        ln_beta = (HypothesisTestEngine._ln_gamma(a)
                    + HypothesisTestEngine._ln_gamma(b)
                    - HypothesisTestEngine._ln_gamma(a + b))

        # Front factor: x^a * (1-x)^b / (a * B(a,b))
        front = math.exp(
            a * math.log(x) + b * math.log(1.0 - x) - ln_beta
        ) / a

        # Use symmetry relation when x > (a+1)/(a+b+2) for better CF convergence
        if x > (a + 1.0) / (a + b + 2.0):
            return 1.0 - HypothesisTestEngine._regularised_incomplete_beta(
                b, a, 1.0 - x,
            )

        return front * HypothesisTestEngine._beta_cf(a, b, x)

    @staticmethod
    def _t_cdf(t_val: float, df: float) -> float:
        """CDF of Student's t-distribution with *df* degrees of freedom."""
        if df <= 0:
            return 0.5
        x = df / (df + t_val * t_val)
        beta_val = 0.5 * HypothesisTestEngine._regularised_incomplete_beta(
            df / 2.0, 0.5, x,
        )
        if t_val >= 0:
            return 1.0 - beta_val
        return beta_val

    @staticmethod
    def _regularised_gamma_lower(a: float, x: float) -> float:
        """Lower regularised incomplete gamma function P(a, x).

        Uses series expansion for x < a + 1, continued fraction otherwise.
        """
        if x < 0.0:
            return 0.0
        if x == 0.0:
            return 0.0

        ln_ga = HypothesisTestEngine._ln_gamma(a)

        if x < a + 1.0:
            # Series expansion
            ap = a
            s = 1.0 / a
            delta = s
            for _ in range(200):
                ap += 1.0
                delta *= x / ap
                s += delta
                if abs(delta) < abs(s) * 1e-14:
                    break
            return s * math.exp(a * math.log(x) - x - ln_ga)
        else:
            # Continued fraction (Legendre)
            b_cf = x + 1.0 - a
            c = 1e30
            d = 1.0 / b_cf if abs(b_cf) > 1e-30 else 1e30
            f = d
            for i in range(1, 200):
                an = -i * (i - a)
                b_cf += 2.0
                d = an * d + b_cf
                if abs(d) < 1e-30:
                    d = 1e-30
                c = b_cf + an / c
                if abs(c) < 1e-30:
                    c = 1e-30
                d = 1.0 / d
                delta = d * c
                f *= delta
                if abs(delta - 1.0) < 1e-14:
                    break
            return 1.0 - f * math.exp(a * math.log(x) - x - ln_ga)

    @staticmethod
    def _chi2_cdf(x: float, k: float) -> float:
        """CDF of the chi-squared distribution with *k* degrees of freedom."""
        if x <= 0.0:
            return 0.0
        return HypothesisTestEngine._regularised_gamma_lower(k / 2.0, x / 2.0)

    @staticmethod
    def _f_cdf(f_val: float, d1: float, d2: float) -> float:
        """CDF of the F-distribution with (d1, d2) degrees of freedom."""
        if f_val <= 0.0:
            return 0.0
        x = d1 * f_val / (d1 * f_val + d2)
        return HypothesisTestEngine._regularised_incomplete_beta(
            d1 / 2.0, d2 / 2.0, x,
        )

    # ------------------------------------------------------------------
    # Public hypothesis tests
    # ------------------------------------------------------------------

    def t_test_one_sample(
        self,
        data: List[float],
        mu0: float = 0.0,
        alpha: float = 0.05,
    ) -> HypothesisTest:
        """One-sample t-test: is the population mean equal to *mu0*?

        Args:
            data: Sample observations.
            mu0: Hypothesised population mean under H0.
            alpha: Significance level.

        Returns:
            HypothesisTest result.
        """
        n = len(data)
        if n < 2:
            return HypothesisTest(
                test_name='one_sample_t_test',
                null_hypothesis=f'mu == {mu0}',
                alternative_hypothesis=f'mu != {mu0}',
                test_statistic=0.0,
                p_value=1.0,
                reject_null=False,
                confidence_level=1.0 - alpha,
                sample_size=n,
                conclusion='Insufficient data (n < 2)',
            )

        x_bar = self._mean(data)
        s = self._std(data, ddof=1)
        se = s / math.sqrt(n)
        t_stat = (x_bar - mu0) / se if se > 0 else 0.0
        df = n - 1

        # Two-tailed p-value
        p_value = 2.0 * (1.0 - self._t_cdf(abs(t_stat), df))
        reject = p_value < alpha

        if reject:
            conclusion = (
                f'Reject H0: sample mean ({x_bar:.4f}) differs significantly '
                f'from {mu0} (t={t_stat:.4f}, p={p_value:.6f})'
            )
        else:
            conclusion = (
                f'Fail to reject H0: no significant difference between '
                f'sample mean ({x_bar:.4f}) and {mu0} (t={t_stat:.4f}, p={p_value:.6f})'
            )

        return HypothesisTest(
            test_name='one_sample_t_test',
            null_hypothesis=f'mu == {mu0}',
            alternative_hypothesis=f'mu != {mu0}',
            test_statistic=t_stat,
            p_value=p_value,
            reject_null=reject,
            confidence_level=1.0 - alpha,
            sample_size=n,
            conclusion=conclusion,
        )

    def t_test_two_sample(
        self,
        data1: List[float],
        data2: List[float],
        alpha: float = 0.05,
    ) -> HypothesisTest:
        """Welch's two-sample t-test: do two populations have the same mean?

        Uses the Welch-Satterthwaite approximation for degrees of freedom
        (does not assume equal variances).

        Args:
            data1: First sample observations.
            data2: Second sample observations.
            alpha: Significance level.

        Returns:
            HypothesisTest result.
        """
        n1, n2 = len(data1), len(data2)
        total_n = n1 + n2

        if n1 < 2 or n2 < 2:
            return HypothesisTest(
                test_name='two_sample_t_test_welch',
                null_hypothesis='mu1 == mu2',
                alternative_hypothesis='mu1 != mu2',
                test_statistic=0.0,
                p_value=1.0,
                reject_null=False,
                confidence_level=1.0 - alpha,
                sample_size=total_n,
                conclusion='Insufficient data (each sample needs n >= 2)',
            )

        x1 = self._mean(data1)
        x2 = self._mean(data2)
        v1 = self._variance(data1, ddof=1)
        v2 = self._variance(data2, ddof=1)

        se = math.sqrt(v1 / n1 + v2 / n2) if (v1 / n1 + v2 / n2) > 0 else 1e-15
        t_stat = (x1 - x2) / se

        # Welch-Satterthwaite degrees of freedom
        num = (v1 / n1 + v2 / n2) ** 2
        denom = ((v1 / n1) ** 2 / (n1 - 1) + (v2 / n2) ** 2 / (n2 - 1))
        df = num / denom if denom > 0 else 1.0

        p_value = 2.0 * (1.0 - self._t_cdf(abs(t_stat), df))
        reject = p_value < alpha

        if reject:
            conclusion = (
                f'Reject H0: means differ significantly '
                f'(x1={x1:.4f}, x2={x2:.4f}, t={t_stat:.4f}, p={p_value:.6f})'
            )
        else:
            conclusion = (
                f'Fail to reject H0: no significant difference between means '
                f'(x1={x1:.4f}, x2={x2:.4f}, t={t_stat:.4f}, p={p_value:.6f})'
            )

        return HypothesisTest(
            test_name='two_sample_t_test_welch',
            null_hypothesis='mu1 == mu2',
            alternative_hypothesis='mu1 != mu2',
            test_statistic=t_stat,
            p_value=p_value,
            reject_null=reject,
            confidence_level=1.0 - alpha,
            sample_size=total_n,
            conclusion=conclusion,
        )

    def f_test_variance(
        self,
        data1: List[float],
        data2: List[float],
        alpha: float = 0.05,
    ) -> HypothesisTest:
        """F-test for equality of variances between two samples.

        Args:
            data1: First sample observations.
            data2: Second sample observations.
            alpha: Significance level.

        Returns:
            HypothesisTest result.
        """
        n1, n2 = len(data1), len(data2)
        total_n = n1 + n2

        if n1 < 2 or n2 < 2:
            return HypothesisTest(
                test_name='f_test_variance',
                null_hypothesis='sigma1^2 == sigma2^2',
                alternative_hypothesis='sigma1^2 != sigma2^2',
                test_statistic=0.0,
                p_value=1.0,
                reject_null=False,
                confidence_level=1.0 - alpha,
                sample_size=total_n,
                conclusion='Insufficient data (each sample needs n >= 2)',
            )

        v1 = self._variance(data1, ddof=1)
        v2 = self._variance(data2, ddof=1)

        # Ensure the larger variance is in the numerator
        if v1 >= v2:
            f_stat = v1 / v2 if v2 > 0 else float('inf')
            df1 = n1 - 1
            df2 = n2 - 1
        else:
            f_stat = v2 / v1 if v1 > 0 else float('inf')
            df1 = n2 - 1
            df2 = n1 - 1

        # Two-tailed p-value
        p_value = 2.0 * (1.0 - self._f_cdf(f_stat, df1, df2))
        p_value = min(p_value, 1.0)
        reject = p_value < alpha

        if reject:
            conclusion = (
                f'Reject H0: variances differ significantly '
                f'(F={f_stat:.4f}, p={p_value:.6f})'
            )
        else:
            conclusion = (
                f'Fail to reject H0: no significant difference in variances '
                f'(F={f_stat:.4f}, p={p_value:.6f})'
            )

        return HypothesisTest(
            test_name='f_test_variance',
            null_hypothesis='sigma1^2 == sigma2^2',
            alternative_hypothesis='sigma1^2 != sigma2^2',
            test_statistic=f_stat,
            p_value=p_value,
            reject_null=reject,
            confidence_level=1.0 - alpha,
            sample_size=total_n,
            conclusion=conclusion,
        )

    def chi_squared_goodness_of_fit(
        self,
        observed: List[float],
        expected: List[float],
        alpha: float = 0.05,
    ) -> HypothesisTest:
        """Chi-squared goodness-of-fit test.

        Tests whether observed frequencies match the expected distribution.

        Args:
            observed: Observed frequency counts.
            expected: Expected frequency counts.
            alpha: Significance level.

        Returns:
            HypothesisTest result.
        """
        n = len(observed)
        total_obs = int(sum(observed))

        if n != len(expected) or n < 2:
            return HypothesisTest(
                test_name='chi_squared_goodness_of_fit',
                null_hypothesis='observed matches expected distribution',
                alternative_hypothesis='observed does not match expected distribution',
                test_statistic=0.0,
                p_value=1.0,
                reject_null=False,
                confidence_level=1.0 - alpha,
                sample_size=total_obs,
                conclusion='Invalid input: observed and expected must have equal length >= 2',
            )

        chi2 = 0.0
        for o, e in zip(observed, expected):
            if e > 0:
                chi2 += (o - e) ** 2 / e

        df = n - 1
        p_value = 1.0 - self._chi2_cdf(chi2, df)
        reject = p_value < alpha

        if reject:
            conclusion = (
                f'Reject H0: observed distribution differs significantly '
                f'from expected (chi2={chi2:.4f}, df={df}, p={p_value:.6f})'
            )
        else:
            conclusion = (
                f'Fail to reject H0: observed distribution consistent with '
                f'expected (chi2={chi2:.4f}, df={df}, p={p_value:.6f})'
            )

        return HypothesisTest(
            test_name='chi_squared_goodness_of_fit',
            null_hypothesis='observed matches expected distribution',
            alternative_hypothesis='observed does not match expected distribution',
            test_statistic=chi2,
            p_value=p_value,
            reject_null=reject,
            confidence_level=1.0 - alpha,
            sample_size=total_obs,
            conclusion=conclusion,
        )

    def process_shift_detection(
        self,
        data: List[float],
        baseline_mean: float,
        baseline_std: float,
        alpha: float = 0.05,
    ) -> HypothesisTest:
        """Detect whether a manufacturing process has shifted from its baseline.

        Combines a one-sample t-test against the baseline mean with a check
        of how many standard deviations the sample mean has drifted.

        Args:
            data: Recent process measurements.
            baseline_mean: Historical baseline mean.
            baseline_std: Historical baseline standard deviation.
            alpha: Significance level.

        Returns:
            HypothesisTest result with shift-specific conclusion.
        """
        n = len(data)
        if n < 2:
            return HypothesisTest(
                test_name='process_shift_detection',
                null_hypothesis=f'process mean == {baseline_mean}',
                alternative_hypothesis=f'process has shifted from baseline mean {baseline_mean}',
                test_statistic=0.0,
                p_value=1.0,
                reject_null=False,
                confidence_level=1.0 - alpha,
                sample_size=n,
                conclusion='Insufficient data (n < 2)',
            )

        x_bar = self._mean(data)
        s = self._std(data, ddof=1)
        se = s / math.sqrt(n)
        t_stat = (x_bar - baseline_mean) / se if se > 0 else 0.0
        df = n - 1

        p_value = 2.0 * (1.0 - self._t_cdf(abs(t_stat), df))
        reject = p_value < alpha

        # Compute shift magnitude in baseline-std units
        shift_sigmas = abs(x_bar - baseline_mean) / baseline_std if baseline_std > 0 else 0.0

        if reject:
            direction = 'above' if x_bar > baseline_mean else 'below'
            conclusion = (
                f'Process shift detected: mean shifted {shift_sigmas:.2f} sigma '
                f'{direction} baseline (x_bar={x_bar:.4f}, baseline={baseline_mean:.4f}, '
                f't={t_stat:.4f}, p={p_value:.6f})'
            )
        else:
            conclusion = (
                f'No significant process shift: mean within expected range '
                f'(x_bar={x_bar:.4f}, baseline={baseline_mean:.4f}, '
                f'shift={shift_sigmas:.2f} sigma, t={t_stat:.4f}, p={p_value:.6f})'
            )

        return HypothesisTest(
            test_name='process_shift_detection',
            null_hypothesis=f'process mean == {baseline_mean}',
            alternative_hypothesis=f'process has shifted from baseline mean {baseline_mean}',
            test_statistic=t_stat,
            p_value=p_value,
            reject_null=reject,
            confidence_level=1.0 - alpha,
            sample_size=n,
            conclusion=conclusion,
        )

    def run_test(self, test_type: str, **kwargs: Any) -> HypothesisTest:
        """Dispatcher that runs the appropriate hypothesis test.

        Args:
            test_type: One of 'one_sample_t', 'two_sample_t', 'f_test',
                       'chi_squared', or 'process_shift'.
            **kwargs: Arguments forwarded to the selected test method.

        Returns:
            HypothesisTest result.

        Raises:
            ValueError: If *test_type* is not recognised.
        """
        dispatch: Dict[str, Any] = {
            'one_sample_t': self.t_test_one_sample,
            'two_sample_t': self.t_test_two_sample,
            'f_test': self.f_test_variance,
            'chi_squared': self.chi_squared_goodness_of_fit,
            'process_shift': self.process_shift_detection,
        }

        handler = dispatch.get(test_type)
        if handler is None:
            raise ValueError(
                f"Unknown test_type '{test_type}'. "
                f"Valid types: {', '.join(sorted(dispatch))}"
            )
        return handler(**kwargs)


# ---------------------------------------------------------------------------
# Natural Language Explanation Generator
# ---------------------------------------------------------------------------

@dataclass
class ExplanationContext:
    """Context surrounding a CNC event that needs explanation."""
    event_type: str
    severity: float
    parameters: Dict[str, Any]
    timestamp: float
    machine_id: str


@dataclass
class NLGExplanation:
    """A structured, human-readable explanation."""
    title: str
    summary: str
    details: List[str]
    recommendations: List[str]
    confidence: float
    audience: str  # 'operator' | 'engineer' | 'manager'


class NaturalLanguageExplainer:
    """Generates human-readable explanations for CNC events, alerts,
    and recommendations tailored to different audience levels.

    Supports three audience levels:
      - **operator**: concise, action-oriented language
      - **engineer**: includes technical root-cause detail
      - **manager**: high-level impact and cost/downtime focus

    Uses a template system keyed by alarm type with severity-based
    urgency wording.
    """

    # Severity thresholds mapped to urgency words
    _URGENCY_WORDS: List[Tuple[float, str]] = [
        (0.9, 'CRITICAL'),
        (0.7, 'HIGH'),
        (0.5, 'MODERATE'),
        (0.3, 'LOW'),
        (0.0, 'INFORMATIONAL'),
    ]

    # Alarm templates keyed by alarm type
    ALARM_TEMPLATES: Dict[str, Dict[str, Any]] = {
        'CHATTER': {
            'title': '{urgency}: Regenerative chatter detected',
            'summary': (
                'Regenerative chatter vibration has been detected on '
                '{machine_id}. This self-excited vibration can damage the '
                'tool and degrade surface finish.'
            ),
            'details_operator': [
                'Vibration is outside normal operating range.',
                'Surface finish may be affected on the current workpiece.',
            ],
            'details_engineer': [
                'Regenerative chatter at a non-tooth-passing frequency indicates '
                'instability in the cutting process stability lobe diagram.',
                'Phase shift between successive tooth passes causes exponential '
                'growth of vibration amplitude.',
                'Dominant frequency does not align with spindle harmonics.',
            ],
            'details_manager': [
                'Machine vibration issue may cause scrap parts if not addressed.',
                'Potential impact on delivery schedule for current batch.',
            ],
            'recommendations': [
                'Reduce spindle speed by 10-15% to move into a stable lobe.',
                'Reduce depth of cut to lower cutting force.',
                'Check tool stickout length and minimize if possible.',
            ],
        },
        'TOOL_WEAR': {
            'title': '{urgency}: Excessive tool wear detected',
            'summary': (
                'Tool wear on {machine_id} has exceeded the expected rate '
                'based on Taylor tool life predictions. Continued operation '
                'risks poor surface quality and potential tool breakage.'
            ),
            'details_operator': [
                'The current tool is wearing faster than expected.',
                'Part quality may drop if the tool is not replaced soon.',
            ],
            'details_engineer': [
                'Flank wear VB exceeds the threshold predicted by the Taylor '
                'tool life equation for the current cutting parameters.',
                'Accelerated wear may indicate incorrect speed/feed combination '
                'or workpiece material hardness variation.',
                'Crater wear pattern suggests high cutting temperature at the '
                'tool-chip interface.',
            ],
            'details_manager': [
                'Tool replacement is needed sooner than scheduled.',
                'Tooling cost may increase for this production run.',
                'Risk of unplanned downtime if tool breaks during cutting.',
            ],
            'recommendations': [
                'Replace the current tool with a fresh insert.',
                'Reduce cutting speed to extend remaining tool life.',
                'Verify workpiece material hardness matches the process plan.',
            ],
        },
        'THERMAL_DRIFT': {
            'title': '{urgency}: Thermal drift affecting positioning',
            'summary': (
                'Thermal expansion on {machine_id} is causing gradual '
                'positional drift. Dimensional accuracy of machined features '
                'may be compromised.'
            ),
            'details_operator': [
                'Machine is warming up and dimensions may be shifting.',
                'Check part dimensions more frequently during this period.',
            ],
            'details_engineer': [
                'Spindle and column thermal growth is causing TCP drift in '
                'the Z-axis beyond the compensation model prediction.',
                'Temperature gradient between spindle housing and machine '
                'base exceeds the steady-state assumption.',
                'Current thermal compensation coefficients may need recalibration.',
            ],
            'details_manager': [
                'Part dimensional accuracy may be affected.',
                'Additional inspection time may be required.',
                'Consider thermal stabilization period before critical features.',
            ],
            'recommendations': [
                'Allow the machine to reach thermal equilibrium before critical cuts.',
                'Enable thermal compensation if not already active.',
                'Measure and adjust tool length offset to compensate for drift.',
            ],
        },
        'FORCE_OVERLOAD': {
            'title': '{urgency}: Cutting force overload detected',
            'summary': (
                'Cutting forces on {machine_id} have exceeded safe operating '
                'limits. This may cause tool breakage, workpiece movement, or '
                'spindle damage.'
            ),
            'details_operator': [
                'Cutting forces are dangerously high.',
                'Stop the operation if forces continue to climb.',
            ],
            'details_engineer': [
                'Measured cutting forces exceed the Altintas mechanistic model '
                'prediction by a significant margin.',
                'Force spike pattern is consistent with a hard inclusion in the '
                'workpiece material or a dull cutting edge.',
                'Torque on the spindle is approaching the drive motor limit.',
            ],
            'details_manager': [
                'Risk of tool breakage which could damage the workpiece.',
                'Potential for expensive spindle repair if forces are not reduced.',
                'Production may need to be paused for safety.',
            ],
            'recommendations': [
                'Immediately reduce feed rate by 30-50%.',
                'Inspect the tool for damage or excessive wear.',
                'Check workpiece material for unexpected hard spots.',
            ],
        },
        'SURFACE_QUALITY': {
            'title': '{urgency}: Surface quality degradation',
            'summary': (
                'Surface roughness on parts from {machine_id} has exceeded '
                'acceptable limits. Finished parts may not meet specification.'
            ),
            'details_operator': [
                'Surface finish on recent parts is rougher than required.',
                'Check the tool edge for wear or built-up edge.',
            ],
            'details_engineer': [
                'Measured Ra exceeds the drawing specification for the current '
                'feature being machined.',
                'Contributing factors may include tool wear, vibration, or '
                'incorrect feed-per-tooth for the desired finish.',
                'Chip re-cutting due to poor chip evacuation may be a factor.',
            ],
            'details_manager': [
                'Parts may need rework or additional finishing operations.',
                'Scrap rate may increase if the root cause is not addressed.',
                'Customer quality requirements are at risk.',
            ],
            'recommendations': [
                'Replace the tool or re-hone the cutting edge.',
                'Reduce feed per tooth for finishing passes.',
                'Verify coolant flow is adequate for chip evacuation.',
            ],
        },
        'COOLANT_LOW': {
            'title': '{urgency}: Coolant level or flow low',
            'summary': (
                'Coolant system on {machine_id} reports low flow or level. '
                'Insufficient coolant accelerates tool wear and risks thermal '
                'damage to the workpiece.'
            ),
            'details_operator': [
                'Coolant level is low or flow rate has dropped.',
                'Top up coolant or check for blockages in the nozzle.',
            ],
            'details_engineer': [
                'Coolant flow rate has dropped below the minimum threshold '
                'required for the current operation and material combination.',
                'Reduced coolant may cause a spike in tool-chip interface '
                'temperature, accelerating diffusion wear.',
                'Check pump pressure, filter condition, and nozzle alignment.',
            ],
            'details_manager': [
                'Risk of accelerated tool wear increasing tooling costs.',
                'Potential for thermal damage to high-value workpieces.',
                'Coolant system maintenance may be needed.',
            ],
            'recommendations': [
                'Refill coolant reservoir to the correct level.',
                'Check coolant pump and filters for blockage.',
                'Reduce cutting speed until coolant flow is restored.',
            ],
        },
    }

    _DEFAULT_ALARM_TEMPLATE: Dict[str, Any] = {
        'title': '{urgency}: Alarm on {machine_id}',
        'summary': (
            'An alarm of type {alarm_type} has been raised on {machine_id} '
            'with severity {severity_pct}%.'
        ),
        'details_operator': [
            'An alarm condition has been detected.',
            'Monitor the machine and follow standard procedures.',
        ],
        'details_engineer': [
            'An unrecognised alarm type was raised.',
            'Investigate sensor data and logs for root cause.',
        ],
        'details_manager': [
            'A machine alarm requires attention.',
            'Production may be affected until resolved.',
        ],
        'recommendations': [
            'Investigate the alarm and consult the machine manual.',
            'Contact maintenance if the condition persists.',
        ],
    }

    def __init__(self, audience: str = 'operator') -> None:
        self._audience = self._validate_audience(audience)

    # -- Public API --------------------------------------------------------

    def set_audience(self, audience: str) -> None:
        """Adjust the detail level for the target audience.

        Args:
            audience: One of 'operator', 'engineer', or 'manager'.
        """
        self._audience = self._validate_audience(audience)

    @property
    def audience(self) -> str:
        return self._audience

    def explain_alarm(
        self,
        alarm_type: str,
        severity: float,
        context: Optional['ExplanationContext'] = None,
    ) -> 'NLGExplanation':
        """Generate a plain-language explanation for a CNC alarm.

        Args:
            alarm_type: Alarm identifier (e.g. 'CHATTER', 'TOOL_WEAR').
            severity: Severity on a 0-1 scale.
            context: Optional additional context about the event.

        Returns:
            An NLGExplanation tailored to the current audience.
        """
        template = self.ALARM_TEMPLATES.get(
            alarm_type, self._DEFAULT_ALARM_TEMPLATE,
        )
        urgency = self._urgency_word(severity)
        machine_id = context.machine_id if context else 'unknown'
        severity_pct = int(severity * 100)

        fmt = {
            'urgency': urgency,
            'machine_id': machine_id,
            'alarm_type': alarm_type,
            'severity_pct': severity_pct,
        }

        title = template['title'].format(**fmt)
        summary = template['summary'].format(**fmt)
        details = list(self._details_for_audience(template))
        recommendations = list(template.get('recommendations', []))

        # Trim recommendations for non-technical audiences
        if self._audience == 'manager':
            recommendations = recommendations[:1]

        confidence = min(1.0, 0.6 + severity * 0.35)

        return NLGExplanation(
            title=title,
            summary=summary,
            details=details,
            recommendations=recommendations,
            confidence=round(confidence, 3),
            audience=self._audience,
        )

    def explain_process_change(
        self,
        parameter: str,
        old_value: float,
        new_value: float,
        reason: str,
    ) -> 'NLGExplanation':
        """Explain why a process parameter was changed.

        Args:
            parameter: Name of the changed parameter (e.g. 'feed_rate').
            old_value: Previous value.
            new_value: New value.
            reason: Brief reason string from the control system.

        Returns:
            An NLGExplanation describing the change.
        """
        direction = 'increased' if new_value > old_value else 'decreased'
        pct_change = abs(new_value - old_value) / max(abs(old_value), 1e-9) * 100

        title = f'Process parameter change: {parameter}'
        summary = (
            f'{parameter} was {direction} from {old_value:.4g} to '
            f'{new_value:.4g} ({pct_change:.1f}% change). Reason: {reason}.'
        )

        details_map = {
            'operator': [
                f'The {parameter} setting has been adjusted automatically.',
                f'The change was made because: {reason}.',
            ],
            'engineer': [
                f'{parameter} {direction} by {pct_change:.1f}% '
                f'(from {old_value:.4g} to {new_value:.4g}).',
                f'Root cause for adjustment: {reason}.',
                f'Verify downstream process stability after this change.',
            ],
            'manager': [
                f'An automated parameter adjustment was made to maintain '
                f'process quality.',
                f'Reason: {reason}.',
            ],
        }

        details = details_map.get(self._audience, details_map['operator'])
        recommendations = [
            f'Monitor process stability after the {parameter} change.',
            'Verify part quality on the next inspection.',
        ]

        confidence = 0.85

        return NLGExplanation(
            title=title,
            summary=summary,
            details=details,
            recommendations=recommendations,
            confidence=confidence,
            audience=self._audience,
        )

    def explain_prediction(
        self,
        prediction_type: str,
        predicted_value: float,
        confidence: float,
        factors: List[str],
    ) -> 'NLGExplanation':
        """Explain a predictive model output in plain language.

        Args:
            prediction_type: What is being predicted (e.g. 'tool_life').
            predicted_value: The predicted numeric value.
            confidence: Model confidence (0-1).
            factors: List of contributing factor names.

        Returns:
            An NLGExplanation describing the prediction.
        """
        conf_word = 'high' if confidence > 0.8 else (
            'moderate' if confidence > 0.5 else 'low'
        )

        title = f'Prediction: {prediction_type}'
        summary = (
            f'The system predicts {prediction_type} = {predicted_value:.4g} '
            f'with {conf_word} confidence ({confidence:.0%}).'
        )

        factor_str = ', '.join(factors) if factors else 'general process data'
        details_map = {
            'operator': [
                f'Predicted {prediction_type}: {predicted_value:.4g}.',
                f'Confidence: {conf_word}.',
            ],
            'engineer': [
                f'Predicted {prediction_type}: {predicted_value:.4g} '
                f'(confidence: {confidence:.1%}).',
                f'Key contributing factors: {factor_str}.',
                f'Model uncertainty may be higher if operating conditions '
                f'have changed recently.',
            ],
            'manager': [
                f'The AI system predicts {prediction_type} at '
                f'{predicted_value:.4g} with {conf_word} confidence.',
                f'Based on analysis of: {factor_str}.',
            ],
        }

        details = details_map.get(self._audience, details_map['operator'])

        recommendations = []
        if confidence < 0.5:
            recommendations.append(
                'Low confidence — treat this prediction as tentative and '
                'verify with additional measurements.'
            )
        if confidence < 0.8:
            recommendations.append(
                'Consider collecting more data to improve prediction accuracy.'
            )
        recommendations.append(
            f'Review the prediction after the next {prediction_type} observation.'
        )

        return NLGExplanation(
            title=title,
            summary=summary,
            details=details,
            recommendations=recommendations,
            confidence=round(confidence, 3),
            audience=self._audience,
        )

    def explain_recommendation(
        self,
        action: str,
        expected_benefit: str,
        risk: str,
        context: Optional['ExplanationContext'] = None,
    ) -> 'NLGExplanation':
        """Explain why a specific recommendation is being made.

        Args:
            action: The recommended action.
            expected_benefit: What improvement is expected.
            risk: Risk of not taking the action.
            context: Optional event context.

        Returns:
            An NLGExplanation justifying the recommendation.
        """
        machine_id = context.machine_id if context else 'unknown'

        title = f'Recommendation: {action}'
        summary = (
            f'It is recommended to {action} on {machine_id}. '
            f'Expected benefit: {expected_benefit}. '
            f'Risk if not addressed: {risk}.'
        )

        details_map = {
            'operator': [
                f'Action needed: {action}.',
                f'This will help: {expected_benefit}.',
                f'If not done: {risk}.',
            ],
            'engineer': [
                f'Recommended action: {action}.',
                f'Expected benefit: {expected_benefit}.',
                f'Risk assessment: {risk}.',
                f'Machine: {machine_id}.',
            ],
            'manager': [
                f'A maintenance or process action is recommended.',
                f'Benefit: {expected_benefit}.',
                f'Business risk if deferred: {risk}.',
            ],
        }

        details = details_map.get(self._audience, details_map['operator'])
        recommendations = [action]

        severity = context.severity if context else 0.5
        confidence = min(1.0, 0.65 + severity * 0.3)

        return NLGExplanation(
            title=title,
            summary=summary,
            details=details,
            recommendations=recommendations,
            confidence=round(confidence, 3),
            audience=self._audience,
        )

    # -- Private helpers ---------------------------------------------------

    @staticmethod
    def _validate_audience(audience: str) -> str:
        valid = ('operator', 'engineer', 'manager')
        if audience not in valid:
            raise ValueError(
                f"Invalid audience '{audience}'. Must be one of {valid}."
            )
        return audience

    def _urgency_word(self, severity: float) -> str:
        """Map a 0-1 severity to an urgency word."""
        for threshold, word in self._URGENCY_WORDS:
            if severity >= threshold:
                return word
        return 'INFORMATIONAL'

    def _details_for_audience(self, template: dict) -> List[str]:
        """Select the appropriate detail list from a template."""
        key = f'details_{self._audience}'
        return list(template.get(key, template.get('details_operator', [])))


# ---------------------------------------------------------------------------
# Anomaly Scoring Model
# ---------------------------------------------------------------------------


@dataclass
class AnomalyScore:
    """Score for a single feature indicating how anomalous its value is."""
    feature_name: str
    value: float
    z_score: float
    percentile: float
    is_anomaly: bool
    severity: str  # 'normal' | 'mild' | 'moderate' | 'severe'
    contribution_pct: float


@dataclass
class AnomalyReport:
    """Aggregate anomaly report for an entire observation."""
    timestamp: float
    overall_score: float  # 0-100
    anomaly_scores: List[AnomalyScore]
    top_anomalies: List[AnomalyScore]
    is_anomalous: bool
    explanation: str


class AnomalyScoringModel:
    """Scores manufacturing data points for anomaly likelihood using
    statistical (z-score based) methods.

    Workflow:
      1. ``train(data)`` — learn per-feature normal distributions.
      2. ``score(observation)`` — score a single observation.
      3. ``score_batch(observations)`` — score many observations.

    Severity thresholds (absolute z-score):
      * |z| > 3.0  -> severe
      * |z| > 2.0  -> moderate
      * |z| > 1.5  -> mild
      * otherwise   -> normal

    The overall score is a weighted average of individual feature anomaly
    scores, mapped to a 0-100 scale.  A feature whose absolute z-score is
    zero contributes 0; one with |z| >= 4 contributes the maximum.
    """

    _SEVERITY_THRESHOLDS: List[Tuple[float, str]] = [
        (3.0, 'severe'),
        (2.0, 'moderate'),
        (1.5, 'mild'),
        (0.0, 'normal'),
    ]

    def __init__(self) -> None:
        self._means: Dict[str, float] = {}
        self._stds: Dict[str, float] = {}
        self._mins: Dict[str, float] = {}
        self._maxs: Dict[str, float] = {}
        self._counts: Dict[str, int] = {}
        self._bounds: Dict[str, Tuple[float, float]] = {}
        self._trained: bool = False

    # -- Training ----------------------------------------------------------

    def train(self, data: Dict[str, List[float]]) -> None:
        """Learn normal distributions (mean, std) per feature.

        Args:
            data: Mapping of feature name to a list of observed values.

        Raises:
            ValueError: If *data* is empty or any feature has fewer than
                2 observations.
        """
        if not data:
            raise ValueError('Training data must not be empty.')

        for name, values in data.items():
            if len(values) < 2:
                raise ValueError(
                    f"Feature '{name}' needs at least 2 observations "
                    f"(got {len(values)})."
                )

            n = len(values)
            mean = sum(values) / n
            variance = sum((v - mean) ** 2 for v in values) / (n - 1)
            std = math.sqrt(variance)

            self._means[name] = mean
            self._stds[name] = std
            self._mins[name] = min(values)
            self._maxs[name] = max(values)
            self._counts[name] = n

        self._trained = True

    # -- Scoring -----------------------------------------------------------

    def score(self, observation: Dict[str, float]) -> AnomalyReport:
        """Score a single observation and return an :class:`AnomalyReport`.

        Only features that were seen during training are scored; unknown
        features in *observation* are silently ignored.

        Args:
            observation: Mapping of feature name to observed value.

        Returns:
            An AnomalyReport summarising the anomaly assessment.

        Raises:
            RuntimeError: If the model has not been trained yet.
        """
        if not self._trained:
            raise RuntimeError('Model must be trained before scoring.')

        anomaly_scores: List[AnomalyScore] = []

        for feat, value in observation.items():
            if feat not in self._means:
                continue

            mean = self._means[feat]
            std = self._stds[feat]

            # Compute z-score; guard against zero std
            if std > 0:
                z = (value - mean) / std
            else:
                z = 0.0 if value == mean else (
                    4.0 if value > mean else -4.0
                )

            abs_z = abs(z)

            # Check hard bounds
            bounds_violated = False
            if feat in self._bounds:
                lo, hi = self._bounds[feat]
                if value < lo or value > hi:
                    bounds_violated = True
                    abs_z = max(abs_z, 3.0)

            # Determine severity
            severity = self._classify_severity(abs_z)

            # Percentile approximation using the error-function CDF
            percentile = self._normal_cdf(z) * 100.0

            is_anomaly = abs_z >= 1.5 or bounds_violated

            anomaly_scores.append(AnomalyScore(
                feature_name=feat,
                value=value,
                z_score=round(z, 4),
                percentile=round(percentile, 2),
                is_anomaly=is_anomaly,
                severity=severity,
                contribution_pct=0.0,  # filled in below
            ))

        # Compute per-feature anomaly contribution and overall score
        overall_score = self._compute_overall_score(anomaly_scores)

        # Top anomalies: those flagged, sorted by |z| descending
        top_anomalies = sorted(
            [s for s in anomaly_scores if s.is_anomaly],
            key=lambda s: abs(s.z_score),
            reverse=True,
        )

        is_anomalous = overall_score >= 25.0 or len(top_anomalies) > 0
        explanation = self._build_explanation(overall_score, top_anomalies)

        return AnomalyReport(
            timestamp=time.time(),
            overall_score=round(overall_score, 2),
            anomaly_scores=anomaly_scores,
            top_anomalies=top_anomalies,
            is_anomalous=is_anomalous,
            explanation=explanation,
        )

    def score_batch(
        self, observations: List[Dict[str, float]],
    ) -> List[AnomalyReport]:
        """Score multiple observations.

        Args:
            observations: List of observation dicts.

        Returns:
            A list of :class:`AnomalyReport`, one per observation.
        """
        return [self.score(obs) for obs in observations]

    # -- Feature bounds ----------------------------------------------------

    def add_feature_bounds(
        self, feature_name: str, lower: float, upper: float,
    ) -> None:
        """Set hard limits for a feature.

        Any value outside ``[lower, upper]`` is automatically flagged as
        anomalous (severity at least 'severe').

        Args:
            feature_name: Feature identifier.
            lower: Lower acceptable bound.
            upper: Upper acceptable bound.

        Raises:
            ValueError: If *lower* >= *upper*.
        """
        if lower >= upper:
            raise ValueError(
                f"Lower bound ({lower}) must be less than upper bound ({upper})."
            )
        self._bounds[feature_name] = (lower, upper)

    # -- Statistics --------------------------------------------------------

    def get_feature_stats(self) -> Dict[str, Dict[str, float]]:
        """Return descriptive statistics for every trained feature.

        Returns:
            Dict mapping feature name to
            ``{mean, std, min, max, count}``.
        """
        stats: Dict[str, Dict[str, float]] = {}
        for feat in self._means:
            stats[feat] = {
                'mean': self._means[feat],
                'std': self._stds[feat],
                'min': self._mins[feat],
                'max': self._maxs[feat],
                'count': self._counts[feat],
            }
        return stats

    # -- Private helpers ---------------------------------------------------

    @classmethod
    def _classify_severity(cls, abs_z: float) -> str:
        """Map an absolute z-score to a severity label."""
        for threshold, label in cls._SEVERITY_THRESHOLDS:
            if abs_z >= threshold:
                return label
        return 'normal'

    @staticmethod
    def _normal_cdf(z: float) -> float:
        """Approximate the standard-normal CDF using the error function."""
        return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))

    @staticmethod
    def _compute_overall_score(
        scores: List[AnomalyScore],
    ) -> float:
        """Weighted average of feature anomaly scores on a 0-100 scale.

        Each feature contributes ``min(abs_z / 4, 1) * 100``.  The overall
        score is the mean of these contributions, and each score's
        ``contribution_pct`` is updated in place.
        """
        if not scores:
            return 0.0

        raw: List[float] = []
        for s in scores:
            # Map |z| to 0-100, clamping at |z|=4
            feature_score = min(abs(s.z_score) / 4.0, 1.0) * 100.0
            raw.append(feature_score)

        total = sum(raw)
        overall = total / len(raw)

        # Update contribution percentages
        for s, r in zip(scores, raw):
            pct = (r / total * 100.0) if total > 0 else 0.0
            # dataclass fields are mutable; update in place
            object.__setattr__(s, 'contribution_pct', round(pct, 2))

        return overall

    @staticmethod
    def _build_explanation(
        overall_score: float,
        top_anomalies: List[AnomalyScore],
    ) -> str:
        """Build a human-readable explanation string."""
        if not top_anomalies:
            return (
                f'Overall anomaly score is {overall_score:.1f}/100. '
                f'All features are within normal operating ranges.'
            )

        parts = [
            f'Overall anomaly score is {overall_score:.1f}/100.',
        ]
        for a in top_anomalies[:3]:
            parts.append(
                f'{a.feature_name} is {a.severity} '
                f'(z={a.z_score:+.2f}, value={a.value:.4g}).'
            )

        if len(top_anomalies) > 3:
            parts.append(
                f'{len(top_anomalies) - 3} additional anomalous feature(s) detected.'
            )

        return ' '.join(parts)


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = ExplanationGeneratorNode()
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
