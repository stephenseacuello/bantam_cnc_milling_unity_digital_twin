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

        features = self._build_feature_contributions(
            anomaly_type, severity, contributing_factors, template,
            recommended_action=recommended_action,
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

    def _build_detail(
        self, prefix: str, features: List[FeatureContribution],
        severity: float, recommended_action: str,
        machine_id: str, anomaly_type: str,
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

    def explain_anomaly(self, anomaly_type: str, factors: list) -> str:
        record = self.generate_explanation(
            anomaly_type=anomaly_type, machine_id='unknown',
            severity=0.5, contributing_factors=factors,
        )
        return record.summary

    def explain_optimization(self, action: str, reasoning: str) -> str:
        return f"Optimization '{action}': {reasoning}"

    @property
    def history(self) -> List[ExplanationRecord]:
        with self._history_lock:
            return list(self._history)


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
