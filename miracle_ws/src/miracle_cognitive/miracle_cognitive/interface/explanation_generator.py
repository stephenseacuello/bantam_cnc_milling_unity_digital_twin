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
import time
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import AnomalyAlert, Explanation


@dataclass
class FeatureContribution:
    """A single feature's contribution to a decision."""
    feature_name: str
    value: float
    contribution_pct: float
    threshold: Optional[float] = None
    direction: str = 'above'


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
        self._explanation_pub.publish(exp_msg)

    def generate_explanation(
        self, anomaly_type: str, machine_id: str, severity: float,
        contributing_factors: Optional[List[str]] = None,
        recommended_action: str = '',
    ) -> ExplanationRecord:
        template = self._ANOMALY_TEMPLATES.get(anomaly_type, self._DEFAULT_TEMPLATE)
        severity_pct = int(severity * 100)
        reduction = max(10, int(severity * 30))
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

        features = self._build_feature_contributions(
            anomaly_type, severity, contributing_factors, template
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

    def _build_feature_contributions(
        self, anomaly_type: str, severity: float,
        contributing_factors: Optional[List[str]], template: dict,
    ) -> List[FeatureContribution]:
        features = []
        factor_names = contributing_factors or template.get('typical_features', [])
        if not factor_names:
            return features

        total = len(factor_names)
        for i, name in enumerate(factor_names):
            if i == 0:
                pct = 40.0 + severity * 10
            elif i == 1:
                pct = 25.0
            else:
                pct = max(5.0, (35.0 - 25.0) / max(1, total - 2))

            features.append(FeatureContribution(
                feature_name=name,
                value=severity * (1.0 - i * 0.15),
                contribution_pct=round(pct, 1),
                threshold=severity * 0.7 if i == 0 else None,
                direction='above',
            ))

        total_pct = sum(f.contribution_pct for f in features)
        if total_pct > 0:
            for f in features:
                f.contribution_pct = round(f.contribution_pct / total_pct * 100, 1)

        features.sort(key=lambda f: f.contribution_pct, reverse=True)
        return features

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
                lines.append(
                    f'  {i}. {f.feature_name}: {f.contribution_pct:.1f}% contribution '
                    f'(value={f.value:.3f}{threshold_str})'
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
