"""
Causal Inference Engine Node.

Performs causal analysis on manufacturing data to determine
root causes of anomalies and quality issues.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
import threading

from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import AnomalyAlert, KnowledgeUpdate


@dataclass
class CausalLink:
    """A causal relationship."""
    cause: str
    effect: str
    strength: float = 0.5
    evidence_count: int = 0


class CausalInferenceNode(MiracleLifecycleNode):
    """Causal analysis for manufacturing root cause determination.

    Parameters:
        min_evidence (int): Minimum evidence count for causal claim.
        analysis_interval_sec (float): Causal analysis interval.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('causal_inference', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._causal_graph: Dict[str, List[CausalLink]] = {}
        self._anomaly_sub = None
        self._inference_pub = None
        self._analysis_timer = None
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'min_evidence': {'default': 3, 'type': int, 'range': (1, 100)},
            'analysis_interval_sec': {'default': 30.0, 'type': float, 'range': (5.0, 300.0)},
        })

        self._anomaly_sub = self.create_subscription(
            AnomalyAlert, '/miracle/+/anomaly',
            self._on_anomaly, QoSProfiles.alert(),
        )
        self._inference_pub = self.create_publisher(
            KnowledgeUpdate, 'causal_inferences', QoSProfiles.state_data(),
        )

        # Initialize known causal relationships
        self._init_causal_model()
        self.get_logger().info("Causal inference configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        interval = self.get_parameter('analysis_interval_sec').value
        self._analysis_timer = self.create_timer(
            interval, self._run_analysis, callback_group=self.service_callback_group,
        )
        self.get_logger().info("Causal inference activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        if self._analysis_timer is not None:
            self._analysis_timer.cancel()
            self._analysis_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _init_causal_model(self) -> None:
        """Initialize known causal relationships."""
        known_links = [
            CausalLink('HighFeedRate', 'ToolWear', 0.7),
            CausalLink('ToolWear', 'SurfaceRoughnessIncrease', 0.8),
            CausalLink('HighSpindleSpeed', 'ThermalExpansion', 0.6),
            CausalLink('ImproperToolpath', 'Chatter', 0.75),
            CausalLink('WornBearing', 'Vibration', 0.85),
            CausalLink('LowCoolant', 'ThermalDamage', 0.9),
        ]
        for link in known_links:
            if link.cause not in self._causal_graph:
                self._causal_graph[link.cause] = []
            self._causal_graph[link.cause].append(link)

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        """Update causal model with new anomaly evidence."""
        with self._lock:
            for factor in msg.contributing_factors:
                if factor in self._causal_graph:
                    for link in self._causal_graph[factor]:
                        link.evidence_count += 1
                        link.strength = min(1.0, link.strength + 0.01)

    def _run_analysis(self) -> None:
        """Run causal analysis cycle."""
        min_evidence = self.get_parameter('min_evidence').value
        with self._lock:
            for cause, links in self._causal_graph.items():
                for link in links:
                    if link.evidence_count >= min_evidence:
                        msg = KnowledgeUpdate()
                        msg.timestamp = self.get_clock().now().to_msg()
                        msg.update_type = 'CAUSAL'
                        msg.subject = link.cause
                        msg.predicate = 'causes'
                        msg.object_value = link.effect
                        msg.confidence = link.strength
                        msg.source = 'causal_inference'
                        msg.reasoning = f'evidence_count={link.evidence_count}'
                        self._inference_pub.publish(msg)

    def find_root_causes(self, effect: str) -> List[CausalLink]:
        """Find potential root causes for an effect."""
        causes = []
        with self._lock:
            for cause, links in self._causal_graph.items():
                for link in links:
                    if link.effect == effect:
                        causes.append(link)
        return sorted(causes, key=lambda x: x.strength, reverse=True)


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = CausalInferenceNode()
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
