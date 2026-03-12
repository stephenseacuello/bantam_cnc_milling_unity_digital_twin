"""
Causal Inference Engine Node.

Performs causal analysis on manufacturing data to determine
root causes of anomalies and quality issues.  Supports temporal
evidence decay so that stale observations fade out over time,
and integrates with CorrelatedAlert messages to strengthen (or
discover) causal links from the alert correlator.
"""

from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field
import math
import threading
import time

from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import AnomalyAlert, KnowledgeUpdate, CorrelatedAlert


@dataclass
class CausalLink:
    """A causal relationship with temporal evidence tracking."""
    cause: str
    effect: str
    strength: float = 0.5
    evidence_count: int = 0
    last_evidence_time: float = 0.0
    evidence_timestamps: List[float] = field(default_factory=list)
    decay_half_life_sec: float = 3600.0
    stale: bool = False


class CausalInferenceNode(MiracleLifecycleNode):
    """Causal analysis for manufacturing root cause determination.

    Parameters:
        min_evidence (int): Minimum evidence count for causal claim.
        analysis_interval_sec (float): Causal analysis interval.
        decay_half_life_sec (float): Half-life for evidence decay in seconds.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('causal_inference', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._causal_graph: Dict[str, List[CausalLink]] = {}
        self._anomaly_subs = []
        self._inference_pub = None
        self._analysis_timer = None
        self._lock = threading.Lock()
        self._decay_half_life_sec: float = 3600.0

    def _do_configure(self) -> TransitionCallbackReturn:
        params = self.declare_and_validate_parameters({
            'min_evidence': {'default': 3, 'type': int, 'range': (1, 100)},
            'analysis_interval_sec': {'default': 30.0, 'type': float, 'range': (5.0, 300.0)},
            'machine_ids': {'default': 'cnc1,cnc2,cnc3', 'type': str},
            'decay_half_life_sec': {'default': 3600.0, 'type': float, 'range': (60.0, 86400.0)},
        })
        machine_ids = self.get_machine_ids(params)
        self._decay_half_life_sec = params.get('decay_half_life_sec', 3600.0)

        self._anomaly_subs = self.create_multi_machine_subscriptions(
            AnomalyAlert, 'anomaly',
            self._on_anomaly, QoSProfiles.alert(), machine_ids,
        )
        self._inference_pub = self.create_publisher(
            KnowledgeUpdate, 'causal_inferences', QoSProfiles.state_data(),
        )

        # Subscribe to correlated alerts for causal graph enrichment
        self.create_subscription(
            CorrelatedAlert,
            '/miracle/scada/correlated_alerts',
            self._on_correlated_alert,
            QoSProfiles.alert(),
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
        now = time.time()
        known_links = [
            CausalLink('HighFeedRate', 'ToolWear', 0.7, decay_half_life_sec=self._decay_half_life_sec),
            CausalLink('ToolWear', 'SurfaceRoughnessIncrease', 0.8, decay_half_life_sec=self._decay_half_life_sec),
            CausalLink('HighSpindleSpeed', 'ThermalExpansion', 0.6, decay_half_life_sec=self._decay_half_life_sec),
            CausalLink('ImproperToolpath', 'Chatter', 0.75, decay_half_life_sec=self._decay_half_life_sec),
            CausalLink('WornBearing', 'Vibration', 0.85, decay_half_life_sec=self._decay_half_life_sec),
            CausalLink('LowCoolant', 'ThermalDamage', 0.9, decay_half_life_sec=self._decay_half_life_sec),
        ]
        for link in known_links:
            link.last_evidence_time = now
            if link.cause not in self._causal_graph:
                self._causal_graph[link.cause] = []
            self._causal_graph[link.cause].append(link)

    # ------------------------------------------------------------------
    # Temporal decay helpers
    # ------------------------------------------------------------------

    @staticmethod
    def compute_decay_factor(time_since: float, half_life: float) -> float:
        """Compute exponential decay factor: 2^(-time_since / half_life)."""
        if half_life <= 0.0:
            return 1.0
        return math.pow(2.0, -time_since / half_life)

    def _apply_temporal_decay(self, now: Optional[float] = None) -> None:
        """Apply temporal decay to every causal link.

        Links whose decay factor drops below 0.1 are marked as *stale*.
        Must be called while holding ``_lock``.
        """
        if now is None:
            now = time.time()

        for links in self._causal_graph.values():
            for link in links:
                if link.last_evidence_time <= 0.0:
                    link.stale = True
                    continue
                dt = now - link.last_evidence_time
                factor = self.compute_decay_factor(dt, link.decay_half_life_sec)
                if factor < 0.1:
                    link.stale = True
                else:
                    link.stale = False

    def _prune_stale_evidence(self, now: Optional[float] = None) -> int:
        """Remove evidence timestamps older than 4 * half_life.

        Returns the number of pruned timestamps across all links.
        Must be called while holding ``_lock``.
        """
        if now is None:
            now = time.time()

        total_pruned = 0
        for links in self._causal_graph.values():
            for link in links:
                cutoff = now - 4.0 * link.decay_half_life_sec
                before = len(link.evidence_timestamps)
                link.evidence_timestamps = [
                    ts for ts in link.evidence_timestamps if ts > cutoff
                ]
                pruned = before - len(link.evidence_timestamps)
                if pruned > 0:
                    link.evidence_count = max(0, link.evidence_count - pruned)
                    total_pruned += pruned

        if total_pruned > 0:
            try:
                self.get_logger().info(f"Pruned {total_pruned} stale evidence timestamps")
            except Exception:
                pass  # logger not available in unit tests

        return total_pruned

    def _effective_strength(self, link: CausalLink, now: Optional[float] = None) -> float:
        """Return strength weighted by temporal decay."""
        if now is None:
            now = time.time()
        if link.last_evidence_time <= 0.0:
            return 0.0
        dt = now - link.last_evidence_time
        factor = self.compute_decay_factor(dt, link.decay_half_life_sec)
        return link.strength * factor

    def _weighted_evidence_count(self, link: CausalLink, now: Optional[float] = None) -> float:
        """Return evidence count weighted by exponential decay per-timestamp."""
        if now is None:
            now = time.time()
        total = 0.0
        for ts in link.evidence_timestamps:
            dt = now - ts
            total += self.compute_decay_factor(dt, link.decay_half_life_sec)
        return total

    # ------------------------------------------------------------------
    # Evidence update
    # ------------------------------------------------------------------

    def _update_evidence(self, link: CausalLink, now: Optional[float] = None,
                         strength_bump: float = 0.01) -> None:
        """Record new evidence for a causal link.

        Updates timestamp tracking and weighted evidence count.
        """
        if now is None:
            now = time.time()

        link.evidence_timestamps.append(now)
        link.last_evidence_time = now
        link.evidence_count += 1
        link.strength = min(1.0, link.strength + strength_bump)
        link.stale = False

    # ------------------------------------------------------------------
    # Query helpers
    # ------------------------------------------------------------------

    def get_active_causes(self, effect: str, min_strength: float = 0.3,
                          now: Optional[float] = None) -> List[CausalLink]:
        """Return non-stale causal links for *effect*, filtered by decayed strength.

        Sorted by effective strength descending.
        """
        if now is None:
            now = time.time()
        results: List[Tuple[float, CausalLink]] = []
        with self._lock:
            self._apply_temporal_decay(now)
            for links in self._causal_graph.values():
                for link in links:
                    if link.effect != effect:
                        continue
                    eff = self._effective_strength(link, now)
                    if eff >= min_strength:
                        results.append((eff, link))
        results.sort(key=lambda x: x[0], reverse=True)
        return [link for _, link in results]

    def suggest_root_cause(self, anomaly_type: str,
                           now: Optional[float] = None) -> List[Tuple[str, float]]:
        """Rank potential root causes for an anomaly type using decayed strengths.

        Returns ``[(cause, effective_strength), ...]`` sorted by strength descending.
        """
        if now is None:
            now = time.time()
        candidates: List[Tuple[str, float]] = []
        with self._lock:
            self._apply_temporal_decay(now)
            for links in self._causal_graph.values():
                for link in links:
                    if link.effect == anomaly_type:
                        eff = self._effective_strength(link, now)
                        if eff > 0.0:
                            candidates.append((link.cause, eff))
        candidates.sort(key=lambda x: x[1], reverse=True)
        return candidates

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        """Update causal model with new anomaly evidence."""
        now = time.time()
        with self._lock:
            for factor in msg.contributing_factors:
                if factor in self._causal_graph:
                    for link in self._causal_graph[factor]:
                        self._update_evidence(link, now=now)

    def _on_correlated_alert(self, msg: CorrelatedAlert) -> None:
        """Strengthen or discover causal links from a CorrelatedAlert.

        The message's ``contributing_alert_ids`` carry the alert types that
        participated in the correlation, and ``root_cause_hypothesis`` names
        the inferred root cause.
        """
        now = time.time()
        # Extract alert types from contributing_alert_ids (format: "type@...")
        alert_types: List[str] = []
        for cid in msg.contributing_alert_ids:
            parts = cid.split('@')
            if parts:
                alert_types.append(parts[0])

        root_cause = msg.root_cause_hypothesis
        if not root_cause:
            return

        with self._lock:
            for atype in alert_types:
                # Look for an existing link from root_cause -> atype
                found = False
                if root_cause in self._causal_graph:
                    for link in self._causal_graph[root_cause]:
                        if link.effect == atype:
                            self._update_evidence(link, now=now, strength_bump=0.02)
                            found = True
                            break

                if not found:
                    # Create a new low-strength causal link
                    new_link = CausalLink(
                        cause=root_cause,
                        effect=atype,
                        strength=0.2,
                        evidence_count=1,
                        last_evidence_time=now,
                        evidence_timestamps=[now],
                        decay_half_life_sec=self._decay_half_life_sec,
                    )
                    if root_cause not in self._causal_graph:
                        self._causal_graph[root_cause] = []
                    self._causal_graph[root_cause].append(new_link)

    # ------------------------------------------------------------------
    # Analysis cycle
    # ------------------------------------------------------------------

    def _run_analysis(self) -> None:
        """Run causal analysis cycle with temporal decay."""
        min_evidence = self.get_parameter('min_evidence').value
        now = time.time()

        with self._lock:
            self._apply_temporal_decay(now)
            self._prune_stale_evidence(now)

            for cause, links in self._causal_graph.items():
                for link in links:
                    if link.stale:
                        continue
                    weighted = self._weighted_evidence_count(link, now)
                    if weighted >= min_evidence:
                        eff = self._effective_strength(link, now)
                        msg = KnowledgeUpdate()
                        msg.timestamp = self.get_clock().now().to_msg()
                        msg.update_type = 'CAUSAL'
                        msg.subject = link.cause
                        msg.predicate = 'causes'
                        msg.object_value = link.effect
                        msg.confidence = eff
                        msg.source = 'causal_inference'
                        msg.reasoning = (
                            f'evidence_count={link.evidence_count}, '
                            f'weighted={weighted:.2f}, '
                            f'effective_strength={eff:.3f}'
                        )
                        self._inference_pub.publish(msg)

    def find_root_causes(self, effect: str) -> List[CausalLink]:
        """Find potential root causes for an effect."""
        causes = []
        now = time.time()
        with self._lock:
            self._apply_temporal_decay(now)
            for cause, links in self._causal_graph.items():
                for link in links:
                    if link.effect == effect:
                        causes.append(link)
        return sorted(causes, key=lambda x: self._effective_strength(x, now), reverse=True)


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
