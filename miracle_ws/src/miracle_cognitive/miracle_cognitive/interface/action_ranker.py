"""Alternative action ranking for the decision support system.

Generates and ranks multiple corrective actions by multi-criteria
cost/benefit score, presenting operators with the top solutions
alongside a do-nothing risk assessment.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class ActionCandidate:
    """A single corrective action candidate with predicted impacts."""

    action_type: str  # "REDUCE_FEED", "REDUCE_SPEED", "TOOL_CHANGE", "COOLANT_INCREASE", "PAUSE", "SKIP_BLOCK", "ACCEPT_MONITOR", "ACCEPT_DEVIATION", "SHIFT_RPM"
    description: str  # Human-readable description
    feed_override_pct: float  # 0-100, effective feed after action
    speed_override_pct: float  # 0-100, effective spindle after action
    predicted_force_reduction_pct: float  # how much force drops (positive = reduction)
    predicted_rul_extension_min: float  # additional tool life in minutes
    predicted_surface_improvement_pct: float  # surface quality improvement (positive = better)
    cycle_time_impact_pct: float  # +10 means 10% slower
    risk_reduction: float  # 0.0-1.0, how much risk is reduced
    confidence: float  # 0.0-1.0
    reasoning: str  # Why this action helps
    score: float = 0.0  # Computed composite score


@dataclass
class RankedActions:
    """Result of action ranking: sorted candidates with context."""

    situation_summary: str
    ranked_actions: List[ActionCandidate]  # sorted by score descending
    recommended_index: int  # index of top recommendation (usually 0)
    do_nothing_risk: float  # risk of taking no action (0.0-1.0)


# ---------------------------------------------------------------------------
# Action templates per anomaly type
# ---------------------------------------------------------------------------

_ANOMALY_ACTIONS: Dict[str, List[dict]] = {
    "FORCE_WARNING": [
        {
            "action_type": "REDUCE_FEED",
            "description": "Reduce feed rate by 10%",
            "feed_pct": 90.0,
            "speed_pct": 100.0,
            "reasoning": "Lowering feed reduces cutting force proportionally to feed^0.8, addressing the force warning with minimal cycle time impact.",
        },
        {
            "action_type": "REDUCE_FEED",
            "description": "Reduce feed rate by 20%",
            "feed_pct": 80.0,
            "speed_pct": 100.0,
            "reasoning": "A 20% feed reduction significantly lowers cutting force, suitable for moderate force warnings.",
        },
        {
            "action_type": "SKIP_BLOCK",
            "description": "Reduce depth of cut (skip current block)",
            "feed_pct": 100.0,
            "speed_pct": 100.0,
            "reasoning": "Skipping the current heavy-cut block avoids the force spike entirely; the block can be re-machined at lighter depth.",
        },
        {
            "action_type": "TOOL_CHANGE",
            "description": "Change to a fresh tool",
            "feed_pct": 100.0,
            "speed_pct": 100.0,
            "reasoning": "A sharp tool reduces specific cutting force; worn edges require more force for the same material removal rate.",
        },
    ],
    "CHATTER_RISK": [
        {
            "action_type": "SHIFT_RPM",
            "description": "Shift RPM to stable lobe pocket",
            "feed_pct": 100.0,
            "speed_pct": 85.0,
            "reasoning": "Moving spindle speed to a stability lobe pocket eliminates regenerative chatter by changing the phase between successive tooth passes.",
        },
        {
            "action_type": "SKIP_BLOCK",
            "description": "Reduce depth of cut by 20%",
            "feed_pct": 100.0,
            "speed_pct": 100.0,
            "reasoning": "Reducing axial depth of cut drops below the critical stability limit, preventing chatter onset.",
        },
        {
            "action_type": "REDUCE_FEED",
            "description": "Reduce feed rate by 15%",
            "feed_pct": 85.0,
            "speed_pct": 100.0,
            "reasoning": "Lower feed rate reduces chip load and dynamic excitation forces, dampening incipient chatter.",
        },
        {
            "action_type": "TOOL_CHANGE",
            "description": "Change to a tool with different geometry",
            "feed_pct": 100.0,
            "speed_pct": 100.0,
            "reasoning": "A tool with different flute count or helix angle shifts the natural frequency away from the chatter-prone zone.",
        },
    ],
    "THERMAL_WARNING": [
        {
            "action_type": "COOLANT_INCREASE",
            "description": "Increase coolant flow rate",
            "feed_pct": 100.0,
            "speed_pct": 100.0,
            "reasoning": "Higher coolant flow improves heat extraction from the cutting zone, reducing thermal damage risk to both tool and workpiece.",
        },
        {
            "action_type": "REDUCE_FEED",
            "description": "Reduce feed rate by 15%",
            "feed_pct": 85.0,
            "speed_pct": 100.0,
            "reasoning": "Lower feed reduces heat generation from chip formation, allowing the cutting zone to stay within thermal limits.",
        },
        {
            "action_type": "REDUCE_SPEED",
            "description": "Reduce spindle speed by 10%",
            "feed_pct": 100.0,
            "speed_pct": 90.0,
            "reasoning": "Lower cutting speed reduces friction heat at the tool-chip interface, which is the primary heat source in machining.",
        },
        {
            "action_type": "PAUSE",
            "description": "Pause machining for 30 seconds to cool",
            "feed_pct": 0.0,
            "speed_pct": 0.0,
            "reasoning": "A brief pause allows the workpiece and tool to dissipate accumulated heat, preventing thermal damage in subsequent cuts.",
        },
    ],
    "TOOL_WEAR": [
        {
            "action_type": "TOOL_CHANGE",
            "description": "Replace tool immediately",
            "feed_pct": 100.0,
            "speed_pct": 100.0,
            "reasoning": "A fresh tool restores cutting geometry, eliminating wear-induced force increases and surface quality degradation.",
        },
        {
            "action_type": "REDUCE_FEED",
            "description": "Reduce feed rate by 30%",
            "feed_pct": 70.0,
            "speed_pct": 100.0,
            "reasoning": "Aggressive feed reduction compensates for the worn tool's reduced cutting ability, extending remaining useful life.",
        },
        {
            "action_type": "SKIP_BLOCK",
            "description": "Reduce depth of cut by 25%",
            "feed_pct": 100.0,
            "speed_pct": 100.0,
            "reasoning": "Lighter cuts reduce tool loading, slowing further wear progression and preventing catastrophic failure.",
        },
        {
            "action_type": "ACCEPT_MONITOR",
            "description": "Accept current state and monitor closely",
            "feed_pct": 100.0,
            "speed_pct": 100.0,
            "reasoning": "If wear is within acceptable bounds, continuing with enhanced monitoring avoids unnecessary downtime.",
        },
    ],
    "SURFACE_QUALITY": [
        {
            "action_type": "TOOL_CHANGE",
            "description": "Replace tool for better surface finish",
            "feed_pct": 100.0,
            "speed_pct": 100.0,
            "reasoning": "A fresh tool with intact cutting edges produces superior surface finish; worn edge micro-geometry degrades Ra.",
        },
        {
            "action_type": "REDUCE_FEED",
            "description": "Reduce feed rate by 20%",
            "feed_pct": 80.0,
            "speed_pct": 100.0,
            "reasoning": "Surface roughness is proportional to feed^2, so a 20% feed reduction improves Ra by approximately 36%.",
        },
        {
            "action_type": "COOLANT_INCREASE",
            "description": "Increase coolant for better lubrication",
            "feed_pct": 100.0,
            "speed_pct": 100.0,
            "reasoning": "Better lubrication reduces built-up edge formation and friction marks that degrade surface finish.",
        },
        {
            "action_type": "ACCEPT_DEVIATION",
            "description": "Accept surface deviation within tolerance",
            "feed_pct": 100.0,
            "speed_pct": 100.0,
            "reasoning": "If the surface roughness is still within the part tolerance band, accepting the deviation avoids unnecessary rework.",
        },
    ],
}


class ActionRanker:
    """Ranks corrective actions by multi-criteria cost-benefit score.

    The ranker generates candidate actions appropriate for a detected anomaly,
    estimates their physical effects using simplified machining models, and
    scores them with configurable weights to present the operator with
    prioritised alternatives.
    """

    DEFAULT_WEIGHTS: Dict[str, float] = {
        "risk_reduction": 0.35,
        "rul_extension": 0.25,
        "cycle_time": 0.20,  # negative weight — less time impact is better
        "surface_quality": 0.10,
        "confidence": 0.10,
    }

    def __init__(self, weights: Optional[Dict[str, float]] = None):
        self.weights = dict(weights) if weights else dict(self.DEFAULT_WEIGHTS)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def rank_actions(
        self,
        anomaly_type: str,
        severity: float,
        current_state: Optional[Dict] = None,
    ) -> RankedActions:
        """Generate and rank candidate actions for a given anomaly.

        Parameters
        ----------
        anomaly_type : str
            One of the anomaly categories (e.g. ``"FORCE_WARNING"``).
        severity : float
            0.0 – 1.0 severity of the detected anomaly.
        current_state : dict, optional
            ``feed_pct``, ``speed_pct``, ``rul_remaining_min`` from machine.

        Returns
        -------
        RankedActions
        """
        if current_state is None:
            current_state = {}

        candidates = self._generate_candidates(anomaly_type, severity, current_state)

        # Score and sort
        for c in candidates:
            c.score = self._score_action(c)
        candidates.sort(key=lambda c: c.score, reverse=True)

        do_nothing_risk = self._compute_do_nothing_risk(anomaly_type, severity)

        situation = (
            f"{anomaly_type} detected at severity {severity:.0%}. "
            f"{len(candidates)} corrective actions evaluated."
        )

        return RankedActions(
            situation_summary=situation,
            ranked_actions=candidates,
            recommended_index=0,
            do_nothing_risk=do_nothing_risk,
        )

    # ------------------------------------------------------------------
    # Candidate generation
    # ------------------------------------------------------------------

    def _generate_candidates(
        self,
        anomaly_type: str,
        severity: float,
        current_state: Dict,
    ) -> List[ActionCandidate]:
        """Generate ``ActionCandidate`` objects for *anomaly_type*."""

        templates = _ANOMALY_ACTIONS.get(anomaly_type, [])
        current_feed = current_state.get("feed_pct", 100.0)
        current_speed = current_state.get("speed_pct", 100.0)
        current_rul = current_state.get("rul_remaining_min", 30.0)

        candidates: List[ActionCandidate] = []
        for tmpl in templates:
            target_feed = tmpl["feed_pct"]
            target_speed = tmpl["speed_pct"]

            force_red = self._estimate_force_reduction(
                tmpl["action_type"], current_feed, target_feed
            )
            rul_ext = self._estimate_rul_extension(
                tmpl["action_type"], current_feed, target_feed, current_rul
            )
            surface_imp = self._estimate_surface_improvement(
                tmpl["action_type"], current_feed, target_feed
            )
            cycle_impact = self._estimate_cycle_time_impact(
                tmpl["action_type"],
                (target_feed - current_feed) / max(current_feed, 1.0) * 100.0,
            )
            risk_red = self._estimate_risk_reduction(
                tmpl["action_type"], severity, force_red
            )
            confidence = self._estimate_confidence(tmpl["action_type"], severity)

            candidates.append(
                ActionCandidate(
                    action_type=tmpl["action_type"],
                    description=tmpl["description"],
                    feed_override_pct=target_feed,
                    speed_override_pct=target_speed,
                    predicted_force_reduction_pct=force_red,
                    predicted_rul_extension_min=rul_ext,
                    predicted_surface_improvement_pct=surface_imp,
                    cycle_time_impact_pct=cycle_impact,
                    risk_reduction=risk_red,
                    confidence=confidence,
                    reasoning=tmpl["reasoning"],
                )
            )

        return candidates

    # ------------------------------------------------------------------
    # Scoring
    # ------------------------------------------------------------------

    def _score_action(self, candidate: ActionCandidate) -> float:
        """Multi-criteria weighted score for an action.

        Higher is better.  ``cycle_time`` weight is applied negatively
        (less cycle-time impact = higher score).
        """
        w = self.weights

        # Normalise rul_extension to 0-1 range (assume 60 min max benefit)
        rul_norm = min(candidate.predicted_rul_extension_min / 60.0, 1.0)

        # Normalise cycle_time_impact: 0% impact → 1.0, 100% impact → 0.0
        cycle_norm = max(1.0 - abs(candidate.cycle_time_impact_pct) / 100.0, 0.0)

        # Normalise surface improvement to 0-1
        surface_norm = min(max(candidate.predicted_surface_improvement_pct / 100.0, 0.0), 1.0)

        score = (
            w.get("risk_reduction", 0) * candidate.risk_reduction
            + w.get("rul_extension", 0) * rul_norm
            + w.get("cycle_time", 0) * cycle_norm
            + w.get("surface_quality", 0) * surface_norm
            + w.get("confidence", 0) * candidate.confidence
        )
        return round(score, 6)

    # ------------------------------------------------------------------
    # Estimation helpers
    # ------------------------------------------------------------------

    def _estimate_force_reduction(
        self, action_type: str, current_feed_pct: float, target_feed_pct: float
    ) -> float:
        """Estimate force reduction from feed change: F proportional to feed^0.8.

        Returns percentage of force reduction (positive = force drops).
        """
        if action_type == "TOOL_CHANGE":
            return 15.0  # fresh tool has lower specific cutting force
        if action_type in ("PAUSE", "COOLANT_INCREASE", "ACCEPT_MONITOR", "ACCEPT_DEVIATION"):
            return 0.0
        if action_type == "SKIP_BLOCK":
            return 25.0  # depth reduction approximation

        if current_feed_pct <= 0:
            return 0.0

        # F_new / F_old = (target/current)^0.8
        ratio = (target_feed_pct / current_feed_pct) ** 0.8
        reduction_pct = (1.0 - ratio) * 100.0
        return round(max(reduction_pct, 0.0), 2)

    def _estimate_rul_extension(
        self,
        action_type: str,
        current_feed_pct: float,
        target_feed_pct: float,
        current_rul: float,
    ) -> float:
        """Estimate RUL extension from reduced cutting parameters.

        Uses a simplified Taylor-like relationship where tool life is
        inversely related to feed rate raised to a power.
        """
        if action_type == "TOOL_CHANGE":
            return 30.0  # new tool = full fresh life (nominal 30 min)
        if action_type in ("ACCEPT_MONITOR", "ACCEPT_DEVIATION"):
            return 0.0
        if action_type == "PAUSE":
            return 2.0  # marginal cooling benefit
        if action_type == "COOLANT_INCREASE":
            return 5.0  # better cooling extends life moderately
        if action_type == "SKIP_BLOCK":
            return 8.0  # less material = less wear

        if current_feed_pct <= 0 or target_feed_pct <= 0:
            return 0.0

        # Tool life inversely proportional to feed^2 (simplified Taylor)
        # life_new / life_old = (current_feed / target_feed)^2
        life_ratio = (current_feed_pct / target_feed_pct) ** 2
        extension = current_rul * (life_ratio - 1.0)
        return round(max(extension, 0.0), 2)

    def _estimate_surface_improvement(
        self, action_type: str, current_feed_pct: float, target_feed_pct: float
    ) -> float:
        """Estimate surface quality improvement percentage.

        Ra proportional to feed^2, so reduction in feed improves finish.
        """
        if action_type == "TOOL_CHANGE":
            return 30.0
        if action_type == "COOLANT_INCREASE":
            return 10.0
        if action_type in ("PAUSE", "ACCEPT_MONITOR", "ACCEPT_DEVIATION", "SKIP_BLOCK"):
            return 0.0

        if current_feed_pct <= 0:
            return 0.0

        # Ra_new / Ra_old = (target/current)^2
        ratio = (target_feed_pct / current_feed_pct) ** 2
        improvement_pct = (1.0 - ratio) * 100.0
        return round(max(improvement_pct, 0.0), 2)

    def _estimate_cycle_time_impact(
        self, action_type: str, feed_change_pct: float
    ) -> float:
        """Estimate cycle time increase from parameter changes.

        Returns positive percentage meaning slower.
        """
        if action_type == "TOOL_CHANGE":
            return 15.0  # tool change downtime
        if action_type == "PAUSE":
            return 20.0  # 30s pause overhead
        if action_type == "SKIP_BLOCK":
            return 10.0  # rework overhead
        if action_type in ("COOLANT_INCREASE", "ACCEPT_MONITOR", "ACCEPT_DEVIATION", "SHIFT_RPM"):
            return 0.0

        # Feed/speed reduction: cycle time inversely proportional to feed
        if feed_change_pct >= 0:
            return 0.0
        # e.g. -20% feed → ~25% longer cycle (1/0.8 - 1 = 0.25)
        new_ratio = 1.0 + feed_change_pct / 100.0
        if new_ratio <= 0:
            return 100.0
        impact = (1.0 / new_ratio - 1.0) * 100.0
        return round(max(impact, 0.0), 2)

    def _estimate_risk_reduction(
        self, action_type: str, severity: float, force_reduction_pct: float
    ) -> float:
        """Estimate how much this action reduces overall risk (0-1)."""
        if action_type == "TOOL_CHANGE":
            return min(0.9, 0.5 + severity * 0.4)
        if action_type in ("ACCEPT_MONITOR", "ACCEPT_DEVIATION"):
            return 0.05
        if action_type == "PAUSE":
            return 0.3
        if action_type == "COOLANT_INCREASE":
            return 0.4
        if action_type == "SKIP_BLOCK":
            return 0.6
        if action_type == "SHIFT_RPM":
            return 0.7

        # For feed/speed reductions, risk reduction scales with force reduction
        return min(force_reduction_pct / 100.0 * 2.0, 0.8)

    def _estimate_confidence(self, action_type: str, severity: float) -> float:
        """Estimate confidence in the prediction for this action type."""
        base_confidence = {
            "REDUCE_FEED": 0.85,
            "REDUCE_SPEED": 0.80,
            "TOOL_CHANGE": 0.90,
            "COOLANT_INCREASE": 0.70,
            "PAUSE": 0.75,
            "SKIP_BLOCK": 0.65,
            "SHIFT_RPM": 0.60,
            "ACCEPT_MONITOR": 0.50,
            "ACCEPT_DEVIATION": 0.50,
        }
        conf = base_confidence.get(action_type, 0.5)
        # Higher severity → slightly lower confidence (more uncertainty)
        conf *= 1.0 - severity * 0.15
        return round(min(max(conf, 0.0), 1.0), 4)

    # ------------------------------------------------------------------
    # Do-nothing risk
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_do_nothing_risk(anomaly_type: str, severity: float) -> float:
        """Risk of taking no corrective action.

        Base risk depends on anomaly type; severity scales it.
        """
        base = {
            "FORCE_WARNING": 0.5,
            "CHATTER_RISK": 0.6,
            "THERMAL_WARNING": 0.55,
            "TOOL_WEAR": 0.7,
            "SURFACE_QUALITY": 0.3,
        }
        risk = base.get(anomaly_type, 0.4) * (0.5 + severity)
        return round(min(max(risk, 0.0), 1.0), 4)
