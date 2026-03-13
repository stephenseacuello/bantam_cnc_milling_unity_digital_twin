"""
Reasoning Engine Node.

Performs logical reasoning over the knowledge graph.
Supports forward chaining, backward chaining, and rule evaluation.
"""

from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field
import threading

from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import KnowledgeUpdate

try:
    from miracle_msgs.msg import InferredAction
except ImportError:
    InferredAction = None


# Maps (predicate, object) from rule conclusions to actionable control types
_ACTION_MAPPING = {
    ('requires', 'FeedRateReduction'): 'REDUCE_FEED',
    ('causesRisk', 'QualityDrop'): 'TOOL_CHANGE',
    ('mayHave', 'Chatter'): 'REDUCE_SPEED',
    ('requires', 'CoolantIncrease'): 'COOLANT_INCREASE',
    ('requires', 'EmergencyStop'): 'PAUSE',
}


@dataclass
class SimulatedAction:
    """An action with physics-based predicted outcomes."""
    action_type: str           # REDUCE_FEED, TOOL_CHANGE, etc.
    parameters: dict           # e.g. {"feed_reduction_pct": 20}
    predicted_outcomes: dict   # e.g. {"force_change_pct": -16.4, "cycle_time_change_pct": +25}
    confidence: float          # 0-1, from causal simulation
    side_effects: list         # unintended consequences
    net_benefit_score: float   # weighted sum of outcomes (positive = good)
    reasoning_chain: str       # human-readable causal chain


@dataclass
class Rule:
    """An inference rule."""
    name: str
    conditions: List[Tuple[str, str, str]]  # List of (s, p, o) patterns
    conclusion: Tuple[str, str, str]
    confidence: float = 0.9


class ReasoningEngineNode(MiracleLifecycleNode):
    """Logical reasoning over the knowledge graph.

    Parameters:
        max_inference_depth (int): Maximum chaining depth.
        reasoning_interval_sec (float): Periodic reasoning cycle.
    """

    # Default action parameters used when simulating candidate actions.
    _DEFAULT_ACTION_PARAMS: Dict[str, dict] = {
        'REDUCE_FEED': {'feed_reduction_pct': 20},
        'REDUCE_SPEED': {'speed_reduction_pct': 20},
        'TOOL_CHANGE': {},
        'COOLANT_INCREASE': {'coolant_increase_pct': 30},
        'PAUSE': {},
    }

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('reasoning_engine', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._rules: List[Rule] = []
        self._facts: List[Tuple[str, str, str]] = []
        self._inferred: List[Tuple[str, str, str, float]] = []
        self._lock = threading.Lock()
        self._update_sub = None
        self._inference_pub = None
        self._action_pub = None
        self._reason_timer = None
        self._causal_engine: Optional[Any] = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'max_inference_depth': {'default': 10, 'type': int, 'range': (1, 100)},
            'reasoning_interval_sec': {'default': 5.0, 'type': float, 'range': (1.0, 60.0)},
        })

        self._update_sub = self.create_subscription(
            KnowledgeUpdate, '/miracle/cognitive/knowledge_events',
            self._on_knowledge_update, QoSProfiles.state_data(),
        )
        self._inference_pub = self.create_publisher(
            KnowledgeUpdate, 'inferences', QoSProfiles.state_data(),
        )

        if InferredAction is not None:
            self._action_pub = self.create_publisher(
                InferredAction,
                '/miracle/cognitive/inferred_actions',
                QoSProfiles.state_data(),
            )

        self._load_manufacturing_rules()
        self.get_logger().info("Reasoning engine configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        interval = self.get_parameter('reasoning_interval_sec').value
        self._reason_timer = self.create_timer(
            interval, self._run_inference, callback_group=self.service_callback_group,
        )
        self.get_logger().info("Reasoning engine activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        if self._reason_timer is not None:
            self._reason_timer.cancel()
            self._reason_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _load_manufacturing_rules(self) -> None:
        """Load manufacturing domain rules."""
        self._rules = [
            Rule(
                name='tool_wear_causes_quality_drop',
                conditions=[('?tool', 'hasWearLevel', 'HIGH')],
                conclusion=('?tool', 'causesRisk', 'QualityDrop'),
            ),
            Rule(
                name='high_vibration_indicates_chatter',
                conditions=[('?machine', 'hasVibrationLevel', 'HIGH')],
                conclusion=('?machine', 'mayHave', 'Chatter'),
            ),
            Rule(
                name='overload_requires_feedrate_reduction',
                conditions=[('?machine', 'hasSpindleLoad', 'OVERLOAD')],
                conclusion=('?machine', 'requires', 'FeedRateReduction'),
            ),
            Rule(
                name='thermal_overload_requires_coolant',
                conditions=[('?machine', 'hasTemperature', 'CRITICAL')],
                conclusion=('?machine', 'requires', 'CoolantIncrease'),
                confidence=0.85,
            ),
            Rule(
                name='combined_wear_vibration_requires_stop',
                conditions=[
                    ('?machine', 'hasWearLevel', 'HIGH'),
                    ('?machine', 'hasVibrationLevel', 'HIGH'),
                ],
                conclusion=('?machine', 'requires', 'EmergencyStop'),
                confidence=0.95,
            ),
            Rule(
                name='sustained_overload_degrades_tool',
                conditions=[('?machine', 'hasSpindleLoad', 'OVERLOAD')],
                conclusion=('?machine', 'causesRisk', 'ToolDegradation'),
                confidence=0.8,
            ),
        ]

    def _on_knowledge_update(self, msg: KnowledgeUpdate) -> None:
        """Receive knowledge graph updates as facts."""
        with self._lock:
            self._facts.append((msg.subject, msg.predicate, msg.object_value))

    def _run_inference(self) -> None:
        """Run forward-chaining inference."""
        with self._lock:
            new_inferences = []
            for rule in self._rules:
                bindings = self._match_rule(rule)
                for binding in bindings:
                    s = self._apply_binding(rule.conclusion[0], binding)
                    p = self._apply_binding(rule.conclusion[1], binding)
                    o = self._apply_binding(rule.conclusion[2], binding)
                    if (s, p, o) not in [(i[0], i[1], i[2]) for i in self._inferred]:
                        new_inferences.append((s, p, o, rule.confidence))
                        self._inferred.append((s, p, o, rule.confidence))

            for s, p, o, c in new_inferences:
                msg = KnowledgeUpdate()
                msg.timestamp = self.get_clock().now().to_msg()
                msg.update_type = 'INFERRED'
                msg.subject = s
                msg.predicate = p
                msg.object_value = o
                msg.confidence = c
                msg.source = 'reasoning_engine'
                msg.reasoning = 'forward_chaining'
                self._inference_pub.publish(msg)

                # Publish actionable inference if it maps to a control action
                action_type = _ACTION_MAPPING.get((p, o))
                if action_type is not None and self._action_pub is not None and InferredAction is not None:
                    action_msg = InferredAction()
                    action_msg.timestamp = self.get_clock().now().to_msg()
                    action_msg.machine_id = s
                    action_msg.inference_rule = msg.reasoning
                    action_msg.action_type = action_type
                    action_msg.subject = s
                    action_msg.predicate = p
                    action_msg.object_value = o
                    action_msg.confidence = c

                    # Enrich with causal simulation when available
                    sim_result = self._try_simulate_action(action_type)
                    if sim_result is not None:
                        action_msg.reasoning = (
                            f'{action_type} triggered by ({s}, {p}, {o}) conf={c:.2f} | '
                            f'sim: net_benefit={sim_result.net_benefit_score:.2f}, '
                            f'{sim_result.reasoning_chain}'
                        )
                    else:
                        action_msg.reasoning = f'{action_type} triggered by ({s}, {p}, {o}) conf={c:.2f}'

                    self._action_pub.publish(action_msg)
                    self.get_logger().info(
                        f"Published inferred action: {action_type} for {s} (confidence={c:.2f})"
                    )

    # ------------------------------------------------------------------
    # Causal simulation integration
    # ------------------------------------------------------------------

    def _get_causal_engine(self):
        """Lazily obtain a CausalInferenceNode-compatible forward-sim engine.

        Returns None if the causal inference module is unavailable.
        """
        if self._causal_engine is not None:
            return self._causal_engine
        try:
            from miracle_cognitive.knowledge.causal_inference import CausalInferenceNode
            engine = CausalInferenceNode.__new__(CausalInferenceNode)
            # Initialise just the fields needed for forward simulation
            engine._causal_graph = {}
            engine._lock = threading.RLock()
            engine._decay_half_life_sec = 3600.0
            engine._init_causal_model()
            self._causal_engine = engine
        except Exception:
            self._causal_engine = None
        return self._causal_engine

    def _map_action_to_intervention(self, action_type: str, parameters: dict) -> tuple:
        """Map a reasoning engine action to a causal intervention.

        Returns (intervention_variable, change_pct) suitable for
        ``CausalInferenceNode.simulate_intervention``.
        """
        if action_type == 'REDUCE_FEED':
            pct = parameters.get('feed_reduction_pct', 20)
            return ('feed_rate', -pct)
        elif action_type == 'REDUCE_SPEED':
            pct = parameters.get('speed_reduction_pct', 20)
            return ('spindle_speed', -pct)
        elif action_type == 'TOOL_CHANGE':
            # Fresh tool → model as resetting tool_wear to zero (~-100%)
            return ('tool_wear', -100.0)
        elif action_type == 'COOLANT_INCREASE':
            pct = parameters.get('coolant_increase_pct', 30)
            return ('coolant_flow', pct)
        elif action_type == 'PAUSE':
            return (None, 0.0)
        else:
            return (None, 0.0)

    def simulate_action_outcomes(self, action_type: str, parameters: dict,
                                  current_state: dict = None) -> SimulatedAction:
        """Use forward causal simulation to predict action outcomes.

        Maps action types to causal interventions:
        - REDUCE_FEED  -> simulate_intervention("feed_rate", -reduction_pct)
        - REDUCE_SPEED -> simulate_intervention("spindle_speed", -reduction_pct)
        - TOOL_CHANGE  -> simulate fresh tool state
        - COOLANT_INCREASE -> simulate_intervention("coolant_flow", +increase_pct)
        - PAUSE        -> no causal effects, just time delay

        Returns SimulatedAction with physics-based predictions.
        """
        intervention_var, change_pct = self._map_action_to_intervention(action_type, parameters)

        # PAUSE or unknown: no causal effects
        if intervention_var is None:
            return SimulatedAction(
                action_type=action_type,
                parameters=parameters,
                predicted_outcomes={},
                confidence=1.0 if action_type == 'PAUSE' else 0.0,
                side_effects=[],
                net_benefit_score=0.0,
                reasoning_chain=f'{action_type}: no causal effects modelled',
            )

        engine = self._get_causal_engine()
        if engine is None:
            # Causal engine unavailable – return a default simulation
            return SimulatedAction(
                action_type=action_type,
                parameters=parameters,
                predicted_outcomes={},
                confidence=0.0,
                side_effects=[],
                net_benefit_score=0.0,
                reasoning_chain=f'{action_type}: causal engine unavailable',
            )

        results = engine.simulate_intervention(intervention_var, change_pct)

        # Aggregate all intervention results into a single outcomes dict
        predicted_outcomes: Dict[str, float] = {}
        all_side_effects: List[str] = []
        confidences: List[float] = []
        for r in results:
            predicted_outcomes.update(r.affected_effects)
            confidences.append(r.confidence)

        # Build side effects: effects the user didn't directly intend
        primary_effects = self._primary_effects_for(action_type)
        for effect_name, change in predicted_outcomes.items():
            if effect_name not in primary_effects:
                direction = "increase" if change > 0 else "decrease"
                all_side_effects.append(
                    f'{effect_name} will {direction} by {abs(change):.1f}%'
                )

        avg_confidence = sum(confidences) / len(confidences) if confidences else 0.0
        net_benefit = self._compute_net_benefit(predicted_outcomes)

        # Build reasoning chain
        parts = []
        for effect_name, change in predicted_outcomes.items():
            direction = "decrease" if change < 0 else "increase"
            parts.append(f'{effect_name} {direction} {abs(change):.1f}%')
        reasoning = f'{action_type}({intervention_var} {change_pct:+.0f}%) -> ' + ', '.join(parts) if parts else f'{action_type}: no effects predicted'

        return SimulatedAction(
            action_type=action_type,
            parameters=parameters,
            predicted_outcomes=predicted_outcomes,
            confidence=min(1.0, max(0.0, avg_confidence)),
            side_effects=all_side_effects,
            net_benefit_score=net_benefit,
            reasoning_chain=reasoning,
        )

    def _compute_net_benefit(self, outcomes: dict) -> float:
        """Weighted benefit score from predicted outcomes.

        Score = 0.4 * risk_reduction + 0.3 * quality_improvement
              - 0.2 * cycle_time_cost + 0.1 * tool_life_bonus

        Convention: negative change in a harmful metric = good (positive score).
        """
        score = 0.0

        # Risk reduction: cutting force reduction is good
        force_change = outcomes.get('cutting_force', 0.0)
        score += 0.4 * (-force_change / 100.0)  # -16% force -> +0.064

        # Quality improvement: surface roughness reduction is good
        roughness_change = outcomes.get('surface_roughness', 0.0)
        score += 0.3 * (-roughness_change / 100.0)

        # Cycle time cost: cycle time increase is bad
        cycle_change = outcomes.get('cycle_time', 0.0)
        score -= 0.2 * (cycle_change / 100.0)

        # Temperature reduction is good
        temp_change = outcomes.get('temperature', 0.0)
        score += 0.1 * (-temp_change / 100.0)

        # Tool life increase is good
        life_change = outcomes.get('tool_life', 0.0)
        score += 0.1 * (life_change / 100.0)

        return score

    def get_best_action_with_simulation(self, facts: dict) -> Optional[SimulatedAction]:
        """Run inference to get candidate actions, then simulate each one.

        Returns the action with highest net_benefit_score.
        ``facts`` is a dict of {(subject, predicate): object_value} entries
        to inject before reasoning.
        """
        # Inject facts
        with self._lock:
            for (subj, pred), obj in facts.items():
                self._facts.append((subj, pred, obj))

        # Run inference to find candidate actions
        candidates: List[SimulatedAction] = []
        with self._lock:
            for rule in self._rules:
                bindings = self._match_rule(rule)
                for binding in bindings:
                    p = self._apply_binding(rule.conclusion[1], binding)
                    o = self._apply_binding(rule.conclusion[2], binding)
                    action_type = _ACTION_MAPPING.get((p, o))
                    if action_type is not None:
                        params = dict(self._DEFAULT_ACTION_PARAMS.get(action_type, {}))
                        sim = self.simulate_action_outcomes(action_type, params)
                        # Scale confidence by rule confidence
                        sim = SimulatedAction(
                            action_type=sim.action_type,
                            parameters=sim.parameters,
                            predicted_outcomes=sim.predicted_outcomes,
                            confidence=sim.confidence * rule.confidence,
                            side_effects=sim.side_effects,
                            net_benefit_score=sim.net_benefit_score,
                            reasoning_chain=sim.reasoning_chain,
                        )
                        candidates.append(sim)

        if not candidates:
            return None

        return max(candidates, key=lambda s: s.net_benefit_score)

    def _try_simulate_action(self, action_type: str) -> Optional[SimulatedAction]:
        """Attempt to simulate an action; returns None on failure."""
        try:
            params = dict(self._DEFAULT_ACTION_PARAMS.get(action_type, {}))
            return self.simulate_action_outcomes(action_type, params)
        except Exception:
            return None

    @staticmethod
    def _primary_effects_for(action_type: str) -> set:
        """Return the set of effect names the user primarily intends."""
        return {
            'REDUCE_FEED': {'cutting_force', 'surface_roughness'},
            'REDUCE_SPEED': {'temperature', 'surface_finish'},
            'TOOL_CHANGE': {'surface_roughness', 'cutting_force'},
            'COOLANT_INCREASE': {'temperature'},
            'PAUSE': set(),
        }.get(action_type, set())

    def _match_rule(self, rule: Rule) -> List[Dict[str, str]]:
        """Find variable bindings that satisfy rule conditions."""
        bindings_list: List[Dict[str, str]] = [{}]
        for pattern in rule.conditions:
            new_bindings = []
            for binding in bindings_list:
                matches = self._match_pattern(pattern, binding)
                new_bindings.extend(matches)
            bindings_list = new_bindings
        return bindings_list

    def _match_pattern(
        self, pattern: Tuple[str, str, str], binding: Dict[str, str]
    ) -> List[Dict[str, str]]:
        """Match a pattern against facts with variable bindings."""
        results = []
        s_pat, p_pat, o_pat = pattern
        for s, p, o in self._facts:
            new_binding = dict(binding)
            if self._matches(s_pat, s, new_binding) and \
               self._matches(p_pat, p, new_binding) and \
               self._matches(o_pat, o, new_binding):
                results.append(new_binding)
        return results

    @staticmethod
    def _matches(pattern: str, value: str, binding: Dict[str, str]) -> bool:
        """Check if pattern matches value, updating bindings."""
        if pattern.startswith('?'):
            if pattern in binding:
                return binding[pattern] == value
            binding[pattern] = value
            return True
        return pattern == value

    @staticmethod
    def _apply_binding(template: str, binding: Dict[str, str]) -> str:
        """Apply variable bindings to a template."""
        if template.startswith('?') and template in binding:
            return binding[template]
        return template


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = ReasoningEngineNode()
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
