"""
Reasoning Engine Node.

Performs logical reasoning over the knowledge graph.
Supports forward chaining, backward chaining, and rule evaluation.
Includes a standalone ManufacturingKnowledgeGraph for entity-relationship modelling.
"""

from typing import Any, Dict, List, Optional, Set, Tuple
from dataclasses import dataclass, field
from enum import Enum
from collections import deque
import threading
import time

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


# ---------------------------------------------------------------------------
# Manufacturing Knowledge Graph
# ---------------------------------------------------------------------------

class EntityType(Enum):
    """Types of manufacturing knowledge entities."""
    MACHINE = 'machine'
    TOOL = 'tool'
    MATERIAL = 'material'
    PROCESS = 'process'
    PARAMETER = 'parameter'
    OUTCOME = 'outcome'
    FAILURE = 'failure'


class RelationshipType(Enum):
    """Types of relationships between entities."""
    USES = 'uses'
    PRODUCES = 'produces'
    CAUSES = 'causes'
    MITIGATES = 'mitigates'
    REQUIRES = 'requires'
    AFFECTS = 'affects'
    OPTIMAL_FOR = 'optimal_for'


@dataclass
class KnowledgeNode:
    """A node in the manufacturing knowledge graph."""
    node_id: str
    entity_type: EntityType
    name: str
    properties: Dict[str, Any] = field(default_factory=dict)
    created_at: float = field(default_factory=time.time)


@dataclass
class KnowledgeEdge:
    """A directed edge in the knowledge graph."""
    source_id: str
    target_id: str
    relationship: RelationshipType
    weight: float = 1.0  # 0-1 confidence
    evidence: List[str] = field(default_factory=list)


@dataclass
class GraphQuery:
    """A query against the knowledge graph."""
    entity_type: Optional[EntityType] = None
    relationship: Optional[RelationshipType] = None
    source_id: Optional[str] = None
    target_id: Optional[str] = None
    min_weight: float = 0.0


@dataclass
class GraphQueryResult:
    """Result of a knowledge graph query."""
    nodes: List[KnowledgeNode] = field(default_factory=list)
    edges: List[KnowledgeEdge] = field(default_factory=list)
    paths: List[List[str]] = field(default_factory=list)


class ManufacturingKnowledgeGraph:
    """Graph of manufacturing entities and their relationships.

    Supports adding nodes/edges, path finding, neighbour queries,
    transitive inference, and recommendation queries.
    """

    def __init__(self) -> None:
        self._nodes: Dict[str, KnowledgeNode] = {}
        # adjacency: source_id -> list of edges
        self._adj: Dict[str, List[KnowledgeEdge]] = {}
        # reverse adjacency for incoming edges
        self._rev: Dict[str, List[KnowledgeEdge]] = {}

    # -- mutators --

    def add_node(self, node: KnowledgeNode) -> bool:
        """Add a node. Returns False if node_id already exists (no overwrite)."""
        if node.node_id in self._nodes:
            return False
        self._nodes[node.node_id] = node
        self._adj.setdefault(node.node_id, [])
        self._rev.setdefault(node.node_id, [])
        return True

    def add_edge(self, edge: KnowledgeEdge) -> bool:
        """Add an edge. Source and target must already exist."""
        if edge.source_id not in self._nodes or edge.target_id not in self._nodes:
            return False
        self._adj.setdefault(edge.source_id, []).append(edge)
        self._rev.setdefault(edge.target_id, []).append(edge)
        return True

    # -- queries --

    def get_node(self, node_id: str) -> Optional[KnowledgeNode]:
        return self._nodes.get(node_id)

    def query_by_type(self, entity_type: EntityType) -> List[KnowledgeNode]:
        """Return all nodes of a given type."""
        return [n for n in self._nodes.values() if n.entity_type == entity_type]

    def get_neighbors(self, node_id: str,
                      relationship: Optional[RelationshipType] = None,
                      direction: str = 'outgoing') -> List[KnowledgeNode]:
        """Get neighbouring nodes, optionally filtered by relationship type.

        direction: 'outgoing' (default), 'incoming', or 'both'.
        """
        result_ids: List[str] = []
        if direction in ('outgoing', 'both'):
            for edge in self._adj.get(node_id, []):
                if relationship is None or edge.relationship == relationship:
                    result_ids.append(edge.target_id)
        if direction in ('incoming', 'both'):
            for edge in self._rev.get(node_id, []):
                if relationship is None or edge.relationship == relationship:
                    result_ids.append(edge.source_id)
        # Deduplicate while preserving order
        seen: Set[str] = set()
        nodes: List[KnowledgeNode] = []
        for nid in result_ids:
            if nid not in seen:
                seen.add(nid)
                nodes.append(self._nodes[nid])
        return nodes

    def find_path(self, start_id: str, end_id: str) -> Optional[List[str]]:
        """BFS shortest path from start to end. Returns list of node IDs or None."""
        if start_id not in self._nodes or end_id not in self._nodes:
            return None
        if start_id == end_id:
            return [start_id]
        visited: Set[str] = {start_id}
        queue: deque = deque([(start_id, [start_id])])
        while queue:
            current, path = queue.popleft()
            for edge in self._adj.get(current, []):
                nid = edge.target_id
                if nid not in visited:
                    new_path = path + [nid]
                    if nid == end_id:
                        return new_path
                    visited.add(nid)
                    queue.append((nid, new_path))
        return None

    # -- inference --

    def infer_relationships(self, relationship: RelationshipType = RelationshipType.CAUSES,
                             max_depth: int = 5) -> List[KnowledgeEdge]:
        """Transitive closure: if A->B and B->C with the same relationship, infer A->C.

        Confidence of inferred edge = product of intermediate weights.
        Returns newly inferred edges (also added to graph).
        """
        new_edges: List[KnowledgeEdge] = []
        # Build adjacency for the specific relationship
        rel_adj: Dict[str, List[Tuple[str, float, List[str]]]] = {}
        for src, edges in self._adj.items():
            for e in edges:
                if e.relationship == relationship:
                    rel_adj.setdefault(src, []).append((e.target_id, e.weight, e.evidence))

        # Existing direct pairs for this relationship
        existing: Set[Tuple[str, str]] = set()
        for src, targets in rel_adj.items():
            for tgt, _, _ in targets:
                existing.add((src, tgt))

        # BFS from each source
        for origin in list(rel_adj.keys()):
            visited: Set[str] = {origin}
            # (current_node, accumulated_weight, hops)
            queue: deque = deque()
            for tgt, w, ev in rel_adj.get(origin, []):
                if tgt not in visited:
                    visited.add(tgt)
                    queue.append((tgt, w, 1))
            while queue:
                current, acc_weight, hops = queue.popleft()
                if hops >= max_depth:
                    continue
                for next_tgt, w, ev in rel_adj.get(current, []):
                    if next_tgt not in visited:
                        visited.add(next_tgt)
                        combined_weight = acc_weight * w
                        if (origin, next_tgt) not in existing and combined_weight > 0.01:
                            edge = KnowledgeEdge(
                                source_id=origin,
                                target_id=next_tgt,
                                relationship=relationship,
                                weight=round(combined_weight, 4),
                                evidence=[f'inferred: transitive via {hops + 1} hops'],
                            )
                            self._adj.setdefault(origin, []).append(edge)
                            self._rev.setdefault(next_tgt, []).append(edge)
                            existing.add((origin, next_tgt))
                            new_edges.append(edge)
                        queue.append((next_tgt, acc_weight * w, hops + 1))
        return new_edges

    def query_recommendations(self, entity_type: EntityType,
                                context: Dict[str, str]) -> List[Tuple[KnowledgeNode, float]]:
        """Find optimal parameters for a given context by traversing OPTIMAL_FOR edges.

        context maps property names to desired values (e.g. {'material': 'mat_1'}).
        Returns list of (node, relevance_score) sorted by descending score.
        """
        candidates: List[Tuple[KnowledgeNode, float]] = []
        # Find context nodes
        context_ids: Set[str] = set()
        for prop_name, prop_value in context.items():
            for node in self._nodes.values():
                if node.node_id == prop_value or node.properties.get(prop_name) == prop_value:
                    context_ids.add(node.node_id)

        # Walk OPTIMAL_FOR edges pointing to context nodes
        for ctx_id in context_ids:
            for edge in self._rev.get(ctx_id, []):
                if edge.relationship == RelationshipType.OPTIMAL_FOR:
                    source_node = self._nodes.get(edge.source_id)
                    if source_node and (entity_type is None or source_node.entity_type == entity_type):
                        candidates.append((source_node, edge.weight))

        candidates.sort(key=lambda x: x[1], reverse=True)
        return candidates

    # -- statistics --

    @property
    def node_count(self) -> int:
        return len(self._nodes)

    @property
    def edge_count(self) -> int:
        return sum(len(edges) for edges in self._adj.values())

    @property
    def avg_degree(self) -> float:
        if not self._nodes:
            return 0.0
        return self.edge_count / len(self._nodes)

    def connected_components(self) -> int:
        """Number of weakly connected components."""
        if not self._nodes:
            return 0
        visited: Set[str] = set()
        components = 0
        for nid in self._nodes:
            if nid not in visited:
                components += 1
                # BFS undirected
                queue: deque = deque([nid])
                visited.add(nid)
                while queue:
                    current = queue.popleft()
                    for edge in self._adj.get(current, []):
                        if edge.target_id not in visited:
                            visited.add(edge.target_id)
                            queue.append(edge.target_id)
                    for edge in self._rev.get(current, []):
                        if edge.source_id not in visited:
                            visited.add(edge.source_id)
                            queue.append(edge.source_id)
        return components


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
