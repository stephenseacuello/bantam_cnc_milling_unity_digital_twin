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
from itertools import combinations
from math import comb
import math
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


# ---------------------------------------------------------------------------
# Fault Tree Analysis
# ---------------------------------------------------------------------------

class FaultTreeNodeType(Enum):
    """Type of a node in a fault tree."""
    GATE = 'gate'
    EVENT = 'event'


class FaultTreeGateType(Enum):
    """Gate types for fault tree intermediate nodes."""
    AND = 'AND'
    OR = 'OR'
    VOTING = 'VOTING'


@dataclass
class FaultTreeNode:
    """A node in a fault tree (either a gate or a basic event)."""
    node_id: str
    name: str
    node_type: FaultTreeNodeType
    gate_type: Optional[FaultTreeGateType] = None
    probability: float = 0.0  # only meaningful for EVENT nodes
    children: List[str] = field(default_factory=list)
    description: str = ''
    voting_k: int = 0  # k value for VOTING(k/n) gates


@dataclass
class FaultTree:
    """A complete fault tree for failure analysis."""
    root_id: str
    nodes: Dict[str, FaultTreeNode] = field(default_factory=dict)
    name: str = ''
    description: str = ''


class FaultTreeAnalyzer:
    """Builds and evaluates fault trees for manufacturing failure analysis.

    Supports AND, OR, and VOTING(k/n) gate logic, minimal cut set
    identification, Birnbaum importance analysis, and critical path detection.
    """

    def build_tree(self, root_id: str, nodes: Dict[str, FaultTreeNode]) -> FaultTree:
        """Construct and validate a fault tree from a root id and node dict.

        Raises ValueError when the tree is structurally invalid.
        """
        if root_id not in nodes:
            raise ValueError(f"Root node '{root_id}' not found in nodes")

        # Validate all children references exist
        for node in nodes.values():
            for child_id in node.children:
                if child_id not in nodes:
                    raise ValueError(
                        f"Child '{child_id}' of node '{node.node_id}' not found"
                    )

        # Gates must have children; events must not
        for node in nodes.values():
            if node.node_type == FaultTreeNodeType.GATE:
                if not node.children:
                    raise ValueError(
                        f"Gate node '{node.node_id}' has no children"
                    )
                if node.gate_type is None:
                    raise ValueError(
                        f"Gate node '{node.node_id}' has no gate_type"
                    )
                if node.gate_type == FaultTreeGateType.VOTING:
                    n_children = len(node.children)
                    if node.voting_k < 1 or node.voting_k > n_children:
                        raise ValueError(
                            f"VOTING gate '{node.node_id}' has invalid k={node.voting_k} "
                            f"for {n_children} children"
                        )
            elif node.node_type == FaultTreeNodeType.EVENT:
                if node.children:
                    raise ValueError(
                        f"Event node '{node.node_id}' must not have children"
                    )

        # Cycle detection via DFS
        visited: Set[str] = set()
        in_stack: Set[str] = set()

        def _dfs_cycle(nid: str) -> bool:
            visited.add(nid)
            in_stack.add(nid)
            for child_id in nodes[nid].children:
                if child_id in in_stack:
                    return True
                if child_id not in visited and _dfs_cycle(child_id):
                    return True
            in_stack.discard(nid)
            return False

        if _dfs_cycle(root_id):
            raise ValueError("Fault tree contains a cycle")

        return FaultTree(root_id=root_id, nodes=dict(nodes))

    def evaluate(self, tree: FaultTree) -> float:
        """Compute the top event probability by recursively evaluating gates."""
        return self._evaluate_node(tree, tree.root_id)

    def _evaluate_node(self, tree: FaultTree, node_id: str) -> float:
        """Recursively compute probability for a single node."""
        node = tree.nodes[node_id]

        if node.node_type == FaultTreeNodeType.EVENT:
            return node.probability

        child_probs = [self._evaluate_node(tree, cid) for cid in node.children]

        if node.gate_type == FaultTreeGateType.AND:
            result = 1.0
            for p in child_probs:
                result *= p
            return result

        if node.gate_type == FaultTreeGateType.OR:
            result = 1.0
            for p in child_probs:
                result *= (1.0 - p)
            return 1.0 - result

        if node.gate_type == FaultTreeGateType.VOTING:
            k = node.voting_k
            n = len(child_probs)
            # Exact computation: sum over all subsets of size >= k
            total = 0.0
            for r in range(k, n + 1):
                for subset in combinations(range(n), r):
                    p_subset = 1.0
                    for i in range(n):
                        if i in subset:
                            p_subset *= child_probs[i]
                        else:
                            p_subset *= (1.0 - child_probs[i])
                    total += p_subset
            return total

        raise ValueError(f"Unknown gate type: {node.gate_type}")

    def get_minimal_cut_sets(self, tree: FaultTree) -> List[Set[str]]:
        """Find minimal combinations of basic events that cause the top event.

        A cut set is a set of basic events whose simultaneous failure causes
        the top event.  A minimal cut set has no proper subset that is also a
        cut set.
        """
        raw_sets = self._cut_sets_for_node(tree, tree.root_id)

        # Minimise: remove any set that is a superset of another
        minimal: List[Set[str]] = []
        sorted_sets = sorted(raw_sets, key=len)
        for candidate in sorted_sets:
            if not any(existing <= candidate for existing in minimal):
                minimal.append(candidate)
        return minimal

    def _cut_sets_for_node(self, tree: FaultTree, node_id: str) -> List[Set[str]]:
        """Recursively compute cut sets for a node."""
        node = tree.nodes[node_id]

        if node.node_type == FaultTreeNodeType.EVENT:
            return [{node_id}]

        children_cut_sets = [
            self._cut_sets_for_node(tree, cid) for cid in node.children
        ]

        if node.gate_type == FaultTreeGateType.AND:
            # AND gate: cross product of children's cut sets
            result = children_cut_sets[0]
            for child_sets in children_cut_sets[1:]:
                new_result: List[Set[str]] = []
                for s1 in result:
                    for s2 in child_sets:
                        new_result.append(s1 | s2)
                result = new_result
            return result

        if node.gate_type == FaultTreeGateType.OR:
            # OR gate: union of all children's cut sets
            result: List[Set[str]] = []
            for child_sets in children_cut_sets:
                result.extend(child_sets)
            return result

        if node.gate_type == FaultTreeGateType.VOTING:
            # VOTING(k/n): union of cross products of k-subsets of children
            k = node.voting_k
            n = len(children_cut_sets)
            result: List[Set[str]] = []
            for child_indices in combinations(range(n), k):
                selected = [children_cut_sets[i] for i in child_indices]
                cross = selected[0]
                for s in selected[1:]:
                    new_cross: List[Set[str]] = []
                    for s1 in cross:
                        for s2 in s:
                            new_cross.append(s1 | s2)
                    cross = new_cross
                result.extend(cross)
            return result

        raise ValueError(f"Unknown gate type: {node.gate_type}")

    def importance_analysis(self, tree: FaultTree) -> Dict[str, float]:
        """Birnbaum importance for each basic event.

        Birnbaum importance = dP_top / dP_event, computed as:
            I_B(i) = P(top | event_i=1) - P(top | event_i=0)
        """
        basic_events = [
            n for n in tree.nodes.values()
            if n.node_type == FaultTreeNodeType.EVENT
        ]

        importances: Dict[str, float] = {}
        for event in basic_events:
            original_prob = event.probability

            # P(top | event = 1)
            event.probability = 1.0
            p_top_1 = self.evaluate(tree)

            # P(top | event = 0)
            event.probability = 0.0
            p_top_0 = self.evaluate(tree)

            # Restore
            event.probability = original_prob

            importances[event.node_id] = p_top_1 - p_top_0

        return importances

    def get_critical_path(self, tree: FaultTree) -> List[str]:
        """Return the path from root to a leaf with the highest probability contribution.

        At each gate, the child with the highest evaluated probability is
        selected, forming the critical path through the tree.
        """
        path: List[str] = []
        current_id = tree.root_id

        while True:
            path.append(current_id)
            node = tree.nodes[current_id]
            if node.node_type == FaultTreeNodeType.EVENT:
                break
            if not node.children:
                break
            # Pick child with highest probability
            best_child = max(
                node.children,
                key=lambda cid: self._evaluate_node(tree, cid),
            )
            current_id = best_child

        return path


# ---------------------------------------------------------------------------
# Decision Tree Classifier for Fault Diagnosis
# ---------------------------------------------------------------------------

@dataclass
class DiagnosisNode:
    """A node in a fault diagnosis decision tree."""
    node_id: str
    question: str
    feature: str
    threshold: float
    yes_child: Optional[str] = None
    no_child: Optional[str] = None
    diagnosis: Optional[str] = None
    confidence: float = 0.0
    recommended_action: str = ''


@dataclass
class DiagnosisResult:
    """Result returned by traversing the fault diagnosis tree."""
    diagnosis: str
    confidence: float
    path: List[str]
    recommended_action: str
    features_checked: Dict[str, Any]


class FaultDiagnosisTree:
    """Decision tree classifier for diagnosing manufacturing faults.

    Supports building a tree of yes/no questions based on sensor feature
    thresholds, traversing it with live feature values, and returning a
    structured diagnosis with confidence and recommended corrective action.
    """

    def __init__(self) -> None:
        self._nodes: Dict[str, DiagnosisNode] = {}
        self._root_id: Optional[str] = None

    # -- tree construction --

    def add_node(self, node: DiagnosisNode) -> None:
        """Add a diagnosis node to the tree."""
        self._nodes[node.node_id] = node

    def set_root(self, node_id: str) -> None:
        """Set the root node of the decision tree.

        Raises KeyError if the node_id has not been added yet.
        """
        if node_id not in self._nodes:
            raise KeyError(f"Node '{node_id}' not found in tree")
        self._root_id = node_id

    # -- diagnosis --

    def diagnose(self, features: Dict[str, float]) -> DiagnosisResult:
        """Traverse the tree using *features* and return a DiagnosisResult.

        At each internal node the feature value is compared against the
        threshold.  If the feature value exceeds the threshold the yes-child
        is followed, otherwise the no-child.  Traversal stops at a leaf
        (a node with a non-None ``diagnosis``).

        Raises RuntimeError if the tree has no root or a traversal reaches a
        dead-end (missing child reference).
        """
        if self._root_id is None:
            raise RuntimeError("No root node set for the diagnosis tree")

        path: List[str] = []
        features_checked: Dict[str, Any] = {}
        current_id = self._root_id

        while current_id is not None:
            node = self._nodes.get(current_id)
            if node is None:
                raise RuntimeError(f"Node '{current_id}' referenced but not found in tree")

            path.append(node.node_id)

            # Leaf node – return diagnosis
            if node.diagnosis is not None:
                return DiagnosisResult(
                    diagnosis=node.diagnosis,
                    confidence=node.confidence,
                    path=path,
                    recommended_action=node.recommended_action,
                    features_checked=features_checked,
                )

            # Internal node – evaluate feature threshold
            feature_value = features.get(node.feature)
            if feature_value is None:
                # Feature missing: treat as not exceeding threshold (no branch)
                features_checked[node.feature] = None
                current_id = node.no_child
            else:
                features_checked[node.feature] = feature_value
                if feature_value > node.threshold:
                    current_id = node.yes_child
                else:
                    current_id = node.no_child

        raise RuntimeError("Traversal ended without reaching a diagnosis leaf")

    # -- queries --

    def get_all_diagnoses(self) -> List[str]:
        """Return a sorted list of all possible leaf diagnoses in the tree."""
        diagnoses: List[str] = []
        for node in self._nodes.values():
            if node.diagnosis is not None:
                diagnoses.append(node.diagnosis)
        diagnoses.sort()
        return diagnoses

    # -- default tree --

    def build_default_tree(self) -> None:
        """Populate the tree with a pre-built CNC fault diagnosis decision tree.

        Structure::

            [vibration > 5.0 mm/s?]
              YES -> [cutting_force > 800 N?]
                       YES -> Chatter (conf 0.90)
                       NO  -> Bearing Wear (conf 0.80)
              NO  -> [temperature > 60 degC?]
                       YES -> [coolant_flow > 5 L/min?]
                                YES -> Thermal Drift (conf 0.75)
                                NO  -> Coolant System Failure (conf 0.85)
                       NO  -> [surface_roughness > Ra 3.2?]
                                YES -> [feed_rate > 500 mm/min?]
                                         YES -> Feed Rate Issue (conf 0.80)
                                         NO  -> Tool Wear (conf 0.85)
                                NO  -> Normal Operation (conf 0.95)
        """
        self._nodes.clear()
        self._root_id = None

        nodes = [
            DiagnosisNode(
                node_id='root',
                question='Is vibration > 5 mm/s?',
                feature='vibration',
                threshold=5.0,
                yes_child='check_force',
                no_child='check_temp',
            ),
            DiagnosisNode(
                node_id='check_force',
                question='Is cutting force > 800 N?',
                feature='cutting_force',
                threshold=800.0,
                yes_child='diag_chatter',
                no_child='diag_bearing',
            ),
            DiagnosisNode(
                node_id='diag_chatter',
                question='',
                feature='',
                threshold=0.0,
                diagnosis='Chatter',
                confidence=0.90,
                recommended_action='Reduce spindle speed and depth of cut',
            ),
            DiagnosisNode(
                node_id='diag_bearing',
                question='',
                feature='',
                threshold=0.0,
                diagnosis='Bearing Wear',
                confidence=0.80,
                recommended_action='Schedule bearing replacement',
            ),
            DiagnosisNode(
                node_id='check_temp',
                question='Is temperature > 60 °C?',
                feature='temperature',
                threshold=60.0,
                yes_child='check_coolant',
                no_child='check_roughness',
            ),
            DiagnosisNode(
                node_id='check_coolant',
                question='Is coolant flow > 5 L/min?',
                feature='coolant_flow',
                threshold=5.0,
                yes_child='diag_thermal_drift',
                no_child='diag_coolant',
            ),
            DiagnosisNode(
                node_id='diag_coolant',
                question='',
                feature='',
                threshold=0.0,
                diagnosis='Coolant System Failure',
                confidence=0.85,
                recommended_action='Inspect coolant pump and lines',
            ),
            DiagnosisNode(
                node_id='diag_thermal_drift',
                question='',
                feature='',
                threshold=0.0,
                diagnosis='Thermal Drift',
                confidence=0.75,
                recommended_action='Allow machine warm-up and recalibrate',
            ),
            DiagnosisNode(
                node_id='check_roughness',
                question='Is surface roughness > Ra 3.2?',
                feature='surface_roughness',
                threshold=3.2,
                yes_child='check_feed',
                no_child='diag_normal',
            ),
            DiagnosisNode(
                node_id='check_feed',
                question='Is feed rate > 500 mm/min?',
                feature='feed_rate',
                threshold=500.0,
                yes_child='diag_feed',
                no_child='diag_tool_wear',
            ),
            DiagnosisNode(
                node_id='diag_feed',
                question='',
                feature='',
                threshold=0.0,
                diagnosis='Feed Rate Issue',
                confidence=0.80,
                recommended_action='Reduce feed rate',
            ),
            DiagnosisNode(
                node_id='diag_tool_wear',
                question='',
                feature='',
                threshold=0.0,
                diagnosis='Tool Wear',
                confidence=0.85,
                recommended_action='Replace worn tool insert',
            ),
            DiagnosisNode(
                node_id='diag_normal',
                question='',
                feature='',
                threshold=0.0,
                diagnosis='Normal Operation',
                confidence=0.95,
                recommended_action='No action required',
            ),
        ]

        for n in nodes:
            self.add_node(n)
        self.set_root('root')

    # -- validation --

    def validate_tree(self) -> List[str]:
        """Validate the tree structure and return a list of error messages.

        Checks performed:
        - Root is set
        - All child references point to existing nodes
        - The tree is acyclic (no node is reachable from itself)
        - Every internal (non-leaf) node has at least one child
        - Every leaf node has a diagnosis

        Returns an empty list when the tree is valid.
        """
        errors: List[str] = []

        if self._root_id is None:
            errors.append("No root node set")
            return errors

        if self._root_id not in self._nodes:
            errors.append(f"Root node '{self._root_id}' not found in tree")
            return errors

        # Check child references
        for node in self._nodes.values():
            if node.diagnosis is not None:
                # Leaf node
                continue
            for child_attr in ('yes_child', 'no_child'):
                child_id = getattr(node, child_attr)
                if child_id is not None and child_id not in self._nodes:
                    errors.append(
                        f"Node '{node.node_id}': {child_attr} '{child_id}' not found"
                    )
            if node.yes_child is None and node.no_child is None:
                errors.append(
                    f"Internal node '{node.node_id}' has no children"
                )

        # Leaf nodes must have a diagnosis (already guaranteed by the
        # condition above but let's be explicit)
        for node in self._nodes.values():
            if node.yes_child is None and node.no_child is None and node.diagnosis is None:
                errors.append(
                    f"Leaf node '{node.node_id}' has no diagnosis"
                )

        # Acyclicity check via DFS
        visited: Set[str] = set()
        in_stack: Set[str] = set()

        def _dfs(nid: str) -> None:
            if nid not in self._nodes:
                return
            visited.add(nid)
            in_stack.add(nid)
            node = self._nodes[nid]
            for child_id in (node.yes_child, node.no_child):
                if child_id is None:
                    continue
                if child_id in in_stack:
                    errors.append(f"Cycle detected involving node '{child_id}'")
                elif child_id not in visited:
                    _dfs(child_id)
            in_stack.discard(nid)

        _dfs(self._root_id)

        return errors


# ---------------------------------------------------------------------------
# Adaptive Learning Rate Scheduler
# ---------------------------------------------------------------------------

@dataclass
class LearningEpisode:
    """Record of a single learning episode for adaptive model updates."""
    episode_id: int
    timestamp: float
    learning_rate: float
    loss: float
    metric_value: float
    model_name: str


@dataclass
class SchedulerConfig:
    """Configuration for the adaptive learning rate scheduler."""
    initial_lr: float = 0.01
    min_lr: float = 0.0001
    max_lr: float = 0.1
    patience: int = 5
    decay_factor: float = 0.5
    warmup_episodes: int = 10


class AdaptiveLearningScheduler:
    """Manages learning rate schedules for adaptive manufacturing models.

    Supports three scheduling strategies that compose together:
    - **Warmup**: linearly ramp from ``min_lr`` to ``initial_lr`` over the
      first ``warmup_episodes`` episodes.
    - **Plateau**: reduce the learning rate by ``decay_factor`` when the loss
      has not improved for ``patience`` consecutive episodes.
    - **Cosine annealing**: optionally apply cosine decay over a fixed cycle
      length, oscillating between the current LR and ``min_lr``.

    The scheduler tracks every episode and exposes helpers for early stopping,
    best-episode retrieval, and full history inspection.
    """

    def __init__(
        self,
        config: Optional[SchedulerConfig] = None,
        model_name: str = 'default',
        use_cosine_annealing: bool = False,
        cosine_cycle_length: int = 50,
    ) -> None:
        self._config = config or SchedulerConfig()
        self._model_name = model_name
        self._use_cosine_annealing = use_cosine_annealing
        self._cosine_cycle_length = max(cosine_cycle_length, 1)

        # Mutable state
        self._current_lr: float = self._config.initial_lr
        self._episodes: List[LearningEpisode] = []
        self._best_loss: Optional[float] = None
        self._episodes_since_improvement: int = 0
        self._episode_counter: int = 0

    # -- public API --

    def get_learning_rate(self) -> float:
        """Return the current learning rate."""
        return self._current_lr

    def step(self, loss: float, metric_value: float = 0.0) -> float:
        """Record an episode and adjust the learning rate.

        Parameters
        ----------
        loss : float
            Training loss for this episode.
        metric_value : float, optional
            An auxiliary metric (e.g. accuracy) to store alongside the loss.

        Returns
        -------
        float
            The learning rate that will be used for the *next* episode.
        """
        episode = LearningEpisode(
            episode_id=self._episode_counter,
            timestamp=time.time(),
            learning_rate=self._current_lr,
            loss=loss,
            metric_value=metric_value,
            model_name=self._model_name,
        )
        self._episodes.append(episode)
        self._episode_counter += 1

        # --- Warmup phase ---
        if self._episode_counter <= self._config.warmup_episodes:
            progress = self._episode_counter / self._config.warmup_episodes
            self._current_lr = (
                self._config.min_lr
                + (self._config.initial_lr - self._config.min_lr) * progress
            )
            return self._current_lr

        # --- Track improvement for plateau logic ---
        if self._best_loss is None or loss < self._best_loss:
            self._best_loss = loss
            self._episodes_since_improvement = 0
        else:
            self._episodes_since_improvement += 1

        # --- Plateau decay ---
        if self._episodes_since_improvement >= self._config.patience:
            self._current_lr = max(
                self._current_lr * self._config.decay_factor,
                self._config.min_lr,
            )
            self._episodes_since_improvement = 0

        # --- Cosine annealing (applied on top of current LR) ---
        if self._use_cosine_annealing:
            # Episode index within the current cosine cycle
            t = (self._episode_counter - self._config.warmup_episodes) % self._cosine_cycle_length
            cosine_factor = 0.5 * (1.0 + math.cos(math.pi * t / self._cosine_cycle_length))
            self._current_lr = (
                self._config.min_lr
                + (self._current_lr - self._config.min_lr) * cosine_factor
            )

        # Clamp
        self._current_lr = max(self._current_lr, self._config.min_lr)
        self._current_lr = min(self._current_lr, self._config.max_lr)

        return self._current_lr

    def reset(self) -> None:
        """Reset the scheduler to its initial state."""
        self._current_lr = self._config.initial_lr
        self._episodes.clear()
        self._best_loss = None
        self._episodes_since_improvement = 0
        self._episode_counter = 0

    def get_history(self) -> List[LearningEpisode]:
        """Return all recorded learning episodes."""
        return list(self._episodes)

    def get_best_episode(self) -> Optional[LearningEpisode]:
        """Return the episode with the lowest loss, or *None* if no episodes."""
        if not self._episodes:
            return None
        return min(self._episodes, key=lambda e: e.loss)

    def should_stop_early(
        self,
        min_episodes: int = 20,
        no_improve_limit: int = 10,
    ) -> bool:
        """Check whether training should stop early.

        Returns ``True`` when at least *min_episodes* have been recorded **and**
        the loss has not improved for *no_improve_limit* consecutive episodes.
        """
        if len(self._episodes) < min_episodes:
            return False
        return self._episodes_since_improvement >= no_improve_limit


# ---------------------------------------------------------------------------
# Causal Impact Analyzer
# ---------------------------------------------------------------------------

@dataclass
class ParameterChange:
    """A recorded change to a manufacturing parameter."""
    parameter_name: str
    old_value: float
    new_value: float
    timestamp: float
    change_type: str  # 'manual' | 'automatic' | 'drift'

    @property
    def magnitude(self) -> float:
        """Absolute magnitude of the change."""
        return abs(self.new_value - self.old_value)

    @property
    def direction_sign(self) -> float:
        """Sign of the change: +1, -1, or 0."""
        diff = self.new_value - self.old_value
        if diff > 0:
            return 1.0
        elif diff < 0:
            return -1.0
        return 0.0


@dataclass
class OutcomeObservation:
    """An observed outcome metric at a point in time."""
    metric_name: str
    value: float
    timestamp: float


@dataclass
class ImpactResult:
    """Result of analysing the impact of a parameter on an outcome metric."""
    parameter_name: str
    metric_name: str
    correlation: float
    impact_magnitude: float
    confidence: float
    direction: str  # 'positive' | 'negative' | 'neutral'
    lag_sec: float


@dataclass
class ImpactReport:
    """Aggregated report of parameter impacts on manufacturing outcomes."""
    changes: List['ParameterChange']
    impacts: List[ImpactResult]
    summary: str
    timestamp: float


class CausalImpactAnalyzer:
    """Analyses the impact of parameter changes on manufacturing outcomes.

    Records parameter changes and outcome metric observations, then computes
    Pearson correlation between change magnitudes and outcome deltas within
    configurable time windows. This enables operators to understand which
    parameter adjustments have the largest effect on quality, throughput, and
    other KPIs.
    """

    def __init__(self) -> None:
        self._changes: List[ParameterChange] = []
        self._outcomes: List[OutcomeObservation] = []
        self._lock = threading.Lock()

    # -- recording --

    def record_change(self, change: ParameterChange) -> None:
        """Record a parameter change event."""
        with self._lock:
            self._changes.append(change)

    def record_outcome(self, observation: OutcomeObservation) -> None:
        """Record an outcome metric observation."""
        with self._lock:
            self._outcomes.append(observation)

    # -- analysis helpers --

    @staticmethod
    def _pearson(xs: List[float], ys: List[float]) -> float:
        """Compute Pearson correlation coefficient for two equal-length lists.

        Returns 0.0 when the inputs are too short or have zero variance.
        """
        n = len(xs)
        if n < 2 or n != len(ys):
            return 0.0

        mean_x = sum(xs) / n
        mean_y = sum(ys) / n

        cov = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
        var_x = sum((x - mean_x) ** 2 for x in xs)
        var_y = sum((y - mean_y) ** 2 for y in ys)

        denom = math.sqrt(var_x * var_y)
        if denom == 0.0:
            return 0.0
        return cov / denom

    def _outcomes_for_metric(self, metric_name: str) -> List[OutcomeObservation]:
        """Return outcome observations for the given metric, sorted by time."""
        return sorted(
            [o for o in self._outcomes if o.metric_name == metric_name],
            key=lambda o: o.timestamp,
        )

    def _outcome_delta_after(
        self, metric_name: str, t_start: float, window_sec: float,
    ) -> Optional[float]:
        """Compute the outcome metric change within a window after *t_start*.

        Returns the difference between the last and first observations found
        in the interval ``[t_start, t_start + window_sec]``, or *None* if
        fewer than two observations fall inside the window.
        """
        t_end = t_start + window_sec
        relevant = [
            o for o in self._outcomes
            if o.metric_name == metric_name and t_start <= o.timestamp <= t_end
        ]
        relevant.sort(key=lambda o: o.timestamp)
        if len(relevant) < 2:
            return None
        return relevant[-1].value - relevant[0].value

    # -- public analysis API --

    def analyze_impact(
        self,
        parameter_name: str,
        metric_name: str,
        window_sec: float = 60.0,
    ) -> ImpactResult:
        """Compute the impact of *parameter_name* changes on *metric_name*.

        For every recorded change of *parameter_name*, the method looks at the
        outcome metric delta within *window_sec* seconds after the change.  It
        then computes the Pearson correlation between change magnitudes and
        outcome deltas.

        Parameters
        ----------
        parameter_name : str
            Name of the manufacturing parameter whose changes to analyse.
        metric_name : str
            Name of the outcome metric to correlate against.
        window_sec : float
            Duration (seconds) after each change within which to measure
            outcome deltas.

        Returns
        -------
        ImpactResult
        """
        with self._lock:
            changes = [
                c for c in self._changes
                if c.parameter_name == parameter_name
            ]
            changes.sort(key=lambda c: c.timestamp)

            change_magnitudes: List[float] = []
            outcome_deltas: List[float] = []
            lags: List[float] = []

            for c in changes:
                delta = self._outcome_delta_after(metric_name, c.timestamp, window_sec)
                if delta is not None:
                    change_magnitudes.append(c.new_value - c.old_value)
                    outcome_deltas.append(delta)
                    # Lag is the midpoint of the observation window used
                    t_end = c.timestamp + window_sec
                    relevant = [
                        o for o in self._outcomes
                        if o.metric_name == metric_name
                        and c.timestamp <= o.timestamp <= t_end
                    ]
                    if relevant:
                        avg_ts = sum(o.timestamp for o in relevant) / len(relevant)
                        lags.append(avg_ts - c.timestamp)
                    else:
                        lags.append(window_sec / 2.0)

            correlation = self._pearson(change_magnitudes, outcome_deltas)

            if outcome_deltas:
                impact_magnitude = sum(abs(d) for d in outcome_deltas) / len(outcome_deltas)
            else:
                impact_magnitude = 0.0

            n = len(change_magnitudes)
            confidence = min(1.0, n / 10.0) if n > 0 else 0.0

            if abs(correlation) < 0.1:
                direction = 'neutral'
            elif correlation > 0:
                direction = 'positive'
            else:
                direction = 'negative'

            avg_lag = sum(lags) / len(lags) if lags else 0.0

            return ImpactResult(
                parameter_name=parameter_name,
                metric_name=metric_name,
                correlation=correlation,
                impact_magnitude=impact_magnitude,
                confidence=confidence,
                direction=direction,
                lag_sec=avg_lag,
            )

    def get_impact_report(self, window_sec: float = 60.0) -> ImpactReport:
        """Analyse all recorded parameter changes against all observed metrics.

        Returns an :class:`ImpactReport` with a list of :class:`ImpactResult`
        entries for every (parameter, metric) combination that has data.
        """
        with self._lock:
            param_names = list({c.parameter_name for c in self._changes})
            metric_names = list({o.metric_name for o in self._outcomes})
            all_changes = list(self._changes)

        impacts: List[ImpactResult] = []
        for pname in param_names:
            for mname in metric_names:
                result = self.analyze_impact(pname, mname, window_sec)
                impacts.append(result)

        impacts.sort(key=lambda r: abs(r.correlation), reverse=True)

        if impacts:
            top = impacts[0]
            summary = (
                f"Top impact: '{top.parameter_name}' -> '{top.metric_name}' "
                f"(correlation={top.correlation:.3f}, "
                f"magnitude={top.impact_magnitude:.3f})"
            )
        else:
            summary = "No impacts detected — insufficient data."

        return ImpactReport(
            changes=all_changes,
            impacts=impacts,
            summary=summary,
            timestamp=time.time(),
        )

    def get_most_impactful_change(self) -> Optional[ImpactResult]:
        """Return the :class:`ImpactResult` with the highest absolute impact.

        Impact is ranked first by ``abs(correlation)`` and then by
        ``impact_magnitude`` as a tiebreaker.  Returns *None* when no impacts
        have been computed yet.
        """
        report = self.get_impact_report()
        if not report.impacts:
            return None
        return max(
            report.impacts,
            key=lambda r: (abs(r.correlation), r.impact_magnitude),
        )


# ---------------------------------------------------------------------------
# Feature Importance Ranker
# ---------------------------------------------------------------------------


@dataclass
class FeatureScore:
    """Importance score for a single input feature."""
    feature_name: str
    importance_score: float
    rank: int
    correlation: float
    direction: str  # 'positive' | 'negative' | 'neutral'


@dataclass
class ImportanceReport:
    """Full report of feature importance analysis."""
    features: List[FeatureScore]
    target_metric: str
    method: str
    total_features: int
    timestamp: float


class FeatureImportanceRanker:
    """Ranks input features by their importance to manufacturing outcomes.

    Supports three methods for computing feature importance:

    * **correlation** -- Pearson |r| between each feature and the target.
    * **mutual_information** -- Binned mutual-information approximation.
    * **permutation** -- Measures how much target variance increases when a
      feature column is randomly shuffled.

    Typical usage::

        ranker = FeatureImportanceRanker()
        ranker.train(data, target)
        top = ranker.rank_features(top_n=5)
    """

    _VALID_METHODS = ('correlation', 'mutual_information', 'permutation')

    def __init__(self, method: str = 'permutation', n_bins: int = 10,
                 n_repeats: int = 5, seed: int = 42) -> None:
        self._method = method
        self._n_bins = n_bins
        self._n_repeats = n_repeats
        self._seed = seed
        self._data: Dict[str, List[float]] = {}
        self._target: List[float] = []
        self._scores: Dict[str, FeatureScore] = {}
        self._trained = False

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _pearson(xs: List[float], ys: List[float]) -> float:
        """Pearson correlation coefficient.  Returns 0.0 on degenerate input."""
        n = len(xs)
        if n < 2 or n != len(ys):
            return 0.0
        mean_x = sum(xs) / n
        mean_y = sum(ys) / n
        cov = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
        var_x = sum((x - mean_x) ** 2 for x in xs)
        var_y = sum((y - mean_y) ** 2 for y in ys)
        denom = math.sqrt(var_x * var_y)
        if denom == 0.0:
            return 0.0
        return cov / denom

    @staticmethod
    def _variance(values: List[float]) -> float:
        """Population variance of *values*."""
        n = len(values)
        if n == 0:
            return 0.0
        mean = sum(values) / n
        return sum((v - mean) ** 2 for v in values) / n

    def _bin_values(self, values: List[float]) -> List[int]:
        """Assign each value to an equal-width bin index."""
        lo = min(values)
        hi = max(values)
        if hi == lo:
            return [0] * len(values)
        width = (hi - lo) / self._n_bins
        return [min(int((v - lo) / width), self._n_bins - 1) for v in values]

    def _mutual_information(self, xs: List[float], ys: List[float]) -> float:
        """Binned mutual-information approximation."""
        n = len(xs)
        if n < 2:
            return 0.0
        bx = self._bin_values(xs)
        by = self._bin_values(ys)

        # Joint and marginal counts
        joint: Dict[Tuple[int, int], int] = {}
        cx: Dict[int, int] = {}
        cy: Dict[int, int] = {}
        for i in range(n):
            key = (bx[i], by[i])
            joint[key] = joint.get(key, 0) + 1
            cx[bx[i]] = cx.get(bx[i], 0) + 1
            cy[by[i]] = cy.get(by[i], 0) + 1

        mi = 0.0
        for (a, b), count in joint.items():
            p_xy = count / n
            p_x = cx[a] / n
            p_y = cy[b] / n
            if p_xy > 0 and p_x > 0 and p_y > 0:
                mi += p_xy * math.log(p_xy / (p_x * p_y))
        return max(mi, 0.0)

    def _permutation_importance(self, feature: List[float],
                                target: List[float]) -> float:
        """Permutation importance: mean increase in target prediction error
        when *feature* is shuffled."""
        import random
        rng = random.Random(self._seed)
        n = len(target)
        if n < 2:
            return 0.0

        # Baseline: variance of residuals using simple linear prediction
        r = self._pearson(feature, target)
        baseline_var = self._variance(target) * (1.0 - r * r)

        shuffled_vars: List[float] = []
        for _ in range(self._n_repeats):
            perm = list(feature)
            rng.shuffle(perm)
            r_perm = self._pearson(perm, target)
            perm_var = self._variance(target) * (1.0 - r_perm * r_perm)
            shuffled_vars.append(perm_var)

        mean_shuffled = sum(shuffled_vars) / len(shuffled_vars)
        target_var = self._variance(target)
        if target_var == 0.0:
            return 0.0
        return max((mean_shuffled - baseline_var) / target_var, 0.0)

    def _compute_importance(self, feature: List[float],
                            target: List[float]) -> float:
        """Compute importance score using the configured method."""
        if self._method == 'correlation':
            return abs(self._pearson(feature, target))
        elif self._method == 'mutual_information':
            return self._mutual_information(feature, target)
        elif self._method == 'permutation':
            return self._permutation_importance(feature, target)
        raise ValueError(f"Unknown method: {self._method}")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def train(self, data: Dict[str, List[float]],
              target: List[float]) -> None:
        """Compute feature correlations and importance scores.

        Parameters
        ----------
        data : dict
            Mapping of feature name to list of float observations.
        target : list of float
            Target outcome values aligned with the feature observations.
        """
        if not data or not target:
            raise ValueError("data and target must be non-empty")

        n = len(target)
        for name, values in data.items():
            if len(values) != n:
                raise ValueError(
                    f"Feature '{name}' has {len(values)} samples, "
                    f"expected {n}")

        self._data = {k: list(v) for k, v in data.items()}
        self._target = list(target)

        raw_scores: Dict[str, Tuple[float, float]] = {}
        for name, values in self._data.items():
            corr = self._pearson(values, self._target)
            importance = self._compute_importance(values, self._target)
            raw_scores[name] = (importance, corr)

        # Sort by importance descending to assign ranks
        ranked = sorted(raw_scores.items(), key=lambda item: item[1][0],
                        reverse=True)

        self._scores = {}
        for rank_idx, (name, (imp, corr)) in enumerate(ranked, start=1):
            if abs(corr) < 0.1:
                direction = 'neutral'
            elif corr > 0:
                direction = 'positive'
            else:
                direction = 'negative'
            self._scores[name] = FeatureScore(
                feature_name=name,
                importance_score=imp,
                rank=rank_idx,
                correlation=corr,
                direction=direction,
            )

        self._trained = True

    def rank_features(self, top_n: Optional[int] = None) -> List[FeatureScore]:
        """Return features sorted by importance, optionally limited to *top_n*.

        Parameters
        ----------
        top_n : int or None
            If given, only the top *top_n* features are returned.

        Returns
        -------
        list of FeatureScore
        """
        if not self._trained:
            raise RuntimeError("Must call train() before rank_features()")
        ordered = sorted(self._scores.values(), key=lambda s: s.rank)
        if top_n is not None:
            ordered = ordered[:top_n]
        return ordered

    def get_importance(self, feature_name: str) -> FeatureScore:
        """Return the :class:`FeatureScore` for a specific feature.

        Raises
        ------
        KeyError
            If the feature was not part of the training data.
        """
        if not self._trained:
            raise RuntimeError("Must call train() before get_importance()")
        if feature_name not in self._scores:
            raise KeyError(f"Unknown feature: {feature_name}")
        return self._scores[feature_name]

    def get_report(self, target_metric_name: str) -> ImportanceReport:
        """Generate a full :class:`ImportanceReport`.

        Parameters
        ----------
        target_metric_name : str
            Human-readable name of the target metric (stored in the report).

        Returns
        -------
        ImportanceReport
        """
        if not self._trained:
            raise RuntimeError("Must call train() before get_report()")
        features = self.rank_features()
        return ImportanceReport(
            features=features,
            target_metric=target_metric_name,
            method=self._method,
            total_features=len(features),
            timestamp=time.time(),
        )

    def compare_features(self, feature_a: str,
                         feature_b: str) -> Dict[str, Any]:
        """Compare two features' relative importance.

        Returns a dict with keys: ``feature_a``, ``feature_b``,
        ``more_important``, ``importance_difference``, and
        ``correlation_difference``.
        """
        if not self._trained:
            raise RuntimeError("Must call train() before compare_features()")
        sa = self.get_importance(feature_a)
        sb = self.get_importance(feature_b)
        if sa.importance_score >= sb.importance_score:
            more_important = feature_a
        else:
            more_important = feature_b
        return {
            'feature_a': sa,
            'feature_b': sb,
            'more_important': more_important,
            'importance_difference': abs(sa.importance_score - sb.importance_score),
            'correlation_difference': abs(sa.correlation - sb.correlation),
        }

    def identify_redundant_features(
        self, threshold: float = 0.9,
    ) -> List[Tuple[str, str, float]]:
        """Find highly correlated feature pairs.

        Returns a list of ``(feature_a, feature_b, |r|)`` tuples where the
        absolute Pearson correlation exceeds *threshold*.
        """
        if not self._trained:
            raise RuntimeError(
                "Must call train() before identify_redundant_features()")
        names = list(self._data.keys())
        pairs: List[Tuple[str, str, float]] = []
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                r = abs(self._pearson(self._data[names[i]],
                                      self._data[names[j]]))
                if r > threshold:
                    pairs.append((names[i], names[j], r))
        pairs.sort(key=lambda t: t[2], reverse=True)
        return pairs


# ---------------------------------------------------------------------------
# FMEA Risk Priority Number Calculator
# ---------------------------------------------------------------------------


@dataclass
class FailureMode:
    """A single failure mode entry for FMEA analysis."""

    mode_id: str
    description: str
    component: str
    severity: int        # 1-10
    occurrence: int      # 1-10
    detection: int       # 1-10
    rpn: int = 0
    current_controls: str = ""
    recommended_action: str = ""


@dataclass
class FMEAReport:
    """Aggregated FMEA report across all registered failure modes."""

    failure_modes: List['FailureMode']
    total_rpn: int
    avg_rpn: float
    high_risk_count: int
    critical_modes: List['FailureMode']
    timestamp: float = 0.0


class FMEARiskCalculator:
    """Calculates Failure Mode and Effects Analysis Risk Priority Numbers.

    Provides registration of failure modes, RPN calculation, risk
    classification, report generation, and before/after comparison for
    tracking the effectiveness of corrective actions.
    """

    def __init__(self) -> None:
        self._modes: Dict[str, FailureMode] = {}

    # -- core helpers -------------------------------------------------------

    @staticmethod
    def calculate_rpn(severity: int, occurrence: int, detection: int) -> int:
        """Compute Risk Priority Number = Severity * Occurrence * Detection.

        Each factor must be in the range [1, 10].
        """
        for name, val in [('severity', severity),
                          ('occurrence', occurrence),
                          ('detection', detection)]:
            if not (1 <= val <= 10):
                raise ValueError(
                    f"{name} must be between 1 and 10, got {val}")
        return severity * occurrence * detection

    @staticmethod
    def get_risk_level(rpn: int) -> str:
        """Classify an RPN value into a risk level string.

        * ``'low'``      – RPN < 50
        * ``'medium'``   – 50 <= RPN <= 100
        * ``'high'``     – 100 < RPN <= 200
        * ``'critical'`` – RPN > 200
        """
        if rpn < 50:
            return 'low'
        elif rpn <= 100:
            return 'medium'
        elif rpn <= 200:
            return 'high'
        else:
            return 'critical'

    # -- mode management ----------------------------------------------------

    def add_failure_mode(self, mode: FailureMode) -> FailureMode:
        """Register a failure mode, auto-calculating its RPN."""
        mode.rpn = self.calculate_rpn(
            mode.severity, mode.occurrence, mode.detection)
        self._modes[mode.mode_id] = mode
        return mode

    def get_critical_modes(self, threshold: int = 125) -> List[FailureMode]:
        """Return failure modes whose RPN exceeds *threshold*, sorted desc."""
        critical = [m for m in self._modes.values() if m.rpn > threshold]
        critical.sort(key=lambda m: m.rpn, reverse=True)
        return critical

    # -- reporting ----------------------------------------------------------

    def get_report(self) -> FMEAReport:
        """Generate a full FMEA report with modes sorted by RPN descending."""
        modes = sorted(self._modes.values(),
                       key=lambda m: m.rpn, reverse=True)
        total = sum(m.rpn for m in modes)
        avg = total / len(modes) if modes else 0.0
        high_risk = sum(1 for m in modes if m.rpn > 200)
        critical = [m for m in modes if m.rpn > 125]
        return FMEAReport(
            failure_modes=modes,
            total_rpn=total,
            avg_rpn=avg,
            high_risk_count=high_risk,
            critical_modes=critical,
            timestamp=time.time(),
        )

    # -- recommendations ----------------------------------------------------

    def suggest_action(self, mode: FailureMode) -> str:
        """Suggest a corrective action targeting the highest contributing factor.

        The suggestion targets whichever of severity, occurrence, or
        detection is the largest contributor to the RPN.
        """
        factors = {
            'severity': mode.severity,
            'occurrence': mode.occurrence,
            'detection': mode.detection,
        }
        worst = max(factors, key=factors.get)  # type: ignore[arg-type]
        suggestions = {
            'severity': (
                f"Reduce severity (currently {mode.severity}/10): "
                "redesign the component or add redundancy to mitigate the "
                "effect of failure mode '{mode.description}'."
            ),
            'occurrence': (
                f"Reduce occurrence (currently {mode.occurrence}/10): "
                "improve process controls, use higher-reliability parts, "
                "or add preventive maintenance for '{mode.component}'."
            ),
            'detection': (
                f"Improve detection (currently {mode.detection}/10): "
                "add sensors, inspections, or automated monitoring to "
                "detect '{mode.description}' earlier."
            ),
        }
        return suggestions[worst]

    # -- before / after comparison ------------------------------------------

    def compare_before_after(
        self,
        mode_id: str,
        new_severity: int,
        new_occurrence: int,
        new_detection: int,
    ) -> Dict[str, Any]:
        """Compare original RPN with a proposed new set of S/O/D ratings.

        Returns a dict with keys ``before_rpn``, ``after_rpn``,
        ``rpn_reduction``, ``reduction_pct``, ``before_risk``, and
        ``after_risk``.
        """
        if mode_id not in self._modes:
            raise KeyError(f"Unknown failure mode: {mode_id}")
        original = self._modes[mode_id]
        before_rpn = original.rpn
        after_rpn = self.calculate_rpn(
            new_severity, new_occurrence, new_detection)
        reduction = before_rpn - after_rpn
        pct = (reduction / before_rpn * 100.0) if before_rpn else 0.0
        return {
            'before_rpn': before_rpn,
            'after_rpn': after_rpn,
            'rpn_reduction': reduction,
            'reduction_pct': round(pct, 2),
            'before_risk': self.get_risk_level(before_rpn),
            'after_risk': self.get_risk_level(after_rpn),
        }


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
