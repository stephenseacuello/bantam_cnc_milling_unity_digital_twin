"""Tests for forward causal simulation in the causal inference engine.

Validates physics-based transfer functions, forward propagation,
causal path finding, intervention simulation, and intervention point
discovery -- all without a ROS2 installation.
"""

import math
import os
import sys
import threading
import time
from collections import deque
from typing import Dict, List, Optional, Tuple
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Stub ROS2 / project dependencies
# ---------------------------------------------------------------------------
for mod in ['rclpy', 'rclpy.node', 'rclpy.callback_groups', 'rclpy.qos',
            'rclpy.lifecycle', 'rclpy.executors',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_core.heartbeat_mixin',
            'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

_base = sys.modules['miracle_core.lifecycle_node_base']
_base.MiracleLifecycleNode = type('FakeNode', (), {
    'CRITICALITY_LOW': 'LOW',
    'CRITICALITY_MEDIUM': 'MEDIUM',
    'CRITICALITY_HIGH': 'HIGH',
    'CRITICALITY_CRITICAL': 'CRITICAL',
    '__init__': lambda self, *a, **kw: None,
    'get_logger': lambda self: MagicMock(),
    'create_publisher': lambda self, *a, **kw: MagicMock(),
    'create_subscription': lambda self, *a, **kw: MagicMock(),
    'create_timer': lambda self, *a, **kw: MagicMock(),
    'declare_and_validate_parameters': lambda self, specs: {
        k: MagicMock(value=v['default']) for k, v in specs.items()
    },
    'get_parameter': lambda self, name: MagicMock(value=0),
})

sys.modules.pop('miracle_cognitive.knowledge.causal_inference', None)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_cognitive.knowledge.causal_inference import (
    CausalLink, CausalPath, InterventionResult, CausalInferenceNode,
)


# ---------------------------------------------------------------------------
# Lightweight engine that mirrors CausalInferenceNode forward-sim methods
# without ROS2 dependencies.
# ---------------------------------------------------------------------------

class _ForwardCausalEngine:
    """Pure-logic forward causal engine extracted from CausalInferenceNode."""

    PHYSICS_TRANSFER_MAP = CausalInferenceNode.PHYSICS_TRANSFER_MAP

    def __init__(self, half_life: float = 3600.0):
        self._causal_graph: Dict[str, List[CausalLink]] = {}
        self._lock = threading.RLock()
        self._decay_half_life_sec = half_life

    def add_link(self, link: CausalLink) -> None:
        self._causal_graph.setdefault(link.cause, []).append(link)

    @staticmethod
    def compute_decay_factor(time_since: float, half_life: float) -> float:
        if half_life <= 0.0:
            return 1.0
        return math.pow(2.0, -time_since / half_life)

    def _effective_strength(self, link: CausalLink,
                            now: Optional[float] = None) -> float:
        if now is None:
            now = time.time()
        if link.last_evidence_time <= 0.0:
            return 0.0
        dt = now - link.last_evidence_time
        return link.strength * self.compute_decay_factor(dt, link.decay_half_life_sec)

    # -- forward simulation (mirrors CausalInferenceNode) ------------------

    def _get_physics_transfer_function(self, cause: str, effect: str):
        multiplier = self.PHYSICS_TRANSFER_MAP.get((cause, effect))
        if multiplier is not None:
            return lambda change_pct: change_pct * multiplier
        with self._lock:
            if cause in self._causal_graph:
                for link in self._causal_graph[cause]:
                    if link.effect == effect:
                        s = link.strength
                        return lambda change_pct, _s=s: change_pct * _s
        return None

    def _forward_propagate(self, node: str, change_pct: float,
                           visited: Optional[set] = None) -> Dict[str, float]:
        if visited is None:
            visited = {node}
        effects: Dict[str, float] = {}

        for (cause, effect), _mult in self.PHYSICS_TRANSFER_MAP.items():
            if cause == node and effect not in visited:
                tf = self._get_physics_transfer_function(cause, effect)
                if tf is not None:
                    propagated_change = tf(change_pct)
                    effects[effect] = propagated_change
                    visited.add(effect)
                    downstream = self._forward_propagate(effect, propagated_change, visited)
                    for k, v in downstream.items():
                        if k not in effects:
                            effects[k] = v
                        elif abs(v) > abs(effects[k]):
                            effects[k] = v

        with self._lock:
            links = self._causal_graph.get(node, [])
            for link in links:
                if link.effect not in visited and link.effect not in effects:
                    tf = self._get_physics_transfer_function(node, link.effect)
                    if tf is not None:
                        propagated_change = tf(change_pct)
                    else:
                        propagated_change = change_pct * link.strength
                    effects[link.effect] = propagated_change
                    visited.add(link.effect)
                    downstream = self._forward_propagate(link.effect, propagated_change, visited)
                    for k, v in downstream.items():
                        if k not in effects:
                            effects[k] = v
                        elif abs(v) > abs(effects[k]):
                            effects[k] = v

        return effects

    def find_all_causal_paths(self, source: str, target: str,
                              max_depth: int = 5) -> List[CausalPath]:
        now = time.time()
        all_paths: List[CausalPath] = []
        queue = [(source, [source], 1.0)]

        while queue:
            current, path, strength_prod = queue.pop(0)
            if len(path) > max_depth + 1:
                continue
            with self._lock:
                links = self._causal_graph.get(current, [])
                for link in links:
                    if link.effect in path:
                        continue
                    new_path = path + [link.effect]
                    eff_strength = self._effective_strength(link, now)
                    new_strength = strength_prod * (eff_strength if eff_strength > 0 else link.strength)
                    decay = self.compute_decay_factor(
                        now - link.last_evidence_time if link.last_evidence_time > 0 else 0,
                        link.decay_half_life_sec,
                    )
                    if link.effect == target:
                        all_paths.append(CausalPath(
                            nodes=new_path,
                            strength=new_strength,
                            decay_factor=decay,
                        ))
                    else:
                        queue.append((link.effect, new_path, new_strength))
        return all_paths

    def simulate_intervention(self, intervention: str, change_pct: float) -> List[InterventionResult]:
        propagated = self._forward_propagate(intervention, change_pct)
        if not propagated:
            return []

        results = []
        now = time.time()
        for effect_name, effect_change in propagated.items():
            paths = self.find_all_causal_paths(intervention, effect_name)
            if paths:
                best_path = max(paths, key=lambda p: p.strength)
                confidence = best_path.strength * best_path.decay_factor
                confidence *= math.pow(0.9, len(best_path.nodes) - 2)
            else:
                confidence = 0.5

            side_effects = []
            for other_effect, other_change in propagated.items():
                if other_effect != effect_name:
                    direction = "increase" if other_change > 0 else "decrease"
                    side_effects.append(
                        f"{other_effect} will {direction} by {abs(other_change):.1f}%"
                    )

            direction = "increase" if effect_change > 0 else "decrease"
            reasoning = (
                f"Changing {intervention} by {change_pct:+.1f}% is predicted to "
                f"{direction} {effect_name} by {abs(effect_change):.1f}% "
                f"(confidence: {confidence:.2f})"
            )
            results.append(InterventionResult(
                intervention=intervention,
                intervention_value=change_pct,
                affected_effects={effect_name: effect_change},
                confidence=min(1.0, max(0.0, confidence)),
                reasoning=reasoning,
                side_effects=side_effects,
            ))
        return results

    def get_intervention_points(self, target_effect: str) -> List[Tuple[str, float, float]]:
        interventions: Dict[str, Tuple[float, float]] = {}
        now = time.time()

        for (cause, effect), multiplier in self.PHYSICS_TRANSFER_MAP.items():
            if effect == target_effect:
                impact = abs(multiplier * 10.0)
                confidence = 0.9
                if cause not in interventions or impact > interventions[cause][0]:
                    interventions[cause] = (impact, confidence)

        with self._lock:
            for cause_name in self._causal_graph:
                paths = self.find_all_causal_paths(cause_name, target_effect)
                for path in paths:
                    impact = path.strength * 10.0
                    conf = path.strength * path.decay_factor * math.pow(0.9, len(path.nodes) - 2)
                    if cause_name not in interventions or impact > interventions[cause_name][0]:
                        interventions[cause_name] = (impact, min(1.0, conf))

        result = [
            (name, impact, conf)
            for name, (impact, conf) in interventions.items()
        ]
        result.sort(key=lambda x: x[1], reverse=True)
        return result


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_link(cause='A', effect='B', strength=0.8, half_life=3600.0,
               last_evidence_time=None, evidence_timestamps=None):
    now = time.time()
    return CausalLink(
        cause=cause,
        effect=effect,
        strength=strength,
        evidence_count=len(evidence_timestamps) if evidence_timestamps else 0,
        last_evidence_time=last_evidence_time if last_evidence_time is not None else now,
        evidence_timestamps=list(evidence_timestamps) if evidence_timestamps else [],
        decay_half_life_sec=half_life,
    )


def _engine(links=None, half_life=3600.0):
    eng = _ForwardCausalEngine(half_life=half_life)
    if links:
        for link in links:
            eng.add_link(link)
    return eng


# ---------------------------------------------------------------------------
# Tests (25 total)
# ---------------------------------------------------------------------------

class TestPhysicsTransferFunctions:
    """Tests for physics-based transfer function accuracy."""

    def test_feed_rate_to_cutting_force(self):
        """F proportional to f^0.8: -20% feed -> -16% force."""
        eng = _engine()
        tf = eng._get_physics_transfer_function('feed_rate', 'cutting_force')
        assert tf is not None
        result = tf(-20.0)
        assert result == pytest.approx(-20.0 * 0.8, abs=0.01)

    def test_feed_rate_to_surface_roughness(self):
        """Ra proportional to f^2: -20% feed -> -40% Ra."""
        eng = _engine()
        tf = eng._get_physics_transfer_function('feed_rate', 'surface_roughness')
        assert tf is not None
        result = tf(-20.0)
        assert result == pytest.approx(-40.0, abs=0.01)

    def test_feed_rate_to_tool_life(self):
        """Tool life proportional to 1/f^2: -20% feed -> +40% life."""
        eng = _engine()
        tf = eng._get_physics_transfer_function('feed_rate', 'tool_life')
        assert tf is not None
        result = tf(-20.0)
        assert result == pytest.approx(40.0, abs=0.01)

    def test_feed_rate_to_cycle_time(self):
        """Cycle time inversely proportional: -20% feed -> +20% cycle time."""
        eng = _engine()
        tf = eng._get_physics_transfer_function('feed_rate', 'cycle_time')
        assert tf is not None
        result = tf(-20.0)
        assert result == pytest.approx(20.0, abs=0.01)

    def test_spindle_speed_to_temperature(self):
        """T proportional to V^0.5: +20% speed -> +10% temp."""
        eng = _engine()
        tf = eng._get_physics_transfer_function('spindle_speed', 'temperature')
        assert tf is not None
        result = tf(20.0)
        assert result == pytest.approx(10.0, abs=0.01)

    def test_depth_of_cut_to_cutting_force(self):
        """Force linear with depth: +30% depth -> +30% force."""
        eng = _engine()
        tf = eng._get_physics_transfer_function('depth_of_cut', 'cutting_force')
        assert tf is not None
        result = tf(30.0)
        assert result == pytest.approx(30.0, abs=0.01)

    def test_coolant_flow_to_temperature(self):
        """Coolant reduces temp: +50% coolant -> -30% temp."""
        eng = _engine()
        tf = eng._get_physics_transfer_function('coolant_flow', 'temperature')
        assert tf is not None
        result = tf(50.0)
        assert result == pytest.approx(-30.0, abs=0.01)

    def test_tool_wear_to_surface_roughness(self):
        """Worn tool rougher: +10% wear -> +5% roughness."""
        eng = _engine()
        tf = eng._get_physics_transfer_function('tool_wear', 'surface_roughness')
        assert tf is not None
        result = tf(10.0)
        assert result == pytest.approx(5.0, abs=0.01)

    def test_tool_wear_to_cutting_force(self):
        """Edge buildup: +10% wear -> +3% force."""
        eng = _engine()
        tf = eng._get_physics_transfer_function('tool_wear', 'cutting_force')
        assert tf is not None
        result = tf(10.0)
        assert result == pytest.approx(3.0, abs=0.01)

    def test_unknown_pair_returns_none(self):
        """Unknown cause-effect pair with no graph link returns None."""
        eng = _engine()
        tf = eng._get_physics_transfer_function('humidity', 'cutting_force')
        assert tf is None

    def test_fallback_to_graph_link_strength(self):
        """When no physics model, use causal link strength as multiplier."""
        link = _make_link(cause='vibration', effect='chatter', strength=0.7)
        eng = _engine([link])
        tf = eng._get_physics_transfer_function('vibration', 'chatter')
        assert tf is not None
        result = tf(10.0)
        assert result == pytest.approx(7.0, abs=0.01)


class TestForwardPropagation:
    """Tests for _forward_propagate through the causal graph."""

    def test_direct_propagation_feed_rate(self):
        """Feed rate change propagates to all direct physics effects."""
        eng = _engine()
        effects = eng._forward_propagate('feed_rate', -20.0)
        assert 'cutting_force' in effects
        assert 'surface_roughness' in effects
        assert 'tool_life' in effects
        assert 'cycle_time' in effects

    def test_feed_rate_force_value(self):
        """Feed rate -20% -> cutting_force -16%."""
        eng = _engine()
        effects = eng._forward_propagate('feed_rate', -20.0)
        assert effects['cutting_force'] == pytest.approx(-16.0, abs=0.1)

    def test_feed_rate_surface_roughness_value(self):
        """Feed rate -20% -> surface_roughness -40%."""
        eng = _engine()
        effects = eng._forward_propagate('feed_rate', -20.0)
        assert effects['surface_roughness'] == pytest.approx(-40.0, abs=0.1)

    def test_feed_rate_tool_life_value(self):
        """Feed rate -20% -> tool_life +40%."""
        eng = _engine()
        effects = eng._forward_propagate('feed_rate', -20.0)
        assert effects['tool_life'] == pytest.approx(40.0, abs=0.1)

    def test_no_circular_loops(self):
        """Forward propagation should not revisit nodes."""
        link1 = _make_link(cause='A', effect='B', strength=0.8)
        link2 = _make_link(cause='B', effect='A', strength=0.5)
        eng = _engine([link1, link2])
        # Should terminate without infinite loop
        effects = eng._forward_propagate('A', 10.0)
        assert 'B' in effects
        assert 'A' not in effects  # should not loop back

    def test_multi_hop_propagation(self):
        """Changes propagate through intermediate nodes."""
        link1 = _make_link(cause='X', effect='Y', strength=0.8)
        link2 = _make_link(cause='Y', effect='Z', strength=0.5)
        eng = _engine([link1, link2])
        effects = eng._forward_propagate('X', 10.0)
        assert 'Y' in effects
        assert 'Z' in effects
        # Y should be 10 * 0.8 = 8.0
        assert effects['Y'] == pytest.approx(8.0, abs=0.1)
        # Z should be 8.0 * 0.5 = 4.0
        assert effects['Z'] == pytest.approx(4.0, abs=0.1)

    def test_unknown_intervention_empty(self):
        """Unknown intervention source returns empty dict."""
        eng = _engine()
        effects = eng._forward_propagate('nonexistent_param', 10.0)
        assert effects == {}


class TestFindAllCausalPaths:
    """Tests for find_all_causal_paths."""

    def test_direct_path(self):
        """Direct link found as single path."""
        link = _make_link(cause='A', effect='B', strength=0.9)
        eng = _engine([link])
        paths = eng.find_all_causal_paths('A', 'B')
        assert len(paths) == 1
        assert paths[0].nodes == ['A', 'B']

    def test_indirect_path(self):
        """Two-hop path found: A -> C -> B."""
        link1 = _make_link(cause='A', effect='C', strength=0.8)
        link2 = _make_link(cause='C', effect='B', strength=0.6)
        eng = _engine([link1, link2])
        paths = eng.find_all_causal_paths('A', 'B')
        assert len(paths) == 1
        assert paths[0].nodes == ['A', 'C', 'B']

    def test_both_direct_and_indirect(self):
        """Both direct A->B and indirect A->C->B found."""
        link_direct = _make_link(cause='A', effect='B', strength=0.9)
        link1 = _make_link(cause='A', effect='C', strength=0.7)
        link2 = _make_link(cause='C', effect='B', strength=0.5)
        eng = _engine([link_direct, link1, link2])
        paths = eng.find_all_causal_paths('A', 'B')
        assert len(paths) == 2

    def test_no_path_returns_empty(self):
        """No connection returns empty list."""
        link = _make_link(cause='A', effect='B', strength=0.9)
        eng = _engine([link])
        paths = eng.find_all_causal_paths('B', 'A')  # wrong direction
        assert paths == []

    def test_max_depth_limits_search(self):
        """Paths longer than max_depth are not found."""
        links = []
        # Chain: N0 -> N1 -> N2 -> N3 -> N4 -> N5 -> N6
        for i in range(6):
            links.append(_make_link(cause=f'N{i}', effect=f'N{i+1}', strength=0.9))
        eng = _engine(links)
        # max_depth=3 should NOT find path of length 7
        paths = eng.find_all_causal_paths('N0', 'N6', max_depth=3)
        assert len(paths) == 0
        # max_depth=6 should find it
        paths = eng.find_all_causal_paths('N0', 'N6', max_depth=6)
        assert len(paths) == 1

    def test_path_strength_is_product(self):
        """Path strength is product of link strengths."""
        now = time.time()
        link1 = _make_link(cause='A', effect='B', strength=0.8, last_evidence_time=now)
        link2 = _make_link(cause='B', effect='C', strength=0.5, last_evidence_time=now)
        eng = _engine([link1, link2])
        paths = eng.find_all_causal_paths('A', 'C')
        assert len(paths) == 1
        # Strength is product of effective strengths
        assert paths[0].strength == pytest.approx(0.8 * 0.5, abs=0.01)


class TestSimulateIntervention:
    """Tests for the full simulate_intervention workflow."""

    def test_feed_rate_reduction_results(self):
        """Simulating feed rate reduction produces multiple results."""
        eng = _engine()
        results = eng.simulate_intervention('feed_rate', -20.0)
        assert len(results) > 0
        effect_names = set()
        for r in results:
            effect_names.update(r.affected_effects.keys())
        assert 'cutting_force' in effect_names
        assert 'surface_roughness' in effect_names

    def test_intervention_result_fields(self):
        """InterventionResult has correct field values."""
        eng = _engine()
        results = eng.simulate_intervention('feed_rate', -20.0)
        force_result = None
        for r in results:
            if 'cutting_force' in r.affected_effects:
                force_result = r
                break
        assert force_result is not None
        assert force_result.intervention == 'feed_rate'
        assert force_result.intervention_value == -20.0
        assert force_result.affected_effects['cutting_force'] == pytest.approx(-16.0, abs=0.1)
        assert 0.0 <= force_result.confidence <= 1.0
        assert len(force_result.reasoning) > 0

    def test_side_effects_listed(self):
        """Each result lists side effects on other variables."""
        eng = _engine()
        results = eng.simulate_intervention('feed_rate', -20.0)
        # At least one result should have side effects (feed_rate affects multiple things)
        has_side_effects = any(len(r.side_effects) > 0 for r in results)
        assert has_side_effects

    def test_unknown_intervention_empty_results(self):
        """Unknown intervention returns empty list."""
        eng = _engine()
        results = eng.simulate_intervention('nonexistent_param', -10.0)
        assert results == []

    def test_spindle_speed_temperature(self):
        """Spindle speed increase -> temperature increase."""
        eng = _engine()
        results = eng.simulate_intervention('spindle_speed', 30.0)
        temp_result = None
        for r in results:
            if 'temperature' in r.affected_effects:
                temp_result = r
                break
        assert temp_result is not None
        assert temp_result.affected_effects['temperature'] == pytest.approx(15.0, abs=0.1)

    def test_depth_of_cut_force(self):
        """Depth of cut +50% -> force +50% (linear)."""
        eng = _engine()
        results = eng.simulate_intervention('depth_of_cut', 50.0)
        force_result = None
        for r in results:
            if 'cutting_force' in r.affected_effects:
                force_result = r
                break
        assert force_result is not None
        assert force_result.affected_effects['cutting_force'] == pytest.approx(50.0, abs=0.1)


class TestGetInterventionPoints:
    """Tests for get_intervention_points."""

    def test_cutting_force_interventions(self):
        """Multiple interventions affect cutting_force."""
        eng = _engine()
        points = eng.get_intervention_points('cutting_force')
        names = [p[0] for p in points]
        assert 'feed_rate' in names
        assert 'depth_of_cut' in names
        assert 'tool_wear' in names

    def test_sorted_by_impact(self):
        """Results sorted by impact magnitude descending."""
        eng = _engine()
        points = eng.get_intervention_points('cutting_force')
        impacts = [p[1] for p in points]
        assert impacts == sorted(impacts, reverse=True)

    def test_temperature_interventions(self):
        """Spindle speed and coolant flow affect temperature."""
        eng = _engine()
        points = eng.get_intervention_points('temperature')
        names = [p[0] for p in points]
        assert 'spindle_speed' in names
        assert 'coolant_flow' in names

    def test_unknown_target_returns_empty(self):
        """Unknown target effect returns empty list."""
        eng = _engine()
        points = eng.get_intervention_points('nonexistent_effect')
        assert points == []


class TestConfidenceDecay:
    """Tests for confidence behavior with path length and temporal decay."""

    def test_confidence_decreases_with_path_length(self):
        """Longer paths should have lower confidence."""
        now = time.time()
        # Short path: A -> B
        link_short = _make_link(cause='A', effect='B', strength=0.9, last_evidence_time=now)
        # Long path: A -> C -> D -> B
        link1 = _make_link(cause='A', effect='C', strength=0.9, last_evidence_time=now)
        link2 = _make_link(cause='C', effect='D', strength=0.9, last_evidence_time=now)
        link3 = _make_link(cause='D', effect='B', strength=0.9, last_evidence_time=now)

        eng_short = _engine([link_short])
        eng_long = _engine([link1, link2, link3])

        paths_short = eng_short.find_all_causal_paths('A', 'B')
        paths_long = eng_long.find_all_causal_paths('A', 'B')

        assert len(paths_short) == 1
        assert len(paths_long) == 1
        # Longer path has lower combined strength
        assert paths_long[0].strength < paths_short[0].strength

    def test_temporal_decay_affects_path_strength(self):
        """Old evidence produces weaker path strength."""
        now = time.time()
        link_fresh = _make_link(cause='A', effect='B', strength=0.9,
                                last_evidence_time=now)
        link_old = _make_link(cause='A', effect='B', strength=0.9,
                              last_evidence_time=now - 3600.0)  # 1 half-life

        eng_fresh = _engine([link_fresh])
        eng_old = _engine([link_old])

        paths_fresh = eng_fresh.find_all_causal_paths('A', 'B')
        paths_old = eng_old.find_all_causal_paths('A', 'B')

        assert paths_fresh[0].strength > paths_old[0].strength


class TestDataclasses:
    """Tests for InterventionResult and CausalPath dataclasses."""

    def test_intervention_result_creation(self):
        r = InterventionResult(
            intervention='feed_rate',
            intervention_value=-20.0,
            affected_effects={'cutting_force': -16.0},
            confidence=0.85,
            reasoning='test',
            side_effects=['cycle_time will increase by 20.0%'],
        )
        assert r.intervention == 'feed_rate'
        assert r.intervention_value == -20.0
        assert r.affected_effects == {'cutting_force': -16.0}
        assert r.confidence == 0.85
        assert len(r.side_effects) == 1

    def test_causal_path_creation(self):
        p = CausalPath(
            nodes=['A', 'B', 'C'],
            strength=0.72,
            decay_factor=0.95,
        )
        assert p.nodes == ['A', 'B', 'C']
        assert p.strength == 0.72
        assert p.decay_factor == 0.95
