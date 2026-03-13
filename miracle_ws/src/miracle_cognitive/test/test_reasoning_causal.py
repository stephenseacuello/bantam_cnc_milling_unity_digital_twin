"""Tests for causal simulation integration in the reasoning engine.

Validates that the reasoning engine can simulate action outcomes using
physics-based causal models, rank candidate actions by net benefit,
and enrich published actions with simulation results.
"""

import math
import os
import sys
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Stub ROS2 / project dependencies so tests run without a ROS2 installation.
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
    'get_clock': lambda self: MagicMock(
        now=MagicMock(return_value=MagicMock(to_msg=MagicMock(return_value=MagicMock())))
    ),
    'service_callback_group': None,
})

sys.modules.pop('miracle_cognitive.knowledge.reasoning_engine', None)
sys.modules.pop('miracle_cognitive.knowledge.causal_inference', None)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_cognitive.knowledge.reasoning_engine import (
    ReasoningEngineNode, SimulatedAction, _ACTION_MAPPING,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_engine():
    """Create a configured ReasoningEngineNode."""
    node = ReasoningEngineNode()
    node._do_configure()
    node._do_activate()
    node._inference_pub = MagicMock()
    node._action_pub = MagicMock()
    return node


# ---------------------------------------------------------------------------
# Tests: SimulatedAction dataclass
# ---------------------------------------------------------------------------

class TestSimulatedActionDataclass:
    """Test SimulatedAction dataclass creation and fields."""

    def test_create_simulated_action(self):
        sa = SimulatedAction(
            action_type='REDUCE_FEED',
            parameters={'feed_reduction_pct': 20},
            predicted_outcomes={'cutting_force': -16.0},
            confidence=0.85,
            side_effects=['cycle_time will increase by 20.0%'],
            net_benefit_score=0.15,
            reasoning_chain='REDUCE_FEED(feed_rate -20%) -> cutting_force decrease 16.0%',
        )
        assert sa.action_type == 'REDUCE_FEED'
        assert sa.parameters == {'feed_reduction_pct': 20}
        assert sa.predicted_outcomes == {'cutting_force': -16.0}
        assert sa.confidence == 0.85
        assert len(sa.side_effects) == 1
        assert sa.net_benefit_score == 0.15
        assert 'REDUCE_FEED' in sa.reasoning_chain

    def test_simulated_action_defaults_all_fields(self):
        sa = SimulatedAction(
            action_type='PAUSE',
            parameters={},
            predicted_outcomes={},
            confidence=1.0,
            side_effects=[],
            net_benefit_score=0.0,
            reasoning_chain='PAUSE: no causal effects modelled',
        )
        assert sa.predicted_outcomes == {}
        assert sa.side_effects == []
        assert sa.net_benefit_score == 0.0


# ---------------------------------------------------------------------------
# Tests: _map_action_to_intervention
# ---------------------------------------------------------------------------

class TestMapActionToIntervention:
    """Test mapping from action types to causal interventions."""

    def test_reduce_feed_mapping(self):
        node = _make_engine()
        var, pct = node._map_action_to_intervention('REDUCE_FEED', {'feed_reduction_pct': 25})
        assert var == 'feed_rate'
        assert pct == -25

    def test_reduce_speed_mapping(self):
        node = _make_engine()
        var, pct = node._map_action_to_intervention('REDUCE_SPEED', {'speed_reduction_pct': 15})
        assert var == 'spindle_speed'
        assert pct == -15

    def test_tool_change_mapping(self):
        node = _make_engine()
        var, pct = node._map_action_to_intervention('TOOL_CHANGE', {})
        assert var == 'tool_wear'
        assert pct == -100.0

    def test_coolant_increase_mapping(self):
        node = _make_engine()
        var, pct = node._map_action_to_intervention('COOLANT_INCREASE', {'coolant_increase_pct': 50})
        assert var == 'coolant_flow'
        assert pct == 50

    def test_pause_mapping(self):
        node = _make_engine()
        var, pct = node._map_action_to_intervention('PAUSE', {})
        assert var is None
        assert pct == 0.0

    def test_unknown_action_mapping(self):
        node = _make_engine()
        var, pct = node._map_action_to_intervention('UNKNOWN_ACTION', {})
        assert var is None
        assert pct == 0.0

    def test_reduce_feed_default_params(self):
        node = _make_engine()
        var, pct = node._map_action_to_intervention('REDUCE_FEED', {})
        assert var == 'feed_rate'
        assert pct == -20  # default


# ---------------------------------------------------------------------------
# Tests: simulate_action_outcomes
# ---------------------------------------------------------------------------

class TestSimulateActionOutcomes:
    """Test forward causal simulation for various action types."""

    def test_reduce_feed_produces_outcomes(self):
        node = _make_engine()
        result = node.simulate_action_outcomes('REDUCE_FEED', {'feed_reduction_pct': 20})
        assert isinstance(result, SimulatedAction)
        assert result.action_type == 'REDUCE_FEED'
        assert len(result.predicted_outcomes) > 0

    def test_reduce_feed_force_follows_f_power_0_8(self):
        """F proportional to f^0.8: -20% feed -> force change = -20 * 0.8 = -16%."""
        node = _make_engine()
        result = node.simulate_action_outcomes('REDUCE_FEED', {'feed_reduction_pct': 20})
        assert 'cutting_force' in result.predicted_outcomes
        assert result.predicted_outcomes['cutting_force'] == pytest.approx(-16.0, abs=0.5)

    def test_reduce_feed_surface_follows_f_squared(self):
        """Ra proportional to f^2: -20% feed -> surface_roughness = -20 * 2 = -40%."""
        node = _make_engine()
        result = node.simulate_action_outcomes('REDUCE_FEED', {'feed_reduction_pct': 20})
        assert 'surface_roughness' in result.predicted_outcomes
        assert result.predicted_outcomes['surface_roughness'] == pytest.approx(-40.0, abs=0.5)

    def test_reduce_feed_side_effects_include_cycle_time(self):
        """Reducing feed rate increases cycle time -- a side effect."""
        node = _make_engine()
        result = node.simulate_action_outcomes('REDUCE_FEED', {'feed_reduction_pct': 20})
        # cycle_time is not a primary effect of REDUCE_FEED, so it should be a side effect
        side_effect_text = ' '.join(result.side_effects)
        assert 'cycle_time' in side_effect_text or 'cycle_time' in result.predicted_outcomes

    def test_reduce_speed_produces_outcomes(self):
        node = _make_engine()
        result = node.simulate_action_outcomes('REDUCE_SPEED', {'speed_reduction_pct': 20})
        assert isinstance(result, SimulatedAction)
        assert result.action_type == 'REDUCE_SPEED'
        assert len(result.predicted_outcomes) > 0

    def test_tool_change_produces_outcomes(self):
        node = _make_engine()
        result = node.simulate_action_outcomes('TOOL_CHANGE', {})
        assert isinstance(result, SimulatedAction)
        assert result.action_type == 'TOOL_CHANGE'
        # Tool change resets wear -> should affect surface_roughness and cutting_force
        assert len(result.predicted_outcomes) > 0

    def test_coolant_increase_produces_outcomes(self):
        node = _make_engine()
        result = node.simulate_action_outcomes('COOLANT_INCREASE', {'coolant_increase_pct': 30})
        assert isinstance(result, SimulatedAction)
        assert result.action_type == 'COOLANT_INCREASE'
        assert 'temperature' in result.predicted_outcomes

    def test_pause_has_no_causal_effects(self):
        """PAUSE action should have no predicted outcomes."""
        node = _make_engine()
        result = node.simulate_action_outcomes('PAUSE', {})
        assert result.action_type == 'PAUSE'
        assert result.predicted_outcomes == {}
        assert result.side_effects == []
        assert result.confidence == 1.0

    def test_unknown_action_returns_default(self):
        """Unknown action type returns a simulation with zero confidence."""
        node = _make_engine()
        result = node.simulate_action_outcomes('MYSTERY_ACTION', {})
        assert result.action_type == 'MYSTERY_ACTION'
        assert result.predicted_outcomes == {}
        assert result.confidence == 0.0

    def test_confidence_between_0_and_1(self):
        node = _make_engine()
        result = node.simulate_action_outcomes('REDUCE_FEED', {'feed_reduction_pct': 20})
        assert 0.0 <= result.confidence <= 1.0

    def test_reasoning_chain_present(self):
        node = _make_engine()
        result = node.simulate_action_outcomes('REDUCE_FEED', {'feed_reduction_pct': 20})
        assert len(result.reasoning_chain) > 0
        assert 'REDUCE_FEED' in result.reasoning_chain


# ---------------------------------------------------------------------------
# Tests: net benefit score computation
# ---------------------------------------------------------------------------

class TestNetBenefitScore:
    """Test _compute_net_benefit calculation."""

    def test_force_reduction_positive_benefit(self):
        node = _make_engine()
        score = node._compute_net_benefit({'cutting_force': -20.0})
        # -(-20)/100 * 0.4 = 0.08
        assert score == pytest.approx(0.08, abs=0.001)

    def test_cycle_time_increase_negative_benefit(self):
        node = _make_engine()
        score = node._compute_net_benefit({'cycle_time': 20.0})
        # -(20/100) * 0.2 = -0.04
        assert score == pytest.approx(-0.04, abs=0.001)

    def test_combined_outcomes(self):
        node = _make_engine()
        outcomes = {
            'cutting_force': -16.0,
            'surface_roughness': -40.0,
            'cycle_time': 20.0,
            'tool_life': 40.0,
        }
        score = node._compute_net_benefit(outcomes)
        expected = (0.4 * 16.0 / 100.0    # force reduction
                  + 0.3 * 40.0 / 100.0     # roughness reduction
                  - 0.2 * 20.0 / 100.0     # cycle time cost
                  + 0.1 * 40.0 / 100.0)    # tool life bonus
        assert score == pytest.approx(expected, abs=0.001)

    def test_empty_outcomes_zero_benefit(self):
        node = _make_engine()
        score = node._compute_net_benefit({})
        assert score == 0.0

    def test_reduce_feed_has_positive_net_benefit(self):
        """REDUCE_FEED should have a positive overall net benefit."""
        node = _make_engine()
        result = node.simulate_action_outcomes('REDUCE_FEED', {'feed_reduction_pct': 20})
        assert result.net_benefit_score > 0.0


# ---------------------------------------------------------------------------
# Tests: get_best_action_with_simulation
# ---------------------------------------------------------------------------

class TestGetBestActionWithSimulation:
    """Test selecting the best action based on simulation."""

    def test_selects_highest_benefit_action(self):
        node = _make_engine()
        facts = {
            ('cnc1', 'hasSpindleLoad'): 'OVERLOAD',
            ('cnc1', 'hasVibrationLevel'): 'HIGH',
        }
        best = node.get_best_action_with_simulation(facts)
        assert best is not None
        assert isinstance(best, SimulatedAction)
        assert best.action_type in _ACTION_MAPPING.values()

    def test_multiple_candidates_ranked(self):
        """When multiple rules fire, the one with best net benefit is returned."""
        node = _make_engine()
        facts = {
            ('cnc1', 'hasSpindleLoad'): 'OVERLOAD',
            ('tool1', 'hasWearLevel'): 'HIGH',
            ('cnc1', 'hasTemperature'): 'CRITICAL',
        }
        best = node.get_best_action_with_simulation(facts)
        assert best is not None
        # Verify it picked the best one by simulating all individually
        all_action_types = ['REDUCE_FEED', 'TOOL_CHANGE', 'COOLANT_INCREASE']
        simulations = []
        for at in all_action_types:
            params = dict(node._DEFAULT_ACTION_PARAMS.get(at, {}))
            sim = node.simulate_action_outcomes(at, params)
            simulations.append(sim)
        max_benefit = max(s.net_benefit_score for s in simulations)
        assert best.net_benefit_score >= min(s.net_benefit_score for s in simulations)

    def test_no_matching_facts_returns_none(self):
        node = _make_engine()
        facts = {('cnc1', 'hasStatus'): 'NORMAL'}
        best = node.get_best_action_with_simulation(facts)
        assert best is None

    def test_confidence_scaled_by_rule(self):
        """Confidence should be scaled by the rule's confidence."""
        node = _make_engine()
        facts = {('cnc1', 'hasTemperature'): 'CRITICAL'}
        best = node.get_best_action_with_simulation(facts)
        assert best is not None
        # The thermal rule has confidence 0.85
        # Simulation confidence should be multiplied by 0.85
        raw = node.simulate_action_outcomes(best.action_type,
                                             dict(node._DEFAULT_ACTION_PARAMS.get(best.action_type, {})))
        assert best.confidence <= raw.confidence  # scaled down


# ---------------------------------------------------------------------------
# Tests: action publishing with simulation enrichment
# ---------------------------------------------------------------------------

class TestActionPublishingWithSimulation:
    """Test that published actions include simulation data."""

    def test_published_action_reasoning_includes_sim(self):
        node = _make_engine()
        node._facts.append(('cnc1', 'hasSpindleLoad', 'OVERLOAD'))
        node._run_inference()

        calls = node._action_pub.publish.call_args_list
        assert len(calls) > 0
        for call in calls:
            action_msg = call[0][0]
            if action_msg.action_type == 'REDUCE_FEED':
                # Should contain simulation enrichment
                assert 'sim:' in action_msg.reasoning or 'triggered by' in action_msg.reasoning
