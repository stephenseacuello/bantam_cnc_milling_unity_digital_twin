"""Tests for live-data integration in ActionRanker.

Covers MachineContext, causal inference integration, infeasible action
filtering, and history-aware ranking.
"""

import os
import sys
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Mock ROS2 / miracle_core dependencies before importing the module under test
# ---------------------------------------------------------------------------
for mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.callback_groups",
    "rclpy.qos",
    "rclpy.lifecycle",
    "miracle_core.lifecycle_node_base",
    "miracle_core.qos_profiles",
    "miracle_core.heartbeat_mixin",
    "miracle_msgs",
    "miracle_msgs.msg",
]:
    sys.modules.setdefault(mod, MagicMock())

sys.modules["miracle_core.lifecycle_node_base"].MiracleLifecycleNode = type(
    "FakeNode",
    (),
    {"__init__": lambda self, *a, **kw: None},
)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from miracle_cognitive.interface.action_ranker import (
    ActionCandidate,
    ActionRanker,
    MachineContext,
    RankedActions,
)


# ===========================================================================
# Fixtures
# ===========================================================================


@pytest.fixture
def ranker():
    return ActionRanker()


@pytest.fixture
def default_context():
    """A context with all defaults — should have no effect on ranking."""
    return MachineContext()


@pytest.fixture
def low_rul_context():
    return MachineContext(
        machine_id="cnc-1",
        tool_rul_minutes=3.0,
        tool_wear_vb_mm=0.28,
    )


@pytest.fixture
def reduced_feed_context():
    return MachineContext(
        machine_id="cnc-1",
        current_feed_pct=70.0,
    )


@pytest.fixture
def max_coolant_context():
    return MachineContext(
        machine_id="cnc-1",
        coolant_type="cryogenic",
    )


@pytest.fixture
def near_end_context():
    return MachineContext(
        machine_id="cnc-1",
        program_progress_pct=95.0,
    )


@pytest.fixture
def high_spindle_context():
    return MachineContext(
        machine_id="cnc-1",
        spindle_load_pct=90.0,
    )


# ===========================================================================
# MachineContext dataclass
# ===========================================================================


class TestMachineContext:
    def test_creation_with_all_fields(self):
        ctx = MachineContext(
            machine_id="cnc-42",
            current_feed_pct=85.0,
            current_speed_pct=90.0,
            tool_rul_minutes=12.5,
            tool_wear_vb_mm=0.15,
            spindle_load_pct=65.0,
            temperature_c=180.0,
            coolant_type="high_pressure",
            program_progress_pct=55.0,
            recent_anomaly_count=3,
            active_overrides=["rapid_override"],
        )
        assert ctx.machine_id == "cnc-42"
        assert ctx.current_feed_pct == 85.0
        assert ctx.coolant_type == "high_pressure"
        assert ctx.recent_anomaly_count == 3
        assert ctx.active_overrides == ["rapid_override"]

    def test_defaults(self):
        ctx = MachineContext()
        assert ctx.machine_id == ""
        assert ctx.current_feed_pct == 100.0
        assert ctx.tool_rul_minutes == 30.0
        assert ctx.coolant_type == "flood"
        assert ctx.active_overrides == []


# ===========================================================================
# Ranking with context vs without
# ===========================================================================


class TestContextVsNoContext:
    def test_with_context_returns_ranked_actions(self, ranker, default_context):
        result = ranker.rank_actions(
            "FORCE_WARNING", 0.5, machine_context=default_context
        )
        assert isinstance(result, RankedActions)
        assert len(result.ranked_actions) > 0

    def test_without_context_still_works(self, ranker):
        result = ranker.rank_actions("FORCE_WARNING", 0.5)
        assert isinstance(result, RankedActions)
        assert len(result.ranked_actions) > 0

    def test_default_context_preserves_candidate_count(self, ranker, default_context):
        without = ranker.rank_actions("FORCE_WARNING", 0.5)
        with_ctx = ranker.rank_actions(
            "FORCE_WARNING", 0.5, machine_context=default_context
        )
        # Default context should not filter any actions
        assert len(with_ctx.ranked_actions) == len(without.ranked_actions)


# ===========================================================================
# Low RUL boosts TOOL_CHANGE
# ===========================================================================


class TestLowRulBoost:
    def test_low_rul_boosts_tool_change_score(self, ranker, low_rul_context):
        without = ranker.rank_actions("TOOL_WEAR", 0.6)
        with_ctx = ranker.rank_actions(
            "TOOL_WEAR", 0.6, machine_context=low_rul_context
        )

        def _tool_change_score(result):
            for a in result.ranked_actions:
                if a.action_type == "TOOL_CHANGE":
                    return a.score
            return 0.0

        assert _tool_change_score(with_ctx) > _tool_change_score(without)

    def test_low_rul_adds_urgency_reasoning(self, ranker, low_rul_context):
        result = ranker.rank_actions(
            "TOOL_WEAR", 0.6, machine_context=low_rul_context
        )
        tool_change = [a for a in result.ranked_actions if a.action_type == "TOOL_CHANGE"]
        assert len(tool_change) == 1
        assert "urgency" in tool_change[0].reasoning.lower()


# ===========================================================================
# Already-reduced feed diminishes further reduction benefit
# ===========================================================================


class TestReducedFeedDiminishedBenefit:
    def test_already_reduced_feed_lowers_force_reduction(self, ranker, reduced_feed_context):
        without = ranker.rank_actions("FORCE_WARNING", 0.5)
        with_ctx = ranker.rank_actions(
            "FORCE_WARNING", 0.5, machine_context=reduced_feed_context
        )

        def _reduce_feed_force_red(result):
            for a in result.ranked_actions:
                if a.action_type == "REDUCE_FEED":
                    return a.predicted_force_reduction_pct
            return 0.0

        # With feed already at 70%, the predicted benefit should be smaller
        assert _reduce_feed_force_red(with_ctx) < _reduce_feed_force_red(without)


# ===========================================================================
# Coolant at max removes COOLANT_INCREASE
# ===========================================================================


class TestCoolantAtMax:
    def test_coolant_at_max_removes_increase_action(self, ranker, max_coolant_context):
        result = ranker.rank_actions(
            "THERMAL_WARNING", 0.5, machine_context=max_coolant_context
        )
        types = [a.action_type for a in result.ranked_actions]
        assert "COOLANT_INCREASE" not in types

    def test_coolant_not_at_max_keeps_increase_action(self, ranker):
        ctx = MachineContext(coolant_type="flood")
        result = ranker.rank_actions(
            "THERMAL_WARNING", 0.5, machine_context=ctx
        )
        types = [a.action_type for a in result.ranked_actions]
        assert "COOLANT_INCREASE" in types


# ===========================================================================
# Near program end favours conservative actions
# ===========================================================================


class TestNearProgramEnd:
    def test_near_end_favours_conservative(self, ranker, near_end_context):
        result = ranker.rank_actions(
            "TOOL_WEAR", 0.5, machine_context=near_end_context
        )
        # ACCEPT_MONITOR is conservative and should be boosted
        accept = [a for a in result.ranked_actions if a.action_type == "ACCEPT_MONITOR"]
        assert len(accept) == 1
        assert "conservative" in accept[0].reasoning.lower()

    def test_near_end_penalises_tool_change(self, ranker, near_end_context):
        without = ranker.rank_actions("TOOL_WEAR", 0.5)
        with_ctx = ranker.rank_actions(
            "TOOL_WEAR", 0.5, machine_context=near_end_context
        )

        def _tool_change_risk_red(result):
            for a in result.ranked_actions:
                if a.action_type == "TOOL_CHANGE":
                    return a.risk_reduction
            return 0.0

        assert _tool_change_risk_red(with_ctx) < _tool_change_risk_red(without)


# ===========================================================================
# Causal results override template estimates
# ===========================================================================


class TestCausalIntegration:
    def test_causal_overrides_force_reduction(self, ranker):
        causal = [
            {
                "action_type": "REDUCE_FEED",
                "predicted_force_reduction_pct": 16.0,
                "confidence": 0.95,
                "reasoning": "reducing feed 20% -> force drops 16%",
            }
        ]
        result = ranker.rank_actions(
            "FORCE_WARNING", 0.5, causal_interventions=causal
        )
        reduce_feed = [
            a for a in result.ranked_actions if a.action_type == "REDUCE_FEED"
        ]
        # At least one REDUCE_FEED should have the causal override
        matched = [a for a in reduce_feed if a.predicted_force_reduction_pct == 16.0]
        assert len(matched) >= 1

    def test_causal_adjusts_confidence(self, ranker):
        causal = [
            {
                "action_type": "TOOL_CHANGE",
                "confidence": 0.99,
                "reasoning": "physics model high confidence",
            }
        ]
        without = ranker.rank_actions("FORCE_WARNING", 0.5)
        with_causal = ranker.rank_actions(
            "FORCE_WARNING", 0.5, causal_interventions=causal
        )

        def _tc_confidence(result):
            for a in result.ranked_actions:
                if a.action_type == "TOOL_CHANGE":
                    return a.confidence
            return 0.0

        # Causal confidence of 0.99 blended in should raise overall confidence
        assert _tc_confidence(with_causal) > _tc_confidence(without)

    def test_causal_adds_reasoning(self, ranker):
        causal = [
            {
                "action_type": "REDUCE_FEED",
                "reasoning": "FEM simulation confirms force drop",
            }
        ]
        result = ranker.rank_actions(
            "FORCE_WARNING", 0.5, causal_interventions=causal
        )
        reduce_feed = [
            a for a in result.ranked_actions if a.action_type == "REDUCE_FEED"
        ]
        assert any("Causal" in a.reasoning for a in reduce_feed)


# ===========================================================================
# Infeasible action filtering
# ===========================================================================


class TestInfeasibleFiltering:
    def test_feed_below_minimum_filtered(self, ranker):
        ctx = MachineContext(current_feed_pct=15.0)  # already very low
        result = ranker.rank_actions(
            "FORCE_WARNING", 0.5, machine_context=ctx
        )
        # Any REDUCE_FEED action at 70% of 15% = 10.5% < 20% should be removed
        for a in result.ranked_actions:
            if a.action_type == "REDUCE_FEED":
                effective = ctx.current_feed_pct * a.feed_override_pct / 100.0
                assert effective >= 20.0

    def test_pause_filtered_in_critical_section(self, ranker):
        ctx = MachineContext(active_overrides=["critical_section"])
        result = ranker.rank_actions(
            "THERMAL_WARNING", 0.5, machine_context=ctx
        )
        types = [a.action_type for a in result.ranked_actions]
        assert "PAUSE" not in types

    def test_tool_change_filtered_no_replacement(self, ranker):
        ctx = MachineContext(active_overrides=["no_replacement_tool"])
        result = ranker.rank_actions(
            "TOOL_WEAR", 0.5, machine_context=ctx
        )
        types = [a.action_type for a in result.ranked_actions]
        assert "TOOL_CHANGE" not in types

    def test_minimum_feed_override_enforced(self, ranker):
        """Even at normal feed, aggressive reductions that would
        go below 20% effective should be filtered."""
        ctx = MachineContext(current_feed_pct=22.0)
        result = ranker.rank_actions(
            "TOOL_WEAR", 0.5, machine_context=ctx
        )
        for a in result.ranked_actions:
            if a.action_type == "REDUCE_FEED":
                effective = ctx.current_feed_pct * a.feed_override_pct / 100.0
                assert effective >= 20.0, (
                    f"Feed action {a.description} has effective feed "
                    f"{effective:.1f}% < 20% minimum"
                )


# ===========================================================================
# History-aware ranking
# ===========================================================================


class TestHistoryAwareRanking:
    def test_failed_action_penalised(self, ranker):
        past = [
            {"action_type": "REDUCE_FEED", "outcome": "failure", "effectiveness": 0.1}
        ]
        without = ranker.rank_actions("FORCE_WARNING", 0.5)
        with_hist = ranker.rank_with_history("FORCE_WARNING", 0.5, past_actions=past)

        def _reduce_feed_score(result):
            for a in result.ranked_actions:
                if a.action_type == "REDUCE_FEED":
                    return a.score
            return 0.0

        assert _reduce_feed_score(with_hist) < _reduce_feed_score(without)

    def test_successful_action_boosted(self, ranker):
        past = [
            {"action_type": "TOOL_CHANGE", "outcome": "success", "effectiveness": 0.9}
        ]
        without = ranker.rank_actions("TOOL_WEAR", 0.6)
        with_hist = ranker.rank_with_history("TOOL_WEAR", 0.6, past_actions=past)

        def _tool_change_score(result):
            for a in result.ranked_actions:
                if a.action_type == "TOOL_CHANGE":
                    return a.score
            return 0.0

        assert _tool_change_score(with_hist) > _tool_change_score(without)

    def test_history_adds_reasoning(self, ranker):
        past = [
            {"action_type": "REDUCE_FEED", "outcome": "failure", "effectiveness": 0.2}
        ]
        result = ranker.rank_with_history("FORCE_WARNING", 0.5, past_actions=past)
        reduce_feed = [a for a in result.ranked_actions if a.action_type == "REDUCE_FEED"]
        assert any("Penalised" in a.reasoning for a in reduce_feed)

    def test_no_history_same_as_rank_actions(self, ranker):
        r1 = ranker.rank_actions("FORCE_WARNING", 0.5)
        r2 = ranker.rank_with_history("FORCE_WARNING", 0.5, past_actions=None)
        scores1 = [a.score for a in r1.ranked_actions]
        scores2 = [a.score for a in r2.ranked_actions]
        assert scores1 == scores2


# ===========================================================================
# Combined context + causal + history
# ===========================================================================


class TestCombinedLiveData:
    def test_combined_context_causal_history(self, ranker, low_rul_context):
        causal = [
            {
                "action_type": "TOOL_CHANGE",
                "predicted_force_reduction_pct": 20.0,
                "confidence": 0.95,
                "reasoning": "physics model predicts 20% force drop",
            }
        ]
        past = [
            {"action_type": "REDUCE_FEED", "outcome": "failure", "effectiveness": 0.1}
        ]
        result = ranker.rank_with_history(
            "TOOL_WEAR",
            0.7,
            machine_context=low_rul_context,
            causal_interventions=causal,
            past_actions=past,
        )
        # TOOL_CHANGE should be top (low RUL + causal boost + feed failed)
        assert result.ranked_actions[0].action_type == "TOOL_CHANGE"


# ===========================================================================
# Score directionality
# ===========================================================================


class TestScoreDirectionality:
    def test_higher_spindle_load_increases_force_reduction(self, ranker, high_spindle_context):
        without = ranker.rank_actions("FORCE_WARNING", 0.5)
        with_ctx = ranker.rank_actions(
            "FORCE_WARNING", 0.5, machine_context=high_spindle_context
        )

        def _max_force_red(result):
            return max(
                (a.predicted_force_reduction_pct for a in result.ranked_actions),
                default=0.0,
            )

        # Higher spindle load should scale force reduction up
        assert _max_force_red(with_ctx) >= _max_force_red(without)

    def test_scores_remain_sorted_after_context(self, ranker, low_rul_context):
        result = ranker.rank_actions(
            "TOOL_WEAR", 0.6, machine_context=low_rul_context
        )
        scores = [a.score for a in result.ranked_actions]
        assert scores == sorted(scores, reverse=True)

    def test_scores_remain_sorted_after_history(self, ranker):
        past = [
            {"action_type": "REDUCE_FEED", "outcome": "failure", "effectiveness": 0.1}
        ]
        result = ranker.rank_with_history("FORCE_WARNING", 0.5, past_actions=past)
        scores = [a.score for a in result.ranked_actions]
        assert scores == sorted(scores, reverse=True)
