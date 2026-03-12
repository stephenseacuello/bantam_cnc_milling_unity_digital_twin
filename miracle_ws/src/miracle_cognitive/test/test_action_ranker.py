"""Tests for the ActionRanker decision-support module."""

import math
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

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..")
)

from miracle_cognitive.interface.action_ranker import (
    ActionCandidate,
    ActionRanker,
    RankedActions,
)


# ===========================================================================
# Fixtures
# ===========================================================================


@pytest.fixture
def ranker():
    return ActionRanker()


@pytest.fixture
def custom_ranker():
    """Ranker with cycle_time dominating."""
    return ActionRanker(
        weights={
            "risk_reduction": 0.05,
            "rul_extension": 0.05,
            "cycle_time": 0.70,
            "surface_quality": 0.10,
            "confidence": 0.10,
        }
    )


# ===========================================================================
# Action generation per anomaly type
# ===========================================================================


class TestActionGeneration:
    """Verify that each anomaly type produces the correct candidate set."""

    @pytest.mark.parametrize(
        "anomaly_type, expected_count",
        [
            ("FORCE_WARNING", 4),
            ("CHATTER_RISK", 4),
            ("THERMAL_WARNING", 4),
            ("TOOL_WEAR", 4),
            ("SURFACE_QUALITY", 4),
        ],
    )
    def test_candidate_count_per_anomaly(self, ranker, anomaly_type, expected_count):
        result = ranker.rank_actions(anomaly_type, severity=0.5)
        assert len(result.ranked_actions) == expected_count

    def test_force_warning_candidates(self, ranker):
        result = ranker.rank_actions("FORCE_WARNING", severity=0.6)
        types = [a.action_type for a in result.ranked_actions]
        assert "REDUCE_FEED" in types
        assert "TOOL_CHANGE" in types

    def test_chatter_risk_candidates(self, ranker):
        result = ranker.rank_actions("CHATTER_RISK", severity=0.5)
        types = [a.action_type for a in result.ranked_actions]
        assert "SHIFT_RPM" in types

    def test_thermal_warning_candidates(self, ranker):
        result = ranker.rank_actions("THERMAL_WARNING", severity=0.5)
        types = [a.action_type for a in result.ranked_actions]
        assert "COOLANT_INCREASE" in types
        assert "PAUSE" in types

    def test_tool_wear_candidates(self, ranker):
        result = ranker.rank_actions("TOOL_WEAR", severity=0.7)
        types = [a.action_type for a in result.ranked_actions]
        assert "TOOL_CHANGE" in types
        assert "ACCEPT_MONITOR" in types

    def test_surface_quality_candidates(self, ranker):
        result = ranker.rank_actions("SURFACE_QUALITY", severity=0.4)
        types = [a.action_type for a in result.ranked_actions]
        assert "ACCEPT_DEVIATION" in types
        assert "COOLANT_INCREASE" in types


# ===========================================================================
# Tool change always available
# ===========================================================================


class TestToolChangeAlwaysAvailable:
    @pytest.mark.parametrize(
        "anomaly_type",
        ["FORCE_WARNING", "CHATTER_RISK", "TOOL_WEAR", "SURFACE_QUALITY"],
    )
    def test_tool_change_present(self, ranker, anomaly_type):
        result = ranker.rank_actions(anomaly_type, severity=0.5)
        types = [a.action_type for a in result.ranked_actions]
        assert "TOOL_CHANGE" in types


# ===========================================================================
# Scoring & ranking order
# ===========================================================================


class TestScoringAndRanking:
    def test_ranked_descending_by_score(self, ranker):
        result = ranker.rank_actions("FORCE_WARNING", severity=0.6)
        scores = [a.score for a in result.ranked_actions]
        assert scores == sorted(scores, reverse=True)

    def test_recommended_index_is_zero(self, ranker):
        result = ranker.rank_actions("TOOL_WEAR", severity=0.5)
        assert result.recommended_index == 0

    def test_confidence_affects_score(self, ranker):
        """Actions with higher confidence should score higher, all else equal."""
        c1 = ActionCandidate(
            action_type="REDUCE_FEED",
            description="test",
            feed_override_pct=80,
            speed_override_pct=100,
            predicted_force_reduction_pct=16,
            predicted_rul_extension_min=10,
            predicted_surface_improvement_pct=20,
            cycle_time_impact_pct=10,
            risk_reduction=0.5,
            confidence=0.9,
            reasoning="high confidence",
        )
        c2 = ActionCandidate(
            action_type="REDUCE_FEED",
            description="test",
            feed_override_pct=80,
            speed_override_pct=100,
            predicted_force_reduction_pct=16,
            predicted_rul_extension_min=10,
            predicted_surface_improvement_pct=20,
            cycle_time_impact_pct=10,
            risk_reduction=0.5,
            confidence=0.3,
            reasoning="low confidence",
        )
        s1 = ranker._score_action(c1)
        s2 = ranker._score_action(c2)
        assert s1 > s2

    def test_custom_weights_change_ranking(self, custom_ranker):
        """When cycle_time weight dominates, zero-cost actions rank higher."""
        result = custom_ranker.rank_actions("THERMAL_WARNING", severity=0.5)
        top = result.ranked_actions[0]
        # Coolant increase has 0% cycle time impact
        assert top.cycle_time_impact_pct == 0.0


# ===========================================================================
# Force reduction estimation (F proportional to feed^0.8)
# ===========================================================================


class TestForceReduction:
    def test_10pct_feed_reduction(self, ranker):
        reduction = ranker._estimate_force_reduction("REDUCE_FEED", 100.0, 90.0)
        # (1 - (90/100)^0.8) * 100 ≈ 8.14%
        expected = (1.0 - (0.9 ** 0.8)) * 100.0
        assert reduction == pytest.approx(expected, abs=0.1)

    def test_20pct_feed_reduction(self, ranker):
        reduction = ranker._estimate_force_reduction("REDUCE_FEED", 100.0, 80.0)
        expected = (1.0 - (0.8 ** 0.8)) * 100.0
        assert reduction == pytest.approx(expected, abs=0.1)

    def test_no_change(self, ranker):
        reduction = ranker._estimate_force_reduction("REDUCE_FEED", 100.0, 100.0)
        assert reduction == pytest.approx(0.0, abs=0.01)

    def test_tool_change_fixed_reduction(self, ranker):
        reduction = ranker._estimate_force_reduction("TOOL_CHANGE", 100.0, 100.0)
        assert reduction == 15.0

    def test_pause_no_reduction(self, ranker):
        reduction = ranker._estimate_force_reduction("PAUSE", 100.0, 0.0)
        assert reduction == 0.0


# ===========================================================================
# RUL extension estimation
# ===========================================================================


class TestRulExtension:
    def test_feed_reduction_extends_rul(self, ranker):
        ext = ranker._estimate_rul_extension("REDUCE_FEED", 100.0, 80.0, 30.0)
        # life_ratio = (100/80)^2 = 1.5625 → ext = 30 * 0.5625 = 16.875
        expected = 30.0 * ((100.0 / 80.0) ** 2 - 1.0)
        assert ext == pytest.approx(expected, abs=0.1)

    def test_tool_change_fixed_extension(self, ranker):
        ext = ranker._estimate_rul_extension("TOOL_CHANGE", 100.0, 100.0, 5.0)
        assert ext == 30.0

    def test_no_feed_change_no_extension(self, ranker):
        ext = ranker._estimate_rul_extension("REDUCE_FEED", 100.0, 100.0, 30.0)
        assert ext == pytest.approx(0.0, abs=0.01)

    def test_accept_monitor_no_extension(self, ranker):
        ext = ranker._estimate_rul_extension("ACCEPT_MONITOR", 100.0, 100.0, 30.0)
        assert ext == 0.0


# ===========================================================================
# Cycle time impact
# ===========================================================================


class TestCycleTimeImpact:
    def test_feed_reduction_increases_cycle_time(self, ranker):
        # 20% feed reduction → (1/0.8 - 1)*100 = 25%
        impact = ranker._estimate_cycle_time_impact("REDUCE_FEED", -20.0)
        assert impact == pytest.approx(25.0, abs=0.1)

    def test_no_change_no_impact(self, ranker):
        impact = ranker._estimate_cycle_time_impact("REDUCE_FEED", 0.0)
        assert impact == 0.0

    def test_tool_change_fixed_impact(self, ranker):
        impact = ranker._estimate_cycle_time_impact("TOOL_CHANGE", 0.0)
        assert impact == 15.0

    def test_coolant_no_impact(self, ranker):
        impact = ranker._estimate_cycle_time_impact("COOLANT_INCREASE", 0.0)
        assert impact == 0.0


# ===========================================================================
# Do-nothing risk
# ===========================================================================


class TestDoNothingRisk:
    def test_risk_increases_with_severity(self, ranker):
        low = ranker.rank_actions("FORCE_WARNING", severity=0.2)
        high = ranker.rank_actions("FORCE_WARNING", severity=0.9)
        assert high.do_nothing_risk > low.do_nothing_risk

    def test_risk_bounded_0_1(self, ranker):
        result = ranker.rank_actions("TOOL_WEAR", severity=1.0)
        assert 0.0 <= result.do_nothing_risk <= 1.0

    def test_tool_wear_higher_base_risk(self, ranker):
        tool = ranker.rank_actions("TOOL_WEAR", severity=0.5)
        surface = ranker.rank_actions("SURFACE_QUALITY", severity=0.5)
        assert tool.do_nothing_risk > surface.do_nothing_risk


# ===========================================================================
# Severity edge cases
# ===========================================================================


class TestSeverityEdgeCases:
    def test_severity_zero_low_impact_actions(self, ranker):
        result = ranker.rank_actions("FORCE_WARNING", severity=0.0)
        # At severity 0, do_nothing_risk should be low
        assert result.do_nothing_risk < 0.3
        # All candidates still generated
        assert len(result.ranked_actions) == 4

    def test_severity_one_aggressive_actions_ranked_higher(self, ranker):
        result = ranker.rank_actions("TOOL_WEAR", severity=1.0)
        top = result.ranked_actions[0]
        # At max severity, tool change (most aggressive) should be top
        assert top.action_type == "TOOL_CHANGE"
        assert result.do_nothing_risk > 0.9


# ===========================================================================
# Unknown anomaly type (graceful fallback)
# ===========================================================================


class TestUnknownAnomalyType:
    def test_unknown_anomaly_returns_empty_actions(self, ranker):
        result = ranker.rank_actions("TOTALLY_UNKNOWN", severity=0.5)
        assert len(result.ranked_actions) == 0
        assert result.do_nothing_risk > 0.0

    def test_unknown_anomaly_has_situation_summary(self, ranker):
        result = ranker.rank_actions("TOTALLY_UNKNOWN", severity=0.5)
        assert "TOTALLY_UNKNOWN" in result.situation_summary


# ===========================================================================
# RankedActions dataclass
# ===========================================================================


class TestRankedActionsDataclass:
    def test_situation_summary_populated(self, ranker):
        result = ranker.rank_actions("CHATTER_RISK", severity=0.6)
        assert "CHATTER_RISK" in result.situation_summary
        assert "60%" in result.situation_summary

    def test_result_is_ranked_actions_instance(self, ranker):
        result = ranker.rank_actions("FORCE_WARNING", severity=0.5)
        assert isinstance(result, RankedActions)

    def test_all_candidates_have_scores(self, ranker):
        result = ranker.rank_actions("THERMAL_WARNING", severity=0.5)
        for action in result.ranked_actions:
            assert action.score > 0.0


# ===========================================================================
# Surface improvement estimation
# ===========================================================================


class TestSurfaceImprovement:
    def test_feed_reduction_improves_surface(self, ranker):
        imp = ranker._estimate_surface_improvement("REDUCE_FEED", 100.0, 80.0)
        # (1 - (80/100)^2) * 100 = 36%
        assert imp == pytest.approx(36.0, abs=0.1)

    def test_tool_change_fixed_improvement(self, ranker):
        imp = ranker._estimate_surface_improvement("TOOL_CHANGE", 100.0, 100.0)
        assert imp == 30.0

    def test_pause_no_improvement(self, ranker):
        imp = ranker._estimate_surface_improvement("PAUSE", 100.0, 0.0)
        assert imp == 0.0
