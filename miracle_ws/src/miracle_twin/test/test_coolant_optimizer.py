"""Tests for CoolantOptimizer and CoolantRecommendation in CuttingSimProxy."""
import math

import pytest

from miracle_twin.cutting_sim_proxy import (
    CoolantConfig,
    CoolantOptimizer,
    CoolantRecommendation,
    CuttingSimProxy,
    GCodeBlock,
    ToolState,
)


# ---------------------------------------------------------------------------
# CoolantRecommendation dataclass basics
# ---------------------------------------------------------------------------

class TestCoolantRecommendationDataclass:
    """Verify the CoolantRecommendation dataclass fields."""

    def test_fields_present(self):
        rec = CoolantRecommendation(
            recommended_type='flood',
            current_type='dry',
            reason='test',
            thermal_improvement_pct=50.0,
            wear_improvement_pct=30.0,
            cost_factor=3.5,
            environmental_score=0.45,
        )
        assert rec.recommended_type == 'flood'
        assert rec.current_type == 'dry'
        assert rec.reason == 'test'
        assert rec.thermal_improvement_pct == 50.0
        assert rec.wear_improvement_pct == 30.0
        assert rec.cost_factor == 3.5
        assert rec.environmental_score == 0.45


# ---------------------------------------------------------------------------
# CoolantOptimizer — low speed scenarios
# ---------------------------------------------------------------------------

class TestLowSpeedRecommendations:
    """Low speed + shallow cut should recommend dry or mist."""

    def test_very_low_speed_shallow_cut_recommends_dry(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('finishing', 40.0, 0.5, 30.0, 0.001, 'flood')
        assert rec.recommended_type == 'dry'

    def test_moderate_low_speed_shallow_cut_recommends_mist(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('finishing', 80.0, 1.0, 30.0, 0.001, 'flood')
        assert rec.recommended_type == 'mist'

    def test_low_speed_dry_saves_cost(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('finishing', 40.0, 0.5, 30.0, 0.001, 'flood')
        assert rec.cost_factor <= 1.0

    def test_low_speed_dry_has_high_environmental_score(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('finishing', 40.0, 0.5, 30.0, 0.001, 'flood')
        assert rec.environmental_score >= 0.9


# ---------------------------------------------------------------------------
# CoolantOptimizer — high speed deep cut scenarios
# ---------------------------------------------------------------------------

class TestHighSpeedDeepCut:
    """High speed + deep cut should recommend flood or high_pressure."""

    def test_high_speed_deep_cut_recommends_high_pressure(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('roughing', 250.0, 4.0, 100.0, 0.005, 'dry')
        assert rec.recommended_type == 'high_pressure'

    def test_moderate_speed_deep_cut_recommends_flood(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('roughing', 120.0, 2.5, 80.0, 0.005, 'dry')
        assert rec.recommended_type == 'flood'

    def test_high_speed_deep_cut_has_reason(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('roughing', 250.0, 4.0, 100.0, 0.005, 'dry')
        assert len(rec.reason) > 0


# ---------------------------------------------------------------------------
# CoolantOptimizer — high temperature scenarios
# ---------------------------------------------------------------------------

class TestHighTemperature:
    """Near or exceeding thermal limit should upgrade coolant."""

    def test_high_temp_recommends_flood(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('general', 80.0, 2.0, 280.0, 0.005, 'mist')
        assert rec.recommended_type == 'flood'

    def test_high_temp_high_speed_recommends_high_pressure(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('roughing', 180.0, 2.0, 300.0, 0.005, 'mist')
        assert rec.recommended_type == 'high_pressure'

    def test_high_temp_reason_mentions_temperature(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('general', 80.0, 2.0, 280.0, 0.005, 'mist')
        assert 'temperature' in rec.reason.lower() or 'limit' in rec.reason.lower()


# ---------------------------------------------------------------------------
# CoolantOptimizer — high wear rate scenarios
# ---------------------------------------------------------------------------

class TestHighWearRate:
    """High wear rate should upgrade coolant to extend tool life."""

    def test_high_wear_rate_upgrades_to_flood(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('general', 80.0, 2.0, 100.0, 0.03, 'mist')
        assert rec.recommended_type == 'flood'

    def test_high_wear_rate_high_speed_upgrades_to_high_pressure(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('roughing', 180.0, 2.0, 100.0, 0.03, 'mist')
        assert rec.recommended_type == 'high_pressure'


# ---------------------------------------------------------------------------
# CoolantOptimizer — titanium / Inconel materials
# ---------------------------------------------------------------------------

class TestHardMaterials:
    """Titanium and Inconel should always get high_pressure or cryogenic."""

    def test_titanium_recommends_high_pressure(self):
        opt = CoolantOptimizer(material='Ti-6Al-4V')
        rec = opt.recommend_coolant('roughing', 60.0, 1.0, 100.0, 0.005, 'flood')
        assert rec.recommended_type in ('high_pressure', 'cryogenic')

    def test_inconel_recommends_high_pressure(self):
        opt = CoolantOptimizer(material='Inconel-718')
        rec = opt.recommend_coolant('roughing', 40.0, 0.5, 100.0, 0.005, 'dry')
        assert rec.recommended_type in ('high_pressure', 'cryogenic')

    def test_titanium_high_temp_recommends_cryogenic(self):
        opt = CoolantOptimizer(material='Ti-6Al-4V')
        rec = opt.recommend_coolant('roughing', 80.0, 2.0, 300.0, 0.01, 'flood')
        assert rec.recommended_type == 'cryogenic'

    def test_inconel_high_temp_recommends_cryogenic(self):
        opt = CoolantOptimizer(material='Inconel-718')
        rec = opt.recommend_coolant('roughing', 80.0, 2.0, 300.0, 0.01, 'flood')
        assert rec.recommended_type == 'cryogenic'

    def test_titanium_reason_mentions_material(self):
        opt = CoolantOptimizer(material='Ti-6Al-4V')
        rec = opt.recommend_coolant('roughing', 60.0, 1.0, 100.0, 0.005, 'flood')
        assert 'Ti-6Al-4V' in rec.reason


# ---------------------------------------------------------------------------
# CoolantOptimizer — aluminum 6061-T6 various scenarios
# ---------------------------------------------------------------------------

class TestAluminum6061:
    """Various scenarios for standard aluminum alloy."""

    def test_default_material_is_aluminum(self):
        opt = CoolantOptimizer()
        assert opt.material == '6061-T6'

    def test_moderate_conditions_recommend_flood(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('general', 130.0, 2.5, 120.0, 0.005, 'dry')
        assert rec.recommended_type == 'flood'

    def test_aluminum_low_speed_finishing(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('finishing', 50.0, 0.3, 40.0, 0.001, 'flood')
        assert rec.recommended_type == 'dry'

    def test_aluminum_high_speed_roughing(self):
        opt = CoolantOptimizer(material='6061-T6')
        rec = opt.recommend_coolant('roughing', 220.0, 5.0, 150.0, 0.005, 'dry')
        assert rec.recommended_type == 'high_pressure'


# ---------------------------------------------------------------------------
# Environmental score ordering
# ---------------------------------------------------------------------------

class TestEnvironmentalScore:
    """Verify environmental scores follow expected ordering."""

    def test_dry_has_highest_env_score(self):
        assert CoolantOptimizer.ENV_SCORE['dry'] == 1.0

    def test_env_score_ordering(self):
        scores = CoolantOptimizer.ENV_SCORE
        assert scores['dry'] > scores['mist']
        assert scores['mist'] > scores['cryogenic']
        assert scores['cryogenic'] > scores['flood']
        assert scores['flood'] > scores['high_pressure']

    def test_all_env_scores_between_0_and_1(self):
        for ct, score in CoolantOptimizer.ENV_SCORE.items():
            assert 0.0 <= score <= 1.0, f'{ct} score {score} out of range'


# ---------------------------------------------------------------------------
# Cost factor accuracy
# ---------------------------------------------------------------------------

class TestCostFactor:
    """Verify cost model values and ordering."""

    def test_dry_is_cheapest(self):
        assert CoolantOptimizer.COST_MODEL['dry'] == 1.0

    def test_cost_ordering(self):
        costs = CoolantOptimizer.COST_MODEL
        assert costs['dry'] < costs['mist']
        assert costs['mist'] < costs['flood']
        assert costs['flood'] < costs['high_pressure']
        assert costs['high_pressure'] < costs['cryogenic']

    def test_get_coolant_cost_model_returns_dict(self):
        opt = CoolantOptimizer()
        model = opt.get_coolant_cost_model()
        assert isinstance(model, dict)
        assert len(model) == 5

    def test_cost_model_matches_class_constant(self):
        opt = CoolantOptimizer()
        model = opt.get_coolant_cost_model()
        for ct in ('dry', 'mist', 'flood', 'high_pressure', 'cryogenic'):
            assert model[ct] == CoolantOptimizer.COST_MODEL[ct]


# ---------------------------------------------------------------------------
# evaluate_all_coolants
# ---------------------------------------------------------------------------

class TestEvaluateAllCoolants:
    """Verify all-coolant evaluation and sorting."""

    def test_returns_five_recommendations(self):
        opt = CoolantOptimizer()
        results = opt.evaluate_all_coolants('roughing', 150.0, 3.0)
        assert len(results) == 5

    def test_all_results_are_coolant_recommendations(self):
        opt = CoolantOptimizer()
        results = opt.evaluate_all_coolants('roughing', 150.0, 3.0)
        for rec in results:
            assert isinstance(rec, CoolantRecommendation)

    def test_sorted_by_net_benefit_descending(self):
        opt = CoolantOptimizer()
        results = opt.evaluate_all_coolants('roughing', 150.0, 3.0)
        benefits = [
            (r.thermal_improvement_pct + r.wear_improvement_pct) / max(r.cost_factor, 0.01)
            for r in results
        ]
        for i in range(len(benefits) - 1):
            assert benefits[i] >= benefits[i + 1]

    def test_dry_has_zero_improvement(self):
        opt = CoolantOptimizer()
        results = opt.evaluate_all_coolants('roughing', 150.0, 3.0)
        dry = [r for r in results if r.recommended_type == 'dry'][0]
        assert dry.thermal_improvement_pct == 0.0
        assert dry.wear_improvement_pct == 0.0

    def test_each_coolant_type_present(self):
        opt = CoolantOptimizer()
        results = opt.evaluate_all_coolants('roughing', 150.0, 3.0)
        types = {r.recommended_type for r in results}
        assert types == {'dry', 'mist', 'flood', 'high_pressure', 'cryogenic'}


# ---------------------------------------------------------------------------
# Edge cases: zero speed, zero depth
# ---------------------------------------------------------------------------

class TestEdgeCases:
    """Edge cases — zero speed, zero depth, unknown material."""

    def test_zero_speed_recommends_dry(self):
        opt = CoolantOptimizer()
        rec = opt.recommend_coolant('general', 0.0, 2.0, 50.0, 0.0, 'flood')
        assert rec.recommended_type == 'dry'

    def test_zero_depth_recommends_dry(self):
        opt = CoolantOptimizer()
        rec = opt.recommend_coolant('general', 100.0, 0.0, 50.0, 0.0, 'flood')
        assert rec.recommended_type == 'dry'

    def test_negative_speed_recommends_dry(self):
        opt = CoolantOptimizer()
        rec = opt.recommend_coolant('general', -10.0, 2.0, 50.0, 0.0, 'flood')
        assert rec.recommended_type == 'dry'

    def test_zero_speed_reason_mentions_idle(self):
        opt = CoolantOptimizer()
        rec = opt.recommend_coolant('general', 0.0, 2.0, 50.0, 0.0, 'flood')
        assert 'idle' in rec.reason.lower() or 'unnecessary' in rec.reason.lower()

    def test_unknown_material_falls_back_to_aluminum(self):
        opt = CoolantOptimizer(material='unobtanium')
        rec = opt.recommend_coolant('roughing', 80.0, 1.0, 100.0, 0.005, 'dry')
        # Should not crash and should behave like 6061-T6
        opt_al = CoolantOptimizer(material='6061-T6')
        rec_al = opt_al.recommend_coolant('roughing', 80.0, 1.0, 100.0, 0.005, 'dry')
        assert rec.recommended_type == rec_al.recommended_type


# ---------------------------------------------------------------------------
# CuttingSimProxy.get_coolant_recommendation integration
# ---------------------------------------------------------------------------

class TestCuttingSimProxyIntegration:
    """Integration: CuttingSimProxy.get_coolant_recommendation."""

    def test_returns_coolant_recommendation(self):
        proxy = CuttingSimProxy()
        block = GCodeBlock(
            feed_rate_mmpm=500.0,
            spindle_rpm=8000.0,
            axial_depth_mm=1.5,
            radial_depth_mm=3.175,
            length_mm=20.0,
        )
        tool = ToolState()
        rec = proxy.get_coolant_recommendation(block, tool)
        assert isinstance(rec, CoolantRecommendation)

    def test_current_type_matches_proxy_coolant(self):
        proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='mist'))
        block = GCodeBlock(
            feed_rate_mmpm=500.0,
            spindle_rpm=8000.0,
            axial_depth_mm=1.5,
            radial_depth_mm=3.175,
            length_mm=20.0,
        )
        tool = ToolState()
        rec = proxy.get_coolant_recommendation(block, tool)
        assert rec.current_type == 'mist'

    def test_idle_block_recommends_dry(self):
        proxy = CuttingSimProxy()
        block = GCodeBlock(
            feed_rate_mmpm=0.0,
            spindle_rpm=0.0,
            axial_depth_mm=0.0,
            radial_depth_mm=0.0,
            length_mm=0.0,
        )
        tool = ToolState()
        rec = proxy.get_coolant_recommendation(block, tool)
        assert rec.recommended_type == 'dry'

    def test_with_titanium_material(self):
        proxy = CuttingSimProxy()
        block = GCodeBlock(
            feed_rate_mmpm=300.0,
            spindle_rpm=3000.0,
            axial_depth_mm=2.0,
            radial_depth_mm=3.175,
            length_mm=20.0,
        )
        tool = ToolState()
        rec = proxy.get_coolant_recommendation(block, tool, material='Ti-6Al-4V')
        assert rec.recommended_type in ('high_pressure', 'cryogenic')
