"""Tests for CoolantFlowSimulator, NozzleConfig, and CoolantEffectiveness."""
import math
import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_twin.cutting_sim_proxy import (
    CoolantEffectiveness,
    CoolantFlowSimulator,
    NozzleConfig,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def sim():
    return CoolantFlowSimulator()


@pytest.fixture
def flood_nozzle():
    return NozzleConfig(
        nozzle_id='N1',
        position_offset_mm=(10.0, 5.0, 0.0),
        angle_deg=45.0,
        flow_rate_lpm=12.0,
        pressure_bar=10.0,
        nozzle_type='flood',
    )


@pytest.fixture
def jet_nozzle():
    return NozzleConfig(
        nozzle_id='N2',
        position_offset_mm=(5.0, 3.0, 0.0),
        angle_deg=15.0,
        flow_rate_lpm=8.0,
        pressure_bar=50.0,
        nozzle_type='jet',
    )


@pytest.fixture
def through_tool_nozzle():
    return NozzleConfig(
        nozzle_id='N3',
        position_offset_mm=(0.0, 0.0, 0.0),
        angle_deg=0.0,
        flow_rate_lpm=6.0,
        pressure_bar=60.0,
        nozzle_type='through_tool',
    )


# ===================================================================
# NozzleConfig dataclass validation
# ===================================================================

class TestNozzleConfig:
    """Verify NozzleConfig construction and validation."""

    def test_valid_nozzle_types(self):
        for ntype in ('flood', 'mist', 'through_tool', 'jet'):
            n = NozzleConfig(
                nozzle_id='test',
                position_offset_mm=(0.0, 0.0, 0.0),
                angle_deg=0.0,
                flow_rate_lpm=5.0,
                pressure_bar=10.0,
                nozzle_type=ntype,
            )
            assert n.nozzle_type == ntype

    def test_invalid_nozzle_type_raises(self):
        with pytest.raises(ValueError, match="nozzle_type"):
            NozzleConfig(
                nozzle_id='bad',
                position_offset_mm=(0.0, 0.0, 0.0),
                angle_deg=0.0,
                flow_rate_lpm=5.0,
                pressure_bar=10.0,
                nozzle_type='laser',
            )


# ===================================================================
# evaluate_nozzle
# ===================================================================

class TestEvaluateNozzle:
    """Tests for single-nozzle effectiveness evaluation."""

    def test_returns_coolant_effectiveness(self, sim, flood_nozzle):
        result = sim.evaluate_nozzle(flood_nozzle, tool_diameter=10.0,
                                     cutting_depth=3.0, spindle_rpm=5000)
        assert isinstance(result, CoolantEffectiveness)

    def test_scores_within_range(self, sim, flood_nozzle):
        result = sim.evaluate_nozzle(flood_nozzle, 10.0, 3.0, 5000)
        assert 0.0 <= result.coverage_pct <= 100.0
        assert 0.0 <= result.thermal_reduction_pct <= 100.0
        assert 0.0 <= result.chip_evacuation_score <= 100.0
        assert 0.0 <= result.lubrication_score <= 100.0
        assert 0.0 <= result.overall_effectiveness <= 100.0

    def test_through_tool_nozzle_high_effectiveness(self, sim, through_tool_nozzle):
        result = sim.evaluate_nozzle(through_tool_nozzle, 8.0, 5.0, 3000)
        # Through-tool has highest base effectiveness (90)
        assert result.overall_effectiveness > 40.0

    def test_higher_pressure_increases_penetration(self, sim):
        low_p = NozzleConfig('LP', (5.0, 0.0, 0.0), 45.0, 10.0, 5.0, 'flood')
        high_p = NozzleConfig('HP', (5.0, 0.0, 0.0), 45.0, 10.0, 60.0, 'flood')
        r_low = sim.evaluate_nozzle(low_p, 10.0, 5.0, 5000)
        r_high = sim.evaluate_nozzle(high_p, 10.0, 5.0, 5000)
        assert r_high.penetration_depth_mm > r_low.penetration_depth_mm

    def test_invalid_tool_diameter_raises(self, sim, flood_nozzle):
        with pytest.raises(ValueError, match="tool_diameter"):
            sim.evaluate_nozzle(flood_nozzle, 0.0, 3.0, 5000)


# ===================================================================
# evaluate_system
# ===================================================================

class TestEvaluateSystem:
    """Tests for multi-nozzle system evaluation."""

    def test_empty_nozzle_list_returns_zeros(self, sim):
        result = sim.evaluate_system([], 10.0, 3.0, 5000)
        assert result.overall_effectiveness == 0.0
        assert result.coverage_pct == 0.0

    def test_multi_nozzle_better_than_single(self, sim, flood_nozzle, jet_nozzle):
        single = sim.evaluate_nozzle(flood_nozzle, 10.0, 3.0, 5000)
        multi = sim.evaluate_system([flood_nozzle, jet_nozzle], 10.0, 3.0, 5000)
        assert multi.overall_effectiveness >= single.overall_effectiveness

    def test_system_scores_within_range(self, sim, flood_nozzle, jet_nozzle):
        result = sim.evaluate_system([flood_nozzle, jet_nozzle], 10.0, 3.0, 5000)
        assert 0.0 <= result.coverage_pct <= 100.0
        assert 0.0 <= result.thermal_reduction_pct <= 100.0
        assert 0.0 <= result.overall_effectiveness <= 100.0


# ===================================================================
# recommend_nozzle_position
# ===================================================================

class TestRecommendNozzlePosition:
    """Tests for nozzle position recommendations."""

    def test_drilling_recommends_through_tool(self, sim):
        rec = sim.recommend_nozzle_position(10.0, 'drilling')
        assert rec['recommended_nozzle_type'] == 'through_tool'
        assert rec['angle_deg'] == 0.0

    def test_finishing_recommends_mist(self, sim):
        rec = sim.recommend_nozzle_position(10.0, 'finishing')
        assert rec['recommended_nozzle_type'] == 'mist'

    def test_distance_scales_with_diameter(self, sim):
        rec_small = sim.recommend_nozzle_position(5.0, 'general')
        rec_large = sim.recommend_nozzle_position(20.0, 'general')
        assert rec_large['distance_mm'] > rec_small['distance_mm']

    def test_unknown_operation_uses_general(self, sim):
        rec = sim.recommend_nozzle_position(10.0, 'unknown_op')
        general = sim.recommend_nozzle_position(10.0, 'general')
        assert rec['angle_deg'] == general['angle_deg']
        assert rec['recommended_nozzle_type'] == general['recommended_nozzle_type']


# ===================================================================
# calculate_required_flow
# ===================================================================

class TestCalculateRequiredFlow:
    """Tests for minimum flow rate calculation."""

    def test_titanium_needs_more_than_aluminum(self, sim):
        ti = sim.calculate_required_flow(10.0, 100.0, 'titanium')
        al = sim.calculate_required_flow(10.0, 100.0, 'aluminum')
        assert ti > al

    def test_minimum_flow_floor(self, sim):
        flow = sim.calculate_required_flow(1.0, 1.0, 'aluminum')
        assert flow >= 2.0

    def test_flow_increases_with_diameter(self, sim):
        small = sim.calculate_required_flow(5.0, 100.0, 'steel')
        large = sim.calculate_required_flow(25.0, 100.0, 'steel')
        assert large > small


# ===================================================================
# get_through_tool_effectiveness
# ===================================================================

class TestGetThroughToolEffectiveness:
    """Tests for through-tool coolant evaluation."""

    def test_returns_coolant_effectiveness(self, sim):
        result = sim.get_through_tool_effectiveness(50.0, 10.0, 30.0)
        assert isinstance(result, CoolantEffectiveness)

    def test_higher_pressure_improves_scores(self, sim):
        low = sim.get_through_tool_effectiveness(10.0, 10.0, 30.0)
        high = sim.get_through_tool_effectiveness(60.0, 10.0, 30.0)
        assert high.overall_effectiveness > low.overall_effectiveness

    def test_deep_hole_reduces_effectiveness(self, sim):
        shallow = sim.get_through_tool_effectiveness(40.0, 10.0, 10.0)
        deep = sim.get_through_tool_effectiveness(40.0, 10.0, 150.0)
        assert shallow.overall_effectiveness > deep.overall_effectiveness

    def test_invalid_hole_diameter_raises(self, sim):
        with pytest.raises(ValueError, match="hole_diameter"):
            sim.get_through_tool_effectiveness(50.0, 0.0, 10.0)
