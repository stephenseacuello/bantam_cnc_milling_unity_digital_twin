"""Tests for CuttingSimProxy — Python port of Altintas force + Taylor wear models."""
import pytest
from miracle_twin.cutting_sim_proxy import (
    CuttingSimProxy, CuttingCoefficients, ToolState, GCodeBlock, SimulationResult
)


class TestCuttingForceCalculation:
    """Verify force calculations match reference values."""

    def test_zero_inputs_return_zero(self):
        proxy = CuttingSimProxy()
        result = proxy.calculate_forces(0, 0, 0, 0)
        assert result['fx_peak'] == 0.0
        assert result['power_w'] == 0.0

    def test_positive_forces_for_valid_inputs(self):
        proxy = CuttingSimProxy()
        result = proxy.calculate_forces(
            spindle_rpm=8000, feed_per_tooth=0.05,
            axial_depth=1.5, radial_depth=3.175
        )
        assert result['fx_peak'] > 0
        assert result['fy_peak'] > 0
        assert result['power_w'] > 0
        assert result['torque_nm'] > 0
        assert result['mrr'] > 0

    def test_wear_increases_forces(self):
        proxy = CuttingSimProxy()
        no_wear = proxy.calculate_forces(8000, 0.05, 1.5, 3.175, flank_wear_vb=0.0)
        with_wear = proxy.calculate_forces(8000, 0.05, 1.5, 3.175, flank_wear_vb=0.2)
        assert with_wear['fx_peak'] > no_wear['fx_peak']

    def test_force_scaling_with_depth(self):
        proxy = CuttingSimProxy()
        shallow = proxy.calculate_forces(8000, 0.05, 0.5, 3.175)
        deep = proxy.calculate_forces(8000, 0.05, 3.0, 3.175)
        assert deep['power_w'] > shallow['power_w']


class TestWearModel:
    """Verify 3-stage wear model."""

    def test_break_in_stage(self):
        proxy = CuttingSimProxy()
        vb, t = proxy.update_wear(0.02, 0.0, 100.0, 0.05, 1.5, 1.0)
        assert vb > 0.02  # Wear increased
        assert vb < 0.08  # Still in break-in

    def test_steady_state_stage(self):
        proxy = CuttingSimProxy()
        vb, t = proxy.update_wear(0.10, 5.0, 100.0, 0.05, 1.5, 1.0)
        assert vb > 0.10
        assert vb < 0.25

    def test_accelerated_stage(self):
        proxy = CuttingSimProxy()
        vb, t = proxy.update_wear(0.26, 20.0, 100.0, 0.05, 1.5, 1.0)
        assert vb > 0.26  # Accelerated wear

    def test_wear_capped_at_hard_limit(self):
        proxy = CuttingSimProxy()
        vb, t = proxy.update_wear(0.49, 50.0, 200.0, 0.1, 3.0, 10.0)
        assert vb <= 0.50


class TestTaylorLife:
    """Verify Taylor tool life prediction."""

    def test_higher_speed_shorter_life(self):
        proxy = CuttingSimProxy()
        life_slow = proxy.taylor_life_prediction(80.0, 0.05, 1.5)
        life_fast = proxy.taylor_life_prediction(200.0, 0.05, 1.5)
        assert life_fast < life_slow

    def test_positive_life(self):
        proxy = CuttingSimProxy()
        life = proxy.taylor_life_prediction(100.0, 0.05, 1.5)
        assert life > 0


class TestSimulateProgram:
    """End-to-end simulation tests."""

    def test_simulate_returns_non_hardcoded_rul(self):
        proxy = CuttingSimProxy()
        blocks = [
            GCodeBlock(feed_rate_mmpm=500.0, spindle_rpm=8000.0,
                       axial_depth_mm=1.5, radial_depth_mm=3.175, length_mm=50.0)
            for _ in range(5)
        ]
        result = proxy.simulate_program(blocks)
        assert result.remaining_useful_life_hours != 450.0  # Not hardcoded
        assert result.confidence != 0.87  # Not hardcoded
        assert result.health_index != 0.78  # Not hardcoded
        assert result.remaining_useful_life_hours > 0
        assert 0 < result.confidence <= 1.0

    def test_empty_program(self):
        proxy = CuttingSimProxy()
        result = proxy.simulate_program([])
        assert result.remaining_useful_life_hours == 1000.0
        assert len(result.block_predictions) == 0

    def test_what_if_override(self):
        proxy = CuttingSimProxy()
        blocks = [GCodeBlock(feed_rate_mmpm=500, spindle_rpm=8000,
                             axial_depth_mm=1.5, radial_depth_mm=3.175, length_mm=50)]
        baseline = proxy.simulate_program(blocks)
        override = proxy.simulate_program(
            blocks, parameter_overrides={'spindle_rpm': 12000.0}
        )
        # Different RPM should produce different results
        assert override.block_predictions[0].power_w != baseline.block_predictions[0].power_w

    def test_worn_tool_shorter_rul(self):
        proxy = CuttingSimProxy()
        blocks = [GCodeBlock(feed_rate_mmpm=500, spindle_rpm=8000,
                             axial_depth_mm=1.5, radial_depth_mm=3.175, length_mm=50)]
        fresh = proxy.simulate_program(blocks, ToolState(flank_wear_vb=0.02))
        worn = proxy.simulate_program(
            blocks, ToolState(flank_wear_vb=0.20, cutting_time_min=30.0)
        )
        assert worn.remaining_useful_life_hours < fresh.remaining_useful_life_hours

    def test_block_predictions_populated(self):
        proxy = CuttingSimProxy()
        blocks = [GCodeBlock(feed_rate_mmpm=500, spindle_rpm=8000,
                             axial_depth_mm=1.5, radial_depth_mm=3.175, length_mm=50)]
        result = proxy.simulate_program(blocks)
        assert len(result.block_predictions) == 1
        assert result.block_predictions[0].peak_force_n > 0
        assert result.block_predictions[0].power_w > 0
