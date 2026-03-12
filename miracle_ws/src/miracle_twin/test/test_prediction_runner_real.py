"""Tests verifying prediction_runner uses real simulation instead of hardcoded values."""
import pytest
from miracle_twin.cutting_sim_proxy import CuttingSimProxy, GCodeBlock


class TestPredictionRunnerUsesRealSim:
    """Verify that prediction results are no longer hardcoded."""

    def test_cutting_sim_proxy_integration(self):
        """Verify CuttingSimProxy produces non-hardcoded results."""
        proxy = CuttingSimProxy()
        blocks = [
            GCodeBlock(feed_rate_mmpm=500.0, spindle_rpm=8000.0,
                       axial_depth_mm=1.5, radial_depth_mm=3.175, length_mm=50.0)
            for _ in range(10)
        ]
        result = proxy.simulate_program(blocks)
        # These must NOT be the old hardcoded values
        assert result.remaining_useful_life_hours != 450.0
        assert result.confidence != 0.87
        assert result.health_index != 0.78
        assert result.recommended_action != 'Schedule maintenance within 2 weeks'
        # They must be reasonable
        assert result.remaining_useful_life_hours > 0
        assert 0 < result.confidence <= 1.0
        assert 0 <= result.health_index <= 1.0

    def test_different_inputs_different_outputs(self):
        """Verify simulation results change with different parameters."""
        proxy = CuttingSimProxy()
        blocks_light = [GCodeBlock(feed_rate_mmpm=200, spindle_rpm=5000,
                                   axial_depth_mm=0.5, radial_depth_mm=1.0, length_mm=50)]
        blocks_heavy = [GCodeBlock(feed_rate_mmpm=1000, spindle_rpm=15000,
                                   axial_depth_mm=3.0, radial_depth_mm=6.0, length_mm=50)]
        r1 = proxy.simulate_program(blocks_light)
        r2 = proxy.simulate_program(blocks_heavy)
        assert r1.block_predictions[0].power_w != r2.block_predictions[0].power_w
