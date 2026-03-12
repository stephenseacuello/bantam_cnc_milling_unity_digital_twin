"""Tests for tool deflection model and surface roughness prediction."""
import pytest
from miracle_twin.cutting_sim_proxy import (
    CuttingSimProxy,
    GCodeBlock,
    ToolDeflectionModel,
    ToolState,
    SurfaceRoughnessModel,
)
from miracle_twin.tool_library import ToolDefinition


# ---------------------------------------------------------------------------
# ToolDeflectionModel tests
# ---------------------------------------------------------------------------

class TestToolDeflectionModel:
    """Verify cantilever beam deflection model for end mills."""

    def test_deflection_increases_with_force(self):
        low = ToolDeflectionModel.compute_deflection(50.0, 6.35, 30.0, 200.0)
        high = ToolDeflectionModel.compute_deflection(200.0, 6.35, 30.0, 200.0)
        assert high > low

    def test_deflection_increases_with_overhang(self):
        short = ToolDeflectionModel.compute_deflection(100.0, 6.35, 20.0, 200.0)
        long = ToolDeflectionModel.compute_deflection(100.0, 6.35, 50.0, 200.0)
        assert long > short

    def test_carbide_deflects_less_than_hss(self):
        hss = ToolDeflectionModel.compute_deflection(100.0, 6.35, 30.0, 200.0)
        carbide = ToolDeflectionModel.compute_deflection(100.0, 6.35, 30.0, 620.0)
        assert carbide < hss

    def test_smaller_diameter_deflects_more(self):
        large = ToolDeflectionModel.compute_deflection(100.0, 10.0, 30.0, 200.0)
        small = ToolDeflectionModel.compute_deflection(100.0, 3.0, 30.0, 200.0)
        assert small > large

    def test_zero_force_gives_zero_deflection(self):
        result = ToolDeflectionModel.compute_deflection(0.0, 6.35, 30.0, 200.0)
        assert result == 0.0

    def test_dimensional_error_matches_deflection(self):
        defl = ToolDeflectionModel.compute_deflection(100.0, 6.35, 30.0, 200.0)
        err = ToolDeflectionModel.compute_dimensional_error(100.0, 6.35, 30.0, 200.0)
        assert defl == err

    def test_zero_modulus_returns_zero(self):
        result = ToolDeflectionModel.compute_deflection(100.0, 6.35, 30.0, 0.0)
        assert result == 0.0

    def test_deflection_is_positive_for_positive_force(self):
        result = ToolDeflectionModel.compute_deflection(100.0, 6.35, 30.0, 200.0)
        assert result > 0.0

    def test_negative_force_treated_as_positive(self):
        pos = ToolDeflectionModel.compute_deflection(100.0, 6.35, 30.0, 200.0)
        neg = ToolDeflectionModel.compute_deflection(-100.0, 6.35, 30.0, 200.0)
        assert pos == neg


# ---------------------------------------------------------------------------
# SurfaceRoughnessModel tests
# ---------------------------------------------------------------------------

class TestSurfaceRoughnessModel:
    """Verify theoretical and empirical surface roughness prediction."""

    def test_ra_increases_with_feed_per_tooth(self):
        low_feed = SurfaceRoughnessModel.compute_ra(0.02, 0.4)
        high_feed = SurfaceRoughnessModel.compute_ra(0.10, 0.4)
        assert high_feed > low_feed

    def test_ra_increases_with_flank_wear(self):
        no_wear = SurfaceRoughnessModel.compute_ra(0.05, 0.4, flank_wear_vb_mm=0.0)
        with_wear = SurfaceRoughnessModel.compute_ra(0.05, 0.4, flank_wear_vb_mm=0.2)
        assert with_wear > no_wear

    def test_ra_increases_with_vibration(self):
        no_vib = SurfaceRoughnessModel.compute_ra(0.05, 0.4, vibration_amplitude_mm=0.0)
        with_vib = SurfaceRoughnessModel.compute_ra(0.05, 0.4, vibration_amplitude_mm=0.05)
        assert with_vib > no_vib

    def test_larger_nose_radius_gives_better_finish(self):
        small_r = SurfaceRoughnessModel.compute_ra(0.05, 0.2)
        large_r = SurfaceRoughnessModel.compute_ra(0.05, 0.8)
        assert large_r < small_r

    def test_depth_factor_worsens_finish_for_deep_cuts(self):
        shallow = SurfaceRoughnessModel.compute_ra(0.05, 0.4, depth_of_cut_mm=0.5)
        deep = SurfaceRoughnessModel.compute_ra(0.05, 0.4, depth_of_cut_mm=5.0)
        assert deep > shallow

    def test_minimum_ra_is_0_1(self):
        # Very small feed and no wear should clamp at 0.1 um
        result = SurfaceRoughnessModel.compute_ra(0.001, 0.8)
        assert result >= 0.1

    def test_zero_nose_radius_uses_default(self):
        # Should not raise, uses fallback 0.4 mm
        result = SurfaceRoughnessModel.compute_ra(0.05, 0.0)
        assert result > 0.0

    def test_ra_with_all_contributions(self):
        result = SurfaceRoughnessModel.compute_ra(
            feed_per_tooth_mm=0.08,
            nose_radius_mm=0.4,
            vibration_amplitude_mm=0.02,
            flank_wear_vb_mm=0.1,
            depth_of_cut_mm=3.0,
        )
        # Should be a reasonable positive value
        assert result > 0.5


# ---------------------------------------------------------------------------
# Integration tests — simulate_program includes deflection/surface
# ---------------------------------------------------------------------------

class TestSimulationIntegration:
    """Verify deflection and surface Ra appear in simulation output."""

    @pytest.fixture
    def proxy_with_tool(self):
        proxy = CuttingSimProxy()
        proxy.set_tool('HSS_2F_6mm')
        return proxy

    @pytest.fixture
    def standard_blocks(self):
        return [
            GCodeBlock(
                feed_rate_mmpm=500, spindle_rpm=8000,
                axial_depth_mm=1.5, radial_depth_mm=3.175,
                length_mm=20.0,
            ),
            GCodeBlock(
                feed_rate_mmpm=500, spindle_rpm=8000,
                axial_depth_mm=1.5, radial_depth_mm=3.175,
                length_mm=20.0,
            ),
        ]

    def test_simulate_program_includes_deflection(self, proxy_with_tool, standard_blocks):
        result = proxy_with_tool.simulate_program(standard_blocks)
        assert result.max_deflection_mm > 0.0
        assert result.max_dimensional_error_mm > 0.0

    def test_simulate_program_includes_surface_ra(self, proxy_with_tool, standard_blocks):
        result = proxy_with_tool.simulate_program(standard_blocks)
        assert result.avg_surface_ra_um > 0.0
        assert result.max_surface_ra_um > 0.0

    def test_block_predictions_have_deflection(self, proxy_with_tool, standard_blocks):
        result = proxy_with_tool.simulate_program(standard_blocks)
        for bp in result.block_predictions:
            assert bp.deflection_mm >= 0.0
            assert bp.surface_ra_um >= 0.1

    def test_what_if_compare_shows_surface_ra_changes(self):
        proxy = CuttingSimProxy()
        proxy.set_tool('HSS_2F_6mm')
        blocks = [
            GCodeBlock(
                feed_rate_mmpm=500, spindle_rpm=8000,
                axial_depth_mm=1.5, radial_depth_mm=3.175,
                length_mm=20.0,
            ),
        ]
        comparison = proxy.what_if_compare(
            blocks,
            baseline_params={'feed_rate': 500},
            override_params={'feed_rate': 1000},
        )
        assert 'surface_ra_change_pct' in comparison.delta
        assert 'deflection_change_pct' in comparison.delta

    def test_what_if_surface_ra_increases_with_higher_feed(self):
        proxy = CuttingSimProxy()
        proxy.set_tool('HSS_2F_6mm')
        blocks = [
            GCodeBlock(
                feed_rate_mmpm=500, spindle_rpm=8000,
                axial_depth_mm=1.5, radial_depth_mm=3.175,
                length_mm=20.0,
            ),
        ]
        comparison = proxy.what_if_compare(
            blocks,
            baseline_params={'feed_rate': 300},
            override_params={'feed_rate': 1200},
        )
        # Higher feed -> worse surface finish -> positive Ra change
        assert comparison.delta['surface_ra_change_pct'] > 0.0

    def test_carbide_tool_less_deflection_in_simulation(self):
        proxy_hss = CuttingSimProxy()
        proxy_hss.set_tool('HSS_2F_6mm')
        proxy_carbide = CuttingSimProxy()
        proxy_carbide.set_tool('CARBIDE_4F_10mm')
        blocks = [
            GCodeBlock(
                feed_rate_mmpm=500, spindle_rpm=8000,
                axial_depth_mm=1.5, radial_depth_mm=3.175,
                length_mm=20.0,
            ),
        ]
        result_hss = proxy_hss.simulate_program(blocks)
        result_carbide = proxy_carbide.simulate_program(blocks)
        # Carbide has higher modulus AND larger diameter -> less deflection
        assert result_carbide.max_deflection_mm < result_hss.max_deflection_mm
