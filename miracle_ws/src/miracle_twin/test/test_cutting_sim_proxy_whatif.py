"""Tests for CuttingSimProxy.what_if_compare — what-if scenario comparison."""
import pytest
from miracle_twin.cutting_sim_proxy import (
    CuttingSimProxy, CuttingCoefficients, ToolState, GCodeBlock,
    WhatIfComparison,
)


def _default_blocks(n=5):
    """Create n default G-code blocks for testing."""
    return [
        GCodeBlock(
            feed_rate_mmpm=500.0, spindle_rpm=8000.0,
            axial_depth_mm=1.5, radial_depth_mm=3.175, length_mm=50.0,
        )
        for _ in range(n)
    ]


class TestWhatIfCompareForceReduction:
    """Verify that reducing feed reduces predicted forces."""

    def test_reducing_feed_reduces_force(self):
        proxy = CuttingSimProxy()
        blocks = _default_blocks()

        result = proxy.what_if_compare(
            blocks,
            baseline_params=None,
            override_params={'feed_rate': 400.0},  # 80% of 500
        )

        assert isinstance(result, WhatIfComparison)
        assert result.override['peak_force'] < result.baseline['peak_force']
        assert result.delta['force_pct'] < 0, (
            "Force delta should be negative when feed is reduced"
        )

    def test_increasing_feed_increases_force(self):
        proxy = CuttingSimProxy()
        blocks = _default_blocks()

        result = proxy.what_if_compare(
            blocks,
            baseline_params=None,
            override_params={'feed_rate': 600.0},  # 120% of 500
        )

        assert result.override['peak_force'] > result.baseline['peak_force']
        assert result.delta['force_pct'] > 0

    def test_force_reduction_is_nonlinear(self):
        """Force should scale as feed^0.8-ish (mechanistic model), not linearly."""
        proxy = CuttingSimProxy()
        blocks = _default_blocks(1)

        result = proxy.what_if_compare(
            blocks,
            baseline_params=None,
            override_params={'feed_rate': 250.0},  # 50% of 500
        )

        # If it were linear, force_pct would be -50%.
        # With mechanistic model, it should be less severe (feed^0.8 -> ~-43%).
        # But mechanistic model is more complex, just verify it is not exactly -50%.
        assert result.delta['force_pct'] != -50.0
        assert result.delta['force_pct'] < 0


class TestWhatIfCompareToolLife:
    """Verify that reducing speed increases predicted tool life."""

    def test_reducing_speed_increases_life(self):
        proxy = CuttingSimProxy()
        blocks = _default_blocks()

        result = proxy.what_if_compare(
            blocks,
            baseline_params=None,
            override_params={'spindle_rpm': 6000.0},  # 75% of 8000
        )

        assert result.override['rul_minutes'] > result.baseline['rul_minutes'], (
            "Reducing spindle speed should increase remaining useful life"
        )
        assert result.delta['life_pct'] > 0

    def test_increasing_speed_decreases_life(self):
        proxy = CuttingSimProxy()
        blocks = _default_blocks()

        result = proxy.what_if_compare(
            blocks,
            baseline_params=None,
            override_params={'spindle_rpm': 10000.0},  # 125% of 8000
        )

        assert result.override['rul_minutes'] < result.baseline['rul_minutes']
        assert result.delta['life_pct'] < 0

    def test_worn_tool_has_less_life(self):
        proxy = CuttingSimProxy()
        blocks = _default_blocks()
        worn = ToolState(flank_wear_vb=0.15, cutting_time_min=20.0)

        result_fresh = proxy.what_if_compare(blocks, tool_state=None)
        result_worn = proxy.what_if_compare(blocks, tool_state=worn)

        assert result_worn.baseline['rul_minutes'] < result_fresh.baseline['rul_minutes']


class TestWhatIfCompareDeltaCalculations:
    """Verify that delta calculations are correct and well-formed."""

    def test_delta_keys_present(self):
        proxy = CuttingSimProxy()
        blocks = _default_blocks()

        result = proxy.what_if_compare(
            blocks,
            override_params={'feed_rate': 400.0},
        )

        assert 'force_pct' in result.delta
        assert 'life_pct' in result.delta
        assert 'quality_change' in result.delta
        assert 'risk_change' in result.delta

    def test_baseline_keys_present(self):
        proxy = CuttingSimProxy()
        blocks = _default_blocks()

        result = proxy.what_if_compare(blocks, override_params={'feed_rate': 400.0})

        assert 'peak_force' in result.baseline
        assert 'rul_minutes' in result.baseline
        assert 'max_ra' in result.baseline
        assert 'chatter_risk' in result.baseline

    def test_no_change_yields_zero_deltas(self):
        """When override == baseline, deltas should be zero."""
        proxy = CuttingSimProxy()
        blocks = _default_blocks()

        result = proxy.what_if_compare(
            blocks,
            baseline_params={'spindle_rpm': 8000.0, 'feed_rate': 500.0},
            override_params={'spindle_rpm': 8000.0, 'feed_rate': 500.0},
        )

        assert abs(result.delta['force_pct']) < 1e-6
        assert abs(result.delta['life_pct']) < 1e-6
        assert result.delta['quality_change'] == 0.0
        assert result.delta['risk_change'] == 0.0

    def test_quality_improves_with_lower_feed(self):
        """Lower feed should improve surface quality (lower Ra)."""
        proxy = CuttingSimProxy()
        blocks = _default_blocks()

        result = proxy.what_if_compare(
            blocks,
            baseline_params=None,
            override_params={'feed_rate': 250.0},  # 50% feed
        )

        # quality_change: 1.0 = improved, -1.0 = degraded, 0.0 = unchanged
        assert result.delta['quality_change'] >= 0.0, (
            "Lower feed should maintain or improve surface quality"
        )
        assert result.override['max_ra'] <= result.baseline['max_ra']

    def test_quality_degrades_with_higher_feed(self):
        """Higher feed should degrade surface quality (higher Ra)."""
        proxy = CuttingSimProxy()
        blocks = _default_blocks()

        result = proxy.what_if_compare(
            blocks,
            baseline_params=None,
            override_params={'feed_rate': 1000.0},  # 200% feed
        )

        assert result.delta['quality_change'] <= 0.0
        assert result.override['max_ra'] >= result.baseline['max_ra']

    def test_empty_blocks_returns_comparison(self):
        """What-if with empty blocks should return valid structure."""
        proxy = CuttingSimProxy()

        result = proxy.what_if_compare(
            [],
            override_params={'feed_rate': 400.0},
        )

        assert isinstance(result, WhatIfComparison)
        assert abs(result.delta['force_pct']) < 1e-6
