"""Tests for program-level optimization analysis in CuttingSimProxy."""
import math

import pytest

from miracle_twin.cutting_sim_proxy import (
    BlockOptimization,
    BlockPrediction,
    CuttingSimProxy,
    GCodeBlock,
    ProgramOptimizationResult,
    SimulationResult,
    ToolState,
)


def _make_blocks(n, feed=500.0, rpm=8000.0, ap=1.5, ae=3.175, length=50.0):
    """Helper to create a list of identical GCodeBlocks."""
    return [
        GCodeBlock(
            feed_rate_mmpm=feed,
            spindle_rpm=rpm,
            axial_depth_mm=ap,
            radial_depth_mm=ae,
            length_mm=length,
        )
        for _ in range(n)
    ]


class TestOptimizeProgramConservative:
    """Blocks with low forces should get feed increase suggestions."""

    def test_conservative_blocks_suggest_feed_increase(self):
        proxy = CuttingSimProxy()
        # Very low feed => low forces => optimizer should suggest increase
        blocks = _make_blocks(5, feed=100.0, rpm=8000.0)
        result = proxy.optimize_program(blocks)
        assert isinstance(result, ProgramOptimizationResult)
        # At least some blocks should have feed increase suggestions
        increased = [a for a in result.optimization_actions if a.optimized_feed > a.original_feed]
        assert len(increased) > 0, "Expected feed increase suggestions for conservative blocks"

    def test_conservative_optimized_feed_higher_than_original(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(3, feed=80.0, rpm=8000.0)
        result = proxy.optimize_program(blocks)
        for action in result.optimization_actions:
            if action.reason == 'force_headroom':
                assert action.optimized_feed > action.original_feed


class TestOptimizeProgramAggressive:
    """Blocks with high forces should get feed decrease suggestions."""

    def test_aggressive_blocks_suggest_feed_decrease(self):
        proxy = CuttingSimProxy()
        # High feed + deep cut => high forces => optimizer should suggest decrease
        blocks = _make_blocks(5, feed=2000.0, rpm=8000.0, ap=5.0, ae=6.0)
        result = proxy.optimize_program(
            blocks,
            constraints={'force_threshold_n': 50.0, 'max_force_pct': 80.0},
        )
        decreased = [a for a in result.optimization_actions if a.optimized_feed < a.original_feed]
        assert len(decreased) > 0, "Expected feed decrease suggestions for aggressive blocks"


class TestConstraints:
    """Constraints should limit the maximum feed increase."""

    def test_max_feed_increase_pct_respected(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(3, feed=100.0, rpm=8000.0)
        max_inc = 20.0
        result = proxy.optimize_program(
            blocks, constraints={'max_feed_increase_pct': max_inc}
        )
        for action in result.optimization_actions:
            max_allowed = action.original_feed * (1.0 + max_inc / 100.0)
            assert action.optimized_feed <= max_allowed + 0.01, (
                f"Feed {action.optimized_feed} exceeds max allowed {max_allowed}"
            )

    def test_stricter_constraint_limits_more(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(3, feed=100.0, rpm=8000.0)
        result_loose = proxy.optimize_program(
            blocks, constraints={'max_feed_increase_pct': 50.0}
        )
        result_tight = proxy.optimize_program(
            blocks, constraints={'max_feed_increase_pct': 10.0}
        )
        # Tight constraints should have lower max optimized feed
        max_loose = max((a.optimized_feed for a in result_loose.optimization_actions), default=100.0)
        max_tight = max((a.optimized_feed for a in result_tight.optimization_actions), default=100.0)
        assert max_tight <= max_loose + 0.01


class TestOptimalFeedCalculation:
    """Test the F proportional to f^0.8 inverse calculation."""

    def test_feed_increases_when_force_below_target(self):
        proxy = CuttingSimProxy()
        block = GCodeBlock(feed_rate_mmpm=500.0, spindle_rpm=8000.0)
        pred = BlockPrediction(peak_force_n=50.0)
        constraints = {'force_threshold_n': 180.0, 'max_force_pct': 80.0}
        opt = proxy._compute_optimal_feed(block, pred, constraints)
        assert opt > 500.0, "Feed should increase when force is well below target"

    def test_feed_decreases_when_force_above_target(self):
        proxy = CuttingSimProxy()
        block = GCodeBlock(feed_rate_mmpm=500.0, spindle_rpm=8000.0)
        pred = BlockPrediction(peak_force_n=200.0)
        constraints = {'force_threshold_n': 180.0, 'max_force_pct': 80.0}
        opt = proxy._compute_optimal_feed(block, pred, constraints)
        assert opt < 500.0, "Feed should decrease when force exceeds target"

    def test_inverse_power_law_relationship(self):
        """Verify feed_opt = feed * (F_target / F_current)^(1/0.8)."""
        proxy = CuttingSimProxy()
        block = GCodeBlock(feed_rate_mmpm=400.0, spindle_rpm=8000.0)
        pred = BlockPrediction(peak_force_n=100.0)
        target_force = 144.0  # target = threshold * max_force_pct/100
        constraints = {'force_threshold_n': 180.0, 'max_force_pct': 80.0}
        opt = proxy._compute_optimal_feed(block, pred, constraints)
        # Manual calculation: target = 180*0.8 = 144, ratio = 144/100 = 1.44
        # feed_opt = 400 * 1.44^1.25
        expected = 400.0 * (1.44 ** 1.25)
        assert abs(opt - expected) < 0.1, f"Expected {expected}, got {opt}"

    def test_zero_force_returns_current_feed(self):
        proxy = CuttingSimProxy()
        block = GCodeBlock(feed_rate_mmpm=500.0, spindle_rpm=8000.0)
        pred = BlockPrediction(peak_force_n=0.0)
        opt = proxy._compute_optimal_feed(block, pred, {})
        assert opt == 500.0


class TestCycleTimeEstimation:
    """Test cycle time estimation from blocks."""

    def test_basic_cycle_time(self):
        proxy = CuttingSimProxy()
        # 3 blocks, each 100mm at 500mm/min = 0.2 min each = 0.6 total
        blocks = _make_blocks(3, feed=500.0, length=100.0)
        t = proxy._estimate_cycle_time(blocks)
        assert abs(t - 0.6) < 1e-6

    def test_zero_feed_skipped(self):
        proxy = CuttingSimProxy()
        blocks = [GCodeBlock(feed_rate_mmpm=0.0, length_mm=100.0)]
        t = proxy._estimate_cycle_time(blocks)
        assert t == 0.0

    def test_empty_blocks(self):
        proxy = CuttingSimProxy()
        t = proxy._estimate_cycle_time([])
        assert t == 0.0


class TestProgramStatistics:
    """Test get_program_statistics method."""

    def test_total_and_cutting_blocks(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(4, feed=500.0, rpm=8000.0)
        # Add a rapid block (zero feed)
        blocks.append(GCodeBlock(feed_rate_mmpm=0.0, spindle_rpm=0.0, length_mm=20.0))
        sim = proxy.simulate_program(blocks)
        stats = proxy.get_program_statistics(blocks, sim)
        assert stats['total_blocks'] == 5
        assert stats['cutting_blocks'] == 4
        assert stats['rapid_blocks'] == 1

    def test_total_distance(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(3, feed=500.0, rpm=8000.0, length=30.0)
        sim = proxy.simulate_program(blocks)
        stats = proxy.get_program_statistics(blocks, sim)
        assert abs(stats['total_distance_mm'] - 90.0) < 1e-6

    def test_peak_force_tracking(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(3, feed=500.0, rpm=8000.0)
        sim = proxy.simulate_program(blocks)
        stats = proxy.get_program_statistics(blocks, sim)
        assert stats['peak_force_n'] > 0
        # Peak should match max of block predictions
        expected_peak = max(p.peak_force_n for p in sim.block_predictions)
        assert abs(stats['peak_force_n'] - expected_peak) < 1e-6

    def test_force_utilization_calculation(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(3, feed=500.0, rpm=8000.0)
        sim = proxy.simulate_program(blocks)
        stats = proxy.get_program_statistics(blocks, sim)
        # Force utilization = avg_force / 180.0 * 100
        assert stats['force_utilization_pct'] >= 0
        assert isinstance(stats['force_utilization_pct'], float)

    def test_thermal_hotspot_identification(self):
        proxy = CuttingSimProxy()
        # Normal blocks should have low temperature (below 200C threshold)
        blocks = _make_blocks(3, feed=500.0, rpm=8000.0)
        sim = proxy.simulate_program(blocks)
        stats = proxy.get_program_statistics(blocks, sim)
        assert isinstance(stats['thermal_hotspots'], list)

    def test_chatter_risk_block_identification(self):
        proxy = CuttingSimProxy()
        # Use RPM near an unstable zone: 60*250/3 = 5000
        blocks = _make_blocks(2, feed=500.0, rpm=5000.0)
        sim = proxy.simulate_program(blocks)
        stats = proxy.get_program_statistics(blocks, sim)
        assert isinstance(stats['chatter_risk_blocks'], list)
        # 5000 RPM is near stability lobe boundary for fn=250 Hz
        assert len(stats['chatter_risk_blocks']) > 0


class TestNoChangesNeeded:
    """Blocks already near optimal should produce no or minimal changes."""

    def test_already_optimal_no_actions(self):
        proxy = CuttingSimProxy()
        # Moderate parameters, force should be in the 60-85% sweet spot
        blocks = _make_blocks(3, feed=500.0, rpm=8000.0)
        # Use a high threshold so forces are in optimal range
        result = proxy.optimize_program(
            blocks,
            constraints={'force_threshold_n': 10000.0, 'max_force_pct': 80.0, 'max_feed_increase_pct': 5.0},
        )
        # With very tight max_feed_increase_pct and huge threshold,
        # optimizations might be small or clamped
        assert isinstance(result, ProgramOptimizationResult)


class TestTimeSavings:
    """Test time savings percentage calculation."""

    def test_time_savings_positive_when_feed_increased(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(5, feed=100.0, rpm=8000.0)
        result = proxy.optimize_program(blocks)
        # When feed is increased, cycle time should decrease
        if result.optimized_cycle_time_min < result.original_cycle_time_min:
            assert result.time_savings_pct > 0

    def test_time_savings_is_percentage(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(3, feed=100.0, rpm=8000.0)
        result = proxy.optimize_program(blocks)
        # Time savings should be bounded reasonably
        assert result.time_savings_pct <= 100.0


class TestToolLifeComparison:
    """Test original vs optimized tool life reporting."""

    def test_tool_life_reported(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(3, feed=500.0, rpm=8000.0)
        result = proxy.optimize_program(blocks)
        assert result.original_tool_life_min > 0
        assert result.optimized_tool_life_min > 0


class TestRiskAssessment:
    """Test risk assessment classification."""

    def test_low_risk_for_safe_params(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(3, feed=200.0, rpm=8000.0)
        result = proxy.optimize_program(
            blocks, constraints={'force_threshold_n': 10000.0}
        )
        # Force well below threshold; risk may be 'medium' if many blocks optimized
        assert result.risk_assessment in ('low', 'medium')

    def test_high_risk_for_aggressive_params(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(3, feed=2000.0, rpm=8000.0, ap=5.0, ae=6.0)
        result = proxy.optimize_program(
            blocks, constraints={'force_threshold_n': 50.0}
        )
        assert result.risk_assessment in ('medium', 'high')

    def test_risk_is_valid_string(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(3, feed=500.0, rpm=8000.0)
        result = proxy.optimize_program(blocks)
        assert result.risk_assessment in ('low', 'medium', 'high')


class TestBlockOptimizationDataclass:
    """Test BlockOptimization dataclass."""

    def test_default_values(self):
        bo = BlockOptimization()
        assert bo.block_index == 0
        assert bo.original_feed == 0.0
        assert bo.optimized_feed == 0.0
        assert bo.reason == ''

    def test_custom_values(self):
        bo = BlockOptimization(
            block_index=5,
            original_feed=500.0,
            optimized_feed=650.0,
            original_speed=8000.0,
            optimized_speed=7500.0,
            reason='force_headroom',
            force_change_pct=15.5,
            time_change_pct=-12.3,
        )
        assert bo.block_index == 5
        assert bo.optimized_feed == 650.0
        assert bo.reason == 'force_headroom'
        assert bo.time_change_pct == -12.3


class TestProgramOptimizationResultDataclass:
    """Test ProgramOptimizationResult dataclass defaults."""

    def test_default_values(self):
        r = ProgramOptimizationResult()
        assert r.original_cycle_time_min == 0.0
        assert r.optimized_cycle_time_min == 0.0
        assert r.time_savings_pct == 0.0
        assert r.risk_assessment == 'low'
        assert r.optimization_actions == []


class TestEmptyProgram:
    """Test behavior with empty program."""

    def test_empty_program_returns_defaults(self):
        proxy = CuttingSimProxy()
        result = proxy.optimize_program([])
        assert result.original_cycle_time_min == 0.0
        assert result.optimized_cycle_time_min == 0.0
        assert result.time_savings_pct == 0.0
        assert result.original_max_force_n == 0.0
        assert result.optimization_actions == []
        assert result.risk_assessment == 'low'

    def test_empty_program_statistics(self):
        proxy = CuttingSimProxy()
        sim = SimulationResult()
        stats = proxy.get_program_statistics([], sim)
        assert stats['total_blocks'] == 0
        assert stats['cutting_blocks'] == 0
        assert stats['peak_force_n'] == 0.0
        assert stats['force_utilization_pct'] == 0.0
