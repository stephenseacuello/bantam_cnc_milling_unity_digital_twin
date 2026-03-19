"""Tests for ConstantChipLoadOptimizer logic (Python mirror of C# implementation).

Validates adjusted feed calculation, engagement angle computation,
path optimization, time savings, safety clamping, and edge cases.
"""

import math
import pytest
from dataclasses import dataclass, field
from typing import List


# ---- Python mirror of C# types ----

@dataclass
class EngagementSegment:
    startIndex: int = 0
    endIndex: int = 0
    engagementAngleDeg: float = 0.0
    radialDepthMm: float = 0.0
    adjustedFeedMmMin: float = 0.0


@dataclass
class ChipLoadOptResult:
    segments: List[EngagementSegment] = field(default_factory=list)
    originalFeed: float = 0.0
    avgAdjustedFeed: float = 0.0
    maxFeedIncrease: float = 0.0
    minFeedDecrease: float = 0.0
    timeReductionPct: float = 0.0


# ---- Python mirror of ConstantChipLoadOptimizer ----

class ConstantChipLoadOptimizer:
    MAX_FEED_MULTIPLIER = 1.5

    def CalculateAdjustedFeed(self, baseFeed: float,
                               fullEngagementAngle: float,
                               currentEngagementAngle: float) -> float:
        if baseFeed <= 0.0:
            raise ValueError("Base feed must be positive.")
        if fullEngagementAngle <= 0.0:
            raise ValueError("Full engagement angle must be positive.")
        if currentEngagementAngle <= 0.0:
            raise ValueError("Current engagement angle must be positive.")

        ratio = fullEngagementAngle / currentEngagementAngle
        adjusted = baseFeed * math.sqrt(ratio)

        max_feed = baseFeed * self.MAX_FEED_MULTIPLIER
        return min(adjusted, max_feed)

    def CalculateEngagement(self, toolDiameter: float,
                             radialDepth: float) -> float:
        if toolDiameter <= 0.0:
            raise ValueError("Tool diameter must be positive.")
        if radialDepth < 0.0:
            raise ValueError("Radial depth cannot be negative.")

        radius = toolDiameter / 2.0
        if radialDepth > toolDiameter:
            radialDepth = toolDiameter

        cos_val = 1.0 - radialDepth / radius
        cos_val = max(-1.0, min(1.0, cos_val))
        return math.degrees(math.acos(cos_val))

    def OptimizePath(self, radialDepths: List[float],
                      baseFeed: float,
                      toolDiameter: float) -> ChipLoadOptResult:
        if not radialDepths:
            raise ValueError("Path must contain at least one point.")
        if baseFeed <= 0.0:
            raise ValueError("Base feed must be positive.")
        if toolDiameter <= 0.0:
            raise ValueError("Tool diameter must be positive.")

        full_engagement = self.CalculateEngagement(toolDiameter, toolDiameter)

        segments: List[EngagementSegment] = []
        seg_start = 0
        prev_depth = radialDepths[0]

        for i in range(1, len(radialDepths) + 1):
            changed = (i == len(radialDepths)) or \
                      (abs(radialDepths[i] - prev_depth) > 1e-6)

            if changed:
                depth = prev_depth
                angle = self.CalculateEngagement(toolDiameter, depth)
                adj_feed = (self.CalculateAdjustedFeed(baseFeed, full_engagement, angle)
                            if angle > 0.0 else baseFeed)
                segments.append(EngagementSegment(
                    startIndex=seg_start,
                    endIndex=i - 1,
                    engagementAngleDeg=angle,
                    radialDepthMm=depth,
                    adjustedFeedMmMin=adj_feed,
                ))
                if i < len(radialDepths):
                    seg_start = i
                    prev_depth = radialDepths[i]

        result = ChipLoadOptResult(
            segments=segments,
            originalFeed=baseFeed,
        )

        if not segments:
            return result

        sum_feed = sum(s.adjustedFeedMmMin for s in segments)
        max_ratio = max(s.adjustedFeedMmMin / baseFeed for s in segments)
        min_ratio = min(s.adjustedFeedMmMin / baseFeed for s in segments)

        result.avgAdjustedFeed = sum_feed / len(segments)
        result.maxFeedIncrease = max_ratio
        result.minFeedDecrease = min_ratio
        result.timeReductionPct = self.EstimateTimeSavings(baseFeed, result.avgAdjustedFeed)

        return result

    def EstimateTimeSavings(self, originalFeed: float,
                             optimizedFeed: float) -> float:
        if originalFeed <= 0.0 or optimizedFeed <= 0.0:
            return 0.0
        if optimizedFeed <= originalFeed:
            return 0.0
        saving = (1.0 - originalFeed / optimizedFeed) * 100.0
        return max(0.0, min(100.0, saving))


# ---- Fixtures ----

@pytest.fixture
def opt():
    return ConstantChipLoadOptimizer()


# ---- Tests ----

class TestCalculateAdjustedFeed:
    """Tests for CalculateAdjustedFeed."""

    def test_same_angle_returns_base_feed(self, opt):
        """When current engagement equals full engagement, feed is unchanged."""
        result = opt.CalculateAdjustedFeed(1000.0, 180.0, 180.0)
        assert result == pytest.approx(1000.0, rel=1e-4)

    def test_half_angle_increases_feed(self, opt):
        """Half the engagement angle -> feed * sqrt(2) ~= 1414, but clamped to 1500."""
        result = opt.CalculateAdjustedFeed(1000.0, 180.0, 90.0)
        expected = min(1000.0 * math.sqrt(2), 1500.0)
        assert result == pytest.approx(expected, rel=1e-4)

    def test_feed_clamped_at_150_percent(self, opt):
        """Very small engagement should clamp adjusted feed to 150% of base."""
        result = opt.CalculateAdjustedFeed(1000.0, 180.0, 10.0)
        assert result == pytest.approx(1500.0, rel=1e-4)

    def test_invalid_base_feed_raises(self, opt):
        """Zero or negative base feed raises ValueError."""
        with pytest.raises(ValueError):
            opt.CalculateAdjustedFeed(0.0, 180.0, 90.0)
        with pytest.raises(ValueError):
            opt.CalculateAdjustedFeed(-100.0, 180.0, 90.0)


class TestCalculateEngagement:
    """Tests for CalculateEngagement."""

    def test_full_diameter_gives_180_degrees(self, opt):
        """Radial depth == tool diameter -> 180 degree engagement."""
        angle = opt.CalculateEngagement(10.0, 10.0)
        assert angle == pytest.approx(180.0, rel=1e-4)

    def test_half_diameter_gives_90_degrees(self, opt):
        """Radial depth == radius -> 90 degrees (acos(0) = 90)."""
        angle = opt.CalculateEngagement(10.0, 5.0)
        assert angle == pytest.approx(90.0, rel=1e-4)

    def test_zero_depth_gives_zero(self, opt):
        """Zero radial depth -> 0 degree engagement."""
        angle = opt.CalculateEngagement(10.0, 0.0)
        assert angle == pytest.approx(0.0, abs=1e-4)

    def test_negative_diameter_raises(self, opt):
        """Negative tool diameter raises ValueError."""
        with pytest.raises(ValueError):
            opt.CalculateEngagement(-5.0, 2.0)


class TestOptimizePath:
    """Tests for OptimizePath."""

    def test_uniform_depth_single_segment(self, opt):
        """Constant radial depth produces a single segment."""
        depths = [5.0, 5.0, 5.0, 5.0]
        result = opt.OptimizePath(depths, 1000.0, 10.0)
        assert len(result.segments) == 1
        assert result.segments[0].startIndex == 0
        assert result.segments[0].endIndex == 3

    def test_varying_depth_multiple_segments(self, opt):
        """Changing radial depth creates multiple segments."""
        depths = [5.0, 5.0, 2.5, 2.5, 10.0]
        result = opt.OptimizePath(depths, 1000.0, 10.0)
        assert len(result.segments) == 3
        # lighter engagement (2.5 mm) should have higher adjusted feed
        seg_light = result.segments[1]  # depth 2.5 mm
        seg_full = result.segments[2]   # depth 10.0 mm (full)
        assert seg_light.adjustedFeedMmMin >= seg_full.adjustedFeedMmMin

    def test_max_feed_increase_capped(self, opt):
        """maxFeedIncrease ratio should never exceed 1.5."""
        depths = [1.0, 1.0, 10.0, 10.0]
        result = opt.OptimizePath(depths, 1000.0, 10.0)
        assert result.maxFeedIncrease <= 1.5 + 1e-6

    def test_empty_path_raises(self, opt):
        """Empty path raises ValueError."""
        with pytest.raises(ValueError):
            opt.OptimizePath([], 1000.0, 10.0)


class TestEstimateTimeSavings:
    """Tests for EstimateTimeSavings."""

    def test_same_feed_zero_savings(self, opt):
        """No speed increase -> 0% savings."""
        assert opt.EstimateTimeSavings(1000.0, 1000.0) == pytest.approx(0.0)

    def test_double_feed_50pct_savings(self, opt):
        """Doubling feed -> 50% time savings."""
        assert opt.EstimateTimeSavings(1000.0, 2000.0) == pytest.approx(50.0, rel=1e-4)

    def test_slower_feed_zero_savings(self, opt):
        """Slower optimized feed returns 0% (no negative savings)."""
        assert opt.EstimateTimeSavings(1000.0, 500.0) == pytest.approx(0.0)

    def test_zero_feeds_return_zero(self, opt):
        """Zero or negative feeds produce 0% savings."""
        assert opt.EstimateTimeSavings(0.0, 1000.0) == pytest.approx(0.0)
        assert opt.EstimateTimeSavings(1000.0, 0.0) == pytest.approx(0.0)
