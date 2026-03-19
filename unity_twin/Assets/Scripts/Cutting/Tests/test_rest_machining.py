"""Tests for RestMachiningDetector logic (Python mirror of C# implementation).

Validates rest-volume calculation, corner analysis, fillet analysis,
cleanup tool suggestion, cleanup time estimation, and edge cases.
"""

import math
import pytest
from dataclasses import dataclass, field
from typing import List, Tuple


# ---- Python mirror of C# types ----

@dataclass
class RestRegion:
    center: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    extentMm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    volumeMm3: float = 0.0
    cornerRadius: float = 0.0
    requiredToolDiaMm: float = 0.0
    depth: float = 0.0


@dataclass
class RestAnalysis:
    regions: List[RestRegion] = field(default_factory=list)
    totalRestVolumeMm3: float = 0.0
    largestRegionVolumeMm3: float = 0.0
    suggestedToolDiaMm: float = 0.0
    estimatedCleanupTimeMin: float = 0.0
    regionCount: int = 0


# ---- Python mirror of RestMachiningDetector ----

class RestMachiningDetector:

    # ── Corner rest-volume calculation ─────────────────────────────

    def GetRestVolume(self, toolDia: float, cornerRadius: float, depth: float) -> float:
        """Volume of rest material left in a single internal corner."""
        if toolDia <= 0.0 or depth <= 0.0:
            return 0.0

        toolRadius = toolDia / 2.0
        if toolRadius <= cornerRadius:
            return 0.0

        factor = 1.0 - math.pi / 4.0
        area = factor * (toolRadius ** 2 - cornerRadius ** 2)
        return area * depth

    # ── Analyse internal corners of a rectangular pocket ──────────

    def AnalyzeCorners(self, pocketWidth: float, pocketLength: float,
                       pocketDepth: float, toolDiameter: float) -> RestAnalysis:
        analysis = RestAnalysis()

        if pocketWidth <= 0.0 or pocketLength <= 0.0 or \
           pocketDepth <= 0.0 or toolDiameter <= 0.0:
            return analysis

        toolRadius = toolDiameter / 2.0
        cornerRadius = 0.0  # sharp internal corners

        vol = self.GetRestVolume(toolDiameter, cornerRadius, pocketDepth)
        if vol <= 0.0:
            return analysis

        corners = [
            (0.0, 0.0, 0.0),
            (pocketWidth, 0.0, 0.0),
            (pocketWidth, pocketLength, 0.0),
            (0.0, pocketLength, 0.0),
        ]

        for c in corners:
            region = RestRegion(
                center=c,
                extentMm=(toolRadius, toolRadius, pocketDepth),
                volumeMm3=vol,
                cornerRadius=toolRadius,
                requiredToolDiaMm=toolRadius,
                depth=pocketDepth,
            )
            analysis.regions.append(region)

        analysis.regionCount = len(analysis.regions)
        analysis.totalRestVolumeMm3 = vol * analysis.regionCount
        analysis.largestRegionVolumeMm3 = vol
        analysis.suggestedToolDiaMm = self.SuggestCleanupTool(toolRadius)
        analysis.estimatedCleanupTimeMin = 0.0

        return analysis

    # ── Analyse fillet regions ────────────────────────────────────

    def AnalyzeFillets(self, filletRadius: float, toolDiameter: float,
                       pathLength: float) -> RestAnalysis:
        analysis = RestAnalysis()

        if filletRadius <= 0.0 or toolDiameter <= 0.0 or pathLength <= 0.0:
            return analysis

        toolRadius = toolDiameter / 2.0
        if toolRadius <= filletRadius:
            return analysis  # tool already fits

        factor = 1.0 - math.pi / 4.0
        restArea = factor * (toolRadius ** 2 - filletRadius ** 2)
        vol = restArea * pathLength

        region = RestRegion(
            center=(pathLength / 2.0, 0.0, 0.0),
            extentMm=(pathLength, toolRadius - filletRadius, 0.0),
            volumeMm3=vol,
            cornerRadius=toolRadius,
            requiredToolDiaMm=filletRadius * 2.0,
            depth=pathLength,
        )

        analysis.regions.append(region)
        analysis.regionCount = 1
        analysis.totalRestVolumeMm3 = vol
        analysis.largestRegionVolumeMm3 = vol
        analysis.suggestedToolDiaMm = self.SuggestCleanupTool(filletRadius)
        analysis.estimatedCleanupTimeMin = 0.0

        return analysis

    # ── Tool recommendation ──────────────────────────────────────

    def SuggestCleanupTool(self, maxRestCornerRadius: float) -> float:
        if maxRestCornerRadius <= 0.0:
            return 0.0
        return 2.0 * maxRestCornerRadius * 0.8

    # ── Cleanup time estimation ──────────────────────────────────

    def EstimateCleanupTime(self, regions: List[RestRegion],
                            feedRateMmPerMin: float,
                            stepoverMm: float) -> float:
        if not regions:
            return 0.0
        if feedRateMmPerMin <= 0.0 or stepoverMm <= 0.0:
            return 0.0

        totalTime = 0.0
        for region in regions:
            width = max(region.extentMm[0], region.extentMm[1])
            passCount = math.ceil(width / stepoverMm)
            passLength = region.depth
            pathLength = passCount * passLength
            totalTime += pathLength / feedRateMmPerMin

        return totalTime


# ---- Fixtures ----

@pytest.fixture
def detector():
    return RestMachiningDetector()


# ---- Tests ----

class TestGetRestVolume:
    """Rest volume calculation for a single corner."""

    def test_positive_rest_volume_sharp_corner(self, detector):
        """A tool in a sharp corner (cornerRadius=0) should leave rest material."""
        vol = detector.GetRestVolume(toolDia=10.0, cornerRadius=0.0, depth=20.0)
        # Expected: (1 - pi/4) * (5^2 - 0^2) * 20 = (1 - 0.7854) * 25 * 20
        expected = (1.0 - math.pi / 4.0) * 25.0 * 20.0
        assert vol == pytest.approx(expected, rel=1e-6)
        assert vol > 0.0

    def test_zero_rest_when_tool_fits(self, detector):
        """If the tool radius equals the corner radius, rest volume is zero."""
        vol = detector.GetRestVolume(toolDia=10.0, cornerRadius=5.0, depth=20.0)
        assert vol == pytest.approx(0.0)

    def test_zero_rest_when_tool_smaller(self, detector):
        """If the tool radius is smaller than the corner radius, no rest."""
        vol = detector.GetRestVolume(toolDia=6.0, cornerRadius=5.0, depth=20.0)
        assert vol == pytest.approx(0.0)

    def test_zero_on_invalid_inputs(self, detector):
        """Zero or negative tool diameter or depth should return 0."""
        assert detector.GetRestVolume(0.0, 0.0, 10.0) == 0.0
        assert detector.GetRestVolume(-5.0, 0.0, 10.0) == 0.0
        assert detector.GetRestVolume(10.0, 0.0, 0.0) == 0.0
        assert detector.GetRestVolume(10.0, 0.0, -1.0) == 0.0


class TestAnalyzeCorners:
    """Corner analysis for rectangular pockets."""

    def test_four_corners_detected(self, detector):
        """A rectangular pocket should produce exactly 4 rest regions."""
        result = detector.AnalyzeCorners(
            pocketWidth=50.0, pocketLength=30.0,
            pocketDepth=10.0, toolDiameter=12.0)
        assert result.regionCount == 4
        assert len(result.regions) == 4

    def test_total_volume_is_four_times_single(self, detector):
        """Total rest volume should be 4 x the single-corner volume."""
        tool_dia = 12.0
        depth = 10.0
        single_vol = detector.GetRestVolume(tool_dia, 0.0, depth)

        result = detector.AnalyzeCorners(
            pocketWidth=50.0, pocketLength=30.0,
            pocketDepth=depth, toolDiameter=tool_dia)
        assert result.totalRestVolumeMm3 == pytest.approx(4.0 * single_vol, rel=1e-6)

    def test_invalid_pocket_returns_empty(self, detector):
        """Zero or negative pocket dimensions should return empty analysis."""
        result = detector.AnalyzeCorners(0.0, 30.0, 10.0, 12.0)
        assert result.regionCount == 0


class TestAnalyzeFillets:
    """Fillet analysis for rest material detection."""

    def test_fillet_rest_detected_when_tool_too_large(self, detector):
        """When tool radius > fillet radius, rest material is detected."""
        result = detector.AnalyzeFillets(
            filletRadius=2.0, toolDiameter=10.0, pathLength=100.0)
        assert result.regionCount == 1
        assert result.totalRestVolumeMm3 > 0.0

    def test_no_fillet_rest_when_tool_fits(self, detector):
        """When tool radius <= fillet radius, no rest material."""
        result = detector.AnalyzeFillets(
            filletRadius=6.0, toolDiameter=10.0, pathLength=100.0)
        assert result.regionCount == 0
        assert result.totalRestVolumeMm3 == 0.0

    def test_fillet_volume_scales_with_path_length(self, detector):
        """Doubling path length should double the rest volume."""
        r1 = detector.AnalyzeFillets(filletRadius=2.0, toolDiameter=10.0, pathLength=50.0)
        r2 = detector.AnalyzeFillets(filletRadius=2.0, toolDiameter=10.0, pathLength=100.0)
        assert r2.totalRestVolumeMm3 == pytest.approx(
            2.0 * r1.totalRestVolumeMm3, rel=1e-6)


class TestSuggestCleanupTool:
    """Cleanup tool recommendation."""

    def test_suggested_tool_less_than_twice_radius(self, detector):
        """Suggested tool diameter must be < 2 * corner radius."""
        corner_radius = 5.0
        suggested = detector.SuggestCleanupTool(corner_radius)
        assert suggested < 2.0 * corner_radius
        assert suggested > 0.0

    def test_suggested_tool_is_80_percent(self, detector):
        """Suggested tool should be 80% of the max possible diameter."""
        corner_radius = 5.0
        expected = 2.0 * corner_radius * 0.8
        assert detector.SuggestCleanupTool(corner_radius) == pytest.approx(expected)

    def test_zero_radius_returns_zero(self, detector):
        """Zero corner radius should return zero."""
        assert detector.SuggestCleanupTool(0.0) == 0.0


class TestEstimateCleanupTime:
    """Cleanup time estimation."""

    def test_positive_time_for_valid_regions(self, detector):
        """Valid regions with valid feed and stepover produce positive time."""
        regions = [
            RestRegion(
                center=(0.0, 0.0, 0.0),
                extentMm=(5.0, 5.0, 10.0),
                volumeMm3=100.0,
                cornerRadius=5.0,
                requiredToolDiaMm=4.0,
                depth=10.0,
            )
        ]
        time = detector.EstimateCleanupTime(
            regions, feedRateMmPerMin=500.0, stepoverMm=1.0)
        assert time > 0.0

    def test_empty_regions_return_zero(self, detector):
        """Empty region list should return zero time."""
        assert detector.EstimateCleanupTime([], 500.0, 1.0) == 0.0

    def test_zero_feed_returns_zero(self, detector):
        """Zero feed rate should return zero time (guard clause)."""
        regions = [RestRegion(depth=10.0, extentMm=(5.0, 5.0, 10.0))]
        assert detector.EstimateCleanupTime(regions, 0.0, 1.0) == 0.0

    def test_time_calculation_correctness(self, detector):
        """Verify the time calculation formula: passes * depth / feedRate."""
        region = RestRegion(
            extentMm=(10.0, 5.0, 0.0),
            depth=20.0,
        )
        # width = max(10, 5) = 10, passCount = ceil(10 / 2) = 5
        # pathLength = 5 * 20 = 100, time = 100 / 1000 = 0.1
        time = detector.EstimateCleanupTime(
            [region], feedRateMmPerMin=1000.0, stepoverMm=2.0)
        assert time == pytest.approx(0.1, rel=1e-6)


class TestIntegration:
    """End-to-end integration tests."""

    def test_corner_analysis_suggests_smaller_tool(self, detector):
        """Corner analysis should suggest a tool smaller than the original."""
        original_dia = 20.0
        result = detector.AnalyzeCorners(
            pocketWidth=100.0, pocketLength=80.0,
            pocketDepth=15.0, toolDiameter=original_dia)
        assert result.suggestedToolDiaMm < original_dia
        assert result.suggestedToolDiaMm > 0.0

    def test_full_workflow_corners(self, detector):
        """Full workflow: analyze corners, then estimate cleanup time."""
        result = detector.AnalyzeCorners(
            pocketWidth=60.0, pocketLength=40.0,
            pocketDepth=12.0, toolDiameter=16.0)

        assert result.regionCount == 4
        assert result.totalRestVolumeMm3 > 0.0

        # Now estimate cleanup time
        time = detector.EstimateCleanupTime(
            result.regions, feedRateMmPerMin=800.0, stepoverMm=1.5)
        assert time > 0.0
