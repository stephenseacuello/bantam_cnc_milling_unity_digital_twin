"""Tests for PocketFeatureRecognizer logic (Python mirror of C# implementation).

Validates rectangular pocket recognition, circular pocket recognition,
pocket classification, strategy recommendation, machining time estimation,
optimal tool diameter suggestion, and full analysis workflow.
"""

import math
import pytest
from dataclasses import dataclass, field
from typing import List


# ---- Python mirror of C# types ----

@dataclass
class PocketFeature:
    pocketId: str = ""
    pocketType: str = ""            # rectangular, circular, oblong, irregular
    width: float = 0.0              # mm
    length: float = 0.0             # mm
    depth: float = 0.0              # mm
    cornerRadius: float = 0.0       # mm
    area: float = 0.0               # mm^2 (floor area)
    volume: float = 0.0             # mm^3
    islandCount: int = 0


@dataclass
class PocketAnalysis:
    pockets: List[PocketFeature] = field(default_factory=list)
    totalVolume: float = 0.0        # mm^3
    deepestPocket: float = 0.0      # mm
    recommendedStrategy: str = ""   # spiral, zigzag, contour, plunge


# ---- Python mirror of PocketFeatureRecognizer ----

class PocketFeatureRecognizer:

    def __init__(self):
        self._next_id = 1

    def _next_pocket_id(self) -> str:
        pid = f"PKT-{self._next_id:04d}"
        self._next_id += 1
        return pid

    # ── Recognition helpers ──────────────────────────────────────

    def RecognizeRectangular(self, width: float, length: float,
                             depth: float, cornerRadius: float) -> PocketFeature:
        """Recognize a rectangular pocket from dimensions and corner radius."""
        if width <= 0.0 or length <= 0.0 or depth <= 0.0:
            return None

        cr = max(cornerRadius, 0.0)
        # Floor area: rect minus corner squares plus quarter-circle areas
        rect_area = width * length - 4.0 * cr * cr + math.pi * cr * cr
        vol = rect_area * depth

        return PocketFeature(
            pocketId=self._next_pocket_id(),
            pocketType="rectangular",
            width=width,
            length=length,
            depth=depth,
            cornerRadius=cr,
            area=rect_area,
            volume=vol,
            islandCount=0,
        )

    def RecognizeCircular(self, diameter: float, depth: float) -> PocketFeature:
        """Recognize a circular pocket from diameter and depth."""
        if diameter <= 0.0 or depth <= 0.0:
            return None

        radius = diameter / 2.0
        circ_area = math.pi * radius * radius
        vol = circ_area * depth

        return PocketFeature(
            pocketId=self._next_pocket_id(),
            pocketType="circular",
            width=diameter,
            length=diameter,
            depth=depth,
            cornerRadius=radius,
            area=circ_area,
            volume=vol,
            islandCount=0,
        )

    # ── Classification ───────────────────────────────────────────

    def ClassifyPocket(self, width: float, length: float, depth: float) -> str:
        """Classify pocket as shallow, deep, narrow, or standard."""
        if width <= 0.0 or length <= 0.0 or depth <= 0.0:
            return "invalid"

        min_dim = min(width, length)
        max_dim = max(width, length)
        depth_ratio = depth / min_dim
        slot_ratio = min_dim / max_dim

        if depth_ratio > 3.0:
            return "deep"
        if slot_ratio < 0.25:
            return "narrow"
        if depth_ratio < 0.25:
            return "shallow"

        return "standard"

    # ── Strategy recommendation ──────────────────────────────────

    def RecommendStrategy(self, pocket: PocketFeature) -> str:
        """Suggest machining strategy for the given pocket."""
        if pocket is None:
            return "unknown"

        classification = self.ClassifyPocket(pocket.width, pocket.length, pocket.depth)

        if classification == "deep":
            return "plunge"
        if classification == "narrow":
            return "zigzag"

        if pocket.pocketType == "circular":
            return "spiral"

        if pocket.islandCount > 0:
            return "contour"

        return "spiral"

    # ── Machining time estimation ────────────────────────────────

    def EstimateMachiningTime(self, pocket: PocketFeature, feedRate: float,
                              stepover: float, toolDia: float) -> float:
        """Rough estimate of machining time in minutes."""
        if pocket is None:
            return 0.0
        if feedRate <= 0.0 or stepover <= 0.0 or toolDia <= 0.0:
            return 0.0

        depth_per_pass = toolDia
        z_passes = max(1, math.ceil(pocket.depth / depth_per_pass))

        effective_width = pocket.width
        lateral_passes = max(1, math.ceil(effective_width / stepover))

        path_per_layer = lateral_passes * pocket.length
        total_path = path_per_layer * z_passes

        return total_path / feedRate

    # ── Optimal tool diameter ────────────────────────────────────

    def GetOptimalToolDiameter(self, pocket: PocketFeature) -> float:
        """Suggest optimal tool diameter based on pocket geometry."""
        if pocket is None:
            return 0.0

        min_dim = min(pocket.width, pocket.length)
        dim_limit = min_dim * 0.8

        if pocket.cornerRadius > 0.0:
            corner_limit = pocket.cornerRadius * 2.0
            return min(dim_limit, corner_limit)

        return dim_limit

    # ── Full analysis helper ─────────────────────────────────────

    def Analyze(self, pockets: List[PocketFeature]) -> PocketAnalysis:
        """Build a PocketAnalysis from a list of pocket features."""
        analysis = PocketAnalysis()

        if not pockets:
            return analysis

        analysis.pockets = list(pockets)
        total_vol = 0.0
        max_depth = 0.0

        for p in pockets:
            total_vol += p.volume
            if p.depth > max_depth:
                max_depth = p.depth

        analysis.totalVolume = total_vol
        analysis.deepestPocket = max_depth
        analysis.recommendedStrategy = self.RecommendStrategy(pockets[0])

        return analysis


# ---- Fixtures ----

@pytest.fixture
def recognizer():
    return PocketFeatureRecognizer()


# ---- Tests ----

class TestRecognizeRectangular:
    """Rectangular pocket recognition."""

    def test_basic_rectangular_pocket(self, recognizer):
        """A valid rectangular pocket should be created with correct type and ID."""
        pocket = recognizer.RecognizeRectangular(50.0, 30.0, 10.0, 3.0)
        assert pocket is not None
        assert pocket.pocketType == "rectangular"
        assert pocket.pocketId.startswith("PKT-")
        assert pocket.width == 50.0
        assert pocket.length == 30.0
        assert pocket.depth == 10.0
        assert pocket.cornerRadius == 3.0

    def test_rectangular_area_and_volume(self, recognizer):
        """Area and volume should account for corner radius adjustments."""
        w, l, d, cr = 40.0, 20.0, 5.0, 2.0
        pocket = recognizer.RecognizeRectangular(w, l, d, cr)
        expected_area = w * l - 4.0 * cr * cr + math.pi * cr * cr
        expected_vol = expected_area * d
        assert pocket.area == pytest.approx(expected_area, rel=1e-6)
        assert pocket.volume == pytest.approx(expected_vol, rel=1e-6)

    def test_rectangular_zero_corner_radius(self, recognizer):
        """With zero corner radius, area should be simple width * length."""
        pocket = recognizer.RecognizeRectangular(50.0, 30.0, 10.0, 0.0)
        assert pocket.area == pytest.approx(50.0 * 30.0, rel=1e-6)
        assert pocket.volume == pytest.approx(50.0 * 30.0 * 10.0, rel=1e-6)

    def test_rectangular_invalid_returns_none(self, recognizer):
        """Invalid dimensions should return None."""
        assert recognizer.RecognizeRectangular(0.0, 30.0, 10.0, 3.0) is None
        assert recognizer.RecognizeRectangular(50.0, -1.0, 10.0, 3.0) is None
        assert recognizer.RecognizeRectangular(50.0, 30.0, 0.0, 3.0) is None


class TestRecognizeCircular:
    """Circular pocket recognition."""

    def test_basic_circular_pocket(self, recognizer):
        """A valid circular pocket should have correct type and dimensions."""
        pocket = recognizer.RecognizeCircular(20.0, 8.0)
        assert pocket is not None
        assert pocket.pocketType == "circular"
        assert pocket.width == 20.0
        assert pocket.length == 20.0
        assert pocket.depth == 8.0
        assert pocket.cornerRadius == pytest.approx(10.0)

    def test_circular_area_and_volume(self, recognizer):
        """Area and volume should match pi*r^2 and pi*r^2*d."""
        diameter, depth = 30.0, 12.0
        pocket = recognizer.RecognizeCircular(diameter, depth)
        r = diameter / 2.0
        expected_area = math.pi * r * r
        expected_vol = expected_area * depth
        assert pocket.area == pytest.approx(expected_area, rel=1e-6)
        assert pocket.volume == pytest.approx(expected_vol, rel=1e-6)

    def test_circular_invalid_returns_none(self, recognizer):
        """Invalid diameter or depth should return None."""
        assert recognizer.RecognizeCircular(0.0, 10.0) is None
        assert recognizer.RecognizeCircular(20.0, -5.0) is None


class TestClassifyPocket:
    """Pocket classification based on aspect ratios."""

    def test_classify_deep(self, recognizer):
        """Depth / min(w, l) > 3 classifies as deep."""
        assert recognizer.ClassifyPocket(10.0, 10.0, 40.0) == "deep"

    def test_classify_shallow(self, recognizer):
        """Depth / min(w, l) < 0.25 classifies as shallow."""
        assert recognizer.ClassifyPocket(40.0, 40.0, 5.0) == "shallow"

    def test_classify_narrow(self, recognizer):
        """min(w, l) / max(w, l) < 0.25 classifies as narrow."""
        assert recognizer.ClassifyPocket(5.0, 100.0, 10.0) == "narrow"

    def test_classify_standard(self, recognizer):
        """A pocket that fits no special criteria is standard."""
        assert recognizer.ClassifyPocket(30.0, 40.0, 15.0) == "standard"

    def test_classify_invalid(self, recognizer):
        """Invalid dimensions return 'invalid'."""
        assert recognizer.ClassifyPocket(0.0, 10.0, 10.0) == "invalid"


class TestRecommendStrategy:
    """Strategy recommendation for pockets."""

    def test_deep_pocket_gets_plunge(self, recognizer):
        """Deep pockets should use plunge milling."""
        pocket = PocketFeature(width=10.0, length=10.0, depth=50.0,
                               pocketType="rectangular")
        assert recognizer.RecommendStrategy(pocket) == "plunge"

    def test_circular_pocket_gets_spiral(self, recognizer):
        """Standard circular pockets should use spiral strategy."""
        pocket = recognizer.RecognizeCircular(30.0, 5.0)
        assert recognizer.RecommendStrategy(pocket) == "spiral"

    def test_narrow_pocket_gets_zigzag(self, recognizer):
        """Narrow pockets should use zigzag strategy."""
        pocket = PocketFeature(width=5.0, length=100.0, depth=10.0,
                               pocketType="rectangular")
        assert recognizer.RecommendStrategy(pocket) == "zigzag"

    def test_island_pocket_gets_contour(self, recognizer):
        """A standard pocket with islands should use contour strategy."""
        pocket = PocketFeature(width=50.0, length=50.0, depth=10.0,
                               pocketType="rectangular", islandCount=2)
        assert recognizer.RecommendStrategy(pocket) == "contour"

    def test_none_pocket_returns_unknown(self, recognizer):
        """None pocket should return 'unknown'."""
        assert recognizer.RecommendStrategy(None) == "unknown"


class TestEstimateMachiningTime:
    """Machining time estimation."""

    def test_positive_time_for_valid_pocket(self, recognizer):
        """Valid pocket and parameters should produce positive time."""
        pocket = recognizer.RecognizeRectangular(40.0, 20.0, 10.0, 2.0)
        time = recognizer.EstimateMachiningTime(pocket, feedRate=500.0,
                                                stepover=5.0, toolDia=10.0)
        assert time > 0.0

    def test_time_calculation_correctness(self, recognizer):
        """Verify the exact time calculation formula."""
        pocket = PocketFeature(width=20.0, length=50.0, depth=10.0)
        # toolDia=10 => depth_per_pass=10, z_passes=ceil(10/10)=1
        # lateral_passes=ceil(20/5)=4, path_per_layer=4*50=200
        # total_path=200*1=200, time=200/1000=0.2
        time = recognizer.EstimateMachiningTime(pocket, feedRate=1000.0,
                                                stepover=5.0, toolDia=10.0)
        assert time == pytest.approx(0.2, rel=1e-6)

    def test_time_scales_with_depth(self, recognizer):
        """Doubling depth (requiring more Z-passes) should increase time."""
        pocket_shallow = PocketFeature(width=20.0, length=50.0, depth=5.0)
        pocket_deep = PocketFeature(width=20.0, length=50.0, depth=15.0)
        t_shallow = recognizer.EstimateMachiningTime(pocket_shallow, 500.0, 5.0, 10.0)
        t_deep = recognizer.EstimateMachiningTime(pocket_deep, 500.0, 5.0, 10.0)
        assert t_deep > t_shallow

    def test_time_zero_on_invalid(self, recognizer):
        """Invalid parameters should return zero time."""
        pocket = recognizer.RecognizeRectangular(20.0, 20.0, 10.0, 1.0)
        assert recognizer.EstimateMachiningTime(pocket, 0.0, 5.0, 10.0) == 0.0
        assert recognizer.EstimateMachiningTime(None, 500.0, 5.0, 10.0) == 0.0


class TestGetOptimalToolDiameter:
    """Optimal tool diameter suggestion."""

    def test_tool_limited_by_dimension(self, recognizer):
        """Tool should be 80% of narrowest dimension when no corner radius."""
        pocket = PocketFeature(width=20.0, length=40.0, depth=10.0, cornerRadius=0.0)
        tool = recognizer.GetOptimalToolDiameter(pocket)
        assert tool == pytest.approx(20.0 * 0.8, rel=1e-6)

    def test_tool_limited_by_corner_radius(self, recognizer):
        """When corner radius is small, tool is limited to 2 * cornerRadius."""
        pocket = PocketFeature(width=50.0, length=50.0, depth=10.0, cornerRadius=3.0)
        tool = recognizer.GetOptimalToolDiameter(pocket)
        # dim_limit = 50*0.8 = 40, corner_limit = 3*2 = 6 => min(40, 6) = 6
        assert tool == pytest.approx(6.0, rel=1e-6)

    def test_tool_zero_for_none_pocket(self, recognizer):
        """None pocket should return zero."""
        assert recognizer.GetOptimalToolDiameter(None) == 0.0


class TestAnalyze:
    """Full analysis workflow."""

    def test_analysis_aggregates_volumes(self, recognizer):
        """Analyze should sum volumes and find deepest pocket."""
        p1 = recognizer.RecognizeRectangular(30.0, 20.0, 5.0, 1.0)
        p2 = recognizer.RecognizeCircular(25.0, 12.0)
        analysis = recognizer.Analyze([p1, p2])
        assert analysis.totalVolume == pytest.approx(p1.volume + p2.volume, rel=1e-6)
        assert analysis.deepestPocket == pytest.approx(12.0)
        assert len(analysis.pockets) == 2
        assert analysis.recommendedStrategy != ""

    def test_analysis_empty_list(self, recognizer):
        """Empty list should produce empty analysis."""
        analysis = recognizer.Analyze([])
        assert analysis.totalVolume == 0.0
        assert analysis.deepestPocket == 0.0
        assert len(analysis.pockets) == 0


class TestUniqueIds:
    """Pocket ID generation."""

    def test_ids_are_unique(self, recognizer):
        """Each pocket should receive a unique ID."""
        p1 = recognizer.RecognizeRectangular(20.0, 20.0, 5.0, 1.0)
        p2 = recognizer.RecognizeRectangular(30.0, 30.0, 5.0, 1.0)
        p3 = recognizer.RecognizeCircular(15.0, 8.0)
        ids = {p1.pocketId, p2.pocketId, p3.pocketId}
        assert len(ids) == 3
