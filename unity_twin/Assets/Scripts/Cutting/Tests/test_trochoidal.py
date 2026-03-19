"""Tests for TrochoidalPathGenerator logic (Python mirror of C# implementation).

Validates trochoidal radius calculation, max engagement angle,
slot path generation, time saving estimates, parameter validation,
and directional variants.
"""

import math
import pytest
from dataclasses import dataclass, field
from typing import List, Tuple


# ---- Python mirror of C# types ----

@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __mul__(self, s: float) -> "Vector3":
        return Vector3(self.x * s, self.y * s, self.z * s)

    def __rmul__(self, s: float) -> "Vector3":
        return self.__mul__(s)

    def magnitude(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def normalized(self) -> "Vector3":
        m = self.magnitude()
        if m < 1e-12:
            return Vector3(0, 0, 0)
        return Vector3(self.x / m, self.y / m, self.z / m)

    @staticmethod
    def distance(a: "Vector3", b: "Vector3") -> float:
        return (a - b).magnitude()


@dataclass
class TrochoidalParams:
    slotWidth: float = 0.0
    toolDiameter: float = 0.0
    stepover: float = 0.0
    trochoidRadius: float = 0.0
    feedRate: float = 0.0
    direction: str = "climb"


@dataclass
class TrochoidalPath:
    points: List[Vector3] = field(default_factory=list)
    totalLength: float = 0.0
    numberOfLoops: int = 0
    estimatedTimeSec: float = 0.0
    maxEngagementDeg: float = 0.0


# ---- Python mirror of TrochoidalPathGenerator ----

class TrochoidalPathGenerator:
    POINTS_PER_LOOP = 36

    def GenerateSlotPath(self, startPoint: Vector3, endPoint: Vector3,
                         param: TrochoidalParams) -> TrochoidalPath:
        self.ValidateParams(param)

        result = TrochoidalPath()
        slot_dir = endPoint - startPoint
        slot_length = slot_dir.magnitude()

        if slot_length < 1e-6:
            result.points.append(startPoint)
            return result

        forward = slot_dir.normalized()
        # Lateral direction perpendicular to slot (in XZ plane)
        lateral = Vector3(-forward.z, 0.0, forward.x)
        if lateral.magnitude() < 1e-6:
            lateral = Vector3(-forward.y, forward.x, 0.0)
        lateral = lateral.normalized()

        radius = param.trochoidRadius
        step = param.stepover
        loop_count = max(1, math.ceil(slot_length / step))

        total_len = 0.0
        prev = startPoint
        result.points.append(startPoint)

        is_climb = param.direction != "conventional"

        for loop in range(loop_count):
            base_advance = loop * step

            for i in range(1, self.POINTS_PER_LOOP + 1):
                t = i / self.POINTS_PER_LOOP
                angle = t * math.pi * 2.0
                if not is_climb:
                    angle = -angle

                loop_advance = min(base_advance + t * step, slot_length)
                pt = startPoint + forward * loop_advance + lateral * (radius * math.sin(angle))
                pt = Vector3(pt.x, startPoint.y, pt.z)  # keep Y constant

                result.points.append(pt)
                total_len += Vector3.distance(prev, pt)
                prev = pt

        # Final move to end point
        result.points.append(endPoint)
        total_len += Vector3.distance(prev, endPoint)

        result.totalLength = total_len
        result.numberOfLoops = loop_count
        result.maxEngagementDeg = self.CalculateMaxEngagement(
            radius, param.toolDiameter, param.stepover)
        result.estimatedTimeSec = (total_len / param.feedRate) * 60.0 if param.feedRate > 0 else 0.0

        return result

    def CalculateTrochoidRadius(self, slotWidth: float, toolDiameter: float) -> float:
        if slotWidth <= 0:
            raise ValueError("Slot width must be positive.")
        if toolDiameter <= 0:
            raise ValueError("Tool diameter must be positive.")
        if toolDiameter >= slotWidth:
            raise ValueError(
                "Tool diameter must be smaller than slot width for trochoidal milling.")
        return (slotWidth - toolDiameter) / 2.0

    def CalculateMaxEngagement(self, trochoidRadius: float,
                                toolDiameter: float,
                                stepover: float) -> float:
        if trochoidRadius <= 0:
            raise ValueError("Trochoid radius must be positive.")
        if toolDiameter <= 0:
            raise ValueError("Tool diameter must be positive.")
        if stepover <= 0:
            raise ValueError("Stepover must be positive.")

        effective_radius = trochoidRadius + toolDiameter / 2.0
        sin_val = stepover / (2.0 * effective_radius)
        sin_val = max(0.0, min(1.0, sin_val))
        angle_deg = 2.0 * math.degrees(math.asin(sin_val))
        return max(0.0, min(180.0, angle_deg))

    def EstimateTimeSaving(self, conventionalTimeSec: float,
                            trochoidalTimeSec: float) -> float:
        if conventionalTimeSec <= 0:
            return 0.0
        return ((conventionalTimeSec - trochoidalTimeSec) / conventionalTimeSec) * 100.0

    def ValidateParams(self, p: TrochoidalParams) -> None:
        if p is None:
            raise ValueError("Params must not be None.")
        if p.slotWidth <= 0:
            raise ValueError("Slot width must be positive.")
        if p.toolDiameter <= 0:
            raise ValueError("Tool diameter must be positive.")
        if p.toolDiameter >= p.slotWidth:
            raise ValueError("Tool diameter must be smaller than slot width.")
        if p.stepover <= 0:
            raise ValueError("Stepover must be positive.")
        if p.trochoidRadius <= 0:
            raise ValueError("Trochoid radius must be positive.")
        if p.stepover >= 2.0 * p.trochoidRadius:
            raise ValueError(
                "Stepover must be less than the trochoid diameter (2 * radius).")
        if p.feedRate <= 0:
            raise ValueError("Feed rate must be positive.")
        if p.direction not in ("climb", "conventional"):
            raise ValueError("Direction must be 'climb' or 'conventional'.")


# ---- Fixtures ----

@pytest.fixture
def gen():
    return TrochoidalPathGenerator()


@pytest.fixture
def default_params():
    """Standard params: 12 mm slot, 6 mm tool, 2 mm stepover, 3 mm radius."""
    return TrochoidalParams(
        slotWidth=12.0,
        toolDiameter=6.0,
        stepover=2.0,
        trochoidRadius=3.0,
        feedRate=1000.0,
        direction="climb",
    )


# ---- Tests ----

class TestCalculateTrochoidRadius:
    """Tests for CalculateTrochoidRadius."""

    def test_optimal_radius_basic(self, gen):
        """radius = (slotWidth - toolDiameter) / 2 for a standard case."""
        r = gen.CalculateTrochoidRadius(12.0, 6.0)
        assert r == pytest.approx(3.0, rel=1e-6)

    def test_narrow_slot(self, gen):
        """Smaller difference yields smaller radius."""
        r = gen.CalculateTrochoidRadius(8.0, 6.0)
        assert r == pytest.approx(1.0, rel=1e-6)

    def test_tool_too_large_raises(self, gen):
        """Tool diameter >= slot width should raise."""
        with pytest.raises(ValueError):
            gen.CalculateTrochoidRadius(6.0, 6.0)
        with pytest.raises(ValueError):
            gen.CalculateTrochoidRadius(5.0, 6.0)

    def test_zero_slot_width_raises(self, gen):
        """Zero slot width is invalid."""
        with pytest.raises(ValueError):
            gen.CalculateTrochoidRadius(0.0, 6.0)


class TestCalculateMaxEngagement:
    """Tests for CalculateMaxEngagement."""

    def test_small_stepover_low_engagement(self, gen):
        """Small stepover relative to effective radius gives a small angle."""
        angle = gen.CalculateMaxEngagement(3.0, 6.0, 1.0)
        # effective_radius = 3 + 3 = 6; sin_val = 0.5/6 ~ 0.0833
        expected = 2.0 * math.degrees(math.asin(1.0 / 12.0))
        assert angle == pytest.approx(expected, rel=1e-4)

    def test_larger_stepover_higher_engagement(self, gen):
        """Increasing stepover increases engagement angle."""
        angle_small = gen.CalculateMaxEngagement(3.0, 6.0, 1.0)
        angle_large = gen.CalculateMaxEngagement(3.0, 6.0, 4.0)
        assert angle_large > angle_small

    def test_engagement_clamped_at_180(self, gen):
        """Even with extreme stepover, engagement does not exceed 180 deg."""
        angle = gen.CalculateMaxEngagement(1.0, 2.0, 100.0)
        assert angle <= 180.0

    def test_invalid_inputs_raise(self, gen):
        """Zero or negative inputs should raise."""
        with pytest.raises(ValueError):
            gen.CalculateMaxEngagement(0.0, 6.0, 1.0)
        with pytest.raises(ValueError):
            gen.CalculateMaxEngagement(3.0, 0.0, 1.0)
        with pytest.raises(ValueError):
            gen.CalculateMaxEngagement(3.0, 6.0, 0.0)


class TestEstimateTimeSaving:
    """Tests for EstimateTimeSaving."""

    def test_trochoidal_takes_longer_negative_saving(self, gen):
        """Trochoidal is typically longer -> negative saving percentage."""
        saving = gen.EstimateTimeSaving(60.0, 90.0)
        assert saving == pytest.approx(-50.0, rel=1e-4)

    def test_equal_times_zero_saving(self, gen):
        """Same machining time -> 0% saving."""
        saving = gen.EstimateTimeSaving(60.0, 60.0)
        assert saving == pytest.approx(0.0)

    def test_trochoidal_faster_positive_saving(self, gen):
        """If trochoidal is faster (rare), saving is positive."""
        saving = gen.EstimateTimeSaving(100.0, 60.0)
        assert saving == pytest.approx(40.0, rel=1e-4)

    def test_zero_conventional_returns_zero(self, gen):
        """Zero conventional time returns 0 to avoid division by zero."""
        assert gen.EstimateTimeSaving(0.0, 50.0) == pytest.approx(0.0)


class TestValidateParams:
    """Tests for ValidateParams."""

    def test_valid_params_no_error(self, gen, default_params):
        """Standard params should not raise."""
        gen.ValidateParams(default_params)  # should not raise

    def test_tool_wider_than_slot_raises(self, gen, default_params):
        """Tool diameter >= slot width is invalid."""
        default_params.toolDiameter = 15.0
        with pytest.raises(ValueError, match="smaller than slot width"):
            gen.ValidateParams(default_params)

    def test_stepover_too_large_raises(self, gen, default_params):
        """Stepover >= trochoid diameter is invalid."""
        default_params.stepover = 6.0  # == 2 * trochoidRadius
        with pytest.raises(ValueError, match="trochoid diameter"):
            gen.ValidateParams(default_params)

    def test_invalid_direction_raises(self, gen, default_params):
        """Direction must be 'climb' or 'conventional'."""
        default_params.direction = "sideways"
        with pytest.raises(ValueError, match="Direction"):
            gen.ValidateParams(default_params)

    def test_negative_feed_raises(self, gen, default_params):
        """Feed rate must be positive."""
        default_params.feedRate = -100.0
        with pytest.raises(ValueError, match="Feed rate"):
            gen.ValidateParams(default_params)


class TestGenerateSlotPath:
    """Tests for GenerateSlotPath."""

    def test_path_has_expected_loop_count(self, gen, default_params):
        """Number of loops = ceil(slot_length / stepover)."""
        start = Vector3(0, 0, 0)
        end = Vector3(10, 0, 0)
        path = gen.GenerateSlotPath(start, end, default_params)
        expected_loops = math.ceil(10.0 / default_params.stepover)
        assert path.numberOfLoops == expected_loops

    def test_path_starts_and_ends_correctly(self, gen, default_params):
        """First point is start, last point is end."""
        start = Vector3(0, 0, 0)
        end = Vector3(20, 0, 0)
        path = gen.GenerateSlotPath(start, end, default_params)
        assert path.points[0].x == pytest.approx(start.x, abs=1e-6)
        assert path.points[0].z == pytest.approx(start.z, abs=1e-6)
        assert path.points[-1].x == pytest.approx(end.x, abs=1e-6)
        assert path.points[-1].z == pytest.approx(end.z, abs=1e-6)

    def test_total_length_greater_than_slot(self, gen, default_params):
        """Trochoidal path is always longer than a straight slot."""
        start = Vector3(0, 0, 0)
        end = Vector3(20, 0, 0)
        straight_len = Vector3.distance(start, end)
        path = gen.GenerateSlotPath(start, end, default_params)
        assert path.totalLength > straight_len

    def test_estimated_time_positive(self, gen, default_params):
        """Machining time must be positive for a non-zero slot."""
        start = Vector3(0, 0, 0)
        end = Vector3(15, 0, 0)
        path = gen.GenerateSlotPath(start, end, default_params)
        assert path.estimatedTimeSec > 0.0

    def test_conventional_direction_generates_path(self, gen, default_params):
        """Conventional direction also produces a valid path."""
        default_params.direction = "conventional"
        start = Vector3(0, 0, 0)
        end = Vector3(10, 0, 0)
        path = gen.GenerateSlotPath(start, end, default_params)
        assert path.numberOfLoops > 0
        assert path.totalLength > 0.0

    def test_zero_length_slot_returns_single_point(self, gen, default_params):
        """A zero-length slot returns a single start point and zero loops."""
        start = Vector3(5, 0, 0)
        path = gen.GenerateSlotPath(start, start, default_params)
        assert len(path.points) == 1
        assert path.numberOfLoops == 0
        assert path.totalLength == pytest.approx(0.0)

    def test_max_engagement_populated(self, gen, default_params):
        """maxEngagementDeg is populated and positive."""
        start = Vector3(0, 0, 0)
        end = Vector3(10, 0, 0)
        path = gen.GenerateSlotPath(start, end, default_params)
        assert path.maxEngagementDeg > 0.0
        assert path.maxEngagementDeg <= 180.0

    def test_point_count_matches_loops(self, gen, default_params):
        """Total points = 1 (start) + loops * POINTS_PER_LOOP + 1 (end)."""
        start = Vector3(0, 0, 0)
        end = Vector3(10, 0, 0)
        path = gen.GenerateSlotPath(start, end, default_params)
        expected_points = 1 + path.numberOfLoops * gen.POINTS_PER_LOOP + 1
        assert len(path.points) == expected_points
