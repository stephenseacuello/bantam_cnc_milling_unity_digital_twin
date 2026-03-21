"""Tests for ThreadMillingCalculator logic (Python mirror of C# implementation).

Validates helical diameter calculation, radial depth, helical path generation,
feed rate, thread tolerances, cycle time estimation, and full calculation flow.
"""

import math
import pytest
from dataclasses import dataclass, field
from typing import List, Optional


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

    @staticmethod
    def distance(a: "Vector3", b: "Vector3") -> float:
        return (a - b).magnitude()

    @staticmethod
    def zero() -> "Vector3":
        return Vector3(0.0, 0.0, 0.0)


@dataclass
class ThreadSpec:
    nominalDiameter: float = 10.0
    pitch: float = 1.5
    threadType: str = "metric"
    isInternal: bool = True
    threadClass: str = "6H"
    length: float = 10.0


@dataclass
class ThreadMillResult:
    helicalDiameter: float = 0.0
    radialDepth: float = 0.0
    numberOfPasses: int = 0
    feedRate: float = 0.0
    spindleSpeed: int = 0
    cycleTime: float = 0.0
    threadPath: List[Vector3] = field(default_factory=list)


@dataclass
class ThreadTolerances:
    majorDiameterMin: float = 0.0
    majorDiameterMax: float = 0.0
    minorDiameterMin: float = 0.0
    minorDiameterMax: float = 0.0


# ---- Python mirror of ThreadMillingCalculator ----

class ThreadMillingCalculator:
    THREAD_DEPTH_FACTOR = 0.6134
    MAX_RADIAL_DEPTH_PER_PASS = 0.15
    DEFAULT_POINTS_PER_REV = 72

    def CalculateHelicalDiameter(self, spec: ThreadSpec, toolDiameter: float) -> float:
        if spec is None:
            raise ValueError("Thread spec must not be None.")
        if toolDiameter <= 0:
            raise ValueError("Tool diameter must be positive.")
        if spec.nominalDiameter <= 0:
            raise ValueError("Nominal diameter must be positive.")

        if spec.isInternal:
            d = spec.nominalDiameter - toolDiameter
            if d <= 0:
                raise ValueError(
                    "Tool diameter must be smaller than nominal diameter for internal threads.")
            return d
        else:
            return spec.nominalDiameter + toolDiameter

    def CalculateRadialDepth(self, pitch: float) -> float:
        if pitch <= 0:
            raise ValueError("Pitch must be positive.")
        return self.THREAD_DEPTH_FACTOR * pitch

    def GenerateHelicalPath(self, center: Vector3, diameter: float,
                            pitch: float, length: float,
                            pointsPerRev: int = 0) -> List[Vector3]:
        if diameter <= 0:
            raise ValueError("Diameter must be positive.")
        if pitch <= 0:
            raise ValueError("Pitch must be positive.")
        if length <= 0:
            raise ValueError("Length must be positive.")

        if pointsPerRev <= 0:
            pointsPerRev = self.DEFAULT_POINTS_PER_REV

        radius = diameter / 2.0
        totalRevolutions = length / pitch
        totalPoints = max(1, math.ceil(totalRevolutions * pointsPerRev))

        path: List[Vector3] = []

        for i in range(totalPoints + 1):
            t = i / totalPoints  # 0..1
            angle = t * totalRevolutions * 2.0 * math.pi
            y = center.y + t * length
            x = center.x + radius * math.cos(angle)
            z = center.z + radius * math.sin(angle)
            path.append(Vector3(x, y, z))

        return path

    def CalculateFeedRate(self, pitch: float, spindleSpeed: int) -> float:
        if pitch <= 0:
            raise ValueError("Pitch must be positive.")
        if spindleSpeed <= 0:
            raise ValueError("Spindle speed must be positive.")
        return pitch * spindleSpeed

    def GetThreadTolerances(self, spec: ThreadSpec) -> ThreadTolerances:
        if spec is None:
            raise ValueError("Thread spec must not be None.")

        d = spec.nominalDiameter
        p = spec.pitch
        threadDepth = self.CalculateRadialDepth(p)

        tol = ThreadTolerances()
        minorDiameterBasic = d - 2.0 * threadDepth

        if spec.threadClass == "6H":
            tol.majorDiameterMin = d
            tol.majorDiameterMax = d + 0.050
            tol.minorDiameterMin = minorDiameterBasic
            tol.minorDiameterMax = minorDiameterBasic + 0.060
        elif spec.threadClass == "4H5H":
            tol.majorDiameterMin = d
            tol.majorDiameterMax = d + 0.032
            tol.minorDiameterMin = minorDiameterBasic
            tol.minorDiameterMax = minorDiameterBasic + 0.040
        elif spec.threadClass == "6g":
            tol.majorDiameterMin = d - 0.060
            tol.majorDiameterMax = d
            tol.minorDiameterMin = minorDiameterBasic - 0.060
            tol.minorDiameterMax = minorDiameterBasic
        elif spec.threadClass == "2B":
            tol.majorDiameterMin = d
            tol.majorDiameterMax = d + 0.048
            tol.minorDiameterMin = minorDiameterBasic
            tol.minorDiameterMax = minorDiameterBasic + 0.055
        elif spec.threadClass == "2A":
            tol.majorDiameterMin = d - 0.055
            tol.majorDiameterMax = d
            tol.minorDiameterMin = minorDiameterBasic - 0.055
            tol.minorDiameterMax = minorDiameterBasic
        else:
            tol.majorDiameterMin = d - 0.050
            tol.majorDiameterMax = d + 0.050
            tol.minorDiameterMin = minorDiameterBasic - 0.050
            tol.minorDiameterMax = minorDiameterBasic + 0.050

        return tol

    def EstimateCycleTime(self, spec: ThreadSpec, feedRate: float) -> float:
        if spec is None:
            raise ValueError("Thread spec must not be None.")
        if feedRate <= 0:
            raise ValueError("Feed rate must be positive.")
        if spec.pitch <= 0:
            raise ValueError("Pitch must be positive.")
        if spec.nominalDiameter <= 0:
            raise ValueError("Nominal diameter must be positive.")
        if spec.length <= 0:
            raise ValueError("Thread length must be positive.")

        totalRadialDepth = self.CalculateRadialDepth(spec.pitch)
        passes = max(1, math.ceil(totalRadialDepth / self.MAX_RADIAL_DEPTH_PER_PASS))

        helicalCircumference = math.pi * spec.nominalDiameter
        revolutions = spec.length / spec.pitch
        pathLengthPerPass = revolutions * helicalCircumference

        totalPathLength = pathLengthPerPass * passes

        # feedRate is mm/min -> time in seconds
        return (totalPathLength / feedRate) * 60.0

    def Calculate(self, spec: ThreadSpec, toolDiameter: float,
                  spindleSpeed: int, pointsPerRev: int = 0) -> ThreadMillResult:
        if spec is None:
            raise ValueError("Thread spec must not be None.")

        result = ThreadMillResult()
        result.helicalDiameter = self.CalculateHelicalDiameter(spec, toolDiameter)
        result.radialDepth = self.CalculateRadialDepth(spec.pitch)
        result.numberOfPasses = max(1,
            math.ceil(result.radialDepth / self.MAX_RADIAL_DEPTH_PER_PASS))
        result.spindleSpeed = spindleSpeed
        result.feedRate = self.CalculateFeedRate(spec.pitch, spindleSpeed)
        result.cycleTime = self.EstimateCycleTime(spec, result.feedRate)
        result.threadPath = self.GenerateHelicalPath(
            Vector3.zero(), result.helicalDiameter,
            spec.pitch, spec.length,
            pointsPerRev)

        return result


# ---- Fixtures ----

@pytest.fixture
def calc():
    return ThreadMillingCalculator()


@pytest.fixture
def m10_internal_spec():
    """M10x1.5 internal thread, class 6H, 10 mm deep."""
    return ThreadSpec(
        nominalDiameter=10.0,
        pitch=1.5,
        threadType="metric",
        isInternal=True,
        threadClass="6H",
        length=10.0,
    )


@pytest.fixture
def m10_external_spec():
    """M10x1.5 external thread, class 6g, 15 mm long."""
    return ThreadSpec(
        nominalDiameter=10.0,
        pitch=1.5,
        threadType="metric",
        isInternal=False,
        threadClass="6g",
        length=15.0,
    )


# ---- Tests ----

class TestCalculateHelicalDiameter:
    """Tests for CalculateHelicalDiameter."""

    def test_internal_thread_diameter(self, calc, m10_internal_spec):
        """Internal helical diameter = nominalDiameter - toolDiameter."""
        hd = calc.CalculateHelicalDiameter(m10_internal_spec, 4.0)
        assert hd == pytest.approx(6.0, rel=1e-6)

    def test_external_thread_diameter(self, calc, m10_external_spec):
        """External helical diameter = nominalDiameter + toolDiameter."""
        hd = calc.CalculateHelicalDiameter(m10_external_spec, 4.0)
        assert hd == pytest.approx(14.0, rel=1e-6)

    def test_internal_tool_too_large_raises(self, calc, m10_internal_spec):
        """Tool diameter >= nominal diameter is invalid for internal threads."""
        with pytest.raises(ValueError, match="smaller than nominal diameter"):
            calc.CalculateHelicalDiameter(m10_internal_spec, 10.0)
        with pytest.raises(ValueError, match="smaller than nominal diameter"):
            calc.CalculateHelicalDiameter(m10_internal_spec, 12.0)

    def test_zero_tool_diameter_raises(self, calc, m10_internal_spec):
        """Zero tool diameter is invalid."""
        with pytest.raises(ValueError, match="Tool diameter must be positive"):
            calc.CalculateHelicalDiameter(m10_internal_spec, 0.0)


class TestCalculateRadialDepth:
    """Tests for CalculateRadialDepth."""

    def test_metric_depth_formula(self, calc):
        """Radial depth = 0.6134 * pitch for 60-degree threads."""
        depth = calc.CalculateRadialDepth(1.5)
        assert depth == pytest.approx(0.6134 * 1.5, rel=1e-6)

    def test_fine_pitch(self, calc):
        """Fine pitch (e.g. 0.5 mm) produces proportionally smaller depth."""
        depth = calc.CalculateRadialDepth(0.5)
        assert depth == pytest.approx(0.6134 * 0.5, rel=1e-6)

    def test_zero_pitch_raises(self, calc):
        """Zero pitch is invalid."""
        with pytest.raises(ValueError, match="Pitch must be positive"):
            calc.CalculateRadialDepth(0.0)


class TestGenerateHelicalPath:
    """Tests for GenerateHelicalPath."""

    def test_path_point_count(self, calc):
        """Point count = ceil(revolutions * pointsPerRev) + 1."""
        center = Vector3.zero()
        path = calc.GenerateHelicalPath(center, 6.0, 1.5, 10.0, 36)
        revolutions = 10.0 / 1.5
        expected_count = math.ceil(revolutions * 36) + 1
        assert len(path) == expected_count

    def test_path_starts_at_correct_position(self, calc):
        """First point should be at (center.x + radius, center.y, center.z)."""
        center = Vector3(5.0, 0.0, 3.0)
        diameter = 8.0
        path = calc.GenerateHelicalPath(center, diameter, 1.0, 5.0, 36)
        # At t=0, angle=0 -> cos(0)=1, sin(0)=0
        assert path[0].x == pytest.approx(center.x + diameter / 2.0, abs=1e-6)
        assert path[0].y == pytest.approx(center.y, abs=1e-6)
        assert path[0].z == pytest.approx(center.z, abs=1e-6)

    def test_path_ends_at_correct_height(self, calc):
        """Last point Y should equal center.y + length."""
        center = Vector3.zero()
        length = 12.0
        path = calc.GenerateHelicalPath(center, 6.0, 1.5, length, 36)
        assert path[-1].y == pytest.approx(center.y + length, abs=1e-4)

    def test_helix_radius_consistent(self, calc):
        """All points should be at the specified radius from the centre axis."""
        center = Vector3.zero()
        diameter = 10.0
        radius = diameter / 2.0
        path = calc.GenerateHelicalPath(center, diameter, 2.0, 8.0, 36)
        for pt in path:
            r = math.sqrt((pt.x - center.x) ** 2 + (pt.z - center.z) ** 2)
            assert r == pytest.approx(radius, abs=1e-4)

    def test_invalid_diameter_raises(self, calc):
        """Zero or negative diameter is invalid."""
        with pytest.raises(ValueError, match="Diameter must be positive"):
            calc.GenerateHelicalPath(Vector3.zero(), 0.0, 1.0, 5.0)


class TestCalculateFeedRate:
    """Tests for CalculateFeedRate."""

    def test_basic_feed_rate(self, calc):
        """Feed = pitch * RPM."""
        feed = calc.CalculateFeedRate(1.5, 2000)
        assert feed == pytest.approx(3000.0, rel=1e-6)

    def test_fine_pitch_feed(self, calc):
        """Fine pitch at high RPM."""
        feed = calc.CalculateFeedRate(0.5, 5000)
        assert feed == pytest.approx(2500.0, rel=1e-6)

    def test_zero_spindle_speed_raises(self, calc):
        """Zero RPM is invalid."""
        with pytest.raises(ValueError, match="Spindle speed must be positive"):
            calc.CalculateFeedRate(1.5, 0)


class TestGetThreadTolerances:
    """Tests for GetThreadTolerances."""

    def test_6H_internal_tolerances(self, calc, m10_internal_spec):
        """6H class: major min = nominal, major max = nominal + 0.050."""
        tol = calc.GetThreadTolerances(m10_internal_spec)
        assert tol.majorDiameterMin == pytest.approx(10.0, abs=1e-6)
        assert tol.majorDiameterMax == pytest.approx(10.050, abs=1e-6)

    def test_6g_external_tolerances(self, calc, m10_external_spec):
        """6g class: major max = nominal, major min = nominal - 0.060."""
        tol = calc.GetThreadTolerances(m10_external_spec)
        assert tol.majorDiameterMax == pytest.approx(10.0, abs=1e-6)
        assert tol.majorDiameterMin == pytest.approx(9.940, abs=1e-6)

    def test_minor_diameter_consistent(self, calc, m10_internal_spec):
        """Minor diameter min should equal nominalDiameter - 2 * threadDepth."""
        tol = calc.GetThreadTolerances(m10_internal_spec)
        expected_minor = 10.0 - 2.0 * 0.6134 * 1.5
        assert tol.minorDiameterMin == pytest.approx(expected_minor, abs=1e-4)

    def test_unknown_class_falls_back(self, calc):
        """Unknown thread class returns symmetric general-purpose tolerances."""
        spec = ThreadSpec(nominalDiameter=8.0, pitch=1.25, threadClass="99Z")
        tol = calc.GetThreadTolerances(spec)
        assert tol.majorDiameterMin == pytest.approx(8.0 - 0.050, abs=1e-6)
        assert tol.majorDiameterMax == pytest.approx(8.0 + 0.050, abs=1e-6)


class TestEstimateCycleTime:
    """Tests for EstimateCycleTime."""

    def test_cycle_time_positive(self, calc, m10_internal_spec):
        """Cycle time must be positive for valid inputs."""
        ct = calc.EstimateCycleTime(m10_internal_spec, 3000.0)
        assert ct > 0.0

    def test_higher_feed_shorter_time(self, calc, m10_internal_spec):
        """Doubling feed rate should halve cycle time."""
        ct1 = calc.EstimateCycleTime(m10_internal_spec, 1000.0)
        ct2 = calc.EstimateCycleTime(m10_internal_spec, 2000.0)
        assert ct2 == pytest.approx(ct1 / 2.0, rel=1e-6)

    def test_zero_feed_rate_raises(self, calc, m10_internal_spec):
        """Zero feed rate is invalid."""
        with pytest.raises(ValueError, match="Feed rate must be positive"):
            calc.EstimateCycleTime(m10_internal_spec, 0.0)

    def test_longer_thread_increases_time(self, calc):
        """A longer thread at the same pitch takes more time."""
        short = ThreadSpec(nominalDiameter=10.0, pitch=1.5, length=10.0)
        long = ThreadSpec(nominalDiameter=10.0, pitch=1.5, length=20.0)
        ct_short = calc.EstimateCycleTime(short, 1000.0)
        ct_long = calc.EstimateCycleTime(long, 1000.0)
        assert ct_long == pytest.approx(ct_short * 2.0, rel=1e-6)


class TestCalculateFullResult:
    """Tests for the convenience Calculate method."""

    def test_full_calculation_returns_all_fields(self, calc, m10_internal_spec):
        """Calculate populates every field of ThreadMillResult."""
        result = calc.Calculate(m10_internal_spec, toolDiameter=4.0,
                                spindleSpeed=2000, pointsPerRev=36)
        assert result.helicalDiameter == pytest.approx(6.0, rel=1e-6)
        assert result.radialDepth == pytest.approx(0.6134 * 1.5, rel=1e-6)
        assert result.numberOfPasses >= 1
        assert result.feedRate == pytest.approx(1.5 * 2000, rel=1e-6)
        assert result.spindleSpeed == 2000
        assert result.cycleTime > 0.0
        assert len(result.threadPath) > 0

    def test_full_calculation_external(self, calc, m10_external_spec):
        """Calculate works for external threads as well."""
        result = calc.Calculate(m10_external_spec, toolDiameter=4.0,
                                spindleSpeed=3000, pointsPerRev=36)
        assert result.helicalDiameter == pytest.approx(14.0, rel=1e-6)
        assert result.feedRate == pytest.approx(1.5 * 3000, rel=1e-6)
        assert len(result.threadPath) > 0
