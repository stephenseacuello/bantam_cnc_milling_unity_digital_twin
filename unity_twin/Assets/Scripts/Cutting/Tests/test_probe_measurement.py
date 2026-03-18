"""Tests for ProbeMeasurementSimulator logic (Python mirror of C# implementation).

Validates touch probe measurement cycles (G38.2 probing moves) for bore,
boss, and plane features, including form error calculation, pass/fail
determination, and measurement report generation.
"""

import math
import pytest
from dataclasses import dataclass, field
from typing import List, Optional
from datetime import datetime, timezone


# ---- Vector3 helper (mirrors UnityEngine.Vector3) ----

@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vector3":
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> "Vector3":
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> "Vector3":
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)

    def __neg__(self) -> "Vector3":
        return Vector3(-self.x, -self.y, -self.z)

    def magnitude(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def normalized(self) -> "Vector3":
        m = self.magnitude()
        if m < 1e-10:
            return Vector3(0, 0, 0)
        return Vector3(self.x / m, self.y / m, self.z / m)

    @staticmethod
    def distance(a: "Vector3", b: "Vector3") -> float:
        return (a - b).magnitude()

    @staticmethod
    def cross(a: "Vector3", b: "Vector3") -> "Vector3":
        return Vector3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x,
        )

    @staticmethod
    def right() -> "Vector3":
        return Vector3(1, 0, 0)

    @staticmethod
    def forward() -> "Vector3":
        return Vector3(0, 0, 1)

    @staticmethod
    def up() -> "Vector3":
        return Vector3(0, 1, 0)

    @staticmethod
    def zero() -> "Vector3":
        return Vector3(0, 0, 0)


# ---- Python mirror of C# data classes ----

@dataclass
class ProbePoint:
    position: Vector3 = field(default_factory=Vector3)
    normal: Vector3 = field(default_factory=Vector3)
    measured_value: float = 0.0
    nominal_value: float = 0.0
    deviation: float = 0.0
    timestamp: str = ""


@dataclass
class ProbeResult:
    points: List[ProbePoint] = field(default_factory=list)
    feature_type: str = ""          # bore, boss, plane, edge, slot
    measured_diameter: float = 0.0
    measured_position: Vector3 = field(default_factory=Vector3)
    form_error: float = 0.0
    position_error: float = 0.0
    passed: bool = False


@dataclass
class ProbeCycle:
    cycle_id: str = ""
    feature_type: str = ""
    nominal_diameter: float = 0.0
    nominal_position: Vector3 = field(default_factory=Vector3)
    tolerance: float = 0.0
    points: List[ProbePoint] = field(default_factory=list)
    result: Optional[ProbeResult] = None


# ---- ProbeMeasurementSimulator (Python mirror) ----

class ProbeMeasurementSimulator:
    """Simulates touch probe measurement cycles (G38.2 probing moves).

    Generates synthetic probe data with configurable Gaussian noise,
    computes form errors (circularity / flatness), and produces
    formatted measurement reports.
    """

    def __init__(self, noise_std_dev: float = 0.002, seed: Optional[int] = None):
        self.noise_std_dev = noise_std_dev
        import random
        self._rng = random.Random(seed)
        self._cycle_counter = 0

    # ── Noise Generation ──────────────────────────────────────────

    def _gaussian_noise(self) -> float:
        """Generate Gaussian-distributed noise via Box-Muller transform."""
        u1 = 1.0 - self._rng.random()  # avoid log(0)
        u2 = self._rng.random()
        z = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
        return z * self.noise_std_dev

    def _next_cycle_id(self) -> str:
        self._cycle_counter += 1
        return f"PROBE-{self._cycle_counter:04d}"

    # ── Bore Measurement ──────────────────────────────────────────

    def simulate_bore_measurement(
        self,
        center: Vector3,
        nominal_dia: float,
        tolerance: float,
        num_points: int = 8,
    ) -> ProbeCycle:
        """Simulate a bore (internal cylinder) probing cycle.

        Points are distributed evenly around the bore circumference with
        Gaussian noise applied to the radial measurement.
        """
        if num_points < 3:
            raise ValueError("At least 3 points are required for bore measurement.")

        nominal_radius = nominal_dia / 2.0
        points: List[ProbePoint] = []
        ts = datetime.now(timezone.utc).isoformat()

        for i in range(num_points):
            angle = (2.0 * math.pi * i) / num_points
            noise = self._gaussian_noise()
            measured_radius = nominal_radius + noise

            normal = Vector3(math.cos(angle), math.sin(angle), 0.0)
            pos = center + normal * measured_radius

            points.append(ProbePoint(
                position=pos,
                normal=normal,
                measured_value=measured_radius * 2.0,
                nominal_value=nominal_dia,
                deviation=noise * 2.0,
                timestamp=ts,
            ))

        form_error = self.calculate_form_error(points, "bore")
        measured_dia = sum(p.measured_value for p in points) / len(points)
        measured_center = self._calculate_measured_center(points, center)
        position_error = Vector3.distance(measured_center, center)
        passed = abs(measured_dia - nominal_dia) <= tolerance and form_error <= tolerance

        result = ProbeResult(
            points=points,
            feature_type="bore",
            measured_diameter=measured_dia,
            measured_position=measured_center,
            form_error=form_error,
            position_error=position_error,
            passed=passed,
        )

        return ProbeCycle(
            cycle_id=self._next_cycle_id(),
            feature_type="bore",
            nominal_diameter=nominal_dia,
            nominal_position=center,
            tolerance=tolerance,
            points=points,
            result=result,
        )

    # ── Boss Measurement ──────────────────────────────────────────

    def simulate_boss_measurement(
        self,
        center: Vector3,
        nominal_dia: float,
        tolerance: float,
        num_points: int = 8,
    ) -> ProbeCycle:
        """Simulate a boss (external cylinder) probing cycle.

        Points are probed inward from outside the boss circumference.
        """
        if num_points < 3:
            raise ValueError("At least 3 points are required for boss measurement.")

        nominal_radius = nominal_dia / 2.0
        points: List[ProbePoint] = []
        ts = datetime.now(timezone.utc).isoformat()

        for i in range(num_points):
            angle = (2.0 * math.pi * i) / num_points
            noise = self._gaussian_noise()
            measured_radius = nominal_radius + noise

            outward = Vector3(math.cos(angle), math.sin(angle), 0.0)
            normal = -outward
            pos = center + outward * measured_radius

            points.append(ProbePoint(
                position=pos,
                normal=normal,
                measured_value=measured_radius * 2.0,
                nominal_value=nominal_dia,
                deviation=noise * 2.0,
                timestamp=ts,
            ))

        form_error = self.calculate_form_error(points, "boss")
        measured_dia = sum(p.measured_value for p in points) / len(points)
        measured_center = self._calculate_measured_center(points, center)
        position_error = Vector3.distance(measured_center, center)
        passed = abs(measured_dia - nominal_dia) <= tolerance and form_error <= tolerance

        result = ProbeResult(
            points=points,
            feature_type="boss",
            measured_diameter=measured_dia,
            measured_position=measured_center,
            form_error=form_error,
            position_error=position_error,
            passed=passed,
        )

        return ProbeCycle(
            cycle_id=self._next_cycle_id(),
            feature_type="boss",
            nominal_diameter=nominal_dia,
            nominal_position=center,
            tolerance=tolerance,
            points=points,
            result=result,
        )

    # ── Plane Measurement ─────────────────────────────────────────

    def simulate_plane_measurement(
        self,
        origin: Vector3,
        normal: Vector3,
        width: float,
        length: float,
        num_points: int = 9,
    ) -> ProbeCycle:
        """Simulate a surface flatness probing cycle.

        Points are distributed in a grid across the plane surface with
        Gaussian noise applied along the surface normal.
        """
        if num_points < 3:
            raise ValueError("At least 3 points are required for plane measurement.")

        normal = normal.normalized()

        # Build local coordinate frame on the plane
        tangent1 = Vector3.cross(normal, Vector3.right()).normalized()
        if tangent1.magnitude() < 0.01:
            tangent1 = Vector3.cross(normal, Vector3.forward()).normalized()
        tangent2 = Vector3.cross(normal, tangent1).normalized()

        points: List[ProbePoint] = []
        ts = datetime.now(timezone.utc).isoformat()

        grid_size = max(2, math.ceil(math.sqrt(num_points)))
        generated = 0

        for i in range(grid_size):
            if generated >= num_points:
                break
            for j in range(grid_size):
                if generated >= num_points:
                    break

                u = i / (grid_size - 1) if grid_size > 1 else 0.5
                v = j / (grid_size - 1) if grid_size > 1 else 0.5

                local_offset = tangent1 * ((u - 0.5) * width) + tangent2 * ((v - 0.5) * length)
                noise = self._gaussian_noise()
                pos = origin + local_offset + normal * noise

                points.append(ProbePoint(
                    position=pos,
                    normal=normal,
                    measured_value=noise,
                    nominal_value=0.0,
                    deviation=noise,
                    timestamp=ts,
                ))
                generated += 1

        form_error = self.calculate_form_error(points, "plane")

        result = ProbeResult(
            points=points,
            feature_type="plane",
            measured_diameter=0.0,
            measured_position=origin,
            form_error=form_error,
            position_error=0.0,
            passed=form_error <= 0.01,  # default 10 um flatness tolerance
        )

        return ProbeCycle(
            cycle_id=self._next_cycle_id(),
            feature_type="plane",
            nominal_diameter=0.0,
            nominal_position=origin,
            tolerance=0.01,
            points=points,
            result=result,
        )

    # ── Form Error Calculation ────────────────────────────────────

    def calculate_form_error(self, points: List[ProbePoint], feature_type: str) -> float:
        """Calculate form error for probed points.

        For bore/boss: circularity (max radius - min radius).
        For plane: flatness (max deviation - min deviation).
        """
        if not points:
            return 0.0

        if feature_type in ("bore", "boss"):
            radii = [p.measured_value / 2.0 for p in points]
            return max(radii) - min(radii)
        elif feature_type == "plane":
            devs = [p.deviation for p in points]
            return max(devs) - min(devs)
        else:
            return max(abs(p.deviation) for p in points)

    # ── Measurement Report ────────────────────────────────────────

    def get_measurement_report(self, cycle: ProbeCycle) -> str:
        """Generate a formatted measurement report for a probe cycle."""
        if cycle is None:
            raise ValueError("cycle must not be None")
        if cycle.result is None:
            raise ValueError("ProbeCycle has no result.")

        r = cycle.result
        lines = [
            "=" * 51,
            f"  PROBE MEASUREMENT REPORT -- {cycle.cycle_id}",
            "=" * 51,
            f"  Feature Type    : {cycle.feature_type}",
        ]

        if cycle.feature_type in ("bore", "boss"):
            lines.append(f"  Nominal Diameter: {cycle.nominal_diameter:.4f} mm")
            lines.append(f"  Measured Diameter: {r.measured_diameter:.4f} mm")
            lines.append(f"  Diameter Error  : {abs(r.measured_diameter - cycle.nominal_diameter):.4f} mm")

        np = cycle.nominal_position
        mp = r.measured_position
        lines.append(f"  Nominal Position: ({np.x:.4f}, {np.y:.4f}, {np.z:.4f})")
        lines.append(f"  Measured Position: ({mp.x:.4f}, {mp.y:.4f}, {mp.z:.4f})")
        lines.append(f"  Form Error      : {r.form_error:.4f} mm")
        lines.append(f"  Position Error  : {r.position_error:.4f} mm")
        lines.append(f"  Tolerance       : {cycle.tolerance:.4f} mm")
        lines.append(f"  Points Measured : {len(r.points)}")
        lines.append(f"  Verdict         : {'PASS' if r.passed else 'FAIL'}")
        lines.append("=" * 51)

        return "\n".join(lines)

    # ── Helpers ───────────────────────────────────────────────────

    def _calculate_measured_center(
        self, points: List[ProbePoint], nominal_center: Vector3
    ) -> Vector3:
        if not points:
            return nominal_center
        sx = sum(p.position.x for p in points)
        sy = sum(p.position.y for p in points)
        sz = sum(p.position.z for p in points)
        n = len(points)
        return Vector3(sx / n, sy / n, sz / n)


# ---- Fixtures ----

@pytest.fixture
def simulator():
    """Seeded simulator with small noise for deterministic tests."""
    return ProbeMeasurementSimulator(noise_std_dev=0.002, seed=42)


@pytest.fixture
def bore_center():
    return Vector3(100.0, 50.0, 0.0)


@pytest.fixture
def boss_center():
    return Vector3(200.0, 100.0, 0.0)


# ---- Tests ----

def test_bore_measurement_generates_correct_number_of_points(simulator, bore_center):
    """Bore probing cycle produces the requested number of probe points."""
    cycle = simulator.simulate_bore_measurement(bore_center, 25.0, 0.05, num_points=12)
    assert len(cycle.points) == 12
    assert len(cycle.result.points) == 12
    assert cycle.feature_type == "bore"
    assert cycle.result.feature_type == "bore"


def test_bore_measurement_diameter_close_to_nominal(simulator, bore_center):
    """Measured bore diameter is within a reasonable range of the nominal value."""
    cycle = simulator.simulate_bore_measurement(bore_center, 25.0, 0.05, num_points=16)
    # With noise_std_dev=0.002, measured diameter should be very close to 25.0
    assert cycle.result.measured_diameter == pytest.approx(25.0, abs=0.05)


def test_bore_measurement_pass_with_generous_tolerance(simulator, bore_center):
    """Bore measurement passes when tolerance is generous relative to noise."""
    cycle = simulator.simulate_bore_measurement(bore_center, 25.0, 0.1, num_points=8)
    assert cycle.result.passed is True


def test_bore_measurement_fail_with_tight_tolerance():
    """Bore measurement fails when tolerance is extremely tight and noise is large."""
    # Use high noise to guarantee failure
    noisy_sim = ProbeMeasurementSimulator(noise_std_dev=0.5, seed=99)
    cycle = noisy_sim.simulate_bore_measurement(Vector3(0, 0, 0), 25.0, 0.001, num_points=8)
    assert cycle.result.passed is False


def test_boss_measurement_generates_correct_feature_type(simulator, boss_center):
    """Boss probing cycle produces the correct feature type and cycle ID."""
    cycle = simulator.simulate_boss_measurement(boss_center, 30.0, 0.05, num_points=8)
    assert cycle.feature_type == "boss"
    assert cycle.result.feature_type == "boss"
    assert cycle.cycle_id.startswith("PROBE-")


def test_boss_measurement_normals_point_inward(simulator, boss_center):
    """Boss probe normals point inward (toward center)."""
    cycle = simulator.simulate_boss_measurement(boss_center, 30.0, 0.05, num_points=8)
    for point in cycle.points:
        # The normal should point roughly from the point toward the center
        to_center = boss_center - point.position
        # dot product of normal and to_center should be positive (same general direction)
        dot = (point.normal.x * to_center.x +
               point.normal.y * to_center.y +
               point.normal.z * to_center.z)
        assert dot > 0, "Boss probe normal should point toward center"


def test_plane_measurement_flatness(simulator):
    """Plane measurement computes a small flatness form error with low noise."""
    origin = Vector3(0, 0, 0)
    normal = Vector3(0, 0, 1)
    cycle = simulator.simulate_plane_measurement(origin, normal, 100.0, 100.0, num_points=9)
    assert cycle.feature_type == "plane"
    assert cycle.result.feature_type == "plane"
    # With 0.002 noise_std_dev, flatness should be small (< 0.02 mm typically)
    assert cycle.result.form_error < 0.05
    assert cycle.result.form_error >= 0.0


def test_plane_measurement_minimum_points_enforced(simulator):
    """Plane measurement raises an error with fewer than 3 points."""
    with pytest.raises(ValueError, match="At least 3 points"):
        simulator.simulate_plane_measurement(
            Vector3(0, 0, 0), Vector3(0, 0, 1), 50.0, 50.0, num_points=2
        )


def test_bore_measurement_minimum_points_enforced(simulator, bore_center):
    """Bore measurement raises an error with fewer than 3 points."""
    with pytest.raises(ValueError, match="At least 3 points"):
        simulator.simulate_bore_measurement(bore_center, 25.0, 0.05, num_points=2)


def test_form_error_circularity_for_bore(simulator, bore_center):
    """Form error for bore is circularity (max_radius - min_radius)."""
    cycle = simulator.simulate_bore_measurement(bore_center, 25.0, 0.05, num_points=8)
    radii = [p.measured_value / 2.0 for p in cycle.points]
    expected_circularity = max(radii) - min(radii)
    assert cycle.result.form_error == pytest.approx(expected_circularity)


def test_form_error_flatness_for_plane(simulator):
    """Form error for plane is flatness (max_deviation - min_deviation)."""
    origin = Vector3(0, 0, 0)
    normal = Vector3(0, 0, 1)
    cycle = simulator.simulate_plane_measurement(origin, normal, 50.0, 50.0, num_points=9)
    devs = [p.deviation for p in cycle.points]
    expected_flatness = max(devs) - min(devs)
    assert cycle.result.form_error == pytest.approx(expected_flatness)


def test_measurement_report_contains_key_fields(simulator, bore_center):
    """Measurement report includes all essential fields and verdict."""
    cycle = simulator.simulate_bore_measurement(bore_center, 25.0, 0.1, num_points=8)
    report = simulator.get_measurement_report(cycle)

    assert cycle.cycle_id in report
    assert "bore" in report
    assert "Nominal Diameter" in report
    assert "Measured Diameter" in report
    assert "Form Error" in report
    assert "Position Error" in report
    assert "Tolerance" in report
    assert "Points Measured" in report
    assert "PASS" in report or "FAIL" in report


def test_measurement_report_fail_verdict():
    """Measurement report shows FAIL for an out-of-tolerance measurement."""
    noisy_sim = ProbeMeasurementSimulator(noise_std_dev=1.0, seed=7)
    cycle = noisy_sim.simulate_bore_measurement(Vector3(0, 0, 0), 25.0, 0.001, num_points=8)
    report = noisy_sim.get_measurement_report(cycle)
    assert "FAIL" in report


def test_cycle_ids_are_sequential(simulator, bore_center):
    """Each probe cycle gets a unique, sequential cycle ID."""
    c1 = simulator.simulate_bore_measurement(bore_center, 25.0, 0.1, num_points=4)
    c2 = simulator.simulate_boss_measurement(bore_center, 30.0, 0.1, num_points=4)
    c3 = simulator.simulate_plane_measurement(
        Vector3(0, 0, 0), Vector3(0, 0, 1), 50.0, 50.0, num_points=4
    )
    assert c1.cycle_id == "PROBE-0001"
    assert c2.cycle_id == "PROBE-0002"
    assert c3.cycle_id == "PROBE-0003"


def test_form_error_empty_points(simulator):
    """Form error returns 0 for an empty point list."""
    assert simulator.calculate_form_error([], "bore") == 0.0
    assert simulator.calculate_form_error([], "plane") == 0.0


def test_position_error_close_to_zero_for_bore(simulator, bore_center):
    """Position error is small when noise is small (center is accurate)."""
    cycle = simulator.simulate_bore_measurement(bore_center, 25.0, 0.1, num_points=32)
    # With many points and small noise, measured center should be near nominal
    assert cycle.result.position_error < 0.01
