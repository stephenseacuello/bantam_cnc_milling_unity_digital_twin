"""Tests for ArcFittingOptimizer logic (Python mirror of C# implementation).

Validates least-squares circle fitting, arc detection across XY/XZ/YZ planes,
tolerance-based acceptance/rejection, straight-line handling, mixed paths,
small-segment edge cases, and line-reduction percentage reporting.
"""

import math
import pytest
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import Enum


# ---- Python mirror of C# types ----

class ArcPlane(Enum):
    XY = 'XY'
    XZ = 'XZ'
    YZ = 'YZ'


class PathSegmentType(Enum):
    LINE = 'LINE'
    ARC = 'ARC'


@dataclass
class ArcCandidate:
    center: Tuple[float, float, float]
    radius: float
    start_angle: float
    sweep_angle: float
    plane: ArcPlane
    start_point: Tuple[float, float, float]
    end_point: Tuple[float, float, float]
    max_deviation: float


@dataclass
class PathSegment:
    segment_type: PathSegmentType
    start_point: Tuple[float, float, float]
    end_point: Tuple[float, float, float]
    center: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    radius: float = 0.0
    is_clockwise: bool = False


@dataclass
class ArcFitResult:
    original_segment_count: int = 0
    fitted_arc_count: int = 0
    line_reduction_pct: float = 0.0
    total_deviation: float = 0.0


# ---- Helper: 2-D coordinate extraction ----

def _get_plane_coords(p: Tuple[float, float, float], plane: ArcPlane) -> Tuple[float, float]:
    if plane == ArcPlane.XY:
        return p[0], p[1]
    elif plane == ArcPlane.XZ:
        return p[0], p[2]
    else:  # YZ
        return p[1], p[2]


# ---- Python mirror of ArcFittingOptimizer ----

class ArcFittingOptimizer:
    def __init__(self, tolerance: float = 0.005, plane_tolerance: float = 0.001,
                 min_points_for_arc: int = 3):
        self.tolerance = tolerance
        self.plane_tolerance = plane_tolerance
        self.min_points_for_arc = min_points_for_arc

    # ── Plane detection ──────────────────────────────────────────────

    def detect_plane(self, points: List[Tuple[float, float, float]]) -> Optional[ArcPlane]:
        """Determine which principal plane the points lie on.
        Returns the plane if coplanar within plane_tolerance, else None."""
        if not points or len(points) < 2:
            return None

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]

        span_x = max(xs) - min(xs)
        span_y = max(ys) - min(ys)
        span_z = max(zs) - min(zs)

        # Flat axis is the one with smallest span
        if span_z <= span_x and span_z <= span_y:
            return ArcPlane.XY if span_z <= self.plane_tolerance else None
        if span_y <= span_x and span_y <= span_z:
            return ArcPlane.XZ if span_y <= self.plane_tolerance else None
        # span_x is smallest
        return ArcPlane.YZ if span_x <= self.plane_tolerance else None

    # ── Least-squares circle fitting (Kasa method) ────────────────────

    def fit_circle(self, points: List[Tuple[float, float, float]], plane: ArcPlane):
        """Fit a circle via the Kasa algebraic method.
        Returns (center_u, center_v, radius) or None if degenerate."""
        n = len(points)
        if n < 3:
            return None

        sum_u = sum_v = sum_u2 = sum_v2 = 0.0
        sum_uv = sum_u3 = sum_v3 = sum_u2v = sum_uv2 = 0.0

        for p in points:
            u, v = _get_plane_coords(p, plane)
            u2, v2 = u * u, v * v
            sum_u += u;  sum_v += v
            sum_u2 += u2; sum_v2 += v2
            sum_uv += u * v
            sum_u3 += u2 * u; sum_v3 += v2 * v
            sum_u2v += u2 * v; sum_uv2 += u * v2

        # 3x3 normal equations
        A = [
            [sum_u2, sum_uv, sum_u],
            [sum_uv, sum_v2, sum_v],
            [sum_u,  sum_v,  n],
        ]
        B = [
            -(sum_u3 + sum_uv2),
            -(sum_u2v + sum_v3),
            -(sum_u2 + sum_v2),
        ]

        det = _det3(A)
        if abs(det) < 1e-12:
            return None

        a = _det3_substituted(A, B, 0) / det
        b = _det3_substituted(A, B, 1) / det
        c = _det3_substituted(A, B, 2) / det

        cu = -a * 0.5
        cv = -b * 0.5
        r_sq = a * a * 0.25 + b * b * 0.25 - c
        if r_sq < 0:
            return None
        r = math.sqrt(r_sq)
        if r < 1e-6:
            return None
        return cu, cv, r

    # ── Build ArcCandidate ───────────────────────────────────────────

    def build_candidate(self, points, plane, cu, cv, radius):
        """Build an ArcCandidate from fitted circle parameters and source points."""
        # Out-of-plane coordinate (average)
        if plane == ArcPlane.XY:
            oop = sum(p[2] for p in points) / len(points)
            center_3d = (cu, cv, oop)
        elif plane == ArcPlane.XZ:
            oop = sum(p[1] for p in points) / len(points)
            center_3d = (cu, oop, cv)
        else:  # YZ
            oop = sum(p[0] for p in points) / len(points)
            center_3d = (oop, cu, cv)

        su, sv = _get_plane_coords(points[0], plane)
        eu, ev = _get_plane_coords(points[-1], plane)
        start_angle = math.atan2(sv - cv, su - cu)
        end_angle = math.atan2(ev - cv, eu - cu)

        sweep_ccw = end_angle - start_angle
        if sweep_ccw < 0:
            sweep_ccw += 2 * math.pi
        sweep_cw = sweep_ccw - 2 * math.pi

        # Determine winding via cross product of chords
        if len(points) >= 3:
            mu, mv = _get_plane_coords(points[1], plane)
            cross = (mu - su) * (ev - sv) - (mv - sv) * (eu - su)
            sweep = sweep_ccw if cross >= 0 else sweep_cw
        else:
            sweep = sweep_ccw

        # Max deviation
        max_dev = 0.0
        for p in points:
            pu, pv = _get_plane_coords(p, plane)
            dist = math.sqrt((pu - cu) ** 2 + (pv - cv) ** 2)
            dev = abs(dist - radius)
            if dev > max_dev:
                max_dev = dev

        return ArcCandidate(
            center=center_3d,
            radius=radius,
            start_angle=start_angle,
            sweep_angle=sweep,
            plane=plane,
            start_point=points[0],
            end_point=points[-1],
            max_deviation=max_dev,
        )

    # ── High-level optimisation ───────────────────────────────────────

    def optimize_tool_path(self, points, tolerance=None):
        """Replace runs of coplanar collinear G1 points with arc segments.
        Returns list of PathSegment."""
        if tolerance is not None:
            self.tolerance = tolerance
        tol = self.tolerance

        segments = []
        if not points or len(points) < 2:
            return segments

        n = len(points)
        i = 0

        while i < n - 1:
            best_end = -1
            best_candidate = None

            if i + self.min_points_for_arc - 1 < n:
                for end in range(i + self.min_points_for_arc - 1, n):
                    window = points[i:end + 1]

                    plane = self.detect_plane(window)
                    if plane is None:
                        break

                    fit = self.fit_circle(window, plane)
                    if fit is None:
                        break

                    cu, cv, r = fit
                    candidate = self.build_candidate(window, plane, cu, cv, r)

                    if candidate.max_deviation <= tol:
                        best_end = end
                        best_candidate = candidate
                    else:
                        break

            if best_end is not None and best_end >= 0 and best_candidate is not None:
                segments.append(PathSegment(
                    segment_type=PathSegmentType.ARC,
                    start_point=best_candidate.start_point,
                    end_point=best_candidate.end_point,
                    center=best_candidate.center,
                    radius=best_candidate.radius,
                    is_clockwise=best_candidate.sweep_angle < 0,
                ))
                i = best_end
            else:
                segments.append(PathSegment(
                    segment_type=PathSegmentType.LINE,
                    start_point=points[i],
                    end_point=points[i + 1],
                ))
                i += 1

        return segments

    def analyze(self, points, tolerance=None):
        """Run optimize_tool_path and return an ArcFitResult with metrics."""
        if tolerance is not None:
            self.tolerance = tolerance

        result = ArcFitResult()
        if not points or len(points) < 2:
            return result

        result.original_segment_count = len(points) - 1
        segments = self.optimize_tool_path(points, self.tolerance)

        arc_count = 0
        total_dev = 0.0

        idx = 0
        for seg in segments:
            if seg.segment_type == PathSegmentType.ARC:
                arc_count += 1
                arc_start = idx
                arc_end = arc_start
                for j in range(arc_start, len(points)):
                    if points[j] == seg.end_point:
                        arc_end = j
                        break

                window = points[arc_start:arc_end + 1]
                plane = self.detect_plane(window)
                if plane is not None:
                    fit = self.fit_circle(window, plane)
                    if fit is not None:
                        cu, cv, r = fit
                        cand = self.build_candidate(window, plane, cu, cv, r)
                        total_dev += cand.max_deviation
                idx = arc_end
            else:
                idx += 1

        result.fitted_arc_count = arc_count
        result.total_deviation = total_dev

        if result.original_segment_count > 0:
            result.line_reduction_pct = (
                (1.0 - len(segments) / result.original_segment_count) * 100.0
            )

        return result


# ---- Helper: 3x3 determinant ----

def _det3(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
          - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
          + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def _det3_substituted(m, b, col):
    tmp = [row[:] for row in m]
    for r in range(3):
        tmp[r][col] = b[r]
    return _det3(tmp)


# ---- Helper: generate circular arc points ----

def _arc_points_xy(cx, cy, r, start_deg, end_deg, n, z=0.0):
    """Generate n points along a circular arc in the XY plane."""
    pts = []
    for i in range(n):
        t = i / (n - 1) if n > 1 else 0.0
        angle = math.radians(start_deg + t * (end_deg - start_deg))
        pts.append((cx + r * math.cos(angle), cy + r * math.sin(angle), z))
    return pts


def _arc_points_xz(cx, cz, r, start_deg, end_deg, n, y=0.0):
    """Generate n points along a circular arc in the XZ plane."""
    pts = []
    for i in range(n):
        t = i / (n - 1) if n > 1 else 0.0
        angle = math.radians(start_deg + t * (end_deg - start_deg))
        pts.append((cx + r * math.cos(angle), y, cz + r * math.sin(angle)))
    return pts


# ---- Tests ----

@pytest.fixture
def optimizer():
    return ArcFittingOptimizer(tolerance=0.005, plane_tolerance=0.001)


class TestCircleFitting:
    """Circle fitting from known circular points."""

    def test_full_circle_recovery(self, optimizer):
        """Points sampled from a known circle should recover center and radius."""
        cx, cy, r = 10.0, 20.0, 5.0
        pts = _arc_points_xy(cx, cy, r, 0, 350, 36)

        fit = optimizer.fit_circle(pts, ArcPlane.XY)
        assert fit is not None
        cu, cv, fitted_r = fit
        assert cu == pytest.approx(cx, abs=0.01)
        assert cv == pytest.approx(cy, abs=0.01)
        assert fitted_r == pytest.approx(r, abs=0.01)

    def test_quarter_arc_recovery(self, optimizer):
        """Quarter-circle arc should also recover center and radius."""
        cx, cy, r = 0.0, 0.0, 10.0
        pts = _arc_points_xy(cx, cy, r, 0, 90, 20)

        fit = optimizer.fit_circle(pts, ArcPlane.XY)
        assert fit is not None
        cu, cv, fitted_r = fit
        assert cu == pytest.approx(cx, abs=0.05)
        assert cv == pytest.approx(cy, abs=0.05)
        assert fitted_r == pytest.approx(r, abs=0.05)


class TestArcDetectionXY:
    """Arc detection in XY plane."""

    def test_arc_segment_emitted(self, optimizer):
        """A run of circular points in XY should produce an ARC segment."""
        pts = _arc_points_xy(0, 0, 10.0, 0, 90, 20)
        segments = optimizer.optimize_tool_path(pts, tolerance=0.01)

        arc_segments = [s for s in segments if s.segment_type == PathSegmentType.ARC]
        assert len(arc_segments) >= 1

    def test_arc_center_and_radius(self, optimizer):
        """Emitted arc segment should have correct center and radius."""
        cx, cy, r = 5.0, 5.0, 8.0
        pts = _arc_points_xy(cx, cy, r, 0, 90, 15)
        segments = optimizer.optimize_tool_path(pts, tolerance=0.01)

        arc_seg = [s for s in segments if s.segment_type == PathSegmentType.ARC][0]
        # Center check (XY plane => center is (cu, cv, z))
        assert arc_seg.center[0] == pytest.approx(cx, abs=0.1)
        assert arc_seg.center[1] == pytest.approx(cy, abs=0.1)
        assert arc_seg.radius == pytest.approx(r, abs=0.1)


class TestToleranceRejection:
    """Tolerance rejection — deviation too high."""

    def test_noisy_points_rejected(self, optimizer):
        """Points with large radial noise exceeding tolerance should not form an arc."""
        cx, cy, r = 0.0, 0.0, 10.0
        pts = _arc_points_xy(cx, cy, r, 0, 90, 10)

        # Add large RADIAL noise to interior points — alternating inward/outward
        # so least-squares cannot absorb it by shifting center/radius.
        noisy_pts = list(pts)
        perturbations = [0, 0, 2.0, -1.5, 3.0, -2.0, 1.5, -3.0, 0, 0]
        for i in range(len(noisy_pts)):
            px, py, pz = noisy_pts[i]
            d = math.sqrt(px * px + py * py)
            if d > 1e-9 and perturbations[i] != 0:
                scale = (d + perturbations[i]) / d
                noisy_pts[i] = (px * scale, py * scale, pz)

        # With very tight tolerance, these wildly distorted points should be
        # rejected and fall back to line segments.
        segments = optimizer.optimize_tool_path(noisy_pts, tolerance=0.001)
        line_segments = [s for s in segments if s.segment_type == PathSegmentType.LINE]
        assert len(line_segments) > 0


class TestStraightLineNotFitted:
    """Straight line not fitted as arc."""

    def test_collinear_points_remain_lines(self, optimizer):
        """Perfectly collinear points should not be fitted as arcs."""
        pts = [(i * 1.0, 0.0, 0.0) for i in range(10)]
        segments = optimizer.optimize_tool_path(pts, tolerance=0.005)

        arc_segments = [s for s in segments if s.segment_type == PathSegmentType.ARC]
        assert len(arc_segments) == 0

        line_segments = [s for s in segments if s.segment_type == PathSegmentType.LINE]
        assert len(line_segments) == len(pts) - 1


class TestMixedPath:
    """Mixed path: lines + arcs."""

    def test_mixed_line_and_arc_segments(self, optimizer):
        """A path that starts with a straight section then curves should
        produce both LINE and ARC segments."""
        # Straight section: 5 points along X axis
        straight = [(i * 2.0, 0.0, 0.0) for i in range(5)]
        # Curved section: quarter circle
        arc = _arc_points_xy(8.0, 5.0, 5.0, 270, 360, 15, z=0.0)

        # Connect them
        pts = straight + arc

        segments = optimizer.optimize_tool_path(pts, tolerance=0.01)
        types = {s.segment_type for s in segments}
        assert PathSegmentType.LINE in types
        assert PathSegmentType.ARC in types


class TestXZPlaneDetection:
    """XZ plane detection."""

    def test_xz_plane_detected(self, optimizer):
        """Points lying in the XZ plane (constant Y) should be detected."""
        plane = optimizer.detect_plane([(1, 0, 0), (0, 0, 1), (-1, 0, 0)])
        assert plane == ArcPlane.XZ

    def test_arc_fitting_in_xz(self, optimizer):
        """An arc in the XZ plane should be fitted correctly."""
        pts = _arc_points_xz(0, 0, 10.0, 0, 90, 15, y=5.0)
        segments = optimizer.optimize_tool_path(pts, tolerance=0.01)

        arc_segments = [s for s in segments if s.segment_type == PathSegmentType.ARC]
        assert len(arc_segments) >= 1


class TestSmallSegmentHandling:
    """Small segment handling (< 3 points)."""

    def test_two_points_produce_line(self, optimizer):
        """Only 2 points cannot form an arc — must produce a single line."""
        pts = [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
        segments = optimizer.optimize_tool_path(pts, tolerance=0.005)

        assert len(segments) == 1
        assert segments[0].segment_type == PathSegmentType.LINE

    def test_single_point_produces_nothing(self, optimizer):
        """A single point (no segments) should return an empty list."""
        segments = optimizer.optimize_tool_path([(0, 0, 0)], tolerance=0.005)
        assert len(segments) == 0

    def test_empty_input(self, optimizer):
        """Empty input should return an empty list."""
        segments = optimizer.optimize_tool_path([], tolerance=0.005)
        assert len(segments) == 0


class TestLineReductionPercentage:
    """Line reduction percentage calculation."""

    def test_full_arc_replacement_reduction(self, optimizer):
        """Replacing many linear segments with a single arc should report
        significant line reduction."""
        pts = _arc_points_xy(0, 0, 10.0, 0, 90, 20)
        result = optimizer.analyze(pts, tolerance=0.01)

        # 19 original segments replaced by 1 arc => high reduction
        assert result.original_segment_count == 19
        assert result.fitted_arc_count >= 1
        assert result.line_reduction_pct > 50.0

    def test_no_reduction_for_lines(self, optimizer):
        """Collinear points have 0% reduction (all remain lines)."""
        pts = [(i * 1.0, 0.0, 0.0) for i in range(10)]
        result = optimizer.analyze(pts, tolerance=0.005)

        assert result.fitted_arc_count == 0
        assert result.line_reduction_pct == pytest.approx(0.0, abs=0.01)

    def test_total_deviation_within_tolerance(self, optimizer):
        """Total deviation across fitted arcs should be within tolerance."""
        pts = _arc_points_xy(0, 0, 10.0, 0, 180, 30)
        result = optimizer.analyze(pts, tolerance=0.01)

        if result.fitted_arc_count > 0:
            avg_dev = result.total_deviation / result.fitted_arc_count
            assert avg_dev <= 0.01
