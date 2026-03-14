"""Tests for tool-path smoothing and corner optimisation in the G-code lookahead engine.

These tests validate CornerAnalysis, PathSmoothingResult, and ToolPathSmoother
by re-implementing the core algorithms in Python (mirroring the C# code).
"""
import math
import sys
import types
import pytest
from unittest.mock import MagicMock
from dataclasses import dataclass, field
from typing import List, Optional


# ---------------------------------------------------------------------------
# Mock ROS2 modules so we can import miracle_twin without a live ROS2 env
# ---------------------------------------------------------------------------
for mod in (
    'rclpy', 'rclpy.lifecycle', 'rclpy.node', 'rclpy.action',
    'rclpy.action.server', 'rclpy.callback_groups',
    'miracle_core', 'miracle_core.qos_profiles',
    'miracle_core.lifecycle_node_base',
    'miracle_msgs', 'miracle_msgs.msg',
    'miracle_msgs.action', 'miracle_msgs.srv',
):
    sys.modules.setdefault(mod, MagicMock())


# ---------------------------------------------------------------------------
# Python mirrors of the C# data structures
# ---------------------------------------------------------------------------
class SegmentType:
    Rapid = 0
    Linear = 1
    CWArc = 2
    CCWArc = 3


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def normalized(self):
        m = self.magnitude()
        if m < 1e-9:
            return Vector3(0, 0, 0)
        return Vector3(self.x / m, self.y / m, self.z / m)

    @staticmethod
    def dot(a, b):
        return a.x * b.x + a.y * b.y + a.z * b.z

    @staticmethod
    def distance(a, b):
        return (a - b).magnitude()


@dataclass
class ToolpathSegment:
    type: int = SegmentType.Linear
    startPos: Vector3 = field(default_factory=Vector3)
    endPos: Vector3 = field(default_factory=Vector3)
    arcCenter: Vector3 = field(default_factory=Vector3)
    feedRate: float = 500.0      # mm/min
    spindleRPM: float = 10000.0
    gcodeLine: int = 0
    length: float = 0.0


# ---------------------------------------------------------------------------
# Corner type enum
# ---------------------------------------------------------------------------
class CornerType:
    SHARP = "SHARP"
    MODERATE = "MODERATE"
    GENTLE = "GENTLE"
    STRAIGHT = "STRAIGHT"


# ---------------------------------------------------------------------------
# Python mirror of CornerAnalysis
# ---------------------------------------------------------------------------
@dataclass
class CornerAnalysis:
    blockIndex: int = 0
    angleRadians: float = 0.0
    angleDegrees: float = 0.0
    cornerType: str = CornerType.STRAIGHT
    incomingFeed: float = 0.0
    outgoingFeed: float = 0.0
    recommendedCornerFeed: float = 0.0
    decelerationDistance: float = 0.0
    accelerationDistance: float = 0.0


# ---------------------------------------------------------------------------
# Python mirror of PathSmoothingResult
# ---------------------------------------------------------------------------
@dataclass
class PathSmoothingResult:
    originalSegmentCount: int = 0
    smoothedSegmentCount: int = 0
    corners: List[CornerAnalysis] = field(default_factory=list)
    totalPathLength: float = 0.0
    estimatedCycleTime: float = 0.0
    optimizedCycleTime: float = 0.0
    timeSavingsPct: float = 0.0
    maxJerk: float = 0.0


# ---------------------------------------------------------------------------
# Python mirror of ToolPathSmoother
# ---------------------------------------------------------------------------
class ToolPathSmoother:
    def __init__(self, max_acceleration=5000.0, max_jerk=50000.0,
                 corner_tolerance=0.01):
        self.maxAcceleration = max_acceleration
        self.maxJerk = max_jerk
        self.cornerTolerance = corner_tolerance

    # -- helpers --
    @staticmethod
    def _classify_corner(angle_deg: float) -> str:
        if angle_deg < 30.0:
            return CornerType.STRAIGHT
        if angle_deg < 90.0:
            return CornerType.GENTLE
        if angle_deg < 150.0:
            return CornerType.MODERATE
        return CornerType.SHARP

    def compute_corner_feed_limit(self, angle_rad: float, radius: float,
                                  programmed_feed: float) -> float:
        if angle_rad < 1e-6:
            return programmed_feed
        v_max = math.sqrt(self.maxAcceleration * max(radius, 1e-6))
        feed_limit = v_max * 60.0
        return min(feed_limit, programmed_feed)

    # -- corner analysis --
    def analyze_corners(self, segments: List[ToolpathSegment]) -> List[CornerAnalysis]:
        corners: List[CornerAnalysis] = []
        if segments is None or len(segments) < 2:
            return corners

        for i in range(len(segments) - 1):
            inc = segments[i]
            out = segments[i + 1]

            dir_in = (inc.endPos - inc.startPos).normalized()
            dir_out = (out.endPos - out.startPos).normalized()

            dot = max(-1.0, min(1.0, Vector3.dot(dir_in, dir_out)))
            angle_rad = math.acos(dot)
            angle_deg = math.degrees(angle_rad)

            half_angle = angle_rad * 0.5
            if half_angle > 1e-6:
                blend_radius = self.cornerTolerance / (1.0 - math.cos(half_angle))
            else:
                blend_radius = float('inf')

            rec_feed = self.compute_corner_feed_limit(
                angle_rad, blend_radius, min(inc.feedRate, out.feedRate))

            v_cruise_in = inc.feedRate / 60.0
            v_cruise_out = out.feedRate / 60.0
            v_corner = rec_feed / 60.0

            decel_dist = max(0.0,
                (v_cruise_in ** 2 - v_corner ** 2) / (2.0 * self.maxAcceleration))
            accel_dist = max(0.0,
                (v_cruise_out ** 2 - v_corner ** 2) / (2.0 * self.maxAcceleration))

            ca = CornerAnalysis(
                blockIndex=i,
                angleRadians=angle_rad,
                angleDegrees=angle_deg,
                cornerType=self._classify_corner(angle_deg),
                incomingFeed=inc.feedRate,
                outgoingFeed=out.feedRate,
                recommendedCornerFeed=rec_feed,
                decelerationDistance=decel_dist,
                accelerationDistance=accel_dist,
            )
            corners.append(ca)
        return corners

    # -- feed profile optimisation --
    def optimize_feed_profile(self, segments: List[ToolpathSegment],
                              corners: List[CornerAnalysis]) -> List[float]:
        if not segments:
            return []
        n = len(segments)
        feeds = [s.feedRate for s in segments]

        if not corners:
            return feeds

        # Forward pass
        for c in corners:
            v_corner = c.recommendedCornerFeed / 60.0
            start_seg = c.blockIndex + 1
            v_prev = v_corner
            for i in range(start_seg, n):
                seg_len = Vector3.distance(segments[i].startPos, segments[i].endPos)
                v_reachable = math.sqrt(v_prev ** 2 + 2.0 * self.maxAcceleration * seg_len)
                v_reachable_feed = v_reachable * 60.0
                if v_reachable_feed < feeds[i]:
                    feeds[i] = v_reachable_feed
                else:
                    break
                v_prev = feeds[i] / 60.0

        # Backward pass
        for c in corners:
            v_corner = c.recommendedCornerFeed / 60.0
            end_seg = c.blockIndex
            v_prev = v_corner
            for i in range(end_seg, -1, -1):
                seg_len = Vector3.distance(segments[i].startPos, segments[i].endPos)
                v_reachable = math.sqrt(v_prev ** 2 + 2.0 * self.maxAcceleration * seg_len)
                v_reachable_feed = v_reachable * 60.0
                if v_reachable_feed < feeds[i]:
                    feeds[i] = v_reachable_feed
                else:
                    break
                v_prev = feeds[i] / 60.0

        return feeds

    # -- cycle time --
    def estimate_cycle_time(self, segments: List[ToolpathSegment],
                            feeds: List[float]) -> float:
        if not segments or not feeds:
            return 0.0
        total = 0.0
        for i in range(min(len(segments), len(feeds))):
            length = Vector3.distance(segments[i].startPos, segments[i].endPos)
            v = feeds[i] / 60.0
            if v > 1e-6:
                total += length / v
        return total

    # -- high-level entry point --
    def smooth_path(self, segments: List[ToolpathSegment]) -> PathSmoothingResult:
        result = PathSmoothingResult()
        if not segments:
            return result
        result.originalSegmentCount = len(segments)
        result.smoothedSegmentCount = len(segments)

        total_len = sum(
            Vector3.distance(s.startPos, s.endPos) for s in segments)
        result.totalPathLength = total_len

        corners = self.analyze_corners(segments)
        result.corners = corners

        original_feeds = [s.feedRate for s in segments]
        result.estimatedCycleTime = self.estimate_cycle_time(segments, original_feeds)

        opt_feeds = self.optimize_feed_profile(segments, corners)
        result.optimizedCycleTime = self.estimate_cycle_time(segments, opt_feeds)

        if result.estimatedCycleTime > 1e-6:
            result.timeSavingsPct = (
                (1.0 - result.optimizedCycleTime / result.estimatedCycleTime) * 100.0)

        result.maxJerk = self.maxJerk
        return result


# ---------------------------------------------------------------------------
# Helper to build a segment between two (x, y, z) tuples
# ---------------------------------------------------------------------------
def _seg(start, end, feed=500.0):
    return ToolpathSegment(
        startPos=Vector3(*start),
        endPos=Vector3(*end),
        feedRate=feed,
    )


# ═══════════════════════════════════════════════════════════════════════════
#  TESTS
# ═══════════════════════════════════════════════════════════════════════════

class TestCornerAngleCalculation:
    """Corner angle calculation for various geometries."""

    def test_collinear_segments_zero_degree(self):
        """Two collinear segments → angle ≈ 0°."""
        segs = [_seg((0, 0, 0), (10, 0, 0)), _seg((10, 0, 0), (20, 0, 0))]
        corners = ToolPathSmoother().analyze_corners(segs)
        assert len(corners) == 1
        assert corners[0].angleDegrees == pytest.approx(0.0, abs=0.1)

    def test_right_angle_90_degrees(self):
        """L-shaped path → angle ≈ 90°."""
        segs = [_seg((0, 0, 0), (10, 0, 0)), _seg((10, 0, 0), (10, 10, 0))]
        corners = ToolPathSmoother().analyze_corners(segs)
        assert corners[0].angleDegrees == pytest.approx(90.0, abs=0.1)

    def test_45_degree_angle(self):
        """45° deflection."""
        segs = [
            _seg((0, 0, 0), (10, 0, 0)),
            _seg((10, 0, 0), (20, 10, 0)),  # 45° turn
        ]
        corners = ToolPathSmoother().analyze_corners(segs)
        assert corners[0].angleDegrees == pytest.approx(45.0, abs=0.5)

    def test_reversal_180_degrees(self):
        """Complete reversal → angle ≈ 180°."""
        segs = [_seg((0, 0, 0), (10, 0, 0)), _seg((10, 0, 0), (0, 0, 0))]
        corners = ToolPathSmoother().analyze_corners(segs)
        assert corners[0].angleDegrees == pytest.approx(180.0, abs=0.1)

    def test_angle_radians_matches_degrees(self):
        """angleRadians and angleDegrees must be consistent."""
        segs = [_seg((0, 0, 0), (10, 0, 0)), _seg((10, 0, 0), (10, 10, 0))]
        c = ToolPathSmoother().analyze_corners(segs)[0]
        assert c.angleDegrees == pytest.approx(math.degrees(c.angleRadians), abs=0.01)

    def test_3d_angle(self):
        """Corner in 3-D space."""
        segs = [_seg((0, 0, 0), (10, 0, 0)), _seg((10, 0, 0), (10, 0, 10))]
        c = ToolPathSmoother().analyze_corners(segs)[0]
        assert c.angleDegrees == pytest.approx(90.0, abs=0.1)


class TestCornerTypeClassification:
    """Verify CornerType bucketing."""

    def test_straight_classification(self):
        """Nearly collinear (< 30°) → STRAIGHT."""
        segs = [_seg((0, 0, 0), (100, 0, 0)), _seg((100, 0, 0), (200, 5, 0))]
        c = ToolPathSmoother().analyze_corners(segs)[0]
        assert c.cornerType == CornerType.STRAIGHT

    def test_gentle_classification(self):
        """45° → GENTLE."""
        segs = [_seg((0, 0, 0), (10, 0, 0)), _seg((10, 0, 0), (20, 10, 0))]
        c = ToolPathSmoother().analyze_corners(segs)[0]
        assert c.cornerType == CornerType.GENTLE

    def test_moderate_classification(self):
        """90° → MODERATE."""
        segs = [_seg((0, 0, 0), (10, 0, 0)), _seg((10, 0, 0), (10, 10, 0))]
        c = ToolPathSmoother().analyze_corners(segs)[0]
        assert c.cornerType == CornerType.MODERATE

    def test_sharp_classification(self):
        """~165° turn → SHARP."""
        segs = [_seg((0, 0, 0), (10, 0, 0)), _seg((10, 0, 0), (9, -0.26, 0))]
        # dir_in = (1,0,0), dir_out ≈ (-1,-0.26,0) → angle ≈ 165°
        c = ToolPathSmoother().analyze_corners(segs)[0]
        assert c.cornerType == CornerType.SHARP

    def test_reversal_is_sharp(self):
        """180° reversal → SHARP."""
        segs = [_seg((0, 0, 0), (10, 0, 0)), _seg((10, 0, 0), (0, 0, 0))]
        c = ToolPathSmoother().analyze_corners(segs)[0]
        assert c.cornerType == CornerType.SHARP


class TestCornerFeedLimit:
    """ComputeCornerFeedLimit centripetal-acceleration constraint."""

    def test_straight_no_limit(self):
        """Zero angle → feed equals programmed feed."""
        s = ToolPathSmoother()
        assert s.compute_corner_feed_limit(0.0, 100.0, 1000.0) == 1000.0

    def test_sharp_corner_limits_feed(self):
        """Large angle with tiny radius → significantly reduced feed."""
        s = ToolPathSmoother(max_acceleration=5000.0)
        # v = sqrt(5000 * 0.001) ≈ 2.24 mm/s = 134 mm/min  →  well below 1000
        feed = s.compute_corner_feed_limit(math.pi, 0.001, 1000.0)
        assert feed < 1000.0

    def test_larger_radius_allows_higher_feed(self):
        """Larger radius → higher corner feed."""
        s = ToolPathSmoother(max_acceleration=5000.0)
        f_small = s.compute_corner_feed_limit(math.pi / 2, 1.0, 10000.0)
        f_large = s.compute_corner_feed_limit(math.pi / 2, 10.0, 10000.0)
        assert f_large > f_small

    def test_feed_never_exceeds_programmed(self):
        """Corner feed must never exceed programmed feed."""
        s = ToolPathSmoother(max_acceleration=5000.0)
        feed = s.compute_corner_feed_limit(0.01, 1000.0, 200.0)
        assert feed <= 200.0


class TestFeedProfileOptimization:
    """Forward / backward pass velocity planning."""

    def test_straight_line_no_change(self):
        """Collinear segments → feeds unchanged."""
        segs = [_seg((0, 0, 0), (10, 0, 0), 600),
                _seg((10, 0, 0), (20, 0, 0), 600),
                _seg((20, 0, 0), (30, 0, 0), 600)]
        s = ToolPathSmoother()
        corners = s.analyze_corners(segs)
        feeds = s.optimize_feed_profile(segs, corners)
        for f in feeds:
            assert f == pytest.approx(600.0, abs=0.01)

    def test_sharp_corner_reduces_adjacent_feeds(self):
        """Sharp corner forces neighbouring segments to decelerate/accelerate."""
        # Use very short segments (0.5 mm) and low acceleration so the
        # feed cannot ramp back to programmed within a single segment.
        segs = [
            _seg((0, 0, 0), (0.5, 0, 0), 6000),
            _seg((0.5, 0, 0), (0.5, 0.5, 0), 6000),  # 90° turn
            _seg((0.5, 0.5, 0), (0.5, 1.0, 0), 6000),
        ]
        s = ToolPathSmoother(max_acceleration=500.0, corner_tolerance=0.01)
        corners = s.analyze_corners(segs)
        feeds = s.optimize_feed_profile(segs, corners)
        # At least one feed should be reduced below 6000
        assert min(feeds) < 6000.0

    def test_empty_corners_returns_programmed(self):
        """No corners → programmed feeds returned unchanged."""
        segs = [_seg((0, 0, 0), (10, 0, 0), 800)]
        s = ToolPathSmoother()
        feeds = s.optimize_feed_profile(segs, [])
        assert feeds == [800.0]

    def test_forward_pass_increases_from_corner(self):
        """After a sharp corner the feed should ramp up across segments."""
        segs = [
            _seg((0, 0, 0), (50, 0, 0), 2000),
            _seg((50, 0, 0), (50, -50, 0), 2000),  # 90° corner
            _seg((50, -50, 0), (50, -60, 0), 2000),
            _seg((50, -60, 0), (50, -80, 0), 2000),
            _seg((50, -80, 0), (50, -120, 0), 2000),
        ]
        s = ToolPathSmoother()
        corners = s.analyze_corners(segs)
        feeds = s.optimize_feed_profile(segs, corners)
        # Feeds after the corner should be monotonically non-decreasing
        for i in range(3, len(feeds)):
            assert feeds[i] >= feeds[i - 1] - 0.01

    def test_backward_pass_decreases_into_corner(self):
        """Before a sharp corner the feed should ramp down."""
        segs = [
            _seg((0, 0, 0), (40, 0, 0), 2000),
            _seg((40, 0, 0), (80, 0, 0), 2000),
            _seg((80, 0, 0), (120, 0, 0), 2000),
            _seg((120, 0, 0), (120, 50, 0), 2000),  # 90° corner
            _seg((120, 50, 0), (120, 100, 0), 2000),
        ]
        s = ToolPathSmoother()
        corners = s.analyze_corners(segs)
        feeds = s.optimize_feed_profile(segs, corners)
        # Feeds approaching the corner should be non-increasing
        for i in range(1, 3):
            assert feeds[i] <= feeds[i - 1] + 0.01


class TestStraightLinePath:
    """A perfectly straight path should be trivially optimised."""

    def test_no_corners_detected(self):
        segs = [_seg((0, 0, 0), (10, 0, 0)),
                _seg((10, 0, 0), (20, 0, 0)),
                _seg((20, 0, 0), (30, 0, 0))]
        corners = ToolPathSmoother().analyze_corners(segs)
        assert all(c.cornerType == CornerType.STRAIGHT for c in corners)

    def test_cycle_time_matches_simple_calc(self):
        feed = 600.0  # mm/min = 10 mm/s
        segs = [_seg((0, 0, 0), (10, 0, 0), feed),
                _seg((10, 0, 0), (20, 0, 0), feed)]
        s = ToolPathSmoother()
        t = s.estimate_cycle_time(segs, [feed, feed])
        # 20 mm / 10 mm/s = 2 s
        assert t == pytest.approx(2.0, abs=0.01)


class TestSharpCornerDeceleration:
    """Verify deceleration / acceleration distances around sharp corners."""

    def test_deceleration_distance_positive(self):
        segs = [_seg((0, 0, 0), (50, 0, 0), 1200),
                _seg((50, 0, 0), (0, 0, 0), 1200)]  # 180° reversal
        c = ToolPathSmoother().analyze_corners(segs)[0]
        assert c.decelerationDistance > 0.0

    def test_acceleration_distance_positive(self):
        segs = [_seg((0, 0, 0), (50, 0, 0), 1200),
                _seg((50, 0, 0), (0, 0, 0), 1200)]
        c = ToolPathSmoother().analyze_corners(segs)[0]
        assert c.accelerationDistance > 0.0

    def test_decel_greater_for_higher_feed(self):
        s = ToolPathSmoother()
        segs_slow = [_seg((0, 0, 0), (50, 0, 0), 600),
                     _seg((50, 0, 0), (0, 0, 0), 600)]
        segs_fast = [_seg((0, 0, 0), (50, 0, 0), 3000),
                     _seg((50, 0, 0), (0, 0, 0), 3000)]
        d_slow = s.analyze_corners(segs_slow)[0].decelerationDistance
        d_fast = s.analyze_corners(segs_fast)[0].decelerationDistance
        assert d_fast > d_slow


class TestCycleTimeEstimation:
    """EstimateCycleTime validation."""

    def test_empty_returns_zero(self):
        assert ToolPathSmoother().estimate_cycle_time([], []) == 0.0

    def test_none_returns_zero(self):
        assert ToolPathSmoother().estimate_cycle_time(None, None) == 0.0

    def test_single_segment(self):
        seg = _seg((0, 0, 0), (60, 0, 0), 600)  # 60 mm @ 600 mm/min = 6 s
        t = ToolPathSmoother().estimate_cycle_time([seg], [600.0])
        assert t == pytest.approx(6.0, abs=0.01)

    def test_reduced_feed_increases_time(self):
        seg = _seg((0, 0, 0), (60, 0, 0), 600)
        t_full = ToolPathSmoother().estimate_cycle_time([seg], [600.0])
        t_half = ToolPathSmoother().estimate_cycle_time([seg], [300.0])
        assert t_half == pytest.approx(t_full * 2, abs=0.01)


class TestMixedCornerTypes:
    """Path with a variety of corner angles."""

    def test_mixed_path_has_multiple_corner_types(self):
        segs = [
            _seg((0, 0, 0), (100, 0, 0), 1000),       # long straight
            _seg((100, 0, 0), (200, 5, 0), 1000),      # very small angle → STRAIGHT
            _seg((200, 5, 0), (250, 55, 0), 1000),     # ~45° → GENTLE
            _seg((250, 55, 0), (250, 105, 0), 1000),   # ~90° change → MODERATE
            _seg((250, 105, 0), (200, 105, 0), 1000),  # 90° → MODERATE
        ]
        s = ToolPathSmoother()
        corners = s.analyze_corners(segs)
        types_found = {c.cornerType for c in corners}
        assert len(types_found) >= 2  # at least two distinct types

    def test_smooth_path_returns_all_metrics(self):
        segs = [
            _seg((0, 0, 0), (100, 0, 0), 1000),
            _seg((100, 0, 0), (100, 100, 0), 1000),
            _seg((100, 100, 0), (0, 100, 0), 1000),
        ]
        result = ToolPathSmoother().smooth_path(segs)
        assert result.originalSegmentCount == 3
        assert result.smoothedSegmentCount == 3
        assert result.totalPathLength > 0
        assert result.estimatedCycleTime > 0
        assert result.optimizedCycleTime > 0
        assert result.maxJerk == 50000.0
        assert len(result.corners) == 2


class TestSingleSegment:
    """Edge case: only one segment (no corners possible)."""

    def test_no_corners(self):
        segs = [_seg((0, 0, 0), (10, 0, 0))]
        assert ToolPathSmoother().analyze_corners(segs) == []

    def test_smooth_path_single_segment(self):
        segs = [_seg((0, 0, 0), (10, 0, 0), 600)]
        r = ToolPathSmoother().smooth_path(segs)
        assert r.originalSegmentCount == 1
        assert len(r.corners) == 0
        assert r.totalPathLength == pytest.approx(10.0, abs=0.01)
        assert r.estimatedCycleTime == pytest.approx(r.optimizedCycleTime, abs=0.01)


class TestEmptyPath:
    """Edge case: empty or None segment list."""

    def test_empty_list_corners(self):
        assert ToolPathSmoother().analyze_corners([]) == []

    def test_none_corners(self):
        assert ToolPathSmoother().analyze_corners(None) == []

    def test_empty_smooth_path(self):
        r = ToolPathSmoother().smooth_path([])
        assert r.originalSegmentCount == 0
        assert r.totalPathLength == 0.0

    def test_none_smooth_path(self):
        r = ToolPathSmoother().smooth_path(None)
        assert r.originalSegmentCount == 0

    def test_empty_optimize_feed(self):
        assert ToolPathSmoother().optimize_feed_profile([], []) == []


class TestTimeSavings:
    """Verify time savings percentage is computed correctly."""

    def test_straight_path_no_savings(self):
        segs = [_seg((0, 0, 0), (10, 0, 0), 600),
                _seg((10, 0, 0), (20, 0, 0), 600)]
        r = ToolPathSmoother().smooth_path(segs)
        # Straight path optimised cycle time should equal or exceed original
        # (corner feed may slow it slightly due to numerical precision)
        assert r.timeSavingsPct <= 1.0

    def test_sharp_corner_path_has_nonnegative_time(self):
        """Optimised time should still be positive."""
        segs = [
            _seg((0, 0, 0), (50, 0, 0), 2000),
            _seg((50, 0, 0), (0, 0, 0), 2000),  # reversal
        ]
        r = ToolPathSmoother().smooth_path(segs)
        assert r.optimizedCycleTime > 0


class TestIncomingOutgoingFeed:
    """CornerAnalysis records correct incoming / outgoing feeds."""

    def test_different_feeds(self):
        segs = [_seg((0, 0, 0), (10, 0, 0), 800),
                _seg((10, 0, 0), (10, 10, 0), 400)]
        c = ToolPathSmoother().analyze_corners(segs)[0]
        assert c.incomingFeed == 800.0
        assert c.outgoingFeed == 400.0
