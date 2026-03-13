"""Tests for fixture collision detection in the G-code lookahead engine.

These tests validate the FixtureCollisionProfile, CollisionCheckResult,
CheckFixtureCollisions, and GenerateCollisionAvoidancePath logic by
re-implementing the core algorithms in Python (mirroring the C# code).
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
# Python mirror of the C# data structures under test
# ---------------------------------------------------------------------------
class SegmentType:
    Rapid = 0    # G00
    Linear = 1   # G01
    CWArc = 2    # G02
    CCWArc = 3   # G03


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


@dataclass
class ToolpathSegment:
    type: int = SegmentType.Linear
    startPos: Vector3 = field(default_factory=Vector3)
    endPos: Vector3 = field(default_factory=Vector3)
    arcCenter: Vector3 = field(default_factory=Vector3)
    feedRate: float = 500.0
    spindleRPM: float = 10000.0
    gcodeLine: int = 0
    length: float = 0.0


@dataclass
class LookaheadResult:
    segmentIndex: int = 0
    collisionFlag: bool = False


@dataclass
class FixtureCollisionProfile:
    boundsMin: Vector3 = field(default_factory=Vector3)
    boundsMax: Vector3 = field(default_factory=Vector3)
    clampZones: list = field(default_factory=list)
    safeRetractHeight: float = 50.0

    def contains_point(self, point: Vector3, margin: float = 0.0) -> bool:
        return (point.x >= self.boundsMin.x - margin and point.x <= self.boundsMax.x + margin
                and point.y >= self.boundsMin.y - margin and point.y <= self.boundsMax.y + margin
                and point.z >= self.boundsMin.z - margin and point.z <= self.boundsMax.z + margin)

    def distance_to_surface(self, point: Vector3) -> float:
        dx = max(self.boundsMin.x - point.x, point.x - self.boundsMax.x)
        dy = max(self.boundsMin.y - point.y, point.y - self.boundsMax.y)
        dz = max(self.boundsMin.z - point.z, point.z - self.boundsMax.z)
        if dx < 0 and dy < 0 and dz < 0:
            return max(dx, dy, dz)
        cx = max(dx, 0.0)
        cy = max(dy, 0.0)
        cz = max(dz, 0.0)
        return math.sqrt(cx * cx + cy * cy + cz * cz)


@dataclass
class CollisionCheckResult:
    hasCollision: bool = False
    collisionBlockIndex: int = -1
    collisionPoint: Vector3 = field(default_factory=Vector3)
    clearanceDistance: float = float('inf')
    recommendation: str = ""


# ---------------------------------------------------------------------------
# Python mirror of CheckFixtureCollisions / GenerateCollisionAvoidancePath
# ---------------------------------------------------------------------------
RAPID_SAFETY_MARGIN = 2.0
ARC_COLLISION_SAMPLES = 8


def _add_arc_sample_points(seg: ToolpathSegment) -> List[Vector3]:
    center = seg.arcCenter
    start_off = seg.startPos - center
    end_off = seg.endPos - center
    start_angle = math.atan2(start_off.z, start_off.x)
    end_angle = math.atan2(end_off.z, end_off.x)
    radius = start_off.magnitude()

    if seg.type == SegmentType.CWArc:
        sweep = start_angle - end_angle
        if sweep <= 0:
            sweep += 2 * math.pi
        sweep = -sweep
    else:
        sweep = end_angle - start_angle
        if sweep <= 0:
            sweep += 2 * math.pi

    z_start = seg.startPos.y
    z_end = seg.endPos.y
    points = []
    for s in range(1, ARC_COLLISION_SAMPLES + 1):
        t = s / (ARC_COLLISION_SAMPLES + 1)
        angle = start_angle + sweep * t
        y = z_start + (z_end - z_start) * t
        pt = Vector3(
            center.x + radius * math.cos(angle),
            y,
            center.z + radius * math.sin(angle),
        )
        points.append(pt)
    return points


def check_fixture_collisions(
    results: Optional[list],
    fixture: Optional[FixtureCollisionProfile],
    segments: Optional[list] = None,
) -> CollisionCheckResult:
    output = CollisionCheckResult()
    if not results or fixture is None:
        return output

    for i, r in enumerate(results):
        seg = None
        has_seg = (segments is not None
                   and 0 <= r.segmentIndex < len(segments))
        if has_seg:
            seg = segments[r.segmentIndex]

        move_type = seg.type if has_seg else SegmentType.Linear
        margin = RAPID_SAFETY_MARGIN if move_type == SegmentType.Rapid else 0.0

        points: List[Vector3] = []
        if has_seg:
            points.append(seg.startPos)
            points.append(seg.endPos)
            if move_type in (SegmentType.CWArc, SegmentType.CCWArc):
                points.extend(_add_arc_sample_points(seg))
        else:
            continue

        for pt in points:
            dist = fixture.distance_to_surface(pt)
            eff = dist - margin
            if eff < output.clearanceDistance:
                output.clearanceDistance = eff

            if fixture.contains_point(pt, margin):
                output.hasCollision = True
                output.collisionBlockIndex = i
                output.collisionPoint = pt
                if move_type == SegmentType.Rapid:
                    label = "rapid move (G00)"
                elif move_type in (SegmentType.CWArc, SegmentType.CCWArc):
                    label = "arc move"
                else:
                    label = "cutting move"
                output.recommendation = (
                    f"Collision detected at block {i} during {label}. "
                    f"Retract to safe height Z={fixture.safeRetractHeight:.1f} mm "
                    f"before traversing fixture zone."
                )
                return output

    return output


def generate_collision_avoidance_path(
    collision_block: int,
    fixture: Optional[FixtureCollisionProfile],
    segments: Optional[list] = None,
    cached_results: Optional[list] = None,
) -> List[Vector3]:
    waypoints: List[Vector3] = []
    if fixture is None or collision_block < 0:
        return waypoints

    start = Vector3()
    end = Vector3()
    if (segments is not None
            and cached_results is not None
            and collision_block < len(cached_results)):
        seg_idx = cached_results[collision_block].segmentIndex
        if 0 <= seg_idx < len(segments):
            start = segments[seg_idx].startPos
            end = segments[seg_idx].endPos

    safe_z = fixture.safeRetractHeight
    waypoints.append(Vector3(start.x, safe_z, start.z))
    waypoints.append(Vector3(end.x, safe_z, end.z))
    waypoints.append(end)
    return waypoints


# ---------------------------------------------------------------------------
# Helpers to build test data
# ---------------------------------------------------------------------------
def _fixture_at(min_v, max_v, safe_z=50.0):
    return FixtureCollisionProfile(
        boundsMin=Vector3(*min_v),
        boundsMax=Vector3(*max_v),
        safeRetractHeight=safe_z,
    )


def _seg(start, end, seg_type=SegmentType.Linear, arc_center=(0, 0, 0)):
    return ToolpathSegment(
        type=seg_type,
        startPos=Vector3(*start),
        endPos=Vector3(*end),
        arcCenter=Vector3(*arc_center),
    )


def _result(idx):
    return LookaheadResult(segmentIndex=idx)


# ===================================================================
# TESTS
# ===================================================================

class TestFixtureCollisionProfileContains:
    """FixtureCollisionProfile.contains_point basic behaviour."""

    def test_point_inside(self):
        fp = _fixture_at((0, 0, 0), (10, 10, 10))
        assert fp.contains_point(Vector3(5, 5, 5))

    def test_point_outside(self):
        fp = _fixture_at((0, 0, 0), (10, 10, 10))
        assert not fp.contains_point(Vector3(15, 5, 5))

    def test_point_on_boundary(self):
        fp = _fixture_at((0, 0, 0), (10, 10, 10))
        assert fp.contains_point(Vector3(10, 10, 10))

    def test_point_outside_with_margin_brings_inside(self):
        fp = _fixture_at((0, 0, 0), (10, 10, 10))
        assert fp.contains_point(Vector3(11, 5, 5), margin=1.5)

    def test_point_outside_margin_not_enough(self):
        fp = _fixture_at((0, 0, 0), (10, 10, 10))
        assert not fp.contains_point(Vector3(12, 5, 5), margin=1.0)


class TestFixtureDistanceToSurface:
    """FixtureCollisionProfile.distance_to_surface."""

    def test_point_outside_positive_distance(self):
        fp = _fixture_at((0, 0, 0), (10, 10, 10))
        d = fp.distance_to_surface(Vector3(13, 5, 5))
        assert d == pytest.approx(3.0)

    def test_point_inside_negative_distance(self):
        fp = _fixture_at((0, 0, 0), (10, 10, 10))
        d = fp.distance_to_surface(Vector3(5, 5, 5))
        assert d < 0

    def test_point_on_surface_zero_distance(self):
        fp = _fixture_at((0, 0, 0), (10, 10, 10))
        d = fp.distance_to_surface(Vector3(10, 5, 5))
        assert d == pytest.approx(0.0)

    def test_corner_distance(self):
        fp = _fixture_at((0, 0, 0), (10, 10, 10))
        d = fp.distance_to_surface(Vector3(13, 14, 10))
        assert d == pytest.approx(5.0)


class TestCheckFixtureCollisionsNoCollision:
    """Cases where no collision should be detected."""

    def test_path_outside_fixture(self):
        fixture = _fixture_at((20, 20, 20), (30, 30, 30))
        seg = _seg((0, 0, 0), (10, 0, 0))
        res = check_fixture_collisions(
            [_result(0)], fixture, [seg])
        assert not res.hasCollision
        assert res.collisionBlockIndex == -1

    def test_empty_results_returns_no_collision(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        res = check_fixture_collisions([], fixture, [])
        assert not res.hasCollision

    def test_none_results_returns_no_collision(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        res = check_fixture_collisions(None, fixture)
        assert not res.hasCollision

    def test_none_fixture_returns_no_collision(self):
        seg = _seg((5, 5, 5), (6, 6, 6))
        res = check_fixture_collisions([_result(0)], None, [seg])
        assert not res.hasCollision


class TestCheckFixtureCollisionsRapidMove:
    """Rapid (G00) moves with 2 mm safety margin."""

    def test_rapid_collision_inside_fixture(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        seg = _seg((5, 5, 5), (6, 6, 6), SegmentType.Rapid)
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert res.hasCollision
        assert "rapid move (G00)" in res.recommendation

    def test_rapid_collision_within_safety_margin(self):
        """A rapid whose endpoint is 1 mm outside the fixture still collides
        because of the 2 mm safety margin."""
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        seg = _seg((15, 15, 15), (11, 5, 5), SegmentType.Rapid)
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert res.hasCollision
        assert res.collisionBlockIndex == 0

    def test_rapid_outside_margin_no_collision(self):
        """A rapid whose closest point is > 2 mm away does not collide."""
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        seg = _seg((15, 5, 5), (13, 5, 5), SegmentType.Rapid)
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert not res.hasCollision


class TestCheckFixtureCollisionsCuttingMove:
    """Linear cutting (G01) moves — no extra margin."""

    def test_cutting_start_inside(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        seg = _seg((5, 5, 5), (15, 5, 5), SegmentType.Linear)
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert res.hasCollision
        assert "cutting move" in res.recommendation

    def test_cutting_end_inside(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        seg = _seg((15, 5, 5), (5, 5, 5), SegmentType.Linear)
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert res.hasCollision

    def test_cutting_fully_outside(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        seg = _seg((15, 15, 15), (20, 20, 20), SegmentType.Linear)
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert not res.hasCollision


class TestCheckFixtureCollisionsArcMove:
    """Arc (G02/G03) collision sampling at 8 intermediate points."""

    def test_cw_arc_collision(self):
        """CW arc from (5,0,0) to (0,0,5) sweeps 270 degrees through
        quadrants 4, 3, and 2. We place a fixture that catches the arc
        as it passes through (0, 0, -5) at the bottom of its sweep."""
        fixture = _fixture_at((-1, -1, -6), (1, 1, -4))
        seg = _seg(
            (5, 0, 0), (0, 0, 5),
            seg_type=SegmentType.CWArc,
            arc_center=(0, 0, 0),
        )
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert res.hasCollision
        assert "arc move" in res.recommendation

    def test_ccw_arc_no_collision(self):
        """Arc that stays far from fixture."""
        fixture = _fixture_at((100, 100, 100), (110, 110, 110))
        seg = _seg(
            (5, 0, 0), (0, 0, 5),
            seg_type=SegmentType.CCWArc,
            arc_center=(0, 0, 0),
        )
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert not res.hasCollision

    def test_arc_sampling_generates_8_points(self):
        """Verify that 8 sample points are generated for an arc segment."""
        seg = _seg(
            (5, 0, 0), (0, 0, 5),
            seg_type=SegmentType.CWArc,
            arc_center=(0, 0, 0),
        )
        pts = _add_arc_sample_points(seg)
        assert len(pts) == 8


class TestClearanceDistance:
    """Minimum clearance distance tracking."""

    def test_clearance_positive_when_outside(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        seg = _seg((15, 5, 5), (20, 5, 5))
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert res.clearanceDistance > 0
        assert res.clearanceDistance == pytest.approx(5.0)

    def test_clearance_negative_when_inside(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        seg = _seg((5, 5, 5), (6, 6, 6))
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert res.clearanceDistance < 0

    def test_clearance_accounts_for_rapid_margin(self):
        """For rapids, effective clearance = distance - 2mm margin."""
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        seg = _seg((13, 5, 5), (14, 5, 5), SegmentType.Rapid)
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        # Closest point is (13,5,5); distance_to_surface = 3, minus 2mm margin = 1
        assert res.clearanceDistance == pytest.approx(1.0)


class TestSafeRetractRecommendation:
    """Recommendation string includes safe retract height."""

    def test_recommendation_includes_safe_height(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10), safe_z=75.0)
        seg = _seg((5, 5, 5), (6, 6, 6))
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert "75.0" in res.recommendation
        assert "Retract to safe height" in res.recommendation

    def test_recommendation_mentions_block_index(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        segs = [
            _seg((50, 50, 50), (60, 60, 60)),
            _seg((5, 5, 5), (6, 6, 6)),
        ]
        results = [_result(0), _result(1)]
        res = check_fixture_collisions(results, fixture, segs)
        assert "block 1" in res.recommendation


class TestMultipleCollisionsFirstReported:
    """When multiple blocks collide, only the first is reported."""

    def test_first_collision_reported(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        segs = [
            _seg((5, 5, 5), (6, 6, 6)),
            _seg((7, 7, 7), (8, 8, 8)),
        ]
        results = [_result(0), _result(1)]
        res = check_fixture_collisions(results, fixture, segs)
        assert res.hasCollision
        assert res.collisionBlockIndex == 0


class TestGenerateCollisionAvoidancePath:
    """GenerateCollisionAvoidancePath waypoints."""

    def test_basic_avoidance_path(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10), safe_z=60.0)
        segs = [_seg((5, 2, 5), (8, 2, 8))]
        cached = [_result(0)]
        path = generate_collision_avoidance_path(0, fixture, segs, cached)
        assert len(path) == 3
        # Retract point: same XZ as start, Y = safeZ
        assert path[0].x == pytest.approx(5.0)
        assert path[0].y == pytest.approx(60.0)
        assert path[0].z == pytest.approx(5.0)
        # Traverse point: target XZ, Y = safeZ
        assert path[1].x == pytest.approx(8.0)
        assert path[1].y == pytest.approx(60.0)
        assert path[1].z == pytest.approx(8.0)
        # Plunge to target
        assert path[2].x == pytest.approx(8.0)
        assert path[2].y == pytest.approx(2.0)
        assert path[2].z == pytest.approx(8.0)

    def test_avoidance_path_none_fixture(self):
        path = generate_collision_avoidance_path(0, None)
        assert path == []

    def test_avoidance_path_negative_block(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        path = generate_collision_avoidance_path(-1, fixture)
        assert path == []

    def test_avoidance_path_no_segments(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10), safe_z=40.0)
        path = generate_collision_avoidance_path(0, fixture, None, [_result(0)])
        # No segments means start/end default to origin
        assert len(path) == 3
        assert path[0].y == pytest.approx(40.0)


class TestEdgeCases:
    """Edge cases and boundary conditions."""

    def test_segment_index_out_of_range(self):
        """LookaheadResult references a segment beyond the list."""
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        res = check_fixture_collisions(
            [LookaheadResult(segmentIndex=99)], fixture, [_seg((5, 5, 5), (6, 6, 6))])
        # segmentIndex 99 is out of range; block is skipped
        assert not res.hasCollision

    def test_no_segments_provided(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        res = check_fixture_collisions([_result(0)], fixture, None)
        assert not res.hasCollision

    def test_fixture_degenerate_zero_size(self):
        """Zero-size fixture at a point."""
        fixture = _fixture_at((5, 5, 5), (5, 5, 5))
        seg = _seg((5, 5, 5), (5, 5, 5))
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert res.hasCollision

    def test_large_number_of_blocks(self):
        """Stress-test with many blocks, collision at block 50."""
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        segs = []
        results = []
        for i in range(100):
            if i == 50:
                segs.append(_seg((5, 5, 5), (6, 6, 6)))
            else:
                segs.append(_seg((50 + i, 50, 50), (51 + i, 50, 50)))
            results.append(_result(i))
        res = check_fixture_collisions(results, fixture, segs)
        assert res.hasCollision
        assert res.collisionBlockIndex == 50

    def test_collision_point_is_recorded(self):
        fixture = _fixture_at((0, 0, 0), (10, 10, 10))
        seg = _seg((5, 5, 5), (15, 15, 15))
        res = check_fixture_collisions([_result(0)], fixture, [seg])
        assert res.hasCollision
        assert res.collisionPoint.x == pytest.approx(5.0)
        assert res.collisionPoint.y == pytest.approx(5.0)
        assert res.collisionPoint.z == pytest.approx(5.0)
