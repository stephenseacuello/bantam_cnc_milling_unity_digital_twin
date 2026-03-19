"""Tests for LeadInOutGenerator logic (Python mirror of C# implementation).

Validates lead-in / lead-out move generation for LINEAR, ARC, TANGENTIAL,
RAMP, and HELICAL approach types, helical pocket entry, config validation,
and material/operation-based recommendation heuristics.
"""

import math
import pytest
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import Enum


# ── Python mirror of C# types ────────────────────────────────────

class ApproachType(Enum):
    LINEAR = "LINEAR"
    ARC = "ARC"
    TANGENTIAL = "TANGENTIAL"
    RAMP = "RAMP"
    HELICAL = "HELICAL"


@dataclass
class LeadMove:
    start_point: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    end_point: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    approach: ApproachType = ApproachType.LINEAR
    arc_center: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    radius: float = 0.0
    ramp_angle_deg: float = 0.0


@dataclass
class LeadConfig:
    approach: ApproachType = ApproachType.ARC
    radius: float = 5.0
    ramp_angle: float = 3.0
    overlap_pct: float = 10.0
    helical_pitch: float = 2.0


# ── Helper math ──────────────────────────────────────────────────

def _vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _vec_length(a):
    return math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)


def _vec_normalize(a):
    ln = _vec_length(a)
    if ln < 1e-9:
        return (1.0, 0.0, 0.0)
    return (a[0] / ln, a[1] / ln, a[2] / ln)


def _vec_dist(a, b):
    return _vec_length(_vec_sub(a, b))


# ── Python mirror of LeadInOutGenerator ──────────────────────────

class LeadInOutGenerator:

    def validate_config(self, config: LeadConfig) -> List[str]:
        errors: List[str] = []
        if config is None:
            return ["Config is null"]
        if config.radius <= 0:
            errors.append("Radius must be > 0")
        if config.ramp_angle < 0.1 or config.ramp_angle > 89.9:
            errors.append("RampAngle must be between 0.1 and 89.9 degrees")
        if config.overlap_pct < 0 or config.overlap_pct > 100:
            errors.append("OverlapPct must be between 0 and 100")
        if config.helical_pitch <= 0:
            errors.append("HelicalPitch must be > 0")
        return errors

    # ── Lead-in ──────────────────────────────────────────────────

    def generate_lead_in(self, entry_point, cut_direction, config: LeadConfig) -> List[LeadMove]:
        d = _vec_normalize(cut_direction)
        moves: List[LeadMove] = []

        if config.approach == ApproachType.LINEAR:
            moves.append(self._build_linear(entry_point, d, config.radius, is_lead_in=True))
        elif config.approach == ApproachType.ARC:
            moves.append(self._build_arc(entry_point, d, config.radius, is_lead_in=True))
        elif config.approach == ApproachType.TANGENTIAL:
            moves.append(self._build_tangential(entry_point, d, config.radius, is_lead_in=True))
        elif config.approach == ApproachType.RAMP:
            moves.append(self._build_ramp(entry_point, d, config.radius,
                                          config.ramp_angle, is_lead_in=True))
        elif config.approach == ApproachType.HELICAL:
            moves.extend(self._build_helical(entry_point, config.radius,
                                             config.helical_pitch, is_lead_in=True))
        return moves

    # ── Lead-out ─────────────────────────────────────────────────

    def generate_lead_out(self, exit_point, cut_direction, config: LeadConfig) -> List[LeadMove]:
        d = _vec_normalize(cut_direction)
        moves: List[LeadMove] = []

        if config.approach == ApproachType.LINEAR:
            moves.append(self._build_linear(exit_point, d, config.radius, is_lead_in=False))
        elif config.approach == ApproachType.ARC:
            moves.append(self._build_arc(exit_point, d, config.radius, is_lead_in=False))
        elif config.approach == ApproachType.TANGENTIAL:
            moves.append(self._build_tangential(exit_point, d, config.radius, is_lead_in=False))
        elif config.approach == ApproachType.RAMP:
            moves.append(self._build_ramp(exit_point, d, config.radius,
                                          config.ramp_angle, is_lead_in=False))
        elif config.approach == ApproachType.HELICAL:
            moves.extend(self._build_helical(exit_point, config.radius,
                                             config.helical_pitch, is_lead_in=False))
        return moves

    # ── Helical pocket entry ─────────────────────────────────────

    def generate_helical_entry(self, center, target_depth: float,
                               diameter: float, pitch: float) -> List[LeadMove]:
        moves: List[LeadMove] = []
        radius = diameter * 0.5
        if radius <= 0 or pitch <= 0:
            return moves

        total_descent = abs(target_depth)
        revolutions = total_descent / pitch
        segments = max(8, math.ceil(revolutions * 16))
        angle_step = revolutions * 2.0 * math.pi / segments
        z_step = total_descent / segments

        prev = _vec_add(center, (radius, 0.0, 0.0))
        for i in range(1, segments + 1):
            angle = i * angle_step
            z = -i * z_step
            nxt = _vec_add(center, (radius * math.cos(angle),
                                    radius * math.sin(angle), z))
            moves.append(LeadMove(
                start_point=prev,
                end_point=nxt,
                approach=ApproachType.HELICAL,
                arc_center=center,
                radius=radius,
            ))
            prev = nxt
        return moves

    # ── Recommendation heuristic ─────────────────────────────────

    def get_recommended_approach(self, material: str, operation: str) -> ApproachType:
        mat = (material or "").upper()
        op = (operation or "").upper()

        if "POCKET" in op:
            return ApproachType.HELICAL
        if "DRILL" in op or "BORE" in op:
            return ApproachType.RAMP
        if "TITANIUM" in mat or "INCONEL" in mat or "HARDENED" in mat or "STAINLESS" in mat:
            return ApproachType.ARC
        if "PROFILE" in op or "CONTOUR" in op:
            return ApproachType.TANGENTIAL
        return ApproachType.LINEAR

    # ── Private builders ─────────────────────────────────────────

    def _build_linear(self, point, d, radius, is_lead_in):
        offset = _vec_scale(d, radius)
        start = _vec_sub(point, offset) if is_lead_in else point
        end = point if is_lead_in else _vec_add(point, offset)
        return LeadMove(start_point=start, end_point=end,
                        approach=ApproachType.LINEAR, radius=radius)

    def _build_arc(self, point, d, radius, is_lead_in):
        perp = _vec_normalize((-d[1], d[0], 0.0))
        center = _vec_add(point, _vec_scale(perp, radius))
        arc_start = _vec_sub(center, _vec_scale(d, radius)) if is_lead_in else point
        arc_end = point if is_lead_in else _vec_add(center, _vec_scale(d, radius))
        return LeadMove(start_point=arc_start, end_point=arc_end,
                        approach=ApproachType.ARC, arc_center=center, radius=radius)

    def _build_tangential(self, point, d, radius, is_lead_in):
        perp = _vec_normalize((-d[1], d[0], 0.0))
        combined = _vec_normalize(_vec_add(d, perp))
        offset = _vec_scale(combined, radius)
        start = _vec_sub(point, offset) if is_lead_in else point
        end = point if is_lead_in else _vec_add(point, offset)
        return LeadMove(start_point=start, end_point=end,
                        approach=ApproachType.TANGENTIAL, radius=radius)

    def _build_ramp(self, point, d, radius, angle_deg, is_lead_in):
        z_delta = radius * math.tan(math.radians(angle_deg))
        offset = _vec_add(_vec_scale(d, radius), (0.0, 0.0, z_delta))
        start = _vec_sub(point, offset) if is_lead_in else point
        end = point if is_lead_in else _vec_add(point, offset)
        return LeadMove(start_point=start, end_point=end,
                        approach=ApproachType.RAMP, ramp_angle_deg=angle_deg, radius=radius)

    def _build_helical(self, point, radius, pitch, is_lead_in):
        moves: List[LeadMove] = []
        segments = 16
        angle_step = 2.0 * math.pi / segments
        z_step = pitch / segments
        direction = -1 if is_lead_in else 1

        prev = point
        for i in range(1, segments + 1):
            angle = i * angle_step
            z = direction * i * z_step
            nxt = _vec_add(point, (radius * math.cos(angle) - radius,
                                   radius * math.sin(angle), z))
            moves.append(LeadMove(start_point=prev, end_point=nxt,
                                  approach=ApproachType.HELICAL,
                                  arc_center=point, radius=radius))
            prev = nxt
        return moves


# ── Fixtures ─────────────────────────────────────────────────────

@pytest.fixture
def gen():
    return LeadInOutGenerator()


# ── Tests ────────────────────────────────────────────────────────

class TestLeadInLinear:
    """Linear lead-in should produce a single move ending at the entry point."""

    def test_linear_lead_in_ends_at_entry(self, gen):
        entry = (10.0, 20.0, 0.0)
        direction = (1.0, 0.0, 0.0)
        config = LeadConfig(approach=ApproachType.LINEAR, radius=5.0)
        moves = gen.generate_lead_in(entry, direction, config)

        assert len(moves) == 1
        m = moves[0]
        assert m.approach == ApproachType.LINEAR
        # End point must match entry
        assert _vec_dist(m.end_point, entry) < 1e-6
        # Start point should be radius distance behind entry along direction
        assert _vec_dist(m.start_point, (5.0, 20.0, 0.0)) < 1e-6


class TestLeadOutLinear:
    """Linear lead-out should start at exit and extend by radius."""

    def test_linear_lead_out_starts_at_exit(self, gen):
        exit_pt = (50.0, 30.0, 0.0)
        direction = (0.0, 1.0, 0.0)
        config = LeadConfig(approach=ApproachType.LINEAR, radius=8.0)
        moves = gen.generate_lead_out(exit_pt, direction, config)

        assert len(moves) == 1
        m = moves[0]
        assert _vec_dist(m.start_point, exit_pt) < 1e-6
        assert _vec_dist(m.end_point, (50.0, 38.0, 0.0)) < 1e-6


class TestArcLeadIn:
    """Arc lead-in must have a valid arc center at radius distance from entry."""

    def test_arc_lead_in_center_at_radius(self, gen):
        entry = (0.0, 0.0, 0.0)
        direction = (1.0, 0.0, 0.0)
        config = LeadConfig(approach=ApproachType.ARC, radius=5.0)
        moves = gen.generate_lead_in(entry, direction, config)

        assert len(moves) == 1
        m = moves[0]
        assert m.approach == ApproachType.ARC
        # Arc center should be radius away from entry perpendicular to direction
        dist_to_center = _vec_dist(m.arc_center, entry)
        assert abs(dist_to_center - 5.0) < 1e-6


class TestRampLeadIn:
    """Ramp lead-in should have a Z component proportional to tan(angle)."""

    def test_ramp_has_z_component(self, gen):
        entry = (0.0, 0.0, 0.0)
        direction = (1.0, 0.0, 0.0)
        config = LeadConfig(approach=ApproachType.RAMP, radius=10.0, ramp_angle=5.0)
        moves = gen.generate_lead_in(entry, direction, config)

        assert len(moves) == 1
        m = moves[0]
        assert m.approach == ApproachType.RAMP
        expected_z = 10.0 * math.tan(math.radians(5.0))
        # Start Z should be below entry by z_delta (negative offset from entry)
        z_diff = abs(m.start_point[2] - (-expected_z))
        assert z_diff < 1e-6
        assert abs(m.ramp_angle_deg - 5.0) < 1e-6


class TestHelicalLeadIn:
    """Helical lead-in should produce multiple segments forming a spiral."""

    def test_helical_produces_multiple_segments(self, gen):
        entry = (0.0, 0.0, 0.0)
        direction = (1.0, 0.0, 0.0)
        config = LeadConfig(approach=ApproachType.HELICAL, radius=4.0, helical_pitch=2.0)
        moves = gen.generate_lead_in(entry, direction, config)

        assert len(moves) == 16  # full revolution, 16 segments
        # Each move should be HELICAL type
        for m in moves:
            assert m.approach == ApproachType.HELICAL
        # Chain continuity: each move's end == next move's start
        for i in range(len(moves) - 1):
            assert _vec_dist(moves[i].end_point, moves[i + 1].start_point) < 1e-6


class TestHelicalEntry:
    """Helical pocket entry should spiral down to target depth."""

    def test_helical_entry_reaches_depth(self, gen):
        center = (50.0, 50.0, 0.0)
        target_depth = -10.0
        diameter = 12.0
        pitch = 2.0
        moves = gen.generate_helical_entry(center, target_depth, diameter, pitch)

        assert len(moves) > 0
        # Last point should be close to target depth
        last_z = moves[-1].end_point[2]
        assert abs(last_z - target_depth) < 0.1
        # All moves should be HELICAL
        for m in moves:
            assert m.approach == ApproachType.HELICAL
            assert abs(m.radius - 6.0) < 1e-6  # diameter / 2


class TestValidateConfig:
    """Config validation should catch invalid parameters."""

    def test_valid_config(self, gen):
        config = LeadConfig()
        errors = gen.validate_config(config)
        assert len(errors) == 0

    def test_invalid_radius(self, gen):
        config = LeadConfig(radius=-1.0)
        errors = gen.validate_config(config)
        assert any("Radius" in e for e in errors)

    def test_invalid_ramp_angle(self, gen):
        config = LeadConfig(ramp_angle=0.0)
        errors = gen.validate_config(config)
        assert any("RampAngle" in e for e in errors)

    def test_invalid_overlap(self, gen):
        config = LeadConfig(overlap_pct=150.0)
        errors = gen.validate_config(config)
        assert any("OverlapPct" in e for e in errors)

    def test_invalid_helical_pitch(self, gen):
        config = LeadConfig(helical_pitch=0.0)
        errors = gen.validate_config(config)
        assert any("HelicalPitch" in e for e in errors)


class TestGetRecommendedApproach:
    """Recommendation heuristic should match material/operation combos."""

    def test_pocket_gets_helical(self, gen):
        assert gen.get_recommended_approach("Aluminum", "pocket") == ApproachType.HELICAL

    def test_drill_gets_ramp(self, gen):
        assert gen.get_recommended_approach("Steel", "drill") == ApproachType.RAMP

    def test_titanium_profile_gets_arc(self, gen):
        # Material takes precedence for hard materials
        assert gen.get_recommended_approach("Titanium", "profile") == ApproachType.ARC

    def test_aluminum_profile_gets_tangential(self, gen):
        assert gen.get_recommended_approach("Aluminum", "profile") == ApproachType.TANGENTIAL

    def test_default_gets_linear(self, gen):
        assert gen.get_recommended_approach("Plastic", "facing") == ApproachType.LINEAR


class TestTangentialLeadIn:
    """Tangential lead-in blends direction and perpendicular components."""

    def test_tangential_lead_in_geometry(self, gen):
        entry = (0.0, 0.0, 0.0)
        direction = (1.0, 0.0, 0.0)
        config = LeadConfig(approach=ApproachType.TANGENTIAL, radius=5.0)
        moves = gen.generate_lead_in(entry, direction, config)

        assert len(moves) == 1
        m = moves[0]
        assert m.approach == ApproachType.TANGENTIAL
        # End point is entry
        assert _vec_dist(m.end_point, entry) < 1e-6
        # Start point should be at ~radius distance from entry
        dist = _vec_dist(m.start_point, entry)
        assert abs(dist - 5.0) < 1e-4
