"""Tests for CannedCycleLibrary logic (Python mirror of C# implementation).

Validates canned cycle expansion into linear moves for G81-G89 cycles,
parameter validation, cycle time estimation, peck drilling sequence
correctness, and human-readable cycle descriptions.
"""

import pytest
import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Tuple, Optional


# ---- Python mirror of C# types ----

class CannedCycleType(Enum):
    """Standard canned drilling/boring/tapping cycles."""
    DRILL = auto()        # G81
    DRILL_DWELL = auto()  # G82
    PECK_DRILL = auto()   # G83
    TAP = auto()          # G84
    BORE = auto()         # G85
    BORE_DWELL = auto()   # G86
    BACK_BORE = auto()    # G87
    BORE_STOP = auto()    # G89


@dataclass
class CannedCycleParams:
    """Parameters that fully define a canned cycle invocation."""
    cycle_type: CannedCycleType = CannedCycleType.DRILL
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    r: float = 0.0
    q: float = 0.0   # peck depth for G83
    p: float = 0.0   # dwell ms for G82/G86/G89
    f: float = 100.0  # feed rate mm/min
    depth: float = 0.0


@dataclass
class Vector3:
    """Minimal 3D vector."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def approx_eq(self, other: "Vector3", tol: float = 1e-4) -> bool:
        return (abs(self.x - other.x) < tol and
                abs(self.y - other.y) < tol and
                abs(self.z - other.z) < tol)


@dataclass
class CycleMoveSegment:
    """A single linear move segment produced by cycle expansion."""
    start: Vector3
    end: Vector3


@dataclass
class CycleExpansion:
    """Result of expanding a canned cycle."""
    moves: List[CycleMoveSegment] = field(default_factory=list)
    total_moves: int = 0
    estimated_time_sec: float = 0.0
    cycle_name: str = ""


class CannedCycleLibrary:
    """Manages and expands standard canned drilling/boring/tapping
    cycles (G81-G89) into sequences of linear move segments."""

    PECK_CLEARANCE = 0.5  # mm
    RAPID_RATE = 10000.0  # mm/min

    # ---- Descriptions ----

    _DESCRIPTIONS = {
        CannedCycleType.DRILL:       "G81 Drilling Cycle",
        CannedCycleType.DRILL_DWELL: "G82 Drilling Cycle with Dwell",
        CannedCycleType.PECK_DRILL:  "G83 Peck Drilling Cycle",
        CannedCycleType.TAP:         "G84 Tapping Cycle",
        CannedCycleType.BORE:        "G85 Boring Cycle",
        CannedCycleType.BORE_DWELL:  "G86 Boring Cycle with Dwell",
        CannedCycleType.BACK_BORE:   "G87 Back Boring Cycle",
        CannedCycleType.BORE_STOP:   "G89 Boring Cycle with Dwell and Feed Retract",
    }

    # ---- Public API ----

    def expand_cycle(self, params: CannedCycleParams) -> CycleExpansion:
        """Expands a canned cycle into individual linear move segments."""
        if params is None:
            raise ValueError("params must not be None")

        errors = self.validate_params(params)
        if errors:
            raise ValueError("; ".join(errors))

        expansion = CycleExpansion(
            cycle_name=self.get_cycle_description(params.cycle_type)
        )

        expander = {
            CannedCycleType.DRILL:       self._expand_drill,
            CannedCycleType.DRILL_DWELL: self._expand_drill_dwell,
            CannedCycleType.PECK_DRILL:  self._expand_peck_drill,
            CannedCycleType.TAP:         self._expand_tap,
            CannedCycleType.BORE:        self._expand_bore,
            CannedCycleType.BORE_DWELL:  self._expand_bore_dwell,
            CannedCycleType.BACK_BORE:   self._expand_back_bore,
            CannedCycleType.BORE_STOP:   self._expand_bore_stop,
        }.get(params.cycle_type)

        if expander is None:
            raise ValueError(f"Unknown cycle type: {params.cycle_type}")

        expander(params, expansion)

        expansion.total_moves = len(expansion.moves)
        expansion.estimated_time_sec = self.estimate_cycle_time(params, params.f)
        return expansion

    def get_cycle_description(self, cycle_type: CannedCycleType) -> str:
        """Returns a human-readable description of the given cycle type."""
        return self._DESCRIPTIONS.get(cycle_type, "Unknown Cycle")

    def estimate_cycle_time(self, params: CannedCycleParams, feed_rate: float) -> float:
        """Estimates the execution time (seconds) for a cycle at the given feed rate."""
        if params is None:
            raise ValueError("params must not be None")
        if feed_rate <= 0.0:
            raise ValueError("feed_rate must be positive")

        total_depth = params.r - params.z  # positive value
        dwell_sec = params.p / 1000.0

        if params.cycle_type == CannedCycleType.DRILL:
            return (total_depth / feed_rate + total_depth / self.RAPID_RATE) * 60.0

        elif params.cycle_type == CannedCycleType.DRILL_DWELL:
            return (total_depth / feed_rate + total_depth / self.RAPID_RATE) * 60.0 + dwell_sec

        elif params.cycle_type == CannedCycleType.PECK_DRILL:
            remaining = total_depth
            time_sec = 0.0
            current_depth_from_r = 0.0
            while remaining > 0.0:
                peck = min(params.q, remaining)
                # Rapid from R to (previous depth - clearance) if not first peck
                if current_depth_from_r > 0.0:
                    rapid_down = current_depth_from_r - self.PECK_CLEARANCE
                    time_sec += (rapid_down / self.RAPID_RATE) * 60.0
                # Feed the peck increment (+ clearance recovery except first)
                feed_dist = peck + self.PECK_CLEARANCE if current_depth_from_r > 0.0 else peck
                time_sec += (feed_dist / feed_rate) * 60.0
                current_depth_from_r += peck
                remaining -= peck
                # Rapid retract to R
                time_sec += (current_depth_from_r / self.RAPID_RATE) * 60.0
            return time_sec

        elif params.cycle_type == CannedCycleType.TAP:
            return (total_depth / feed_rate + total_depth / feed_rate) * 60.0

        elif params.cycle_type == CannedCycleType.BORE:
            return (total_depth / feed_rate + total_depth / feed_rate) * 60.0

        elif params.cycle_type == CannedCycleType.BORE_DWELL:
            return (total_depth / feed_rate + total_depth / self.RAPID_RATE) * 60.0 + dwell_sec

        elif params.cycle_type == CannedCycleType.BACK_BORE:
            return (total_depth / self.RAPID_RATE + total_depth / feed_rate + total_depth / self.RAPID_RATE) * 60.0

        elif params.cycle_type == CannedCycleType.BORE_STOP:
            return (total_depth / feed_rate + total_depth / feed_rate) * 60.0 + dwell_sec

        return 0.0

    def validate_params(self, params: CannedCycleParams) -> List[str]:
        """Validates cycle parameters. Returns a list of error strings (empty if valid)."""
        if params is None:
            raise ValueError("params must not be None")

        errors: List[str] = []
        total_depth = params.r - params.z

        if total_depth <= 0.0:
            errors.append("Depth must be positive (r must be greater than z)")

        if params.f <= 0.0:
            errors.append("Feed rate (f) must be positive")

        if params.cycle_type == CannedCycleType.PECK_DRILL and params.q <= 0.0:
            errors.append("Peck depth (q) must be positive for G83 peck drilling")

        if params.cycle_type in (CannedCycleType.DRILL_DWELL,
                                  CannedCycleType.BORE_DWELL,
                                  CannedCycleType.BORE_STOP) and params.p < 0.0:
            errors.append("Dwell time (p) must not be negative for dwell cycles")

        return errors

    # ---- Private expansion helpers ----

    @staticmethod
    def _add_move(exp: CycleExpansion, start: Vector3, end: Vector3):
        exp.moves.append(CycleMoveSegment(start=start, end=end))

    def _expand_drill(self, p: CannedCycleParams, exp: CycleExpansion):
        """G81: Rapid to R, feed to Z, rapid retract to R."""
        xy_r = Vector3(p.x, p.y, p.r)
        xy_z = Vector3(p.x, p.y, p.z)
        self._add_move(exp, Vector3(p.x, p.y, p.r), xy_r)
        self._add_move(exp, xy_r, xy_z)
        self._add_move(exp, xy_z, xy_r)

    def _expand_drill_dwell(self, p: CannedCycleParams, exp: CycleExpansion):
        """G82: Rapid to R, feed to Z, dwell, rapid retract to R."""
        xy_r = Vector3(p.x, p.y, p.r)
        xy_z = Vector3(p.x, p.y, p.z)
        self._add_move(exp, Vector3(p.x, p.y, p.r), xy_r)
        self._add_move(exp, xy_r, xy_z)
        self._add_move(exp, xy_z, xy_r)

    def _expand_peck_drill(self, p: CannedCycleParams, exp: CycleExpansion):
        """G83: Peck drilling with full retract between pecks."""
        xy_r = Vector3(p.x, p.y, p.r)
        remaining = p.r - p.z  # total depth (positive)
        current_z = p.r

        # Initial positioning at R
        self._add_move(exp, Vector3(p.x, p.y, p.r), xy_r)

        first_peck = True
        while remaining > 1e-9:
            peck = min(p.q, remaining)
            target_z = current_z - peck

            if not first_peck:
                # Rapid from R down to (previous depth + clearance)
                rapid_target_z = current_z + self.PECK_CLEARANCE
                self._add_move(exp, xy_r, Vector3(p.x, p.y, rapid_target_z))
                # Feed through clearance + peck
                self._add_move(exp, Vector3(p.x, p.y, rapid_target_z),
                               Vector3(p.x, p.y, target_z))
            else:
                # Feed from R to first peck depth
                self._add_move(exp, xy_r, Vector3(p.x, p.y, target_z))
                first_peck = False

            current_z = target_z
            remaining -= peck

            # Rapid retract to R
            self._add_move(exp, Vector3(p.x, p.y, current_z), xy_r)

    def _expand_tap(self, p: CannedCycleParams, exp: CycleExpansion):
        """G84: Rapid to R, feed to Z, reverse-feed retract to R."""
        xy_r = Vector3(p.x, p.y, p.r)
        xy_z = Vector3(p.x, p.y, p.z)
        self._add_move(exp, Vector3(p.x, p.y, p.r), xy_r)
        self._add_move(exp, xy_r, xy_z)
        self._add_move(exp, xy_z, xy_r)

    def _expand_bore(self, p: CannedCycleParams, exp: CycleExpansion):
        """G85: Rapid to R, feed to Z, feed retract to R."""
        xy_r = Vector3(p.x, p.y, p.r)
        xy_z = Vector3(p.x, p.y, p.z)
        self._add_move(exp, Vector3(p.x, p.y, p.r), xy_r)
        self._add_move(exp, xy_r, xy_z)
        self._add_move(exp, xy_z, xy_r)

    def _expand_bore_dwell(self, p: CannedCycleParams, exp: CycleExpansion):
        """G86: Rapid to R, feed to Z, dwell, spindle stop, rapid retract."""
        xy_r = Vector3(p.x, p.y, p.r)
        xy_z = Vector3(p.x, p.y, p.z)
        self._add_move(exp, Vector3(p.x, p.y, p.r), xy_r)
        self._add_move(exp, xy_r, xy_z)
        self._add_move(exp, xy_z, xy_r)

    def _expand_back_bore(self, p: CannedCycleParams, exp: CycleExpansion):
        """G87: Back boring -- rapid to bottom, feed up, rapid retract."""
        xy_r = Vector3(p.x, p.y, p.r)
        xy_z = Vector3(p.x, p.y, p.z)
        self._add_move(exp, Vector3(p.x, p.y, p.r), xy_z)
        self._add_move(exp, xy_z, xy_r)
        self._add_move(exp, xy_r, xy_r)

    def _expand_bore_stop(self, p: CannedCycleParams, exp: CycleExpansion):
        """G89: Rapid to R, feed to Z, dwell, feed retract to R."""
        xy_r = Vector3(p.x, p.y, p.r)
        xy_z = Vector3(p.x, p.y, p.z)
        self._add_move(exp, Vector3(p.x, p.y, p.r), xy_r)
        self._add_move(exp, xy_r, xy_z)
        self._add_move(exp, xy_z, xy_r)


# ---- Fixtures (pytest) ----

@pytest.fixture
def library():
    return CannedCycleLibrary()


def _make_params(**kwargs) -> CannedCycleParams:
    """Helper to create CannedCycleParams with sensible defaults."""
    defaults = dict(
        cycle_type=CannedCycleType.DRILL,
        x=10.0, y=20.0, z=-25.0, r=2.0,
        q=5.0, p=500.0, f=200.0, depth=27.0,
    )
    defaults.update(kwargs)
    return CannedCycleParams(**defaults)


# ---- Tests ----

def test_expand_g81_drill_produces_three_moves(library):
    """G81 drill cycle expands to 3 moves: position, feed-down, retract."""
    params = _make_params(cycle_type=CannedCycleType.DRILL)
    expansion = library.expand_cycle(params)

    assert expansion.total_moves == 3
    assert expansion.cycle_name == "G81 Drilling Cycle"

    # Move 1: position at R-plane
    assert expansion.moves[0].end.z == pytest.approx(2.0)
    # Move 2: feed to depth
    assert expansion.moves[1].start.z == pytest.approx(2.0)
    assert expansion.moves[1].end.z == pytest.approx(-25.0)
    # Move 3: retract to R
    assert expansion.moves[2].start.z == pytest.approx(-25.0)
    assert expansion.moves[2].end.z == pytest.approx(2.0)


def test_expand_g83_peck_drill_sequence(library):
    """G83 peck drill with total depth 27mm and peck 5mm produces correct
    sequence: rapid to R, then for each peck: (rapid down, feed peck,
    rapid retract to R)."""
    params = _make_params(
        cycle_type=CannedCycleType.PECK_DRILL,
        z=-25.0, r=2.0, q=5.0
    )
    expansion = library.expand_cycle(params)

    assert expansion.cycle_name == "G83 Peck Drilling Cycle"

    # Total depth = 2.0 - (-25.0) = 27.0, peck = 5.0
    # Pecks: 5, 5, 5, 5, 5, 2 => 6 pecks
    # Move count: 1 (initial pos) + 2 (first peck: feed + retract)
    #   + 3 * 5 (subsequent pecks: rapid-down + feed + retract) = 18
    num_pecks = math.ceil(27.0 / 5.0)
    assert num_pecks == 6

    # First peck: position(1) + feed(1) + retract(1) = 3
    # Each subsequent peck: rapid-down(1) + feed(1) + retract(1) = 3
    # Total = 1 + 2 + 3*5 = 18
    assert expansion.total_moves == 1 + 2 + 3 * (num_pecks - 1)

    # Verify first peck feeds from R to R-5 = -3.0
    first_feed = expansion.moves[1]
    assert first_feed.start.z == pytest.approx(2.0)
    assert first_feed.end.z == pytest.approx(-3.0)

    # Verify first retract goes back to R
    first_retract = expansion.moves[2]
    assert first_retract.start.z == pytest.approx(-3.0)
    assert first_retract.end.z == pytest.approx(2.0)

    # Verify second peck rapid-down stops above previous depth
    second_rapid = expansion.moves[3]
    assert second_rapid.start.z == pytest.approx(2.0)  # from R
    expected_rapid_z = -3.0 + CannedCycleLibrary.PECK_CLEARANCE  # -2.5
    assert second_rapid.end.z == pytest.approx(expected_rapid_z)


def test_validate_params_depth_must_be_positive(library):
    """ValidateParams rejects z >= r (non-positive depth)."""
    params = _make_params(z=10.0, r=5.0)  # z above r -> invalid
    errors = library.validate_params(params)
    assert len(errors) >= 1
    assert any("Depth" in e for e in errors)


def test_validate_params_feed_must_be_positive(library):
    """ValidateParams rejects zero or negative feed rate."""
    params = _make_params(f=0.0)
    errors = library.validate_params(params)
    assert any("Feed rate" in e for e in errors)


def test_validate_params_peck_depth_required_for_g83(library):
    """ValidateParams rejects G83 with q <= 0."""
    params = _make_params(cycle_type=CannedCycleType.PECK_DRILL, q=0.0)
    errors = library.validate_params(params)
    assert any("Peck depth" in e for e in errors)


def test_validate_params_valid_drill_returns_no_errors(library):
    """ValidateParams returns empty list for a valid G81 cycle."""
    params = _make_params(cycle_type=CannedCycleType.DRILL)
    errors = library.validate_params(params)
    assert errors == []


def test_get_cycle_description_all_types(library):
    """GetCycleDescription returns non-empty description for every type."""
    for ct in CannedCycleType:
        desc = library.get_cycle_description(ct)
        assert len(desc) > 0
        assert "Unknown" not in desc


def test_estimate_cycle_time_drill_positive(library):
    """EstimateCycleTime returns a positive value for G81."""
    params = _make_params(cycle_type=CannedCycleType.DRILL)
    time_sec = library.estimate_cycle_time(params, 200.0)
    assert time_sec > 0.0


def test_estimate_cycle_time_dwell_adds_time(library):
    """G82 (dwell) takes longer than G81 (no dwell) with same geometry."""
    base = _make_params(cycle_type=CannedCycleType.DRILL, p=0.0)
    dwell = _make_params(cycle_type=CannedCycleType.DRILL_DWELL, p=1000.0)

    t_base = library.estimate_cycle_time(base, 200.0)
    t_dwell = library.estimate_cycle_time(dwell, 200.0)

    # Dwell should add exactly 1 second (1000 ms)
    assert t_dwell == pytest.approx(t_base + 1.0, abs=0.01)


def test_expand_cycle_invalid_params_raises(library):
    """ExpandCycle raises ValueError when parameters are invalid."""
    params = _make_params(z=10.0, r=5.0)  # depth not positive
    with pytest.raises(ValueError, match="Depth"):
        library.expand_cycle(params)


def test_expand_g85_bore_xy_preserved(library):
    """G85 bore cycle preserves X/Y coordinates in all moves."""
    params = _make_params(cycle_type=CannedCycleType.BORE, x=42.0, y=17.5)
    expansion = library.expand_cycle(params)

    for seg in expansion.moves:
        assert seg.start.x == pytest.approx(42.0)
        assert seg.start.y == pytest.approx(17.5)
        assert seg.end.x == pytest.approx(42.0)
        assert seg.end.y == pytest.approx(17.5)


def test_expand_g84_tap_three_moves(library):
    """G84 tapping cycle expands to 3 moves."""
    params = _make_params(cycle_type=CannedCycleType.TAP)
    expansion = library.expand_cycle(params)

    assert expansion.total_moves == 3
    assert "Tapping" in expansion.cycle_name

    # Feed down and feed retract (spindle reverses)
    assert expansion.moves[1].end.z == pytest.approx(-25.0)
    assert expansion.moves[2].end.z == pytest.approx(2.0)


def test_estimate_cycle_time_zero_feed_raises(library):
    """EstimateCycleTime raises ValueError for zero feed rate."""
    params = _make_params()
    with pytest.raises(ValueError, match="positive"):
        library.estimate_cycle_time(params, 0.0)


def test_validate_params_negative_dwell_for_g82(library):
    """ValidateParams rejects negative dwell time for G82."""
    params = _make_params(cycle_type=CannedCycleType.DRILL_DWELL, p=-100.0)
    errors = library.validate_params(params)
    assert any("Dwell" in e for e in errors)


def test_expand_g87_back_bore_rapid_to_bottom_first(library):
    """G87 back bore starts with a rapid move to the bottom (Z depth)."""
    params = _make_params(cycle_type=CannedCycleType.BACK_BORE)
    expansion = library.expand_cycle(params)

    assert expansion.total_moves == 3
    # First real move goes from R to Z (rapid to bottom)
    assert expansion.moves[0].end.z == pytest.approx(-25.0)
    # Second move feeds back up from Z to R
    assert expansion.moves[1].start.z == pytest.approx(-25.0)
    assert expansion.moves[1].end.z == pytest.approx(2.0)
