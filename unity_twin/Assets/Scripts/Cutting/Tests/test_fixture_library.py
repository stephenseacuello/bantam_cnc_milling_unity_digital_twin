"""Tests for FixtureLibraryManager logic (Python mirror of C# implementation).

Validates fixture CRUD operations, recommendation engine scoring,
type-based filtering, fixture comparison, and default library loading.
"""

import pytest
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional


# ---- Python mirror of C# types ----

@dataclass
class FixtureDefinition:
    fixture_id: str = ""
    name: str = ""
    fixture_type: str = ""          # vise | chuck | vacuum | fixture_plate | tombstone
    max_clamping_force_n: float = 0.0
    jaw_width_mm: float = 0.0
    max_workpiece_dia_mm: float = 0.0
    min_workpiece_dia_mm: float = 0.0
    repeatability_mm: float = 0.0
    setup_time_min: float = 0.0
    compatible_machines: List[str] = field(default_factory=list)


@dataclass
class FixtureRecommendation:
    fixture: Optional[FixtureDefinition] = None
    suitability_score: float = 0.0
    reasons: List[str] = field(default_factory=list)


class FixtureLibraryManager:
    """Manages a library of workholding fixture definitions.

    Provides CRUD operations, scored recommendations based on workpiece
    requirements, type-based filtering, and side-by-side comparison.
    Pre-loaded with five standard fixtures on construction.
    """

    def __init__(self):
        self._fixtures: Dict[str, FixtureDefinition] = {}
        self._load_defaults()

    # ── CRUD ────────────────────────────────────────────────────────

    def add_fixture(self, defn: FixtureDefinition) -> None:
        """Add a fixture definition to the library."""
        if defn is None:
            raise ValueError("fixture definition must not be None")
        if not defn.fixture_id:
            raise ValueError("fixture_id must not be empty")
        self._fixtures[defn.fixture_id] = defn

    def remove_fixture(self, fixture_id: str) -> bool:
        """Remove a fixture by its ID. Returns True if found and removed."""
        if not fixture_id or fixture_id not in self._fixtures:
            return False
        del self._fixtures[fixture_id]
        return True

    def get_fixture(self, fixture_id: str) -> Optional[FixtureDefinition]:
        """Retrieve a fixture by its ID. Returns None if not found."""
        if not fixture_id:
            return None
        return self._fixtures.get(fixture_id)

    def get_all_fixtures(self) -> List[FixtureDefinition]:
        """Returns all fixture definitions in the library."""
        return list(self._fixtures.values())

    # ── Query / Recommendation ─────────────────────────────────────

    def get_fixtures_by_type(self, fixture_type: str) -> List[FixtureDefinition]:
        """Filter fixtures by type (e.g. 'vise', 'chuck')."""
        return [f for f in self._fixtures.values()
                if f.fixture_type.lower() == fixture_type.lower()]

    def recommend_fixture(
        self,
        workpiece_dia_mm: float,
        required_force_n: float,
        machine_id: str,
    ) -> List[FixtureRecommendation]:
        """Recommend fixtures for a given workpiece diameter, required clamping
        force, and machine identifier.  Returns a list of FixtureRecommendation
        sorted by descending suitability score."""
        recommendations: List[FixtureRecommendation] = []

        for fixture in self._fixtures.values():
            score = 0.0
            reasons: List[str] = []

            # ── Machine compatibility (hard requirement — 0 if incompatible)
            machine_ok = (
                len(fixture.compatible_machines) == 0
                or machine_id in fixture.compatible_machines
            )
            if machine_ok:
                score += 30.0
                reasons.append(f"Compatible with machine {machine_id}")
            else:
                reasons.append(f"Not compatible with machine {machine_id}")

            # ── Clamping force capacity
            if fixture.max_clamping_force_n >= required_force_n:
                max_force = max(fixture.max_clamping_force_n, 1.0)
                force_ratio = required_force_n / max_force
                if 0.3 <= force_ratio <= 0.8:
                    force_score = 30.0
                elif force_ratio < 0.3:
                    force_score = 30.0 * (force_ratio / 0.3)
                else:
                    force_score = 30.0 * ((1.0 - force_ratio) / 0.2)
                score += force_score
                reasons.append(
                    f"Force capacity {fixture.max_clamping_force_n}N "
                    f"meets requirement of {required_force_n}N"
                )
            else:
                reasons.append(
                    f"Insufficient clamping force "
                    f"({fixture.max_clamping_force_n}N < {required_force_n}N)"
                )

            # ── Workpiece size range
            if (fixture.min_workpiece_dia_mm
                    <= workpiece_dia_mm
                    <= fixture.max_workpiece_dia_mm):
                rng = fixture.max_workpiece_dia_mm - fixture.min_workpiece_dia_mm
                mid = (fixture.max_workpiece_dia_mm + fixture.min_workpiece_dia_mm) * 0.5
                dist_from_mid = abs(workpiece_dia_mm - mid)
                if rng > 0:
                    size_score = 25.0 * (1.0 - dist_from_mid / (rng * 0.5))
                else:
                    size_score = 25.0
                score += max(size_score, 5.0)
                reasons.append("Workpiece diameter within fixture range")
            else:
                reasons.append(
                    f"Workpiece diameter {workpiece_dia_mm}mm outside range "
                    f"[{fixture.min_workpiece_dia_mm}\u2013{fixture.max_workpiece_dia_mm}mm]"
                )

            # ── Repeatability bonus
            if fixture.repeatability_mm <= 0.01:
                score += 10.0
                reasons.append("Excellent repeatability")
            elif fixture.repeatability_mm <= 0.025:
                score += 7.0
                reasons.append("Good repeatability")
            else:
                score += 3.0
                reasons.append("Moderate repeatability")

            # ── Setup time bonus
            if fixture.setup_time_min <= 5.0:
                score += 5.0
                reasons.append("Quick setup time")
            elif fixture.setup_time_min <= 15.0:
                score += 3.0
                reasons.append("Moderate setup time")
            else:
                score += 1.0
                reasons.append("Lengthy setup time")

            score = max(0.0, min(100.0, score))

            recommendations.append(FixtureRecommendation(
                fixture=fixture,
                suitability_score=score,
                reasons=reasons,
            ))

        recommendations.sort(key=lambda r: r.suitability_score, reverse=True)
        return recommendations

    def compare_fixtures(
        self, id1: str, id2: str
    ) -> Optional[Dict[str, Tuple[str, str]]]:
        """Compare two fixtures side-by-side.  Returns a dict mapping attribute
        names to (fixture1_value, fixture2_value) string tuples.
        Returns None if either fixture ID is not found."""
        f1 = self.get_fixture(id1)
        f2 = self.get_fixture(id2)
        if f1 is None or f2 is None:
            return None

        return {
            "name": (f1.name, f2.name),
            "fixtureType": (f1.fixture_type, f2.fixture_type),
            "maxClampingForceN": (
                f"{f1.max_clamping_force_n:.1f}",
                f"{f2.max_clamping_force_n:.1f}",
            ),
            "jawWidthMm": (
                f"{f1.jaw_width_mm:.1f}",
                f"{f2.jaw_width_mm:.1f}",
            ),
            "maxWorkpieceDiaMm": (
                f"{f1.max_workpiece_dia_mm:.1f}",
                f"{f2.max_workpiece_dia_mm:.1f}",
            ),
            "minWorkpieceDiaMm": (
                f"{f1.min_workpiece_dia_mm:.1f}",
                f"{f2.min_workpiece_dia_mm:.1f}",
            ),
            "repeatabilityMm": (
                f"{f1.repeatability_mm:.4f}",
                f"{f2.repeatability_mm:.4f}",
            ),
            "setupTimeMin": (
                f"{f1.setup_time_min:.1f}",
                f"{f2.setup_time_min:.1f}",
            ),
            "compatibleMachines": (
                ",".join(f1.compatible_machines),
                ",".join(f2.compatible_machines),
            ),
        }

    # ── Default fixtures ───────────────────────────────────────────

    def _load_defaults(self) -> None:
        self.add_fixture(FixtureDefinition(
            fixture_id="KURT-DL640",
            name="Kurt DL640 6\" Double Lock Vise",
            fixture_type="vise",
            max_clamping_force_n=44480.0,
            jaw_width_mm=152.4,
            max_workpiece_dia_mm=152.4,
            min_workpiece_dia_mm=5.0,
            repeatability_mm=0.0127,
            setup_time_min=5.0,
            compatible_machines=["VMC-500", "VMC-750", "VMC-1000", "HMC-500"],
        ))

        self.add_fixture(FixtureDefinition(
            fixture_id="3JAW-200",
            name="200mm 3-Jaw Universal Chuck",
            fixture_type="chuck",
            max_clamping_force_n=55000.0,
            jaw_width_mm=0.0,
            max_workpiece_dia_mm=200.0,
            min_workpiece_dia_mm=10.0,
            repeatability_mm=0.025,
            setup_time_min=10.0,
            compatible_machines=["LATHE-200", "LATHE-300", "MILL-TURN-500"],
        ))

        self.add_fixture(FixtureDefinition(
            fixture_id="VAC-TABLE-600",
            name="600x400mm Vacuum Table",
            fixture_type="vacuum",
            max_clamping_force_n=8000.0,
            jaw_width_mm=0.0,
            max_workpiece_dia_mm=600.0,
            min_workpiece_dia_mm=50.0,
            repeatability_mm=0.05,
            setup_time_min=3.0,
            compatible_machines=["VMC-500", "VMC-750", "VMC-1000", "ROUTER-1200"],
        ))

        self.add_fixture(FixtureDefinition(
            fixture_id="MOD-PLATE-400",
            name="400x400mm Modular Fixture Plate",
            fixture_type="fixture_plate",
            max_clamping_force_n=35000.0,
            jaw_width_mm=0.0,
            max_workpiece_dia_mm=380.0,
            min_workpiece_dia_mm=20.0,
            repeatability_mm=0.005,
            setup_time_min=15.0,
            compatible_machines=["VMC-500", "VMC-750", "VMC-1000", "HMC-500", "5AX-400"],
        ))

        self.add_fixture(FixtureDefinition(
            fixture_id="TOMB-4SIDE-300",
            name="300mm 4-Sided Tombstone",
            fixture_type="tombstone",
            max_clamping_force_n=60000.0,
            jaw_width_mm=0.0,
            max_workpiece_dia_mm=280.0,
            min_workpiece_dia_mm=15.0,
            repeatability_mm=0.01,
            setup_time_min=25.0,
            compatible_machines=["HMC-500", "HMC-630", "5AX-400"],
        ))


# ---- Fixtures (pytest) ----

@pytest.fixture
def manager():
    return FixtureLibraryManager()


# ---- Tests ----

def test_defaults_loaded(manager):
    """Library is pre-loaded with 5 default fixture definitions."""
    all_fixtures = manager.get_all_fixtures()
    assert len(all_fixtures) == 5
    ids = {f.fixture_id for f in all_fixtures}
    assert "KURT-DL640" in ids
    assert "3JAW-200" in ids
    assert "VAC-TABLE-600" in ids
    assert "MOD-PLATE-400" in ids
    assert "TOMB-4SIDE-300" in ids


def test_get_fixture_by_id(manager):
    """Retrieving a fixture by ID returns the correct definition."""
    vise = manager.get_fixture("KURT-DL640")
    assert vise is not None
    assert vise.name == "Kurt DL640 6\" Double Lock Vise"
    assert vise.fixture_type == "vise"
    assert vise.max_clamping_force_n == pytest.approx(44480.0)
    assert vise.jaw_width_mm == pytest.approx(152.4)
    assert vise.repeatability_mm == pytest.approx(0.0127)


def test_get_fixture_not_found(manager):
    """Requesting a non-existent fixture ID returns None."""
    assert manager.get_fixture("DOES-NOT-EXIST") is None
    assert manager.get_fixture("") is None
    assert manager.get_fixture(None) is None


def test_add_and_remove_fixture(manager):
    """Custom fixtures can be added and removed."""
    custom = FixtureDefinition(
        fixture_id="CUSTOM-001",
        name="Custom Magnetic Chuck",
        fixture_type="chuck",
        max_clamping_force_n=20000.0,
        jaw_width_mm=0.0,
        max_workpiece_dia_mm=300.0,
        min_workpiece_dia_mm=30.0,
        repeatability_mm=0.02,
        setup_time_min=2.0,
        compatible_machines=["VMC-500"],
    )
    manager.add_fixture(custom)
    assert len(manager.get_all_fixtures()) == 6
    assert manager.get_fixture("CUSTOM-001") is not None
    assert manager.get_fixture("CUSTOM-001").name == "Custom Magnetic Chuck"

    removed = manager.remove_fixture("CUSTOM-001")
    assert removed is True
    assert len(manager.get_all_fixtures()) == 5
    assert manager.get_fixture("CUSTOM-001") is None

    # Removing again returns False
    assert manager.remove_fixture("CUSTOM-001") is False


def test_add_fixture_validation(manager):
    """Adding a fixture with empty ID raises ValueError."""
    with pytest.raises(ValueError, match="fixture_id"):
        manager.add_fixture(FixtureDefinition(fixture_id="", name="Bad"))

    with pytest.raises(ValueError):
        manager.add_fixture(None)


def test_get_fixtures_by_type(manager):
    """Filtering fixtures by type returns the correct subset."""
    vises = manager.get_fixtures_by_type("vise")
    assert len(vises) == 1
    assert vises[0].fixture_id == "KURT-DL640"

    chucks = manager.get_fixtures_by_type("chuck")
    assert len(chucks) == 1
    assert chucks[0].fixture_id == "3JAW-200"

    tombstones = manager.get_fixtures_by_type("tombstone")
    assert len(tombstones) == 1
    assert tombstones[0].fixture_id == "TOMB-4SIDE-300"

    # Case-insensitive
    vacuums = manager.get_fixtures_by_type("VACUUM")
    assert len(vacuums) == 1

    # Non-existent type
    none_found = manager.get_fixtures_by_type("collet")
    assert len(none_found) == 0


def test_recommend_fixture_compatible_machine(manager):
    """Recommendation ranks compatible fixtures higher than incompatible ones."""
    # Use a low force so that all fixtures meet the requirement; this
    # isolates the machine-compatibility scoring component.
    recs = manager.recommend_fixture(
        workpiece_dia_mm=100.0,
        required_force_n=5000.0,
        machine_id="VMC-500",
    )
    assert len(recs) == 5

    # All recommendations sorted descending
    scores = [r.suitability_score for r in recs]
    assert scores == sorted(scores, reverse=True)

    # Fixtures compatible with VMC-500: KURT-DL640, VAC-TABLE-600, MOD-PLATE-400
    # The top-ranked fixtures should all be VMC-500-compatible
    compatible_ids = {"KURT-DL640", "VAC-TABLE-600", "MOD-PLATE-400"}
    top_three_ids = {r.fixture.fixture_id for r in recs[:3]}
    assert top_three_ids == compatible_ids

    # Each recommendation has at least one reason
    for rec in recs:
        assert len(rec.reasons) > 0


def test_recommend_fixture_force_requirement(manager):
    """Fixtures with insufficient clamping force score lower."""
    # Vacuum table max force is 8000N; require 10000N
    recs = manager.recommend_fixture(
        workpiece_dia_mm=100.0,
        required_force_n=10000.0,
        machine_id="VMC-500",
    )
    vac_rec = [r for r in recs if r.fixture.fixture_id == "VAC-TABLE-600"][0]
    vise_rec = [r for r in recs if r.fixture.fixture_id == "KURT-DL640"][0]

    # Vacuum table cannot meet force requirement, so should score lower
    assert vise_rec.suitability_score > vac_rec.suitability_score
    # Check that the reason mentions insufficient force
    insufficient = [r for r in vac_rec.reasons if "Insufficient" in r]
    assert len(insufficient) == 1


def test_recommend_fixture_workpiece_out_of_range(manager):
    """Fixtures unable to hold the workpiece diameter score lower."""
    # Workpiece 500mm won't fit in the 6" Kurt vise (max 152.4mm)
    recs = manager.recommend_fixture(
        workpiece_dia_mm=500.0,
        required_force_n=5000.0,
        machine_id="VMC-500",
    )
    vise_rec = [r for r in recs if r.fixture.fixture_id == "KURT-DL640"][0]
    vac_rec = [r for r in recs if r.fixture.fixture_id == "VAC-TABLE-600"][0]

    # Vacuum table can hold up to 600mm, vise only 152.4mm
    assert vac_rec.suitability_score > vise_rec.suitability_score
    # Vise reason should mention outside range
    outside = [r for r in vise_rec.reasons if "outside range" in r]
    assert len(outside) == 1


def test_compare_fixtures(manager):
    """Comparing two fixtures returns attribute differences."""
    diff = manager.compare_fixtures("KURT-DL640", "3JAW-200")
    assert diff is not None
    assert len(diff) == 9

    assert diff["fixtureType"] == ("vise", "chuck")
    assert diff["maxClampingForceN"] == ("44480.0", "55000.0")
    assert diff["jawWidthMm"] == ("152.4", "0.0")
    assert diff["setupTimeMin"] == ("5.0", "10.0")
    assert "name" in diff
    assert "repeatabilityMm" in diff
    assert "compatibleMachines" in diff


def test_compare_fixtures_not_found(manager):
    """Comparing with a non-existent fixture ID returns None."""
    assert manager.compare_fixtures("KURT-DL640", "NONEXISTENT") is None
    assert manager.compare_fixtures("NONEXISTENT", "KURT-DL640") is None
    assert manager.compare_fixtures("NOPE", "ALSO-NOPE") is None


def test_recommendation_scores_within_bounds(manager):
    """All recommendation scores are clamped between 0 and 100."""
    recs = manager.recommend_fixture(
        workpiece_dia_mm=50.0,
        required_force_n=30000.0,
        machine_id="HMC-500",
    )
    for rec in recs:
        assert 0.0 <= rec.suitability_score <= 100.0
