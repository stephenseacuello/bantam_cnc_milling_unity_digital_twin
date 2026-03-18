"""Tests for CoordinateSystemManager logic (Python mirror of C# implementation).

Validates work coordinate system management, offset get/set, active system
switching, point transformation between systems, custom system registration,
and transform chain computation.
"""

import pytest
import math
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple


# ---- Python mirror of C# types ----

@dataclass
class WorkCoordinateSystem:
    name: str = ""
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0
    rotation_deg: float = 0.0
    is_active: bool = False
    description: str = ""


@dataclass
class CoordinateTransform:
    from_system: str = ""
    to_system: str = ""
    delta_x: float = 0.0
    delta_y: float = 0.0
    delta_z: float = 0.0


class CoordinateSystemManager:
    """Manages multiple work coordinate systems (G54-G59 and extended
    G54.1 P1-P48).  Provides methods to set/get offsets, activate a
    system, transform points between systems, and register custom
    coordinate systems.
    """

    STANDARD_NAMES = ("G54", "G55", "G56", "G57", "G58", "G59")

    def __init__(self):
        self._systems: Dict[str, WorkCoordinateSystem] = {}
        for i, name in enumerate(self.STANDARD_NAMES):
            wcs = WorkCoordinateSystem(
                name=name,
                description=f"Standard work coordinate system {name}",
            )
            if i == 0:
                wcs.is_active = True  # G54 active by default
            self._systems[name] = wcs

    # -- Set / Get offsets -------------------------------------------

    def set_offset(self, name: str, x: float, y: float, z: float) -> None:
        """Sets the XYZ offset of a named coordinate system."""
        if not name:
            raise ValueError("name must not be null or empty")
        if name not in self._systems:
            raise KeyError(f"Coordinate system '{name}' not found")
        wcs = self._systems[name]
        wcs.offset_x = x
        wcs.offset_y = y
        wcs.offset_z = z

    def get_offset(self, name: str) -> Tuple[float, float, float]:
        """Gets the current XYZ offset of a named coordinate system."""
        if not name:
            raise ValueError("name must not be null or empty")
        if name not in self._systems:
            raise KeyError(f"Coordinate system '{name}' not found")
        wcs = self._systems[name]
        return (wcs.offset_x, wcs.offset_y, wcs.offset_z)

    # -- Active system -----------------------------------------------

    def set_active_system(self, name: str) -> None:
        """Activates the named coordinate system and deactivates all others."""
        if not name:
            raise ValueError("name must not be null or empty")
        if name not in self._systems:
            raise KeyError(f"Coordinate system '{name}' not found")
        for wcs in self._systems.values():
            wcs.is_active = False
        self._systems[name].is_active = True

    # -- Transform ---------------------------------------------------

    def transform_point(
        self, x: float, y: float, z: float,
        from_system: str, to_system: str
    ) -> Tuple[float, float, float]:
        """Transforms a point from one coordinate system to another by
        converting through machine coordinates."""
        if not from_system:
            raise ValueError("from_system must not be null or empty")
        if not to_system:
            raise ValueError("to_system must not be null or empty")
        if from_system not in self._systems:
            raise KeyError(f"Coordinate system '{from_system}' not found")
        if to_system not in self._systems:
            raise KeyError(f"Coordinate system '{to_system}' not found")

        src = self._systems[from_system]
        dst = self._systems[to_system]

        # Point in machine coords = point_in_from + from.offset
        mach_x = x + src.offset_x
        mach_y = y + src.offset_y
        mach_z = z + src.offset_z

        # Point in target coords = machine_point - to.offset
        return (mach_x - dst.offset_x, mach_y - dst.offset_y, mach_z - dst.offset_z)

    # -- Query -------------------------------------------------------

    def get_all_systems(self) -> List[WorkCoordinateSystem]:
        """Returns all defined coordinate systems."""
        return list(self._systems.values())

    # -- Custom systems ----------------------------------------------

    def add_custom_system(
        self, name: str, x: float, y: float, z: float, description: str = ""
    ) -> None:
        """Adds a custom coordinate system (typically G54.1 Pn)."""
        if not name:
            raise ValueError("name must not be null or empty")
        if name in self._systems:
            raise RuntimeError(f"Coordinate system '{name}' already exists")
        self._systems[name] = WorkCoordinateSystem(
            name=name,
            offset_x=x,
            offset_y=y,
            offset_z=z,
            description=description,
        )

    # -- Transform chain ---------------------------------------------

    def get_transform_chain(
        self, from_system: str, to_system: str
    ) -> CoordinateTransform:
        """Returns a CoordinateTransform describing the delta between two
        coordinate systems."""
        if not from_system:
            raise ValueError("from_system must not be null or empty")
        if not to_system:
            raise ValueError("to_system must not be null or empty")
        if from_system not in self._systems:
            raise KeyError(f"Coordinate system '{from_system}' not found")
        if to_system not in self._systems:
            raise KeyError(f"Coordinate system '{to_system}' not found")

        src = self._systems[from_system]
        dst = self._systems[to_system]

        return CoordinateTransform(
            from_system=from_system,
            to_system=to_system,
            delta_x=dst.offset_x - src.offset_x,
            delta_y=dst.offset_y - src.offset_y,
            delta_z=dst.offset_z - src.offset_z,
        )


# ---- Fixtures (pytest) ----

@pytest.fixture
def manager():
    return CoordinateSystemManager()


# ---- Tests ----

def test_initial_standard_systems(manager):
    """Constructor creates G54-G59 with zero offsets; G54 is active."""
    systems = manager.get_all_systems()
    names = {s.name for s in systems}
    assert names == {"G54", "G55", "G56", "G57", "G58", "G59"}

    for s in systems:
        assert s.offset_x == pytest.approx(0.0)
        assert s.offset_y == pytest.approx(0.0)
        assert s.offset_z == pytest.approx(0.0)

    active = [s for s in systems if s.is_active]
    assert len(active) == 1
    assert active[0].name == "G54"


def test_set_and_get_offset(manager):
    """SetOffset stores values that GetOffset retrieves."""
    manager.set_offset("G54", 100.0, 200.0, -50.0)
    x, y, z = manager.get_offset("G54")
    assert x == pytest.approx(100.0)
    assert y == pytest.approx(200.0)
    assert z == pytest.approx(-50.0)


def test_set_offset_unknown_system_raises(manager):
    """Setting offset on a non-existent system raises KeyError."""
    with pytest.raises(KeyError, match="not found"):
        manager.set_offset("G60", 1.0, 2.0, 3.0)


def test_set_active_system(manager):
    """SetActiveSystem activates the named system and deactivates all others."""
    manager.set_active_system("G56")
    systems = manager.get_all_systems()

    active = [s for s in systems if s.is_active]
    assert len(active) == 1
    assert active[0].name == "G56"

    # Switch again
    manager.set_active_system("G59")
    active = [s for s in manager.get_all_systems() if s.is_active]
    assert len(active) == 1
    assert active[0].name == "G59"


def test_set_active_system_unknown_raises(manager):
    """Activating a non-existent system raises KeyError."""
    with pytest.raises(KeyError, match="not found"):
        manager.set_active_system("G99")


def test_transform_point_between_systems(manager):
    """TransformPoint correctly converts a point between two offset systems."""
    manager.set_offset("G54", 100.0, 200.0, 0.0)
    manager.set_offset("G55", 300.0, 100.0, 0.0)

    # Point (10, 20, 5) in G54 -> machine = (110, 220, 5)
    # -> in G55 = (110 - 300, 220 - 100, 5 - 0) = (-190, 120, 5)
    rx, ry, rz = manager.transform_point(10.0, 20.0, 5.0, "G54", "G55")
    assert rx == pytest.approx(-190.0)
    assert ry == pytest.approx(120.0)
    assert rz == pytest.approx(5.0)


def test_transform_point_same_system_identity(manager):
    """Transforming a point to the same system returns the original point."""
    manager.set_offset("G54", 50.0, 60.0, 70.0)
    rx, ry, rz = manager.transform_point(1.0, 2.0, 3.0, "G54", "G54")
    assert rx == pytest.approx(1.0)
    assert ry == pytest.approx(2.0)
    assert rz == pytest.approx(3.0)


def test_add_custom_system(manager):
    """AddCustomSystem registers a G54.1 Pn style system."""
    manager.add_custom_system("G54.1 P1", 500.0, 600.0, -10.0,
                              description="Tombstone face A")
    systems = manager.get_all_systems()
    names = {s.name for s in systems}
    assert "G54.1 P1" in names

    x, y, z = manager.get_offset("G54.1 P1")
    assert x == pytest.approx(500.0)
    assert y == pytest.approx(600.0)
    assert z == pytest.approx(-10.0)


def test_add_custom_system_duplicate_raises(manager):
    """Adding a custom system with an existing name raises RuntimeError."""
    manager.add_custom_system("G54.1 P1", 0.0, 0.0, 0.0)
    with pytest.raises(RuntimeError, match="already exists"):
        manager.add_custom_system("G54.1 P1", 1.0, 1.0, 1.0)


def test_add_standard_name_as_custom_raises(manager):
    """Adding a custom system using a standard name (G54) raises RuntimeError."""
    with pytest.raises(RuntimeError, match="already exists"):
        manager.add_custom_system("G54", 0.0, 0.0, 0.0)


def test_get_transform_chain(manager):
    """GetTransformChain returns the correct delta between systems."""
    manager.set_offset("G54", 100.0, 200.0, 50.0)
    manager.set_offset("G55", 400.0, 100.0, 30.0)

    chain = manager.get_transform_chain("G54", "G55")
    assert chain.from_system == "G54"
    assert chain.to_system == "G55"
    assert chain.delta_x == pytest.approx(300.0)
    assert chain.delta_y == pytest.approx(-100.0)
    assert chain.delta_z == pytest.approx(-20.0)


def test_transform_chain_inverse(manager):
    """Transform chain from A->B has negated deltas compared to B->A."""
    manager.set_offset("G56", 10.0, 20.0, 30.0)
    manager.set_offset("G57", 70.0, 80.0, 90.0)

    forward = manager.get_transform_chain("G56", "G57")
    backward = manager.get_transform_chain("G57", "G56")

    assert forward.delta_x == pytest.approx(-backward.delta_x)
    assert forward.delta_y == pytest.approx(-backward.delta_y)
    assert forward.delta_z == pytest.approx(-backward.delta_z)


def test_transform_point_with_custom_system(manager):
    """TransformPoint works between a standard and custom system."""
    manager.set_offset("G54", 0.0, 0.0, 0.0)
    manager.add_custom_system("G54.1 P5", 1000.0, 2000.0, -100.0,
                              description="Pallet station 5")

    # Point (0, 0, 0) in G54 -> machine = (0, 0, 0)
    # -> in G54.1 P5 = (0 - 1000, 0 - 2000, 0 - (-100)) = (-1000, -2000, 100)
    rx, ry, rz = manager.transform_point(0.0, 0.0, 0.0, "G54", "G54.1 P5")
    assert rx == pytest.approx(-1000.0)
    assert ry == pytest.approx(-2000.0)
    assert rz == pytest.approx(100.0)


def test_get_offset_empty_name_raises(manager):
    """GetOffset with empty name raises ValueError."""
    with pytest.raises(ValueError, match="null or empty"):
        manager.get_offset("")


def test_multiple_custom_systems(manager):
    """Multiple G54.1 Pn systems can coexist and be queried independently."""
    for i in range(1, 49):
        manager.add_custom_system(
            f"G54.1 P{i}", float(i * 10), float(i * 20), float(i * -5),
            description=f"Extended WCS P{i}"
        )

    # Should have 6 standard + 48 custom = 54 total
    assert len(manager.get_all_systems()) == 54

    x, y, z = manager.get_offset("G54.1 P48")
    assert x == pytest.approx(480.0)
    assert y == pytest.approx(960.0)
    assert z == pytest.approx(-240.0)
