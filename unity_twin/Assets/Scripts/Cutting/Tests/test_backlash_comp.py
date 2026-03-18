"""Tests for BacklashCompensationManager logic (Python mirror of C# implementation).

Validates per-axis backlash configuration, directional compensation,
backlash verification testing, recalibration checks, and edge cases.
"""

import pytest
from dataclasses import dataclass, field
from datetime import datetime, timezone, timedelta
from typing import Dict, List, Optional


# ---- Python mirror of C# types ----

@dataclass
class AxisBacklash:
    axis_name: str = ""
    backlash_mm: float = 0.0
    compensation_mm: float = 0.0
    last_calibrated: str = ""
    direction: int = 1  # +1 or -1

    def __init__(self, axis_name: str = "", backlash_mm: float = 0.0):
        self.axis_name = axis_name
        self.backlash_mm = backlash_mm
        self.compensation_mm = backlash_mm  # default: compensate fully
        self.last_calibrated = datetime.now(timezone.utc).isoformat()
        self.direction = 1


@dataclass
class BacklashTestResult:
    axis_name: str = ""
    measured_backlash: float = 0.0
    applied_compensation: float = 0.0
    residual_error: float = 0.0
    test_date: str = ""
    passed: bool = False


class BacklashCompensationManager:
    """Manages per-axis backlash compensation values for CNC machines.

    Tracks mechanical backlash, applies directional compensation on
    direction reversals, and supports calibration verification testing.
    """

    PASS_THRESHOLD_MM = 0.005

    def __init__(self):
        self._axes: Dict[str, AxisBacklash] = {
            "X": AxisBacklash("X", 0.01),
            "Y": AxisBacklash("Y", 0.01),
            "Z": AxisBacklash("Z", 0.01),
        }

    # -- Set / Get ---------------------------------------------------

    def set_backlash(self, axis: str, value: float) -> None:
        """Set (or update) the backlash value for a given axis."""
        if not axis:
            raise ValueError("axis must not be null or empty")
        if value < 0:
            raise ValueError("backlash value must be non-negative")

        if axis in self._axes:
            self._axes[axis].backlash_mm = value
            self._axes[axis].compensation_mm = value
            self._axes[axis].last_calibrated = datetime.now(timezone.utc).isoformat()
        else:
            self._axes[axis] = AxisBacklash(axis, value)

    def get_backlash(self, axis: str) -> Optional[AxisBacklash]:
        """Return the current backlash configuration for an axis."""
        if not axis:
            return None
        return self._axes.get(axis)

    # -- Compensation ------------------------------------------------

    def apply_compensation(self, axis: str, move_direction: int) -> float:
        """Calculate compensation offset when moving an axis.

        A non-zero offset is returned only when the direction reverses.
        """
        if not axis:
            raise ValueError("axis must not be null or empty")
        if move_direction not in (1, -1):
            raise ValueError("move_direction must be +1 or -1")
        if axis not in self._axes:
            raise KeyError(f"Axis '{axis}' is not registered")

        ab = self._axes[axis]
        if move_direction != ab.direction:
            ab.direction = move_direction
            return ab.compensation_mm
        return 0.0

    # -- Testing / Verification --------------------------------------

    def run_backlash_test(self, axis: str, measured_value: float) -> BacklashTestResult:
        """Run a backlash verification test on an axis."""
        if not axis:
            raise ValueError("axis must not be null or empty")
        if axis not in self._axes:
            raise KeyError(f"Axis '{axis}' is not registered")

        ab = self._axes[axis]
        residual = abs(measured_value - ab.compensation_mm)

        return BacklashTestResult(
            axis_name=axis,
            measured_backlash=measured_value,
            applied_compensation=ab.compensation_mm,
            residual_error=residual,
            test_date=datetime.now(timezone.utc).isoformat(),
            passed=residual <= self.PASS_THRESHOLD_MM,
        )

    # -- Query helpers -----------------------------------------------

    def get_all_axes(self) -> List[AxisBacklash]:
        """Return all registered axis configurations."""
        return list(self._axes.values())

    def needs_recalibration(self, axis: str, max_age_days: int) -> bool:
        """Check whether an axis's calibration is older than max_age_days."""
        if not axis:
            raise ValueError("axis must not be null or empty")
        if axis not in self._axes:
            raise KeyError(f"Axis '{axis}' is not registered")

        last_cal = datetime.fromisoformat(self._axes[axis].last_calibrated)
        return (datetime.now(timezone.utc) - last_cal).total_seconds() > max_age_days * 86400

    def get_total_compensation(self) -> float:
        """Sum of compensation values across every registered axis."""
        return sum(ab.compensation_mm for ab in self._axes.values())


# ---- Fixtures (pytest) ----

@pytest.fixture
def manager():
    return BacklashCompensationManager()


# ---- Tests ----

def test_initial_default_axes(manager):
    """Constructor initialises X, Y, Z with 0.01 mm backlash."""
    axes = manager.get_all_axes()
    assert len(axes) == 3

    for ab in axes:
        assert ab.axis_name in ("X", "Y", "Z")
        assert ab.backlash_mm == pytest.approx(0.01)
        assert ab.compensation_mm == pytest.approx(0.01)
        assert ab.direction == 1


def test_set_and_get_backlash(manager):
    """SetBacklash updates an existing axis and GetBacklash retrieves it."""
    manager.set_backlash("X", 0.025)
    ab = manager.get_backlash("X")
    assert ab is not None
    assert ab.backlash_mm == pytest.approx(0.025)
    assert ab.compensation_mm == pytest.approx(0.025)


def test_set_backlash_creates_new_axis(manager):
    """SetBacklash on a non-existent axis registers it."""
    manager.set_backlash("A", 0.05)
    ab = manager.get_backlash("A")
    assert ab is not None
    assert ab.axis_name == "A"
    assert ab.backlash_mm == pytest.approx(0.05)
    assert len(manager.get_all_axes()) == 4  # X, Y, Z, A


def test_apply_compensation_on_reversal(manager):
    """ApplyCompensation returns offset only when direction reverses."""
    # Initial direction is +1; moving in +1 should yield 0
    offset = manager.apply_compensation("X", 1)
    assert offset == pytest.approx(0.0)

    # Reverse to -1: should yield the compensation value
    offset = manager.apply_compensation("X", -1)
    assert offset == pytest.approx(0.01)

    # Continue in -1: no compensation
    offset = manager.apply_compensation("X", -1)
    assert offset == pytest.approx(0.0)

    # Reverse again to +1
    offset = manager.apply_compensation("X", 1)
    assert offset == pytest.approx(0.01)


def test_run_backlash_test_pass(manager):
    """RunBacklashTest passes when measured value is close to compensation."""
    result = manager.run_backlash_test("X", 0.012)
    assert result.passed is True
    assert result.axis_name == "X"
    assert result.measured_backlash == pytest.approx(0.012)
    assert result.applied_compensation == pytest.approx(0.01)
    assert result.residual_error == pytest.approx(0.002)
    assert result.test_date != ""


def test_run_backlash_test_fail(manager):
    """RunBacklashTest fails when residual error exceeds threshold."""
    result = manager.run_backlash_test("Y", 0.03)
    assert result.passed is False
    assert result.residual_error == pytest.approx(0.02)


def test_get_total_compensation(manager):
    """GetTotalCompensation sums all axis compensation values."""
    total = manager.get_total_compensation()
    assert total == pytest.approx(0.03)  # 3 axes * 0.01

    manager.set_backlash("X", 0.05)
    total = manager.get_total_compensation()
    assert total == pytest.approx(0.07)  # 0.05 + 0.01 + 0.01


def test_needs_recalibration_fresh(manager):
    """A freshly calibrated axis does NOT need recalibration."""
    assert manager.needs_recalibration("X", max_age_days=30) is False


def test_needs_recalibration_old(manager):
    """An axis calibrated long ago DOES need recalibration."""
    # Backdate the calibration timestamp
    old_date = (datetime.now(timezone.utc) - timedelta(days=60)).isoformat()
    manager.get_backlash("Z").last_calibrated = old_date
    assert manager.needs_recalibration("Z", max_age_days=30) is True


def test_set_backlash_negative_raises(manager):
    """Setting a negative backlash value raises ValueError."""
    with pytest.raises(ValueError, match="non-negative"):
        manager.set_backlash("X", -0.01)


def test_apply_compensation_invalid_direction(manager):
    """ApplyCompensation rejects directions other than +1/-1."""
    with pytest.raises(ValueError, match="move_direction"):
        manager.apply_compensation("X", 0)
    with pytest.raises(ValueError, match="move_direction"):
        manager.apply_compensation("X", 2)


def test_get_backlash_unknown_axis(manager):
    """GetBacklash returns None for an unregistered axis."""
    assert manager.get_backlash("W") is None
    assert manager.get_backlash("") is None
    assert manager.get_backlash(None) is None


def test_run_backlash_test_unknown_axis_raises(manager):
    """RunBacklashTest raises for an unregistered axis."""
    with pytest.raises(KeyError, match="not registered"):
        manager.run_backlash_test("W", 0.01)
