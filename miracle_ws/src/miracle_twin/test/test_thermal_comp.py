"""Tests for ThermalCompensationCalculator in CuttingSimProxy."""
import sys
from unittest.mock import MagicMock

# Mock modules that are unavailable in the test environment.
for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

# Also mock ROS2 / miracle infrastructure used transitively.
for mod in (
    'rclpy', 'rclpy.lifecycle', 'rclpy.node', 'rclpy.qos',
    'rclpy.parameter', 'rclpy.callback_groups', 'rclpy.executors',
    'std_msgs', 'std_msgs.msg',
    'miracle_core.gcode_parser', 'miracle_core.tool_library',
    'miracle_msgs', 'miracle_msgs.msg', 'miracle_msgs.srv',
):
    if mod in sys.modules:
        existing = sys.modules[mod]
        if not hasattr(existing, '__path__'):
            setattr(existing, '__path__', [])
    else:
        sys.modules[mod] = MagicMock()

import math
import time

import pytest

from miracle_twin.cutting_sim_proxy import (
    CompensationOffset,
    ThermalCompensationCalculator,
    ThermalCompensationReport,
    ThermalReading,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_calculator(*axes_mm):
    """Return a pre-configured calculator.

    *axes_mm* is a sequence of (axis_name, length_mm) tuples.
    """
    calc = ThermalCompensationCalculator()
    for axis, length in axes_mm:
        calc.set_axis_length(axis, length)
    return calc


def _reading(axis: str, temp: float, loc: str = "bearing") -> ThermalReading:
    return ThermalReading(axis=axis, temperature_c=temp,
                          timestamp=time.time(), sensor_location=loc)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestThermalCompensationCalculator:
    """Unit tests for ThermalCompensationCalculator."""

    # 1. Basic expansion calculation
    def test_basic_expansion(self):
        calc = _make_calculator(("X", 500.0))
        calc.set_reference_temperature("X", 20.0)
        calc.record_temperature(_reading("X", 30.0))  # +10 degC

        offset = calc.calculate_compensation("X")

        # DeltaL = 500mm * 11.7e-6 * 10 * 1000 = 58.5 um
        assert offset.axis == "X"
        assert math.isclose(offset.offset_um, 58.5, rel_tol=1e-4)
        assert math.isclose(offset.temperature_delta_c, 10.0)
        assert offset.reference_temp_c == 20.0

    # 2. Default reference temperature is 20 degC
    def test_default_reference_temperature(self):
        calc = _make_calculator(("Y", 400.0))
        calc.record_temperature(_reading("Y", 25.0))

        offset = calc.calculate_compensation("Y")

        assert offset.reference_temp_c == 20.0
        assert math.isclose(offset.temperature_delta_c, 5.0)

    # 3. Custom CTE
    def test_custom_cte(self):
        calc = _make_calculator(("Z", 300.0))
        cte_aluminum = 23.1e-6
        calc.set_cte("Z", cte_aluminum)
        calc.set_reference_temperature("Z", 20.0)
        calc.record_temperature(_reading("Z", 30.0))

        offset = calc.calculate_compensation("Z")
        expected = 300.0 * cte_aluminum * 10.0 * 1000.0  # 69.3 um
        assert math.isclose(offset.offset_um, expected, rel_tol=1e-4)
        assert offset.cte == cte_aluminum

    # 4. No readings raises ValueError
    def test_no_readings_raises(self):
        calc = _make_calculator(("X", 500.0))
        with pytest.raises(ValueError, match="No temperature readings"):
            calc.calculate_compensation("X")

    # 5. No axis length raises ValueError
    def test_no_axis_length_raises(self):
        calc = ThermalCompensationCalculator()
        calc.record_temperature(_reading("X", 25.0))
        with pytest.raises(ValueError, match="No axis length configured"):
            calc.calculate_compensation("X")

    # 6. Report aggregates all axes
    def test_report_aggregation(self):
        calc = _make_calculator(("X", 500.0), ("Y", 400.0), ("Z", 300.0))
        calc.set_reference_temperature("X", 20.0)
        calc.set_reference_temperature("Y", 20.0)
        calc.set_reference_temperature("Z", 20.0)

        calc.record_temperature(_reading("X", 30.0))
        calc.record_temperature(_reading("Y", 25.0))
        calc.record_temperature(_reading("Z", 22.0))

        report = calc.get_report()

        assert isinstance(report, ThermalCompensationReport)
        assert len(report.offsets) == 3
        assert report.total_compensation_um > 0
        assert report.max_axis_error_um > 0
        # X axis has the largest delta so it should define max
        x_offset = next(o for o in report.offsets if o.axis == "X")
        assert math.isclose(report.max_axis_error_um, abs(x_offset.offset_um),
                            rel_tol=1e-4)

    # 7. is_compensation_needed with default threshold
    def test_compensation_needed(self):
        calc = _make_calculator(("X", 500.0))
        calc.set_reference_temperature("X", 20.0)
        # Small delta: 500 * 11.7e-6 * 0.5 * 1000 = 2.925 um < 5 um
        calc.record_temperature(_reading("X", 20.5))
        assert calc.is_compensation_needed() is False

        # Larger delta: 500 * 11.7e-6 * 2 * 1000 = 11.7 um > 5 um
        calc.record_temperature(_reading("X", 22.0))
        assert calc.is_compensation_needed() is True

    # 8. is_compensation_needed with custom threshold
    def test_compensation_needed_custom_threshold(self):
        calc = _make_calculator(("X", 500.0))
        calc.set_reference_temperature("X", 20.0)
        # 500 * 11.7e-6 * 2 * 1000 = 11.7 um
        calc.record_temperature(_reading("X", 22.0))
        assert calc.is_compensation_needed(threshold_um=15.0) is False
        assert calc.is_compensation_needed(threshold_um=10.0) is True

    # 9. Latest reading is used for compensation
    def test_latest_reading_used(self):
        calc = _make_calculator(("X", 500.0))
        calc.set_reference_temperature("X", 20.0)
        calc.record_temperature(_reading("X", 25.0))
        calc.record_temperature(_reading("X", 30.0))  # latest

        offset = calc.calculate_compensation("X")
        assert math.isclose(offset.temperature_delta_c, 10.0)

    # 10. Negative temperature delta (cooling below reference)
    def test_negative_delta(self):
        calc = _make_calculator(("X", 500.0))
        calc.set_reference_temperature("X", 20.0)
        calc.record_temperature(_reading("X", 15.0))

        offset = calc.calculate_compensation("X")
        assert offset.offset_um < 0
        expected = 500.0 * 11.7e-6 * (-5.0) * 1000.0
        assert math.isclose(offset.offset_um, expected, rel_tol=1e-4)

    # 11. Report is_significant flag
    def test_report_significance(self):
        calc = _make_calculator(("X", 500.0))
        calc.set_reference_temperature("X", 20.0)

        # Small change -> not significant
        calc.record_temperature(_reading("X", 20.1))
        report = calc.get_report()
        assert report.is_significant is False

        # Large change -> significant
        calc.record_temperature(_reading("X", 35.0))
        report = calc.get_report()
        assert report.is_significant is True
