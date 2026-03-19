"""Tests for ToolRunoutCompensator in CuttingSimProxy."""
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
    RunoutCompensation,
    RunoutEffect,
    RunoutMeasurement,
    ToolRunoutCompensator,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_measurement(
    tir_mm: float = 0.02,
    eccentricity_mm: float = 0.01,
    angle_deg: float = 45.0,
    timestamp: float = 0.0,
    method: str = 'dial_indicator',
) -> RunoutMeasurement:
    return RunoutMeasurement(
        tir_mm=tir_mm,
        eccentricity_mm=eccentricity_mm,
        angle_deg=angle_deg,
        timestamp=timestamp or time.time(),
        measurement_method=method,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestRecordAndHistory:
    """Tests for record_measurement and get_runout_history."""

    def test_record_single_measurement(self):
        comp = ToolRunoutCompensator()
        m = _make_measurement()
        comp.record_measurement(m)
        history = comp.get_runout_history()
        assert len(history) == 1
        # Returned list is a copy of the internal list (not the same object).
        assert comp.get_runout_history() is not comp.get_runout_history()
        assert history[0].tir_mm == m.tir_mm

    def test_record_multiple_measurements_preserves_order(self):
        comp = ToolRunoutCompensator()
        m1 = _make_measurement(tir_mm=0.01, timestamp=1.0)
        m2 = _make_measurement(tir_mm=0.02, timestamp=2.0)
        m3 = _make_measurement(tir_mm=0.03, timestamp=3.0)
        comp.record_measurement(m1)
        comp.record_measurement(m2)
        comp.record_measurement(m3)
        history = comp.get_runout_history()
        assert len(history) == 3
        assert [h.tir_mm for h in history] == [0.01, 0.02, 0.03]

    def test_empty_history(self):
        comp = ToolRunoutCompensator()
        assert comp.get_runout_history() == []


class TestCalculateEffects:
    """Tests for calculate_effects."""

    def test_zero_runout_gives_zero_effects(self):
        comp = ToolRunoutCompensator()
        effect = comp.calculate_effects(tir_mm=0.0, num_flutes=4, feed_per_tooth=0.1)
        assert effect.force_variation_pct == 0.0
        assert effect.surface_roughness_increase_um == 0.0
        assert effect.effective_chip_load_variation_pct == 0.0
        assert effect.tool_life_reduction_pct == 0.0

    def test_moderate_runout_effects(self):
        comp = ToolRunoutCompensator()
        effect = comp.calculate_effects(tir_mm=0.02, num_flutes=4, feed_per_tooth=0.1)
        # chip_load_var = (0.02 / 0.1) * 100 = 20%
        assert effect.effective_chip_load_variation_pct == 20.0
        # force_var = 20 * 1.2 = 24%
        assert effect.force_variation_pct == 24.0
        # roughness = (0.02/0.01)*2*(2/4) = 2.0 um
        assert effect.surface_roughness_increase_um == 2.0
        # life_reduction = (0.02/0.01)*5 = 10%
        assert effect.tool_life_reduction_pct == 10.0

    def test_high_runout_life_reduction_capped(self):
        comp = ToolRunoutCompensator()
        effect = comp.calculate_effects(tir_mm=0.5, num_flutes=2, feed_per_tooth=0.1)
        # life_reduction = (0.5/0.01)*5 = 250 => capped at 80
        assert effect.tool_life_reduction_pct == 80.0

    def test_invalid_feed_per_tooth_raises(self):
        comp = ToolRunoutCompensator()
        with pytest.raises(ValueError, match="feed_per_tooth must be positive"):
            comp.calculate_effects(tir_mm=0.01, num_flutes=4, feed_per_tooth=0.0)


class TestRecommendCompensation:
    """Tests for recommend_compensation."""

    def test_low_runout_speed_increase(self):
        comp = ToolRunoutCompensator()
        result = comp.recommend_compensation(tir_mm=0.01, num_flutes=4, feed_per_tooth=0.1)
        # TIR <= 0.03 => speed_adj = +5%
        assert result.speed_adjustment_pct == 5.0
        # Feed should be reduced
        assert result.feed_adjustment_pct < 0

    def test_high_runout_speed_decrease(self):
        comp = ToolRunoutCompensator()
        result = comp.recommend_compensation(tir_mm=0.05, num_flutes=4, feed_per_tooth=0.1)
        # TIR > 0.03 => speed reduction
        assert result.speed_adjustment_pct < 0

    def test_compensation_has_positive_improvement(self):
        comp = ToolRunoutCompensator()
        result = comp.recommend_compensation(tir_mm=0.02, num_flutes=4, feed_per_tooth=0.1)
        assert result.estimated_improvement_pct > 0


class TestGetEffectiveChipLoads:
    """Tests for get_effective_chip_loads."""

    def test_zero_runout_gives_uniform_loads(self):
        comp = ToolRunoutCompensator()
        loads = comp.get_effective_chip_loads(nominal_fpt=0.1, tir_mm=0.0, num_flutes=4)
        assert len(loads) == 4
        assert all(l == 0.1 for l in loads)

    def test_nonzero_runout_sinusoidal_distribution(self):
        comp = ToolRunoutCompensator()
        loads = comp.get_effective_chip_loads(nominal_fpt=0.1, tir_mm=0.02, num_flutes=4)
        assert len(loads) == 4
        # Flute 0 (angle=0) gets max: 0.1 + 0.01 = 0.11
        assert loads[0] == pytest.approx(0.11, abs=1e-5)
        # Flute 2 (angle=pi) gets min: 0.1 - 0.01 = 0.09
        assert loads[2] == pytest.approx(0.09, abs=1e-5)
        # Flutes 1 and 3 (angle=pi/2, 3pi/2) stay at nominal
        assert loads[1] == pytest.approx(0.1, abs=1e-5)
        assert loads[3] == pytest.approx(0.1, abs=1e-5)

    def test_chip_loads_sum_roughly_to_nominal_times_flutes(self):
        comp = ToolRunoutCompensator()
        loads = comp.get_effective_chip_loads(nominal_fpt=0.1, tir_mm=0.03, num_flutes=6)
        assert len(loads) == 6
        # Sum should be close to 6 * 0.1 = 0.6 (runout redistributes, not adds)
        assert sum(loads) == pytest.approx(0.6, abs=0.01)

    def test_chip_load_never_negative(self):
        comp = ToolRunoutCompensator()
        # Large runout relative to fpt — ensure no negative loads
        loads = comp.get_effective_chip_loads(nominal_fpt=0.01, tir_mm=0.05, num_flutes=2)
        assert all(l >= 0.0 for l in loads)


class TestIsAcceptable:
    """Tests for is_acceptable."""

    def test_within_default_tolerance(self):
        comp = ToolRunoutCompensator()
        assert comp.is_acceptable(tir_mm=0.005) is True

    def test_at_tolerance_boundary(self):
        comp = ToolRunoutCompensator()
        assert comp.is_acceptable(tir_mm=0.01) is True

    def test_exceeds_default_tolerance(self):
        comp = ToolRunoutCompensator()
        assert comp.is_acceptable(tir_mm=0.015) is False

    def test_custom_tolerance(self):
        comp = ToolRunoutCompensator()
        assert comp.is_acceptable(tir_mm=0.025, tolerance_mm=0.03) is True
        assert comp.is_acceptable(tir_mm=0.035, tolerance_mm=0.03) is False
