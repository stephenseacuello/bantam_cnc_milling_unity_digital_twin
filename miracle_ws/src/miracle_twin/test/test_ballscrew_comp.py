"""Tests for BallScrewCompensationModel — ball screw pitch error and
thermal growth compensation.

Validates profile creation, pitch-error recording, interpolation,
thermal compensation, backlash calculation, compensation table
generation, and screw health assessment.
"""
import sys
import math
from unittest.mock import MagicMock

# ── Module shim (standard pattern) ──────────────────────────────────
for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_twin.cutting_sim_proxy import (
    BallScrewCompensationModel,
    BallScrewProfile,
    CompensationResult,
    PitchErrorPoint,
)


# ── Helpers ──────────────────────────────────────────────────────────

def _make_model_with_errors() -> BallScrewCompensationModel:
    """Return a model pre-loaded with forward and reverse errors on X."""
    model = BallScrewCompensationModel()
    model.create_profile('X', length=500.0, pitch=10.0, thermal_coefficient=0.012)
    # Forward errors
    model.add_pitch_error('X', 0.0, 0.0, 'forward')
    model.add_pitch_error('X', 100.0, 5.0, 'forward')
    model.add_pitch_error('X', 200.0, 8.0, 'forward')
    model.add_pitch_error('X', 300.0, 12.0, 'forward')
    model.add_pitch_error('X', 400.0, 10.0, 'forward')
    model.add_pitch_error('X', 500.0, 6.0, 'forward')
    # Reverse errors (shifted by backlash)
    model.add_pitch_error('X', 0.0, -2.0, 'reverse')
    model.add_pitch_error('X', 100.0, 2.0, 'reverse')
    model.add_pitch_error('X', 200.0, 4.0, 'reverse')
    model.add_pitch_error('X', 300.0, 7.0, 'reverse')
    model.add_pitch_error('X', 400.0, 5.0, 'reverse')
    model.add_pitch_error('X', 500.0, 1.0, 'reverse')
    return model


# ── Tests ────────────────────────────────────────────────────────────


class TestCreateProfile:
    def test_creates_profile_with_correct_attributes(self):
        model = BallScrewCompensationModel()
        profile = model.create_profile('X', length=600.0, pitch=10.0)
        assert profile.axis == 'X'
        assert profile.screw_length_mm == 600.0
        assert profile.nominal_pitch_mm == 10.0
        assert profile.thermal_coefficient == 0.012
        assert profile.pitch_errors_forward == []
        assert profile.pitch_errors_reverse == []

    def test_custom_thermal_coefficient(self):
        model = BallScrewCompensationModel()
        profile = model.create_profile('Y', length=400.0, pitch=5.0, thermal_coefficient=0.02)
        assert profile.thermal_coefficient == 0.02


class TestAddPitchError:
    def test_add_forward_error(self):
        model = BallScrewCompensationModel()
        model.create_profile('X', 500.0, 10.0)
        model.add_pitch_error('X', 100.0, 5.0, 'forward')
        profile = model._profiles['X']
        assert len(profile.pitch_errors_forward) == 1
        assert profile.pitch_errors_forward[0].error_um == 5.0

    def test_add_reverse_error(self):
        model = BallScrewCompensationModel()
        model.create_profile('X', 500.0, 10.0)
        model.add_pitch_error('X', 100.0, 3.0, 'reverse')
        assert len(model._profiles['X'].pitch_errors_reverse) == 1

    def test_errors_sorted_by_position(self):
        model = BallScrewCompensationModel()
        model.create_profile('X', 500.0, 10.0)
        model.add_pitch_error('X', 300.0, 10.0, 'forward')
        model.add_pitch_error('X', 100.0, 5.0, 'forward')
        model.add_pitch_error('X', 200.0, 8.0, 'forward')
        positions = [p.position_mm for p in model._profiles['X'].pitch_errors_forward]
        assert positions == [100.0, 200.0, 300.0]

    def test_unknown_axis_raises(self):
        model = BallScrewCompensationModel()
        with pytest.raises(KeyError):
            model.add_pitch_error('Z', 0.0, 1.0, 'forward')

    def test_invalid_direction_raises(self):
        model = BallScrewCompensationModel()
        model.create_profile('X', 500.0, 10.0)
        with pytest.raises(ValueError):
            model.add_pitch_error('X', 0.0, 1.0, 'sideways')


class TestGetCompensation:
    def test_interpolated_pitch_error(self):
        model = _make_model_with_errors()
        result = model.get_compensation('X', 150.0, 'forward', temperature_delta=0.0)
        # Midpoint between 100mm (5um) and 200mm (8um) => 6.5um
        assert result.pitch_comp_um == pytest.approx(6.5, abs=0.01)
        assert result.thermal_comp_um == pytest.approx(0.0)

    def test_thermal_compensation(self):
        model = _make_model_with_errors()
        result = model.get_compensation('X', 250.0, 'forward', temperature_delta=5.0)
        # thermal = 0.012 * 5.0 * 250.0 = 15.0 um
        assert result.thermal_comp_um == pytest.approx(15.0, abs=0.01)

    def test_total_compensation_includes_pitch_and_thermal(self):
        model = _make_model_with_errors()
        result = model.get_compensation('X', 200.0, 'forward', temperature_delta=2.0)
        # pitch = 8.0, thermal = 0.012 * 2.0 * 200.0 = 4.8
        assert result.total_comp_um == pytest.approx(8.0 + 4.8, abs=0.01)

    def test_reverse_direction_uses_reverse_errors(self):
        model = _make_model_with_errors()
        fwd = model.get_compensation('X', 100.0, 'forward')
        rev = model.get_compensation('X', 100.0, 'reverse')
        assert fwd.pitch_comp_um == pytest.approx(5.0)
        assert rev.pitch_comp_um == pytest.approx(2.0)

    def test_compensation_at_endpoint(self):
        model = _make_model_with_errors()
        result = model.get_compensation('X', 500.0, 'forward')
        assert result.pitch_comp_um == pytest.approx(6.0)

    def test_compensation_at_zero(self):
        model = _make_model_with_errors()
        result = model.get_compensation('X', 0.0, 'forward')
        assert result.pitch_comp_um == pytest.approx(0.0)

    def test_unknown_axis_raises(self):
        model = BallScrewCompensationModel()
        with pytest.raises(KeyError):
            model.get_compensation('Z', 0.0)


class TestGetBacklash:
    def test_backlash_at_known_point(self):
        model = _make_model_with_errors()
        # At 100mm: forward=5, reverse=2 => backlash=3
        bl = model.get_backlash('X', 100.0)
        assert bl == pytest.approx(3.0)

    def test_backlash_interpolated(self):
        model = _make_model_with_errors()
        # At 150mm: fwd= (5+8)/2=6.5, rev= (2+4)/2=3.0 => backlash=3.5
        bl = model.get_backlash('X', 150.0)
        assert bl == pytest.approx(3.5, abs=0.01)

    def test_backlash_zero_when_no_reverse(self):
        model = BallScrewCompensationModel()
        model.create_profile('Y', 300.0, 5.0)
        model.add_pitch_error('Y', 0.0, 2.0, 'forward')
        model.add_pitch_error('Y', 100.0, 4.0, 'forward')
        # No reverse errors => interpolation returns 0 => backlash = |fwd - 0|
        bl = model.get_backlash('Y', 50.0)
        assert bl == pytest.approx(3.0, abs=0.01)


class TestGenerateCompTable:
    def test_table_covers_full_range(self):
        model = _make_model_with_errors()
        table = model.generate_comp_table('X', interval_mm=100.0)
        positions = [r.position_mm for r in table]
        assert positions[0] == 0.0
        assert positions[-1] == 500.0

    def test_table_entry_count(self):
        model = _make_model_with_errors()
        table = model.generate_comp_table('X', interval_mm=100.0)
        # 0, 100, 200, 300, 400, 500 => 6 entries
        assert len(table) == 6

    def test_table_with_non_divisible_interval(self):
        model = _make_model_with_errors()
        table = model.generate_comp_table('X', interval_mm=150.0)
        # 0, 150, 300, 450 => then endpoint 500 appended => 5
        assert table[-1].position_mm == 500.0
        assert len(table) == 5

    def test_table_includes_thermal_effect(self):
        model = _make_model_with_errors()
        table = model.generate_comp_table('X', interval_mm=250.0, temperature_delta=3.0)
        # At 250mm: thermal = 0.012 * 3.0 * 250 = 9.0
        entry_250 = [r for r in table if r.position_mm == 250.0][0]
        assert entry_250.thermal_comp_um == pytest.approx(9.0, abs=0.01)


class TestGetScrewHealth:
    def test_good_screw(self):
        model = BallScrewCompensationModel()
        model.create_profile('X', 500.0, 10.0)
        model.add_pitch_error('X', 0.0, 1.0, 'forward')
        model.add_pitch_error('X', 250.0, 2.0, 'forward')
        model.add_pitch_error('X', 500.0, 3.0, 'forward')
        model.add_pitch_error('X', 0.0, 0.5, 'reverse')
        model.add_pitch_error('X', 250.0, 1.5, 'reverse')
        model.add_pitch_error('X', 500.0, 2.0, 'reverse')
        health = model.get_screw_health('X')
        assert health['grade'] == 'good'
        assert health['max_error_um'] <= 5.0

    def test_worn_screw(self):
        model = BallScrewCompensationModel()
        model.create_profile('Z', 400.0, 5.0)
        model.add_pitch_error('Z', 0.0, 5.0, 'forward')
        model.add_pitch_error('Z', 200.0, 25.0, 'forward')
        model.add_pitch_error('Z', 400.0, 18.0, 'forward')
        model.add_pitch_error('Z', 0.0, -5.0, 'reverse')
        model.add_pitch_error('Z', 200.0, 15.0, 'reverse')
        model.add_pitch_error('Z', 400.0, 8.0, 'reverse')
        health = model.get_screw_health('Z')
        assert health['grade'] in ('worn', 'critical')
        assert health['max_error_um'] >= 15.0

    def test_no_errors_returns_good(self):
        model = BallScrewCompensationModel()
        model.create_profile('A', 300.0, 5.0)
        health = model.get_screw_health('A')
        assert health['grade'] == 'good'
        assert health['max_error_um'] == 0.0

    def test_health_includes_backlash(self):
        model = _make_model_with_errors()
        health = model.get_screw_health('X')
        assert 'max_backlash_um' in health
        assert health['max_backlash_um'] > 0.0

    def test_health_includes_statistics(self):
        model = _make_model_with_errors()
        health = model.get_screw_health('X')
        assert 'mean_error_um' in health
        assert 'error_std_um' in health
        assert health['mean_error_um'] > 0.0
        assert health['error_std_um'] >= 0.0
