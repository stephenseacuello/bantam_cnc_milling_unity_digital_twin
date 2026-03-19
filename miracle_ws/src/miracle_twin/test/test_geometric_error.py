"""Tests for GeometricErrorModel — machine geometric error analysis.

Validates squareness/straightness/positioning error modelling,
volumetric error calculation (RSS), compensation table generation,
and ISO 230-2 grade classification.
"""
import sys
import math
from unittest.mock import MagicMock

# ── Module shim (standard pattern) ──────────────────────────────────
for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_twin.cutting_sim_proxy import (
    GeometricError,
    ErrorCompensation,
    GeometricErrorModel,
    MachineAccuracyReport,
)


# ── Helpers ──────────────────────────────────────────────────────────

def _make_model_with_positioning_errors() -> GeometricErrorModel:
    """Return a model pre-loaded with positioning errors on X axis."""
    model = GeometricErrorModel()
    model.record_error(GeometricError('positioning', 'X', 5.0, 'positive', 0.0))
    model.record_error(GeometricError('positioning', 'X', 10.0, 'positive', 100.0))
    model.record_error(GeometricError('positioning', 'X', 15.0, 'positive', 200.0))
    return model


# ── Tests ────────────────────────────────────────────────────────────


class TestRecordError:
    def test_record_single_error(self):
        model = GeometricErrorModel()
        err = GeometricError('squareness', 'XY', 3.5, 'positive', 0.0)
        model.record_error(err)
        report = model.get_accuracy_report()
        assert len(report.errors) == 1
        assert report.errors[0].error_type == 'squareness'

    def test_record_multiple_errors(self):
        model = GeometricErrorModel()
        model.record_error(GeometricError('squareness', 'XY', 3.5, 'positive', 0.0))
        model.record_error(GeometricError('straightness', 'X', 2.0, 'bidirectional', 50.0))
        model.record_error(GeometricError('positioning', 'Z', 4.0, 'negative', 100.0))
        report = model.get_accuracy_report()
        assert len(report.errors) == 3


class TestGetPositioningError:
    def test_no_data_returns_zero(self):
        model = GeometricErrorModel()
        assert model.get_positioning_error('X', 50.0) == 0.0

    def test_single_point_returns_value(self):
        model = GeometricErrorModel()
        model.record_error(GeometricError('positioning', 'X', 7.0, 'positive', 100.0))
        assert model.get_positioning_error('X', 50.0) == 7.0

    def test_interpolation_midpoint(self):
        model = _make_model_with_positioning_errors()
        # Midpoint between 0 mm (5 um) and 100 mm (10 um) => 7.5 um
        result = model.get_positioning_error('X', 50.0)
        assert result == pytest.approx(7.5)

    def test_interpolation_clamp_low(self):
        model = _make_model_with_positioning_errors()
        # Below range should clamp to first value
        assert model.get_positioning_error('X', -10.0) == pytest.approx(5.0)

    def test_interpolation_clamp_high(self):
        model = _make_model_with_positioning_errors()
        # Above range should clamp to last value
        assert model.get_positioning_error('X', 300.0) == pytest.approx(15.0)

    def test_exact_data_point(self):
        model = _make_model_with_positioning_errors()
        assert model.get_positioning_error('X', 100.0) == pytest.approx(10.0)

    def test_different_axis_ignored(self):
        model = _make_model_with_positioning_errors()
        # Y axis has no data
        assert model.get_positioning_error('Y', 50.0) == 0.0


class TestCalculateVolumetricError:
    def test_empty_model_returns_zero(self):
        model = GeometricErrorModel()
        assert model.calculate_volumetric_error(0, 0, 0) == 0.0

    def test_single_positioning_error(self):
        model = GeometricErrorModel()
        model.record_error(GeometricError('positioning', 'X', 4.0, 'positive', 0.0))
        result = model.calculate_volumetric_error(0, 0, 0)
        assert result == pytest.approx(4.0)

    def test_rss_of_multiple_errors(self):
        model = GeometricErrorModel()
        model.record_error(GeometricError('squareness', 'XY', 3.0, 'positive', 0.0))
        model.record_error(GeometricError('straightness', 'Z', 4.0, 'bidirectional', 0.0))
        expected = math.sqrt(3.0**2 + 4.0**2)
        result = model.calculate_volumetric_error(0, 0, 0)
        assert result == pytest.approx(expected)

    def test_positioning_uses_interpolation(self):
        model = _make_model_with_positioning_errors()
        # At X=50 the interpolated positioning error is 7.5 um
        result = model.calculate_volumetric_error(50.0, 0, 0)
        assert result == pytest.approx(7.5)


class TestGenerateCompensationTable:
    def test_compensation_values_negate_error(self):
        model = _make_model_with_positioning_errors()
        positions = [0.0, 50.0, 100.0, 200.0]
        table = model.generate_compensation_table('X', positions)
        assert len(table) == 4
        # Compensation should be negative of the positioning error
        assert table[0].compensation_um == pytest.approx(-5.0)
        assert table[1].compensation_um == pytest.approx(-7.5)
        assert table[2].compensation_um == pytest.approx(-10.0)
        assert table[3].compensation_um == pytest.approx(-15.0)

    def test_compensation_dataclass_fields(self):
        model = _make_model_with_positioning_errors()
        table = model.generate_compensation_table('X', [50.0])
        entry = table[0]
        assert isinstance(entry, ErrorCompensation)
        assert entry.axis == 'X'
        assert entry.position_mm == 50.0
        assert entry.error_type == 'positioning'


class TestGetAccuracyReport:
    def test_empty_report(self):
        model = GeometricErrorModel()
        report = model.get_accuracy_report()
        assert isinstance(report, MachineAccuracyReport)
        assert report.total_volumetric_error_um == 0.0
        assert report.worst_axis == ''
        assert report.calibration_needed is False

    def test_worst_axis_identification(self):
        model = GeometricErrorModel()
        model.record_error(GeometricError('positioning', 'X', 5.0, 'positive', 0.0))
        model.record_error(GeometricError('positioning', 'Y', 12.0, 'positive', 0.0))
        model.record_error(GeometricError('positioning', 'Z', 3.0, 'negative', 0.0))
        report = model.get_accuracy_report()
        assert report.worst_axis == 'Y'

    def test_calibration_needed_flag(self):
        model = GeometricErrorModel()
        # Add large errors to exceed threshold (15 um)
        model.record_error(GeometricError('positioning', 'X', 12.0, 'positive', 0.0))
        model.record_error(GeometricError('squareness', 'XY', 10.0, 'positive', 0.0))
        report = model.get_accuracy_report()
        assert report.total_volumetric_error_um > 15.0
        assert report.calibration_needed is True

    def test_calibration_not_needed_small_errors(self):
        model = GeometricErrorModel()
        model.record_error(GeometricError('positioning', 'X', 2.0, 'positive', 0.0))
        report = model.get_accuracy_report()
        assert report.calibration_needed is False


class TestClassifyIsoGrade:
    @pytest.mark.parametrize("error_um,expected_grade", [
        (0.0, '1'),
        (2.5, '1'),
        (3.0, '1'),
        (3.1, '2'),
        (5.0, '2'),
        (8.0, '3'),
        (12.0, '4'),
        (20.0, '5'),
        (30.0, '6'),
        (50.0, '7'),
        (80.0, '8'),
        (120.0, '9'),
        (200.0, '10'),
        (201.0, 'ungraded'),
    ])
    def test_grade_boundaries(self, error_um, expected_grade):
        assert GeometricErrorModel.classify_iso_grade(error_um) == expected_grade

    def test_report_includes_iso_grade(self):
        model = GeometricErrorModel()
        model.record_error(GeometricError('positioning', 'X', 2.0, 'positive', 0.0))
        report = model.get_accuracy_report()
        assert report.iso_grade in ('1', '2', '3', '4', '5', '6', '7', '8', '9', '10', 'ungraded')
