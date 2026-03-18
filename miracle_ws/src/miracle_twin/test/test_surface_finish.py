"""Tests for SurfaceFinishPredictor — Ra prediction from cutting parameters."""

import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import math
import pytest

from miracle_twin.cutting_sim_proxy import (
    SurfaceFinishPredictor,
    SurfaceFinishInput,
    SurfaceFinishResult,
)


def _default_input(**overrides) -> SurfaceFinishInput:
    """Helper: build a SurfaceFinishInput with sensible defaults."""
    defaults = dict(
        feed_per_tooth_mm=0.1,
        tool_nose_radius_mm=0.8,
        cutting_speed_m_min=200.0,
        depth_of_cut_mm=1.0,
        tool_wear_vb_mm=0.0,
        vibration_amplitude_mm=0.0,
    )
    defaults.update(overrides)
    return SurfaceFinishInput(**defaults)


class TestPredictRa:
    """Core Ra prediction tests."""

    def test_ideal_roughness_formula(self):
        """Ra_ideal = f^2 / (32*R) * 1000  (mm -> um)."""
        predictor = SurfaceFinishPredictor()
        inp = _default_input(
            feed_per_tooth_mm=0.1,
            tool_nose_radius_mm=0.8,
            cutting_speed_m_min=200.0,  # speed factor = 1.0
            tool_wear_vb_mm=0.0,
            vibration_amplitude_mm=0.0,
        )
        result = predictor.predict_ra(inp)
        expected_ideal = (0.1 ** 2 / (32.0 * 0.8)) * 1000.0  # 0.390625 um
        assert abs(result.ra_theoretical - expected_ideal) < 0.001
        assert abs(result.ra_predicted - expected_ideal) < 0.001

    def test_wear_contribution(self):
        """Flank wear should increase predicted Ra by 0.5*VB*1000 um."""
        predictor = SurfaceFinishPredictor()
        no_wear = predictor.predict_ra(_default_input(tool_wear_vb_mm=0.0))
        with_wear = predictor.predict_ra(_default_input(tool_wear_vb_mm=0.1))
        # At speed=200 m/min, speed_factor=1.0, so wear adds 0.5*0.1*1000 = 50 um
        delta = with_wear.ra_predicted - no_wear.ra_predicted
        assert abs(delta - 50.0) < 0.01

    def test_vibration_contribution(self):
        """Vibration adds amplitude * 0.8 * 1000 um."""
        predictor = SurfaceFinishPredictor()
        no_vib = predictor.predict_ra(_default_input(vibration_amplitude_mm=0.0))
        with_vib = predictor.predict_ra(_default_input(vibration_amplitude_mm=0.01))
        delta = with_vib.ra_predicted - no_vib.ra_predicted
        expected = 0.01 * 0.8 * 1000.0  # 8.0 um
        assert abs(delta - expected) < 0.01

    def test_speed_correction_factor(self):
        """Lower speeds produce higher Ra due to BUE effects."""
        predictor = SurfaceFinishPredictor()
        high_speed = predictor.predict_ra(_default_input(cutting_speed_m_min=200.0))
        low_speed = predictor.predict_ra(_default_input(cutting_speed_m_min=100.0))
        # At 100 m/min: factor = 1.0 + 0.1*(1 - 100/200) = 1.05
        assert low_speed.ra_predicted > high_speed.ra_predicted

    def test_meets_target_true(self):
        """When predicted Ra <= target, meets_target should be True."""
        predictor = SurfaceFinishPredictor()
        inp = _default_input()  # clean cut, low Ra
        result = predictor.predict_ra(inp, target_ra=10.0)
        assert result.meets_target is True

    def test_meets_target_false(self):
        """When predicted Ra > target, meets_target should be False."""
        predictor = SurfaceFinishPredictor()
        inp = _default_input(tool_wear_vb_mm=0.5)  # high wear -> high Ra
        result = predictor.predict_ra(inp, target_ra=0.1)
        assert result.meets_target is False

    def test_ra_components_sum(self):
        """Sum of components should equal the predicted Ra."""
        predictor = SurfaceFinishPredictor()
        inp = _default_input(
            tool_wear_vb_mm=0.05,
            vibration_amplitude_mm=0.005,
        )
        result = predictor.predict_ra(inp)
        component_sum = sum(result.ra_components.values())
        assert abs(component_sum - result.ra_predicted) < 0.01

    def test_zero_nose_radius_raises(self):
        """Zero nose radius is physically invalid and must raise."""
        predictor = SurfaceFinishPredictor()
        inp = _default_input(tool_nose_radius_mm=0.0)
        with pytest.raises(ValueError):
            predictor.predict_ra(inp)


class TestISOGrade:
    """ISO N-grade mapping."""

    def test_known_grades(self):
        predictor = SurfaceFinishPredictor()
        assert predictor.get_iso_grade(0.02) == 'N1'
        assert predictor.get_iso_grade(0.05) == 'N2'
        assert predictor.get_iso_grade(0.1) == 'N3'
        assert predictor.get_iso_grade(0.8) == 'N6'
        assert predictor.get_iso_grade(1.6) == 'N7'
        assert predictor.get_iso_grade(3.2) == 'N8'
        assert predictor.get_iso_grade(50.0) == 'N12'

    def test_exceeds_n12(self):
        predictor = SurfaceFinishPredictor()
        assert predictor.get_iso_grade(100.0) == 'N12+'

    def test_grade_in_result(self):
        predictor = SurfaceFinishPredictor()
        result = predictor.predict_ra(_default_input())
        assert result.quality_grade.startswith('N')


class TestRecommendParameters:
    """Parameter recommendation to achieve target Ra."""

    def test_achievable_target(self):
        predictor = SurfaceFinishPredictor()
        inp = _default_input(feed_per_tooth_mm=0.2, cutting_speed_m_min=100.0)
        rec = predictor.recommend_parameters(target_ra=1.0, current_input=inp)
        assert rec['achievable'] is True
        assert rec['predicted_ra'] <= 1.0

    def test_unachievable_with_high_wear(self):
        """With extreme wear the target may not be reachable."""
        predictor = SurfaceFinishPredictor()
        inp = _default_input(
            feed_per_tooth_mm=0.05,
            tool_wear_vb_mm=0.5,       # very worn tool
            vibration_amplitude_mm=0.1, # significant vibration
        )
        rec = predictor.recommend_parameters(target_ra=0.01, current_input=inp)
        assert rec['achievable'] is False

    def test_recommendation_reduces_feed(self):
        """Recommended feed should be <= current when target is tight."""
        predictor = SurfaceFinishPredictor()
        inp = _default_input(feed_per_tooth_mm=0.15, cutting_speed_m_min=120.0)
        rec = predictor.recommend_parameters(target_ra=1.0, current_input=inp)
        assert rec['recommended_feed_per_tooth_mm'] <= 0.15


class TestPredictBatch:
    """Batch prediction."""

    def test_batch_returns_correct_count(self):
        predictor = SurfaceFinishPredictor()
        inputs = [_default_input(feed_per_tooth_mm=0.05 * (i + 1)) for i in range(5)]
        results = predictor.predict_batch(inputs, target_ra=3.0)
        assert len(results) == 5

    def test_batch_ra_increases_with_feed(self):
        predictor = SurfaceFinishPredictor()
        inputs = [_default_input(feed_per_tooth_mm=f) for f in [0.05, 0.10, 0.20]]
        results = predictor.predict_batch(inputs, target_ra=50.0)
        ra_values = [r.ra_predicted for r in results]
        assert ra_values == sorted(ra_values), "Ra should increase with feed"

    def test_batch_empty_list(self):
        predictor = SurfaceFinishPredictor()
        assert predictor.predict_batch([], target_ra=1.0) == []
