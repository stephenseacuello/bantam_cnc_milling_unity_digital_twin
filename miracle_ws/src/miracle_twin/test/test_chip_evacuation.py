"""Tests for ChipEvacuationModel, ChipFormation, and EvacuationStatus."""
import math
import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_twin.cutting_sim_proxy import (
    ChipEvacuationModel,
    ChipFormation,
    EvacuationStatus,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def model():
    return ChipEvacuationModel()


# ===================================================================
# ChipFormation dataclass defaults
# ===================================================================

class TestChipFormationDefaults:
    def test_default_chip_type(self):
        cf = ChipFormation()
        assert cf.chip_type == 'continuous'

    def test_default_values(self):
        cf = ChipFormation()
        assert cf.chip_thickness_mm == 0.0
        assert cf.chip_ratio == 1.0
        assert cf.shear_angle_deg == 25.0
        assert cf.chip_curl_radius_mm == 5.0


# ===================================================================
# EvacuationStatus dataclass defaults
# ===================================================================

class TestEvacuationStatusDefaults:
    def test_default_effectiveness(self):
        es = EvacuationStatus()
        assert es.effectiveness_pct == 100.0
        assert es.clogging_risk == 'low'
        assert es.coolant_sufficient is True


# ===================================================================
# predict_chip_type
# ===================================================================

class TestPredictChipType:
    def test_ductile_material_continuous(self, model):
        """6061-T6 at moderate speed and low feed should produce continuous chips."""
        result = model.predict_chip_type('6061-T6', feed_mm=0.10, speed_m_min=200.0, rake_angle_deg=10.0)
        assert isinstance(result, ChipFormation)
        assert result.chip_type == 'continuous'

    def test_ductile_material_built_up_edge(self, model):
        """Low speed + high feed on aluminum -> built-up edge."""
        result = model.predict_chip_type('aluminum', feed_mm=0.20, speed_m_min=20.0, rake_angle_deg=5.0)
        assert result.chip_type == 'built_up_edge'

    def test_ductile_material_segmented(self, model):
        """High feed on ductile material -> segmented."""
        result = model.predict_chip_type('copper', feed_mm=0.30, speed_m_min=100.0, rake_angle_deg=10.0)
        assert result.chip_type == 'segmented'

    def test_brittle_material_discontinuous(self, model):
        """Cast iron should always produce discontinuous chips."""
        result = model.predict_chip_type('cast_iron', feed_mm=0.10, speed_m_min=100.0, rake_angle_deg=5.0)
        assert result.chip_type == 'discontinuous'

    def test_hard_material_segmented(self, model):
        """Titanium at high feed -> segmented."""
        result = model.predict_chip_type('Ti-6Al-4V', feed_mm=0.25, speed_m_min=60.0, rake_angle_deg=5.0)
        assert result.chip_type == 'segmented'

    def test_shear_angle_positive(self, model):
        """Predicted shear angle should be positive for valid inputs."""
        result = model.predict_chip_type('6061-T6', feed_mm=0.10, speed_m_min=200.0, rake_angle_deg=12.0)
        assert result.shear_angle_deg > 0


# ===================================================================
# calculate_chip_thickness
# ===================================================================

class TestCalculateChipThickness:
    def test_90_degree_lead_angle(self, model):
        """At 90-deg lead angle, chip thickness equals feed."""
        h = model.calculate_chip_thickness(feed_mm=0.15, depth_mm=2.0, width_mm=3.0, lead_angle_deg=90.0)
        assert pytest.approx(h, abs=1e-6) == 0.15

    def test_45_degree_lead_angle(self, model):
        """At 45-deg lead angle, chip thickness = feed * sin(45)."""
        h = model.calculate_chip_thickness(feed_mm=0.20, depth_mm=2.0, width_mm=3.0, lead_angle_deg=45.0)
        assert pytest.approx(h, abs=1e-6) == 0.20 * math.sin(math.radians(45.0))

    def test_zero_feed_returns_zero(self, model):
        h = model.calculate_chip_thickness(feed_mm=0.0, depth_mm=2.0, width_mm=3.0)
        assert h == 0.0

    def test_zero_depth_returns_zero(self, model):
        h = model.calculate_chip_thickness(feed_mm=0.15, depth_mm=0.0, width_mm=3.0)
        assert h == 0.0


# ===================================================================
# calculate_chip_ratio
# ===================================================================

class TestCalculateChipRatio:
    def test_ratio_less_than_one(self, model):
        """Chip thicker than feed -> ratio < 1."""
        r = model.calculate_chip_ratio(feed_mm=0.10, chip_thickness_measured=0.20)
        assert r == pytest.approx(0.5)

    def test_ratio_equals_one(self, model):
        r = model.calculate_chip_ratio(feed_mm=0.15, chip_thickness_measured=0.15)
        assert r == pytest.approx(1.0)

    def test_zero_measured_returns_zero(self, model):
        r = model.calculate_chip_ratio(feed_mm=0.10, chip_thickness_measured=0.0)
        assert r == 0.0

    def test_zero_feed_returns_zero(self, model):
        r = model.calculate_chip_ratio(feed_mm=0.0, chip_thickness_measured=0.20)
        assert r == 0.0


# ===================================================================
# evaluate_evacuation
# ===================================================================

class TestEvaluateEvacuation:
    def test_high_pressure_low_risk(self, model):
        """High coolant pressure with 2 flutes should be very effective."""
        status = model.evaluate_evacuation(
            chip_volume_rate=10000, flute_count=2,
            coolant_pressure_bar=80.0, hole_depth_ratio=0.0,
        )
        assert status.effectiveness_pct >= 70.0
        assert status.clogging_risk == 'low'
        assert status.coolant_sufficient is True

    def test_low_pressure_high_clogging(self, model):
        """Very low coolant pressure should flag high clogging risk."""
        status = model.evaluate_evacuation(
            chip_volume_rate=10000, flute_count=4,
            coolant_pressure_bar=1.0, hole_depth_ratio=8.0,
        )
        assert status.clogging_risk in ('medium', 'high')
        assert status.coolant_sufficient is False

    def test_deep_hole_recommendation(self, model):
        """Deep holes with moderate pressure should recommend through-tool coolant."""
        status = model.evaluate_evacuation(
            chip_volume_rate=5000, flute_count=2,
            coolant_pressure_bar=40.0, hole_depth_ratio=6.0,
        )
        assert 'deep hole' in status.recommendation.lower() or 'through-tool' in status.recommendation.lower()

    def test_adequate_conditions(self, model):
        """Good conditions should say evacuation is adequate."""
        status = model.evaluate_evacuation(
            chip_volume_rate=5000, flute_count=2,
            coolant_pressure_bar=80.0, hole_depth_ratio=0.0,
        )
        assert 'adequate' in status.recommendation.lower()


# ===================================================================
# recommend_chip_breaker
# ===================================================================

class TestRecommendChipBreaker:
    def test_discontinuous_no_breaker(self, model):
        rec = model.recommend_chip_breaker('discontinuous', 'cast_iron')
        assert 'no chip breaker' in rec.lower()

    def test_built_up_edge(self, model):
        rec = model.recommend_chip_breaker('built_up_edge', 'aluminum')
        assert 'polished' in rec.lower() or 'positive-rake' in rec.lower()

    def test_continuous_ductile(self, model):
        rec = model.recommend_chip_breaker('continuous', '6061-T6')
        assert 'tight' in rec.lower() or 'curl' in rec.lower()

    def test_segmented_hard(self, model):
        rec = model.recommend_chip_breaker('segmented', 'Inconel_718')
        assert 'heavy-duty' in rec.lower() or 'negative land' in rec.lower()


# ===================================================================
# get_shear_angle (Merchant's equation)
# ===================================================================

class TestGetShearAngle:
    def test_known_values(self, model):
        """For r=0.5, alpha=10 deg the formula should give a positive angle."""
        angle = model.get_shear_angle(chip_ratio=0.5, rake_angle_deg=10.0)
        assert angle > 0
        # Verify against manual calculation
        alpha = math.radians(10.0)
        expected = math.degrees(math.atan2(0.5 * math.cos(alpha), 1.0 - 0.5 * math.sin(alpha)))
        assert angle == pytest.approx(expected, abs=0.01)

    def test_zero_ratio(self, model):
        angle = model.get_shear_angle(chip_ratio=0.0, rake_angle_deg=10.0)
        assert angle == 0.0

    def test_ratio_one_zero_rake(self, model):
        """r=1.0, alpha=0 => tan(phi)=1 => phi=45 degrees."""
        angle = model.get_shear_angle(chip_ratio=1.0, rake_angle_deg=0.0)
        assert angle == pytest.approx(45.0, abs=0.01)

    def test_high_ratio_high_rake(self, model):
        """When denominator approaches zero the result should cap at 90 deg."""
        # r=1.0, alpha=90 => denominator = 1 - 1*sin(90) = 0 => 90
        angle = model.get_shear_angle(chip_ratio=1.0, rake_angle_deg=90.0)
        assert angle == 90.0
