"""Tests for SpecificCuttingEnergyModel in cutting_sim_proxy."""

import sys
import math
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_twin.cutting_sim_proxy import (
    CuttingEnergyInput,
    CuttingEnergyResult,
    SpecificCuttingEnergyModel,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _default_input(**overrides) -> CuttingEnergyInput:
    defaults = dict(
        material='steel',
        cutting_speed_m_min=150.0,
        feed_per_tooth_mm=0.10,
        depth_of_cut_mm=2.0,
        width_of_cut_mm=10.0,
        tool_diameter_mm=12.0,
        num_flutes=4,
        rake_angle_deg=6.0,
    )
    defaults.update(overrides)
    return CuttingEnergyInput(**defaults)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestMaterialLookup:
    """get_material_specific_energy should return known values."""

    def test_known_materials_return_correct_values(self):
        model = SpecificCuttingEnergyModel()
        assert model.get_material_specific_energy('aluminum') == 0.7
        assert model.get_material_specific_energy('steel') == 2.5
        assert model.get_material_specific_energy('stainless') == 3.0
        assert model.get_material_specific_energy('titanium') == 4.0
        assert model.get_material_specific_energy('cast_iron') == 1.5
        assert model.get_material_specific_energy('inconel') == 5.0
        assert model.get_material_specific_energy('brass') == 1.0
        assert model.get_material_specific_energy('copper') == 1.2

    def test_unknown_material_raises_key_error(self):
        model = SpecificCuttingEnergyModel()
        with pytest.raises(KeyError, match="Unknown material"):
            model.get_material_specific_energy('unobtanium')

    def test_case_insensitive_lookup(self):
        model = SpecificCuttingEnergyModel()
        assert model.get_material_specific_energy('Steel') == 2.5
        assert model.get_material_specific_energy('ALUMINUM') == 0.7


class TestApplyCorrections:
    """apply_corrections should adjust base energy for chip thickness, rake, and wear."""

    def test_higher_feed_lowers_specific_energy(self):
        """Kienzle model: thicker chip -> lower specific energy."""
        model = SpecificCuttingEnergyModel()
        kc_low_feed = model.apply_corrections(2.5, feed=0.05, rake_angle=0.0, material='steel')
        kc_high_feed = model.apply_corrections(2.5, feed=0.20, rake_angle=0.0, material='steel')
        assert kc_low_feed > kc_high_feed

    def test_positive_rake_reduces_energy(self):
        """Positive rake angle should reduce specific energy."""
        model = SpecificCuttingEnergyModel()
        kc_zero_rake = model.apply_corrections(2.5, feed=0.10, rake_angle=0.0, material='steel')
        kc_pos_rake = model.apply_corrections(2.5, feed=0.10, rake_angle=10.0, material='steel')
        assert kc_pos_rake < kc_zero_rake

    def test_wear_increases_energy(self):
        """Worn tool requires more energy."""
        model = SpecificCuttingEnergyModel()
        kc_sharp = model.apply_corrections(2.5, feed=0.10, rake_angle=6.0, wear_factor=1.0, material='steel')
        kc_worn = model.apply_corrections(2.5, feed=0.10, rake_angle=6.0, wear_factor=1.3, material='steel')
        assert kc_worn > kc_sharp
        assert kc_worn == pytest.approx(kc_sharp * 1.3, rel=1e-6)

    def test_very_small_feed_is_clamped(self):
        """Feed is clamped to 0.001 to avoid numerical blow-up."""
        model = SpecificCuttingEnergyModel()
        kc = model.apply_corrections(2.5, feed=0.0, rake_angle=0.0, material='steel')
        assert kc > 0 and math.isfinite(kc)


class TestCalculateTorque:
    """calculate_torque should follow T = P*9549/RPM."""

    def test_known_torque_value(self):
        model = SpecificCuttingEnergyModel()
        torque = model.calculate_torque(power_kw=5.0, rpm=3000.0)
        expected = 5.0 * 9549.0 / 3000.0
        assert torque == pytest.approx(expected, rel=1e-6)

    def test_zero_rpm_returns_zero(self):
        model = SpecificCuttingEnergyModel()
        assert model.calculate_torque(power_kw=5.0, rpm=0.0) == 0.0


class TestCalculate:
    """Full calculation pipeline."""

    def test_steel_baseline_positive_results(self):
        model = SpecificCuttingEnergyModel()
        inp = _default_input(material='steel')
        result = model.calculate(inp)

        assert result.specific_energy_j_mm3 > 0
        assert result.total_power_kw > 0
        assert result.mrr_mm3_min > 0
        assert result.tangential_force_n > 0
        assert result.torque_nm > 0
        assert result.efficiency_pct == pytest.approx(80.0)

    def test_aluminum_requires_less_power_than_steel(self):
        model = SpecificCuttingEnergyModel()
        res_al = model.calculate(_default_input(material='aluminum'))
        res_st = model.calculate(_default_input(material='steel'))
        assert res_al.total_power_kw < res_st.total_power_kw

    def test_mrr_scales_with_depth(self):
        model = SpecificCuttingEnergyModel()
        res_small = model.calculate(_default_input(depth_of_cut_mm=1.0))
        res_large = model.calculate(_default_input(depth_of_cut_mm=4.0))
        assert res_large.mrr_mm3_min == pytest.approx(
            res_small.mrr_mm3_min * 4.0, rel=1e-4
        )

    def test_result_is_dataclass(self):
        model = SpecificCuttingEnergyModel()
        result = model.calculate(_default_input())
        assert isinstance(result, CuttingEnergyResult)


class TestCompareMaterials:
    """compare_materials should return results for each requested material."""

    def test_returns_all_requested_materials(self):
        model = SpecificCuttingEnergyModel()
        materials = ['aluminum', 'steel', 'titanium']
        results = model.compare_materials(materials, _default_input())
        assert set(results.keys()) == set(materials)
        for mat in materials:
            assert isinstance(results[mat], CuttingEnergyResult)

    def test_ordering_by_power(self):
        """Harder materials should require more power (same cutting params)."""
        model = SpecificCuttingEnergyModel()
        materials = ['aluminum', 'steel', 'inconel']
        results = model.compare_materials(materials, _default_input())
        assert results['aluminum'].total_power_kw < results['steel'].total_power_kw
        assert results['steel'].total_power_kw < results['inconel'].total_power_kw
