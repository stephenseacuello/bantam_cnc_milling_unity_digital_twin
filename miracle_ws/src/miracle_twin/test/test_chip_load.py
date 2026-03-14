"""Tests for ChipLoadMetrics, ChipLoadMonitor, and CuttingSimProxy.get_chip_load_analysis."""
import math
import pytest

from miracle_twin.cutting_sim_proxy import (
    ChipLoadMetrics,
    ChipLoadMonitor,
    CuttingSimProxy,
    GCodeBlock,
    ToolState,
)


# ---------------------------------------------------------------------------
# Helper factories
# ---------------------------------------------------------------------------

def _make_block(feed=500.0, rpm=8000.0, ap=1.5, ae=3.175, length=20.0):
    return GCodeBlock(
        feed_rate_mmpm=feed,
        spindle_rpm=rpm,
        axial_depth_mm=ap,
        radial_depth_mm=ae,
        length_mm=length,
    )


def _monitor(material='6061-T6', flutes=2, dia=6.35):
    return ChipLoadMonitor(material=material, num_flutes=flutes, tool_diameter_mm=dia)


# ===================================================================
# ChipLoadMetrics dataclass
# ===================================================================

class TestChipLoadMetricsDefaults:
    def test_default_chip_load_is_zero(self):
        m = ChipLoadMetrics()
        assert m.chip_load_mm == 0.0

    def test_default_is_not_optimal(self):
        m = ChipLoadMetrics()
        assert m.is_optimal is False

    def test_default_recommendation_empty(self):
        m = ChipLoadMetrics()
        assert m.recommendation == ''


# ===================================================================
# Basic chip load calculation
# ===================================================================

class TestBasicChipLoad:
    def test_chip_load_formula(self):
        """chip_load = feed / (rpm * flutes)."""
        mon = _monitor(flutes=2)
        m = mon.compute_chip_load(feed_rate_mmpm=1000.0, rpm=5000.0)
        expected = 1000.0 / (5000.0 * 2)
        assert m.chip_load_mm == pytest.approx(expected)

    def test_chip_load_four_flutes(self):
        mon = _monitor(flutes=4)
        m = mon.compute_chip_load(feed_rate_mmpm=1200.0, rpm=6000.0)
        assert m.chip_load_mm == pytest.approx(1200.0 / (6000.0 * 4))

    def test_higher_feed_gives_higher_chip_load(self):
        mon = _monitor()
        m1 = mon.compute_chip_load(500.0, 8000.0)
        m2 = mon.compute_chip_load(1000.0, 8000.0)
        assert m2.chip_load_mm > m1.chip_load_mm

    def test_higher_rpm_gives_lower_chip_load(self):
        mon = _monitor()
        m1 = mon.compute_chip_load(500.0, 4000.0)
        m2 = mon.compute_chip_load(500.0, 8000.0)
        assert m2.chip_load_mm < m1.chip_load_mm


# ===================================================================
# Chip thinning factor
# ===================================================================

class TestChipThinning:
    def test_full_slotting_thinning_is_one(self):
        """ae == d -> thinning factor should be 1.0 (no thinning)."""
        mon = _monitor(dia=10.0)
        m = mon.compute_chip_load(1000.0, 5000.0, ae_mm=10.0)
        assert m.chip_thinning_factor == pytest.approx(1.0)

    def test_half_engagement_thinning(self):
        """ae = d/2 -> inner_term = 1-2*(0.5) = 0 -> thinning = 1.0."""
        mon = _monitor(dia=10.0)
        m = mon.compute_chip_load(1000.0, 5000.0, ae_mm=5.0)
        assert m.chip_thinning_factor == pytest.approx(1.0)

    def test_quarter_engagement_thinning_less_than_one(self):
        """ae = d/4 -> inner_term = 1-2*0.25 = 0.5 -> thinning = sqrt(1-0.25) = sqrt(0.75)."""
        mon = _monitor(dia=10.0)
        m = mon.compute_chip_load(1000.0, 5000.0, ae_mm=2.5)
        expected = math.sqrt(1.0 - (1.0 - 2 * 0.25) ** 2)
        assert m.chip_thinning_factor == pytest.approx(expected)

    def test_effective_chip_load_larger_with_thinning(self):
        """With partial engagement, effective chip load > nominal chip load."""
        mon = _monitor(dia=10.0)
        m = mon.compute_chip_load(1000.0, 5000.0, ae_mm=2.5)
        assert m.effective_chip_load_mm > m.chip_load_mm

    def test_full_engagement_effective_equals_nominal(self):
        mon = _monitor(dia=10.0)
        m = mon.compute_chip_load(1000.0, 5000.0, ae_mm=10.0)
        assert m.effective_chip_load_mm == pytest.approx(m.chip_load_mm)


# ===================================================================
# MRR calculation
# ===================================================================

class TestMRR:
    def test_mrr_formula(self):
        """MRR = ap * ae * feed_rate (mm^3/min), converted to cm^3/min."""
        mon = _monitor()
        ap, ae, feed = 2.0, 3.0, 600.0
        m = mon.compute_chip_load(feed, 5000.0, ae_mm=ae, ap_mm=ap)
        expected_mm3 = ap * ae * feed
        assert m.chip_volume_rate_mm3_per_min == pytest.approx(expected_mm3)
        assert m.mrr_cm3_per_min == pytest.approx(expected_mm3 / 1000.0)

    def test_mrr_increases_with_feed(self):
        mon = _monitor()
        m1 = mon.compute_chip_load(300.0, 5000.0, ae_mm=3.0, ap_mm=2.0)
        m2 = mon.compute_chip_load(600.0, 5000.0, ae_mm=3.0, ap_mm=2.0)
        assert m2.mrr_cm3_per_min > m1.mrr_cm3_per_min

    def test_mrr_zero_at_zero_feed(self):
        mon = _monitor()
        m = mon.compute_chip_load(0.0, 5000.0, ae_mm=3.0, ap_mm=2.0)
        # zero feed, rpm > 0, chip_load = 0 but rpm valid => chip_load=0
        assert m.chip_volume_rate_mm3_per_min == pytest.approx(0.0)


# ===================================================================
# Optimal range checking
# ===================================================================

class TestOptimalRange:
    def test_in_range_is_optimal(self):
        """Chip load within [0.05, 0.15] for 6061-T6 is optimal."""
        mon = _monitor()
        # feed = 0.10 * rpm * flutes = 0.10 * 5000 * 2 = 1000
        m = mon.compute_chip_load(1000.0, 5000.0)
        assert m.is_optimal is True

    def test_too_low_not_optimal(self):
        mon = _monitor()
        # chip_load = 100 / (5000*2) = 0.01 < 0.05
        m = mon.compute_chip_load(100.0, 5000.0)
        assert m.is_optimal is False
        assert 'below' in m.recommendation.lower()

    def test_too_high_not_optimal(self):
        mon = _monitor()
        # chip_load = 5000 / (5000*2) = 0.5 > 0.15
        m = mon.compute_chip_load(5000.0, 5000.0)
        assert m.is_optimal is False
        assert 'exceeds' in m.recommendation.lower()

    def test_deviation_negative_when_below(self):
        mon = _monitor()
        m = mon.compute_chip_load(100.0, 5000.0)
        assert m.deviation_from_optimal_pct < 0

    def test_deviation_positive_when_above(self):
        mon = _monitor()
        m = mon.compute_chip_load(5000.0, 5000.0)
        assert m.deviation_from_optimal_pct > 0

    def test_deviation_near_zero_at_midpoint(self):
        mon = _monitor()
        mid = (0.05 + 0.15) / 2.0  # 0.10
        feed = mid * 5000 * 2  # 1000
        m = mon.compute_chip_load(feed, 5000.0)
        assert abs(m.deviation_from_optimal_pct) < 1.0


# ===================================================================
# Recutting risk
# ===================================================================

class TestRecuttingRisk:
    def test_low_chip_load_has_risk(self):
        mon = _monitor()
        assert mon.check_recutting_risk(0.01) is True

    def test_optimal_chip_load_no_risk(self):
        mon = _monitor()
        assert mon.check_recutting_risk(0.10) is False

    def test_high_chip_load_no_risk(self):
        mon = _monitor()
        assert mon.check_recutting_risk(0.20) is False

    def test_boundary_no_risk(self):
        """Exactly at the minimum boundary = no recutting."""
        mon = _monitor()
        rng_min, _ = mon._get_range()
        assert mon.check_recutting_risk(rng_min) is False


# ===================================================================
# Optimal feed calculation
# ===================================================================

class TestOptimalFeed:
    def test_optimal_feed_formula(self):
        """feed = mid_chip_load * rpm * flutes."""
        mon = _monitor()
        rpm = 8000.0
        mid = (0.05 + 0.15) / 2.0
        expected = mid * rpm * 2
        assert mon.get_optimal_feed(rpm) == pytest.approx(expected)

    def test_optimal_feed_zero_rpm(self):
        mon = _monitor()
        assert mon.get_optimal_feed(0.0) == 0.0

    def test_optimal_feed_varies_with_material(self):
        mon_al = _monitor(material='6061-T6')
        mon_ti = _monitor(material='Ti-6Al-4V')
        rpm = 5000.0
        assert mon_al.get_optimal_feed(rpm) > mon_ti.get_optimal_feed(rpm)


# ===================================================================
# Different materials
# ===================================================================

class TestMaterialRanges:
    def test_aluminum_range(self):
        mon = _monitor(material='6061-T6')
        rng = mon._get_range()
        assert rng == (0.05, 0.15)

    def test_stainless_range(self):
        mon = _monitor(material='304-SS')
        rng = mon._get_range()
        assert rng == (0.02, 0.08)

    def test_titanium_range(self):
        mon = _monitor(material='Ti-6Al-4V')
        rng = mon._get_range()
        assert rng == (0.01, 0.05)

    def test_7075_range(self):
        mon = _monitor(material='7075-T6')
        rng = mon._get_range()
        assert rng == (0.04, 0.12)

    def test_unknown_material_fallback(self):
        mon = _monitor(material='Inconel-718')
        rng = mon._get_range()
        assert rng == (0.05, 0.15)  # default fallback

    def test_titanium_specific_energy_higher(self):
        mon_al = _monitor(material='6061-T6')
        mon_ti = _monitor(material='Ti-6Al-4V')
        m_al = mon_al.compute_chip_load(1000.0, 5000.0)
        m_ti = mon_ti.compute_chip_load(1000.0, 5000.0)
        assert m_ti.specific_energy_j_per_mm3 > m_al.specific_energy_j_per_mm3


# ===================================================================
# Trend analysis
# ===================================================================

class TestTrendAnalysis:
    def test_empty_history(self):
        mon = _monitor()
        result = mon.analyze_trend([])
        assert result['trend'] == 'stable'
        assert result['stability_score'] == 1.0

    def test_stable_trend(self):
        mon = _monitor()
        history = [ChipLoadMetrics(chip_load_mm=0.10) for _ in range(10)]
        result = mon.analyze_trend(history)
        assert result['trend'] == 'stable'
        assert result['stability_score'] == pytest.approx(1.0)

    def test_increasing_trend(self):
        mon = _monitor()
        history = [ChipLoadMetrics(chip_load_mm=0.05 + i * 0.01) for i in range(10)]
        result = mon.analyze_trend(history)
        assert result['trend'] == 'increasing'

    def test_decreasing_trend(self):
        mon = _monitor()
        history = [ChipLoadMetrics(chip_load_mm=0.15 - i * 0.01) for i in range(10)]
        result = mon.analyze_trend(history)
        assert result['trend'] == 'decreasing'

    def test_avg_chip_load(self):
        mon = _monitor()
        history = [ChipLoadMetrics(chip_load_mm=v) for v in [0.08, 0.10, 0.12]]
        result = mon.analyze_trend(history)
        assert result['avg_chip_load'] == pytest.approx(0.10)

    def test_std_chip_load(self):
        mon = _monitor()
        history = [ChipLoadMetrics(chip_load_mm=v) for v in [0.10, 0.10, 0.10]]
        result = mon.analyze_trend(history)
        assert result['std_chip_load'] == pytest.approx(0.0)

    def test_low_stability_high_variance(self):
        mon = _monitor()
        # Large variance relative to mean
        history = [ChipLoadMetrics(chip_load_mm=v) for v in [0.01, 0.20, 0.01, 0.20]]
        result = mon.analyze_trend(history)
        assert result['stability_score'] < 0.5


# ===================================================================
# Edge cases
# ===================================================================

class TestEdgeCases:
    def test_zero_rpm(self):
        mon = _monitor()
        m = mon.compute_chip_load(500.0, 0.0)
        assert m.chip_load_mm == 0.0
        assert 'Invalid' in m.recommendation

    def test_zero_flutes(self):
        mon = _monitor(flutes=0)
        m = mon.compute_chip_load(500.0, 5000.0)
        assert m.chip_load_mm == 0.0
        assert 'Invalid' in m.recommendation

    def test_zero_feed(self):
        mon = _monitor()
        m = mon.compute_chip_load(0.0, 5000.0)
        assert m.chip_load_mm == pytest.approx(0.0)
        # 0 < 0.05 so below range
        assert m.is_optimal is False

    def test_very_small_radial_engagement(self):
        """Very small ae should still compute without error."""
        mon = _monitor(dia=10.0)
        m = mon.compute_chip_load(1000.0, 5000.0, ae_mm=0.1)
        assert m.chip_thinning_factor > 0
        assert m.effective_chip_load_mm >= m.chip_load_mm


# ===================================================================
# Specific energy
# ===================================================================

class TestSpecificEnergy:
    def test_aluminum_specific_energy(self):
        mon = _monitor(material='6061-T6')
        m = mon.compute_chip_load(1000.0, 5000.0)
        assert m.specific_energy_j_per_mm3 == pytest.approx(0.9)

    def test_stainless_specific_energy(self):
        mon = _monitor(material='304-SS')
        m = mon.compute_chip_load(1000.0, 5000.0)
        assert m.specific_energy_j_per_mm3 == pytest.approx(2.8)

    def test_unknown_material_default_energy(self):
        mon = _monitor(material='Unobtanium')
        m = mon.compute_chip_load(1000.0, 5000.0)
        assert m.specific_energy_j_per_mm3 == pytest.approx(1.0)


# ===================================================================
# CuttingSimProxy.get_chip_load_analysis
# ===================================================================

class TestProxyChipLoadAnalysis:
    def test_returns_chip_load_metrics(self):
        proxy = CuttingSimProxy()
        block = _make_block(feed=500.0, rpm=8000.0)
        result = proxy.get_chip_load_analysis(block)
        assert isinstance(result, ChipLoadMetrics)

    def test_uses_tool_state_flutes(self):
        proxy = CuttingSimProxy()
        block = _make_block(feed=1000.0, rpm=5000.0)
        tool = ToolState(flute_count=4, diameter_mm=10.0)
        result = proxy.get_chip_load_analysis(block, tool_state=tool)
        expected = 1000.0 / (5000.0 * 4)
        assert result.chip_load_mm == pytest.approx(expected)

    def test_default_tool_state(self):
        proxy = CuttingSimProxy()
        block = _make_block(feed=500.0, rpm=8000.0)
        result = proxy.get_chip_load_analysis(block)
        expected = 500.0 / (8000.0 * 2)  # default 2 flutes
        assert result.chip_load_mm == pytest.approx(expected)

    def test_zero_rpm_handled(self):
        proxy = CuttingSimProxy()
        block = _make_block(feed=500.0, rpm=0.0)
        result = proxy.get_chip_load_analysis(block)
        assert result.chip_load_mm == 0.0
