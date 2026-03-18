"""Tests for PowerConsumptionModel, PowerReading, PowerProfile, and PowerSummary."""
import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import math
import pytest

from miracle_twin.cutting_sim_proxy import (
    PowerConsumptionModel,
    PowerProfile,
    PowerReading,
    PowerSummary,
)


# ---------------------------------------------------------------------------
# Helper factories
# ---------------------------------------------------------------------------

def _model(profile=None):
    return PowerConsumptionModel(profile=profile)


def _reading(ts=0.0, spindle=0.0, axis=0.0, coolant=0.0, aux=0.0, total=0.0):
    return PowerReading(
        timestamp=ts,
        spindle_power_kw=spindle,
        axis_power_kw=axis,
        coolant_power_kw=coolant,
        auxiliary_power_kw=aux,
        total_power_kw=total,
    )


# ===================================================================
# PowerReading dataclass
# ===================================================================

class TestPowerReadingDefaults:
    def test_default_total_is_zero(self):
        r = PowerReading()
        assert r.total_power_kw == 0.0

    def test_default_timestamp_is_zero(self):
        r = PowerReading()
        assert r.timestamp == 0.0


# ===================================================================
# PowerProfile dataclass
# ===================================================================

class TestPowerProfileDefaults:
    def test_default_idle_power(self):
        p = PowerProfile()
        assert p.idle_power_kw == 1.5

    def test_default_spindle_constant(self):
        p = PowerProfile()
        assert p.spindle_constant_kw == 2.0

    def test_default_load_factor(self):
        p = PowerProfile()
        assert p.spindle_load_factor == 0.15

    def test_default_axis_power_per_feed(self):
        p = PowerProfile()
        assert p.axis_power_per_feed == 0.0001

    def test_default_coolant_power(self):
        p = PowerProfile()
        assert p.coolant_power_kw == 0.75

    def test_default_auxiliary_power(self):
        p = PowerProfile()
        assert p.auxiliary_power_kw == 0.5


# ===================================================================
# PowerSummary dataclass
# ===================================================================

class TestPowerSummaryDefaults:
    def test_default_total_energy_is_zero(self):
        s = PowerSummary()
        assert s.total_energy_kwh == 0.0

    def test_default_carbon_is_zero(self):
        s = PowerSummary()
        assert s.carbon_kg == 0.0


# ===================================================================
# PowerConsumptionModel — estimate_power
# ===================================================================

class TestEstimatePower:
    def test_idle_machine_has_base_power(self):
        """Spindle off, no feed, no coolant => idle + aux only."""
        m = _model()
        r = m.estimate_power(spindle_rpm=0, spindle_load_pct=0, feed_mm_min=0, coolant_on=False)
        # idle_power_kw(1.5) + aux(0.5) = 2.0
        assert r.total_power_kw == pytest.approx(2.0)
        assert r.spindle_power_kw == 0.0
        assert r.coolant_power_kw == 0.0

    def test_coolant_adds_power(self):
        """Turning coolant on should add coolant_power_kw to total."""
        m = _model()
        r_off = m.estimate_power(spindle_rpm=0, spindle_load_pct=0, feed_mm_min=0, coolant_on=False)
        r_on = m.estimate_power(spindle_rpm=0, spindle_load_pct=0, feed_mm_min=0, coolant_on=True)
        assert r_on.total_power_kw == pytest.approx(r_off.total_power_kw + 0.75)

    def test_spindle_running_adds_constant(self):
        """Spindle at 0% load still draws the constant spindle power."""
        m = _model()
        r = m.estimate_power(spindle_rpm=1000, spindle_load_pct=0, feed_mm_min=0, coolant_on=False)
        # idle(1.5) + spindle_constant(2.0) + aux(0.5) = 4.0
        assert r.spindle_power_kw == pytest.approx(2.0)
        assert r.total_power_kw == pytest.approx(4.0)

    def test_spindle_load_increases_power(self):
        """Higher spindle load percentage should increase spindle power."""
        m = _model()
        r_low = m.estimate_power(spindle_rpm=10000, spindle_load_pct=20, feed_mm_min=0, coolant_on=False)
        r_high = m.estimate_power(spindle_rpm=10000, spindle_load_pct=80, feed_mm_min=0, coolant_on=False)
        assert r_high.spindle_power_kw > r_low.spindle_power_kw

    def test_feed_rate_adds_axis_power(self):
        """Feed rate should contribute axis power."""
        m = _model()
        r = m.estimate_power(spindle_rpm=0, spindle_load_pct=0, feed_mm_min=5000, coolant_on=False)
        # axis_power_per_feed(0.0001) * 5000 = 0.5
        assert r.axis_power_kw == pytest.approx(0.5)

    def test_custom_profile(self):
        """Model should respect a custom PowerProfile."""
        prof = PowerProfile(idle_power_kw=3.0, spindle_constant_kw=5.0,
                            spindle_load_factor=0.2, axis_power_per_feed=0.0002,
                            coolant_power_kw=1.0, auxiliary_power_kw=1.0)
        m = _model(profile=prof)
        r = m.estimate_power(spindle_rpm=0, spindle_load_pct=0, feed_mm_min=0, coolant_on=False)
        # idle(3.0) + aux(1.0)
        assert r.total_power_kw == pytest.approx(4.0)


# ===================================================================
# PowerConsumptionModel — record & summary
# ===================================================================

class TestRecordAndSummary:
    def test_empty_summary_returns_zeros(self):
        m = _model()
        s = m.get_summary()
        assert s.total_energy_kwh == 0.0
        assert s.peak_power_kw == 0.0

    def test_single_reading_returns_zeros(self):
        """Need at least two readings for energy integration."""
        m = _model()
        m.record_reading(_reading(ts=0, total=5.0))
        s = m.get_summary()
        assert s.total_energy_kwh == 0.0

    def test_two_readings_compute_energy(self):
        """Two readings 1 hour apart at constant 3 kW => 3 kWh."""
        m = _model()
        m.record_reading(_reading(ts=0, total=3.0))
        m.record_reading(_reading(ts=3600, total=3.0))
        s = m.get_summary()
        assert s.total_energy_kwh == pytest.approx(3.0)
        assert s.avg_power_kw == pytest.approx(3.0)
        assert s.peak_power_kw == pytest.approx(3.0)

    def test_cost_and_carbon(self):
        m = _model()
        m.record_reading(_reading(ts=0, total=2.0))
        m.record_reading(_reading(ts=3600, total=2.0))  # 1h at 2kW = 2kWh
        s = m.get_summary(electricity_rate_per_kwh=0.20, carbon_factor_kg_per_kwh=0.4)
        assert s.cost_estimate == pytest.approx(0.4)   # 2 * 0.20
        assert s.carbon_kg == pytest.approx(0.8)        # 2 * 0.4

    def test_peak_power_detected(self):
        m = _model()
        m.record_reading(_reading(ts=0, total=1.0))
        m.record_reading(_reading(ts=60, total=10.0))
        m.record_reading(_reading(ts=120, total=3.0))
        s = m.get_summary()
        assert s.peak_power_kw == pytest.approx(10.0)

    def test_idle_vs_cutting_classification(self):
        """Readings with spindle power > 0 classified as cutting."""
        m = _model()
        # Two idle readings (spindle=0)
        m.record_reading(_reading(ts=0, spindle=0.0, total=2.0))
        m.record_reading(_reading(ts=3600, spindle=0.0, total=2.0))
        # Two cutting readings (spindle>0)
        m.record_reading(_reading(ts=3600, spindle=3.0, total=5.0))
        m.record_reading(_reading(ts=7200, spindle=3.0, total=5.0))
        s = m.get_summary()
        assert s.idle_energy_kwh == pytest.approx(2.0)
        assert s.cutting_energy_kwh == pytest.approx(5.0)


# ===================================================================
# PowerConsumptionModel — get_power_profile
# ===================================================================

class TestGetPowerProfile:
    def test_returns_profile(self):
        prof = PowerProfile(idle_power_kw=2.5)
        m = _model(profile=prof)
        assert m.get_power_profile() is prof

    def test_default_profile(self):
        m = _model()
        p = m.get_power_profile()
        assert p.idle_power_kw == 1.5


# ===================================================================
# PowerConsumptionModel — predict_job_energy
# ===================================================================

class TestPredictJobEnergy:
    def test_pure_idle_job(self):
        """A job that is entirely idle."""
        m = _model()
        s = m.predict_job_energy(
            cutting_time_min=0, rapid_time_min=0, idle_time_min=60,
            avg_spindle_load=0, coolant_on=False,
        )
        # idle_power(1.5) + aux(0.5) = 2.0 kW for 1h => 2.0 kWh
        assert s.total_energy_kwh == pytest.approx(2.0)
        assert s.idle_energy_kwh == pytest.approx(2.0)

    def test_cutting_phase_energy(self):
        """Cutting with spindle load and coolant contributes more energy."""
        m = _model()
        s = m.predict_job_energy(
            cutting_time_min=60, rapid_time_min=0, idle_time_min=0,
            avg_spindle_load=50, coolant_on=True,
        )
        # cutting_power = idle(1.5) + spindle_const(2.0) + load(0.15*0.5) + coolant(0.75) + aux(0.5)
        #               = 1.5 + 2.0 + 0.075 + 0.75 + 0.5 = 4.825
        # 1h => 4.825 kWh
        assert s.total_energy_kwh == pytest.approx(4.825)

    def test_mixed_phases(self):
        """Job with cutting, rapid, and idle phases."""
        m = _model()
        s = m.predict_job_energy(
            cutting_time_min=30, rapid_time_min=10, idle_time_min=5,
            avg_spindle_load=60, coolant_on=True,
        )
        assert s.total_energy_kwh > 0
        assert s.peak_power_kw > 0
        assert s.avg_power_kw > 0

    def test_zero_time_job(self):
        m = _model()
        s = m.predict_job_energy(
            cutting_time_min=0, rapid_time_min=0, idle_time_min=0,
            avg_spindle_load=0, coolant_on=False,
        )
        assert s.total_energy_kwh == 0.0
        assert s.avg_power_kw == 0.0
