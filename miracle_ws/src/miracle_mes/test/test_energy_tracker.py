"""Tests for energy consumption tracking and optimization.

Validates EnergyProfile, EnergyTracker, and DigitalThreadNode
energy recording / querying methods.
"""

import sys
import time
from unittest.mock import MagicMock

import pytest

# Mock ROS2 and miracle_core dependencies before importing the module
for mod in ['rclpy', 'rclpy.node', 'rclpy.callback_groups', 'rclpy.qos', 'rclpy.lifecycle',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
            'miracle_core.exceptions',
            'miracle_msgs', 'miracle_msgs.msg', 'miracle_msgs.srv']:
    sys.modules.setdefault(mod, MagicMock())
sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = type('FakeNode', (), {
    'CRITICALITY_HIGH': 'HIGH',
    'CRITICALITY_MEDIUM': 'MEDIUM',
    '__init__': lambda self, *a, **kw: None,
    'get_logger': lambda self: MagicMock(),
    'create_publisher': lambda self, *a, **kw: MagicMock(),
    'create_subscription': lambda self, *a, **kw: MagicMock(),
    'create_timer': lambda self, *a, **kw: MagicMock(),
    'declare_and_validate_parameters': lambda self, specs: {k: MagicMock(value=v['default']) for k, v in specs.items()},
    'get_parameter': lambda self, name: MagicMock(value=0),
})
sys.modules.pop('miracle_mes.digital_thread', None)

from miracle_mes.digital_thread import (  # noqa: E402
    DigitalThreadNode, EnergyProfile, EnergyTracker,
)


# ---- Fixtures -------------------------------------------------------

@pytest.fixture
def tracker():
    """Return a fresh EnergyTracker."""
    return EnergyTracker(grid_emission_factor_kg_per_kwh=0.4)


@pytest.fixture
def dt():
    """Return a fresh DigitalThreadNode."""
    return DigitalThreadNode()


def _populate_tracker(tracker, base_time=1000.0, n=10, interval=60.0,
                      spindle=3.0, axis=0.8, coolant=0.5, auxiliary=0.2):
    """Helper: add *n* evenly spaced power samples to *tracker*."""
    for i in range(n):
        tracker.record_power(
            timestamp=base_time + i * interval,
            spindle_kw=spindle,
            axis_kw=axis,
            coolant_kw=coolant,
            auxiliary_kw=auxiliary,
        )


# =====================================================================
# EnergyProfile dataclass
# =====================================================================

class TestEnergyProfile:
    def test_default_values(self):
        p = EnergyProfile()
        assert p.total_kwh == 0.0
        assert p.carbon_footprint_kg == 0.0

    def test_custom_values(self):
        p = EnergyProfile(total_kwh=10.0, peak_power_kw=7.5, carbon_footprint_kg=4.0)
        assert p.total_kwh == 10.0
        assert p.peak_power_kw == 7.5
        assert p.carbon_footprint_kg == 4.0


# =====================================================================
# Power recording
# =====================================================================

class TestPowerRecording:
    def test_record_single_sample(self, tracker):
        tracker.record_power(1000.0, 3.0, 0.8, 0.5, 0.2)
        assert len(tracker._power_log) == 1
        assert tracker._power_log[0]['total_kw'] == pytest.approx(4.5)

    def test_record_multiple_samples(self, tracker):
        _populate_tracker(tracker, n=5)
        assert len(tracker._power_log) == 5

    def test_total_kw_computed(self, tracker):
        tracker.record_power(0, 1.0, 2.0, 3.0, 4.0)
        assert tracker._power_log[0]['total_kw'] == pytest.approx(10.0)


# =====================================================================
# Energy profile computation
# =====================================================================

class TestComputeEnergyProfile:
    def test_basic_integration(self, tracker):
        # 2 samples 1 hour apart, constant 4.5 kW => 4.5 kWh
        tracker.record_power(0, 3.0, 0.8, 0.5, 0.2)
        tracker.record_power(3600, 3.0, 0.8, 0.5, 0.2)
        profile = tracker.compute_energy_profile(0, 3600)
        assert profile.total_kwh == pytest.approx(4.5, rel=1e-3)

    def test_spindle_kwh(self, tracker):
        tracker.record_power(0, 3.0, 0.0, 0.0, 0.0)
        tracker.record_power(3600, 3.0, 0.0, 0.0, 0.0)
        profile = tracker.compute_energy_profile(0, 3600)
        assert profile.spindle_kwh == pytest.approx(3.0, rel=1e-3)
        assert profile.axis_kwh == pytest.approx(0.0, abs=1e-9)

    def test_axis_kwh(self, tracker):
        tracker.record_power(0, 0.0, 2.0, 0.0, 0.0)
        tracker.record_power(3600, 0.0, 2.0, 0.0, 0.0)
        profile = tracker.compute_energy_profile(0, 3600)
        assert profile.axis_kwh == pytest.approx(2.0, rel=1e-3)

    def test_coolant_kwh(self, tracker):
        tracker.record_power(0, 0.0, 0.0, 1.5, 0.0)
        tracker.record_power(3600, 0.0, 0.0, 1.5, 0.0)
        profile = tracker.compute_energy_profile(0, 3600)
        assert profile.coolant_kwh == pytest.approx(1.5, rel=1e-3)

    def test_auxiliary_kwh(self, tracker):
        tracker.record_power(0, 0.0, 0.0, 0.0, 0.3)
        tracker.record_power(3600, 0.0, 0.0, 0.0, 0.3)
        profile = tracker.compute_energy_profile(0, 3600)
        assert profile.auxiliary_kwh == pytest.approx(0.3, rel=1e-3)

    def test_peak_power(self, tracker):
        tracker.record_power(0, 1.0, 0.0, 0.0, 0.0)
        tracker.record_power(1800, 5.0, 1.0, 0.5, 0.2)
        tracker.record_power(3600, 2.0, 0.0, 0.0, 0.0)
        profile = tracker.compute_energy_profile(0, 3600)
        assert profile.peak_power_kw == pytest.approx(6.7, rel=1e-3)

    def test_avg_power(self, tracker):
        tracker.record_power(0, 3.0, 0.8, 0.5, 0.2)
        tracker.record_power(3600, 3.0, 0.8, 0.5, 0.2)
        profile = tracker.compute_energy_profile(0, 3600)
        assert profile.avg_power_kw == pytest.approx(4.5, rel=1e-3)

    def test_trapezoidal_varying_power(self, tracker):
        # Linearly increasing spindle: 0 -> 4 kW over 1 hour => 2 kWh
        tracker.record_power(0, 0.0, 0.0, 0.0, 0.0)
        tracker.record_power(3600, 4.0, 0.0, 0.0, 0.0)
        profile = tracker.compute_energy_profile(0, 3600)
        assert profile.spindle_kwh == pytest.approx(2.0, rel=1e-3)
        assert profile.total_kwh == pytest.approx(2.0, rel=1e-3)

    def test_window_filtering(self, tracker):
        # Only samples within the window should be used
        tracker.record_power(0, 10.0, 0.0, 0.0, 0.0)  # outside
        tracker.record_power(100, 2.0, 0.0, 0.0, 0.0)
        tracker.record_power(3700, 2.0, 0.0, 0.0, 0.0)
        tracker.record_power(9999, 10.0, 0.0, 0.0, 0.0)  # outside
        profile = tracker.compute_energy_profile(100, 3700)
        assert profile.spindle_kwh == pytest.approx(2.0, rel=1e-3)


# =====================================================================
# Carbon footprint
# =====================================================================

class TestCarbonFootprint:
    def test_carbon_calculation(self, tracker):
        tracker.record_power(0, 3.0, 0.8, 0.5, 0.2)
        tracker.record_power(3600, 3.0, 0.8, 0.5, 0.2)
        profile = tracker.compute_energy_profile(0, 3600)
        expected = 4.5 * 0.4
        assert profile.carbon_footprint_kg == pytest.approx(expected, rel=1e-3)

    def test_custom_emission_factor(self):
        t = EnergyTracker(grid_emission_factor_kg_per_kwh=0.8)
        t.record_power(0, 5.0, 0.0, 0.0, 0.0)
        t.record_power(3600, 5.0, 0.0, 0.0, 0.0)
        profile = t.compute_energy_profile(0, 3600)
        assert profile.carbon_footprint_kg == pytest.approx(5.0 * 0.8, rel=1e-3)


# =====================================================================
# Energy per part
# =====================================================================

class TestEnergyPerPart:
    def test_single_part(self, tracker):
        tracker.record_power(0, 3.0, 0.8, 0.5, 0.2)
        tracker.record_power(3600, 3.0, 0.8, 0.5, 0.2)
        profile = tracker.compute_energy_profile(0, 3600, parts_produced=1)
        assert profile.energy_per_part_kwh == pytest.approx(4.5, rel=1e-3)

    def test_multiple_parts(self, tracker):
        tracker.record_power(0, 3.0, 0.8, 0.5, 0.2)
        tracker.record_power(3600, 3.0, 0.8, 0.5, 0.2)
        profile = tracker.compute_energy_profile(0, 3600, parts_produced=3)
        assert profile.energy_per_part_kwh == pytest.approx(4.5 / 3, rel=1e-3)

    def test_zero_parts_treated_as_one(self, tracker):
        tracker.record_power(0, 3.0, 0.8, 0.5, 0.2)
        tracker.record_power(3600, 3.0, 0.8, 0.5, 0.2)
        profile = tracker.compute_energy_profile(0, 3600, parts_produced=0)
        assert profile.energy_per_part_kwh == pytest.approx(4.5, rel=1e-3)


# =====================================================================
# Specific energy (per cm3)
# =====================================================================

class TestSpecificEnergy:
    def test_specific_energy(self, tracker):
        tracker.record_power(0, 3.0, 0.8, 0.5, 0.2)
        tracker.record_power(3600, 3.0, 0.8, 0.5, 0.2)
        profile = tracker.compute_energy_profile(0, 3600, volume_removed_cm3=10.0)
        # 4.5 kWh = 4500 Wh, / 10 cm3 = 450 Wh/cm3
        assert profile.energy_per_cm3_wh == pytest.approx(450.0, rel=1e-3)

    def test_zero_volume(self, tracker):
        tracker.record_power(0, 3.0, 0.8, 0.5, 0.2)
        tracker.record_power(3600, 3.0, 0.8, 0.5, 0.2)
        profile = tracker.compute_energy_profile(0, 3600, volume_removed_cm3=0)
        assert profile.energy_per_cm3_wh == 0.0


# =====================================================================
# Subsystem breakdown percentages
# =====================================================================

class TestPowerBreakdown:
    def test_breakdown_sums_to_100(self, tracker):
        tracker.record_power(0, 3.0, 0.8, 0.5, 0.2)
        tracker.record_power(3600, 3.0, 0.8, 0.5, 0.2)
        bd = tracker.get_power_breakdown(0, 3600)
        total_pct = bd['spindle_pct'] + bd['axis_pct'] + bd['coolant_pct'] + bd['auxiliary_pct']
        assert total_pct == pytest.approx(100.0, abs=0.1)

    def test_breakdown_proportions(self, tracker):
        tracker.record_power(0, 3.0, 0.8, 0.5, 0.2)
        tracker.record_power(3600, 3.0, 0.8, 0.5, 0.2)
        bd = tracker.get_power_breakdown(0, 3600)
        assert bd['spindle_pct'] == pytest.approx(3.0 / 4.5 * 100, rel=1e-2)

    def test_breakdown_no_data(self, tracker):
        bd = tracker.get_power_breakdown(0, 3600)
        assert bd['spindle_pct'] == 0.0


# =====================================================================
# Energy trend
# =====================================================================

class TestEnergyTrend:
    def test_trend_returns_slots(self, tracker):
        now = time.time()
        # Add samples over 2 hours
        for i in range(5):
            tracker.record_power(now - 7200 + i * 1800, 3.0, 0.8, 0.5, 0.2)
        slots = tracker.get_energy_trend(hours_back=3, slot_hours=1)
        assert len(slots) >= 3
        # Each slot is a (start, kwh) tuple
        for start, kwh in slots:
            assert isinstance(start, float)
            assert isinstance(kwh, float)

    def test_trend_empty_slots(self, tracker):
        slots = tracker.get_energy_trend(hours_back=1, slot_hours=0.5)
        assert len(slots) == 2
        for _, kwh in slots:
            assert kwh == 0.0


# =====================================================================
# Job energy estimation
# =====================================================================

class TestEstimateJobEnergy:
    def test_basic_estimate(self, tracker):
        kwh = tracker.estimate_job_energy(60, 80, spindle_power_kw=5.5)
        # spindle: 5.5 * 0.8 * 1h = 4.4
        # axis: 5.5*0.15*1 = 0.825
        # coolant: 5.5*0.10*1 = 0.55
        # aux: 5.5*0.05*1 = 0.275
        expected = 4.4 + 0.825 + 0.55 + 0.275
        assert kwh == pytest.approx(expected, rel=1e-3)

    def test_zero_duration(self, tracker):
        kwh = tracker.estimate_job_energy(0, 80)
        assert kwh == 0.0

    def test_zero_load(self, tracker):
        kwh = tracker.estimate_job_energy(60, 0, spindle_power_kw=5.5)
        # spindle = 0, but axis/coolant/aux still contribute
        assert kwh > 0


# =====================================================================
# Idle energy waste
# =====================================================================

class TestIdleEnergyWaste:
    def test_all_idle(self, tracker):
        tracker.record_power(0, 0.1, 0.05, 0.0, 0.05)
        tracker.record_power(3600, 0.1, 0.05, 0.0, 0.05)
        idle_kwh, idle_pct, cost = tracker.get_idle_energy_waste(0, 3600)
        assert idle_pct == pytest.approx(100.0, rel=1e-2)
        assert idle_kwh > 0
        assert cost == pytest.approx(idle_kwh * 0.12, rel=1e-3)

    def test_no_idle(self, tracker):
        tracker.record_power(0, 3.0, 0.8, 0.5, 0.2)
        tracker.record_power(3600, 3.0, 0.8, 0.5, 0.2)
        idle_kwh, idle_pct, cost = tracker.get_idle_energy_waste(0, 3600)
        assert idle_kwh == pytest.approx(0.0, abs=1e-9)
        assert idle_pct == pytest.approx(0.0, abs=1e-9)

    def test_partial_idle(self, tracker):
        tracker.record_power(0, 0.1, 0.05, 0.0, 0.05)  # idle
        tracker.record_power(1800, 0.1, 0.05, 0.0, 0.05)  # idle
        tracker.record_power(3600, 3.0, 0.8, 0.5, 0.2)  # active
        idle_kwh, idle_pct, cost = tracker.get_idle_energy_waste(0, 3600)
        assert 0 < idle_pct < 100

    def test_no_samples(self, tracker):
        idle_kwh, idle_pct, cost = tracker.get_idle_energy_waste(0, 3600)
        assert idle_kwh == 0.0
        assert idle_pct == 0.0
        assert cost == 0.0


# =====================================================================
# Program comparison
# =====================================================================

class TestComparePrograms:
    def test_identical_programs(self):
        a = EnergyProfile(total_kwh=5.0, spindle_kwh=3.0, peak_power_kw=7.0,
                          energy_per_part_kwh=5.0, carbon_footprint_kg=2.0)
        b = EnergyProfile(total_kwh=5.0, spindle_kwh=3.0, peak_power_kw=7.0,
                          energy_per_part_kwh=5.0, carbon_footprint_kg=2.0)
        cmp = EnergyTracker.compare_programs(a, b)
        assert cmp['total_kwh_diff_pct'] == pytest.approx(0.0)
        assert cmp['carbon_diff_pct'] == pytest.approx(0.0)

    def test_b_uses_more(self):
        a = EnergyProfile(total_kwh=4.0, spindle_kwh=2.0, peak_power_kw=6.0,
                          energy_per_part_kwh=4.0, carbon_footprint_kg=1.6)
        b = EnergyProfile(total_kwh=6.0, spindle_kwh=4.0, peak_power_kw=9.0,
                          energy_per_part_kwh=6.0, carbon_footprint_kg=2.4)
        cmp = EnergyTracker.compare_programs(a, b)
        assert cmp['total_kwh_diff_pct'] == pytest.approx(50.0)
        assert cmp['spindle_kwh_diff_pct'] == pytest.approx(100.0)

    def test_a_zero_total(self):
        a = EnergyProfile()
        b = EnergyProfile(total_kwh=5.0)
        cmp = EnergyTracker.compare_programs(a, b)
        assert cmp['total_kwh_diff_pct'] == 100.0

    def test_both_zero(self):
        a = EnergyProfile()
        b = EnergyProfile()
        cmp = EnergyTracker.compare_programs(a, b)
        assert cmp['total_kwh_diff_pct'] == 0.0


# =====================================================================
# Edge cases
# =====================================================================

class TestEdgeCases:
    def test_no_samples_returns_empty_profile(self, tracker):
        profile = tracker.compute_energy_profile(0, 3600)
        assert profile.total_kwh == 0.0
        assert profile.peak_power_kw == 0.0

    def test_single_sample_returns_empty_energy(self, tracker):
        tracker.record_power(100, 3.0, 0.8, 0.5, 0.2)
        profile = tracker.compute_energy_profile(0, 3600)
        assert profile.total_kwh == 0.0
        assert profile.peak_power_kw == pytest.approx(4.5)
        assert profile.avg_power_kw == pytest.approx(4.5)

    def test_zero_duration(self, tracker):
        tracker.record_power(100, 3.0, 0.8, 0.5, 0.2)
        tracker.record_power(100, 3.0, 0.8, 0.5, 0.2)
        profile = tracker.compute_energy_profile(100, 100)
        # Both samples at same timestamp -> dt = 0 -> total = 0
        assert profile.total_kwh == pytest.approx(0.0, abs=1e-9)


# =====================================================================
# DigitalThreadNode energy entry recording
# =====================================================================

class TestDigitalThreadEnergyEntries:
    def test_entry_type_constants(self):
        assert DigitalThreadNode.ENERGY_RECORDED == 'energy_recorded'
        assert DigitalThreadNode.ENERGY_OPTIMIZATION == 'energy_optimization'

    def test_record_energy_profile(self, dt):
        profile = EnergyProfile(
            total_kwh=4.5, spindle_kwh=3.0, axis_kwh=0.8,
            coolant_kwh=0.5, auxiliary_kwh=0.2, idle_kwh=0.0,
            peak_power_kw=6.7, avg_power_kw=4.5,
            energy_per_part_kwh=4.5, energy_per_cm3_wh=450.0,
            carbon_footprint_kg=1.8,
        )
        dt.record_energy_profile('cnc1', 'prog_A', profile)
        assert len(dt._entries) == 1
        entry = dt._entries[0]
        assert entry['entry_type'] == 'energy_recorded'
        assert entry['machine_id'] == 'cnc1'
        assert entry['program_id'] == 'prog_A'
        assert entry['total_kwh'] == 4.5
        assert entry['carbon_footprint_kg'] == 1.8

    def test_get_energy_history_all(self, dt):
        for i in range(5):
            p = EnergyProfile(total_kwh=float(i))
            dt.record_energy_profile('cnc1', f'prog_{i}', p)
        history = dt.get_energy_history()
        assert len(history) == 5

    def test_get_energy_history_by_machine(self, dt):
        dt.record_energy_profile('cnc1', 'p1', EnergyProfile(total_kwh=1.0))
        dt.record_energy_profile('cnc2', 'p2', EnergyProfile(total_kwh=2.0))
        dt.record_energy_profile('cnc1', 'p3', EnergyProfile(total_kwh=3.0))
        history = dt.get_energy_history(machine_id='cnc1')
        assert len(history) == 2
        assert all(e['machine_id'] == 'cnc1' for e in history)

    def test_get_energy_history_last_n(self, dt):
        for i in range(10):
            dt.record_energy_profile('cnc1', f'p{i}', EnergyProfile(total_kwh=float(i)))
        history = dt.get_energy_history(last_n=3)
        assert len(history) == 3
        # Should be the last 3
        assert history[-1]['total_kwh'] == 9.0

    def test_energy_entry_has_hash_chain(self, dt):
        dt.record_energy_profile('cnc1', 'p1', EnergyProfile(total_kwh=1.0))
        dt.record_energy_profile('cnc1', 'p2', EnergyProfile(total_kwh=2.0))
        assert 'hash' in dt._entries[0]
        assert 'previous_hash' in dt._entries[1]
        assert dt._entries[1]['previous_hash'] == dt._entries[0]['hash']

    def test_genealogy_integrity_with_energy_entries(self, dt):
        dt.record_energy_profile('cnc1', 'p1', EnergyProfile(total_kwh=1.0))
        dt.record_energy_profile('cnc1', 'p2', EnergyProfile(total_kwh=2.0))
        assert dt.verify_genealogy_integrity()
