"""Tests for multi-zone ThermalModel and ThermalZone in CuttingSimProxy."""
import sys
from unittest.mock import MagicMock

# Mock ROS2 and related modules before importing our code
for mod in (
    'rclpy', 'rclpy.lifecycle', 'rclpy.node', 'rclpy.qos',
    'rclpy.parameter', 'rclpy.callback_groups', 'rclpy.executors',
    'std_msgs', 'std_msgs.msg',
):
    sys.modules.setdefault(mod, MagicMock())

# Mock miracle_core and miracle_msgs submodules (not top-level)
for mod in (
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
import pytest

from miracle_twin.cutting_sim_proxy import (
    CoolantConfig,
    CuttingSimProxy,
    GCodeBlock,
    ThermalModel,
    ThermalZone,
)


# ---------------------------------------------------------------------------
# ThermalZone dataclass
# ---------------------------------------------------------------------------

class TestThermalZoneDataclass:
    """Verify ThermalZone fields and defaults."""

    def test_default_temperature_is_25(self):
        z = ThermalZone(zone_id='test')
        assert z.temperature_c == 25.0

    def test_custom_fields(self):
        z = ThermalZone(
            zone_id='spindle',
            temperature_c=30.0,
            thermal_mass_j_per_c=5000.0,
            heat_input_w=100.0,
            heat_dissipation_w_per_c=5.0,
            max_safe_temp_c=80.0,
            adjacent_zones=['tool_holder', 'ambient'],
        )
        assert z.zone_id == 'spindle'
        assert z.temperature_c == 30.0
        assert z.thermal_mass_j_per_c == 5000.0
        assert z.heat_input_w == 100.0
        assert z.heat_dissipation_w_per_c == 5.0
        assert z.max_safe_temp_c == 80.0
        assert z.adjacent_zones == ['tool_holder', 'ambient']

    def test_default_adjacent_zones_is_empty_list(self):
        z = ThermalZone(zone_id='test')
        assert z.adjacent_zones == []


# ---------------------------------------------------------------------------
# ThermalModel — initialization
# ---------------------------------------------------------------------------

class TestThermalModelInit:
    """Verify initial state of ThermalModel."""

    def test_all_zones_at_ambient(self):
        model = ThermalModel()
        state = model.get_thermal_state()
        for zid, temp in state.items():
            assert temp == pytest.approx(25.0), f"{zid} should be at ambient"

    def test_has_five_zones(self):
        model = ThermalModel()
        state = model.get_thermal_state()
        assert len(state) == 5

    def test_zone_ids(self):
        model = ThermalModel()
        state = model.get_thermal_state()
        expected = {'spindle', 'workpiece', 'tool_holder', 'coolant_reservoir', 'ambient'}
        assert set(state.keys()) == expected

    def test_spindle_max_temp_is_80(self):
        model = ThermalModel()
        assert model._zones['spindle'].max_safe_temp_c == 80.0

    def test_workpiece_thermal_mass_is_2000(self):
        model = ThermalModel()
        assert model._zones['workpiece'].thermal_mass_j_per_c == 2000.0

    def test_coolant_reservoir_thermal_mass_is_20000(self):
        model = ThermalModel()
        assert model._zones['coolant_reservoir'].thermal_mass_j_per_c == 20000.0

    def test_tool_holder_max_temp_is_150(self):
        model = ThermalModel()
        assert model._zones['tool_holder'].max_safe_temp_c == 150.0


# ---------------------------------------------------------------------------
# ThermalModel — heat input raises temperature
# ---------------------------------------------------------------------------

class TestHeatInput:
    """Verify that applying cutting power raises zone temperatures."""

    def test_cutting_power_raises_workpiece_temp(self):
        model = ThermalModel()
        model.update(1.0, cutting_power_w=1000.0, coolant_type='dry')
        state = model.get_thermal_state()
        assert state['workpiece'] > 25.0

    def test_cutting_power_raises_tool_holder_temp(self):
        model = ThermalModel()
        model.update(1.0, cutting_power_w=1000.0, coolant_type='dry')
        state = model.get_thermal_state()
        assert state['tool_holder'] > 25.0

    def test_cutting_power_raises_spindle_temp(self):
        model = ThermalModel()
        model.update(1.0, cutting_power_w=1000.0, coolant_type='dry')
        state = model.get_thermal_state()
        assert state['spindle'] > 25.0

    def test_higher_power_means_higher_temp(self):
        m1 = ThermalModel()
        m2 = ThermalModel()
        m1.update(1.0, cutting_power_w=500.0, coolant_type='dry')
        m2.update(1.0, cutting_power_w=2000.0, coolant_type='dry')
        assert m2.get_thermal_state()['workpiece'] > m1.get_thermal_state()['workpiece']

    def test_workpiece_gets_largest_share(self):
        """Workpiece receives 60% of cutting power — should heat fastest."""
        model = ThermalModel()
        model.update(1.0, cutting_power_w=5000.0, coolant_type='dry')
        state = model.get_thermal_state()
        # Workpiece rise should be significant given 60% partition
        wp_rise = state['workpiece'] - 25.0
        sp_rise = state['spindle'] - 25.0
        assert wp_rise > sp_rise

    def test_spindle_power_adds_to_spindle_zone(self):
        m1 = ThermalModel()
        m2 = ThermalModel()
        m1.update(1.0, cutting_power_w=1000.0, spindle_power_w=0.0, coolant_type='dry')
        m2.update(1.0, cutting_power_w=1000.0, spindle_power_w=500.0, coolant_type='dry')
        assert m2.get_thermal_state()['spindle'] > m1.get_thermal_state()['spindle']


# ---------------------------------------------------------------------------
# ThermalModel — dissipation / cooling
# ---------------------------------------------------------------------------

class TestDissipation:
    """Verify that zones cool toward ambient when power is removed."""

    def test_heated_zone_cools_with_zero_power(self):
        model = ThermalModel()
        # Heat up
        for _ in range(10):
            model.update(1.0, cutting_power_w=5000.0, coolant_type='dry')
        hot_temp = model.get_thermal_state()['workpiece']
        assert hot_temp > 25.0
        # Now cool
        for _ in range(50):
            model.update(1.0, cutting_power_w=0.0, coolant_type='dry')
        cooled_temp = model.get_thermal_state()['workpiece']
        assert cooled_temp < hot_temp

    def test_zero_power_cools_toward_ambient(self):
        model = ThermalModel()
        # Heat up first
        for _ in range(20):
            model.update(1.0, cutting_power_w=3000.0, coolant_type='dry')
        # Cool down
        for _ in range(2000):
            model.update(1.0, cutting_power_w=0.0, coolant_type='dry')
        state = model.get_thermal_state()
        for zid in ('spindle', 'workpiece', 'tool_holder', 'coolant_reservoir'):
            assert abs(state[zid] - 25.0) < 2.0, f"{zid} should be near ambient"

    def test_zero_dt_does_not_change_state(self):
        model = ThermalModel()
        model.update(1.0, cutting_power_w=1000.0, coolant_type='dry')
        state_before = model.get_thermal_state()
        model.update(0.0, cutting_power_w=5000.0, coolant_type='dry')
        state_after = model.get_thermal_state()
        for zid in state_before:
            assert state_before[zid] == pytest.approx(state_after[zid])


# ---------------------------------------------------------------------------
# ThermalModel — zone coupling / conduction
# ---------------------------------------------------------------------------

class TestZoneCoupling:
    """Verify inter-zone heat transfer between adjacent zones."""

    def test_tool_holder_heats_adjacent_spindle(self):
        """When tool_holder is hot and spindle is cool, heat should flow."""
        model = ThermalModel()
        # Manually heat tool_holder
        model._zones['tool_holder'].temperature_c = 100.0
        model._zones['spindle'].temperature_c = 25.0
        model.update(1.0, cutting_power_w=0.0, coolant_type='dry')
        # Spindle should warm (it's adjacent to tool_holder)
        assert model._zones['spindle'].temperature_c > 25.0

    def test_non_adjacent_zones_no_direct_conduction(self):
        """Spindle and coolant_reservoir are not adjacent — no direct transfer."""
        model = ThermalModel()
        # Spindle adjacency list: ['tool_holder', 'ambient']
        assert 'coolant_reservoir' not in model._zones['spindle'].adjacent_zones


# ---------------------------------------------------------------------------
# ThermalModel — coolant effect
# ---------------------------------------------------------------------------

class TestCoolantEffect:
    """Verify coolant type affects dissipation rates."""

    def test_flood_cools_better_than_dry(self):
        m_dry = ThermalModel()
        m_flood = ThermalModel()
        for _ in range(10):
            m_dry.update(1.0, cutting_power_w=2000.0, coolant_type='dry')
            m_flood.update(1.0, cutting_power_w=2000.0, coolant_type='flood')
        assert m_flood.get_thermal_state()['workpiece'] < m_dry.get_thermal_state()['workpiece']

    def test_cryogenic_cools_better_than_flood(self):
        m_flood = ThermalModel()
        m_cryo = ThermalModel()
        for _ in range(10):
            m_flood.update(1.0, cutting_power_w=2000.0, coolant_type='flood')
            m_cryo.update(1.0, cutting_power_w=2000.0, coolant_type='cryogenic')
        assert m_cryo.get_thermal_state()['workpiece'] < m_flood.get_thermal_state()['workpiece']

    def test_high_pressure_better_than_mist(self):
        m_mist = ThermalModel()
        m_hp = ThermalModel()
        for _ in range(10):
            m_mist.update(1.0, cutting_power_w=2000.0, coolant_type='mist')
            m_hp.update(1.0, cutting_power_w=2000.0, coolant_type='high_pressure')
        assert m_hp.get_thermal_state()['workpiece'] < m_mist.get_thermal_state()['workpiece']

    def test_unknown_coolant_defaults_to_dry_multiplier(self):
        m_dry = ThermalModel()
        m_unk = ThermalModel()
        for _ in range(5):
            m_dry.update(1.0, cutting_power_w=2000.0, coolant_type='dry')
            m_unk.update(1.0, cutting_power_w=2000.0, coolant_type='nonexistent')
        assert m_unk.get_thermal_state()['workpiece'] == pytest.approx(
            m_dry.get_thermal_state()['workpiece'], rel=1e-6
        )


# ---------------------------------------------------------------------------
# ThermalModel — warnings
# ---------------------------------------------------------------------------

class TestThermalWarnings:
    """Verify thermal warning generation."""

    def test_no_warnings_at_ambient(self):
        model = ThermalModel()
        assert model.get_thermal_warnings() == []

    def test_warning_when_above_80pct_of_limit(self):
        model = ThermalModel()
        # Spindle max is 80C, ambient is 25C, so range = 55C.
        # 80% of range = 44C above ambient = 69C.
        model._zones['spindle'].temperature_c = 70.0
        warnings = model.get_thermal_warnings()
        spindle_warnings = [w for w in warnings if w[0] == 'spindle']
        assert len(spindle_warnings) == 1
        assert spindle_warnings[0][1] == 70.0
        assert spindle_warnings[0][2] == 80.0
        assert spindle_warnings[0][3] >= 80.0

    def test_no_warning_below_80pct(self):
        model = ThermalModel()
        # Spindle: 80% of (80-25)=55 is 44, so 25+44=69.
        # Set to 60C => rise=35, pct=35/55*100=63.6%
        model._zones['spindle'].temperature_c = 60.0
        warnings = model.get_thermal_warnings()
        spindle_warnings = [w for w in warnings if w[0] == 'spindle']
        assert len(spindle_warnings) == 0

    def test_multiple_warnings_possible(self):
        model = ThermalModel()
        model._zones['spindle'].temperature_c = 78.0
        model._zones['tool_holder'].temperature_c = 148.0
        warnings = model.get_thermal_warnings()
        zone_ids = {w[0] for w in warnings}
        assert 'spindle' in zone_ids
        assert 'tool_holder' in zone_ids


# ---------------------------------------------------------------------------
# ThermalModel — trajectory prediction
# ---------------------------------------------------------------------------

class TestTrajectoryPrediction:
    """Verify thermal trajectory prediction."""

    def test_trajectory_returns_correct_number_of_steps(self):
        model = ThermalModel()
        traj = model.predict_thermal_trajectory(1000.0, 10.0, steps=5)
        assert len(traj) == 5

    def test_trajectory_shows_rising_temps_with_power(self):
        model = ThermalModel()
        traj = model.predict_thermal_trajectory(5000.0, 10.0, steps=5, coolant_type='dry')
        # Each step should show workpiece getting hotter
        for i in range(1, len(traj)):
            assert traj[i]['workpiece'] >= traj[i - 1]['workpiece']

    def test_trajectory_does_not_modify_model_state(self):
        model = ThermalModel()
        state_before = model.get_thermal_state()
        model.predict_thermal_trajectory(5000.0, 60.0, steps=20)
        state_after = model.get_thermal_state()
        for zid in state_before:
            assert state_before[zid] == pytest.approx(state_after[zid])


# ---------------------------------------------------------------------------
# ThermalModel — time to limit
# ---------------------------------------------------------------------------

class TestTimeToLimit:
    """Verify time-to-limit estimation."""

    def test_returns_none_for_unknown_zone(self):
        model = ThermalModel()
        assert model.get_time_to_limit('nonexistent') is None

    def test_returns_zero_when_at_limit(self):
        model = ThermalModel()
        model._zones['spindle'].temperature_c = 80.0
        assert model.get_time_to_limit('spindle') == 0.0

    def test_returns_none_when_cooling(self):
        model = ThermalModel()
        # No heat input, zone at ambient — no net heating
        model._zones['spindle'].heat_input_w = 0.0
        result = model.get_time_to_limit('spindle')
        assert result is None

    def test_returns_positive_when_heating(self):
        model = ThermalModel()
        model._zones['spindle'].heat_input_w = 500.0
        ttl = model.get_time_to_limit('spindle')
        assert ttl is not None
        assert ttl > 0

    def test_higher_heat_means_less_time(self):
        m1 = ThermalModel()
        m2 = ThermalModel()
        m1._zones['spindle'].heat_input_w = 100.0
        m2._zones['spindle'].heat_input_w = 1000.0
        ttl1 = m1.get_time_to_limit('spindle')
        ttl2 = m2.get_time_to_limit('spindle')
        assert ttl1 is not None and ttl2 is not None
        assert ttl2 < ttl1


# ---------------------------------------------------------------------------
# ThermalModel — reset
# ---------------------------------------------------------------------------

class TestReset:
    """Verify reset brings everything back to ambient."""

    def test_reset_restores_ambient(self):
        model = ThermalModel()
        for _ in range(20):
            model.update(1.0, cutting_power_w=5000.0, coolant_type='dry')
        # Confirm not at ambient
        state = model.get_thermal_state()
        assert state['workpiece'] > 26.0
        # Reset
        model.reset()
        state = model.get_thermal_state()
        for zid, temp in state.items():
            assert temp == pytest.approx(25.0), f"{zid} not reset"

    def test_reset_clears_heat_input(self):
        model = ThermalModel()
        model.update(1.0, cutting_power_w=5000.0, coolant_type='dry')
        model.reset()
        for zid, zone in model._zones.items():
            assert zone.heat_input_w == 0.0


# ---------------------------------------------------------------------------
# ThermalModel — steady state convergence
# ---------------------------------------------------------------------------

class TestSteadyState:
    """Verify model converges to a steady state with constant power."""

    def test_temperatures_converge(self):
        model = ThermalModel()
        prev_state = model.get_thermal_state()
        for _ in range(2000):
            model.update(1.0, cutting_power_w=500.0, coolant_type='flood')
        state1 = model.get_thermal_state()
        for _ in range(500):
            model.update(1.0, cutting_power_w=500.0, coolant_type='flood')
        state2 = model.get_thermal_state()
        # After long run, temps should barely change
        for zid in ('spindle', 'workpiece', 'tool_holder'):
            assert abs(state2[zid] - state1[zid]) < 0.5


# ---------------------------------------------------------------------------
# CuttingSimProxy integration with ThermalModel
# ---------------------------------------------------------------------------

class TestCuttingSimProxyThermal:
    """Verify CuttingSimProxy has thermal model and update_thermal_state."""

    def test_proxy_has_thermal_model(self):
        proxy = CuttingSimProxy()
        assert isinstance(proxy._thermal_model, ThermalModel)

    def test_update_thermal_state_with_active_block(self):
        proxy = CuttingSimProxy()
        block = GCodeBlock(
            feed_rate_mmpm=500.0,
            spindle_rpm=8000.0,
            axial_depth_mm=1.5,
            radial_depth_mm=3.175,
            length_mm=20.0,
        )
        state = proxy.update_thermal_state(block, dt_sec=1.0)
        assert 'workpiece' in state
        assert state['workpiece'] > 25.0

    def test_update_thermal_state_idle_block(self):
        proxy = CuttingSimProxy()
        block = GCodeBlock(feed_rate_mmpm=0.0, spindle_rpm=0.0)
        state = proxy.update_thermal_state(block, dt_sec=1.0)
        # No power — should remain at ambient
        assert state['workpiece'] == pytest.approx(25.0)

    def test_coolant_affects_proxy_thermal(self):
        proxy_dry = CuttingSimProxy(coolant=CoolantConfig(coolant_type='dry'))
        proxy_flood = CuttingSimProxy(coolant=CoolantConfig(coolant_type='flood'))
        block = GCodeBlock(
            feed_rate_mmpm=500.0,
            spindle_rpm=8000.0,
            axial_depth_mm=1.5,
            radial_depth_mm=3.175,
        )
        for _ in range(10):
            proxy_dry.update_thermal_state(block, dt_sec=1.0)
            proxy_flood.update_thermal_state(block, dt_sec=1.0)
        assert proxy_flood._thermal_model.get_thermal_state()['workpiece'] < \
               proxy_dry._thermal_model.get_thermal_state()['workpiece']
