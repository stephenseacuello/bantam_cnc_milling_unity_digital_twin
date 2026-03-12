"""Tests for coolant/lubrication model in CuttingSimProxy."""
import pytest
from miracle_twin.cutting_sim_proxy import (
    CoolantConfig,
    CuttingSimProxy,
    GCodeBlock,
    ToolState,
)
from miracle_twin.tool_library import ToolLibrary


def _make_blocks(n=5):
    """Create a list of typical G-code blocks for testing."""
    return [
        GCodeBlock(
            feed_rate_mmpm=500.0,
            spindle_rpm=8000.0,
            axial_depth_mm=1.5,
            radial_depth_mm=3.175,
            length_mm=20.0,
        )
        for _ in range(n)
    ]


class TestCoolantConfigDefaults:
    """Verify CoolantConfig defaults are sensible."""

    def test_default_coolant_type_is_flood(self):
        cfg = CoolantConfig()
        assert cfg.coolant_type == 'flood'

    def test_default_flow_rate(self):
        cfg = CoolantConfig()
        assert cfg.flow_rate_lpm == 10.0

    def test_default_concentration(self):
        cfg = CoolantConfig()
        assert cfg.concentration_pct == 8.0

    def test_default_wear_factor_is_less_than_one(self):
        cfg = CoolantConfig()
        assert cfg.wear_factor < 1.0

    def test_default_thermal_factor_is_less_than_one(self):
        cfg = CoolantConfig()
        assert cfg.thermal_factor < 1.0


class TestCoolantWearFactor:
    """Verify wear factor ordering across coolant types."""

    def test_dry_has_highest_wear_factor(self):
        dry = CoolantConfig(coolant_type='dry')
        for ct in ('mist', 'flood', 'high_pressure', 'cryogenic'):
            other = CoolantConfig(coolant_type=ct)
            assert dry.wear_factor > other.wear_factor, (
                f"dry wear_factor ({dry.wear_factor}) should exceed "
                f"{ct} ({other.wear_factor})"
            )

    def test_dry_wear_factor_is_one(self):
        cfg = CoolantConfig(coolant_type='dry')
        assert cfg.wear_factor == 1.0

    def test_flood_wear_factor_approximately_065(self):
        cfg = CoolantConfig(coolant_type='flood', flow_rate_lpm=0.0)
        assert abs(cfg.wear_factor - 0.65) < 0.01

    def test_cryogenic_produces_lowest_wear_factor(self):
        cryo = CoolantConfig(coolant_type='cryogenic')
        for ct in ('dry', 'mist', 'flood', 'high_pressure'):
            other = CoolantConfig(coolant_type=ct)
            assert cryo.wear_factor <= other.wear_factor

    def test_high_flow_rate_further_reduces_wear(self):
        low_flow = CoolantConfig(coolant_type='flood', flow_rate_lpm=2.0)
        high_flow = CoolantConfig(coolant_type='flood', flow_rate_lpm=15.0)
        assert high_flow.wear_factor < low_flow.wear_factor

    def test_flow_rate_diminishing_returns(self):
        """Flow above 15 lpm should not further reduce wear."""
        at_15 = CoolantConfig(coolant_type='flood', flow_rate_lpm=15.0)
        at_30 = CoolantConfig(coolant_type='flood', flow_rate_lpm=30.0)
        # Both are clamped via min(1.0, flow/15)
        assert abs(at_15.wear_factor - at_30.wear_factor) < 1e-9

    def test_invalid_coolant_type_falls_back_to_dry(self):
        cfg = CoolantConfig(coolant_type='banana_juice')
        assert cfg.wear_factor == 1.0

    def test_wear_factor_never_below_minimum(self):
        cfg = CoolantConfig(coolant_type='cryogenic')
        assert cfg.wear_factor >= 0.3


class TestCoolantThermalFactor:
    """Verify thermal factor ordering."""

    def test_dry_thermal_factor_is_one(self):
        cfg = CoolantConfig(coolant_type='dry')
        assert cfg.thermal_factor == 1.0

    def test_flood_temperature_less_than_dry(self):
        dry = CoolantConfig(coolant_type='dry')
        flood = CoolantConfig(coolant_type='flood')
        assert flood.thermal_factor < dry.thermal_factor

    def test_cryogenic_has_lowest_thermal_factor(self):
        cryo = CoolantConfig(coolant_type='cryogenic')
        for ct in ('dry', 'mist', 'flood', 'high_pressure'):
            other = CoolantConfig(coolant_type=ct)
            assert cryo.thermal_factor <= other.thermal_factor

    def test_invalid_coolant_thermal_falls_back_to_dry(self):
        cfg = CoolantConfig(coolant_type='unknown')
        assert cfg.thermal_factor == 1.0

    def test_thermal_factor_never_below_minimum(self):
        cfg = CoolantConfig(coolant_type='cryogenic')
        assert cfg.thermal_factor >= 0.1


class TestChipEvacuationFactor:
    """Verify chip evacuation factor values."""

    def test_dry_has_highest_chip_evacuation_factor(self):
        cfg = CoolantConfig(coolant_type='dry')
        assert cfg.chip_evacuation_factor == 1.0

    def test_high_pressure_best_chip_evacuation(self):
        hp = CoolantConfig(coolant_type='high_pressure')
        for ct in ('dry', 'mist', 'flood', 'cryogenic'):
            other = CoolantConfig(coolant_type=ct)
            assert hp.chip_evacuation_factor <= other.chip_evacuation_factor

    def test_cryogenic_worse_than_flood_for_chips(self):
        cryo = CoolantConfig(coolant_type='cryogenic')
        flood = CoolantConfig(coolant_type='flood')
        assert cryo.chip_evacuation_factor > flood.chip_evacuation_factor


class TestCoolantWearIntegration:
    """Verify coolant affects wear rate in CuttingSimProxy."""

    def test_dry_cutting_produces_highest_wear(self):
        blocks = _make_blocks(10)
        dry_proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='dry'))
        flood_proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='flood'))

        dry_result = dry_proxy.simulate_program(blocks)
        flood_result = flood_proxy.simulate_program(blocks)

        assert dry_result.final_wear_mm > flood_result.final_wear_mm

    def test_cryogenic_produces_least_wear(self):
        blocks = _make_blocks(10)
        dry_proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='dry'))
        cryo_proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='cryogenic'))

        dry_result = dry_proxy.simulate_program(blocks)
        cryo_result = cryo_proxy.simulate_program(blocks)

        assert cryo_result.final_wear_mm < dry_result.final_wear_mm

    def test_set_coolant_method_changes_wear(self):
        blocks = _make_blocks(10)
        proxy = CuttingSimProxy()

        proxy.set_coolant(CoolantConfig(coolant_type='dry'))
        dry_result = proxy.simulate_program(blocks)

        proxy.set_coolant(CoolantConfig(coolant_type='flood'))
        flood_result = proxy.simulate_program(blocks)

        assert dry_result.final_wear_mm > flood_result.final_wear_mm


class TestCoolantTemperatureIntegration:
    """Verify coolant affects temperature predictions."""

    def test_flood_temperature_lower_than_dry(self):
        blocks = _make_blocks(5)
        dry_proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='dry'))
        flood_proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='flood'))

        dry_result = dry_proxy.simulate_program(blocks)
        flood_result = flood_proxy.simulate_program(blocks)

        dry_temp = dry_result.block_predictions[0].temperature_rise_c
        flood_temp = flood_result.block_predictions[0].temperature_rise_c

        assert flood_temp < dry_temp

    def test_cryogenic_temperature_lowest(self):
        blocks = _make_blocks(5)
        cryo_proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='cryogenic'))
        flood_proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='flood'))

        cryo_result = cryo_proxy.simulate_program(blocks)
        flood_result = flood_proxy.simulate_program(blocks)

        cryo_temp = cryo_result.block_predictions[0].temperature_rise_c
        flood_temp = flood_result.block_predictions[0].temperature_rise_c

        assert cryo_temp < flood_temp


class TestChipRecuttingPenalty:
    """Verify chip recutting penalty increases forces for dry cutting."""

    def test_dry_cutting_has_force_penalty(self):
        blocks = _make_blocks(1)
        dry_proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='dry'))
        flood_proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='flood'))

        dry_result = dry_proxy.simulate_program(blocks)
        flood_result = flood_proxy.simulate_program(blocks)

        # Dry has chip_evacuation_factor=1.0 > 0.7, so force penalty applies
        # Flood has chip_evacuation_factor=0.5 <= 0.7, so no penalty
        dry_peak = dry_result.block_predictions[0].peak_force_n
        flood_peak = flood_result.block_predictions[0].peak_force_n

        # Dry should have higher forces due to chip recutting penalty
        assert dry_peak > flood_peak

    def test_mist_has_chip_recutting_penalty(self):
        """Mist has chip_evacuation_factor=0.9 > 0.7, should have penalty."""
        cfg = CoolantConfig(coolant_type='mist')
        assert cfg.chip_evacuation_factor > 0.7

    def test_flood_no_chip_recutting_penalty(self):
        """Flood has chip_evacuation_factor=0.5 <= 0.7, no penalty."""
        cfg = CoolantConfig(coolant_type='flood')
        assert cfg.chip_evacuation_factor <= 0.7


class TestSimulateProgramCoolantParam:
    """Verify simulate_program accepts coolant parameter."""

    def test_coolant_param_overrides_instance(self):
        blocks = _make_blocks(5)
        proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='dry'))

        # Use flood via parameter override
        flood_cfg = CoolantConfig(coolant_type='flood')
        result = proxy.simulate_program(blocks, coolant=flood_cfg)

        # Also run with dry (instance default)
        dry_result = proxy.simulate_program(blocks)

        # Flood via param should give less wear than dry instance default
        assert result.final_wear_mm < dry_result.final_wear_mm

    def test_coolant_param_restores_original(self):
        """After simulate_program with coolant param, instance coolant is restored."""
        proxy = CuttingSimProxy(coolant=CoolantConfig(coolant_type='dry'))
        blocks = _make_blocks(1)

        proxy.simulate_program(blocks, coolant=CoolantConfig(coolant_type='flood'))

        # Instance coolant should still be dry
        assert proxy.coolant.coolant_type == 'dry'


class TestWhatIfCompareCoolant:
    """Verify what_if_compare supports coolant comparison."""

    def test_flood_vs_dry_shows_force_difference(self):
        proxy = CuttingSimProxy()
        blocks = _make_blocks(5)

        dry_cfg = CoolantConfig(coolant_type='dry')
        flood_cfg = CoolantConfig(coolant_type='flood')

        comparison = proxy.what_if_compare(
            blocks,
            baseline_coolant=flood_cfg,
            override_coolant=dry_cfg,
        )

        # Dry should have higher peak force than flood
        assert comparison.override['peak_force'] > comparison.baseline['peak_force']
        assert comparison.delta['force_pct'] > 0

    def test_flood_vs_dry_shows_wear_difference(self):
        """Flood coolant should produce less final wear than dry."""
        proxy = CuttingSimProxy()
        blocks = _make_blocks(10)

        dry_cfg = CoolantConfig(coolant_type='dry')
        flood_cfg = CoolantConfig(coolant_type='flood')

        dry_result = proxy.simulate_program(blocks, coolant=dry_cfg)
        flood_result = proxy.simulate_program(blocks, coolant=flood_cfg)

        assert flood_result.final_wear_mm < dry_result.final_wear_mm


class TestToolLibraryRecommendedCoolant:
    """Verify tool library has recommended coolant types."""

    def test_all_tools_have_recommended_coolant(self):
        library = ToolLibrary()
        valid_types = {'dry', 'mist', 'flood', 'high_pressure', 'cryogenic'}
        for tool in library.list_tools():
            assert tool.recommended_coolant in valid_types, (
                f"Tool {tool.tool_id} has invalid recommended_coolant: "
                f"{tool.recommended_coolant}"
            )

    def test_hss_tools_recommend_flood(self):
        library = ToolLibrary()
        hss_tools = library.find_by_material('HSS')
        assert len(hss_tools) > 0
        for tool in hss_tools:
            assert tool.recommended_coolant == 'flood', (
                f"HSS tool {tool.tool_id} should recommend flood, "
                f"got {tool.recommended_coolant}"
            )

    def test_dlc_coated_tool_recommends_mist(self):
        library = ToolLibrary()
        tool = library.get('CARBIDE_2F_3mm')
        assert tool is not None
        assert tool.coating == 'DLC'
        assert tool.recommended_coolant == 'mist'

    def test_default_recommended_coolant_is_flood(self):
        """ToolDefinition default recommended_coolant is flood."""
        from miracle_twin.tool_library import ToolDefinition
        td = ToolDefinition(tool_id='test', name='test')
        assert td.recommended_coolant == 'flood'
