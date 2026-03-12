"""Tests for ToolLibrary and ToolDefinition — cutting tool database for MIRACLE."""
import math
import os
import tempfile

import pytest

from miracle_twin.tool_library import ToolDefinition, ToolLibrary
from miracle_twin.cutting_sim_proxy import CuttingSimProxy, GCodeBlock, ToolState


class TestToolLibraryBuiltins:
    """Verify built-in tool inventory."""

    def test_builtin_tools_loaded(self):
        lib = ToolLibrary()
        tools = lib.list_tools()
        assert len(tools) == 5

    def test_builtin_tool_ids(self):
        lib = ToolLibrary()
        ids = {t.tool_id for t in lib.list_tools()}
        assert ids == {
            'HSS_2F_6mm', 'CARBIDE_4F_10mm', 'CARBIDE_2F_3mm',
            'HSS_4F_12mm', 'CARBIDE_6F_20mm',
        }


class TestToolLookup:
    """Verify tool lookup methods."""

    def test_get_by_id_returns_correct_tool(self):
        lib = ToolLibrary()
        tool = lib.get('CARBIDE_4F_10mm')
        assert tool is not None
        assert tool.name == 'Carbide 4-Flute 10mm End Mill'
        assert tool.diameter_mm == 10.0
        assert tool.flute_count == 4
        assert tool.ktc == 1200.0

    def test_get_returns_none_for_unknown(self):
        lib = ToolLibrary()
        assert lib.get('NONEXISTENT_TOOL') is None

    def test_get_or_default_returns_default_for_unknown(self):
        lib = ToolLibrary()
        tool = lib.get_or_default('NONEXISTENT_TOOL')
        assert tool is not None
        assert tool.tool_id == 'HSS_2F_6mm'

    def test_get_or_default_returns_correct_tool_for_known(self):
        lib = ToolLibrary()
        tool = lib.get_or_default('CARBIDE_2F_3mm')
        assert tool.tool_id == 'CARBIDE_2F_3mm'

    def test_find_by_material_hss(self):
        lib = ToolLibrary()
        hss = lib.find_by_material('HSS')
        assert len(hss) == 2
        assert all(t.tool_material == 'HSS' for t in hss)

    def test_find_by_material_carbide(self):
        lib = ToolLibrary()
        carbide = lib.find_by_material('Carbide')
        assert len(carbide) == 3
        assert all(t.tool_material == 'Carbide' for t in carbide)

    def test_find_by_material_case_insensitive(self):
        lib = ToolLibrary()
        assert len(lib.find_by_material('hss')) == 2
        assert len(lib.find_by_material('CARBIDE')) == 3

    def test_find_by_diameter_exact(self):
        lib = ToolLibrary()
        results = lib.find_by_diameter(10.0, tolerance_mm=0.1)
        assert len(results) == 1
        assert results[0].tool_id == 'CARBIDE_4F_10mm'

    def test_find_by_diameter_with_tolerance(self):
        lib = ToolLibrary()
        results = lib.find_by_diameter(6.0, tolerance_mm=1.0)
        assert any(t.tool_id == 'HSS_2F_6mm' for t in results)

    def test_find_by_diameter_no_match(self):
        lib = ToolLibrary()
        results = lib.find_by_diameter(50.0, tolerance_mm=0.1)
        assert len(results) == 0


class TestToolRegistration:
    """Verify custom tool registration."""

    def test_register_custom_tool(self):
        lib = ToolLibrary()
        custom = ToolDefinition(
            tool_id='CUSTOM_TEST',
            name='Custom Test Tool',
            diameter_mm=8.0,
            flute_count=3,
            ktc=1000.0,
        )
        lib.register(custom)
        assert lib.get('CUSTOM_TEST') is not None
        assert lib.get('CUSTOM_TEST').diameter_mm == 8.0
        assert len(lib.list_tools()) == 6

    def test_register_overwrites_existing(self):
        lib = ToolLibrary()
        replacement = ToolDefinition(
            tool_id='HSS_2F_6mm',
            name='Replaced Tool',
            ktc=999.0,
        )
        lib.register(replacement)
        assert lib.get('HSS_2F_6mm').ktc == 999.0
        assert len(lib.list_tools()) == 5  # No duplicate


class TestToolDefinitionProperties:
    """Verify ToolDefinition computed properties."""

    def test_helix_angle_rad(self):
        td = ToolDefinition(tool_id='T', name='T', helix_angle_deg=30.0)
        assert abs(td.helix_angle_rad - math.radians(30.0)) < 1e-9

    def test_rake_angle_rad(self):
        td = ToolDefinition(tool_id='T', name='T', rake_angle_deg=10.0)
        assert abs(td.rake_angle_rad - math.radians(10.0)) < 1e-9

    def test_helix_angle_rad_zero(self):
        td = ToolDefinition(tool_id='T', name='T', helix_angle_deg=0.0)
        assert td.helix_angle_rad == 0.0

    def test_helix_angle_rad_45(self):
        td = ToolDefinition(tool_id='T', name='T', helix_angle_deg=45.0)
        assert abs(td.helix_angle_rad - math.pi / 4.0) < 1e-9


class TestYAMLLoading:
    """Verify YAML file loading."""

    def test_load_from_yaml(self):
        yaml_content = """\
tools:
  - tool_id: YAML_TOOL_1
    name: YAML Tool 1
    diameter_mm: 5.0
    flute_count: 3
    ktc: 850.0
    krc: 180.0
  - tool_id: YAML_TOOL_2
    name: YAML Tool 2
    diameter_mm: 8.0
    flute_count: 4
"""
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', delete=False
        ) as f:
            f.write(yaml_content)
            yaml_path = f.name

        try:
            lib = ToolLibrary()
            count = lib.load_from_yaml(yaml_path)
            assert count == 2
            assert lib.get('YAML_TOOL_1') is not None
            assert lib.get('YAML_TOOL_1').diameter_mm == 5.0
            assert lib.get('YAML_TOOL_2').flute_count == 4
            assert len(lib.list_tools()) == 7  # 5 builtin + 2 YAML
        finally:
            os.unlink(yaml_path)

    def test_load_from_nonexistent_yaml(self):
        lib = ToolLibrary()
        count = lib.load_from_yaml('/nonexistent/path/tools.yaml')
        assert count == 0
        assert len(lib.list_tools()) == 5  # unchanged


class TestCuttingSimProxyToolIntegration:
    """Verify CuttingSimProxy integration with ToolLibrary."""

    def test_set_tool_updates_coefficients(self):
        proxy = CuttingSimProxy()
        result = proxy.set_tool('CARBIDE_4F_10mm')
        assert result is True
        assert proxy.coeffs.Ktc == 1200.0
        assert proxy.coeffs.Krc == 250.0
        assert proxy.coeffs.Kac == 120.0
        assert proxy.coeffs.Kte == 20.0
        assert proxy.coeffs.Kre == 15.0
        assert proxy.coeffs.Kae == 7.0

    def test_set_tool_updates_taylor_params(self):
        proxy = CuttingSimProxy()
        proxy.set_tool('CARBIDE_4F_10mm')
        assert proxy.TAYLOR_C == 450.0
        assert proxy.TAYLOR_N == 0.25
        assert proxy.VBMAX == 0.30

    def test_set_tool_returns_false_for_unknown(self):
        proxy = CuttingSimProxy()
        result = proxy.set_tool('NONEXISTENT_TOOL')
        assert result is False
        # Coefficients should remain at defaults
        assert proxy.coeffs.Ktc == 796.0

    def test_set_tool_definition_direct(self):
        proxy = CuttingSimProxy()
        td = ToolDefinition(
            tool_id='DIRECT', name='Direct', ktc=999.0, krc=222.0,
        )
        proxy.set_tool_definition(td)
        assert proxy.coeffs.Ktc == 999.0
        assert proxy.coeffs.Krc == 222.0
        assert proxy.active_tool is not None
        assert proxy.active_tool.tool_id == 'DIRECT'

    def test_simulate_with_tool_id_parameter(self):
        proxy = CuttingSimProxy()
        blocks = [
            GCodeBlock(
                feed_rate_mmpm=500.0, spindle_rpm=8000.0,
                axial_depth_mm=1.5, radial_depth_mm=3.175, length_mm=50.0,
            )
        ]
        result = proxy.simulate_program(blocks, tool_id='CARBIDE_4F_10mm')
        assert len(result.block_predictions) == 1
        assert result.block_predictions[0].peak_force_n > 0
        # Confirm the tool was actually loaded
        assert proxy.active_tool is not None
        assert proxy.active_tool.tool_id == 'CARBIDE_4F_10mm'

    def test_different_tools_produce_different_forces(self):
        blocks = [
            GCodeBlock(
                feed_rate_mmpm=500.0, spindle_rpm=8000.0,
                axial_depth_mm=1.5, radial_depth_mm=3.175, length_mm=50.0,
            )
        ]
        proxy_hss = CuttingSimProxy()
        result_hss = proxy_hss.simulate_program(blocks, tool_id='HSS_2F_6mm')

        proxy_carbide = CuttingSimProxy()
        result_carbide = proxy_carbide.simulate_program(
            blocks, tool_id='CARBIDE_4F_10mm'
        )

        # Different coefficients and geometry should yield different forces
        assert (
            result_hss.block_predictions[0].peak_force_n
            != result_carbide.block_predictions[0].peak_force_n
        )

    def test_simulate_with_unknown_tool_id_uses_defaults(self):
        proxy = CuttingSimProxy()
        blocks = [
            GCodeBlock(
                feed_rate_mmpm=500.0, spindle_rpm=8000.0,
                axial_depth_mm=1.5, radial_depth_mm=3.175, length_mm=50.0,
            )
        ]
        # Unknown tool_id — set_tool returns False, defaults remain
        result = proxy.simulate_program(blocks, tool_id='NONEXISTENT')
        assert len(result.block_predictions) == 1
        assert result.block_predictions[0].peak_force_n > 0
        # Coefficients should still be defaults
        assert proxy.coeffs.Ktc == 796.0

    def test_active_tool_property(self):
        proxy = CuttingSimProxy()
        assert proxy.active_tool is None
        proxy.set_tool('HSS_4F_12mm')
        assert proxy.active_tool is not None
        assert proxy.active_tool.tool_id == 'HSS_4F_12mm'
        assert proxy.active_tool.diameter_mm == 12.0

    def test_tool_geometry_applied_in_simulation(self):
        """Verify that tool geometry from the library is used in simulation."""
        proxy = CuttingSimProxy()
        proxy.set_tool('CARBIDE_6F_20mm')
        blocks = [
            GCodeBlock(
                feed_rate_mmpm=500.0, spindle_rpm=5000.0,
                axial_depth_mm=2.0, radial_depth_mm=5.0, length_mm=50.0,
            )
        ]
        result = proxy.simulate_program(blocks)
        # 6-flute 20mm tool should produce substantial forces
        assert result.block_predictions[0].peak_force_n > 0
        assert result.block_predictions[0].power_w > 0
