"""Tests for ToolCompensationManager — CNC tool offset table management."""
import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_twin.tool_library import (
    CompensationEntry,
    CompensationTable,
    ToolCompensationManager,
)


class TestSetOffset:
    """Verify geometry offset creation and update."""

    def test_set_offset_creates_entry(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T01', h_offset=50.0, d_offset=5.0)
        eff = mgr.get_effective_offset('MC-01', 'T01')
        assert eff['h_total'] == 50.0
        assert eff['d_total'] == 5.0

    def test_set_offset_updates_existing(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T01', h_offset=50.0, d_offset=5.0)
        mgr.set_offset('MC-01', 'T01', h_offset=52.0, d_offset=5.1)
        eff = mgr.get_effective_offset('MC-01', 'T01')
        assert eff['h_total'] == 52.0
        assert eff['d_total'] == 5.1

    def test_set_offset_increments_revision(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T01', h_offset=10.0, d_offset=1.0)
        mgr.set_offset('MC-01', 'T02', h_offset=20.0, d_offset=2.0)
        table = mgr._tables['MC-01']
        assert table.revision == 2


class TestApplyWear:
    """Verify incremental wear offset accumulation."""

    def test_apply_wear_increments(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T01', h_offset=100.0, d_offset=5.0)
        mgr.apply_wear('MC-01', 'T01', wear_h=0.02, wear_d=0.005)
        mgr.apply_wear('MC-01', 'T01', wear_h=0.03, wear_d=0.010)
        eff = mgr.get_effective_offset('MC-01', 'T01')
        assert abs(eff['h_total'] - 100.05) < 1e-9
        assert abs(eff['d_total'] - 5.015) < 1e-9

    def test_apply_wear_missing_tool_raises(self):
        mgr = ToolCompensationManager()
        with pytest.raises(KeyError):
            mgr.apply_wear('MC-01', 'T99', wear_h=0.01, wear_d=0.01)


class TestResetWear:
    """Verify wear offset reset leaves geometry intact."""

    def test_reset_wear_zeros_wear(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T01', h_offset=80.0, d_offset=4.0)
        mgr.apply_wear('MC-01', 'T01', wear_h=0.5, wear_d=0.1)
        mgr.reset_wear('MC-01', 'T01')
        eff = mgr.get_effective_offset('MC-01', 'T01')
        assert eff['h_total'] == 80.0
        assert eff['d_total'] == 4.0


class TestValidateOffsets:
    """Verify offset validation detects suspicious values."""

    def test_validate_warns_excessive_length(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T01', h_offset=250.0, d_offset=5.0)
        warnings = mgr.validate_offsets('MC-01')
        length_warnings = [w for w in warnings if w['field'] == 'h_offset']
        assert len(length_warnings) == 1
        assert '250.000' in length_warnings[0]['message']

    def test_validate_warns_negative_radius(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T01', h_offset=50.0, d_offset=-1.0)
        warnings = mgr.validate_offsets('MC-01')
        radius_warnings = [w for w in warnings if w['field'] == 'd_offset']
        assert len(radius_warnings) == 1

    def test_validate_warns_zero_geometry(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T01', h_offset=0.0, d_offset=0.0)
        warnings = mgr.validate_offsets('MC-01')
        zero_warnings = [w for w in warnings if w['field'] == 'geometry']
        assert len(zero_warnings) == 1

    def test_validate_no_table_returns_warning(self):
        mgr = ToolCompensationManager()
        warnings = mgr.validate_offsets('MISSING')
        assert len(warnings) == 1
        assert warnings[0]['field'] == 'table'

    def test_validate_clean_table_returns_empty(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T01', h_offset=50.0, d_offset=3.0)
        warnings = mgr.validate_offsets('MC-01')
        assert warnings == []


class TestExportTable:
    """Verify table export formatting."""

    def test_export_returns_sorted_rows(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T03', h_offset=30.0, d_offset=3.0)
        mgr.set_offset('MC-01', 'T01', h_offset=10.0, d_offset=1.0)
        mgr.set_offset('MC-01', 'T02', h_offset=20.0, d_offset=2.0)
        rows = mgr.export_table('MC-01')
        assert len(rows) == 3
        assert [r['tool_id'] for r in rows] == ['T01', 'T02', 'T03']

    def test_export_includes_totals(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T01', h_offset=100.0, d_offset=5.0)
        mgr.apply_wear('MC-01', 'T01', wear_h=0.1, wear_d=0.02)
        rows = mgr.export_table('MC-01')
        assert abs(rows[0]['h_total'] - 100.1) < 1e-9
        assert abs(rows[0]['d_total'] - 5.02) < 1e-9

    def test_export_missing_machine_raises(self):
        mgr = ToolCompensationManager()
        with pytest.raises(KeyError):
            mgr.export_table('MISSING')


class TestCompareTables:
    """Verify cross-machine table comparison."""

    def test_compare_common_tools(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T01', h_offset=100.0, d_offset=5.0)
        mgr.set_offset('MC-01', 'T02', h_offset=80.0, d_offset=4.0)
        mgr.set_offset('MC-02', 'T01', h_offset=100.5, d_offset=5.0)
        mgr.set_offset('MC-02', 'T03', h_offset=60.0, d_offset=3.0)

        diffs = mgr.compare_tables('MC-01', 'MC-02')
        # Only T01 is common
        assert len(diffs) == 1
        assert diffs[0]['tool_id'] == 'T01'
        assert abs(diffs[0]['h_diff'] - (-0.5)) < 1e-9
        assert diffs[0]['d_diff'] == 0.0

    def test_compare_missing_machine_raises(self):
        mgr = ToolCompensationManager()
        mgr.set_offset('MC-01', 'T01', h_offset=10.0, d_offset=1.0)
        with pytest.raises(KeyError):
            mgr.compare_tables('MC-01', 'MISSING')
