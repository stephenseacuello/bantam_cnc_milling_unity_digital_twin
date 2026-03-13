"""Tests for per-tool calibration data persistence in ToolLibrary."""
import json
import os
import tempfile
import time

import pytest

from miracle_twin.tool_library import ToolCalibrationData, ToolDefinition, ToolLibrary


# ---------------------------------------------------------------------------
# ToolCalibrationData unit tests
# ---------------------------------------------------------------------------


class TestToolCalibrationDataDefaults:
    """Verify ToolCalibrationData creation and defaults."""

    def test_creation_with_defaults(self):
        cal = ToolCalibrationData(tool_id='T1', machine_id='M1')
        assert cal.tool_id == 'T1'
        assert cal.machine_id == 'M1'
        assert cal.force_scale == 1.0
        assert cal.edge_scale == 1.0
        assert cal.thermal_scale == 1.0
        assert cal.wear_rate_scale == 1.0
        assert cal.calibration_count == 0
        assert cal.total_blocks_measured == 0
        assert cal.last_calibrated_timestamp == 0.0
        assert cal.calibration_history == []

    def test_creation_with_custom_values(self):
        cal = ToolCalibrationData(
            tool_id='T2', machine_id='M2',
            force_scale=1.1, edge_scale=0.9,
        )
        assert cal.force_scale == 1.1
        assert cal.edge_scale == 0.9


class TestApplyCalibration:
    """Verify EMA calibration updates."""

    def test_single_apply_updates_force_scale_with_ema(self):
        cal = ToolCalibrationData(tool_id='T1', machine_id='M1')
        # EMA: 1.0 * 0.7 + 1.5 * 0.3 = 0.7 + 0.45 = 1.15
        cal.apply_calibration(force_corr=1.5)
        assert abs(cal.force_scale - 1.15) < 1e-9

    def test_ema_learning_rate_alpha_030(self):
        """Explicitly verify alpha = 0.3."""
        cal = ToolCalibrationData(tool_id='T1', machine_id='M1')
        cal.apply_calibration(force_corr=2.0)
        # 1.0 * 0.7 + 2.0 * 0.3 = 1.3
        assert abs(cal.force_scale - 1.3) < 1e-9

    def test_multiple_calibrations_converge_toward_correction(self):
        cal = ToolCalibrationData(tool_id='T1', machine_id='M1')
        for _ in range(50):
            cal.apply_calibration(force_corr=1.5)
        # After many iterations, should converge close to 1.5
        assert abs(cal.force_scale - 1.5) < 0.01

    def test_calibration_count_increments(self):
        cal = ToolCalibrationData(tool_id='T1', machine_id='M1')
        cal.apply_calibration(force_corr=1.0)
        assert cal.calibration_count == 1
        cal.apply_calibration(force_corr=1.0)
        assert cal.calibration_count == 2
        cal.apply_calibration(force_corr=1.0)
        assert cal.calibration_count == 3

    def test_calibration_history_tracked(self):
        cal = ToolCalibrationData(tool_id='T1', machine_id='M1')
        cal.apply_calibration(force_corr=1.2)
        assert len(cal.calibration_history) == 1
        ts, scales = cal.calibration_history[0]
        assert ts > 0
        assert 'force' in scales
        assert 'edge' in scales
        assert 'thermal' in scales
        assert 'wear' in scales

    def test_history_cap_at_50_entries(self):
        cal = ToolCalibrationData(tool_id='T1', machine_id='M1')
        for i in range(60):
            cal.apply_calibration(force_corr=1.0 + i * 0.01)
        assert len(cal.calibration_history) == 50

    def test_edge_scale_applied_correctly(self):
        cal = ToolCalibrationData(tool_id='T1', machine_id='M1')
        cal.apply_calibration(force_corr=1.0, edge_corr=2.0)
        # edge: 1.0 * 0.7 + 2.0 * 0.3 = 1.3
        assert abs(cal.edge_scale - 1.3) < 1e-9

    def test_thermal_scale_applied_correctly(self):
        cal = ToolCalibrationData(tool_id='T1', machine_id='M1')
        cal.apply_calibration(force_corr=1.0, thermal_corr=0.5)
        # thermal: 1.0 * 0.7 + 0.5 * 0.3 = 0.85
        assert abs(cal.thermal_scale - 0.85) < 1e-9

    def test_wear_rate_scale_applied_correctly(self):
        cal = ToolCalibrationData(tool_id='T1', machine_id='M1')
        cal.apply_calibration(force_corr=1.0, wear_corr=3.0)
        # wear: 1.0 * 0.7 + 3.0 * 0.3 = 1.6
        assert abs(cal.wear_rate_scale - 1.6) < 1e-9

    def test_blocks_measured_accumulates(self):
        cal = ToolCalibrationData(tool_id='T1', machine_id='M1')
        cal.apply_calibration(force_corr=1.0, blocks=10)
        cal.apply_calibration(force_corr=1.0, blocks=25)
        assert cal.total_blocks_measured == 35

    def test_timestamp_updated_on_calibration(self):
        cal = ToolCalibrationData(tool_id='T1', machine_id='M1')
        before = time.time()
        cal.apply_calibration(force_corr=1.0)
        after = time.time()
        assert before <= cal.last_calibrated_timestamp <= after


# ---------------------------------------------------------------------------
# ToolLibrary calibration method tests
# ---------------------------------------------------------------------------


class TestGetCalibratedTool:
    """Verify get_calibrated_tool returns scaled coefficients."""

    def test_returns_scaled_coefficients(self):
        lib = ToolLibrary()
        lib.update_calibration('HSS_2F_6mm', 'MACHINE_A', force_corr=1.5)
        cal_tool = lib.get_calibrated_tool('HSS_2F_6mm', 'MACHINE_A')
        cal = lib._calibration_data[('HSS_2F_6mm', 'MACHINE_A')]
        base = lib.get('HSS_2F_6mm')
        assert abs(cal_tool.ktc - base.ktc * cal.force_scale) < 1e-6
        assert abs(cal_tool.krc - base.krc * cal.force_scale) < 1e-6
        assert abs(cal_tool.kac - base.kac * cal.force_scale) < 1e-6

    def test_returns_original_when_no_calibration(self):
        lib = ToolLibrary()
        tool = lib.get_calibrated_tool('HSS_2F_6mm', 'MACHINE_X')
        base = lib.get('HSS_2F_6mm')
        assert tool.ktc == base.ktc
        assert tool.krc == base.krc
        assert tool.kte == base.kte

    def test_returns_none_for_unknown_tool(self):
        lib = ToolLibrary()
        assert lib.get_calibrated_tool('NONEXISTENT', 'M1') is None

    def test_edge_coefficients_scaled(self):
        lib = ToolLibrary()
        lib.update_calibration('HSS_2F_6mm', 'M1', force_corr=1.0,
                               edge_corr=2.0)
        cal_tool = lib.get_calibrated_tool('HSS_2F_6mm', 'M1')
        cal = lib._calibration_data[('HSS_2F_6mm', 'M1')]
        base = lib.get('HSS_2F_6mm')
        assert abs(cal_tool.kte - base.kte * cal.edge_scale) < 1e-6
        assert abs(cal_tool.kre - base.kre * cal.edge_scale) < 1e-6
        assert abs(cal_tool.kae - base.kae * cal.edge_scale) < 1e-6

    def test_calibrated_tool_is_deep_copy(self):
        """Calibrated tool should not mutate the original."""
        lib = ToolLibrary()
        lib.update_calibration('HSS_2F_6mm', 'M1', force_corr=1.5)
        cal_tool = lib.get_calibrated_tool('HSS_2F_6mm', 'M1')
        cal_tool.ktc = 0.0  # mutate the copy
        base = lib.get('HSS_2F_6mm')
        assert base.ktc == 796.0  # original unchanged


class TestUpdateCalibration:
    """Verify update_calibration creates/updates entries."""

    def test_creates_new_entry(self):
        lib = ToolLibrary()
        lib.update_calibration('HSS_2F_6mm', 'M1', force_corr=1.2)
        assert ('HSS_2F_6mm', 'M1') in lib._calibration_data
        cal = lib._calibration_data[('HSS_2F_6mm', 'M1')]
        assert cal.calibration_count == 1

    def test_updates_existing_entry(self):
        lib = ToolLibrary()
        lib.update_calibration('HSS_2F_6mm', 'M1', force_corr=1.2)
        lib.update_calibration('HSS_2F_6mm', 'M1', force_corr=1.3)
        cal = lib._calibration_data[('HSS_2F_6mm', 'M1')]
        assert cal.calibration_count == 2

    def test_different_machines_independent(self):
        lib = ToolLibrary()
        lib.update_calibration('HSS_2F_6mm', 'M1', force_corr=1.5)
        lib.update_calibration('HSS_2F_6mm', 'M2', force_corr=0.8)
        cal_m1 = lib._calibration_data[('HSS_2F_6mm', 'M1')]
        cal_m2 = lib._calibration_data[('HSS_2F_6mm', 'M2')]
        assert cal_m1.force_scale != cal_m2.force_scale
        # M1: 1.0*0.7 + 1.5*0.3 = 1.15
        assert abs(cal_m1.force_scale - 1.15) < 1e-9
        # M2: 1.0*0.7 + 0.8*0.3 = 0.94
        assert abs(cal_m2.force_scale - 0.94) < 1e-9


class TestSaveLoadCalibrations:
    """Verify save/load round-trip."""

    def test_save_load_round_trip(self):
        lib = ToolLibrary()
        lib.update_calibration('HSS_2F_6mm', 'M1', force_corr=1.3, blocks=10)
        lib.update_calibration('CARBIDE_4F_10mm', 'M2', force_corr=0.9,
                               edge_corr=1.1, blocks=20)

        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False
        ) as f:
            path = f.name

        try:
            lib.save_calibrations(path)

            lib2 = ToolLibrary()
            count = lib2.load_calibrations(path)
            assert count == 2

            cal1 = lib2._calibration_data[('HSS_2F_6mm', 'M1')]
            orig1 = lib._calibration_data[('HSS_2F_6mm', 'M1')]
            assert abs(cal1.force_scale - orig1.force_scale) < 1e-9
            assert cal1.total_blocks_measured == orig1.total_blocks_measured
            assert cal1.calibration_count == orig1.calibration_count

            cal2 = lib2._calibration_data[('CARBIDE_4F_10mm', 'M2')]
            orig2 = lib._calibration_data[('CARBIDE_4F_10mm', 'M2')]
            assert abs(cal2.edge_scale - orig2.edge_scale) < 1e-9
        finally:
            os.unlink(path)

    def test_load_nonexistent_returns_zero(self):
        lib = ToolLibrary()
        count = lib.load_calibrations('/nonexistent/calibrations.json')
        assert count == 0

    def test_save_creates_valid_json(self):
        lib = ToolLibrary()
        lib.update_calibration('HSS_2F_6mm', 'M1', force_corr=1.1)

        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False
        ) as f:
            path = f.name

        try:
            lib.save_calibrations(path)
            with open(path) as f:
                data = json.load(f)
            assert 'calibrations' in data
            assert len(data['calibrations']) == 1
            assert data['calibrations'][0]['tool_id'] == 'HSS_2F_6mm'
        finally:
            os.unlink(path)

    def test_round_trip_preserves_history(self):
        lib = ToolLibrary()
        lib.update_calibration('HSS_2F_6mm', 'M1', force_corr=1.1)
        lib.update_calibration('HSS_2F_6mm', 'M1', force_corr=1.2)

        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False
        ) as f:
            path = f.name

        try:
            lib.save_calibrations(path)
            lib2 = ToolLibrary()
            lib2.load_calibrations(path)
            cal = lib2._calibration_data[('HSS_2F_6mm', 'M1')]
            assert len(cal.calibration_history) == 2
        finally:
            os.unlink(path)


class TestCalibrationSummary:
    """Verify calibration summary output."""

    def test_summary_includes_all_entries(self):
        lib = ToolLibrary()
        lib.update_calibration('HSS_2F_6mm', 'M1', force_corr=1.1)
        lib.update_calibration('HSS_2F_6mm', 'M2', force_corr=0.9)
        lib.update_calibration('CARBIDE_4F_10mm', 'M1', force_corr=1.3)
        summary = lib.get_calibration_summary()
        assert len(summary) == 3
        assert 'HSS_2F_6mm@M1' in summary
        assert 'HSS_2F_6mm@M2' in summary
        assert 'CARBIDE_4F_10mm@M1' in summary

    def test_summary_contains_expected_keys(self):
        lib = ToolLibrary()
        lib.update_calibration('HSS_2F_6mm', 'M1', force_corr=1.1)
        summary = lib.get_calibration_summary()
        entry = summary['HSS_2F_6mm@M1']
        assert 'force_scale' in entry
        assert 'edge_scale' in entry
        assert 'thermal_scale' in entry
        assert 'wear_rate_scale' in entry
        assert 'calibration_count' in entry
        assert 'total_blocks_measured' in entry
        assert 'last_calibrated_timestamp' in entry

    def test_empty_summary(self):
        lib = ToolLibrary()
        summary = lib.get_calibration_summary()
        assert summary == {}
