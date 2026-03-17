"""Tests for CycleTimeEstimator.

Validates machining cycle time estimation including:
- Block-level estimation (cut, rapid, dwell, tool_change)
- Acceleration/deceleration modelling for short moves
- Summary-stat quick estimation
- Estimated vs actual comparison (over/under/exact)
- Time-saving suggestions
- Default and custom machine capabilities
- Edge cases (empty blocks, zero distances, zero feed)
"""

import os
import sys
import time
from types import ModuleType
from typing import Any, Dict, List, Optional, Set
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Ensure the miracle_mes source directory is on sys.path
# ---------------------------------------------------------------------------
_MES_SRC = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir)
)
if _MES_SRC not in sys.path:
    sys.path.insert(0, _MES_SRC)


# ---------------------------------------------------------------------------
# Stub modules (ROS2 / miracle_core / miracle_msgs)
# ---------------------------------------------------------------------------

# Pre-import miracle_mes so it is the real package, not a stub module.
import miracle_mes  # noqa: E402, F401


def _ensure_module(dotted_name: str) -> ModuleType:
    parts = dotted_name.split('.')
    for i in range(1, len(parts) + 1):
        partial = '.'.join(parts[:i])
        if partial not in sys.modules:
            sys.modules[partial] = ModuleType(partial)
    return sys.modules[dotted_name]


# rclpy stubs
_rclpy = _ensure_module('rclpy')
_rclpy_lifecycle = _ensure_module('rclpy.lifecycle')
_rclpy_lifecycle.TransitionCallbackReturn = MagicMock()
_rclpy_lifecycle.TransitionCallbackReturn.SUCCESS = 'SUCCESS'
_rclpy_action = _ensure_module('rclpy.action')
_rclpy_action.ActionServer = MagicMock()
_rclpy_action.CancelResponse = MagicMock()
_rclpy_action.GoalResponse = MagicMock()
_rclpy_action_server = _ensure_module('rclpy.action.server')
_rclpy_action_server.ServerGoalHandle = MagicMock()
_rclpy_cbg = _ensure_module('rclpy.callback_groups')
_rclpy_cbg.ReentrantCallbackGroup = MagicMock()

# miracle_core stubs -- force-set attributes on existing modules
_mc = _ensure_module('miracle_core')
_mc_lifecycle = _ensure_module('miracle_core.lifecycle_node_base')
_mc_qos = _ensure_module('miracle_core.qos_profiles')


class _StubLifecycleNode:
    CRITICALITY_HIGH = 'HIGH'

    def __init__(self, *a, **kw):
        self.service_callback_group = MagicMock()
        self._logger = MagicMock()

    def get_logger(self):
        return self._logger

    def declare_and_validate_parameters(self, spec):
        return {k: v['default'] for k, v in spec.items()}

    def get_machine_ids(self, params):
        return params.get('machine_ids', 'cnc1,cnc2,cnc3').split(',')

    def create_subscription(self, *a, **kw):
        return MagicMock()

    def create_multi_machine_subscriptions(self, *a, **kw):
        return MagicMock()

    def create_publisher(self, *a, **kw):
        return MagicMock()

    def create_service(self, *a, **kw):
        return MagicMock()

    def create_timer(self, *a, **kw):
        return MagicMock()

    def get_parameter(self, name):
        m = MagicMock()
        m.value = 1.0
        return m

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.nanoseconds = int(time.time() * 1e9)
        clock.now.return_value.to_msg.return_value = MagicMock()
        return clock


_mc_lifecycle.MiracleLifecycleNode = _StubLifecycleNode

_mc_qos.QoSProfiles = MagicMock()
_mc_qos.QoSProfiles.alert.return_value = MagicMock()
_mc_qos.QoSProfiles.state_data.return_value = MagicMock()
_mc_qos.QoSProfiles.command.return_value = MagicMock()

# miracle_msgs stubs
_msgs = _ensure_module('miracle_msgs')
_msgs_msg = _ensure_module('miracle_msgs.msg')
_msgs_action = _ensure_module('miracle_msgs.action')
_msgs_srv = _ensure_module('miracle_msgs.srv')


class _StubMsg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattr__(self, name):
        return ''


_msgs_msg.JobStatus = _StubMsg
_msgs_msg.MachineState = _StubMsg
_msgs_msg.TaskAnnouncement = _StubMsg
_msgs_msg.AlarmEscalation = _StubMsg

_exec_job = MagicMock()
_exec_job.Feedback = _StubMsg
_exec_job.Result = _StubMsg
_msgs_action.ExecuteJob = _exec_job

_submit_task = MagicMock()
_submit_task.Request = _StubMsg
_submit_task.Response = _StubMsg
_msgs_srv.SubmitTask = _submit_task


class _StubDigitalThread:
    ENTRY_MATERIAL_LOADED = 'MATERIAL_LOADED'
    ENTRY_PART_COMPLETE = 'PART_COMPLETE'
    ENTRY_TOOL_INSTALLED = 'TOOL_INSTALLED'
    ENTRY_TOOL_REMOVED = 'TOOL_REMOVED'
    ENTRY_JOB_PAUSED = 'JOB_PAUSED'
    ENTRY_JOB_RESUMED = 'JOB_RESUMED'
    ENTRY_JOB_CANCELLED = 'JOB_CANCELLED'
    ENTRY_JOB_FAILED = 'JOB_FAILED'
    ENTRY_MACHINE_ERROR = 'MACHINE_ERROR'

    def __init__(self):
        self._entries = []

    def record_genealogy_event(self, *a, **kw):
        pass

    def set_active_job(self, **kw):
        pass

    def clear_active_job(self, *a):
        pass

    def record_operation_complete(self, *a, **kw):
        pass


_dt_mod = _ensure_module('miracle_mes.digital_thread')
_dt_mod.DigitalThreadNode = _StubDigitalThread

# ---------------------------------------------------------------------------
# Now import the production code
# ---------------------------------------------------------------------------
from miracle_mes.job_scheduler import (  # noqa: E402
    CycleTimeBreakdown,
    CycleTimeEstimator,
    MachineCapability,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _default_cap() -> MachineCapability:
    """Return default machine capability (matches CycleTimeEstimator.DEFAULT_CAPABILITY)."""
    return MachineCapability()


def _simple_program_blocks() -> List[Dict[str, Any]]:
    """A small realistic G-code program: rapid, cut, dwell, tool_change, cut."""
    return [
        {'type': 'rapid', 'distance_mm': 100.0},
        {'type': 'cut', 'distance_mm': 500.0, 'feed_mm_min': 2000.0},
        {'type': 'dwell', 'dwell_sec': 1.0},
        {'type': 'tool_change'},
        {'type': 'cut', 'distance_mm': 300.0, 'feed_mm_min': 3000.0},
    ]


# ===================================================================
# Tests: MachineCapability defaults
# ===================================================================

class TestMachineCapabilityDefaults:
    def test_default_values(self):
        cap = MachineCapability()
        assert cap.max_feed_mm_min == 5000.0
        assert cap.max_rapid_mm_min == 30000.0
        assert cap.tool_change_sec == 5.0
        assert cap.spindle_accel_sec == 2.0
        assert cap.axis_accel_mm_s2 == 2000.0

    def test_custom_values(self):
        cap = MachineCapability(
            max_feed_mm_min=8000.0,
            max_rapid_mm_min=48000.0,
            tool_change_sec=3.0,
            spindle_accel_sec=1.5,
            axis_accel_mm_s2=3000.0,
        )
        assert cap.max_feed_mm_min == 8000.0
        assert cap.tool_change_sec == 3.0


# ===================================================================
# Tests: estimate_from_blocks
# ===================================================================

class TestEstimateFromBlocks:
    def setup_method(self):
        self.est = CycleTimeEstimator()

    def test_empty_blocks_returns_zero(self):
        bd = self.est.estimate_from_blocks([])
        assert bd.total_time_min == 0.0
        assert bd.efficiency_pct == 0.0

    def test_single_cut_block(self):
        blocks = [{'type': 'cut', 'distance_mm': 1000.0, 'feed_mm_min': 2000.0}]
        bd = self.est.estimate_from_blocks(blocks)
        assert bd.cutting_time_min > 0
        assert bd.rapid_time_min == 0.0
        assert bd.tool_change_time_min == 0.0
        assert bd.dwell_time_min == 0.0
        assert bd.total_time_min == bd.cutting_time_min + bd.spindle_time_min
        assert bd.efficiency_pct == 100.0  # only cutting, no non-productive time

    def test_rapid_block(self):
        blocks = [{'type': 'rapid', 'distance_mm': 500.0}]
        bd = self.est.estimate_from_blocks(blocks)
        assert bd.rapid_time_min > 0
        assert bd.cutting_time_min == 0.0
        assert bd.efficiency_pct == 0.0  # no cutting at all

    def test_dwell_block(self):
        blocks = [{'type': 'dwell', 'dwell_sec': 30.0}]
        bd = self.est.estimate_from_blocks(blocks)
        assert bd.dwell_time_min == 0.5
        assert bd.total_time_min == 0.5

    def test_tool_change_adds_spindle_time(self):
        blocks = [{'type': 'tool_change'}]
        cap = _default_cap()
        bd = self.est.estimate_from_blocks(blocks, cap)
        expected_tc_min = cap.tool_change_sec / 60.0
        expected_sp_min = cap.spindle_accel_sec / 60.0
        assert bd.tool_change_time_min == pytest.approx(expected_tc_min, abs=1e-4)
        assert bd.spindle_time_min == pytest.approx(expected_sp_min, abs=1e-4)

    def test_mixed_program(self):
        blocks = _simple_program_blocks()
        bd = self.est.estimate_from_blocks(blocks)
        assert bd.cutting_time_min > 0
        assert bd.rapid_time_min > 0
        assert bd.dwell_time_min > 0
        assert bd.tool_change_time_min > 0
        assert bd.spindle_time_min > 0
        assert bd.total_time_min > 0
        assert 0 < bd.efficiency_pct < 100

    def test_feed_clamped_to_max(self):
        """Feed exceeding machine max should be clamped."""
        cap = MachineCapability(max_feed_mm_min=1000.0)
        blocks = [{'type': 'cut', 'distance_mm': 1000.0, 'feed_mm_min': 9999.0}]
        bd = self.est.estimate_from_blocks(blocks, cap)
        # With clamped feed of 1000 mm/min, cutting time should be >= 1 min
        assert bd.cutting_time_min >= 1.0

    def test_zero_feed_uses_max(self):
        """Zero feed in block should fall back to max_feed."""
        blocks = [{'type': 'cut', 'distance_mm': 5000.0, 'feed_mm_min': 0}]
        bd = self.est.estimate_from_blocks(blocks)
        assert bd.cutting_time_min > 0

    def test_custom_capability(self):
        """Faster machine should yield shorter cycle time."""
        blocks = _simple_program_blocks()
        slow = MachineCapability(max_feed_mm_min=2000.0, max_rapid_mm_min=10000.0)
        fast = MachineCapability(max_feed_mm_min=10000.0, max_rapid_mm_min=60000.0)
        bd_slow = self.est.estimate_from_blocks(blocks, slow)
        bd_fast = self.est.estimate_from_blocks(blocks, fast)
        assert bd_slow.total_time_min > bd_fast.total_time_min

    def test_short_move_acceleration_limited(self):
        """Very short moves should be handled by acceleration-limited path."""
        cap = MachineCapability(axis_accel_mm_s2=500.0, max_feed_mm_min=60000.0)
        blocks = [{'type': 'cut', 'distance_mm': 0.5, 'feed_mm_min': 60000.0}]
        bd = self.est.estimate_from_blocks(blocks, cap)
        # Should still produce a positive time, acceleration-limited
        assert bd.cutting_time_min > 0


# ===================================================================
# Tests: estimate_from_program_stats
# ===================================================================

class TestEstimateFromProgramStats:
    def setup_method(self):
        self.est = CycleTimeEstimator()

    def test_basic_stats(self):
        bd = self.est.estimate_from_program_stats(
            total_cut_distance=5000.0,
            total_rapid_distance=2000.0,
            num_tool_changes=3,
            total_dwell_sec=10.0,
            avg_feed=2500.0,
        )
        assert bd.cutting_time_min == pytest.approx(5000.0 / 2500.0, abs=0.01)
        assert bd.rapid_time_min > 0
        assert bd.tool_change_time_min > 0
        assert bd.dwell_time_min == pytest.approx(10.0 / 60.0, abs=0.01)
        assert bd.total_time_min > 0
        assert bd.efficiency_pct > 0

    def test_zero_distances(self):
        bd = self.est.estimate_from_program_stats(
            total_cut_distance=0.0,
            total_rapid_distance=0.0,
            num_tool_changes=0,
            total_dwell_sec=0.0,
            avg_feed=2000.0,
        )
        assert bd.total_time_min == 0.0
        assert bd.efficiency_pct == 0.0

    def test_avg_feed_clamped(self):
        cap = MachineCapability(max_feed_mm_min=1000.0)
        bd = self.est.estimate_from_program_stats(
            total_cut_distance=5000.0,
            total_rapid_distance=0.0,
            num_tool_changes=0,
            total_dwell_sec=0.0,
            avg_feed=9999.0,
            capability=cap,
        )
        assert bd.cutting_time_min == pytest.approx(5000.0 / 1000.0, abs=0.01)


# ===================================================================
# Tests: compare_estimated_vs_actual
# ===================================================================

class TestCompareEstimatedVsActual:
    def setup_method(self):
        self.est = CycleTimeEstimator()

    def test_perfect_match(self):
        bd = CycleTimeBreakdown(
            cutting_time_min=5.0, rapid_time_min=1.0, tool_change_time_min=0.5,
            dwell_time_min=0.2, spindle_time_min=0.1, total_time_min=6.8,
            efficiency_pct=73.53,
        )
        result = self.est.compare_estimated_vs_actual(bd, 6.8)
        assert result['accuracy_pct'] == 100.0
        assert result['indication'] == 'exact'

    def test_overestimate(self):
        bd = CycleTimeBreakdown(
            cutting_time_min=5.0, rapid_time_min=1.0, tool_change_time_min=0.5,
            dwell_time_min=0.0, spindle_time_min=0.0, total_time_min=10.0,
            efficiency_pct=50.0,
        )
        result = self.est.compare_estimated_vs_actual(bd, 8.0)
        assert result['indication'] == 'over'
        assert result['deviation_min'] > 0
        assert result['accuracy_pct'] < 100.0

    def test_underestimate(self):
        bd = CycleTimeBreakdown(
            cutting_time_min=3.0, rapid_time_min=0.5, tool_change_time_min=0.0,
            dwell_time_min=0.0, spindle_time_min=0.0, total_time_min=3.5,
            efficiency_pct=85.71,
        )
        result = self.est.compare_estimated_vs_actual(bd, 5.0)
        assert result['indication'] == 'under'
        assert result['deviation_min'] < 0
        assert 0 < result['accuracy_pct'] < 100.0

    def test_zero_actual(self):
        bd = CycleTimeBreakdown(
            cutting_time_min=1.0, rapid_time_min=0.0, tool_change_time_min=0.0,
            dwell_time_min=0.0, spindle_time_min=0.0, total_time_min=1.0,
            efficiency_pct=100.0,
        )
        result = self.est.compare_estimated_vs_actual(bd, 0.0)
        assert result['accuracy_pct'] == 0.0
        assert result['indication'] == 'over'


# ===================================================================
# Tests: suggest_time_savings
# ===================================================================

class TestSuggestTimeSavings:
    def setup_method(self):
        self.est = CycleTimeEstimator()

    def test_high_rapid_ratio(self):
        bd = CycleTimeBreakdown(
            cutting_time_min=5.0, rapid_time_min=3.0, tool_change_time_min=0.0,
            dwell_time_min=0.0, spindle_time_min=0.0, total_time_min=8.0,
            efficiency_pct=62.5,
        )
        suggestions = self.est.suggest_time_savings(bd)
        assert any('rapid' in s.lower() for s in suggestions)

    def test_high_tool_change_ratio(self):
        bd = CycleTimeBreakdown(
            cutting_time_min=5.0, rapid_time_min=0.5, tool_change_time_min=2.0,
            dwell_time_min=0.0, spindle_time_min=0.0, total_time_min=7.5,
            efficiency_pct=66.67,
        )
        suggestions = self.est.suggest_time_savings(bd)
        assert any('tool change' in s.lower() for s in suggestions)

    def test_low_efficiency_suggestion(self):
        bd = CycleTimeBreakdown(
            cutting_time_min=2.0, rapid_time_min=1.0, tool_change_time_min=3.0,
            dwell_time_min=1.0, spindle_time_min=1.0, total_time_min=8.0,
            efficiency_pct=25.0,
        )
        suggestions = self.est.suggest_time_savings(bd)
        assert any('efficiency' in s.lower() and 'below 60' in s.lower() for s in suggestions)

    def test_excellent_efficiency_suggestion(self):
        bd = CycleTimeBreakdown(
            cutting_time_min=9.5, rapid_time_min=0.3, tool_change_time_min=0.1,
            dwell_time_min=0.05, spindle_time_min=0.05, total_time_min=10.0,
            efficiency_pct=95.0,
        )
        suggestions = self.est.suggest_time_savings(bd)
        assert any('excellent' in s.lower() for s in suggestions)

    def test_empty_breakdown_no_suggestions(self):
        bd = CycleTimeBreakdown(
            cutting_time_min=0.0, rapid_time_min=0.0, tool_change_time_min=0.0,
            dwell_time_min=0.0, spindle_time_min=0.0, total_time_min=0.0,
            efficiency_pct=0.0,
        )
        suggestions = self.est.suggest_time_savings(bd)
        assert suggestions == []

    def test_high_dwell_suggestion(self):
        bd = CycleTimeBreakdown(
            cutting_time_min=5.0, rapid_time_min=0.5, tool_change_time_min=0.0,
            dwell_time_min=1.0, spindle_time_min=0.0, total_time_min=6.5,
            efficiency_pct=76.92,
        )
        suggestions = self.est.suggest_time_savings(bd)
        assert any('dwell' in s.lower() for s in suggestions)


# ===================================================================
# Tests: Integration / end-to-end
# ===================================================================

class TestCycleTimeIntegration:
    def test_blocks_then_compare(self):
        """Estimate from blocks, then compare against a realistic actual."""
        est = CycleTimeEstimator()
        blocks = _simple_program_blocks()
        bd = est.estimate_from_blocks(blocks)
        # Simulate actual being 10% longer than estimate
        actual = bd.total_time_min * 1.1
        result = est.compare_estimated_vs_actual(bd, actual)
        assert result['indication'] == 'under'
        assert result['accuracy_pct'] > 80.0

    def test_stats_then_suggest(self):
        """Estimate from stats, then get suggestions."""
        est = CycleTimeEstimator()
        bd = est.estimate_from_program_stats(
            total_cut_distance=10000.0,
            total_rapid_distance=8000.0,
            num_tool_changes=8,
            total_dwell_sec=60.0,
            avg_feed=3000.0,
        )
        suggestions = est.suggest_time_savings(bd)
        assert isinstance(suggestions, list)
        # With 8 tool changes and lots of rapid, we should get suggestions
        assert len(suggestions) > 0
