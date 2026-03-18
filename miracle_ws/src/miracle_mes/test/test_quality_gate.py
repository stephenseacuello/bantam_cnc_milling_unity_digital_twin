"""Tests for QualityGateChecker, QualityGate, QualityCheck, and related dataclasses.

Validates quality gate management including:
- Gate and check registration
- Measurement inspection and pass/fail logic
- Critical check enforcement
- Partial-pass threshold support
- Part inspection history tracking
- Gate-level aggregate statistics
- Stage gating (can_proceed) logic
- Edge cases (missing measurements, empty gates, unknown gates)
"""

import os
import sys
import math
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
    QualityCheck,
    QualityGate,
    QualityGateChecker,
    GateResult,
    InspectionResult,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _diameter_check(check_id: str = 'chk_dia', nominal: float = 25.0,
                    tol_plus: float = 0.05, tol_minus: float = 0.05,
                    is_critical: bool = False) -> QualityCheck:
    return QualityCheck(
        check_id=check_id,
        parameter='diameter',
        nominal=nominal,
        tolerance_plus=tol_plus,
        tolerance_minus=tol_minus,
        unit='mm',
        is_critical=is_critical,
    )


def _surface_check(check_id: str = 'chk_ra', nominal: float = 1.6,
                    tol_plus: float = 0.4, tol_minus: float = 0.4,
                    is_critical: bool = False) -> QualityCheck:
    return QualityCheck(
        check_id=check_id,
        parameter='surface_roughness',
        nominal=nominal,
        tolerance_plus=tol_plus,
        tolerance_minus=tol_minus,
        unit='um',
        is_critical=is_critical,
    )


def _make_gate(gate_id: str = 'gate1', name: str = 'Roughing QC',
               stage: str = 'roughing', checks: Optional[List[QualityCheck]] = None,
               is_mandatory: bool = True,
               pass_threshold_pct: float = 100.0) -> QualityGate:
    return QualityGate(
        gate_id=gate_id,
        name=name,
        stage=stage,
        checks=checks if checks is not None else [_diameter_check(), _surface_check()],
        is_mandatory=is_mandatory,
        pass_threshold_pct=pass_threshold_pct,
    )


def _checker_with_gate(**gate_kw) -> QualityGateChecker:
    """Return a QualityGateChecker with one gate pre-registered."""
    checker = QualityGateChecker()
    checker.add_gate(_make_gate(**gate_kw))
    return checker


# ===================================================================
# Tests: Dataclass construction
# ===================================================================

class TestDataclasses:
    def test_quality_check_fields(self):
        qc = _diameter_check()
        assert qc.check_id == 'chk_dia'
        assert qc.parameter == 'diameter'
        assert qc.nominal == 25.0
        assert qc.tolerance_plus == 0.05
        assert qc.tolerance_minus == 0.05
        assert qc.unit == 'mm'
        assert qc.is_critical is False

    def test_quality_gate_defaults(self):
        gate = QualityGate(gate_id='g1', name='Gate 1', stage='roughing')
        assert gate.checks == []
        assert gate.is_mandatory is True
        assert gate.pass_threshold_pct == 100.0

    def test_gate_result_defaults(self):
        gr = GateResult(gate_id='g1', part_id='p1')
        assert gr.results == []
        assert gr.overall_passed is False
        assert gr.pass_rate_pct == 0.0

    def test_inspection_result_fields(self):
        ir = InspectionResult(
            check_id='chk1', measured_value=25.02, deviation=0.02,
            passed=True, timestamp=1000.0, inspector='op1',
        )
        assert ir.passed is True
        assert ir.deviation == 0.02


# ===================================================================
# Tests: Gate management
# ===================================================================

class TestGateManagement:
    def test_add_and_get_gate(self):
        checker = QualityGateChecker()
        gate = _make_gate()
        checker.add_gate(gate)
        assert checker.get_gate('gate1') is gate

    def test_get_unknown_gate_returns_none(self):
        checker = QualityGateChecker()
        assert checker.get_gate('nonexistent') is None

    def test_get_all_gates(self):
        checker = QualityGateChecker()
        g1 = _make_gate(gate_id='g1')
        g2 = _make_gate(gate_id='g2')
        checker.add_gate(g1)
        checker.add_gate(g2)
        all_gates = checker.get_all_gates()
        assert len(all_gates) == 2
        ids = {g.gate_id for g in all_gates}
        assert ids == {'g1', 'g2'}


# ===================================================================
# Tests: Inspection logic
# ===================================================================

class TestInspection:
    def test_all_checks_pass(self):
        checker = _checker_with_gate()
        result = checker.inspect('gate1', 'part_A', {
            'chk_dia': 25.0,
            'chk_ra': 1.6,
        }, inspector='op1')
        assert result.overall_passed is True
        assert result.pass_rate_pct == 100.0
        assert len(result.results) == 2
        assert all(r.passed for r in result.results)

    def test_check_at_upper_tolerance_passes(self):
        checker = _checker_with_gate()
        result = checker.inspect('gate1', 'part_A', {
            'chk_dia': 25.05,  # exactly at +tol
            'chk_ra': 2.0,    # exactly at +tol
        }, inspector='op1')
        assert result.overall_passed is True

    def test_check_at_lower_tolerance_passes(self):
        checker = _checker_with_gate()
        result = checker.inspect('gate1', 'part_A', {
            'chk_dia': 24.95,  # exactly at -tol
            'chk_ra': 1.25,    # within -tol (lower bound ~1.2)
        }, inspector='op1')
        assert result.overall_passed is True

    def test_check_out_of_tolerance_fails(self):
        checker = _checker_with_gate()
        result = checker.inspect('gate1', 'part_A', {
            'chk_dia': 25.1,  # over +tol
            'chk_ra': 1.6,
        }, inspector='op1')
        assert result.overall_passed is False
        assert result.pass_rate_pct == 50.0

    def test_critical_check_failure_overrides_threshold(self):
        """Even with a 50% pass threshold, a critical check failure fails the gate."""
        critical_dia = _diameter_check(is_critical=True)
        checker = _checker_with_gate(
            checks=[critical_dia, _surface_check()],
            pass_threshold_pct=50.0,
        )
        result = checker.inspect('gate1', 'part_A', {
            'chk_dia': 30.0,  # way out of tolerance (critical)
            'chk_ra': 1.6,    # passes
        }, inspector='op1')
        # 50% pass rate meets the 50% threshold, but critical failure overrides
        assert result.overall_passed is False

    def test_non_critical_failure_with_threshold(self):
        """Non-critical failure still passes if within threshold."""
        checker = _checker_with_gate(pass_threshold_pct=50.0)
        result = checker.inspect('gate1', 'part_A', {
            'chk_dia': 30.0,  # fails (non-critical)
            'chk_ra': 1.6,    # passes
        }, inspector='op1')
        assert result.pass_rate_pct == 50.0
        assert result.overall_passed is True

    def test_missing_measurement_counts_as_failure(self):
        checker = _checker_with_gate()
        result = checker.inspect('gate1', 'part_A', {
            'chk_dia': 25.0,
            # chk_ra is missing
        }, inspector='op1')
        assert result.overall_passed is False
        assert result.pass_rate_pct == 50.0
        ra_result = [r for r in result.results if r.check_id == 'chk_ra'][0]
        assert ra_result.passed is False
        assert math.isnan(ra_result.measured_value)

    def test_unknown_gate_raises_key_error(self):
        checker = QualityGateChecker()
        with pytest.raises(KeyError, match='Unknown gate'):
            checker.inspect('nonexistent', 'part_A', {}, 'op1')

    def test_deviation_computed_correctly(self):
        checker = _checker_with_gate()
        result = checker.inspect('gate1', 'part_A', {
            'chk_dia': 25.03,
            'chk_ra': 1.5,
        }, inspector='op1')
        dia_result = [r for r in result.results if r.check_id == 'chk_dia'][0]
        assert abs(dia_result.deviation - 0.03) < 1e-9
        ra_result = [r for r in result.results if r.check_id == 'chk_ra'][0]
        assert abs(ra_result.deviation - (-0.1)) < 1e-9


# ===================================================================
# Tests: Part history
# ===================================================================

class TestPartHistory:
    def test_history_recorded(self):
        checker = _checker_with_gate()
        checker.inspect('gate1', 'part_A', {'chk_dia': 25.0, 'chk_ra': 1.6}, 'op1')
        history = checker.get_part_history('part_A')
        assert len(history) == 1
        assert history[0].gate_id == 'gate1'

    def test_multiple_inspections_tracked(self):
        checker = _checker_with_gate()
        checker.inspect('gate1', 'part_A', {'chk_dia': 25.0, 'chk_ra': 1.6}, 'op1')
        checker.inspect('gate1', 'part_A', {'chk_dia': 25.0, 'chk_ra': 1.6}, 'op2')
        history = checker.get_part_history('part_A')
        assert len(history) == 2

    def test_empty_history_for_unknown_part(self):
        checker = QualityGateChecker()
        assert checker.get_part_history('unknown') == []

    def test_history_is_a_copy(self):
        checker = _checker_with_gate()
        checker.inspect('gate1', 'part_A', {'chk_dia': 25.0, 'chk_ra': 1.6}, 'op1')
        h1 = checker.get_part_history('part_A')
        h2 = checker.get_part_history('part_A')
        assert h1 is not h2  # different list objects


# ===================================================================
# Tests: Gate statistics
# ===================================================================

class TestGateStatistics:
    def test_stats_no_inspections(self):
        checker = _checker_with_gate()
        stats = checker.get_gate_statistics('gate1')
        assert stats['total_inspections'] == 0
        assert stats['pass_rate_pct'] == 0.0
        assert stats['most_failed_check'] is None
        assert stats['avg_deviation'] == {}

    def test_stats_all_pass(self):
        checker = _checker_with_gate()
        for pid in ['p1', 'p2', 'p3']:
            checker.inspect('gate1', pid, {'chk_dia': 25.0, 'chk_ra': 1.6}, 'op1')
        stats = checker.get_gate_statistics('gate1')
        assert stats['total_inspections'] == 3
        assert stats['pass_rate_pct'] == 100.0

    def test_stats_most_failed_check(self):
        checker = _checker_with_gate()
        # Two parts fail diameter, one part fails surface roughness
        checker.inspect('gate1', 'p1', {'chk_dia': 30.0, 'chk_ra': 1.6}, 'op1')
        checker.inspect('gate1', 'p2', {'chk_dia': 30.0, 'chk_ra': 5.0}, 'op1')
        checker.inspect('gate1', 'p3', {'chk_dia': 25.0, 'chk_ra': 1.6}, 'op1')
        stats = checker.get_gate_statistics('gate1')
        assert stats['most_failed_check'] == 'chk_dia'

    def test_stats_avg_deviation(self):
        checker = _checker_with_gate()
        # nominal is 25.0; measurements 25.02 and 24.98 -> deviations +0.02, -0.02
        checker.inspect('gate1', 'p1', {'chk_dia': 25.02, 'chk_ra': 1.6}, 'op1')
        checker.inspect('gate1', 'p2', {'chk_dia': 24.98, 'chk_ra': 1.6}, 'op1')
        stats = checker.get_gate_statistics('gate1')
        assert abs(stats['avg_deviation']['chk_dia'] - 0.0) < 1e-9


# ===================================================================
# Tests: can_proceed (stage gating)
# ===================================================================

class TestCanProceed:
    def test_proceed_when_no_gates_defined(self):
        checker = QualityGateChecker()
        assert checker.can_proceed('part_A', 'finishing') is True

    def test_proceed_when_mandatory_gate_passed(self):
        checker = _checker_with_gate(gate_id='g_roughing', stage='roughing')
        checker.inspect('g_roughing', 'part_A',
                        {'chk_dia': 25.0, 'chk_ra': 1.6}, 'op1')
        assert checker.can_proceed('part_A', 'semi_finish') is True

    def test_blocked_when_mandatory_gate_not_passed(self):
        checker = _checker_with_gate(gate_id='g_roughing', stage='roughing')
        # Part never inspected
        assert checker.can_proceed('part_A', 'semi_finish') is False

    def test_blocked_when_mandatory_gate_failed(self):
        checker = _checker_with_gate(gate_id='g_roughing', stage='roughing')
        checker.inspect('g_roughing', 'part_A',
                        {'chk_dia': 30.0, 'chk_ra': 1.6}, 'op1')
        # Gate failed (diameter out of tolerance)
        assert checker.can_proceed('part_A', 'semi_finish') is False

    def test_optional_gate_does_not_block(self):
        checker = _checker_with_gate(
            gate_id='g_optional', stage='roughing', is_mandatory=False,
        )
        # Part never inspected, but gate is optional
        assert checker.can_proceed('part_A', 'semi_finish') is True

    def test_custom_stage_always_proceeds(self):
        checker = _checker_with_gate(gate_id='g_roughing', stage='roughing')
        # 'custom_stage' is not in STAGE_ORDER, so always allowed
        assert checker.can_proceed('part_A', 'custom_stage') is True

    def test_multiple_mandatory_gates_all_must_pass(self):
        checker = QualityGateChecker()
        checker.add_gate(_make_gate(gate_id='g_incoming', stage='incoming'))
        checker.add_gate(_make_gate(gate_id='g_roughing', stage='roughing'))
        # Pass incoming but not roughing
        checker.inspect('g_incoming', 'part_A',
                        {'chk_dia': 25.0, 'chk_ra': 1.6}, 'op1')
        # Trying to proceed to semi_finish requires both incoming + roughing
        assert checker.can_proceed('part_A', 'semi_finish') is False
        # Now pass roughing too
        checker.inspect('g_roughing', 'part_A',
                        {'chk_dia': 25.0, 'chk_ra': 1.6}, 'op1')
        assert checker.can_proceed('part_A', 'semi_finish') is True

    def test_proceed_to_first_stage_always_allowed(self):
        checker = _checker_with_gate(gate_id='g_roughing', stage='roughing')
        # 'incoming' is the first stage, no prior stages required
        assert checker.can_proceed('part_A', 'incoming') is True
