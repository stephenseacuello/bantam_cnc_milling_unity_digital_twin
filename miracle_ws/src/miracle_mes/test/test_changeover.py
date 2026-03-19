"""Tests for ChangeoverOptimizer (SMED methodology).

Validates:
- Step registration and analysis
- Category breakdown (internal/external/waste)
- SMED optimisation (externalisation, waste elimination)
- Parallel group identification
- Critical path computation
- Savings estimation
- Checklist generation
- Edge cases (empty optimizer, single step, cycles)
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
    ChangeoverAnalysis,
    ChangeoverOptimizer,
    ChangeoverStep,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_step(
    step_id: str = 's1',
    description: str = 'Test step',
    duration_min: float = 5.0,
    category: str = 'internal',
    can_externalize: bool = False,
    dependencies: Optional[List[str]] = None,
    tools_needed: Optional[List[str]] = None,
) -> ChangeoverStep:
    return ChangeoverStep(
        step_id=step_id,
        description=description,
        duration_min=duration_min,
        category=category,
        can_externalize=can_externalize,
        dependencies=dependencies or [],
        tools_needed=tools_needed or [],
    )


def _populated_optimizer() -> ChangeoverOptimizer:
    """Return an optimizer with a realistic changeover sequence."""
    opt = ChangeoverOptimizer()
    opt.add_step(_make_step('s1', 'Stop machine', 1.0, 'internal'))
    opt.add_step(_make_step('s2', 'Remove old die', 8.0, 'internal',
                            can_externalize=False, dependencies=['s1']))
    opt.add_step(_make_step('s3', 'Clean mounting surface', 3.0, 'internal',
                            can_externalize=False, dependencies=['s2']))
    opt.add_step(_make_step('s4', 'Pre-heat new die', 10.0, 'internal',
                            can_externalize=True,
                            tools_needed=['heater']))
    opt.add_step(_make_step('s5', 'Install new die', 7.0, 'internal',
                            can_externalize=False, dependencies=['s3']))
    opt.add_step(_make_step('s6', 'Gather tools', 4.0, 'external',
                            tools_needed=['wrench', 'gauge']))
    opt.add_step(_make_step('s7', 'Walk to storage', 5.0, 'waste'))
    opt.add_step(_make_step('s8', 'Adjust alignment', 6.0, 'internal',
                            can_externalize=False, dependencies=['s5']))
    return opt


# ===================================================================
# Tests: Step Registration and Analysis
# ===================================================================

class TestAnalyze:
    def test_empty_optimizer_analysis(self):
        opt = ChangeoverOptimizer()
        analysis = opt.analyze()
        assert analysis.total_time_min == 0.0
        assert analysis.internal_time_min == 0.0
        assert analysis.external_time_min == 0.0
        assert analysis.waste_time_min == 0.0
        assert analysis.optimized_time_min == 0.0
        assert analysis.savings_min == 0.0

    def test_single_internal_step(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('s1', 'Install die', 10.0, 'internal'))
        analysis = opt.analyze()
        assert analysis.total_time_min == 10.0
        assert analysis.internal_time_min == 10.0
        assert analysis.external_time_min == 0.0
        assert analysis.waste_time_min == 0.0

    def test_mixed_categories(self):
        opt = _populated_optimizer()
        analysis = opt.analyze()
        # internal: s1(1) + s2(8) + s3(3) + s4(10) + s5(7) + s8(6) = 35
        assert analysis.internal_time_min == 35.0
        # external: s6(4)
        assert analysis.external_time_min == 4.0
        # waste: s7(5)
        assert analysis.waste_time_min == 5.0
        assert analysis.total_time_min == 44.0

    def test_analysis_no_savings(self):
        """Analysis (not optimize) should report zero savings."""
        opt = _populated_optimizer()
        analysis = opt.analyze()
        assert analysis.savings_min == 0.0
        assert analysis.savings_pct == 0.0
        assert analysis.optimized_time_min == analysis.total_time_min


# ===================================================================
# Tests: SMED Optimisation
# ===================================================================

class TestOptimize:
    def test_optimize_removes_waste(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('w1', 'Walk to storage', 5.0, 'waste'))
        result = opt.optimize()
        assert result.waste_time_min == 5.0
        assert result.optimized_time_min == 0.0
        assert result.savings_min == 5.0

    def test_optimize_externalizes_steps(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('s1', 'Pre-heat die', 10.0, 'internal',
                                can_externalize=True))
        opt.add_step(_make_step('s2', 'Install die', 5.0, 'internal',
                                can_externalize=False))
        result = opt.optimize()
        # Only s2 remains internal
        assert result.optimized_time_min == 5.0
        assert result.savings_min == 10.0

    def test_optimize_populated(self):
        opt = _populated_optimizer()
        result = opt.optimize()
        # Internal steps that can NOT be externalized:
        # s1(1) + s2(8) + s3(3) + s5(7) + s8(6) = 25
        # s4 (10 min) is externalizable
        assert result.optimized_time_min == 25.0
        # Savings = waste(5) + externalized(10) = 15
        assert result.savings_min == 15.0
        assert result.savings_pct == pytest.approx(15.0 / 44.0 * 100.0,
                                                   rel=1e-3)

    def test_optimize_already_optimal(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('s1', 'Quick swap', 2.0, 'internal',
                                can_externalize=False))
        result = opt.optimize()
        assert result.optimized_time_min == 2.0
        # External time doesn't add to downtime so savings = total - internal
        assert result.savings_min == 0.0


# ===================================================================
# Tests: Parallel Groups
# ===================================================================

class TestParallelGroups:
    def test_empty(self):
        opt = ChangeoverOptimizer()
        assert opt.get_parallel_groups() == []

    def test_all_independent(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('a', 'A', 1.0))
        opt.add_step(_make_step('b', 'B', 2.0))
        opt.add_step(_make_step('c', 'C', 3.0))
        groups = opt.get_parallel_groups()
        # All independent -> single group
        assert len(groups) == 1
        assert sorted(groups[0]) == ['a', 'b', 'c']

    def test_linear_chain(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('a', 'A', 1.0))
        opt.add_step(_make_step('b', 'B', 2.0, dependencies=['a']))
        opt.add_step(_make_step('c', 'C', 3.0, dependencies=['b']))
        groups = opt.get_parallel_groups()
        assert len(groups) == 3
        assert groups[0] == ['a']
        assert groups[1] == ['b']
        assert groups[2] == ['c']

    def test_diamond_dependency(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('a', 'A', 1.0))
        opt.add_step(_make_step('b', 'B', 2.0, dependencies=['a']))
        opt.add_step(_make_step('c', 'C', 2.0, dependencies=['a']))
        opt.add_step(_make_step('d', 'D', 1.0, dependencies=['b', 'c']))
        groups = opt.get_parallel_groups()
        assert len(groups) == 3
        assert groups[0] == ['a']
        assert sorted(groups[1]) == ['b', 'c']
        assert groups[2] == ['d']


# ===================================================================
# Tests: Critical Path
# ===================================================================

class TestCriticalPath:
    def test_empty(self):
        opt = ChangeoverOptimizer()
        assert opt.get_critical_path() == []

    def test_single_internal_step(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('s1', 'Only step', 5.0, 'internal'))
        path = opt.get_critical_path()
        assert path == ['s1']

    def test_ignores_external_steps(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('e1', 'External', 100.0, 'external'))
        opt.add_step(_make_step('i1', 'Internal', 5.0, 'internal'))
        path = opt.get_critical_path()
        assert path == ['i1']
        assert 'e1' not in path

    def test_longest_chain(self):
        opt = ChangeoverOptimizer()
        # Chain 1: a(1) -> b(2) = 3 min
        opt.add_step(_make_step('a', 'A', 1.0, 'internal'))
        opt.add_step(_make_step('b', 'B', 2.0, 'internal', dependencies=['a']))
        # Chain 2: c(5) -> d(6) = 11 min (longest)
        opt.add_step(_make_step('c', 'C', 5.0, 'internal'))
        opt.add_step(_make_step('d', 'D', 6.0, 'internal', dependencies=['c']))
        path = opt.get_critical_path()
        assert path == ['c', 'd']

    def test_populated_critical_path(self):
        opt = _populated_optimizer()
        path = opt.get_critical_path()
        # Longest internal chain: s1(1)->s2(8)->s3(3)->s5(7)->s8(6) = 25 min
        assert path == ['s1', 's2', 's3', 's5', 's8']


# ===================================================================
# Tests: Savings Estimation
# ===================================================================

class TestEstimateSavings:
    def test_estimate_internal_step(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('s1', 'Pre-heat', 10.0, 'internal',
                                can_externalize=True))
        savings = opt.estimate_savings(['s1'])
        assert savings == 10.0

    def test_estimate_external_step_no_savings(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('e1', 'Gather tools', 4.0, 'external'))
        savings = opt.estimate_savings(['e1'])
        assert savings == 0.0  # already external

    def test_estimate_nonexistent_step(self):
        opt = ChangeoverOptimizer()
        savings = opt.estimate_savings(['does_not_exist'])
        assert savings == 0.0

    def test_estimate_multiple_steps(self):
        opt = _populated_optimizer()
        savings = opt.estimate_savings(['s4'])  # 10 min internal
        assert savings == 10.0


# ===================================================================
# Tests: Checklist Generation
# ===================================================================

class TestChecklist:
    def test_empty_checklist(self):
        opt = ChangeoverOptimizer()
        checklist = opt.get_checklist()
        assert checklist == []

    def test_internal_before_external(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('e1', 'Gather tools', 4.0, 'external'))
        opt.add_step(_make_step('i1', 'Install die', 7.0, 'internal'))
        checklist = opt.get_checklist()
        assert len(checklist) == 2
        assert checklist[0]['category'] == 'internal'
        assert checklist[1]['category'] == 'external'

    def test_waste_excluded(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('w1', 'Walk around', 5.0, 'waste'))
        opt.add_step(_make_step('i1', 'Install', 3.0, 'internal'))
        checklist = opt.get_checklist()
        assert len(checklist) == 1
        assert checklist[0]['step_id'] == 'i1'

    def test_sequential_numbering(self):
        opt = _populated_optimizer()
        checklist = opt.get_checklist()
        sequences = [item['sequence'] for item in checklist]
        assert sequences == list(range(1, len(checklist) + 1))

    def test_checklist_has_required_fields(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('s1', 'Test', 5.0, 'internal',
                                tools_needed=['wrench']))
        checklist = opt.get_checklist()
        item = checklist[0]
        assert 'sequence' in item
        assert 'step_id' in item
        assert 'description' in item
        assert 'duration_min' in item
        assert 'category' in item
        assert 'tools_needed' in item
        assert item['tools_needed'] == ['wrench']


# ===================================================================
# Tests: Recommendations
# ===================================================================

class TestRecommendations:
    def test_recommendations_for_externalizable(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('s1', 'Pre-heat die', 10.0, 'internal',
                                can_externalize=True))
        analysis = opt.analyze()
        assert any("Externalize" in r for r in analysis.recommendations)

    def test_recommendations_for_waste(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('w1', 'Unnecessary walk', 5.0, 'waste'))
        analysis = opt.analyze()
        assert any("Eliminate waste" in r for r in analysis.recommendations)

    def test_already_optimal_recommendation(self):
        opt = ChangeoverOptimizer()
        opt.add_step(_make_step('s1', 'Quick swap', 2.0, 'internal',
                                can_externalize=False))
        analysis = opt.analyze()
        assert any("well-optimized" in r for r in analysis.recommendations)
