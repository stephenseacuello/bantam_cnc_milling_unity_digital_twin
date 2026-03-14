"""Tests for JobQueueOptimizer and JobPriority.

Validates intelligent queue priority optimization including:
- Priority computation with all factors
- Due date urgency (far/near/overdue)
- Setup affinity scoring
- Tool availability impact
- Queue optimization ordering
- Setup batching
- Completion time estimation
- Bottleneck identification
- Parallel job assignment
- Edge cases (single job, empty queue, same priority, etc.)
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
    Job,
    JobPriority,
    JobQueueOptimizer,
    JobSchedulerNode,
    Priority,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_job(
    job_id: str = 'j1',
    priority: int = Priority.NORMAL,
    due_date: float = 0.0,
    fixture_type: str = '',
    required_tools: Optional[List[str]] = None,
    material_status: str = 'LOADED',
    compatible_machines: Optional[List[str]] = None,
    estimated_duration_sec: float = 3600.0,
    **kwargs,
) -> Job:
    return Job(
        priority=priority,
        submit_time=time.time(),
        job_id=job_id,
        program_name=f'prog_{job_id}',
        due_date=due_date,
        fixture_type=fixture_type,
        required_tools=required_tools or [],
        material_status=material_status,
        compatible_machines=compatible_machines or [],
        estimated_duration_sec=estimated_duration_sec,
        **kwargs,
    )


def _default_setup() -> Dict[str, Any]:
    return {
        'fixture_type': 'vise_A',
        'tooling': ['T1', 'T2'],
        'machine_id': 'cnc1',
    }


def _default_tool_states() -> Dict[str, float]:
    return {'T1': 90.0, 'T2': 45.0, 'T3': 10.0, 'T4': 0.0}


# ===================================================================
# Tests: JobPriority dataclass
# ===================================================================

class TestJobPriority:
    def test_dataclass_fields(self):
        jp = JobPriority(
            base_priority=1,
            due_date_urgency=0.8,
            setup_affinity=0.9,
            tool_availability=0.7,
            material_readiness=1.0,
            machine_suitability=0.5,
            composite_score=0.85,
        )
        assert jp.base_priority == 1
        assert jp.due_date_urgency == 0.8
        assert jp.composite_score == 0.85


# ===================================================================
# Tests: JobQueueOptimizer - Priority Computation
# ===================================================================

class TestComputePriority:
    def setup_method(self):
        self.opt = JobQueueOptimizer()
        self.now = time.time()

    def test_basic_priority_computation(self):
        job = _make_job(priority=Priority.NORMAL)
        jp = self.opt.compute_priority(job, _default_setup(), _default_tool_states(), self.now)
        assert isinstance(jp, JobPriority)
        assert jp.composite_score > 0

    def test_rush_higher_than_low(self):
        rush = _make_job('rush', priority=Priority.RUSH)
        low = _make_job('low', priority=Priority.LOW)
        jp_rush = self.opt.compute_priority(rush, _default_setup(), _default_tool_states(), self.now)
        jp_low = self.opt.compute_priority(low, _default_setup(), _default_tool_states(), self.now)
        assert jp_rush.composite_score > jp_low.composite_score

    def test_all_factors_present(self):
        job = _make_job(
            due_date=self.now + 1800,  # 30 min from now
            fixture_type='vise_A',
            required_tools=['T1'],
            material_status='LOADED',
            compatible_machines=['cnc1'],
        )
        jp = self.opt.compute_priority(job, _default_setup(), _default_tool_states(), self.now)
        assert 0 < jp.due_date_urgency <= 1.0
        assert jp.setup_affinity > 0
        assert jp.tool_availability > 0
        assert jp.material_readiness == 1.0
        assert jp.machine_suitability == 1.0

    def test_base_priority_mapping(self):
        for prio_enum, expected_base in [
            (Priority.RUSH, 1), (Priority.HIGH, 2),
            (Priority.NORMAL, 3), (Priority.LOW, 4),
        ]:
            job = _make_job(priority=prio_enum)
            jp = self.opt.compute_priority(job, {}, {}, self.now)
            assert jp.base_priority == expected_base


# ===================================================================
# Tests: Due Date Urgency
# ===================================================================

class TestDueDateUrgency:
    def setup_method(self):
        self.opt = JobQueueOptimizer()
        self.now = time.time()

    def test_no_deadline(self):
        job = _make_job(due_date=0.0)
        jp = self.opt.compute_priority(job, {}, {}, self.now)
        assert jp.due_date_urgency == 0.0

    def test_far_deadline_low_urgency(self):
        job = _make_job(due_date=self.now + 48 * 3600)  # 48h away
        jp = self.opt.compute_priority(job, {}, {}, self.now)
        assert jp.due_date_urgency < 0.1

    def test_near_deadline_high_urgency(self):
        job = _make_job(due_date=self.now + 600)  # 10 min away
        jp = self.opt.compute_priority(job, {}, {}, self.now)
        assert jp.due_date_urgency > 0.5

    def test_overdue_max_urgency(self):
        job = _make_job(due_date=self.now - 3600)  # 1h overdue
        jp = self.opt.compute_priority(job, {}, {}, self.now)
        assert jp.due_date_urgency == 1.0

    def test_urgency_increases_as_deadline_approaches(self):
        far = _make_job('far', due_date=self.now + 24 * 3600)
        mid = _make_job('mid', due_date=self.now + 3600)
        near = _make_job('near', due_date=self.now + 300)
        jp_far = self.opt.compute_priority(far, {}, {}, self.now)
        jp_mid = self.opt.compute_priority(mid, {}, {}, self.now)
        jp_near = self.opt.compute_priority(near, {}, {}, self.now)
        assert jp_far.due_date_urgency < jp_mid.due_date_urgency < jp_near.due_date_urgency


# ===================================================================
# Tests: Setup Affinity
# ===================================================================

class TestSetupAffinity:
    def setup_method(self):
        self.opt = JobQueueOptimizer()
        self.now = time.time()

    def test_same_fixture_high_affinity(self):
        job = _make_job(fixture_type='vise_A', required_tools=['T1', 'T2'])
        setup = {'fixture_type': 'vise_A', 'tooling': ['T1', 'T2']}
        jp = self.opt.compute_priority(job, setup, {}, self.now)
        assert jp.setup_affinity == 1.0

    def test_different_fixture_low_affinity(self):
        job = _make_job(fixture_type='chuck_B', required_tools=['T5'])
        setup = {'fixture_type': 'vise_A', 'tooling': ['T1', 'T2']}
        jp = self.opt.compute_priority(job, setup, {}, self.now)
        assert jp.setup_affinity < 0.5

    def test_no_setup_info_neutral(self):
        job = _make_job(fixture_type='vise_A')
        jp = self.opt.compute_priority(job, {}, {}, self.now)
        assert jp.setup_affinity == 0.5

    def test_partial_tool_overlap(self):
        job = _make_job(fixture_type='vise_A', required_tools=['T1', 'T3'])
        setup = {'fixture_type': 'vise_A', 'tooling': ['T1', 'T2']}
        jp = self.opt.compute_priority(job, setup, {}, self.now)
        # fixture matches (1.0), tool overlap 1/2 (0.5), average = 0.75
        assert jp.setup_affinity == 0.75

    def test_no_fixture_no_tools_neutral(self):
        job = _make_job(fixture_type='', required_tools=[])
        setup = {'fixture_type': 'vise_A', 'tooling': ['T1']}
        jp = self.opt.compute_priority(job, setup, {}, self.now)
        assert jp.setup_affinity == 0.5  # no checks -> neutral


# ===================================================================
# Tests: Tool Availability
# ===================================================================

class TestToolAvailability:
    def setup_method(self):
        self.opt = JobQueueOptimizer()
        self.now = time.time()

    def test_no_tools_required(self):
        job = _make_job(required_tools=[])
        jp = self.opt.compute_priority(job, {}, _default_tool_states(), self.now)
        assert jp.tool_availability == 1.0

    def test_tools_with_good_rul(self):
        job = _make_job(required_tools=['T1'])  # T1 has 90 min RUL
        jp = self.opt.compute_priority(job, {}, _default_tool_states(), self.now)
        assert jp.tool_availability == 1.0

    def test_tools_with_low_rul(self):
        job = _make_job(required_tools=['T3'])  # T3 has 10 min RUL
        jp = self.opt.compute_priority(job, {}, _default_tool_states(), self.now)
        assert jp.tool_availability < 0.3

    def test_tool_dead(self):
        job = _make_job(required_tools=['T4'])  # T4 has 0 RUL
        jp = self.opt.compute_priority(job, {}, _default_tool_states(), self.now)
        assert jp.tool_availability == 0.0

    def test_unknown_tool_state(self):
        job = _make_job(required_tools=['UNKNOWN_TOOL'])
        jp = self.opt.compute_priority(job, {}, _default_tool_states(), self.now)
        assert jp.tool_availability == 0.5

    def test_no_tool_state_data(self):
        job = _make_job(required_tools=['T1'])
        jp = self.opt.compute_priority(job, {}, {}, self.now)
        assert jp.tool_availability == 0.5

    def test_tool_unavailable_deprioritizes(self):
        """Job requiring a dead tool should score lower than one with good tools."""
        good = _make_job('good', required_tools=['T1'], priority=Priority.NORMAL)
        bad = _make_job('bad', required_tools=['T4'], priority=Priority.NORMAL)
        jp_good = self.opt.compute_priority(good, {}, _default_tool_states(), self.now)
        jp_bad = self.opt.compute_priority(bad, {}, _default_tool_states(), self.now)
        assert jp_good.composite_score > jp_bad.composite_score


# ===================================================================
# Tests: Material Readiness
# ===================================================================

class TestMaterialReadiness:
    def setup_method(self):
        self.opt = JobQueueOptimizer()
        self.now = time.time()

    def test_loaded(self):
        job = _make_job(material_status='LOADED')
        jp = self.opt.compute_priority(job, {}, {}, self.now)
        assert jp.material_readiness == 1.0

    def test_queued(self):
        job = _make_job(material_status='QUEUED')
        jp = self.opt.compute_priority(job, {}, {}, self.now)
        assert jp.material_readiness == 0.5

    def test_not_available(self):
        job = _make_job(material_status='NOT_AVAILABLE')
        jp = self.opt.compute_priority(job, {}, {}, self.now)
        assert jp.material_readiness == 0.0


# ===================================================================
# Tests: Machine Suitability
# ===================================================================

class TestMachineSuitability:
    def setup_method(self):
        self.opt = JobQueueOptimizer()
        self.now = time.time()

    def test_no_machine_preference(self):
        job = _make_job(compatible_machines=[])
        jp = self.opt.compute_priority(job, _default_setup(), {}, self.now)
        assert jp.machine_suitability == 1.0

    def test_compatible_machine(self):
        job = _make_job(compatible_machines=['cnc1', 'cnc2'])
        jp = self.opt.compute_priority(job, _default_setup(), {}, self.now)
        assert jp.machine_suitability == 1.0

    def test_incompatible_machine(self):
        job = _make_job(compatible_machines=['cnc5'])
        setup = {'machine_id': 'cnc1'}
        jp = self.opt.compute_priority(job, setup, {}, self.now)
        assert jp.machine_suitability == 0.3


# ===================================================================
# Tests: Queue Optimization Ordering
# ===================================================================

class TestOptimizeQueue:
    def setup_method(self):
        self.opt = JobQueueOptimizer()
        self.now = time.time()

    def test_empty_queue(self):
        result = self.opt.optimize_queue([], {}, {}, self.now)
        assert result == []

    def test_single_job(self):
        job = _make_job('only')
        result = self.opt.optimize_queue([job], {}, {}, self.now)
        assert len(result) == 1
        assert result[0].job_id == 'only'

    def test_rush_before_low(self):
        rush = _make_job('rush', priority=Priority.RUSH)
        low = _make_job('low', priority=Priority.LOW)
        result = self.opt.optimize_queue([low, rush], {}, {}, self.now)
        assert result[0].job_id == 'rush'
        assert result[1].job_id == 'low'

    def test_overdue_job_gets_highest_priority(self):
        overdue = _make_job('overdue', priority=Priority.LOW, due_date=self.now - 3600)
        normal = _make_job('normal', priority=Priority.NORMAL, due_date=0.0)
        result = self.opt.optimize_queue([normal, overdue], {}, {}, self.now)
        # Overdue LOW job should come before NORMAL with no deadline
        assert result[0].job_id == 'overdue'

    def test_all_same_priority_stable_sort(self):
        jobs = [_make_job(f'j{i}', priority=Priority.NORMAL) for i in range(5)]
        result = self.opt.optimize_queue(jobs, {}, {}, self.now)
        assert len(result) == 5
        # With identical scores, idx-based tiebreak preserves original order
        ids = [j.job_id for j in result]
        assert ids == ['j0', 'j1', 'j2', 'j3', 'j4']

    def test_priority_order_respected(self):
        jobs = [
            _make_job('low', priority=Priority.LOW),
            _make_job('rush', priority=Priority.RUSH),
            _make_job('high', priority=Priority.HIGH),
            _make_job('normal', priority=Priority.NORMAL),
        ]
        result = self.opt.optimize_queue(jobs, {}, {}, self.now)
        ids = [j.job_id for j in result]
        assert ids.index('rush') < ids.index('low')
        assert ids.index('high') < ids.index('low')


# ===================================================================
# Tests: Setup Batching
# ===================================================================

class TestSetupBatching:
    def setup_method(self):
        self.opt = JobQueueOptimizer()
        self.now = time.time()

    def test_same_fixture_grouped(self):
        j1 = _make_job('j1', priority=Priority.NORMAL, fixture_type='vise_A')
        j2 = _make_job('j2', priority=Priority.NORMAL, fixture_type='chuck_B')
        j3 = _make_job('j3', priority=Priority.NORMAL, fixture_type='vise_A')
        setup = {'fixture_type': 'vise_A', 'machine_id': 'cnc1'}
        result = self.opt.optimize_queue([j1, j2, j3], setup, {}, self.now)
        ids = [j.job_id for j in result]
        # vise_A jobs should be grouped together and come first (matching current setup)
        vise_a_positions = [ids.index('j1'), ids.index('j3')]
        chuck_b_position = ids.index('j2')
        assert all(p < chuck_b_position for p in vise_a_positions)

    def test_current_fixture_group_first(self):
        j1 = _make_job('j1', priority=Priority.RUSH, fixture_type='chuck_B')
        j2 = _make_job('j2', priority=Priority.NORMAL, fixture_type='vise_A')
        setup = {'fixture_type': 'vise_A', 'machine_id': 'cnc1'}
        result = self.opt.optimize_queue([j1, j2], setup, {}, self.now)
        # vise_A group comes first because it matches current setup
        assert result[0].job_id == 'j2'

    def test_no_fixture_jobs_at_end(self):
        j1 = _make_job('j1', fixture_type='vise_A')
        j2 = _make_job('j2', fixture_type='')
        setup = {'fixture_type': 'vise_A'}
        result = self.opt.optimize_queue([j2, j1], setup, {}, self.now)
        assert result[-1].job_id == 'j2'


# ===================================================================
# Tests: Completion Time Estimation
# ===================================================================

class TestEstimateCompletionTimes:
    def setup_method(self):
        self.opt = JobQueueOptimizer()
        self.now = 1000000.0

    def test_single_job(self):
        job = _make_job('j1', estimated_duration_sec=600)
        result = self.opt.estimate_completion_times([job], self.now)
        assert result['j1'] == self.now + 600

    def test_sequential_jobs(self):
        j1 = _make_job('j1', estimated_duration_sec=600)
        j2 = _make_job('j2', estimated_duration_sec=300)
        result = self.opt.estimate_completion_times([j1, j2], self.now)
        assert result['j1'] == self.now + 600
        assert result['j2'] == self.now + 900

    def test_setup_change_penalty(self):
        j1 = _make_job('j1', fixture_type='vise_A', estimated_duration_sec=600)
        j2 = _make_job('j2', fixture_type='chuck_B', estimated_duration_sec=300)
        result = self.opt.estimate_completion_times([j1, j2], self.now)
        assert result['j1'] == self.now + 600
        # 15 min (900s) setup change + 300s job
        assert result['j2'] == self.now + 600 + 900 + 300

    def test_same_fixture_no_penalty(self):
        j1 = _make_job('j1', fixture_type='vise_A', estimated_duration_sec=600)
        j2 = _make_job('j2', fixture_type='vise_A', estimated_duration_sec=300)
        result = self.opt.estimate_completion_times([j1, j2], self.now)
        assert result['j2'] == self.now + 900

    def test_empty_list(self):
        result = self.opt.estimate_completion_times([], self.now)
        assert result == {}


# ===================================================================
# Tests: Bottleneck Identification
# ===================================================================

class TestIdentifyBottlenecks:
    def setup_method(self):
        self.opt = JobQueueOptimizer()

    def test_no_bottlenecks_with_no_jobs(self):
        result = self.opt.identify_bottlenecks([], ['cnc1', 'cnc2'])
        assert result == []

    def test_no_bottlenecks_with_no_machines(self):
        result = self.opt.identify_bottlenecks([_make_job('j1')], [])
        assert result == []

    def test_machine_bottleneck(self):
        j1 = _make_job('j1', compatible_machines=['cnc1'], estimated_duration_sec=3600)
        j2 = _make_job('j2', compatible_machines=['cnc1'], estimated_duration_sec=3600)
        result = self.opt.identify_bottlenecks([j1, j2], ['cnc1', 'cnc2'])
        machine_bottlenecks = [b for b in result if b[0] == 'cnc1']
        assert len(machine_bottlenecks) == 1
        assert machine_bottlenecks[0][1] == 2  # contention count

    def test_tool_bottleneck(self):
        j1 = _make_job('j1', required_tools=['T1'], estimated_duration_sec=3600)
        j2 = _make_job('j2', required_tools=['T1'], estimated_duration_sec=3600)
        result = self.opt.identify_bottlenecks([j1, j2], ['cnc1'])
        tool_bottlenecks = [b for b in result if b[0] == 'T1']
        assert len(tool_bottlenecks) == 1
        assert tool_bottlenecks[0][1] == 2

    def test_bottlenecks_sorted_by_delay(self):
        j1 = _make_job('j1', compatible_machines=['cnc1'], estimated_duration_sec=1000)
        j2 = _make_job('j2', compatible_machines=['cnc1'], estimated_duration_sec=1000)
        j3 = _make_job('j3', compatible_machines=['cnc2'], estimated_duration_sec=5000)
        j4 = _make_job('j4', compatible_machines=['cnc2'], estimated_duration_sec=5000)
        result = self.opt.identify_bottlenecks([j1, j2, j3, j4], ['cnc1', 'cnc2'])
        # cnc2 has more total load, so higher delay impact
        machine_results = [b for b in result if b[0] in ('cnc1', 'cnc2')]
        assert machine_results[0][0] == 'cnc2'


# ===================================================================
# Tests: Parallel Job Assignment
# ===================================================================

class TestSuggestParallelJobs:
    def setup_method(self):
        self.opt = JobQueueOptimizer()

    def test_empty_jobs(self):
        result = self.opt.suggest_parallel_jobs([], ['cnc1'])
        assert result == []

    def test_empty_machines(self):
        result = self.opt.suggest_parallel_jobs([_make_job('j1')], [])
        assert result == []

    def test_one_job_one_machine(self):
        result = self.opt.suggest_parallel_jobs(
            [_make_job('j1')], ['cnc1'],
        )
        assert result == [('cnc1', 'j1')]

    def test_two_jobs_two_machines(self):
        jobs = [_make_job('j1'), _make_job('j2')]
        result = self.opt.suggest_parallel_jobs(jobs, ['cnc1', 'cnc2'])
        assert len(result) == 2
        machine_ids = {m for m, _ in result}
        job_ids = {j for _, j in result}
        assert machine_ids == {'cnc1', 'cnc2'}
        assert job_ids == {'j1', 'j2'}

    def test_respects_machine_compatibility(self):
        j1 = _make_job('j1', compatible_machines=['cnc2'])
        j2 = _make_job('j2', compatible_machines=['cnc1'])
        result = self.opt.suggest_parallel_jobs([j1, j2], ['cnc1', 'cnc2'])
        result_dict = {j: m for m, j in result}
        assert result_dict['j1'] == 'cnc2'
        assert result_dict['j2'] == 'cnc1'

    def test_more_jobs_than_machines(self):
        jobs = [_make_job(f'j{i}') for i in range(5)]
        result = self.opt.suggest_parallel_jobs(jobs, ['cnc1', 'cnc2'])
        assert len(result) == 2  # only 2 machines

    def test_incompatible_job_skipped(self):
        j1 = _make_job('j1', compatible_machines=['cnc3'])  # not available
        j2 = _make_job('j2', compatible_machines=['cnc1'])
        result = self.opt.suggest_parallel_jobs([j1, j2], ['cnc1', 'cnc2'])
        # j1 can't run on cnc1 or cnc2, j2 can run on cnc1
        assert ('cnc1', 'j2') in result
        assert not any(j == 'j1' for _, j in result)


# ===================================================================
# Tests: JobSchedulerNode integration
# ===================================================================

class TestJobSchedulerNodeIntegration:
    def test_node_has_queue_optimizer(self):
        node = JobSchedulerNode()
        assert hasattr(node, '_queue_optimizer')
        assert isinstance(node._queue_optimizer, JobQueueOptimizer)

    def test_node_has_reoptimize_method(self):
        node = JobSchedulerNode()
        assert hasattr(node, '_reoptimize_queue')
        assert callable(node._reoptimize_queue)

    def test_reoptimize_empty_queue(self):
        node = JobSchedulerNode()
        # Should not raise on empty queue
        node._reoptimize_queue()
        assert len(node._job_queue) == 0

    def test_reoptimize_reorders_queue(self):
        node = JobSchedulerNode()
        low = _make_job('low', priority=Priority.LOW)
        rush = _make_job('rush', priority=Priority.RUSH)
        node._job_queue = [low, rush]
        node._reoptimize_queue()
        # After optimization, rush should come first
        assert node._job_queue[0].job_id == 'rush'


# ===================================================================
# Tests: Custom Weights
# ===================================================================

class TestCustomWeights:
    def test_high_urgency_weight(self):
        opt = JobQueueOptimizer(urgency_weight=0.9, setup_weight=0.0,
                                tool_weight=0.0, material_weight=0.0,
                                machine_weight=0.0)
        now = time.time()
        urgent = _make_job('urgent', priority=Priority.LOW, due_date=now + 60)
        relaxed = _make_job('relaxed', priority=Priority.LOW, due_date=now + 86400)
        jp_u = opt.compute_priority(urgent, {}, {}, now)
        jp_r = opt.compute_priority(relaxed, {}, {}, now)
        assert jp_u.composite_score > jp_r.composite_score

    def test_high_setup_weight(self):
        opt = JobQueueOptimizer(urgency_weight=0.0, setup_weight=0.9,
                                tool_weight=0.0, material_weight=0.0,
                                machine_weight=0.0)
        now = time.time()
        match = _make_job('match', priority=Priority.NORMAL,
                          fixture_type='vise_A', required_tools=['T1', 'T2'])
        no_match = _make_job('no_match', priority=Priority.NORMAL,
                             fixture_type='chuck_B', required_tools=['T5'])
        setup = {'fixture_type': 'vise_A', 'tooling': ['T1', 'T2']}
        jp_m = opt.compute_priority(match, setup, {}, now)
        jp_nm = opt.compute_priority(no_match, setup, {}, now)
        assert jp_m.composite_score > jp_nm.composite_score
