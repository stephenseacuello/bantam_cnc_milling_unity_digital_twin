"""Tests for alarm escalation -> job scheduler integration.

Validates that alarm escalation events correctly pause, block, and
reassign jobs depending on the escalation level/action.  Uses lightweight
stubs that avoid ROS2 dependencies.
"""

import os
import sys
import threading
import time
from dataclasses import dataclass, field
from types import ModuleType
from typing import Any, Dict, List, Optional, Set
from unittest.mock import MagicMock

import pytest

# Ensure the miracle_mes source directory is on sys.path so the real
# miracle_mes package can be found.
_MES_SRC = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir)
)
if _MES_SRC not in sys.path:
    sys.path.insert(0, _MES_SRC)


# ---------------------------------------------------------------------------
# Stub modules injected *before* the production module is imported so that
# ``from miracle_core.…`` / ``from miracle_msgs.…`` resolve without ROS2.
# We must NOT stub miracle_mes itself (it is a real package on sys.path).
# ---------------------------------------------------------------------------

# Pre-import miracle_mes so it is the real package, not a stub module.
import miracle_mes  # noqa: E402, F401


def _ensure_module(dotted_name: str) -> ModuleType:
    """Create a stub module (and parents) in ``sys.modules`` if absent."""
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

# miracle_core stubs
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


# miracle_mes.digital_thread stub -- installed into sys.modules so the
# ``from miracle_mes.digital_thread import DigitalThreadNode`` inside
# job_scheduler.py resolves correctly.

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
        self._entries: List[Dict[str, Any]] = []
        self._lock = threading.Lock()

    def record_genealogy_event(
        self, entry_type, machine_id,
        serial_number='', tool_id='',
        batch_id='', metadata=None,
    ):
        entry = {
            'entry_type': entry_type,
            'machine_id': machine_id,
            'timestamp': time.time(),
            'serial_number': serial_number,
            'tool_id': tool_id,
            'batch_id': batch_id,
        }
        if metadata:
            entry.update(metadata)
        with self._lock:
            self._entries.append(entry)

    def set_active_job(self, **kw):
        pass

    def clear_active_job(self, machine_id):
        pass


_dt_mod = _ensure_module('miracle_mes.digital_thread')
_dt_mod.DigitalThreadNode = _StubDigitalThread

# ---------------------------------------------------------------------------
# NOW we can safely import the production module
# ---------------------------------------------------------------------------
from miracle_mes.job_scheduler import JobSchedulerNode, Job, Priority  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_escalation_msg(
    machine_id: str = 'cnc1',
    action: str = 'PRIORITY_BOOST',
    reason: str = 'test reason',
    alarm_id: str = 'A-001',
    severity: float = 0.5,
    escalation_level: int = 1,
):
    """Build a lightweight alarm-escalation message."""
    msg = MagicMock()
    msg.machine_id = machine_id
    msg.escalation_action = action
    msg.reason = reason
    msg.alarm_id = alarm_id
    msg.severity = severity
    msg.escalation_level = escalation_level
    return msg


def _make_machine_state(machine_id: str, status: str = 'IDLE'):
    """Build a lightweight machine state stub."""
    s = MagicMock()
    s.machine_id = machine_id
    s.status = status
    return s


@pytest.fixture
def scheduler():
    """Provide a fully configured JobSchedulerNode with a digital thread."""
    node = JobSchedulerNode()
    node._do_configure()
    node._digital_thread = _StubDigitalThread()
    return node


@pytest.fixture
def scheduler_no_dt():
    """Provide a configured scheduler *without* a digital thread."""
    node = JobSchedulerNode()
    node._do_configure()
    return node


def _add_active_job(
    scheduler: JobSchedulerNode,
    job_id: str = 'J-100',
    machine_id: str = 'cnc1',
    status: str = 'RUNNING',
) -> Job:
    """Insert a synthetic active job into the scheduler."""
    job = Job(
        priority=Priority.NORMAL,
        submit_time=time.time(),
        job_id=job_id,
        program_name='facing.nc',
        machine_id=machine_id,
        status=status,
        part_serial='SN-001',
        material_batch='BATCH-1',
        tool_id='T-10',
    )
    scheduler._active_jobs[job_id] = job
    return job


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestLevel1PriorityBoost:
    """Level 1 (PRIORITY_BOOST) should only log, never pause a job."""

    def test_no_job_pause(self, scheduler):
        job = _add_active_job(scheduler)
        msg = _make_escalation_msg(action='PRIORITY_BOOST')
        scheduler._on_alarm_escalation(msg)

        assert job.status == 'RUNNING'
        assert len(scheduler._job_history) == 0

    def test_log_warning_emitted(self, scheduler):
        _add_active_job(scheduler)
        msg = _make_escalation_msg(action='PRIORITY_BOOST')
        scheduler._on_alarm_escalation(msg)

        scheduler.get_logger().warn.assert_called()


class TestLevel2ForcedAck:
    """Level 2 (FORCED_ACK_REQUIRED) should pause the active job."""

    def test_active_job_paused(self, scheduler):
        job = _add_active_job(scheduler)
        msg = _make_escalation_msg(action='FORCED_ACK_REQUIRED')
        scheduler._on_alarm_escalation(msg)

        assert job.status == 'PAUSED'

    def test_job_history_recorded(self, scheduler):
        _add_active_job(scheduler)
        msg = _make_escalation_msg(
            action='FORCED_ACK_REQUIRED', reason='spindle overheat',
        )
        scheduler._on_alarm_escalation(msg)

        assert len(scheduler._job_history) == 1
        entry = scheduler._job_history[0]
        assert 'Paused due to alarm escalation' in entry['reason']
        assert 'spindle overheat' in entry['reason']

    def test_no_crash_when_no_active_job(self, scheduler):
        """No active job on the affected machine should not raise."""
        msg = _make_escalation_msg(
            machine_id='cnc_nonexistent',
            action='FORCED_ACK_REQUIRED',
        )
        scheduler._on_alarm_escalation(msg)  # should not raise

    def test_genealogy_event_recorded(self, scheduler):
        _add_active_job(scheduler)
        msg = _make_escalation_msg(action='FORCED_ACK_REQUIRED')
        scheduler._on_alarm_escalation(msg)

        entries = scheduler._digital_thread._entries
        paused = [e for e in entries if e['entry_type'] == 'JOB_PAUSED']
        assert len(paused) == 1
        assert 'alarm escalation' in paused[0].get('reason', '')


class TestLevel3SupervisorNotify:
    """Level 3 (SUPERVISOR_NOTIFY) should pause job and block machine."""

    def test_machine_blocked(self, scheduler):
        _add_active_job(scheduler)
        msg = _make_escalation_msg(action='SUPERVISOR_NOTIFY')
        scheduler._on_alarm_escalation(msg)

        assert 'cnc1' in scheduler.blocked_machines

    def test_job_paused(self, scheduler):
        job = _add_active_job(scheduler)
        msg = _make_escalation_msg(action='SUPERVISOR_NOTIFY')
        scheduler._on_alarm_escalation(msg)

        assert job.status == 'PAUSED'


class TestEmergencyStop:
    """EMERGENCY_STOP should pause, block, and attempt reassignment."""

    def test_job_paused_and_machine_blocked(self, scheduler):
        job = _add_active_job(scheduler)
        msg = _make_escalation_msg(action='EMERGENCY_STOP')
        scheduler._on_alarm_escalation(msg)

        assert job.status == 'PAUSED'
        assert 'cnc1' in scheduler.blocked_machines

    def test_reassignment_attempted(self, scheduler):
        """If another machine is available the job should be reassigned."""
        _add_active_job(scheduler, machine_id='cnc1')
        # Add an available alternate machine
        scheduler._machine_states['cnc2'] = _make_machine_state('cnc2', 'IDLE')

        msg = _make_escalation_msg(machine_id='cnc1', action='EMERGENCY_STOP')
        scheduler._on_alarm_escalation(msg)

        job = scheduler._active_jobs['J-100']
        assert job.machine_id == 'cnc2'
        assert job.status == 'ASSIGNED'

    def test_reassignment_fails_no_machines(self, scheduler):
        """When no alternate machines exist the job stays paused."""
        job = _add_active_job(scheduler, machine_id='cnc1')
        msg = _make_escalation_msg(machine_id='cnc1', action='EMERGENCY_STOP')
        scheduler._on_alarm_escalation(msg)

        assert job.status == 'PAUSED'

    def test_critical_log_emitted(self, scheduler):
        _add_active_job(scheduler)
        msg = _make_escalation_msg(action='EMERGENCY_STOP')
        scheduler._on_alarm_escalation(msg)

        scheduler.get_logger().critical.assert_called()


class TestBlockedMachineScheduling:
    """Blocked machines should be skipped during scheduling."""

    def test_blocked_machine_skipped(self, scheduler):
        scheduler._machine_states['cnc1'] = _make_machine_state('cnc1', 'IDLE')
        scheduler._blocked_machines.add('cnc1')

        # Add a queued job
        import heapq
        job = Job(
            priority=Priority.NORMAL,
            submit_time=time.time(),
            job_id='J-200',
            program_name='test.nc',
        )
        heapq.heappush(scheduler._job_queue, job)

        scheduler._scheduling_cycle()

        # Job should remain in queue -- no assignment because only machine
        # is blocked.
        assert 'J-200' not in scheduler._active_jobs


class TestUnblockMachine:
    """Machine should be unblocked after alarm is cleared."""

    def test_unblock_removes_from_set(self, scheduler):
        scheduler._blocked_machines.add('cnc1')
        scheduler._unblock_machine('cnc1')

        assert 'cnc1' not in scheduler.blocked_machines

    def test_cleared_event_unblocks(self, scheduler):
        scheduler._blocked_machines.add('cnc1')
        msg = _make_escalation_msg(
            machine_id='cnc1', action='CLEARED',
        )
        scheduler._on_alarm_escalation(msg)

        assert 'cnc1' not in scheduler.blocked_machines


class TestReassignJob:
    """reassign_job should move a job to a new machine."""

    def test_reassign_to_specific_machine(self, scheduler):
        _add_active_job(scheduler, machine_id='cnc1')
        scheduler._machine_states['cnc2'] = _make_machine_state('cnc2', 'IDLE')

        result = scheduler.reassign_job('J-100', 'cnc1', to_machine='cnc2')

        assert result is True
        assert scheduler._active_jobs['J-100'].machine_id == 'cnc2'

    def test_reassign_auto_select(self, scheduler):
        _add_active_job(scheduler, machine_id='cnc1')
        scheduler._machine_states['cnc3'] = _make_machine_state('cnc3', 'IDLE')

        result = scheduler.reassign_job('J-100', 'cnc1')

        assert result is True
        assert scheduler._active_jobs['J-100'].machine_id == 'cnc3'

    def test_reassign_fails_no_candidates(self, scheduler):
        _add_active_job(scheduler, machine_id='cnc1')

        result = scheduler.reassign_job('J-100', 'cnc1')

        assert result is False

    def test_reassign_skips_blocked_machines(self, scheduler):
        _add_active_job(scheduler, machine_id='cnc1')
        scheduler._machine_states['cnc2'] = _make_machine_state('cnc2', 'IDLE')
        scheduler._blocked_machines.add('cnc2')

        result = scheduler.reassign_job('J-100', 'cnc1')

        assert result is False


class TestGetBlockedMachines:
    """blocked_machines property should return a copy of the set."""

    def test_returns_correct_set(self, scheduler):
        scheduler._blocked_machines = {'cnc1', 'cnc3'}
        assert scheduler.blocked_machines == {'cnc1', 'cnc3'}

    def test_returns_copy(self, scheduler):
        scheduler._blocked_machines = {'cnc1'}
        result = scheduler.blocked_machines
        result.add('cnc99')
        assert 'cnc99' not in scheduler._blocked_machines


class TestDuplicateEscalation:
    """Multiple escalations on the same machine should not double-pause."""

    def test_already_paused_job_not_re_paused(self, scheduler):
        job = _add_active_job(scheduler, status='RUNNING')
        msg = _make_escalation_msg(action='FORCED_ACK_REQUIRED')

        scheduler._on_alarm_escalation(msg)
        assert job.status == 'PAUSED'
        history_count_after_first = len(scheduler._job_history)

        # Second escalation -- job is already paused
        scheduler._on_alarm_escalation(msg)
        assert job.status == 'PAUSED'
        # No additional history entry
        assert len(scheduler._job_history) == history_count_after_first
