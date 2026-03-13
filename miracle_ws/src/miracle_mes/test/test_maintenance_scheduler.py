"""Tests for predictive maintenance scheduling in the job scheduler.

Covers MaintenanceWindow dataclass, MaintenanceScheduler class, and
integration with JobSchedulerNode.  Uses lightweight stubs that avoid
ROS2 dependencies.
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

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
_MES_SRC = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir)
)
if _MES_SRC not in sys.path:
    sys.path.insert(0, _MES_SRC)

# Pre-import miracle_mes so it is the real package, not a stub.
import miracle_mes  # noqa: E402, F401


def _ensure_module(dotted_name: str) -> ModuleType:
    """Create a stub module (and parents) in sys.modules if absent."""
    parts = dotted_name.split('.')
    for i in range(1, len(parts) + 1):
        partial = '.'.join(parts[:i])
        if partial not in sys.modules:
            sys.modules[partial] = ModuleType(partial)
    return sys.modules[dotted_name]


# ---------------------------------------------------------------------------
# rclpy stubs
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# miracle_core stubs (sub-modules only, not top-level)
# ---------------------------------------------------------------------------
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
        clock.now.return_value.__sub__ = lambda s, o: MagicMock(nanoseconds=0)
        return clock


_mc_lifecycle.MiracleLifecycleNode = _StubLifecycleNode

_mc_qos.QoSProfiles = MagicMock()
_mc_qos.QoSProfiles.alert.return_value = MagicMock()
_mc_qos.QoSProfiles.state_data.return_value = MagicMock()
_mc_qos.QoSProfiles.command.return_value = MagicMock()

# ---------------------------------------------------------------------------
# miracle_msgs stubs
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# miracle_mes.digital_thread stub
# ---------------------------------------------------------------------------


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
        self, entry_type, machine_id='',
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

    def record_operation_complete(self, **kw):
        pass


_dt_mod = _ensure_module('miracle_mes.digital_thread')
_dt_mod.DigitalThreadNode = _StubDigitalThread

# ---------------------------------------------------------------------------
# NOW import the production module
# ---------------------------------------------------------------------------
from miracle_mes.job_scheduler import (  # noqa: E402
    Job,
    JobSchedulerNode,
    MaintenanceScheduler,
    MaintenanceWindow,
    Priority,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def scheduler():
    """Create a standalone MaintenanceScheduler."""
    return MaintenanceScheduler()


@pytest.fixture
def node():
    """Create a JobSchedulerNode with stub infrastructure."""
    n = JobSchedulerNode()
    n._do_configure()
    return n


@pytest.fixture
def node_with_thread(node):
    """Node with a digital thread attached."""
    node._digital_thread = _StubDigitalThread()
    return node


def _make_active_job(node, job_id='job1', machine_id='cnc1', status='RUNNING'):
    """Helper to insert an active job directly."""
    job = Job(
        priority=Priority.NORMAL,
        submit_time=time.time(),
        job_id=job_id,
        program_name='test_prog',
        machine_id=machine_id,
        status=status,
    )
    node._active_jobs[job_id] = job
    return job


# ===========================================================================
# Tests
# ===========================================================================


class TestMaintenanceWindowDataclass:
    """Tests for the MaintenanceWindow dataclass."""

    def test_create_maintenance_window(self):
        w = MaintenanceWindow(
            window_id='w1',
            machine_id='cnc1',
            maintenance_type='TOOL_CHANGE',
            scheduled_start=time.time() + 600,
            estimated_duration_min=15.0,
            priority=2,
            triggered_by='prediction',
            tool_id='T01',
            rul_at_scheduling=25.0,
        )
        assert w.window_id == 'w1'
        assert w.maintenance_type == 'TOOL_CHANGE'
        assert w.priority == 2
        assert w.status == 'PENDING'

    def test_default_optional_fields(self):
        w = MaintenanceWindow(
            window_id='w2',
            machine_id='cnc2',
            maintenance_type='CALIBRATION',
            scheduled_start=time.time(),
            estimated_duration_min=30.0,
            priority=3,
            triggered_by='schedule',
        )
        assert w.tool_id == ''
        assert w.rul_at_scheduling == 0.0


class TestScheduleRoutineMaintenance:
    """Tests for scheduling routine maintenance."""

    def test_schedule_basic(self, scheduler):
        w = scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=120)
        assert w.machine_id == 'cnc1'
        assert w.maintenance_type == 'PREVENTIVE'
        assert w.status == 'PENDING'
        assert w.priority == 5  # > 60 min => routine

    def test_schedule_with_custom_priority(self, scheduler):
        w = scheduler.schedule_maintenance('cnc1', 'COOLANT_REFILL', urgency_minutes=45, priority=2)
        assert w.priority == 2

    def test_schedule_uses_default_duration(self, scheduler):
        w = scheduler.schedule_maintenance('cnc1', 'SPINDLE_SERVICE')
        assert w.estimated_duration_min == 60.0

    def test_schedule_custom_duration(self, scheduler):
        w = scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', estimated_duration_min=90.0)
        assert w.estimated_duration_min == 90.0


class TestScheduleUrgentToolChange:
    """Tests for scheduling urgent tool changes."""

    def test_urgent_tool_change_high_priority(self, scheduler):
        w = scheduler.schedule_maintenance(
            'cnc1', 'TOOL_CHANGE', urgency_minutes=5, tool_id='T01',
        )
        assert w.priority == 1  # < 10 min => critical
        assert w.tool_id == 'T01'

    def test_urgent_interrupts_active_job(self, node):
        _make_active_job(node, 'j1', 'cnc1', 'RUNNING')
        ms = node.maintenance_scheduler
        w = ms.schedule_maintenance('cnc1', 'TOOL_CHANGE', urgency_minutes=5)
        # Job should be paused
        assert node._active_jobs['j1'].status == 'PAUSED'
        assert w.status == 'IN_PROGRESS'

    def test_not_urgent_does_not_interrupt(self, node):
        _make_active_job(node, 'j1', 'cnc1', 'RUNNING')
        ms = node.maintenance_scheduler
        ms.schedule_maintenance('cnc1', 'TOOL_CHANGE', urgency_minutes=30)
        assert node._active_jobs['j1'].status == 'RUNNING'


class TestRULTriggeredAutoScheduling:
    """Tests for automatic tool change scheduling based on RUL."""

    def test_rul_below_threshold_triggers_tool_change(self, scheduler):
        result = scheduler.check_tool_rul('cnc1', 'T01', 20.0)
        assert result is not None
        assert result.maintenance_type == 'TOOL_CHANGE'
        assert result.tool_id == 'T01'
        assert result.triggered_by == 'prediction'

    def test_rul_above_threshold_no_action(self, scheduler):
        result = scheduler.check_tool_rul('cnc1', 'T01', 60.0)
        assert result is None

    def test_rul_at_threshold_no_action(self, scheduler):
        result = scheduler.check_tool_rul('cnc1', 'T01', 30.0)
        assert result is None  # threshold is < 30, not <=

    def test_rul_critical_gets_priority_1(self, scheduler):
        result = scheduler.check_tool_rul('cnc1', 'T01', 5.0)
        assert result is not None
        assert result.priority == 1

    def test_rul_low_but_not_critical_gets_priority_2(self, scheduler):
        result = scheduler.check_tool_rul('cnc1', 'T01', 20.0)
        assert result is not None
        assert result.priority == 2

    def test_rul_records_wear_rate(self, scheduler):
        scheduler.check_tool_rul('cnc1', 'T01', 50.0)
        scheduler.check_tool_rul('cnc1', 'T01', 40.0)
        assert 'cnc1' in scheduler._wear_rates
        assert scheduler._wear_rates['cnc1']['T01'] == 10.0


class TestCalibrationDriftTrigger:
    """Tests for automatic calibration scheduling based on drift."""

    def test_drift_above_threshold_triggers(self, scheduler):
        result = scheduler.check_calibration_drift('cnc1', 10.0)
        assert result is not None
        assert result.maintenance_type == 'CALIBRATION'
        assert result.triggered_by == 'prediction'

    def test_drift_below_threshold_no_action(self, scheduler):
        result = scheduler.check_calibration_drift('cnc1', 5.0)
        assert result is None

    def test_drift_at_threshold_no_action(self, scheduler):
        result = scheduler.check_calibration_drift('cnc1', 8.0)
        assert result is None  # threshold is > 8%, not >=

    def test_high_drift_gets_higher_priority(self, scheduler):
        result = scheduler.check_calibration_drift('cnc1', 20.0)
        assert result is not None
        assert result.priority == 2  # > 15% drift


class TestPriorityOrderingInQueue:
    """Tests for queue ordering by priority then time."""

    def test_higher_priority_first(self, scheduler):
        scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=120, priority=5)
        scheduler.schedule_maintenance('cnc1', 'TOOL_CHANGE', urgency_minutes=30, priority=1, tool_id='T01')
        q = scheduler.get_maintenance_queue()
        assert len(q) == 2
        assert q[0].priority == 1
        assert q[1].priority == 5

    def test_same_priority_ordered_by_time(self, scheduler):
        w1 = scheduler.schedule_maintenance('cnc1', 'TOOL_CHANGE', urgency_minutes=60, priority=3, tool_id='T01')
        w2 = scheduler.schedule_maintenance('cnc2', 'CALIBRATION', urgency_minutes=30, priority=3)
        q = scheduler.get_maintenance_queue()
        assert q[0].scheduled_start <= q[1].scheduled_start

    def test_queue_filter_by_machine(self, scheduler):
        scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        scheduler.schedule_maintenance('cnc2', 'CALIBRATION', urgency_minutes=30)
        q = scheduler.get_maintenance_queue(machine_id='cnc1')
        assert len(q) == 1
        assert q[0].machine_id == 'cnc1'


class TestJobInterruptionForCriticalMaintenance:
    """Tests for job interruption when maintenance is critically urgent."""

    def test_interrupt_pauses_running_job(self, node):
        _make_active_job(node, 'j1', 'cnc1', 'RUNNING')
        ms = node.maintenance_scheduler
        w = ms.schedule_maintenance('cnc1', 'TOOL_CHANGE', urgency_minutes=5, tool_id='T01')
        assert node._active_jobs['j1'].status == 'PAUSED'
        assert w.status == 'IN_PROGRESS'

    def test_interrupt_pauses_assigned_job(self, node):
        _make_active_job(node, 'j1', 'cnc1', 'ASSIGNED')
        ms = node.maintenance_scheduler
        w = ms.schedule_maintenance('cnc1', 'CALIBRATION', urgency_minutes=3)
        assert node._active_jobs['j1'].status == 'PAUSED'

    def test_no_interrupt_above_threshold(self, node):
        _make_active_job(node, 'j1', 'cnc1', 'RUNNING')
        ms = node.maintenance_scheduler
        ms.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=15)
        assert node._active_jobs['j1'].status == 'RUNNING'

    def test_no_interrupt_without_active_job(self, node):
        ms = node.maintenance_scheduler
        w = ms.schedule_maintenance('cnc1', 'TOOL_CHANGE', urgency_minutes=5)
        # Should not error; window stays PENDING since no job to interrupt => still marks IN_PROGRESS logic skipped
        # Actually _find_active_job_on_machine returns None, so status stays PENDING
        assert w.status == 'PENDING'


class TestCompleteAndCancelMaintenance:
    """Tests for completing and cancelling maintenance windows."""

    def test_complete_maintenance(self, scheduler):
        w = scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        assert scheduler.complete_maintenance(w.window_id)
        assert w.status == 'COMPLETED'

    def test_complete_removes_dedup_key(self, scheduler):
        w = scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        scheduler.complete_maintenance(w.window_id)
        # Should be able to schedule same type again
        w2 = scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        assert w2.window_id != w.window_id

    def test_complete_nonexistent_returns_false(self, scheduler):
        assert not scheduler.complete_maintenance('nonexistent')

    def test_complete_already_completed_returns_false(self, scheduler):
        w = scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        scheduler.complete_maintenance(w.window_id)
        assert not scheduler.complete_maintenance(w.window_id)

    def test_cancel_maintenance(self, scheduler):
        w = scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        assert scheduler.cancel_maintenance(w.window_id)
        assert w.status == 'CANCELLED'

    def test_cancel_nonexistent_returns_false(self, scheduler):
        assert not scheduler.cancel_maintenance('nonexistent')

    def test_cancel_already_completed_returns_false(self, scheduler):
        w = scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        scheduler.complete_maintenance(w.window_id)
        assert not scheduler.cancel_maintenance(w.window_id)

    def test_cancelled_not_in_queue(self, scheduler):
        w = scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        scheduler.cancel_maintenance(w.window_id)
        q = scheduler.get_maintenance_queue()
        assert len(q) == 0

    def test_complete_records_in_digital_thread(self, node_with_thread):
        ms = node_with_thread.maintenance_scheduler
        w = ms.schedule_maintenance('cnc1', 'TOOL_CHANGE', urgency_minutes=30, tool_id='T01')
        ms.complete_maintenance(w.window_id)
        entries = node_with_thread._digital_thread._entries
        maint_entries = [e for e in entries if e['entry_type'] == 'MAINTENANCE_COMPLETE']
        assert len(maint_entries) == 1
        assert maint_entries[0]['machine_id'] == 'cnc1'
        assert maint_entries[0]['maintenance_type'] == 'TOOL_CHANGE'

    def test_complete_resumes_paused_job(self, node):
        _make_active_job(node, 'j1', 'cnc1', 'RUNNING')
        ms = node.maintenance_scheduler
        w = ms.schedule_maintenance('cnc1', 'TOOL_CHANGE', urgency_minutes=5, tool_id='T01')
        assert node._active_jobs['j1'].status == 'PAUSED'
        ms.complete_maintenance(w.window_id)
        assert node._active_jobs['j1'].status == 'RUNNING'


class TestMaintenanceForecast:
    """Tests for maintenance forecasting."""

    def test_forecast_with_wear_rate(self, scheduler):
        scheduler.check_tool_rul('cnc1', 'T01', 50.0)
        scheduler.check_tool_rul('cnc1', 'T01', 40.0)
        forecasts = scheduler.get_maintenance_forecast(hours_ahead=24)
        assert len(forecasts) >= 1
        tc = [f for f in forecasts if f['maintenance_type'] == 'TOOL_CHANGE']
        assert len(tc) >= 1
        assert tc[0]['machine_id'] == 'cnc1'
        assert tc[0]['tool_id'] == 'T01'

    def test_forecast_with_drift(self, scheduler):
        scheduler.check_calibration_drift('cnc1', 7.0)  # above 70% of threshold
        forecasts = scheduler.get_maintenance_forecast(hours_ahead=24)
        cal = [f for f in forecasts if f['maintenance_type'] == 'CALIBRATION']
        assert len(cal) >= 1

    def test_forecast_empty_no_data(self, scheduler):
        forecasts = scheduler.get_maintenance_forecast()
        assert forecasts == []

    def test_forecast_sorted_by_urgency(self, scheduler):
        scheduler.check_tool_rul('cnc1', 'T01', 50.0)
        scheduler.check_tool_rul('cnc1', 'T01', 10.0)
        scheduler.check_tool_rul('cnc2', 'T02', 100.0)
        scheduler.check_tool_rul('cnc2', 'T02', 60.0)
        forecasts = scheduler.get_maintenance_forecast(hours_ahead=24)
        for i in range(len(forecasts) - 1):
            assert (forecasts[i]['predicted_minutes_until_needed']
                    <= forecasts[i + 1]['predicted_minutes_until_needed'])


class TestMultipleMachines:
    """Tests for maintenance across multiple machines."""

    def test_schedule_different_machines(self, scheduler):
        scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        scheduler.schedule_maintenance('cnc2', 'CALIBRATION', urgency_minutes=30)
        scheduler.schedule_maintenance('cnc3', 'TOOL_CHANGE', urgency_minutes=45, tool_id='T01')
        q = scheduler.get_maintenance_queue()
        assert len(q) == 3
        machines = {w.machine_id for w in q}
        assert machines == {'cnc1', 'cnc2', 'cnc3'}

    def test_filter_queue_by_machine(self, scheduler):
        scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        scheduler.schedule_maintenance('cnc2', 'CALIBRATION', urgency_minutes=30)
        q1 = scheduler.get_maintenance_queue(machine_id='cnc1')
        q2 = scheduler.get_maintenance_queue(machine_id='cnc2')
        assert len(q1) == 1
        assert len(q2) == 1
        assert q1[0].machine_id == 'cnc1'
        assert q2[0].machine_id == 'cnc2'

    def test_rul_triggers_per_machine(self, scheduler):
        r1 = scheduler.check_tool_rul('cnc1', 'T01', 10.0)
        r2 = scheduler.check_tool_rul('cnc2', 'T02', 15.0)
        assert r1 is not None
        assert r2 is not None
        assert r1.machine_id == 'cnc1'
        assert r2.machine_id == 'cnc2'


class TestEdgeCases:
    """Tests for edge cases."""

    def test_empty_queue(self, scheduler):
        q = scheduler.get_maintenance_queue()
        assert q == []

    def test_duplicate_scheduling_prevention(self, scheduler):
        w1 = scheduler.schedule_maintenance('cnc1', 'TOOL_CHANGE', urgency_minutes=30, tool_id='T01')
        w2 = scheduler.schedule_maintenance('cnc1', 'TOOL_CHANGE', urgency_minutes=20, tool_id='T01')
        # Should return the existing window, not create a new one
        assert w1.window_id == w2.window_id
        q = scheduler.get_maintenance_queue()
        tool_changes = [w for w in q if w.maintenance_type == 'TOOL_CHANGE' and w.tool_id == 'T01']
        assert len(tool_changes) == 1

    def test_different_tools_not_deduplicated(self, scheduler):
        w1 = scheduler.schedule_maintenance('cnc1', 'TOOL_CHANGE', urgency_minutes=30, tool_id='T01')
        w2 = scheduler.schedule_maintenance('cnc1', 'TOOL_CHANGE', urgency_minutes=30, tool_id='T02')
        assert w1.window_id != w2.window_id

    def test_different_types_not_deduplicated(self, scheduler):
        w1 = scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        w2 = scheduler.schedule_maintenance('cnc1', 'CALIBRATION', urgency_minutes=60)
        assert w1.window_id != w2.window_id

    def test_cancel_then_reschedule(self, scheduler):
        w1 = scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        scheduler.cancel_maintenance(w1.window_id)
        w2 = scheduler.schedule_maintenance('cnc1', 'PREVENTIVE', urgency_minutes=60)
        assert w2.window_id != w1.window_id

    def test_rul_zero_clamped_to_one_minute(self, scheduler):
        result = scheduler.check_tool_rul('cnc1', 'T01', 0.5)
        assert result is not None
        # urgency_minutes = max(rul, 1.0) = 1.0
        assert result.priority == 1

    def test_scheduler_on_node(self, node):
        assert node.maintenance_scheduler is not None
        assert isinstance(node.maintenance_scheduler, MaintenanceScheduler)
        assert node.maintenance_scheduler._node is node


class TestCheckMaintenanceTriggers:
    """Tests for the _check_maintenance_triggers integration."""

    def test_triggers_from_machine_state_rul(self, node):
        state = _StubMsg()
        state.machine_id = 'cnc1'
        state.status = 'RUNNING'
        state.tool_id = 'T01'
        state.tool_rul_minutes = 15.0
        node._machine_states['cnc1'] = state

        node._check_maintenance_triggers()

        q = node.maintenance_scheduler.get_maintenance_queue()
        # may be PENDING or IN_PROGRESS depending on active jobs
        all_windows = list(node.maintenance_scheduler._windows.values())
        tool_changes = [w for w in all_windows if w.maintenance_type == 'TOOL_CHANGE']
        assert len(tool_changes) >= 1

    def test_triggers_from_calibration_drift(self, node):
        state = _StubMsg()
        state.machine_id = 'cnc2'
        state.status = 'IDLE'
        state.calibration_drift_pct = 12.0
        node._machine_states['cnc2'] = state

        node._check_maintenance_triggers()

        all_windows = list(node.maintenance_scheduler._windows.values())
        cals = [w for w in all_windows if w.maintenance_type == 'CALIBRATION']
        assert len(cals) >= 1

    def test_no_triggers_when_values_ok(self, node):
        state = _StubMsg()
        state.machine_id = 'cnc1'
        state.status = 'RUNNING'
        state.tool_id = 'T01'
        state.tool_rul_minutes = 60.0
        state.calibration_drift_pct = 2.0
        node._machine_states['cnc1'] = state

        node._check_maintenance_triggers()

        q = node.maintenance_scheduler.get_maintenance_queue()
        assert len(q) == 0
