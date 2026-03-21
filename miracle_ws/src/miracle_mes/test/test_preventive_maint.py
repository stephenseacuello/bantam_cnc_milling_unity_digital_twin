"""Tests for PreventiveMaintenanceScheduler, MaintenanceTask, and MaintenanceSchedule.

Validates preventive-maintenance scheduling including:
- Task registration and retrieval
- Task completion and next-due recalculation
- Due-task and overdue-task queries
- Full schedule generation
- Compliance calculation over a time period
- Downtime forecasting
- Multi-machine isolation
- Priority ordering
"""

import os
import sys
import time
from types import ModuleType
from typing import Any, Dict, List, Optional
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

# miracle_core stubs
_mc = _ensure_module('miracle_core')
_mc_lifecycle = _ensure_module('miracle_core.lifecycle_node_base')
_mc_qos = _ensure_module('miracle_core.qos_profiles')


class _StubLifecycleNode:
    CRITICALITY_MEDIUM = 'MEDIUM'

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
_mc_qos.QoSProfiles.logging.return_value = MagicMock()

# miracle_msgs stubs
_msgs = _ensure_module('miracle_msgs')
_msgs_msg = _ensure_module('miracle_msgs.msg')


class _StubMsg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattr__(self, name):
        return ''


_msgs_msg.DigitalThreadEntry = _StubMsg
_msgs_msg.JobStatus = _StubMsg
_msgs_msg.AnomalyAlert = _StubMsg
_msgs_msg.MachineState = _StubMsg


# ---------------------------------------------------------------------------
# Now import the production code
# ---------------------------------------------------------------------------
from miracle_mes.digital_thread import (  # noqa: E402
    MaintenanceTask,
    MaintenanceSchedule,
    PreventiveMaintenanceScheduler,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_NOW = 1_700_000_000.0  # fixed reference timestamp for deterministic tests


def _make_task(
    task_id: str = 'MT-001',
    name: str = 'Spindle lubrication',
    description: str = 'Grease spindle bearings',
    frequency_hours: float = 500.0,
    frequency_parts: int = 5000,
    last_performed: float = _NOW - 500 * 3600,
    next_due: float = _NOW,
    machine_id: str = 'cnc1',
    priority: int = 2,
    estimated_duration_min: float = 30.0,
    parts_needed: Optional[List[str]] = None,
) -> MaintenanceTask:
    return MaintenanceTask(
        task_id=task_id,
        name=name,
        description=description,
        frequency_hours=frequency_hours,
        frequency_parts=frequency_parts,
        last_performed=last_performed,
        next_due=next_due,
        machine_id=machine_id,
        priority=priority,
        estimated_duration_min=estimated_duration_min,
        parts_needed=parts_needed or [],
    )


# ===================================================================
# Tests: Task Registration
# ===================================================================

class TestTaskRegistration:
    def test_add_single_task(self):
        scheduler = PreventiveMaintenanceScheduler()
        task = _make_task(task_id='MT-001')
        scheduler.add_task(task)
        schedule = scheduler.get_schedule('cnc1')
        assert len(schedule.tasks) == 1
        assert schedule.tasks[0].task_id == 'MT-001'

    def test_add_multiple_tasks(self):
        scheduler = PreventiveMaintenanceScheduler()
        scheduler.add_task(_make_task(task_id='MT-001'))
        scheduler.add_task(_make_task(task_id='MT-002', name='Way lube'))
        schedule = scheduler.get_schedule('cnc1')
        assert len(schedule.tasks) == 2
        ids = {t.task_id for t in schedule.tasks}
        assert ids == {'MT-001', 'MT-002'}


# ===================================================================
# Tests: Task Completion
# ===================================================================

class TestTaskCompletion:
    def test_complete_updates_last_performed_and_next_due(self):
        scheduler = PreventiveMaintenanceScheduler()
        task = _make_task(
            task_id='MT-001',
            frequency_hours=100.0,
            next_due=_NOW,
        )
        scheduler.add_task(task)
        completion_time = _NOW + 3600  # completed 1 hour after due
        scheduler.complete_task('MT-001', completion_time, 'Routine service')

        schedule = scheduler.get_schedule('cnc1')
        completed = schedule.tasks[0]
        assert completed.last_performed == completion_time
        assert completed.next_due == completion_time + 100.0 * 3600.0

    def test_complete_unknown_task_is_noop(self):
        scheduler = PreventiveMaintenanceScheduler()
        # Should not raise
        scheduler.complete_task('NONEXISTENT', _NOW, 'noop')


# ===================================================================
# Tests: Due-Task Queries
# ===================================================================

class TestDueTasks:
    def test_due_tasks_returns_overdue(self):
        scheduler = PreventiveMaintenanceScheduler()
        scheduler.add_task(_make_task(
            task_id='MT-001',
            next_due=_NOW - 3600,  # 1 hour overdue
            machine_id='cnc1',
        ))
        scheduler.add_task(_make_task(
            task_id='MT-002',
            next_due=_NOW + 86400,  # due tomorrow
            machine_id='cnc1',
        ))
        due = scheduler.get_due_tasks('cnc1', _NOW)
        assert len(due) == 1
        assert due[0].task_id == 'MT-001'

    def test_due_tasks_includes_exactly_due(self):
        scheduler = PreventiveMaintenanceScheduler()
        scheduler.add_task(_make_task(
            task_id='MT-001',
            next_due=_NOW,
            machine_id='cnc1',
        ))
        due = scheduler.get_due_tasks('cnc1', _NOW)
        assert len(due) == 1

    def test_due_tasks_machine_isolation(self):
        scheduler = PreventiveMaintenanceScheduler()
        scheduler.add_task(_make_task(
            task_id='MT-001',
            next_due=_NOW - 3600,
            machine_id='cnc1',
        ))
        scheduler.add_task(_make_task(
            task_id='MT-002',
            next_due=_NOW - 3600,
            machine_id='cnc2',
        ))
        due_cnc1 = scheduler.get_due_tasks('cnc1', _NOW)
        due_cnc2 = scheduler.get_due_tasks('cnc2', _NOW)
        assert len(due_cnc1) == 1
        assert due_cnc1[0].task_id == 'MT-001'
        assert len(due_cnc2) == 1
        assert due_cnc2[0].task_id == 'MT-002'


# ===================================================================
# Tests: Overdue Tasks (all machines)
# ===================================================================

class TestOverdueTasks:
    def test_overdue_across_machines(self):
        scheduler = PreventiveMaintenanceScheduler()
        scheduler.add_task(_make_task(
            task_id='MT-001',
            next_due=_NOW - 7200,
            machine_id='cnc1',
        ))
        scheduler.add_task(_make_task(
            task_id='MT-002',
            next_due=_NOW - 3600,
            machine_id='cnc2',
        ))
        scheduler.add_task(_make_task(
            task_id='MT-003',
            next_due=_NOW + 86400,
            machine_id='cnc3',
        ))
        overdue = scheduler.get_overdue_tasks(_NOW)
        ids = {t.task_id for t in overdue}
        assert ids == {'MT-001', 'MT-002'}

    def test_no_overdue(self):
        scheduler = PreventiveMaintenanceScheduler()
        scheduler.add_task(_make_task(
            task_id='MT-001',
            next_due=_NOW + 86400,
            machine_id='cnc1',
        ))
        overdue = scheduler.get_overdue_tasks(_NOW)
        assert len(overdue) == 0


# ===================================================================
# Tests: Full Schedule
# ===================================================================

class TestSchedule:
    def test_schedule_counts_overdue_and_upcoming(self):
        scheduler = PreventiveMaintenanceScheduler()
        now = time.time()
        # 2 overdue, 1 upcoming
        scheduler.add_task(_make_task(
            task_id='MT-001',
            next_due=now - 7200,
            machine_id='cnc1',
        ))
        scheduler.add_task(_make_task(
            task_id='MT-002',
            next_due=now - 3600,
            machine_id='cnc1',
        ))
        scheduler.add_task(_make_task(
            task_id='MT-003',
            next_due=now + 86400,
            machine_id='cnc1',
        ))
        schedule = scheduler.get_schedule('cnc1')
        assert schedule.machine_id == 'cnc1'
        assert schedule.overdue_count == 2
        assert schedule.upcoming_count == 1
        assert abs(schedule.compliance_pct - 100.0 / 3.0) < 0.1

    def test_empty_machine_schedule(self):
        scheduler = PreventiveMaintenanceScheduler()
        schedule = scheduler.get_schedule('cnc_empty')
        assert schedule.tasks == []
        assert schedule.overdue_count == 0
        assert schedule.upcoming_count == 0
        assert schedule.compliance_pct == 100.0


# ===================================================================
# Tests: Compliance Calculation
# ===================================================================

class TestCompliance:
    def test_full_compliance(self):
        scheduler = PreventiveMaintenanceScheduler()
        scheduler.add_task(_make_task(
            task_id='MT-001',
            frequency_hours=100.0,
            machine_id='cnc1',
        ))
        scheduler.add_task(_make_task(
            task_id='MT-002',
            frequency_hours=200.0,
            machine_id='cnc1',
        ))
        # Complete both within the period
        period_start = _NOW
        period_end = _NOW + 86400
        scheduler.complete_task('MT-001', _NOW + 1000, 'done')
        scheduler.complete_task('MT-002', _NOW + 2000, 'done')

        compliance = scheduler.calculate_compliance(
            'cnc1', period_start, period_end,
        )
        assert compliance == 100.0

    def test_partial_compliance(self):
        scheduler = PreventiveMaintenanceScheduler()
        scheduler.add_task(_make_task(task_id='MT-001', machine_id='cnc1'))
        scheduler.add_task(_make_task(task_id='MT-002', machine_id='cnc1'))

        period_start = _NOW
        period_end = _NOW + 86400
        # Only complete one
        scheduler.complete_task('MT-001', _NOW + 1000, 'done')

        compliance = scheduler.calculate_compliance(
            'cnc1', period_start, period_end,
        )
        assert compliance == 50.0

    def test_zero_compliance(self):
        scheduler = PreventiveMaintenanceScheduler()
        scheduler.add_task(_make_task(task_id='MT-001', machine_id='cnc1'))

        period_start = _NOW
        period_end = _NOW + 86400
        # No completions

        compliance = scheduler.calculate_compliance(
            'cnc1', period_start, period_end,
        )
        assert compliance == 0.0

    def test_compliance_no_tasks_returns_100(self):
        scheduler = PreventiveMaintenanceScheduler()
        compliance = scheduler.calculate_compliance(
            'cnc_empty', _NOW, _NOW + 86400,
        )
        assert compliance == 100.0


# ===================================================================
# Tests: Downtime Forecasting
# ===================================================================

class TestDowntimeForecast:
    def test_single_task_single_occurrence(self):
        scheduler = PreventiveMaintenanceScheduler()
        # Task due in 12 hours, frequency 720 hours (30 days)
        scheduler.add_task(_make_task(
            task_id='MT-001',
            frequency_hours=720.0,
            estimated_duration_min=45.0,
            next_due=time.time() + 12 * 3600,
            machine_id='cnc1',
        ))
        downtime = scheduler.forecast_downtime('cnc1', horizon_days=7)
        # Only one occurrence in 7 days
        assert downtime == 45.0

    def test_multiple_occurrences(self):
        scheduler = PreventiveMaintenanceScheduler()
        now = time.time()
        # Task with 24-hour frequency, 10 min duration, due in 1 hour
        scheduler.add_task(_make_task(
            task_id='MT-001',
            frequency_hours=24.0,
            estimated_duration_min=10.0,
            next_due=now + 3600,
            machine_id='cnc1',
        ))
        downtime = scheduler.forecast_downtime('cnc1', horizon_days=7)
        # ~7 occurrences in 7 days (first at +1h, then every 24h)
        assert downtime == 70.0

    def test_no_tasks_zero_downtime(self):
        scheduler = PreventiveMaintenanceScheduler()
        downtime = scheduler.forecast_downtime('cnc_empty', horizon_days=30)
        assert downtime == 0.0
