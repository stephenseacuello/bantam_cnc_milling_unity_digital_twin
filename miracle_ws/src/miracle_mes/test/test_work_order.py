"""Tests for WorkOrderTracker, WorkOrder, WorkOrderStage, and WorkOrderMetrics.

Validates work order lifecycle management including:
- Work order creation with stages
- Stage transitions (start, complete, skip)
- Auto-start of next stage on completion
- Filtering by overall status
- Metrics computation (cycle time, on-time delivery, bottleneck)
- Edge cases (empty tracker, missing stages, single stage)
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
    WorkOrder,
    WorkOrderMetrics,
    WorkOrderStage,
    WorkOrderTracker,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_STAGES = ['cutting', 'deburring', 'inspection', 'packaging']


def _make_tracker_with_order(
    wo_id: str = 'WO-001',
    part_number: str = 'PN-100',
    quantity: int = 10,
    stage_names: Optional[List[str]] = None,
    priority: int = 1,
    customer: str = 'Acme Corp',
    due_date: float = 0.0,
) -> WorkOrderTracker:
    """Return a tracker pre-loaded with a single work order."""
    tracker = WorkOrderTracker()
    tracker.create_work_order(
        wo_id=wo_id,
        part_number=part_number,
        quantity=quantity,
        stage_names=stage_names or list(_STAGES),
        priority=priority,
        customer=customer,
        due_date=due_date,
    )
    return tracker


# ===================================================================
# Tests: WorkOrderStage dataclass
# ===================================================================

class TestWorkOrderStage:
    def test_default_values(self):
        stage = WorkOrderStage(stage_name='cutting')
        assert stage.stage_name == 'cutting'
        assert stage.status == 'pending'
        assert stage.start_time is None
        assert stage.end_time is None
        assert stage.operator == ''
        assert stage.notes == ''

    def test_custom_values(self):
        stage = WorkOrderStage(
            stage_name='inspection',
            status='completed',
            start_time=100.0,
            end_time=200.0,
            operator='John',
            notes='Passed QC',
        )
        assert stage.status == 'completed'
        assert stage.end_time - stage.start_time == 100.0
        assert stage.operator == 'John'


# ===================================================================
# Tests: Work Order Creation
# ===================================================================

class TestCreateWorkOrder:
    def test_create_basic_order(self):
        tracker = WorkOrderTracker()
        wo = tracker.create_work_order(
            wo_id='WO-001',
            part_number='PN-100',
            quantity=10,
            stage_names=['cutting', 'inspection'],
            priority=2,
            customer='Acme Corp',
            due_date=time.time() + 86400,
        )
        assert wo.wo_id == 'WO-001'
        assert wo.part_number == 'PN-100'
        assert wo.quantity == 10
        assert len(wo.stages) == 2
        assert all(s.status == 'pending' for s in wo.stages)
        assert wo.priority == 2
        assert wo.customer == 'Acme Corp'
        assert wo.created_at > 0

    def test_stages_initialised_pending(self):
        tracker = _make_tracker_with_order()
        wo = tracker.get_work_order('WO-001')
        assert len(wo.stages) == 4
        for stage in wo.stages:
            assert stage.status == 'pending'
            assert stage.start_time is None
            assert stage.end_time is None

    def test_retrieve_created_order(self):
        tracker = _make_tracker_with_order()
        wo = tracker.get_work_order('WO-001')
        assert wo.wo_id == 'WO-001'

    def test_get_all_orders(self):
        tracker = WorkOrderTracker()
        tracker.create_work_order('WO-1', 'PN-1', 5, ['cutting'])
        tracker.create_work_order('WO-2', 'PN-2', 8, ['cutting', 'inspection'])
        orders = tracker.get_all_orders()
        assert len(orders) == 2
        ids = {wo.wo_id for wo in orders}
        assert ids == {'WO-1', 'WO-2'}


# ===================================================================
# Tests: Stage Transitions
# ===================================================================

class TestStageTransitions:
    def test_start_stage(self):
        tracker = _make_tracker_with_order()
        tracker.start_stage('WO-001', 'cutting', 'Alice')
        wo = tracker.get_work_order('WO-001')
        cutting = wo.stages[0]
        assert cutting.status == 'in_progress'
        assert cutting.operator == 'Alice'
        assert cutting.start_time is not None

    def test_complete_stage(self):
        tracker = _make_tracker_with_order()
        tracker.start_stage('WO-001', 'cutting', 'Alice')
        tracker.complete_stage('WO-001', 'cutting', 'Clean cut')
        wo = tracker.get_work_order('WO-001')
        cutting = wo.stages[0]
        assert cutting.status == 'completed'
        assert cutting.end_time is not None
        assert cutting.notes == 'Clean cut'

    def test_complete_auto_starts_next_stage(self):
        tracker = _make_tracker_with_order()
        tracker.start_stage('WO-001', 'cutting', 'Alice')
        tracker.complete_stage('WO-001', 'cutting', 'Done')
        wo = tracker.get_work_order('WO-001')
        deburring = wo.stages[1]
        assert deburring.status == 'in_progress'
        assert deburring.start_time is not None

    def test_skip_stage(self):
        tracker = _make_tracker_with_order()
        tracker.skip_stage('WO-001', 'deburring', 'Not required for this material')
        wo = tracker.get_work_order('WO-001')
        deburring = wo.stages[1]
        assert deburring.status == 'skipped'
        assert deburring.notes == 'Not required for this material'
        assert deburring.end_time is not None

    def test_start_nonexistent_stage_raises(self):
        tracker = _make_tracker_with_order()
        with pytest.raises(ValueError, match="Stage 'welding' not found"):
            tracker.start_stage('WO-001', 'welding', 'Bob')

    def test_complete_nonexistent_stage_raises(self):
        tracker = _make_tracker_with_order()
        with pytest.raises(ValueError, match="Stage 'welding' not found"):
            tracker.complete_stage('WO-001', 'welding')

    def test_skip_nonexistent_stage_raises(self):
        tracker = _make_tracker_with_order()
        with pytest.raises(ValueError, match="Stage 'welding' not found"):
            tracker.skip_stage('WO-001', 'welding')

    def test_complete_last_stage_no_auto_start_error(self):
        tracker = WorkOrderTracker()
        tracker.create_work_order('WO-1', 'PN-1', 1, ['only_stage'])
        tracker.start_stage('WO-1', 'only_stage', 'Bob')
        # Should not raise even though there is no next stage
        tracker.complete_stage('WO-1', 'only_stage', 'All done')
        wo = tracker.get_work_order('WO-1')
        assert wo.stages[0].status == 'completed'

    def test_auto_start_skips_already_completed_stages(self):
        """Auto-start should find the first *pending* stage, skipping completed/skipped."""
        tracker = _make_tracker_with_order()
        # Skip deburring (stage index 1) before completing cutting (stage index 0)
        tracker.skip_stage('WO-001', 'deburring', 'skip')
        tracker.start_stage('WO-001', 'cutting', 'Alice')
        tracker.complete_stage('WO-001', 'cutting')
        wo = tracker.get_work_order('WO-001')
        # deburring was already skipped, so inspection (index 2) should auto-start
        assert wo.stages[1].status == 'skipped'
        assert wo.stages[2].status == 'in_progress'


# ===================================================================
# Tests: Status Filtering
# ===================================================================

class TestGetOrdersByStatus:
    def test_all_pending(self):
        tracker = _make_tracker_with_order('WO-1')
        result = tracker.get_orders_by_status('pending')
        assert len(result) == 1
        assert result[0].wo_id == 'WO-1'

    def test_in_progress(self):
        tracker = _make_tracker_with_order('WO-1')
        tracker.start_stage('WO-1', 'cutting', 'Alice')
        result = tracker.get_orders_by_status('in_progress')
        assert len(result) == 1

    def test_completed(self):
        tracker = WorkOrderTracker()
        tracker.create_work_order('WO-1', 'PN-1', 1, ['s1'])
        tracker.start_stage('WO-1', 's1', 'Bob')
        tracker.complete_stage('WO-1', 's1')
        result = tracker.get_orders_by_status('completed')
        assert len(result) == 1

    def test_mixed_statuses(self):
        tracker = WorkOrderTracker()
        tracker.create_work_order('WO-1', 'PN-1', 1, ['s1'])
        tracker.create_work_order('WO-2', 'PN-2', 2, ['s1', 's2'])
        tracker.create_work_order('WO-3', 'PN-3', 3, ['s1'])
        # WO-1: completed
        tracker.start_stage('WO-1', 's1', 'Alice')
        tracker.complete_stage('WO-1', 's1')
        # WO-2: in_progress
        tracker.start_stage('WO-2', 's1', 'Bob')
        # WO-3: pending
        assert len(tracker.get_orders_by_status('completed')) == 1
        assert len(tracker.get_orders_by_status('in_progress')) == 1
        assert len(tracker.get_orders_by_status('pending')) == 1

    def test_skipped_stages_count_as_completed(self):
        """An order where all stages are either completed or skipped is 'completed'."""
        tracker = WorkOrderTracker()
        tracker.create_work_order('WO-1', 'PN-1', 1, ['s1', 's2'])
        tracker.start_stage('WO-1', 's1', 'Alice')
        tracker.complete_stage('WO-1', 's1')
        # s2 was auto-started; skip it instead
        tracker.skip_stage('WO-1', 's2', 'not needed')
        result = tracker.get_orders_by_status('completed')
        assert len(result) == 1


# ===================================================================
# Tests: Metrics
# ===================================================================

class TestMetrics:
    def test_empty_tracker_metrics(self):
        tracker = WorkOrderTracker()
        m = tracker.get_metrics()
        assert m.total_orders == 0
        assert m.completed == 0
        assert m.in_progress == 0
        assert m.avg_cycle_time_min == 0.0
        assert m.on_time_delivery_pct == 0.0
        assert m.stage_bottleneck == ''

    def test_total_and_status_counts(self):
        tracker = WorkOrderTracker()
        tracker.create_work_order('WO-1', 'PN-1', 1, ['s1'])
        tracker.create_work_order('WO-2', 'PN-2', 2, ['s1'])
        tracker.create_work_order('WO-3', 'PN-3', 3, ['s1'])
        # Complete WO-1
        tracker.start_stage('WO-1', 's1', 'A')
        tracker.complete_stage('WO-1', 's1')
        # Start WO-2
        tracker.start_stage('WO-2', 's1', 'B')
        m = tracker.get_metrics()
        assert m.total_orders == 3
        assert m.completed == 1
        assert m.in_progress == 1

    def test_on_time_delivery(self):
        tracker = WorkOrderTracker()
        future = time.time() + 86400  # 24h from now
        tracker.create_work_order('WO-1', 'PN-1', 1, ['s1'], due_date=future)
        tracker.start_stage('WO-1', 's1', 'Alice')
        tracker.complete_stage('WO-1', 's1')
        m = tracker.get_metrics()
        assert m.on_time_delivery_pct == 100.0

    def test_late_delivery(self):
        tracker = WorkOrderTracker()
        past = time.time() - 86400  # yesterday
        tracker.create_work_order('WO-1', 'PN-1', 1, ['s1'], due_date=past)
        tracker.start_stage('WO-1', 's1', 'Alice')
        tracker.complete_stage('WO-1', 's1')
        m = tracker.get_metrics()
        assert m.on_time_delivery_pct == 0.0

    def test_bottleneck_identification(self):
        """The stage with the longest average duration should be identified."""
        tracker = WorkOrderTracker()
        # Create two orders with two stages; make 'slow_stage' take longer
        for wo_id in ('WO-1', 'WO-2'):
            tracker.create_work_order(wo_id, 'PN-1', 1, ['fast_stage', 'slow_stage'])
            tracker.start_stage(wo_id, 'fast_stage', 'A')
            # Simulate time passage by directly setting start/end times
            wo = tracker.get_work_order(wo_id)
            wo.stages[0].start_time = 1000.0
            wo.stages[0].end_time = 1010.0  # 10s
            wo.stages[0].status = 'completed'
            wo.stages[1].start_time = 1010.0
            wo.stages[1].end_time = 1310.0  # 300s
            wo.stages[1].status = 'completed'

        m = tracker.get_metrics()
        assert m.stage_bottleneck == 'slow_stage'

    def test_avg_cycle_time(self):
        tracker = WorkOrderTracker()
        tracker.create_work_order('WO-1', 'PN-1', 1, ['s1'])
        wo = tracker.get_work_order('WO-1')
        wo.stages[0].status = 'completed'
        wo.stages[0].start_time = 0.0
        wo.stages[0].end_time = 120.0  # 120 seconds = 2 minutes
        m = tracker.get_metrics()
        assert m.avg_cycle_time_min == pytest.approx(2.0)

    def test_no_due_date_counts_as_on_time(self):
        """Orders without a due date (due_date <= 0) count as on-time."""
        tracker = WorkOrderTracker()
        tracker.create_work_order('WO-1', 'PN-1', 1, ['s1'], due_date=0.0)
        tracker.start_stage('WO-1', 's1', 'Alice')
        tracker.complete_stage('WO-1', 's1')
        m = tracker.get_metrics()
        assert m.on_time_delivery_pct == 100.0
