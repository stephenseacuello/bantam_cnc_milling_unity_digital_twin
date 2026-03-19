"""Tests for ShiftHandoffManager.

Validates shift handoff management including:
- Shift creation and retrieval
- Handoff item creation, validation, and resolution
- Handoff report generation with open items
- Report acknowledgement and item status propagation
- Open-item filtering by machine
- Handoff history retrieval
- Edge cases (unknown IDs, invalid categories, duplicate items)
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
    HandoffItem,
    HandoffReport,
    ShiftHandoffManager,
    ShiftInfo,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_shift(
    shift_id: str = 'shift_day',
    shift_name: str = 'Day Shift',
    start_time: float = 0.0,
    end_time: float = 28800.0,
    supervisor: str = 'Alice',
    operators: Optional[List[str]] = None,
) -> ShiftInfo:
    return ShiftInfo(
        shift_id=shift_id,
        shift_name=shift_name,
        start_time=start_time,
        end_time=end_time,
        supervisor=supervisor,
        operators=operators or ['op1', 'op2'],
    )


def _make_item(
    item_id: str = 'item_1',
    category: str = 'job_in_progress',
    description: str = 'Roughing pass 60% complete',
    priority: int = 3,
    machine_id: str = 'cnc1',
    status: str = 'open',
) -> HandoffItem:
    return HandoffItem(
        item_id=item_id,
        category=category,
        description=description,
        priority=priority,
        machine_id=machine_id,
        status=status,
    )


# ===================================================================
# Tests: Shift creation and retrieval
# ===================================================================

class TestShiftManagement:
    def setup_method(self):
        self.mgr = ShiftHandoffManager()

    def test_create_and_get_shift(self):
        shift = _make_shift()
        self.mgr.create_shift(shift)
        retrieved = self.mgr.get_shift('shift_day')
        assert retrieved is not None
        assert retrieved.shift_id == 'shift_day'
        assert retrieved.shift_name == 'Day Shift'
        assert retrieved.supervisor == 'Alice'
        assert retrieved.operators == ['op1', 'op2']

    def test_get_nonexistent_shift_returns_none(self):
        assert self.mgr.get_shift('does_not_exist') is None


# ===================================================================
# Tests: Handoff item management
# ===================================================================

class TestHandoffItems:
    def setup_method(self):
        self.mgr = ShiftHandoffManager()

    def test_add_and_retrieve_item(self):
        item = _make_item()
        self.mgr.add_handoff_item(item)
        open_items = self.mgr.get_open_items('cnc1')
        assert len(open_items) == 1
        assert open_items[0].item_id == 'item_1'
        assert open_items[0].status == 'open'

    def test_invalid_category_raises(self):
        item = _make_item(category='invalid_category')
        with pytest.raises(ValueError, match='Invalid category'):
            self.mgr.add_handoff_item(item)

    def test_invalid_priority_raises(self):
        item = _make_item(priority=0)
        with pytest.raises(ValueError, match='Priority must be between'):
            self.mgr.add_handoff_item(item)

        item_high = _make_item(item_id='item_high', priority=6)
        with pytest.raises(ValueError, match='Priority must be between'):
            self.mgr.add_handoff_item(item_high)

    def test_resolve_item(self):
        item = _make_item()
        self.mgr.add_handoff_item(item)
        resolved = self.mgr.resolve_item('item_1')
        assert resolved.status == 'resolved'
        # Should no longer appear in open items
        assert self.mgr.get_open_items('cnc1') == []

    def test_resolve_nonexistent_item_raises(self):
        with pytest.raises(KeyError, match='not found'):
            self.mgr.resolve_item('nonexistent')

    def test_get_open_items_filtered_by_machine(self):
        self.mgr.add_handoff_item(_make_item(item_id='a1', machine_id='cnc1'))
        self.mgr.add_handoff_item(_make_item(item_id='a2', machine_id='cnc2'))
        self.mgr.add_handoff_item(_make_item(item_id='a3', machine_id='cnc1'))
        cnc1_items = self.mgr.get_open_items('cnc1')
        assert len(cnc1_items) == 2
        assert all(i.machine_id == 'cnc1' for i in cnc1_items)

    def test_open_items_sorted_by_priority(self):
        self.mgr.add_handoff_item(_make_item(item_id='low', priority=5))
        self.mgr.add_handoff_item(_make_item(item_id='high', priority=1))
        self.mgr.add_handoff_item(_make_item(item_id='mid', priority=3))
        items = self.mgr.get_open_items('cnc1')
        priorities = [i.priority for i in items]
        assert priorities == [1, 3, 5]

    def test_all_valid_categories_accepted(self):
        categories = [
            'job_in_progress',
            'pending_issue',
            'safety_note',
            'quality_alert',
            'maintenance_needed',
        ]
        for i, cat in enumerate(categories):
            self.mgr.add_handoff_item(_make_item(item_id=f'c{i}', category=cat))
        assert len(self.mgr.get_open_items()) == len(categories)


# ===================================================================
# Tests: Handoff report generation
# ===================================================================

class TestHandoffReport:
    def setup_method(self):
        self.mgr = ShiftHandoffManager()
        self.mgr.create_shift(_make_shift('day', 'Day Shift'))
        self.mgr.create_shift(_make_shift('night', 'Night Shift',
                                          supervisor='Bob'))

    def test_generate_report_with_items(self):
        self.mgr.add_handoff_item(_make_item(item_id='i1', priority=2))
        self.mgr.add_handoff_item(_make_item(item_id='i2', priority=1))
        report = self.mgr.generate_handoff_report('day', 'night',
                                                   notes='Check coolant level')
        assert report.from_shift.shift_id == 'day'
        assert report.to_shift.shift_id == 'night'
        assert len(report.items) == 2
        assert report.notes == 'Check coolant level'
        assert report.created_at > 0
        assert report.acknowledged_by == ''

    def test_generate_report_unknown_shift_raises(self):
        with pytest.raises(KeyError, match='not found'):
            self.mgr.generate_handoff_report('day', 'nonexistent')
        with pytest.raises(KeyError, match='not found'):
            self.mgr.generate_handoff_report('nonexistent', 'night')

    def test_generate_report_excludes_resolved_items(self):
        self.mgr.add_handoff_item(_make_item(item_id='open_item'))
        self.mgr.add_handoff_item(_make_item(item_id='resolved_item'))
        self.mgr.resolve_item('resolved_item')
        report = self.mgr.generate_handoff_report('day', 'night')
        assert len(report.items) == 1
        assert report.items[0].item_id == 'open_item'


# ===================================================================
# Tests: Report acknowledgement
# ===================================================================

class TestAcknowledgeHandoff:
    def setup_method(self):
        self.mgr = ShiftHandoffManager()
        self.mgr.create_shift(_make_shift('day', 'Day Shift'))
        self.mgr.create_shift(_make_shift('night', 'Night Shift'))

    def test_acknowledge_sets_operator_and_item_status(self):
        self.mgr.add_handoff_item(_make_item(item_id='i1'))
        self.mgr.add_handoff_item(_make_item(item_id='i2'))
        report = self.mgr.generate_handoff_report('day', 'night')

        ack_report = self.mgr.acknowledge_handoff(report.report_id, 'Bob')
        assert ack_report.acknowledged_by == 'Bob'
        # All previously-open items should now be acknowledged
        for item in ack_report.items:
            assert item.status == 'acknowledged'
        # Master item store should also reflect the status change
        open_items = self.mgr.get_open_items()
        for item in open_items:
            assert item.status == 'acknowledged'

    def test_acknowledge_unknown_report_raises(self):
        with pytest.raises(KeyError, match='not found'):
            self.mgr.acknowledge_handoff('no_such_report', 'Bob')


# ===================================================================
# Tests: Handoff history
# ===================================================================

class TestHandoffHistory:
    def setup_method(self):
        self.mgr = ShiftHandoffManager()
        self.mgr.create_shift(_make_shift('day', 'Day'))
        self.mgr.create_shift(_make_shift('night', 'Night'))

    def test_history_returns_newest_first(self):
        r1 = self.mgr.generate_handoff_report('day', 'night')
        r2 = self.mgr.generate_handoff_report('night', 'day')
        history = self.mgr.get_handoff_history(last_n=10)
        assert len(history) == 2
        assert history[0].report_id == r2.report_id
        assert history[1].report_id == r1.report_id

    def test_history_limits_results(self):
        for _ in range(5):
            self.mgr.generate_handoff_report('day', 'night')
        history = self.mgr.get_handoff_history(last_n=3)
        assert len(history) == 3

    def test_empty_history(self):
        history = self.mgr.get_handoff_history()
        assert history == []
