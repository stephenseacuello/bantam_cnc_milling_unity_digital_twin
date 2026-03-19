"""Tests for ToolCribManager, ToolCribItem, and ToolTransaction.

Validates tool crib inventory management including:
- Adding tools to inventory
- Checking out tools to operators/machines
- Checking in tools with condition updates
- Sending tools for regrinding
- Scrapping damaged tools
- Reorder alert generation
- Transaction history tracking
- Machine tool lookup
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
    ToolCribItem,
    ToolCribManager,
    ToolTransaction,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_item(
    tool_id: str = 'T001',
    tool_type: str = 'end_mill',
    description: str = '1/2" 4-flute carbide end mill',
    quantity: int = 5,
    min_quantity: int = 2,
    location: str = 'crib',
    condition: str = 'new',
) -> ToolCribItem:
    return ToolCribItem(
        tool_id=tool_id,
        tool_type=tool_type,
        description=description,
        quantity=quantity,
        min_quantity=min_quantity,
        location=location,
        condition=condition,
    )


def _make_manager_with_tools() -> ToolCribManager:
    """Return a manager pre-loaded with a few tools."""
    mgr = ToolCribManager()
    mgr.add_tool(_make_item('T001', quantity=5, min_quantity=2))
    mgr.add_tool(_make_item('T002', tool_type='drill', description='6mm HSS drill',
                             quantity=3, min_quantity=1))
    mgr.add_tool(_make_item('T003', tool_type='face_mill', description='3" face mill',
                             quantity=1, min_quantity=1))
    return mgr


# ===================================================================
# Tests: ToolCribItem dataclass
# ===================================================================

class TestToolCribItem:
    def test_dataclass_fields(self):
        item = _make_item()
        assert item.tool_id == 'T001'
        assert item.tool_type == 'end_mill'
        assert item.description == '1/2" 4-flute carbide end mill'
        assert item.quantity == 5
        assert item.min_quantity == 2
        assert item.location == 'crib'
        assert item.condition == 'new'
        assert item.last_checked_out is None
        assert item.checked_out_by == ''

    def test_default_values(self):
        item = ToolCribItem(tool_id='X', tool_type='t', description='d',
                            quantity=1, min_quantity=0)
        assert item.location == 'crib'
        assert item.condition == 'new'
        assert item.last_checked_out is None
        assert item.checked_out_by == ''


# ===================================================================
# Tests: Add / Get / Get All
# ===================================================================

class TestInventoryManagement:
    def test_add_and_get_tool(self):
        mgr = ToolCribManager()
        item = _make_item('T010')
        mgr.add_tool(item)
        fetched = mgr.get_tool('T010')
        assert fetched is not None
        assert fetched.tool_id == 'T010'

    def test_get_nonexistent_tool_returns_none(self):
        mgr = ToolCribManager()
        assert mgr.get_tool('NONEXISTENT') is None

    def test_get_all_tools(self):
        mgr = _make_manager_with_tools()
        tools = mgr.get_all_tools()
        assert len(tools) == 3
        ids = {t.tool_id for t in tools}
        assert ids == {'T001', 'T002', 'T003'}

    def test_add_tool_records_receive_transaction(self):
        mgr = ToolCribManager()
        mgr.add_tool(_make_item('T050'))
        history = mgr.get_tool_history('T050')
        assert len(history) == 1
        assert history[0].action == 'receive'


# ===================================================================
# Tests: Check-Out
# ===================================================================

class TestCheckOut:
    def test_successful_checkout(self):
        mgr = _make_manager_with_tools()
        result = mgr.check_out('T001', 'operator_A', 'cnc1')
        assert result is True
        tool = mgr.get_tool('T001')
        assert tool.location == 'machine'
        assert tool.quantity == 4
        assert tool.checked_out_by == 'operator_A'
        assert tool.last_checked_out is not None

    def test_checkout_nonexistent_tool(self):
        mgr = ToolCribManager()
        assert mgr.check_out('NOPE', 'op', 'cnc1') is False

    def test_checkout_tool_not_in_crib(self):
        mgr = _make_manager_with_tools()
        mgr.check_out('T001', 'op', 'cnc1')  # moves to machine
        # Second checkout should fail — location is now 'machine'
        assert mgr.check_out('T001', 'op2', 'cnc2') is False

    def test_checkout_creates_transaction(self):
        mgr = _make_manager_with_tools()
        mgr.check_out('T002', 'op_B', 'cnc3')
        history = mgr.get_tool_history('T002')
        checkout_txs = [tx for tx in history if tx.action == 'check_out']
        assert len(checkout_txs) == 1
        assert checkout_txs[0].operator == 'op_B'
        assert checkout_txs[0].machine_id == 'cnc3'


# ===================================================================
# Tests: Check-In
# ===================================================================

class TestCheckIn:
    def test_successful_checkin(self):
        mgr = _make_manager_with_tools()
        mgr.check_out('T001', 'op', 'cnc1')
        result = mgr.check_in('T001', 'op', 'used')
        assert result is True
        tool = mgr.get_tool('T001')
        assert tool.location == 'crib'
        assert tool.condition == 'used'
        assert tool.quantity == 5  # restored
        assert tool.checked_out_by == ''

    def test_checkin_nonexistent_tool(self):
        mgr = ToolCribManager()
        assert mgr.check_in('NOPE', 'op', 'used') is False

    def test_checkin_updates_condition(self):
        mgr = _make_manager_with_tools()
        mgr.check_out('T002', 'op', 'cnc1')
        mgr.check_in('T002', 'op', 'worn')
        tool = mgr.get_tool('T002')
        assert tool.condition == 'worn'


# ===================================================================
# Tests: Regrind
# ===================================================================

class TestRegrind:
    def test_send_to_regrind(self):
        mgr = _make_manager_with_tools()
        result = mgr.send_to_regrind('T001')
        assert result is True
        tool = mgr.get_tool('T001')
        assert tool.location == 'regrind'
        assert tool.quantity == 4

    def test_regrind_nonexistent(self):
        mgr = ToolCribManager()
        assert mgr.send_to_regrind('NOPE') is False

    def test_regrind_tool_not_in_crib(self):
        mgr = _make_manager_with_tools()
        mgr.check_out('T001', 'op', 'cnc1')
        assert mgr.send_to_regrind('T001') is False


# ===================================================================
# Tests: Scrap
# ===================================================================

class TestScrap:
    def test_scrap_tool(self):
        mgr = _make_manager_with_tools()
        result = mgr.scrap_tool('T003', 'Chipped cutting edge')
        assert result is True
        tool = mgr.get_tool('T003')
        assert tool.location == 'scrap'
        assert tool.condition == 'damaged'
        assert tool.quantity == 0

    def test_scrap_nonexistent(self):
        mgr = ToolCribManager()
        assert mgr.scrap_tool('NOPE', 'reason') is False

    def test_scrap_records_reason(self):
        mgr = _make_manager_with_tools()
        mgr.scrap_tool('T002', 'Broken flute')
        history = mgr.get_tool_history('T002')
        scrap_txs = [tx for tx in history if tx.action == 'scrap']
        assert len(scrap_txs) == 1
        assert 'Broken flute' in scrap_txs[0].notes


# ===================================================================
# Tests: Reorder Alerts
# ===================================================================

class TestReorderAlerts:
    def test_no_alerts_above_minimum(self):
        mgr = _make_manager_with_tools()
        alerts = mgr.get_reorder_alerts()
        # T003 has quantity=1, min_quantity=1 -> at minimum -> alert
        alert_ids = {a.tool_id for a in alerts}
        assert 'T001' not in alert_ids  # 5 > 2
        assert 'T002' not in alert_ids  # 3 > 1

    def test_alert_at_minimum(self):
        mgr = _make_manager_with_tools()
        alerts = mgr.get_reorder_alerts()
        alert_ids = {a.tool_id for a in alerts}
        assert 'T003' in alert_ids  # quantity == min_quantity

    def test_alert_below_minimum_after_checkout(self):
        mgr = _make_manager_with_tools()
        # T002 qty=3, min=1 -- checkout reduces to 2
        mgr.check_out('T002', 'op', 'cnc1')
        mgr.check_in('T002', 'op', 'used')
        # Scrap to drive below
        mgr.scrap_tool('T002', 'broken')
        mgr.scrap_tool('T002', 'broken2')
        alerts = mgr.get_reorder_alerts()
        alert_ids = {a.tool_id for a in alerts}
        assert 'T002' in alert_ids


# ===================================================================
# Tests: Tool History
# ===================================================================

class TestToolHistory:
    def test_full_lifecycle_history(self):
        mgr = ToolCribManager()
        mgr.add_tool(_make_item('T100', quantity=5, min_quantity=1))
        mgr.check_out('T100', 'op_A', 'cnc1')
        mgr.check_in('T100', 'op_A', 'used')
        mgr.send_to_regrind('T100')

        history = mgr.get_tool_history('T100')
        actions = [tx.action for tx in history]
        assert actions == ['receive', 'check_out', 'check_in', 'regrind']

    def test_history_empty_for_unknown_tool(self):
        mgr = ToolCribManager()
        assert mgr.get_tool_history('UNKNOWN') == []

    def test_transactions_have_unique_ids(self):
        mgr = _make_manager_with_tools()
        mgr.check_out('T001', 'op', 'cnc1')
        mgr.check_in('T001', 'op', 'used')
        history = mgr.get_tool_history('T001')
        tx_ids = [tx.transaction_id for tx in history]
        assert len(tx_ids) == len(set(tx_ids))


# ===================================================================
# Tests: Tools on Machine
# ===================================================================

class TestToolsOnMachine:
    def test_tools_on_machine_after_checkout(self):
        mgr = _make_manager_with_tools()
        mgr.check_out('T001', 'op', 'cnc1')
        on_machine = mgr.get_tools_on_machine('cnc1')
        assert len(on_machine) == 1
        assert on_machine[0].tool_id == 'T001'

    def test_tools_on_machine_empty_after_checkin(self):
        mgr = _make_manager_with_tools()
        mgr.check_out('T001', 'op', 'cnc1')
        mgr.check_in('T001', 'op', 'used')
        on_machine = mgr.get_tools_on_machine('cnc1')
        assert len(on_machine) == 0

    def test_tools_on_different_machines(self):
        mgr = _make_manager_with_tools()
        mgr.check_out('T001', 'op_A', 'cnc1')
        mgr.check_out('T002', 'op_B', 'cnc2')
        assert len(mgr.get_tools_on_machine('cnc1')) == 1
        assert len(mgr.get_tools_on_machine('cnc2')) == 1
        assert mgr.get_tools_on_machine('cnc1')[0].tool_id == 'T001'
        assert mgr.get_tools_on_machine('cnc2')[0].tool_id == 'T002'

    def test_no_tools_on_unused_machine(self):
        mgr = _make_manager_with_tools()
        assert mgr.get_tools_on_machine('cnc99') == []
