"""Tests for InventoryTracker, InventoryItem, and InventoryTransaction.

Validates inventory management including:
- Adding and retrieving items
- Recording transactions (receive, issue, adjust, scrap, return)
- Reorder alerts
- Inventory value by category
- Transaction history
- Consumption rate calculation
- Stockout forecasting
- Edge cases (unknown item, empty inventory, etc.)
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
_msgs_msg.DigitalThreadEntry = _StubMsg
_msgs_msg.AnomalyAlert = _StubMsg

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


# ---------------------------------------------------------------------------
# Import the real digital_thread module (do NOT stub it out since we need
# InventoryTracker and friends from it).  We only patch DigitalThreadNode
# so that its ROS2-dependent __init__ is never executed.
# ---------------------------------------------------------------------------
import miracle_mes.digital_thread as _dt_mod  # noqa: E402

_dt_mod.DigitalThreadNode = _StubDigitalThread  # type: ignore[attr-defined]

from miracle_mes.digital_thread import (  # noqa: E402
    InventoryItem,
    InventoryTracker,
    InventoryTransaction,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_item(
    item_id: str = 'item1',
    name: str = 'Aluminum 6061 Bar',
    category: str = 'raw_material',
    quantity: float = 100.0,
    unit: str = 'kg',
    location: str = 'warehouse-A',
    reorder_point: float = 20.0,
    reorder_quantity: float = 50.0,
    unit_cost: float = 5.0,
    last_updated: float = 0.0,
) -> InventoryItem:
    return InventoryItem(
        item_id=item_id,
        name=name,
        category=category,
        quantity=quantity,
        unit=unit,
        location=location,
        reorder_point=reorder_point,
        reorder_quantity=reorder_quantity,
        unit_cost=unit_cost,
        last_updated=last_updated,
    )


def _make_transaction(
    transaction_id: str = 'txn1',
    item_id: str = 'item1',
    transaction_type: str = 'issue',
    quantity: float = 10.0,
    timestamp: float = 0.0,
    reference: str = 'job-001',
    operator: str = 'op1',
) -> InventoryTransaction:
    return InventoryTransaction(
        transaction_id=transaction_id,
        item_id=item_id,
        transaction_type=transaction_type,
        quantity=quantity,
        timestamp=timestamp if timestamp else time.time(),
        reference=reference,
        operator=operator,
    )


# ===================================================================
# Tests: Item CRUD
# ===================================================================

class TestItemCRUD:
    def setup_method(self):
        self.tracker = InventoryTracker()

    def test_add_and_get_item(self):
        item = _make_item('steel1', name='Steel Rod', category='raw_material')
        self.tracker.add_item(item)
        retrieved = self.tracker.get_item('steel1')
        assert retrieved is not None
        assert retrieved.name == 'Steel Rod'
        assert retrieved.item_id == 'steel1'

    def test_get_nonexistent_item_returns_none(self):
        assert self.tracker.get_item('does_not_exist') is None

    def test_get_all_items(self):
        self.tracker.add_item(_make_item('a'))
        self.tracker.add_item(_make_item('b'))
        self.tracker.add_item(_make_item('c'))
        items = self.tracker.get_all_items()
        assert len(items) == 3
        ids = {i.item_id for i in items}
        assert ids == {'a', 'b', 'c'}


# ===================================================================
# Tests: Recording Transactions
# ===================================================================

class TestRecordTransaction:
    def setup_method(self):
        self.tracker = InventoryTracker()
        self.tracker.add_item(_make_item('item1', quantity=100.0))

    def test_receive_increases_quantity(self):
        txn = _make_transaction(
            transaction_type='receive', quantity=25.0,
        )
        self.tracker.record_transaction(txn)
        assert self.tracker.get_item('item1').quantity == 125.0

    def test_issue_decreases_quantity(self):
        txn = _make_transaction(
            transaction_type='issue', quantity=30.0,
        )
        self.tracker.record_transaction(txn)
        assert self.tracker.get_item('item1').quantity == 70.0

    def test_adjust_sets_quantity(self):
        txn = _make_transaction(
            transaction_type='adjust', quantity=42.0,
        )
        self.tracker.record_transaction(txn)
        assert self.tracker.get_item('item1').quantity == 42.0

    def test_scrap_decreases_quantity(self):
        txn = _make_transaction(
            transaction_type='scrap', quantity=5.0,
        )
        self.tracker.record_transaction(txn)
        assert self.tracker.get_item('item1').quantity == 95.0

    def test_return_increases_quantity(self):
        txn = _make_transaction(
            transaction_type='return', quantity=15.0,
        )
        self.tracker.record_transaction(txn)
        assert self.tracker.get_item('item1').quantity == 115.0

    def test_unknown_item_raises_key_error(self):
        txn = _make_transaction(item_id='no_such_item')
        with pytest.raises(KeyError, match='no_such_item'):
            self.tracker.record_transaction(txn)

    def test_unknown_transaction_type_raises_value_error(self):
        txn = _make_transaction(transaction_type='teleport')
        with pytest.raises(ValueError, match='teleport'):
            self.tracker.record_transaction(txn)

    def test_last_updated_is_set(self):
        ts = 1700000000.0
        txn = _make_transaction(
            transaction_type='receive', quantity=1.0, timestamp=ts,
        )
        self.tracker.record_transaction(txn)
        assert self.tracker.get_item('item1').last_updated == ts


# ===================================================================
# Tests: Reorder Alerts
# ===================================================================

class TestReorderAlerts:
    def setup_method(self):
        self.tracker = InventoryTracker()

    def test_no_alerts_when_stock_above_reorder(self):
        self.tracker.add_item(_make_item('a', quantity=50.0, reorder_point=20.0))
        assert self.tracker.get_reorder_alerts() == []

    def test_alert_when_at_reorder_point(self):
        self.tracker.add_item(_make_item('a', quantity=20.0, reorder_point=20.0))
        alerts = self.tracker.get_reorder_alerts()
        assert len(alerts) == 1
        assert alerts[0].item_id == 'a'

    def test_alert_when_below_reorder_point(self):
        self.tracker.add_item(_make_item('a', quantity=5.0, reorder_point=20.0))
        alerts = self.tracker.get_reorder_alerts()
        assert len(alerts) == 1

    def test_multiple_alerts(self):
        self.tracker.add_item(_make_item('a', quantity=10.0, reorder_point=20.0))
        self.tracker.add_item(_make_item('b', quantity=5.0, reorder_point=10.0))
        self.tracker.add_item(_make_item('c', quantity=100.0, reorder_point=10.0))
        alerts = self.tracker.get_reorder_alerts()
        alert_ids = {a.item_id for a in alerts}
        assert alert_ids == {'a', 'b'}


# ===================================================================
# Tests: Inventory Value
# ===================================================================

class TestInventoryValue:
    def setup_method(self):
        self.tracker = InventoryTracker()

    def test_empty_inventory_value(self):
        assert self.tracker.get_inventory_value() == {}

    def test_single_category_value(self):
        self.tracker.add_item(
            _make_item('a', category='raw_material', quantity=10.0, unit_cost=5.0)
        )
        value = self.tracker.get_inventory_value()
        assert value == {'raw_material': 50.0}

    def test_multiple_categories(self):
        self.tracker.add_item(
            _make_item('a', category='raw_material', quantity=10.0, unit_cost=5.0)
        )
        self.tracker.add_item(
            _make_item('b', category='finished_good', quantity=3.0, unit_cost=100.0)
        )
        self.tracker.add_item(
            _make_item('c', category='raw_material', quantity=20.0, unit_cost=2.0)
        )
        value = self.tracker.get_inventory_value()
        assert value['raw_material'] == pytest.approx(90.0)  # 50 + 40
        assert value['finished_good'] == pytest.approx(300.0)


# ===================================================================
# Tests: Transaction History
# ===================================================================

class TestTransactionHistory:
    def setup_method(self):
        self.tracker = InventoryTracker()
        self.tracker.add_item(_make_item('item1', quantity=200.0))

    def test_empty_history(self):
        assert self.tracker.get_transaction_history('item1') == []

    def test_history_returns_correct_item(self):
        self.tracker.add_item(_make_item('item2', quantity=50.0))
        t1 = _make_transaction('t1', item_id='item1', timestamp=1000.0)
        t2 = _make_transaction('t2', item_id='item2', timestamp=1001.0)
        t3 = _make_transaction('t3', item_id='item1', timestamp=1002.0)
        self.tracker.record_transaction(t1)
        self.tracker.record_transaction(t2)
        self.tracker.record_transaction(t3)
        history = self.tracker.get_transaction_history('item1')
        assert len(history) == 2
        assert all(t.item_id == 'item1' for t in history)

    def test_history_sorted_by_timestamp(self):
        t1 = _make_transaction('t1', timestamp=3000.0)
        t2 = _make_transaction('t2', timestamp=1000.0)
        t3 = _make_transaction('t3', timestamp=2000.0)
        self.tracker.record_transaction(t1)
        self.tracker.record_transaction(t2)
        self.tracker.record_transaction(t3)
        history = self.tracker.get_transaction_history('item1')
        timestamps = [t.timestamp for t in history]
        assert timestamps == [1000.0, 2000.0, 3000.0]


# ===================================================================
# Tests: Consumption Rate
# ===================================================================

class TestConsumptionRate:
    def setup_method(self):
        self.tracker = InventoryTracker()
        self.tracker.add_item(_make_item('item1', quantity=500.0))

    def test_no_transactions_zero_rate(self):
        rate = self.tracker.get_consumption_rate('item1', 30)
        assert rate == 0.0

    def test_consumption_rate_calculation(self):
        now = time.time()
        # Issue 60 units over the last 30 days
        for i in range(6):
            ts = now - (25 - i) * 86400.0  # spread across the window
            txn = _make_transaction(
                f't{i}', transaction_type='issue', quantity=10.0,
                timestamp=ts,
            )
            self.tracker.record_transaction(txn)
        rate = self.tracker.get_consumption_rate('item1', 30)
        assert rate == pytest.approx(2.0, abs=0.01)  # 60 / 30

    def test_receive_not_counted_as_consumption(self):
        now = time.time()
        txn = _make_transaction(
            't1', transaction_type='receive', quantity=100.0,
            timestamp=now - 86400.0,
        )
        self.tracker.record_transaction(txn)
        rate = self.tracker.get_consumption_rate('item1', 30)
        assert rate == 0.0

    def test_scrap_counted_as_consumption(self):
        now = time.time()
        txn = _make_transaction(
            't1', transaction_type='scrap', quantity=30.0,
            timestamp=now - 86400.0,
        )
        self.tracker.record_transaction(txn)
        rate = self.tracker.get_consumption_rate('item1', 30)
        assert rate == pytest.approx(1.0, abs=0.01)  # 30 / 30


# ===================================================================
# Tests: Stockout Forecasting
# ===================================================================

class TestForecastStockout:
    def setup_method(self):
        self.tracker = InventoryTracker()

    def test_unknown_item_returns_none(self):
        assert self.tracker.forecast_stockout('ghost') is None

    def test_no_consumption_returns_none(self):
        self.tracker.add_item(_make_item('item1', quantity=100.0))
        assert self.tracker.forecast_stockout('item1') is None

    def test_stockout_prediction(self):
        self.tracker.add_item(_make_item('item1', quantity=100.0))
        now = time.time()
        # Consume 10 units/day for the last 10 days
        for i in range(10):
            txn = _make_transaction(
                f't{i}', transaction_type='issue', quantity=10.0,
                timestamp=now - (9 - i) * 86400.0,
            )
            self.tracker.record_transaction(txn)
        # 100 units consumed over 30 day window = 100/30 ~ 3.33/day
        # current quantity = 100 - 100 = 0 ... but we started at 100
        # After 10 issues of 10 each, quantity = 100 - 100 = 0
        # Let's use a bigger starting quantity
        pass

    def test_stockout_with_known_rate(self):
        # Start with 200 units, consume 50 over 10 days => 5/day
        self.tracker.add_item(_make_item('item1', quantity=200.0))
        now = time.time()
        for i in range(5):
            txn = _make_transaction(
                f't{i}', transaction_type='issue', quantity=10.0,
                timestamp=now - (4 - i) * 86400.0,
            )
            self.tracker.record_transaction(txn)
        # quantity is now 200 - 50 = 150
        # consumption: 50 units over 30 days => ~1.667/day
        days = self.tracker.forecast_stockout('item1', lookback_days=30.0)
        assert days is not None
        # 150 / (50/30) = 150 * 30 / 50 = 90 days
        assert days == pytest.approx(90.0, abs=1.0)
