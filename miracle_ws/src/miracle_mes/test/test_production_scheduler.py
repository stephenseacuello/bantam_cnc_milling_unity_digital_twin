"""Tests for ProductionOrderScheduler.

Validates production order scheduling including:
- Order add / remove / get
- Status updates
- Earliest-due-date-first scheduling with priority tiebreaker
- Late order detection
- Per-machine schedule queries
- Production summary computation
- Multi-machine load balancing
- Edge cases (empty orders, single machine, etc.)
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

# miracle_core stubs -- force-set attributes on existing modules
_mc = _ensure_module('miracle_core')
_mc_lifecycle = _ensure_module('miracle_core.lifecycle_node_base')
_mc_qos = _ensure_module('miracle_core.qos_profiles')


class _StubLifecycleNode:
    CRITICALITY_HIGH = 'HIGH'
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
_mc_qos.QoSProfiles.command.return_value = MagicMock()
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
    ProductionOrder,
    ProductionOrderScheduler,
    ScheduleEntry,
    ScheduleResult,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_order(
    order_id: str = 'ORD-001',
    part_number: str = 'PN-100',
    quantity: int = 10,
    due_date: float = 0.0,
    priority: int = 3,
    estimated_time_per_part_min: float = 5.0,
    required_operations: Optional[List[str]] = None,
    material_type: str = 'aluminum',
    customer: str = 'ACME',
    status: str = 'pending',
) -> ProductionOrder:
    return ProductionOrder(
        order_id=order_id,
        part_number=part_number,
        quantity=quantity,
        due_date=due_date if due_date else time.time() + 3600,
        priority=priority,
        estimated_time_per_part_min=estimated_time_per_part_min,
        required_operations=required_operations or ['milling'],
        material_type=material_type,
        customer=customer,
        status=status,
    )


# ===================================================================
# Tests: Order management (add / remove / get)
# ===================================================================

class TestOrderManagement:
    def setup_method(self):
        self.sched = ProductionOrderScheduler()

    def test_add_and_get_order(self):
        order = _make_order('ORD-001')
        self.sched.add_order(order)
        retrieved = self.sched.get_order('ORD-001')
        assert retrieved is not None
        assert retrieved.order_id == 'ORD-001'
        assert retrieved.part_number == 'PN-100'

    def test_get_nonexistent_order_returns_none(self):
        assert self.sched.get_order('MISSING') is None

    def test_remove_order(self):
        order = _make_order('ORD-001')
        self.sched.add_order(order)
        assert self.sched.remove_order('ORD-001') is True
        assert self.sched.get_order('ORD-001') is None

    def test_remove_nonexistent_order(self):
        assert self.sched.remove_order('MISSING') is False


# ===================================================================
# Tests: Status updates
# ===================================================================

class TestStatusUpdate:
    def setup_method(self):
        self.sched = ProductionOrderScheduler()
        self.sched.add_order(_make_order('ORD-001'))

    def test_update_status_valid(self):
        assert self.sched.update_order_status('ORD-001', 'in_progress') is True
        assert self.sched.get_order('ORD-001').status == 'in_progress'

    def test_update_status_invalid_status(self):
        assert self.sched.update_order_status('ORD-001', 'INVALID') is False
        assert self.sched.get_order('ORD-001').status == 'pending'

    def test_update_status_nonexistent_order(self):
        assert self.sched.update_order_status('MISSING', 'completed') is False


# ===================================================================
# Tests: Scheduling
# ===================================================================

class TestSchedule:
    def setup_method(self):
        self.sched = ProductionOrderScheduler()
        self.now = time.time()

    def test_schedule_single_order_single_machine(self):
        order = _make_order('ORD-001', quantity=10, estimated_time_per_part_min=2.0,
                            due_date=self.now + 7200)
        self.sched.add_order(order)
        result = self.sched.schedule(['cnc1'], self.now)

        assert isinstance(result, ScheduleResult)
        assert len(result.entries) == 1
        assert result.entries[0].order_id == 'ORD-001'
        assert result.entries[0].machine_id == 'cnc1'
        assert result.entries[0].quantity == 10
        # 10 parts * 2 min/part = 20 min = 1200 sec
        expected_end = self.now + 1200.0
        assert abs(result.entries[0].end_time - expected_end) < 0.01
        assert result.unscheduled_orders == []
        assert result.total_makespan_min > 0

    def test_earliest_due_date_first(self):
        """Orders with earlier due dates should be scheduled first."""
        o1 = _make_order('ORD-LATE', due_date=self.now + 7200, priority=3)
        o2 = _make_order('ORD-EARLY', due_date=self.now + 1800, priority=3)
        self.sched.add_order(o1)
        self.sched.add_order(o2)
        result = self.sched.schedule(['cnc1'], self.now)

        assert result.entries[0].order_id == 'ORD-EARLY'
        assert result.entries[1].order_id == 'ORD-LATE'

    def test_priority_tiebreaker(self):
        """When due dates are equal, lower priority value wins."""
        o1 = _make_order('ORD-LOW', due_date=self.now + 3600, priority=5)
        o2 = _make_order('ORD-HIGH', due_date=self.now + 3600, priority=1)
        self.sched.add_order(o1)
        self.sched.add_order(o2)
        result = self.sched.schedule(['cnc1'], self.now)

        assert result.entries[0].order_id == 'ORD-HIGH'
        assert result.entries[1].order_id == 'ORD-LOW'

    def test_multi_machine_load_balancing(self):
        """Orders should be distributed across available machines."""
        for i in range(4):
            self.sched.add_order(_make_order(
                f'ORD-{i}',
                quantity=5,
                estimated_time_per_part_min=2.0,
                due_date=self.now + 3600 + i,  # slightly staggered
            ))
        result = self.sched.schedule(['cnc1', 'cnc2'], self.now)

        assert len(result.entries) == 4
        machines_used = {e.machine_id for e in result.entries}
        assert len(machines_used) == 2

    def test_empty_orders(self):
        result = self.sched.schedule(['cnc1', 'cnc2'], self.now)
        assert result.entries == []
        assert result.total_makespan_min == 0.0

    def test_no_machines(self):
        self.sched.add_order(_make_order('ORD-001'))
        result = self.sched.schedule([], self.now)
        assert result.entries == []
        assert 'ORD-001' in result.unscheduled_orders

    def test_on_time_percentage(self):
        """Orders finishing before due date contribute to on-time %."""
        # Order with impossibly tight due date (due in 60s, takes 1000 min)
        o1 = _make_order('ORD-LATE', quantity=100,
                          estimated_time_per_part_min=10.0,
                          due_date=self.now + 60)
        # Order with very generous due date (due in 24h, takes 1 min total)
        # Even though it runs after ORD-LATE (~1000 min), it still finishes
        # well within 24h from now.
        o2 = _make_order('ORD-ONTIME', quantity=1,
                          estimated_time_per_part_min=1.0,
                          due_date=self.now + 86400)
        self.sched.add_order(o1)
        self.sched.add_order(o2)
        result = self.sched.schedule(['cnc1'], self.now)

        # One of two orders is on time -> 50%
        assert result.on_time_pct == 50.0

    def test_machine_utilization(self):
        """Machine utilization should reflect busy time vs makespan."""
        o1 = _make_order('ORD-001', quantity=10,
                          estimated_time_per_part_min=1.0,
                          due_date=self.now + 7200)
        self.sched.add_order(o1)
        result = self.sched.schedule(['cnc1', 'cnc2'], self.now)

        # cnc1 should have work, cnc2 should be idle
        assert result.machine_utilization['cnc1'] > 0
        assert result.machine_utilization['cnc2'] == 0.0

    def test_scheduled_status_set(self):
        """After scheduling, pending orders become 'scheduled'."""
        self.sched.add_order(_make_order('ORD-001'))
        self.sched.schedule(['cnc1'], self.now)
        assert self.sched.get_order('ORD-001').status == 'scheduled'

    def test_only_pending_orders_are_scheduled(self):
        """Non-pending orders should be skipped."""
        o1 = _make_order('ORD-DONE', status='completed')
        o2 = _make_order('ORD-TODO', status='pending')
        self.sched.add_order(o1)
        self.sched.add_order(o2)
        result = self.sched.schedule(['cnc1'], self.now)
        assert len(result.entries) == 1
        assert result.entries[0].order_id == 'ORD-TODO'


# ===================================================================
# Tests: Late orders
# ===================================================================

class TestLateOrders:
    def setup_method(self):
        self.sched = ProductionOrderScheduler()
        self.now = time.time()

    def test_no_late_orders(self):
        self.sched.add_order(_make_order('ORD-001', due_date=self.now + 3600))
        late = self.sched.get_late_orders(self.now)
        assert late == []

    def test_late_pending_order(self):
        self.sched.add_order(_make_order('ORD-LATE', due_date=self.now - 3600))
        late = self.sched.get_late_orders(self.now)
        assert len(late) == 1
        assert late[0].order_id == 'ORD-LATE'

    def test_completed_order_not_late(self):
        self.sched.add_order(_make_order('ORD-DONE', due_date=self.now - 3600,
                                          status='completed'))
        late = self.sched.get_late_orders(self.now)
        assert late == []


# ===================================================================
# Tests: Per-machine schedule
# ===================================================================

class TestMachineSchedule:
    def setup_method(self):
        self.sched = ProductionOrderScheduler()
        self.now = time.time()

    def test_get_schedule_for_machine(self):
        for i in range(4):
            self.sched.add_order(_make_order(
                f'ORD-{i}', quantity=5,
                estimated_time_per_part_min=1.0,
                due_date=self.now + 3600 + i,
            ))
        self.sched.schedule(['cnc1', 'cnc2'], self.now)

        cnc1_entries = self.sched.get_schedule_for_machine('cnc1')
        cnc2_entries = self.sched.get_schedule_for_machine('cnc2')
        assert all(e.machine_id == 'cnc1' for e in cnc1_entries)
        assert all(e.machine_id == 'cnc2' for e in cnc2_entries)
        assert len(cnc1_entries) + len(cnc2_entries) == 4

    def test_empty_machine_schedule(self):
        entries = self.sched.get_schedule_for_machine('cnc_nonexistent')
        assert entries == []


# ===================================================================
# Tests: Production summary
# ===================================================================

class TestProductionSummary:
    def setup_method(self):
        self.sched = ProductionOrderScheduler()

    def test_summary_empty(self):
        summary = self.sched.get_production_summary()
        assert summary['total_orders'] == 0
        assert summary['by_status'] == {}
        assert summary['avg_lead_time_min'] == 0.0

    def test_summary_counts(self):
        self.sched.add_order(_make_order('ORD-1', status='pending'))
        self.sched.add_order(_make_order('ORD-2', status='pending'))
        self.sched.add_order(_make_order('ORD-3', status='completed'))
        summary = self.sched.get_production_summary()
        assert summary['total_orders'] == 3
        assert summary['by_status']['pending'] == 2
        assert summary['by_status']['completed'] == 1

    def test_summary_avg_lead_time(self):
        # 10 parts * 5 min/part = 50 min each
        self.sched.add_order(_make_order('ORD-1', quantity=10,
                                          estimated_time_per_part_min=5.0))
        self.sched.add_order(_make_order('ORD-2', quantity=20,
                                          estimated_time_per_part_min=5.0))
        summary = self.sched.get_production_summary()
        # avg = (50 + 100) / 2 = 75
        assert summary['avg_lead_time_min'] == 75.0


# ===================================================================
# Tests: Remove order clears schedule entries
# ===================================================================

class TestRemoveOrderClearsSchedule:
    def test_remove_clears_entries(self):
        sched = ProductionOrderScheduler()
        now = time.time()
        sched.add_order(_make_order('ORD-001', due_date=now + 3600))
        sched.add_order(_make_order('ORD-002', due_date=now + 7200))
        sched.schedule(['cnc1'], now)

        assert len(sched.get_schedule_for_machine('cnc1')) == 2
        sched.remove_order('ORD-001')
        remaining = sched.get_schedule_for_machine('cnc1')
        assert len(remaining) == 1
        assert remaining[0].order_id == 'ORD-002'
