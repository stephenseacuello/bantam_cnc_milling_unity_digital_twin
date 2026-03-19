"""Tests for CapacityPlanner, CapacitySlot, DemandForecast, and CapacityPlan.

Validates capacity planning against demand forecasts including:
- Machine registration and demand addition
- Capacity plan generation and allocation
- Bottleneck identification
- Overtime calculation
- What-if analysis for adding machines
- Edge cases (no machines, no demand, overloaded capacity)
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
    CapacityPlanner,
    CapacityPlan,
    CapacitySlot,
    DemandForecast,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_demand(
    part_number: str = 'PART-001',
    quantity: int = 100,
    time_per_part_min: float = 6.0,
    due_date: float = 0.0,
    priority: int = 2,
) -> DemandForecast:
    return DemandForecast(
        part_number=part_number,
        quantity=quantity,
        time_per_part_min=time_per_part_min,
        due_date=due_date,
        priority=priority,
    )


# ===================================================================
# Tests: Dataclass Construction
# ===================================================================

class TestDataclasses:
    def test_capacity_slot_fields(self):
        slot = CapacitySlot(
            machine_id='cnc1',
            start_time=0.0,
            end_time=86400.0,
            available_hours=8.0,
            allocated_hours=4.0,
            utilization_pct=50.0,
        )
        assert slot.machine_id == 'cnc1'
        assert slot.available_hours == 8.0
        assert slot.allocated_hours == 4.0
        assert slot.utilization_pct == 50.0

    def test_demand_forecast_defaults(self):
        d = DemandForecast(
            part_number='X',
            quantity=10,
            time_per_part_min=5.0,
            due_date=1000.0,
        )
        assert d.priority == 2  # default

    def test_capacity_plan_defaults(self):
        plan = CapacityPlan()
        assert plan.slots == []
        assert plan.feasible is True
        assert plan.bottleneck_machine == ''
        assert plan.unmet_demand == []


# ===================================================================
# Tests: Machine Registration and Demand Addition
# ===================================================================

class TestRegistration:
    def test_add_machine(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)
        assert 'cnc1' in cp._machines
        assert cp._machines['cnc1'] == 8.0

    def test_add_demand(self):
        cp = CapacityPlanner()
        d = _make_demand()
        cp.add_demand(d)
        assert len(cp._demands) == 1
        assert cp._demands[0].part_number == 'PART-001'


# ===================================================================
# Tests: Plan Generation
# ===================================================================

class TestPlanGeneration:
    def setup_method(self):
        self.cp = CapacityPlanner()
        self.cp.add_machine('cnc1', 8.0)
        self.cp.add_machine('cnc2', 8.0)

    def test_plan_no_demand(self):
        plan = self.cp.plan(planning_horizon_days=5)
        assert plan.feasible is True
        assert plan.total_allocated_hours == 0.0
        assert plan.overall_utilization_pct == 0.0
        assert len(plan.slots) == 2

    def test_plan_feasible_demand(self):
        # 100 parts * 6 min/part = 600 min = 10 hours.
        # Two machines * 8 hrs/day * 5 days = 80 hours total capacity.
        self.cp.add_demand(_make_demand(quantity=100, time_per_part_min=6.0))
        plan = self.cp.plan(planning_horizon_days=5)
        assert plan.feasible is True
        assert plan.total_allocated_hours == 10.0
        assert plan.unmet_demand == []
        assert plan.overall_utilization_pct == pytest.approx(12.5, abs=0.1)

    def test_plan_infeasible_demand(self):
        # Need 200 hours total but only have 80 hours capacity.
        self.cp.add_demand(
            _make_demand(quantity=2000, time_per_part_min=6.0)
        )  # 2000 * 6 / 60 = 200 hours
        plan = self.cp.plan(planning_horizon_days=5)
        assert plan.feasible is False
        assert len(plan.unmet_demand) > 0
        # Total allocated should fill up capacity
        assert plan.total_allocated_hours == pytest.approx(80.0, abs=0.1)

    def test_plan_allocates_to_least_loaded_machine(self):
        # Add two small demands sequentially — first fills cnc1 or cnc2,
        # second should go to the other.
        self.cp.add_demand(_make_demand('P1', quantity=240, time_per_part_min=10.0))
        # 240 * 10 / 60 = 40 hours -> fills one machine for 5 days
        plan = self.cp.plan(planning_horizon_days=5)
        assert plan.feasible is True
        allocated = {s.machine_id: s.allocated_hours for s in plan.slots}
        assert allocated['cnc1'] == 40.0 or allocated['cnc2'] == 40.0

    def test_plan_priority_ordering(self):
        # High priority demand should be allocated before low priority.
        self.cp.add_demand(_make_demand('LOW', quantity=500, time_per_part_min=6.0, priority=3))
        self.cp.add_demand(_make_demand('HIGH', quantity=500, time_per_part_min=6.0, priority=0))
        # Total demand = 1000 * 6 / 60 = 100 hrs, capacity = 80 hrs.
        plan = self.cp.plan(planning_horizon_days=5)
        assert plan.feasible is False
        # The unmet demand should be the LOW priority one.
        unmet_parts = [d.part_number for d in plan.unmet_demand]
        assert 'LOW' in unmet_parts

    def test_plan_no_machines_with_demand(self):
        cp = CapacityPlanner()
        cp.add_demand(_make_demand())
        plan = cp.plan()
        assert plan.feasible is False

    def test_plan_no_machines_no_demand(self):
        cp = CapacityPlanner()
        plan = cp.plan()
        assert plan.feasible is True

    def test_plan_slot_utilization_pct(self):
        self.cp.add_demand(_make_demand(quantity=60, time_per_part_min=10.0))
        # 60 * 10 / 60 = 10 hours -> all on one machine (40 hrs capacity)
        plan = self.cp.plan(planning_horizon_days=5)
        assert any(s.utilization_pct == pytest.approx(25.0, abs=0.1) for s in plan.slots)


# ===================================================================
# Tests: Bottleneck Identification
# ===================================================================

class TestBottleneck:
    def test_identify_bottleneck_single_machine(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)
        cp.add_demand(_make_demand(quantity=100, time_per_part_min=6.0))
        bottleneck = cp.identify_bottleneck()
        assert bottleneck == 'cnc1'

    def test_identify_bottleneck_most_loaded(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)
        cp.add_machine('cnc2', 16.0)
        # 10 hours of demand -> goes to cnc2 first (most remaining).
        # But cnc2 has 80 hrs capacity vs cnc1 40 hrs.
        # Demand goes to cnc2 (most remaining), so cnc2 gets load.
        # We need to create enough demand to differentiate.
        # Add 100 hours total demand across two demands.
        cp.add_demand(_make_demand('P1', quantity=480, time_per_part_min=10.0))
        # 480 * 10/60 = 80 hrs -> cnc2 has 80 hrs capacity, cnc1 has 40 hrs
        # cnc2 gets first 80 hrs (fills up), then we check...
        plan = cp.plan(planning_horizon_days=5)
        assert plan.bottleneck_machine == 'cnc2'

    def test_identify_bottleneck_no_machines(self):
        cp = CapacityPlanner()
        assert cp.identify_bottleneck() == ''


# ===================================================================
# Tests: Overtime Calculation
# ===================================================================

class TestOvertime:
    def test_no_overtime_needed(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)
        cp.add_demand(_make_demand(quantity=10, time_per_part_min=6.0))
        # 10 * 6 / 60 = 1 hour, capacity = 40 hours over 5 days.
        overtime = cp.get_overtime_needed(planning_horizon_days=5)
        assert overtime == 0.0

    def test_overtime_needed(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)
        # Need 50 hours, have 40 hours -> 10 hours overtime.
        cp.add_demand(_make_demand(quantity=500, time_per_part_min=6.0))
        # 500 * 6 / 60 = 50 hours
        overtime = cp.get_overtime_needed(planning_horizon_days=5)
        assert overtime == pytest.approx(10.0, abs=0.1)

    def test_overtime_no_machines(self):
        cp = CapacityPlanner()
        cp.add_demand(_make_demand(quantity=100, time_per_part_min=6.0))
        overtime = cp.get_overtime_needed(planning_horizon_days=5)
        # All demand is overtime since there are no machines.
        assert overtime == pytest.approx(10.0, abs=0.1)

    def test_overtime_no_demand(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)
        overtime = cp.get_overtime_needed(planning_horizon_days=5)
        assert overtime == 0.0


# ===================================================================
# Tests: What-If Analysis
# ===================================================================

class TestWhatIfAddMachine:
    def test_adding_machine_reduces_utilization(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)
        cp.add_demand(_make_demand(quantity=240, time_per_part_min=10.0))
        # 240 * 10 / 60 = 40 hours, capacity = 40 hours -> 100% util.
        plan_before = cp.plan(planning_horizon_days=5)
        util_before = plan_before.overall_utilization_pct

        util_after = cp.what_if_add_machine('cnc2', 8.0)
        # Now 40 hours demand / 80 hours capacity -> 50% util.
        assert util_after < util_before
        assert util_after == pytest.approx(50.0, abs=0.1)

    def test_what_if_does_not_modify_state(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)
        cp.add_demand(_make_demand(quantity=100))
        cp.what_if_add_machine('cnc2', 8.0)
        # cnc2 should NOT be in the machines dict after what-if.
        assert 'cnc2' not in cp._machines

    def test_what_if_with_existing_machine_id(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)
        cp.add_demand(_make_demand(quantity=240, time_per_part_min=10.0))
        # Simulate increasing cnc1 from 8 to 16 hours/day.
        util = cp.what_if_add_machine('cnc1', 16.0)
        # 40 hrs demand / 80 hrs capacity -> 50%.
        assert util == pytest.approx(50.0, abs=0.1)
        # Original hours should be restored.
        assert cp._machines['cnc1'] == 8.0

    def test_what_if_empty_demand(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)
        util = cp.what_if_add_machine('cnc2', 8.0)
        assert util == 0.0


# ===================================================================
# Tests: Edge Cases
# ===================================================================

class TestEdgeCases:
    def test_multiple_demands_partial_allocation(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)  # 40 hrs over 5 days
        # First demand: 30 hours (fits)
        cp.add_demand(_make_demand('P1', quantity=300, time_per_part_min=6.0, priority=0))
        # Second demand: 20 hours (only 10 fit)
        cp.add_demand(_make_demand('P2', quantity=200, time_per_part_min=6.0, priority=1))
        plan = cp.plan(planning_horizon_days=5)
        assert plan.feasible is False
        assert plan.total_allocated_hours == pytest.approx(40.0, abs=0.1)
        assert len(plan.unmet_demand) > 0
        # Unmet should be part of P2.
        unmet_parts = [d.part_number for d in plan.unmet_demand]
        assert 'P2' in unmet_parts

    def test_zero_quantity_demand(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)
        cp.add_demand(_make_demand(quantity=0, time_per_part_min=6.0))
        plan = cp.plan(planning_horizon_days=5)
        assert plan.feasible is True
        assert plan.total_allocated_hours == 0.0

    def test_start_time_affects_slot_times(self):
        cp = CapacityPlanner()
        cp.add_machine('cnc1', 8.0)
        start = 1000000.0
        plan = cp.plan(planning_horizon_days=1, start_time=start)
        slot = plan.slots[0]
        assert slot.start_time == start
        assert slot.end_time == start + 24.0 * 3600.0
