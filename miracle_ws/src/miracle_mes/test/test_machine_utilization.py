"""Tests for MachineUtilizationTracker, MachineStateEvent, and UtilizationMetrics.

Validates machine utilization tracking including:
- Recording state changes and retrieving current state
- Utilization calculation over a time window
- State timeline retrieval
- Fleet-wide utilization aggregation
- Idle-period analysis (count, avg duration, total)
- Machine comparison / ranking by utilization
- Invalid state rejection
- Edge cases with empty data
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

# Stub the DigitalThreadNode so the module-level class doesn't break
# when imported, but leave the standalone classes intact.


class _StubDigitalThread:
    CRITICALITY_MEDIUM = 'MEDIUM'
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
# Now import the production code
# ---------------------------------------------------------------------------
from miracle_mes.digital_thread import (  # noqa: E402
    MachineStateEvent,
    MachineUtilizationTracker,
    UtilizationMetrics,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _evt(
    machine_id: str = 'cnc1',
    state: str = 'running',
    timestamp: float = 0.0,
    operator: str = 'op1',
    job_id: str = 'J1',
) -> MachineStateEvent:
    return MachineStateEvent(
        machine_id=machine_id,
        state=state,
        timestamp=timestamp,
        operator=operator,
        job_id=job_id,
    )


# ===================================================================
# Tests: Record and Current State
# ===================================================================

class TestRecordAndCurrentState:
    def test_record_and_get_current_state(self):
        tracker = MachineUtilizationTracker()
        tracker.record_state_change(_evt(state='idle', timestamp=100.0))
        tracker.record_state_change(_evt(state='running', timestamp=200.0))

        current = tracker.get_current_state('cnc1')
        assert current is not None
        assert current.state == 'running'
        assert current.timestamp == 200.0

    def test_current_state_unknown_machine(self):
        tracker = MachineUtilizationTracker()
        assert tracker.get_current_state('nonexistent') is None

    def test_invalid_state_rejected(self):
        tracker = MachineUtilizationTracker()
        with pytest.raises(ValueError, match="Invalid state"):
            tracker.record_state_change(_evt(state='exploding', timestamp=100.0))


# ===================================================================
# Tests: Utilization Calculation
# ===================================================================

class TestUtilizationCalculation:
    def test_single_state_full_window(self):
        """Machine running for the entire window -> 100% utilization."""
        tracker = MachineUtilizationTracker()
        tracker.record_state_change(_evt(state='running', timestamp=0.0))

        metrics = tracker.get_utilization('cnc1', 0.0, 600.0)
        assert metrics.machine_id == 'cnc1'
        assert metrics.total_time_min == pytest.approx(10.0)
        assert metrics.running_min == pytest.approx(10.0)
        assert metrics.idle_min == pytest.approx(0.0)
        assert metrics.utilization_pct == pytest.approx(100.0)
        assert metrics.availability_pct == pytest.approx(100.0)

    def test_mixed_states(self):
        """Machine transitions through multiple states."""
        tracker = MachineUtilizationTracker()
        # 0-300: running (5 min), 300-600: idle (5 min)
        tracker.record_state_change(_evt(state='running', timestamp=0.0))
        tracker.record_state_change(_evt(state='idle', timestamp=300.0))

        metrics = tracker.get_utilization('cnc1', 0.0, 600.0)
        assert metrics.running_min == pytest.approx(5.0)
        assert metrics.idle_min == pytest.approx(5.0)
        assert metrics.utilization_pct == pytest.approx(50.0)
        assert metrics.availability_pct == pytest.approx(100.0)

    def test_breakdown_reduces_availability(self):
        """Breakdown time reduces availability percentage."""
        tracker = MachineUtilizationTracker()
        # 0-600: running (10 min), 600-1200: breakdown (10 min)
        tracker.record_state_change(_evt(state='running', timestamp=0.0))
        tracker.record_state_change(_evt(state='breakdown', timestamp=600.0))

        metrics = tracker.get_utilization('cnc1', 0.0, 1200.0)
        assert metrics.total_time_min == pytest.approx(20.0)
        assert metrics.running_min == pytest.approx(10.0)
        assert metrics.breakdown_min == pytest.approx(10.0)
        assert metrics.utilization_pct == pytest.approx(50.0)
        assert metrics.availability_pct == pytest.approx(50.0)


# ===================================================================
# Tests: State Timeline
# ===================================================================

class TestStateTimeline:
    def test_timeline_within_window(self):
        tracker = MachineUtilizationTracker()
        tracker.record_state_change(_evt(state='idle', timestamp=100.0))
        tracker.record_state_change(_evt(state='running', timestamp=200.0))
        tracker.record_state_change(_evt(state='idle', timestamp=300.0))
        tracker.record_state_change(_evt(state='off', timestamp=500.0))

        timeline = tracker.get_state_timeline('cnc1', 150.0, 350.0)
        assert len(timeline) == 2
        assert timeline[0].state == 'running'
        assert timeline[1].state == 'idle'

    def test_empty_timeline(self):
        tracker = MachineUtilizationTracker()
        timeline = tracker.get_state_timeline('cnc1', 0.0, 100.0)
        assert timeline == []


# ===================================================================
# Tests: Fleet Utilization
# ===================================================================

class TestFleetUtilization:
    def test_multiple_machines(self):
        tracker = MachineUtilizationTracker()
        # cnc1: running the whole time
        tracker.record_state_change(_evt(machine_id='cnc1', state='running', timestamp=0.0))
        # cnc2: idle the whole time
        tracker.record_state_change(_evt(machine_id='cnc2', state='idle', timestamp=0.0))

        fleet = tracker.get_fleet_utilization(0.0, 600.0)
        assert len(fleet) == 2

        by_id = {m.machine_id: m for m in fleet}
        assert by_id['cnc1'].utilization_pct == pytest.approx(100.0)
        assert by_id['cnc2'].utilization_pct == pytest.approx(0.0)


# ===================================================================
# Tests: Idle Analysis
# ===================================================================

class TestIdleAnalysis:
    def test_idle_periods(self):
        tracker = MachineUtilizationTracker()
        # running 0-120, idle 120-240, running 240-360, idle 360-600
        tracker.record_state_change(_evt(state='running', timestamp=0.0))
        tracker.record_state_change(_evt(state='idle', timestamp=120.0))
        tracker.record_state_change(_evt(state='running', timestamp=240.0))
        tracker.record_state_change(_evt(state='idle', timestamp=360.0))

        analysis = tracker.get_idle_analysis('cnc1', 0.0, 600.0)
        assert analysis['idle_count'] == 2
        # First idle: 120s, second idle: 240s => total 360s = 6 min
        assert analysis['total_idle_min'] == pytest.approx(6.0)
        # avg = 360s / 2 = 180s = 3 min
        assert analysis['avg_idle_duration_min'] == pytest.approx(3.0)

    def test_no_idle(self):
        tracker = MachineUtilizationTracker()
        tracker.record_state_change(_evt(state='running', timestamp=0.0))

        analysis = tracker.get_idle_analysis('cnc1', 0.0, 600.0)
        assert analysis['idle_count'] == 0
        assert analysis['total_idle_min'] == pytest.approx(0.0)
        assert analysis['avg_idle_duration_min'] == pytest.approx(0.0)

    def test_idle_analysis_empty_machine(self):
        tracker = MachineUtilizationTracker()
        analysis = tracker.get_idle_analysis('cnc1', 0.0, 600.0)
        assert analysis['idle_count'] == 0


# ===================================================================
# Tests: Compare Machines
# ===================================================================

class TestCompareMachines:
    def test_ranking_order(self):
        tracker = MachineUtilizationTracker()
        # cnc1: 100% running
        tracker.record_state_change(_evt(machine_id='cnc1', state='running', timestamp=0.0))
        # cnc2: 50% running, 50% idle
        tracker.record_state_change(_evt(machine_id='cnc2', state='running', timestamp=0.0))
        tracker.record_state_change(_evt(machine_id='cnc2', state='idle', timestamp=300.0))
        # cnc3: 0% running (all idle)
        tracker.record_state_change(_evt(machine_id='cnc3', state='idle', timestamp=0.0))

        ranked = tracker.compare_machines(['cnc1', 'cnc2', 'cnc3'], 0.0, 600.0)
        assert len(ranked) == 3
        assert ranked[0].machine_id == 'cnc1'
        assert ranked[0].utilization_pct == pytest.approx(100.0)
        assert ranked[1].machine_id == 'cnc2'
        assert ranked[1].utilization_pct == pytest.approx(50.0)
        assert ranked[2].machine_id == 'cnc3'
        assert ranked[2].utilization_pct == pytest.approx(0.0)
