"""Tests for ScrapTracker, ScrapEvent, and ScrapSummary.

Validates scrap tracking including:
- Recording scrap events
- Summary generation with scrap rate and cost
- Filtering by reason code
- Pareto analysis by cost
- Scrap trend over time periods
- Cost of poor quality (COPQ)
- Rework candidates identification
- Multiple machines and operations
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
    ScrapEvent,
    ScrapSummary,
    ScrapTracker,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_BASE_TIME = 1_700_000_000.0


def _make_event(
    event_id: str = 'EVT-001',
    part_id: str = 'PART-001',
    job_id: str = 'JOB-001',
    machine_id: str = 'cnc1',
    reason_code: str = 'DIM_OUT_OF_TOL',
    reason_description: str = 'Dimension out of tolerance',
    timestamp: float = _BASE_TIME,
    quantity: int = 1,
    material_cost: float = 25.0,
    labor_cost: float = 15.0,
    operation: str = 'roughing',
    is_reworkable: bool = False,
) -> ScrapEvent:
    return ScrapEvent(
        event_id=event_id,
        part_id=part_id,
        job_id=job_id,
        machine_id=machine_id,
        reason_code=reason_code,
        reason_description=reason_description,
        timestamp=timestamp,
        quantity=quantity,
        material_cost=material_cost,
        labor_cost=labor_cost,
        operation=operation,
        is_reworkable=is_reworkable,
    )


def _populate_tracker() -> ScrapTracker:
    """Create a tracker with a representative mix of scrap events."""
    tracker = ScrapTracker()
    tracker.record_scrap(_make_event(
        event_id='E1', part_id='P1', job_id='J1', machine_id='cnc1',
        reason_code='DIM_OUT_OF_TOL', timestamp=_BASE_TIME + 100,
        quantity=2, material_cost=50.0, labor_cost=30.0,
        operation='roughing', is_reworkable=False,
    ))
    tracker.record_scrap(_make_event(
        event_id='E2', part_id='P2', job_id='J1', machine_id='cnc1',
        reason_code='SURFACE_FINISH', timestamp=_BASE_TIME + 200,
        quantity=1, material_cost=25.0, labor_cost=15.0,
        operation='finishing', is_reworkable=True,
    ))
    tracker.record_scrap(_make_event(
        event_id='E3', part_id='P3', job_id='J2', machine_id='cnc2',
        reason_code='TOOL_BREAKAGE', timestamp=_BASE_TIME + 300,
        quantity=3, material_cost=75.0, labor_cost=45.0,
        operation='roughing', is_reworkable=False,
    ))
    tracker.record_scrap(_make_event(
        event_id='E4', part_id='P4', job_id='J2', machine_id='cnc2',
        reason_code='DIM_OUT_OF_TOL', timestamp=_BASE_TIME + 400,
        quantity=1, material_cost=25.0, labor_cost=15.0,
        operation='drilling', is_reworkable=True,
    ))
    tracker.record_scrap(_make_event(
        event_id='E5', part_id='P5', job_id='J3', machine_id='cnc3',
        reason_code='SURFACE_FINISH', timestamp=_BASE_TIME + 500,
        quantity=2, material_cost=40.0, labor_cost=20.0,
        operation='finishing', is_reworkable=True,
    ))
    return tracker


# ===================================================================
# Tests: Recording Scrap Events
# ===================================================================

class TestRecordScrap:
    def test_record_single_event(self):
        tracker = ScrapTracker()
        event = _make_event()
        tracker.record_scrap(event)
        results = tracker.get_scrap_by_reason('DIM_OUT_OF_TOL')
        assert len(results) == 1
        assert results[0].event_id == 'EVT-001'

    def test_record_multiple_events(self):
        tracker = _populate_tracker()
        # Should have events for DIM_OUT_OF_TOL (E1, E4)
        results = tracker.get_scrap_by_reason('DIM_OUT_OF_TOL')
        assert len(results) == 2


# ===================================================================
# Tests: Summary Generation
# ===================================================================

class TestGetSummary:
    def test_summary_counts_and_cost(self):
        tracker = _populate_tracker()
        summary = tracker.get_summary(
            start_time=_BASE_TIME,
            end_time=_BASE_TIME + 600,
            total_parts_produced=100,
        )
        # Total qty: 2 + 1 + 3 + 1 + 2 = 9
        assert summary.total_scrap_count == 9
        # Total cost: (50+30)+(25+15)+(75+45)+(25+15)+(40+20) = 340
        assert summary.total_cost == pytest.approx(340.0)
        # Scrap rate: 9/100 * 100 = 9.0%
        assert summary.scrap_rate_pct == pytest.approx(9.0)

    def test_summary_by_machine(self):
        tracker = _populate_tracker()
        summary = tracker.get_summary(
            start_time=_BASE_TIME,
            end_time=_BASE_TIME + 600,
            total_parts_produced=50,
        )
        assert summary.by_machine == {'cnc1': 3, 'cnc2': 4, 'cnc3': 2}

    def test_summary_by_operation(self):
        tracker = _populate_tracker()
        summary = tracker.get_summary(
            start_time=_BASE_TIME,
            end_time=_BASE_TIME + 600,
            total_parts_produced=50,
        )
        assert summary.by_operation == {'roughing': 5, 'finishing': 3, 'drilling': 1}

    def test_summary_rework_pct(self):
        tracker = _populate_tracker()
        summary = tracker.get_summary(
            start_time=_BASE_TIME,
            end_time=_BASE_TIME + 600,
            total_parts_produced=100,
        )
        # Reworkable qty: 1 + 1 + 2 = 4 out of 9 total
        assert summary.rework_pct == pytest.approx(4 / 9 * 100.0)

    def test_summary_zero_production(self):
        tracker = ScrapTracker()
        summary = tracker.get_summary(
            start_time=_BASE_TIME,
            end_time=_BASE_TIME + 600,
            total_parts_produced=0,
        )
        assert summary.scrap_rate_pct == 0.0
        assert summary.total_scrap_count == 0


# ===================================================================
# Tests: Filter by Reason Code
# ===================================================================

class TestGetScrapByReason:
    def test_existing_reason(self):
        tracker = _populate_tracker()
        results = tracker.get_scrap_by_reason('TOOL_BREAKAGE')
        assert len(results) == 1
        assert results[0].event_id == 'E3'

    def test_nonexistent_reason(self):
        tracker = _populate_tracker()
        results = tracker.get_scrap_by_reason('NONEXISTENT')
        assert results == []


# ===================================================================
# Tests: Pareto Analysis
# ===================================================================

class TestParetoAnalysis:
    def test_pareto_order_by_cost(self):
        tracker = _populate_tracker()
        pareto = tracker.get_pareto_analysis(top_n=3)
        # Costs by reason:
        #   DIM_OUT_OF_TOL: (50+30) + (25+15) = 120
        #   TOOL_BREAKAGE: 75+45 = 120
        #   SURFACE_FINISH: (25+15) + (40+20) = 100
        # Total = 340
        assert len(pareto) == 3
        # First two share the same cost (120), order depends on sort stability
        reason_codes = [r[0] for r in pareto]
        assert set(reason_codes) == {'DIM_OUT_OF_TOL', 'TOOL_BREAKAGE', 'SURFACE_FINISH'}
        # Cumulative pct of last entry should be 100%
        assert pareto[-1][2] == pytest.approx(100.0)

    def test_pareto_top_n_limits(self):
        tracker = _populate_tracker()
        pareto = tracker.get_pareto_analysis(top_n=1)
        assert len(pareto) == 1

    def test_pareto_empty_tracker(self):
        tracker = ScrapTracker()
        pareto = tracker.get_pareto_analysis(top_n=5)
        assert pareto == []


# ===================================================================
# Tests: Scrap Trend
# ===================================================================

class TestScrapTrend:
    def test_trend_buckets(self):
        tracker = _populate_tracker()
        # 5 events from +100 to +500; use 5 periods of 100s each.
        # The last bucket includes the upper boundary (latest event).
        trend = tracker.get_scrap_trend(periods=5, period_duration_sec=100.0)
        assert len(trend) == 5
        counts = [c for _, c in trend]
        assert sum(counts) == 9  # total qty across all events

    def test_trend_empty(self):
        tracker = ScrapTracker()
        trend = tracker.get_scrap_trend(periods=5, period_duration_sec=60.0)
        assert trend == []


# ===================================================================
# Tests: Cost of Poor Quality
# ===================================================================

class TestCostOfPoorQuality:
    def test_copq_breakdown(self):
        tracker = _populate_tracker()
        copq = tracker.get_cost_of_poor_quality(
            start_time=_BASE_TIME,
            end_time=_BASE_TIME + 600,
        )
        # Non-reworkable scrap cost: E1(50+30) + E3(75+45) = 200
        assert copq['scrap_cost'] == pytest.approx(200.0)
        # Rework cost (labor only for reworkable): E2(15) + E4(15) + E5(20) = 50
        assert copq['rework_cost'] == pytest.approx(50.0)
        assert copq['total_copq'] == pytest.approx(250.0)


# ===================================================================
# Tests: Rework Candidates
# ===================================================================

class TestReworkCandidates:
    def test_returns_only_reworkable(self):
        tracker = _populate_tracker()
        candidates = tracker.get_rework_candidates()
        assert len(candidates) == 3
        assert all(c.is_reworkable for c in candidates)
        ids = {c.event_id for c in candidates}
        assert ids == {'E2', 'E4', 'E5'}

    def test_no_reworkable_events(self):
        tracker = ScrapTracker()
        tracker.record_scrap(_make_event(is_reworkable=False))
        candidates = tracker.get_rework_candidates()
        assert candidates == []
