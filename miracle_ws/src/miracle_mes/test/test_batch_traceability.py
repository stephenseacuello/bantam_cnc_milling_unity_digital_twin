"""Tests for BatchTraceabilityManager, MaterialBatch, and BatchUsageRecord.

Validates batch traceability including:
- Batch registration and retrieval
- Usage recording and history
- Remaining quantity calculation
- Forward traceability (batch -> jobs)
- Backward traceability (job -> batches)
- Certification verification (pass / fail)
- Multiple jobs using the same batch
- Expiring batch detection
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
    BatchTraceabilityManager,
    BatchUsageRecord,
    MaterialBatch,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_batch(
    batch_id: str = 'BATCH-001',
    material_type: str = 'AL-6061-T6',
    supplier: str = 'Alcoa',
    lot_number: str = 'LOT-2026-A',
    received_date: float = 0.0,
    quantity: float = 100.0,
    unit: str = 'kg',
    certifications: Optional[List[str]] = None,
    properties: Optional[Dict[str, Any]] = None,
) -> MaterialBatch:
    return MaterialBatch(
        batch_id=batch_id,
        material_type=material_type,
        supplier=supplier,
        lot_number=lot_number,
        received_date=received_date if received_date else time.time(),
        quantity=quantity,
        unit=unit,
        certifications=certifications or [],
        properties=properties or {},
    )


# ===================================================================
# Tests: Batch Registration and Retrieval
# ===================================================================

class TestBatchRegistration:
    def test_register_and_remaining_quantity(self):
        mgr = BatchTraceabilityManager()
        batch = _make_batch(batch_id='B1', quantity=50.0)
        mgr.register_batch(batch)
        assert mgr.get_remaining_quantity('B1') == 50.0

    def test_unknown_batch_remaining_zero(self):
        mgr = BatchTraceabilityManager()
        assert mgr.get_remaining_quantity('NONEXISTENT') == 0.0

    def test_register_multiple_batches(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1', quantity=10.0))
        mgr.register_batch(_make_batch(batch_id='B2', quantity=20.0))
        assert mgr.get_remaining_quantity('B1') == 10.0
        assert mgr.get_remaining_quantity('B2') == 20.0


# ===================================================================
# Tests: Usage Recording and History
# ===================================================================

class TestUsageRecording:
    def test_record_usage_and_history(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1', quantity=100.0))
        mgr.record_usage('B1', 'J1', 'cnc1', 10.0, 'roughing')
        history = mgr.get_batch_history('B1')
        assert len(history) == 1
        assert history[0].batch_id == 'B1'
        assert history[0].job_id == 'J1'
        assert history[0].machine_id == 'cnc1'
        assert history[0].quantity_used == 10.0
        assert history[0].operation == 'roughing'

    def test_multiple_usage_records(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1', quantity=100.0))
        mgr.record_usage('B1', 'J1', 'cnc1', 10.0, 'roughing')
        mgr.record_usage('B1', 'J1', 'cnc1', 5.0, 'finishing')
        mgr.record_usage('B1', 'J2', 'cnc2', 20.0, 'roughing')
        history = mgr.get_batch_history('B1')
        assert len(history) == 3


# ===================================================================
# Tests: Remaining Quantity
# ===================================================================

class TestRemainingQuantity:
    def test_remaining_after_single_use(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1', quantity=100.0))
        mgr.record_usage('B1', 'J1', 'cnc1', 30.0, 'roughing')
        assert mgr.get_remaining_quantity('B1') == 70.0

    def test_remaining_after_multiple_uses(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1', quantity=100.0))
        mgr.record_usage('B1', 'J1', 'cnc1', 25.0, 'roughing')
        mgr.record_usage('B1', 'J2', 'cnc2', 25.0, 'roughing')
        mgr.record_usage('B1', 'J3', 'cnc1', 10.0, 'finishing')
        assert mgr.get_remaining_quantity('B1') == 40.0

    def test_remaining_fully_consumed(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1', quantity=50.0))
        mgr.record_usage('B1', 'J1', 'cnc1', 50.0, 'roughing')
        assert mgr.get_remaining_quantity('B1') == 0.0


# ===================================================================
# Tests: Forward Traceability (batch -> jobs)
# ===================================================================

class TestForwardTraceability:
    def test_single_job(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1'))
        mgr.record_usage('B1', 'J1', 'cnc1', 10.0, 'roughing')
        jobs = mgr.trace_forward('B1')
        assert jobs == ['J1']

    def test_multiple_jobs(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1'))
        mgr.record_usage('B1', 'J1', 'cnc1', 10.0, 'roughing')
        mgr.record_usage('B1', 'J2', 'cnc2', 15.0, 'roughing')
        mgr.record_usage('B1', 'J3', 'cnc1', 5.0, 'finishing')
        jobs = mgr.trace_forward('B1')
        assert set(jobs) == {'J1', 'J2', 'J3'}

    def test_deduplicates_jobs(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1'))
        mgr.record_usage('B1', 'J1', 'cnc1', 10.0, 'roughing')
        mgr.record_usage('B1', 'J1', 'cnc1', 5.0, 'finishing')
        jobs = mgr.trace_forward('B1')
        assert jobs == ['J1']

    def test_unused_batch(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1'))
        jobs = mgr.trace_forward('B1')
        assert jobs == []


# ===================================================================
# Tests: Backward Traceability (job -> batches)
# ===================================================================

class TestBackwardTraceability:
    def test_single_batch(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1'))
        mgr.record_usage('B1', 'J1', 'cnc1', 10.0, 'roughing')
        batches = mgr.trace_backward('J1')
        assert batches == ['B1']

    def test_multiple_batches(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1'))
        mgr.register_batch(_make_batch(batch_id='B2'))
        mgr.record_usage('B1', 'J1', 'cnc1', 10.0, 'roughing')
        mgr.record_usage('B2', 'J1', 'cnc1', 20.0, 'finishing')
        batches = mgr.trace_backward('J1')
        assert set(batches) == {'B1', 'B2'}

    def test_deduplicates_batches(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1'))
        mgr.record_usage('B1', 'J1', 'cnc1', 5.0, 'roughing')
        mgr.record_usage('B1', 'J1', 'cnc1', 3.0, 'finishing')
        batches = mgr.trace_backward('J1')
        assert batches == ['B1']

    def test_job_with_no_material(self):
        mgr = BatchTraceabilityManager()
        batches = mgr.trace_backward('J_UNKNOWN')
        assert batches == []


# ===================================================================
# Tests: Certification Check
# ===================================================================

class TestCertificationCheck:
    def test_has_certification(self):
        mgr = BatchTraceabilityManager()
        batch = _make_batch(
            batch_id='B1',
            certifications=['ISO-9001', 'AS9100', 'NADCAP'],
        )
        mgr.register_batch(batch)
        assert mgr.check_certification('B1', 'ISO-9001') is True
        assert mgr.check_certification('B1', 'AS9100') is True

    def test_missing_certification(self):
        mgr = BatchTraceabilityManager()
        batch = _make_batch(batch_id='B1', certifications=['ISO-9001'])
        mgr.register_batch(batch)
        assert mgr.check_certification('B1', 'NADCAP') is False

    def test_unknown_batch_certification(self):
        mgr = BatchTraceabilityManager()
        assert mgr.check_certification('NONEXISTENT', 'ISO-9001') is False

    def test_no_certifications(self):
        mgr = BatchTraceabilityManager()
        batch = _make_batch(batch_id='B1', certifications=[])
        mgr.register_batch(batch)
        assert mgr.check_certification('B1', 'ISO-9001') is False


# ===================================================================
# Tests: Multiple Jobs Using Same Batch
# ===================================================================

class TestMultipleJobsSameBatch:
    def test_three_jobs_share_batch(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1', quantity=100.0))
        mgr.record_usage('B1', 'J1', 'cnc1', 20.0, 'roughing')
        mgr.record_usage('B1', 'J2', 'cnc2', 30.0, 'roughing')
        mgr.record_usage('B1', 'J3', 'cnc3', 15.0, 'finishing')

        # Forward: batch -> all three jobs
        assert set(mgr.trace_forward('B1')) == {'J1', 'J2', 'J3'}

        # Each job traces back to the same batch
        for jid in ('J1', 'J2', 'J3'):
            assert mgr.trace_backward(jid) == ['B1']

        # Remaining quantity is correct
        assert mgr.get_remaining_quantity('B1') == 35.0

    def test_get_job_materials_returns_correct_records(self):
        mgr = BatchTraceabilityManager()
        mgr.register_batch(_make_batch(batch_id='B1', quantity=100.0))
        mgr.register_batch(_make_batch(batch_id='B2', quantity=50.0))
        mgr.record_usage('B1', 'J1', 'cnc1', 10.0, 'roughing')
        mgr.record_usage('B2', 'J1', 'cnc1', 5.0, 'finishing')
        mgr.record_usage('B1', 'J2', 'cnc2', 20.0, 'roughing')

        j1_materials = mgr.get_job_materials('J1')
        assert len(j1_materials) == 2
        assert {r.batch_id for r in j1_materials} == {'B1', 'B2'}

        j2_materials = mgr.get_job_materials('J2')
        assert len(j2_materials) == 1
        assert j2_materials[0].batch_id == 'B1'


# ===================================================================
# Tests: Expiring Batch Detection
# ===================================================================

class TestExpiringBatches:
    def test_batch_expiring_soon(self):
        mgr = BatchTraceabilityManager()
        # Batch received 25 days ago, shelf life 30 days -> expires in 5 days
        received = time.time() - 25 * 86400.0
        batch = _make_batch(
            batch_id='B1',
            received_date=received,
            properties={'shelf_life_days': 30},
        )
        mgr.register_batch(batch)

        expiring = mgr.get_expiring_batches(days_ahead=7)
        assert len(expiring) == 1
        assert expiring[0].batch_id == 'B1'

    def test_batch_not_expiring_soon(self):
        mgr = BatchTraceabilityManager()
        # Batch received today, shelf life 365 days -> far from expiry
        batch = _make_batch(
            batch_id='B1',
            received_date=time.time(),
            properties={'shelf_life_days': 365},
        )
        mgr.register_batch(batch)

        expiring = mgr.get_expiring_batches(days_ahead=7)
        assert len(expiring) == 0

    def test_batch_already_expired(self):
        mgr = BatchTraceabilityManager()
        # Batch received 60 days ago, shelf life 30 days -> expired 30 days ago
        received = time.time() - 60 * 86400.0
        batch = _make_batch(
            batch_id='B1',
            received_date=received,
            properties={'shelf_life_days': 30},
        )
        mgr.register_batch(batch)

        expiring = mgr.get_expiring_batches(days_ahead=7)
        assert len(expiring) == 1
        assert expiring[0].batch_id == 'B1'

    def test_batch_without_shelf_life_ignored(self):
        mgr = BatchTraceabilityManager()
        batch = _make_batch(batch_id='B1', properties={})
        mgr.register_batch(batch)

        expiring = mgr.get_expiring_batches(days_ahead=7)
        assert len(expiring) == 0

    def test_mixed_batches(self):
        mgr = BatchTraceabilityManager()
        # B1: expiring soon
        mgr.register_batch(_make_batch(
            batch_id='B1',
            received_date=time.time() - 28 * 86400.0,
            properties={'shelf_life_days': 30},
        ))
        # B2: not expiring
        mgr.register_batch(_make_batch(
            batch_id='B2',
            received_date=time.time(),
            properties={'shelf_life_days': 365},
        ))
        # B3: no shelf life property
        mgr.register_batch(_make_batch(
            batch_id='B3',
            properties={},
        ))

        expiring = mgr.get_expiring_batches(days_ahead=7)
        assert len(expiring) == 1
        assert expiring[0].batch_id == 'B1'
