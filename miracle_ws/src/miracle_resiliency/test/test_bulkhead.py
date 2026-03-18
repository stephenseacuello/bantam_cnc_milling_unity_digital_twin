"""Tests for the BulkheadIsolator pattern.

Covers creation, acquire/release, rejection, execute_in_bulkhead,
status reporting, default bulkheads, concurrent access, and error
propagation.
"""

import sys
import threading
import time
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants',
            'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
            'rclpy.callback_groups',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_msgs', 'miracle_msgs.msg',
            'std_msgs', 'std_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_resiliency.recovery_orchestrator import (
    BulkheadConfig,
    BulkheadIsolator,
    BulkheadRejectedError,
    BulkheadStatus,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_isolator(**overrides) -> BulkheadIsolator:
    """Create a BulkheadIsolator without default bulkheads."""
    return BulkheadIsolator(install_defaults=False)


# ---------------------------------------------------------------------------
# 1. Default bulkheads are installed
# ---------------------------------------------------------------------------

class TestDefaultBulkheads:
    def test_defaults_registered(self):
        iso = BulkheadIsolator(install_defaults=True)
        statuses = {s.name for s in iso.get_all_statuses()}
        assert 'SPINDLE_CONTROL' in statuses
        assert 'SENSOR_READING' in statuses
        assert 'ALARM_PROCESSING' in statuses
        assert 'DATABASE_WRITE' in statuses

    def test_default_max_concurrent_values(self):
        iso = BulkheadIsolator(install_defaults=True)
        status_map = {s.name: s for s in iso.get_all_statuses()}
        assert status_map['SPINDLE_CONTROL'].max_concurrent == 2
        assert status_map['SENSOR_READING'].max_concurrent == 10
        assert status_map['ALARM_PROCESSING'].max_concurrent == 5
        assert status_map['DATABASE_WRITE'].max_concurrent == 3


# ---------------------------------------------------------------------------
# 2. Create custom bulkhead
# ---------------------------------------------------------------------------

class TestCreateBulkhead:
    def test_create_and_query(self):
        iso = _make_isolator()
        iso.create_bulkhead(BulkheadConfig(name='TEST', max_concurrent=4))
        status = iso.get_status('TEST')
        assert status.max_concurrent == 4
        assert status.active_count == 0
        assert status.available_slots == 4
        assert status.is_full is False

    def test_create_replaces_existing(self):
        iso = _make_isolator()
        iso.create_bulkhead(BulkheadConfig(name='X', max_concurrent=1))
        iso.create_bulkhead(BulkheadConfig(name='X', max_concurrent=7))
        assert iso.get_status('X').max_concurrent == 7


# ---------------------------------------------------------------------------
# 3. Acquire and release
# ---------------------------------------------------------------------------

class TestAcquireRelease:
    def test_acquire_increments_active(self):
        iso = _make_isolator()
        iso.create_bulkhead(BulkheadConfig(name='A', max_concurrent=3))
        iso.acquire('A')
        status = iso.get_status('A')
        assert status.active_count == 1
        assert status.available_slots == 2

    def test_release_decrements_active(self):
        iso = _make_isolator()
        iso.create_bulkhead(BulkheadConfig(name='A', max_concurrent=3))
        iso.acquire('A')
        iso.release('A')
        status = iso.get_status('A')
        assert status.active_count == 0
        assert status.completed_count == 1

    def test_acquire_unknown_raises_key_error(self):
        iso = _make_isolator()
        with pytest.raises(KeyError):
            iso.acquire('DOES_NOT_EXIST')


# ---------------------------------------------------------------------------
# 4. Rejection when full
# ---------------------------------------------------------------------------

class TestRejection:
    def test_rejected_when_active_and_queue_full(self):
        iso = _make_isolator()
        iso.create_bulkhead(
            BulkheadConfig(name='TINY', max_concurrent=1, queue_size=0)
        )
        iso.acquire('TINY')  # fills the single slot
        with pytest.raises(BulkheadRejectedError):
            iso.acquire('TINY')  # no queue room
        status = iso.get_status('TINY')
        assert status.rejected_count == 1

    def test_rejection_counter_increments(self):
        iso = _make_isolator()
        iso.create_bulkhead(
            BulkheadConfig(name='T', max_concurrent=1, queue_size=0)
        )
        iso.acquire('T')
        for _ in range(3):
            with pytest.raises(BulkheadRejectedError):
                iso.acquire('T')
        assert iso.get_status('T').rejected_count == 3


# ---------------------------------------------------------------------------
# 5. execute_in_bulkhead
# ---------------------------------------------------------------------------

class TestExecuteInBulkhead:
    def test_successful_execution(self):
        iso = _make_isolator()
        iso.create_bulkhead(BulkheadConfig(name='E', max_concurrent=2))
        result = iso.execute_in_bulkhead('E', lambda x: x * 2, 5)
        assert result == 10
        status = iso.get_status('E')
        assert status.active_count == 0
        assert status.completed_count == 1

    def test_releases_on_exception(self):
        iso = _make_isolator()
        iso.create_bulkhead(BulkheadConfig(name='E', max_concurrent=2))

        def _fail():
            raise RuntimeError('boom')

        with pytest.raises(RuntimeError, match='boom'):
            iso.execute_in_bulkhead('E', _fail)
        # Slot must have been released despite the error.
        assert iso.get_status('E').active_count == 0
        assert iso.get_status('E').completed_count == 1

    def test_rejected_propagates(self):
        iso = _make_isolator()
        iso.create_bulkhead(
            BulkheadConfig(name='E', max_concurrent=1, queue_size=0)
        )
        iso.acquire('E')  # fill the only slot
        with pytest.raises(BulkheadRejectedError):
            iso.execute_in_bulkhead('E', lambda: None)


# ---------------------------------------------------------------------------
# 6. get_status
# ---------------------------------------------------------------------------

class TestGetStatus:
    def test_returns_bulkhead_status_type(self):
        iso = _make_isolator()
        iso.create_bulkhead(BulkheadConfig(name='S', max_concurrent=3))
        status = iso.get_status('S')
        assert isinstance(status, BulkheadStatus)

    def test_is_full_when_saturated(self):
        iso = _make_isolator()
        iso.create_bulkhead(
            BulkheadConfig(name='FULL', max_concurrent=1, queue_size=0)
        )
        iso.acquire('FULL')
        assert iso.get_status('FULL').is_full is True


# ---------------------------------------------------------------------------
# 7. get_all_statuses
# ---------------------------------------------------------------------------

class TestGetAllStatuses:
    def test_returns_all_registered(self):
        iso = _make_isolator()
        iso.create_bulkhead(BulkheadConfig(name='A', max_concurrent=1))
        iso.create_bulkhead(BulkheadConfig(name='B', max_concurrent=2))
        all_s = iso.get_all_statuses()
        names = {s.name for s in all_s}
        assert names == {'A', 'B'}


# ---------------------------------------------------------------------------
# 8. Concurrent access (thread-safety smoke test)
# ---------------------------------------------------------------------------

class TestConcurrency:
    def test_concurrent_acquire_release(self):
        iso = _make_isolator()
        iso.create_bulkhead(BulkheadConfig(name='C', max_concurrent=4, queue_size=20))
        errors = []

        def _worker():
            try:
                iso.acquire('C')
                time.sleep(0.01)
                iso.release('C')
            except Exception as exc:
                errors.append(exc)

        threads = [threading.Thread(target=_worker) for _ in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10)
        assert errors == [], f"Unexpected errors: {errors}"
        status = iso.get_status('C')
        assert status.active_count == 0
        assert status.completed_count == 8
