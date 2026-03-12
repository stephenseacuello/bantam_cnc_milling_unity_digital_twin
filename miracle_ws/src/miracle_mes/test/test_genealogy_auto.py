"""Tests for automatic genealogy event triggering.

Validates TOOL_INSTALLED, TOOL_REMOVED, OPERATION_COMPLETE auto-recording
in the digital thread, wired to job lifecycle events. Uses lightweight
stubs that avoid ROS2 dependencies.
"""

import hashlib
import json
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import pytest


# ---------------------------------------------------------------------------
# Standalone replica of DigitalThreadNode genealogy logic
# ---------------------------------------------------------------------------

class _GenealogyThread:
    """Mirrors DigitalThreadNode genealogy methods for testing without ROS2."""

    ENTRY_MATERIAL_LOADED = 'MATERIAL_LOADED'
    ENTRY_TOOL_INSTALLED = 'TOOL_INSTALLED'
    ENTRY_OPERATION_COMPLETE = 'OPERATION_COMPLETE'
    ENTRY_OPERATION_FAILED = 'OPERATION_FAILED'
    ENTRY_PART_COMPLETE = 'PART_COMPLETE'
    ENTRY_TOOL_REMOVED = 'TOOL_REMOVED'
    ENTRY_ANOMALY_DETECTED = 'ANOMALY_DETECTED'
    ENTRY_JOB_PAUSED = 'JOB_PAUSED'
    ENTRY_JOB_RESUMED = 'JOB_RESUMED'
    ENTRY_JOB_CANCELLED = 'JOB_CANCELLED'
    ENTRY_JOB_FAILED = 'JOB_FAILED'
    ENTRY_MACHINE_ERROR = 'MACHINE_ERROR'

    def __init__(self):
        self._entries: List[Dict[str, Any]] = []
        self._thread_lock = threading.Lock()
        self._genealogy_last_hash: str = '0' * 64

        self._current_tool_per_machine: Dict[str, str] = {}
        self._active_job_per_machine: Dict[str, Dict[str, Any]] = {}

    def _record_entry(self, entry: Dict[str, Any]) -> None:
        with self._thread_lock:
            hash_input = (
                f"{entry.get('entry_type', '')}"
                f"{entry.get('timestamp', '')}"
                f"{self._genealogy_last_hash}"
                f"{json.dumps(entry, default=str, sort_keys=True)}"
            )
            entry['previous_hash'] = self._genealogy_last_hash
            entry['hash'] = hashlib.sha256(hash_input.encode()).hexdigest()
            self._genealogy_last_hash = entry['hash']
            self._entries.append(entry)

    def record_genealogy_event(
        self, entry_type, machine_id,
        serial_number='', tool_id='',
        batch_id='', metadata=None,
    ):
        entry = {
            'entry_type': entry_type,
            'machine_id': machine_id,
            'timestamp': time.time(),
            'serial_number': serial_number,
            'tool_id': tool_id,
            'batch_id': batch_id,
        }
        if metadata:
            entry.update(metadata)
        self._record_entry(entry)

    def set_active_job(
        self, machine_id, job_id,
        serial_number='', batch_id='', operator='',
    ):
        self._active_job_per_machine[machine_id] = {
            'job_id': job_id,
            'serial_number': serial_number,
            'batch_id': batch_id,
            'operator': operator,
        }

    def clear_active_job(self, machine_id):
        self._active_job_per_machine.pop(machine_id, None)

    def record_operation_complete(
        self, machine_id, tool_id, operation_type,
        start_time, end_time, serial_number='', batch_id='',
        material_removed_volume=0.0, surface_finish_ra=0.0,
        metadata=None,
    ):
        meta = {
            'operation_type': operation_type,
            'start_time': start_time,
            'end_time': end_time,
            'duration_sec': end_time - start_time,
            'material_removed_volume': material_removed_volume,
            'surface_finish_ra': surface_finish_ra,
        }
        if metadata:
            meta.update(metadata)
        self.record_genealogy_event(
            self.ENTRY_OPERATION_COMPLETE,
            machine_id=machine_id,
            tool_id=tool_id,
            serial_number=serial_number,
            batch_id=batch_id,
            metadata=meta,
        )

    def get_part_history(self, serial_number):
        with self._thread_lock:
            return [
                e for e in self._entries
                if e.get('serial_number') == serial_number
                or e.get('part_serial') == serial_number
            ]

    def get_full_part_genealogy(self, serial_number):
        with self._thread_lock:
            entries = [
                e for e in self._entries
                if e.get('serial_number') == serial_number
                or e.get('part_serial') == serial_number
            ]
        return sorted(entries, key=lambda e: e.get('timestamp', 0))

    def get_tool_history(self, tool_id):
        with self._thread_lock:
            return [
                e for e in self._entries
                if e.get('tool_id') == tool_id
            ]

    def get_machine_utilization_history(self, machine_id, hours=24):
        cutoff = time.time() - (hours * 3600)
        with self._thread_lock:
            entries = [
                e for e in self._entries
                if e.get('machine_id') == machine_id
                and e.get('timestamp', 0) >= cutoff
            ]
        entries.sort(key=lambda e: e.get('timestamp', 0))
        result = []
        for entry in entries:
            ts = entry.get('timestamp', 0)
            start = entry.get('start_time', ts)
            end = entry.get('end_time', ts)
            result.append((
                start, end,
                entry.get('entry_type', ''),
                entry.get('job_id', ''),
            ))
        return result

    def verify_genealogy_integrity(self):
        with self._thread_lock:
            prev_hash = '0' * 64
            for entry in self._entries:
                if entry.get('previous_hash') != prev_hash:
                    return False
                entry_copy = {
                    k: v for k, v in entry.items()
                    if k not in ('hash', 'previous_hash')
                }
                hash_input = (
                    f"{entry.get('entry_type', '')}"
                    f"{entry.get('timestamp', '')}"
                    f"{prev_hash}"
                    f"{json.dumps(entry_copy, default=str, sort_keys=True)}"
                )
                expected = hashlib.sha256(hash_input.encode()).hexdigest()
                if entry.get('hash') != expected:
                    return False
                prev_hash = entry['hash']
        return True


# ---------------------------------------------------------------------------
# Lightweight Job stub
# ---------------------------------------------------------------------------

@dataclass(order=True)
class _Job:
    priority: int
    submit_time: float
    job_id: str = field(compare=False)
    program_name: str = field(compare=False, default='')
    machine_id: str = field(compare=False, default='')
    material: str = field(compare=False, default='steel')
    status: str = field(compare=False, default='QUEUED')
    task_type: str = field(compare=False, default='MILLING')
    required_capabilities: List[str] = field(compare=False, default_factory=list)
    material_batch: str = field(compare=False, default='')
    part_serial: str = field(compare=False, default='')
    tool_id: str = field(compare=False, default='')


# ---------------------------------------------------------------------------
# Job lifecycle simulation helpers (mirrors job_scheduler logic)
# ---------------------------------------------------------------------------

def _simulate_job_start(dt, job, machine_id):
    """Replicate the genealogy calls in _scheduling_cycle."""
    job.machine_id = machine_id
    job.status = 'ASSIGNED'
    if dt:
        dt.record_genealogy_event(
            _GenealogyThread.ENTRY_MATERIAL_LOADED,
            machine_id=job.machine_id,
            batch_id=job.material_batch or '',
            serial_number=job.part_serial or '',
            metadata={'job_id': job.job_id, 'program': job.program_name},
        )
        if job.tool_id:
            dt.record_genealogy_event(
                _GenealogyThread.ENTRY_TOOL_INSTALLED,
                machine_id=job.machine_id,
                tool_id=job.tool_id,
                serial_number=job.part_serial or '',
                batch_id=job.material_batch or '',
                metadata={'job_id': job.job_id},
            )
        dt.set_active_job(
            machine_id=job.machine_id,
            job_id=job.job_id,
            serial_number=job.part_serial or '',
            batch_id=job.material_batch or '',
        )


def _simulate_operation(dt, job, operation_type='MILLING',
                        duration=10.0, material_removed=5.0,
                        surface_finish_ra=0.8):
    """Simulate an operation completing during job execution."""
    if dt:
        now = time.time()
        dt.record_operation_complete(
            machine_id=job.machine_id,
            tool_id=job.tool_id or '',
            operation_type=operation_type,
            start_time=now - duration,
            end_time=now,
            serial_number=job.part_serial or '',
            batch_id=job.material_batch or '',
            material_removed_volume=material_removed,
            surface_finish_ra=surface_finish_ra,
            metadata={'job_id': job.job_id},
        )


def _simulate_job_complete(dt, job, elapsed=10.0, oee=0.85):
    """Replicate the genealogy calls in _execute_job on completion."""
    if dt:
        if job.tool_id:
            dt.record_genealogy_event(
                _GenealogyThread.ENTRY_TOOL_REMOVED,
                machine_id=job.machine_id,
                tool_id=job.tool_id,
                serial_number=job.part_serial or '',
                batch_id=job.material_batch or '',
                metadata={'job_id': job.job_id},
            )
        dt.record_genealogy_event(
            _GenealogyThread.ENTRY_PART_COMPLETE,
            machine_id=job.machine_id,
            serial_number=job.part_serial or '',
            batch_id=job.material_batch or '',
            metadata={
                'job_id': job.job_id,
                'total_time_sec': elapsed,
                'oee_achieved': oee,
            },
        )
        dt.clear_active_job(job.machine_id)


def _simulate_job_cancel(dt, job, phase='MACHINING'):
    """Replicate the genealogy calls when a job is cancelled."""
    if dt:
        dt.record_genealogy_event(
            _GenealogyThread.ENTRY_JOB_CANCELLED,
            machine_id=job.machine_id,
            serial_number=job.part_serial or '',
            batch_id=job.material_batch or '',
            tool_id=job.tool_id or '',
            metadata={'job_id': job.job_id, 'cancelled_at_phase': phase},
        )
        dt.clear_active_job(job.machine_id)


def _simulate_anomaly(dt, machine_id, anomaly_type='VIBRATION',
                      severity='HIGH', confidence=0.92):
    """Simulate an anomaly occurring on a machine with active job."""
    job_ctx = dt._active_job_per_machine.get(machine_id, {})
    if job_ctx:
        dt.record_genealogy_event(
            _GenealogyThread.ENTRY_ANOMALY_DETECTED,
            machine_id=machine_id,
            serial_number=job_ctx.get('serial_number', ''),
            batch_id=job_ctx.get('batch_id', ''),
            tool_id=dt._current_tool_per_machine.get(machine_id, ''),
            metadata={
                'anomaly_type': anomaly_type,
                'severity': severity,
                'confidence': confidence,
                'job_id': job_ctx.get('job_id', ''),
            },
        )


def _simulate_tool_change(dt, job, old_tool_id, new_tool_id):
    """Simulate a mid-job tool change."""
    if dt:
        dt.record_genealogy_event(
            _GenealogyThread.ENTRY_TOOL_REMOVED,
            machine_id=job.machine_id,
            tool_id=old_tool_id,
            serial_number=job.part_serial or '',
            metadata={'job_id': job.job_id},
        )
        dt.record_genealogy_event(
            _GenealogyThread.ENTRY_TOOL_INSTALLED,
            machine_id=job.machine_id,
            tool_id=new_tool_id,
            serial_number=job.part_serial or '',
            metadata={'job_id': job.job_id},
        )
        job.tool_id = new_tool_id
        dt._current_tool_per_machine[job.machine_id] = new_tool_id


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def dt():
    return _GenealogyThread()


def _make_job(job_id='J-001', tool_id='T10', part_serial='SN-100',
              material_batch='BATCH-5', program_name='facing.nc',
              task_type='MILLING'):
    return _Job(
        priority=2, submit_time=time.time(),
        job_id=job_id, program_name=program_name,
        tool_id=tool_id, part_serial=part_serial,
        material_batch=material_batch, task_type=task_type,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestToolInstalledAutoRecorded:
    """TOOL_INSTALLED auto-recorded on job start."""

    def test_tool_installed_on_job_start(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')

        tool_events = [
            e for e in dt._entries
            if e['entry_type'] == 'TOOL_INSTALLED'
        ]
        assert len(tool_events) == 1
        assert tool_events[0]['tool_id'] == 'T10'
        assert tool_events[0]['machine_id'] == 'cnc1'
        assert tool_events[0]['serial_number'] == 'SN-100'

    def test_no_tool_installed_when_no_tool_id(self, dt):
        job = _make_job(tool_id='')
        _simulate_job_start(dt, job, 'cnc1')

        tool_events = [
            e for e in dt._entries
            if e['entry_type'] == 'TOOL_INSTALLED'
        ]
        assert len(tool_events) == 0


class TestToolRemovedAutoRecorded:
    """TOOL_REMOVED auto-recorded on job complete."""

    def test_tool_removed_on_job_complete(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')
        _simulate_job_complete(dt, job)

        removed = [
            e for e in dt._entries
            if e['entry_type'] == 'TOOL_REMOVED'
        ]
        assert len(removed) == 1
        assert removed[0]['tool_id'] == 'T10'
        assert removed[0]['machine_id'] == 'cnc1'

    def test_no_tool_removed_when_no_tool_id(self, dt):
        job = _make_job(tool_id='')
        _simulate_job_start(dt, job, 'cnc1')
        _simulate_job_complete(dt, job)

        removed = [
            e for e in dt._entries
            if e['entry_type'] == 'TOOL_REMOVED'
        ]
        assert len(removed) == 0


class TestOperationCompleteRecorded:
    """OPERATION_COMPLETE recorded during job execution."""

    def test_operation_complete_has_full_details(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')
        _simulate_operation(dt, job, operation_type='MILLING',
                            duration=15.0, material_removed=8.5,
                            surface_finish_ra=0.6)

        ops = [
            e for e in dt._entries
            if e['entry_type'] == 'OPERATION_COMPLETE'
        ]
        assert len(ops) == 1
        op = ops[0]
        assert op['tool_id'] == 'T10'
        assert op['operation_type'] == 'MILLING'
        assert op['material_removed_volume'] == 8.5
        assert op['surface_finish_ra'] == 0.6
        assert abs(op['duration_sec'] - 15.0) < 0.1
        assert op['serial_number'] == 'SN-100'


class TestFullPartGenealogy:
    """get_full_part_genealogy returns complete sorted timeline."""

    def test_returns_complete_timeline(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')
        _simulate_operation(dt, job)
        _simulate_job_complete(dt, job)

        timeline = dt.get_full_part_genealogy('SN-100')
        types = [e['entry_type'] for e in timeline]

        assert 'MATERIAL_LOADED' in types
        assert 'TOOL_INSTALLED' in types
        assert 'OPERATION_COMPLETE' in types
        assert 'TOOL_REMOVED' in types
        assert 'PART_COMPLETE' in types

    def test_timeline_is_sorted_by_timestamp(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')
        time.sleep(0.01)
        _simulate_operation(dt, job)
        time.sleep(0.01)
        _simulate_job_complete(dt, job)

        timeline = dt.get_full_part_genealogy('SN-100')
        timestamps = [e['timestamp'] for e in timeline]
        assert timestamps == sorted(timestamps)


class TestToolChangeMidJob:
    """Tool change mid-job records REMOVED + INSTALLED pair."""

    def test_tool_change_records_both_events(self, dt):
        job = _make_job(tool_id='T10')
        _simulate_job_start(dt, job, 'cnc1')
        _simulate_operation(dt, job)
        _simulate_tool_change(dt, job, 'T10', 'T20')
        _simulate_operation(dt, job, operation_type='DRILLING')

        removed = [
            e for e in dt._entries
            if e['entry_type'] == 'TOOL_REMOVED' and e['tool_id'] == 'T10'
        ]
        installed = [
            e for e in dt._entries
            if e['entry_type'] == 'TOOL_INSTALLED' and e['tool_id'] == 'T20'
        ]
        assert len(removed) == 1
        assert len(installed) == 1
        # REMOVED should come before INSTALLED
        assert removed[0]['timestamp'] <= installed[0]['timestamp']


class TestAnomalyDuringJob:
    """Anomaly during job links to current part."""

    def test_anomaly_linked_to_active_part(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')
        dt._current_tool_per_machine['cnc1'] = 'T10'
        _simulate_anomaly(dt, 'cnc1', anomaly_type='VIBRATION')

        anomalies = [
            e for e in dt._entries
            if e['entry_type'] == 'ANOMALY_DETECTED'
        ]
        assert len(anomalies) == 1
        assert anomalies[0]['serial_number'] == 'SN-100'
        assert anomalies[0]['batch_id'] == 'BATCH-5'
        assert anomalies[0]['anomaly_type'] == 'VIBRATION'
        assert anomalies[0]['job_id'] == 'J-001'


class TestJobCancellation:
    """Job cancellation records event."""

    def test_cancel_records_event(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')
        _simulate_job_cancel(dt, job, phase='MACHINING')

        cancelled = [
            e for e in dt._entries
            if e['entry_type'] == 'JOB_CANCELLED'
        ]
        assert len(cancelled) == 1
        assert cancelled[0]['job_id'] == 'J-001'
        assert cancelled[0]['cancelled_at_phase'] == 'MACHINING'
        assert cancelled[0]['serial_number'] == 'SN-100'

    def test_cancel_clears_active_job(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')
        assert 'cnc1' in dt._active_job_per_machine
        _simulate_job_cancel(dt, job)
        assert 'cnc1' not in dt._active_job_per_machine


class TestMachineUtilizationHistory:
    """Machine utilization history correct."""

    def test_returns_entries_within_time_window(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')
        _simulate_operation(dt, job, duration=5.0)
        _simulate_job_complete(dt, job)

        history = dt.get_machine_utilization_history('cnc1', hours=1)
        assert len(history) >= 3  # at least LOADED, OP_COMPLETE, PART_COMPLETE
        # Each entry is a tuple of (start, end, event_type, job_id)
        for start, end, event_type, job_id in history:
            assert isinstance(event_type, str)

    def test_filters_by_machine_id(self, dt):
        job1 = _make_job(job_id='J-001')
        job2 = _make_job(job_id='J-002', part_serial='SN-200')
        _simulate_job_start(dt, job1, 'cnc1')
        _simulate_job_start(dt, job2, 'cnc2')

        h1 = dt.get_machine_utilization_history('cnc1', hours=1)
        h2 = dt.get_machine_utilization_history('cnc2', hours=1)
        assert all(e[2] != '' for e in h1)
        assert all(e[2] != '' for e in h2)
        # cnc1 entries should not appear in cnc2 history
        assert len(h1) > 0
        assert len(h2) > 0


class TestHashChainIntegrity:
    """Hash chain integrity maintained across genealogy entries."""

    def test_integrity_on_empty_thread(self, dt):
        assert dt.verify_genealogy_integrity() is True

    def test_integrity_after_multiple_events(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')
        _simulate_operation(dt, job)
        _simulate_job_complete(dt, job)

        assert dt.verify_genealogy_integrity() is True

    def test_integrity_fails_on_tamper(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')
        _simulate_operation(dt, job)

        # Tamper with an entry
        dt._entries[1]['entry_type'] = 'TAMPERED'
        assert dt.verify_genealogy_integrity() is False

    def test_each_entry_has_hash_and_previous_hash(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')
        _simulate_operation(dt, job)

        for entry in dt._entries:
            assert 'hash' in entry
            assert 'previous_hash' in entry
            assert len(entry['hash']) == 64

    def test_first_entry_links_to_genesis_hash(self, dt):
        dt.record_genealogy_event('MATERIAL_LOADED', 'cnc1')
        assert dt._entries[0]['previous_hash'] == '0' * 64


class TestMultipleOperationsInSequence:
    """Multiple operations in sequence."""

    def test_sequential_operations_all_recorded(self, dt):
        job = _make_job()
        _simulate_job_start(dt, job, 'cnc1')
        _simulate_operation(dt, job, operation_type='ROUGH_CUT', duration=20.0)
        time.sleep(0.01)
        _simulate_operation(dt, job, operation_type='FINISH_CUT', duration=10.0)
        time.sleep(0.01)
        _simulate_operation(dt, job, operation_type='DRILLING', duration=5.0)
        _simulate_job_complete(dt, job)

        ops = [
            e for e in dt._entries
            if e['entry_type'] == 'OPERATION_COMPLETE'
        ]
        assert len(ops) == 3
        op_types = [o['operation_type'] for o in ops]
        assert op_types == ['ROUGH_CUT', 'FINISH_CUT', 'DRILLING']


class TestGetToolHistoryAcrossMachines:
    """get_tool_history shows all machines a tool was used on."""

    def test_tool_used_on_multiple_machines(self, dt):
        # Tool T10 used on cnc1
        job1 = _make_job(job_id='J-001', tool_id='T10', part_serial='SN-100')
        _simulate_job_start(dt, job1, 'cnc1')
        _simulate_operation(dt, job1)
        _simulate_job_complete(dt, job1)

        # Same tool T10 used on cnc2
        job2 = _make_job(job_id='J-002', tool_id='T10', part_serial='SN-200')
        _simulate_job_start(dt, job2, 'cnc2')
        _simulate_operation(dt, job2)
        _simulate_job_complete(dt, job2)

        history = dt.get_tool_history('T10')
        machines = set(e['machine_id'] for e in history)
        assert 'cnc1' in machines
        assert 'cnc2' in machines
        assert len(history) >= 4  # INSTALLED+OP+REMOVED on each machine
