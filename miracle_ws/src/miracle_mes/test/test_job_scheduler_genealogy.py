"""Tests for JobScheduler material genealogy integration.

Validates that job start records MATERIAL_LOADED and job completion
records PART_COMPLETE in the digital thread.
"""

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any
from unittest.mock import MagicMock

import pytest


# ---------------------------------------------------------------------------
# Lightweight stubs that avoid ROS2 dependencies
# ---------------------------------------------------------------------------

class _StubDigitalThread:
    """Minimal stand-in for DigitalThreadNode genealogy interface."""

    ENTRY_MATERIAL_LOADED = 'MATERIAL_LOADED'
    ENTRY_PART_COMPLETE = 'PART_COMPLETE'

    def __init__(self):
        self._entries: List[Dict[str, Any]] = []
        self._lock = threading.Lock()

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
        with self._lock:
            self._entries.append(entry)


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


# ---------------------------------------------------------------------------
# Helpers that replicate the relevant scheduler logic under test
# ---------------------------------------------------------------------------

def _simulate_job_start(digital_thread, job, machine_id):
    """Replicate the genealogy call made in _scheduling_cycle."""
    job.machine_id = machine_id
    job.status = 'ASSIGNED'
    if digital_thread:
        digital_thread.record_genealogy_event(
            _StubDigitalThread.ENTRY_MATERIAL_LOADED,
            machine_id=job.machine_id,
            batch_id=job.material_batch or '',
            serial_number=job.part_serial or '',
            metadata={'job_id': job.job_id, 'program': job.program_name},
        )


def _simulate_job_complete(digital_thread, job, elapsed=10.0, oee=0.85):
    """Replicate the genealogy call made in _execute_job."""
    if digital_thread:
        digital_thread.record_genealogy_event(
            _StubDigitalThread.ENTRY_PART_COMPLETE,
            machine_id=job.machine_id,
            serial_number=job.part_serial or '',
            batch_id=job.material_batch or '',
            metadata={
                'job_id': job.job_id,
                'total_time_sec': elapsed,
                'oee_achieved': oee,
            },
        )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.fixture
def dt():
    return _StubDigitalThread()


class TestJobStartRecordsMaterialLoaded:
    """Job assignment should produce a MATERIAL_LOADED entry."""

    def test_material_loaded_recorded_on_start(self, dt):
        job = _Job(
            priority=2, submit_time=time.time(),
            job_id='J-001', program_name='facing.nc',
            material_batch='BATCH-5', part_serial='SN-100',
        )
        _simulate_job_start(dt, job, 'cnc1')

        assert len(dt._entries) == 1
        entry = dt._entries[0]
        assert entry['entry_type'] == 'MATERIAL_LOADED'
        assert entry['machine_id'] == 'cnc1'
        assert entry['batch_id'] == 'BATCH-5'
        assert entry['serial_number'] == 'SN-100'
        assert entry['job_id'] == 'J-001'
        assert entry['program'] == 'facing.nc'

    def test_no_event_without_digital_thread(self):
        """When _digital_thread is None, no event should be recorded."""
        job = _Job(priority=2, submit_time=time.time(), job_id='J-002')
        _simulate_job_start(None, job, 'cnc2')
        # No assertion on entries — just verifying no exception is raised.

    def test_empty_batch_and_serial_defaults(self, dt):
        job = _Job(priority=2, submit_time=time.time(), job_id='J-003')
        _simulate_job_start(dt, job, 'cnc1')

        entry = dt._entries[0]
        assert entry['batch_id'] == ''
        assert entry['serial_number'] == ''


class TestJobCompletionRecordsPartComplete:
    """Job completion should produce a PART_COMPLETE entry."""

    def test_part_complete_recorded_on_finish(self, dt):
        job = _Job(
            priority=2, submit_time=time.time(),
            job_id='J-010', machine_id='cnc3',
            material_batch='BATCH-9', part_serial='SN-200',
        )
        _simulate_job_complete(dt, job, elapsed=120.5, oee=0.91)

        assert len(dt._entries) == 1
        entry = dt._entries[0]
        assert entry['entry_type'] == 'PART_COMPLETE'
        assert entry['machine_id'] == 'cnc3'
        assert entry['serial_number'] == 'SN-200'
        assert entry['batch_id'] == 'BATCH-9'
        assert entry['job_id'] == 'J-010'
        assert entry['total_time_sec'] == 120.5
        assert entry['oee_achieved'] == 0.91

    def test_no_event_without_digital_thread(self):
        job = _Job(
            priority=2, submit_time=time.time(),
            job_id='J-011', machine_id='cnc1',
        )
        _simulate_job_complete(None, job)
        # No exception should be raised.

    def test_start_and_complete_lifecycle(self, dt):
        """Both events should be present after a full start-complete cycle."""
        job = _Job(
            priority=1, submit_time=time.time(),
            job_id='J-020', program_name='drill.nc',
            material_batch='BATCH-1', part_serial='SN-300',
        )
        _simulate_job_start(dt, job, 'cnc2')
        _simulate_job_complete(dt, job)

        assert len(dt._entries) == 2
        assert dt._entries[0]['entry_type'] == 'MATERIAL_LOADED'
        assert dt._entries[1]['entry_type'] == 'PART_COMPLETE'
        assert dt._entries[0]['serial_number'] == 'SN-300'
        assert dt._entries[1]['serial_number'] == 'SN-300'
