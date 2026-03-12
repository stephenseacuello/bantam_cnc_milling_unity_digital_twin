"""Tests for Digital Thread material genealogy features.

Validates record_genealogy_event, get_part_history, get_tool_history,
get_batch_traceability, and full lifecycle scenarios.
"""

import threading
import time

import pytest


class _GenealogyThread:
    """Standalone replica of DigitalThreadNode genealogy logic.

    Mirrors the genealogy methods added to DigitalThreadNode so that
    tests can run without a ROS2 lifecycle.
    """

    ENTRY_MATERIAL_LOADED = 'MATERIAL_LOADED'
    ENTRY_TOOL_INSTALLED = 'TOOL_INSTALLED'
    ENTRY_OPERATION_COMPLETE = 'OPERATION_COMPLETE'
    ENTRY_PART_COMPLETE = 'PART_COMPLETE'
    ENTRY_TOOL_REMOVED = 'TOOL_REMOVED'

    def __init__(self):
        self._entries = []
        self._thread_lock = threading.Lock()

    def _record_entry(self, entry):
        with self._thread_lock:
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

    def get_part_history(self, serial_number):
        with self._thread_lock:
            return [
                e for e in self._entries
                if e.get('serial_number') == serial_number or
                   e.get('part_serial') == serial_number
            ]

    def get_tool_history(self, tool_id):
        with self._thread_lock:
            return [
                e for e in self._entries
                if e.get('tool_id') == tool_id
            ]

    def get_batch_traceability(self, batch_id):
        with self._thread_lock:
            return [
                e for e in self._entries
                if e.get('batch_id') == batch_id or
                   e.get('material_batch') == batch_id
            ]


@pytest.fixture
def gt():
    return _GenealogyThread()


class TestRecordGenealogyEvent:
    """Tests for record_genealogy_event."""

    def test_creates_entry_with_all_fields(self, gt):
        gt.record_genealogy_event(
            'MATERIAL_LOADED', 'cnc1',
            serial_number='SN-001', tool_id='T01',
            batch_id='BATCH-A',
        )
        assert len(gt._entries) == 1
        entry = gt._entries[0]
        assert entry['entry_type'] == 'MATERIAL_LOADED'
        assert entry['machine_id'] == 'cnc1'
        assert entry['serial_number'] == 'SN-001'
        assert entry['tool_id'] == 'T01'
        assert entry['batch_id'] == 'BATCH-A'
        assert 'timestamp' in entry

    def test_metadata_merged_into_entry(self, gt):
        gt.record_genealogy_event(
            'OPERATION_COMPLETE', 'cnc2',
            metadata={'job_id': 'J-100', 'extra': 42},
        )
        entry = gt._entries[0]
        assert entry['job_id'] == 'J-100'
        assert entry['extra'] == 42

    def test_defaults_are_empty_strings(self, gt):
        gt.record_genealogy_event('TOOL_INSTALLED', 'cnc3')
        entry = gt._entries[0]
        assert entry['serial_number'] == ''
        assert entry['tool_id'] == ''
        assert entry['batch_id'] == ''

    def test_multiple_events_accumulate(self, gt):
        for i in range(5):
            gt.record_genealogy_event('MATERIAL_LOADED', f'cnc{i}')
        assert len(gt._entries) == 5


class TestGetPartHistory:
    """Tests for get_part_history filtering."""

    def test_filters_by_serial_number(self, gt):
        gt.record_genealogy_event('MATERIAL_LOADED', 'cnc1', serial_number='SN-A')
        gt.record_genealogy_event('MATERIAL_LOADED', 'cnc2', serial_number='SN-B')
        gt.record_genealogy_event('PART_COMPLETE', 'cnc1', serial_number='SN-A')

        history = gt.get_part_history('SN-A')
        assert len(history) == 2
        assert all(e['serial_number'] == 'SN-A' for e in history)

    def test_matches_part_serial_key(self, gt):
        """Entries with 'part_serial' key should also be matched."""
        gt._entries.append({
            'entry_type': 'PART_COMPLETE',
            'part_serial': 'SN-X',
            'machine_id': 'cnc1',
        })
        history = gt.get_part_history('SN-X')
        assert len(history) == 1

    def test_returns_empty_for_unknown_serial(self, gt):
        gt.record_genealogy_event('MATERIAL_LOADED', 'cnc1', serial_number='SN-A')
        assert gt.get_part_history('UNKNOWN') == []


class TestGetToolHistory:
    """Tests for get_tool_history filtering."""

    def test_filters_by_tool_id(self, gt):
        gt.record_genealogy_event('TOOL_INSTALLED', 'cnc1', tool_id='T01')
        gt.record_genealogy_event('TOOL_INSTALLED', 'cnc1', tool_id='T02')
        gt.record_genealogy_event('TOOL_REMOVED', 'cnc1', tool_id='T01')

        history = gt.get_tool_history('T01')
        assert len(history) == 2
        assert all(e['tool_id'] == 'T01' for e in history)

    def test_returns_empty_for_unknown_tool(self, gt):
        gt.record_genealogy_event('TOOL_INSTALLED', 'cnc1', tool_id='T01')
        assert gt.get_tool_history('T99') == []


class TestGetBatchTraceability:
    """Tests for get_batch_traceability filtering."""

    def test_filters_by_batch_id(self, gt):
        gt.record_genealogy_event('MATERIAL_LOADED', 'cnc1', batch_id='B-001')
        gt.record_genealogy_event('MATERIAL_LOADED', 'cnc2', batch_id='B-002')
        gt.record_genealogy_event('PART_COMPLETE', 'cnc1', batch_id='B-001')

        results = gt.get_batch_traceability('B-001')
        assert len(results) == 2

    def test_matches_material_batch_key(self, gt):
        """Entries with 'material_batch' key should also be matched."""
        gt._entries.append({
            'entry_type': 'MATERIAL_LOADED',
            'material_batch': 'B-X',
            'machine_id': 'cnc1',
        })
        results = gt.get_batch_traceability('B-X')
        assert len(results) == 1

    def test_returns_empty_for_unknown_batch(self, gt):
        gt.record_genealogy_event('MATERIAL_LOADED', 'cnc1', batch_id='B-001')
        assert gt.get_batch_traceability('B-999') == []


class TestFullLifecycle:
    """End-to-end lifecycle: material loaded -> operation -> part complete."""

    def test_full_manufacturing_lifecycle(self, gt):
        serial = 'PART-42'
        tool = 'T10'
        batch = 'BATCH-7'

        gt.record_genealogy_event(
            gt.ENTRY_MATERIAL_LOADED, 'cnc1',
            serial_number=serial, batch_id=batch,
            metadata={'job_id': 'J-1'},
        )
        gt.record_genealogy_event(
            gt.ENTRY_TOOL_INSTALLED, 'cnc1',
            serial_number=serial, tool_id=tool,
        )
        gt.record_genealogy_event(
            gt.ENTRY_OPERATION_COMPLETE, 'cnc1',
            serial_number=serial, tool_id=tool,
            metadata={'operation': 'rough_cut'},
        )
        gt.record_genealogy_event(
            gt.ENTRY_TOOL_REMOVED, 'cnc1',
            serial_number=serial, tool_id=tool,
        )
        gt.record_genealogy_event(
            gt.ENTRY_PART_COMPLETE, 'cnc1',
            serial_number=serial, batch_id=batch,
            metadata={'job_id': 'J-1', 'oee': 0.88},
        )

        # Part history captures all five events
        part_hist = gt.get_part_history(serial)
        assert len(part_hist) == 5

        # Tool history captures install, operation, remove
        tool_hist = gt.get_tool_history(tool)
        assert len(tool_hist) == 3

        # Batch traceability captures load and complete
        batch_hist = gt.get_batch_traceability(batch)
        assert len(batch_hist) == 2

        # Verify chronological order (timestamps are monotonically increasing)
        timestamps = [e['timestamp'] for e in part_hist]
        assert timestamps == sorted(timestamps)
