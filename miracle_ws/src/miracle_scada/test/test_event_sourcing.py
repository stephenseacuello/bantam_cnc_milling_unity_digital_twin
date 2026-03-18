"""Tests for EventSourcingStore — append-only event log with replay capability.

Uses the same mock pattern as test_capability_profiler.py so that ROS2 /
miracle_core modules are stubbed out.
"""

import sys
from unittest.mock import MagicMock

# Mock ROS2 and miracle_core sub-module dependencies before importing
for _mod in ['rclpy', 'rclpy.node', 'rclpy.callback_groups', 'rclpy.qos', 'rclpy.lifecycle',
             'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
             'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
             'miracle_core.exceptions',
             'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(_mod, MagicMock())

sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = type('FakeNode', (), {
    'CRITICALITY_HIGH': 'HIGH',
    'CRITICALITY_MEDIUM': 'MEDIUM',
    '__init__': lambda self, *a, **kw: None,
    'get_logger': lambda self: MagicMock(),
    'create_publisher': lambda self, *a, **kw: MagicMock(),
    'create_subscription': lambda self, *a, **kw: MagicMock(),
    'create_timer': lambda self, *a, **kw: MagicMock(),
    'declare_and_validate_parameters': lambda self, specs: {k: MagicMock(value=v['default']) for k, v in specs.items()},
    'get_parameter': lambda self, name: MagicMock(value=0),
})

sys.modules.pop('miracle_scada.kpi_calculator', None)

import time
import uuid
import pytest

from miracle_scada.kpi_calculator import (
    ManufacturingEvent,
    EventStream,
    EventSourcingStore,
    MANUFACTURING_EVENT_TYPES,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_event(
    event_type: str = 'job_started',
    aggregate_id: str = 'machine_1',
    version: int = 1,
    timestamp: float = 100.0,
    data: dict = None,
    source: str = 'test',
    event_id: str = None,
) -> ManufacturingEvent:
    return ManufacturingEvent(
        event_id=event_id or str(uuid.uuid4()),
        event_type=event_type,
        aggregate_id=aggregate_id,
        timestamp=timestamp,
        data=data if data is not None else {},
        version=version,
        source=source,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestManufacturingEventDataclass:
    """Verify the ManufacturingEvent dataclass fields."""

    def test_fields_present(self):
        evt = _make_event(event_id='e1', event_type='tool_changed',
                          aggregate_id='m1', timestamp=1.0,
                          data={'tool': 'T5'}, version=1, source='cnc')
        assert evt.event_id == 'e1'
        assert evt.event_type == 'tool_changed'
        assert evt.aggregate_id == 'm1'
        assert evt.timestamp == 1.0
        assert evt.data == {'tool': 'T5'}
        assert evt.version == 1
        assert evt.source == 'cnc'


class TestEventSourcingStoreAppend:
    """Tests for append_event and version validation."""

    def test_append_single_event(self):
        store = EventSourcingStore()
        evt = _make_event(version=1)
        store.append_event(evt)
        assert store.get_event_count() == 1

    def test_append_sequential_versions(self):
        store = EventSourcingStore()
        for v in range(1, 6):
            store.append_event(_make_event(version=v, timestamp=float(v)))
        assert store.get_event_count() == 5

    def test_reject_invalid_first_version(self):
        store = EventSourcingStore()
        with pytest.raises(ValueError, match="must have version 1"):
            store.append_event(_make_event(version=3))

    def test_reject_out_of_order_version(self):
        store = EventSourcingStore()
        store.append_event(_make_event(version=1))
        with pytest.raises(ValueError, match="Version mismatch"):
            store.append_event(_make_event(version=5))

    def test_reject_unknown_event_type(self):
        store = EventSourcingStore()
        with pytest.raises(ValueError, match="Unknown event type"):
            store.append_event(_make_event(event_type='invalid_type'))


class TestEventSourcingStoreQueries:
    """Tests for query methods."""

    def _populated_store(self) -> EventSourcingStore:
        """Return a store with events across two aggregates."""
        store = EventSourcingStore()
        # machine_1: 3 events
        store.append_event(_make_event(
            aggregate_id='machine_1', version=1, event_type='job_started',
            timestamp=100.0, data={'job': 'J1'}))
        store.append_event(_make_event(
            aggregate_id='machine_1', version=2, event_type='tool_changed',
            timestamp=110.0, data={'tool': 'T2'}))
        store.append_event(_make_event(
            aggregate_id='machine_1', version=3, event_type='job_completed',
            timestamp=120.0, data={'result': 'ok'}))
        # machine_2: 2 events
        store.append_event(_make_event(
            aggregate_id='machine_2', version=1, event_type='alarm_raised',
            timestamp=105.0, data={'alarm': 'overheat'}))
        store.append_event(_make_event(
            aggregate_id='machine_2', version=2, event_type='alarm_cleared',
            timestamp=115.0, data={'alarm': 'overheat'}))
        return store

    def test_get_events_returns_all_for_aggregate(self):
        store = self._populated_store()
        events = store.get_events('machine_1')
        assert len(events) == 3
        assert [e.version for e in events] == [1, 2, 3]

    def test_get_events_unknown_aggregate_returns_empty(self):
        store = self._populated_store()
        assert store.get_events('nonexistent') == []

    def test_get_events_by_type(self):
        store = self._populated_store()
        alarms = store.get_events_by_type('alarm_raised')
        assert len(alarms) == 1
        assert alarms[0].aggregate_id == 'machine_2'

    def test_get_events_in_range(self):
        store = self._populated_store()
        events = store.get_events_in_range(105.0, 115.0)
        assert len(events) == 3  # timestamps 110, 105, 115
        timestamps = sorted(e.timestamp for e in events)
        assert timestamps == [105.0, 110.0, 115.0]

    def test_get_events_in_range_empty(self):
        store = self._populated_store()
        events = store.get_events_in_range(200.0, 300.0)
        assert events == []

    def test_get_aggregate_ids(self):
        store = self._populated_store()
        assert store.get_aggregate_ids() == ['machine_1', 'machine_2']

    def test_get_event_count(self):
        store = self._populated_store()
        assert store.get_event_count() == 5


class TestEventSourcingStoreReplay:
    """Tests for replay and snapshot functionality."""

    def _store_with_param_changes(self) -> EventSourcingStore:
        store = EventSourcingStore()
        store.append_event(_make_event(
            aggregate_id='cnc_3', version=1, event_type='job_started',
            timestamp=10.0, data={'job_id': 'J100', 'status': 'running'}))
        store.append_event(_make_event(
            aggregate_id='cnc_3', version=2, event_type='parameter_changed',
            timestamp=20.0, data={'spindle_speed': 12000}))
        store.append_event(_make_event(
            aggregate_id='cnc_3', version=3, event_type='parameter_changed',
            timestamp=30.0, data={'feed_rate': 500}))
        store.append_event(_make_event(
            aggregate_id='cnc_3', version=4, event_type='measurement_taken',
            timestamp=40.0, data={'diameter': 25.01}))
        store.append_event(_make_event(
            aggregate_id='cnc_3', version=5, event_type='job_completed',
            timestamp=50.0, data={'status': 'completed', 'parts': 42}))
        return store

    def test_replay_all(self):
        store = self._store_with_param_changes()
        events = store.replay('cnc_3')
        assert len(events) == 5

    def test_replay_up_to_version(self):
        store = self._store_with_param_changes()
        events = store.replay('cnc_3', up_to_version=3)
        assert len(events) == 3
        assert events[-1].version == 3

    def test_replay_unknown_aggregate(self):
        store = self._store_with_param_changes()
        assert store.replay('unknown') == []

    def test_snapshot_merges_data(self):
        store = self._store_with_param_changes()
        snapshot = store.get_snapshot('cnc_3')
        assert snapshot['aggregate_id'] == 'cnc_3'
        assert snapshot['current_version'] == 5
        assert snapshot['event_count'] == 5
        state = snapshot['state']
        # Later events overwrite earlier keys
        assert state['status'] == 'completed'
        assert state['spindle_speed'] == 12000
        assert state['feed_rate'] == 500
        assert state['diameter'] == 25.01
        assert state['parts'] == 42
        assert state['job_id'] == 'J100'

    def test_snapshot_empty_aggregate(self):
        store = EventSourcingStore()
        assert store.get_snapshot('nope') == {}


class TestEventStreamDataclass:
    """Verify the EventStream dataclass."""

    def test_defaults(self):
        stream = EventStream(aggregate_id='agg_1')
        assert stream.aggregate_id == 'agg_1'
        assert stream.events == []
        assert stream.current_version == 0


class TestManufacturingEventTypes:
    """Verify the canonical set of event types."""

    def test_all_types_present(self):
        expected = {
            'job_started', 'job_completed', 'tool_changed',
            'alarm_raised', 'alarm_cleared', 'parameter_changed',
            'measurement_taken',
        }
        assert MANUFACTURING_EVENT_TYPES == expected
