"""Tests for HMI Bridge data flow, anomaly buffer, and dashboard snapshot logic.

Tests the data caching, anomaly buffering, and dashboard data assembly
without requiring a live WebSocket server or ROS2 runtime.
"""

import json
import threading
import pytest
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


# ---------------------------------------------------------------------------
# Extracted helpers (mirror the core logic in hmi_bridge.py)
# ---------------------------------------------------------------------------

class _HMIDataStore:
    """Extracted data storage and dashboard assembly logic from HMIBridgeNode.

    Mirrors the thread-safe caching of machine states, KPIs, fleet health,
    and recent anomalies, plus the dashboard data snapshot assembly.
    """

    def __init__(self, max_anomalies: int = 50):
        self._latest_states: Dict[str, Dict] = {}
        self._latest_kpis: Optional[Dict] = None
        self._latest_health: Optional[Dict] = None
        self._recent_anomalies: List[Dict] = []
        self._max_anomalies = max_anomalies
        self._data_lock = threading.Lock()

    def on_state(self, machine_id: str, state_dict: Dict) -> None:
        """Cache latest machine state (mirrors _on_state)."""
        with self._data_lock:
            self._latest_states[machine_id] = state_dict

    def on_kpis(self, kpi_dict: Dict) -> None:
        """Cache latest KPIs (mirrors _on_kpis)."""
        with self._data_lock:
            self._latest_kpis = kpi_dict

    def on_health(self, health_dict: Dict) -> None:
        """Cache latest fleet health (mirrors _on_health)."""
        with self._data_lock:
            self._latest_health = health_dict

    def on_anomaly(self, anomaly_dict: Dict) -> None:
        """Track recent anomalies with buffer limit (mirrors _on_anomaly)."""
        with self._data_lock:
            self._recent_anomalies.append(anomaly_dict)
            if len(self._recent_anomalies) > self._max_anomalies:
                self._recent_anomalies = self._recent_anomalies[-self._max_anomalies:]

    def get_dashboard_data(self) -> Dict[str, Any]:
        """Get current dashboard data snapshot (mirrors get_dashboard_data)."""
        with self._data_lock:
            return {
                'machines': dict(self._latest_states),
                'kpis': self._latest_kpis,
                'fleet_health': self._latest_health,
                'recent_anomalies': list(self._recent_anomalies),
            }

    @property
    def anomaly_count(self) -> int:
        with self._data_lock:
            return len(self._recent_anomalies)


class _WebSocketCommandParser:
    """Extracted command parsing logic from HMIBridgeNode._handle_ws_command."""

    @staticmethod
    def parse_command(message: str) -> Dict:
        """Parse a WebSocket command message.

        Returns:
            Parsed command dict or error dict.
        """
        try:
            cmd = json.loads(message)
            return {'type': 'ack', 'action': cmd.get('action')}
        except json.JSONDecodeError:
            return {'type': 'error', 'message': 'Invalid JSON'}


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def store():
    """Return a fresh HMI data store."""
    return _HMIDataStore(max_anomalies=50)


@pytest.fixture
def small_store():
    """Return a data store with small anomaly buffer for overflow testing."""
    return _HMIDataStore(max_anomalies=5)


@pytest.fixture
def parser():
    """Return a WebSocket command parser."""
    return _WebSocketCommandParser()


# ---------------------------------------------------------------------------
# Test classes
# ---------------------------------------------------------------------------

class TestMachineStateDataFlow:
    """Test machine state data caching and retrieval."""

    def test_single_machine_state(self, store):
        """A single machine state should be cached and retrievable."""
        store.on_state('cnc1', {'status': 'RUNNING', 'spindle_speed': 8000})
        data = store.get_dashboard_data()
        assert 'cnc1' in data['machines']
        assert data['machines']['cnc1']['status'] == 'RUNNING'

    def test_multiple_machine_states(self, store):
        """Multiple machines should be tracked independently."""
        store.on_state('cnc1', {'status': 'RUNNING'})
        store.on_state('cnc2', {'status': 'IDLE'})
        store.on_state('cnc3', {'status': 'PAUSED'})
        data = store.get_dashboard_data()
        assert len(data['machines']) == 3
        assert data['machines']['cnc1']['status'] == 'RUNNING'
        assert data['machines']['cnc2']['status'] == 'IDLE'
        assert data['machines']['cnc3']['status'] == 'PAUSED'

    def test_state_update_overwrites_previous(self, store):
        """Updating a machine state should overwrite the previous one."""
        store.on_state('cnc1', {'status': 'IDLE', 'spindle_speed': 0})
        store.on_state('cnc1', {'status': 'RUNNING', 'spindle_speed': 8000})
        data = store.get_dashboard_data()
        assert data['machines']['cnc1']['status'] == 'RUNNING'
        assert data['machines']['cnc1']['spindle_speed'] == 8000

    def test_empty_dashboard_data(self, store):
        """Empty store should return None/empty values."""
        data = store.get_dashboard_data()
        assert data['machines'] == {}
        assert data['kpis'] is None
        assert data['fleet_health'] is None
        assert data['recent_anomalies'] == []


class TestKPIsAndHealthDataFlow:
    """Test KPI and fleet health caching."""

    def test_kpi_caching(self, store):
        """KPI data should be cached and retrievable."""
        store.on_kpis({'oee': 0.85, 'throughput': 42})
        data = store.get_dashboard_data()
        assert data['kpis']['oee'] == 0.85

    def test_kpi_update(self, store):
        """Updated KPIs should overwrite previous values."""
        store.on_kpis({'oee': 0.75})
        store.on_kpis({'oee': 0.92})
        data = store.get_dashboard_data()
        assert data['kpis']['oee'] == 0.92

    def test_health_caching(self, store):
        """Fleet health data should be cached."""
        store.on_health({'active_nodes': 5, 'degraded_nodes': 1})
        data = store.get_dashboard_data()
        assert data['fleet_health']['active_nodes'] == 5

    def test_health_update(self, store):
        """Updated fleet health should overwrite previous values."""
        store.on_health({'active_nodes': 5})
        store.on_health({'active_nodes': 4})
        data = store.get_dashboard_data()
        assert data['fleet_health']['active_nodes'] == 4


class TestAnomalyBuffer:
    """Test anomaly buffer with ring-buffer behavior."""

    def test_anomaly_stored(self, store):
        """A single anomaly should be stored and retrievable."""
        store.on_anomaly({'type': 'vibration', 'severity': 0.8})
        data = store.get_dashboard_data()
        assert len(data['recent_anomalies']) == 1
        assert data['recent_anomalies'][0]['type'] == 'vibration'

    def test_multiple_anomalies(self, store):
        """Multiple anomalies should accumulate."""
        for i in range(10):
            store.on_anomaly({'id': i, 'severity': 0.5})
        assert store.anomaly_count == 10

    def test_anomaly_buffer_overflow(self, small_store):
        """Buffer should trim to max size when exceeded."""
        for i in range(10):
            small_store.on_anomaly({'id': i})
        assert small_store.anomaly_count == 5
        data = small_store.get_dashboard_data()
        # Should keep the most recent 5 (ids 5-9)
        ids = [a['id'] for a in data['recent_anomalies']]
        assert ids == [5, 6, 7, 8, 9]

    def test_anomaly_buffer_at_exact_limit(self, small_store):
        """At exactly max_anomalies, no trimming should occur."""
        for i in range(5):
            small_store.on_anomaly({'id': i})
        assert small_store.anomaly_count == 5

    def test_anomaly_order_preserved(self, store):
        """Anomalies should be in insertion order."""
        store.on_anomaly({'ts': 1})
        store.on_anomaly({'ts': 2})
        store.on_anomaly({'ts': 3})
        data = store.get_dashboard_data()
        assert [a['ts'] for a in data['recent_anomalies']] == [1, 2, 3]


class TestDashboardDataSnapshot:
    """Test that dashboard data snapshot is a consistent copy."""

    def test_snapshot_is_independent_copy(self, store):
        """Modifying snapshot should not affect store."""
        store.on_state('cnc1', {'status': 'RUNNING'})
        data = store.get_dashboard_data()
        data['machines']['cnc1']['status'] = 'MODIFIED'
        # Original should be unchanged
        fresh_data = store.get_dashboard_data()
        assert fresh_data['machines']['cnc1']['status'] == 'RUNNING'

    def test_snapshot_anomalies_independent(self, store):
        """Modifying snapshot anomalies should not affect store."""
        store.on_anomaly({'id': 1})
        data = store.get_dashboard_data()
        data['recent_anomalies'].append({'id': 999})
        assert store.anomaly_count == 1

    def test_full_dashboard_data(self, store):
        """Full dashboard data should contain all sections."""
        store.on_state('cnc1', {'status': 'RUNNING'})
        store.on_kpis({'oee': 0.9})
        store.on_health({'active_nodes': 3})
        store.on_anomaly({'alert': True})
        data = store.get_dashboard_data()
        assert 'cnc1' in data['machines']
        assert data['kpis']['oee'] == 0.9
        assert data['fleet_health']['active_nodes'] == 3
        assert len(data['recent_anomalies']) == 1

    def test_dashboard_data_serializable(self, store):
        """Dashboard data should be JSON-serializable."""
        store.on_state('cnc1', {'status': 'RUNNING', 'speed': 8000.0})
        store.on_kpis({'oee': 0.85})
        store.on_anomaly({'type': 'vibration'})
        data = store.get_dashboard_data()
        serialized = json.dumps(data)
        assert isinstance(serialized, str)
        deserialized = json.loads(serialized)
        assert deserialized['machines']['cnc1']['status'] == 'RUNNING'


class TestWebSocketCommandParsing:
    """Test WebSocket command parsing."""

    def test_valid_command(self, parser):
        """Valid JSON command should be parsed and acknowledged."""
        result = parser.parse_command('{"action": "start_machine"}')
        assert result['type'] == 'ack'
        assert result['action'] == 'start_machine'

    def test_invalid_json(self, parser):
        """Invalid JSON should return an error response."""
        result = parser.parse_command('not valid json')
        assert result['type'] == 'error'
        assert 'Invalid JSON' in result['message']

    def test_command_without_action(self, parser):
        """Command without action field should still ack with None."""
        result = parser.parse_command('{"data": "test"}')
        assert result['type'] == 'ack'
        assert result['action'] is None

    def test_empty_json_object(self, parser):
        """Empty JSON object should ack with None action."""
        result = parser.parse_command('{}')
        assert result['type'] == 'ack'
        assert result['action'] is None


class TestThreadSafety:
    """Test thread safety of the data store."""

    def test_concurrent_state_updates(self, store):
        """Concurrent state updates should not corrupt data."""
        errors = []

        def update_states(machine_prefix, count):
            try:
                for i in range(count):
                    store.on_state(
                        f'{machine_prefix}_{i % 5}',
                        {'status': 'RUNNING', 'tick': i}
                    )
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=update_states, args=('a', 100)),
            threading.Thread(target=update_states, args=('b', 100)),
            threading.Thread(target=update_states, args=('c', 100)),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0
        data = store.get_dashboard_data()
        assert len(data['machines']) > 0

    def test_concurrent_anomaly_writes(self, small_store):
        """Concurrent anomaly writes should respect buffer limits."""
        errors = []

        def add_anomalies(start, count):
            try:
                for i in range(start, start + count):
                    small_store.on_anomaly({'id': i})
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=add_anomalies, args=(0, 50)),
            threading.Thread(target=add_anomalies, args=(50, 50)),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0
        assert small_store.anomaly_count <= 5
