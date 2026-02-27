"""End-to-end pipeline tests for the MIRACLE data flow.

Validates that:
- Machine state flows through the HMI data store correctly
- WebSocket broadcast assembles valid JSON payloads
- Config loader resolves values with correct precedence
- Dashboard data snapshots match expected schema
"""

import json
import os
import threading
import pytest
from typing import Any, Dict, List, Optional


# ---------------------------------------------------------------------------
# Extracted helper: HMI Data Store (mirrors hmi_bridge.py core logic)
# ---------------------------------------------------------------------------

class _HMIDataStore:
    """Thread-safe data store mirroring the HMI bridge caching layer."""

    def __init__(self, max_anomalies: int = 50):
        self._latest_states: Dict[str, Dict] = {}
        self._latest_kpis: Optional[Dict] = None
        self._latest_health: Optional[Dict] = None
        self._recent_anomalies: List[Dict] = []
        self._max_anomalies = max_anomalies
        self._data_lock = threading.Lock()

    def on_state(self, machine_id: str, state_dict: Dict) -> None:
        with self._data_lock:
            self._latest_states[machine_id] = state_dict

    def on_kpis(self, kpi_dict: Dict) -> None:
        with self._data_lock:
            self._latest_kpis = kpi_dict

    def on_health(self, health_dict: Dict) -> None:
        with self._data_lock:
            self._latest_health = health_dict

    def on_anomaly(self, anomaly_dict: Dict) -> None:
        with self._data_lock:
            self._recent_anomalies.append(anomaly_dict)
            if len(self._recent_anomalies) > self._max_anomalies:
                self._recent_anomalies = self._recent_anomalies[-self._max_anomalies:]

    def get_dashboard_data(self) -> Dict[str, Any]:
        with self._data_lock:
            return {
                'type': 'update',
                'machines': dict(self._latest_states),
                'kpis': self._latest_kpis,
                'fleet_health': self._latest_health,
                'recent_anomalies': list(self._recent_anomalies),
            }


# ---------------------------------------------------------------------------
# Extracted helper: WebSocket Broadcast Simulator
# ---------------------------------------------------------------------------

class _BroadcastSimulator:
    """Simulates the WebSocket broadcast pipeline from hmi_bridge.py.

    Collects JSON payloads that would be sent to clients.
    """

    def __init__(self, store: _HMIDataStore):
        self._store = store
        self.sent_payloads: List[str] = []

    def broadcast(self) -> str:
        """Simulate a broadcast: serialize dashboard data and return the JSON."""
        data = self._store.get_dashboard_data()
        payload = json.dumps(data, default=str)
        self.sent_payloads.append(payload)
        return payload


# ---------------------------------------------------------------------------
# Extracted helper: Config Loader logic (mirrors config_loader.py)
# ---------------------------------------------------------------------------

class _ConfigResolver:
    """Extracted config resolution logic from config_loader.py."""

    def __init__(self, yaml_data: Optional[Dict] = None):
        self._config = yaml_data or {}

    def get_default(self, section: str, key: str, fallback: Any = None) -> Any:
        env_key = f"MIRACLE_{section.upper()}_{key.upper()}"
        env_val = os.environ.get(env_key)
        if env_val is not None:
            return self._coerce(env_val, fallback)

        section_data = self._config.get(section, {})
        if isinstance(section_data, dict) and key in section_data:
            return section_data[key]

        return fallback

    @staticmethod
    def _coerce(value: str, reference: Any) -> Any:
        if reference is None:
            return value
        if isinstance(reference, bool):
            return value.lower() in ("true", "1", "yes")
        if isinstance(reference, int):
            try:
                return int(value)
            except ValueError:
                return int(float(value))
        if isinstance(reference, float):
            return float(value)
        return value


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def store():
    return _HMIDataStore(max_anomalies=50)


@pytest.fixture
def broadcaster(store):
    return _BroadcastSimulator(store)


@pytest.fixture
def config():
    return _ConfigResolver({
        'network': {
            'unity_tcp_port': 10000,
            'websocket_port': 9090,
        },
        'ros2': {
            'domain_id': 42,
        },
        'machines': {
            'ids': ['cnc1', 'cnc2', 'cnc3'],
        },
    })


# ---------------------------------------------------------------------------
# E2E State Flow Tests
# ---------------------------------------------------------------------------

class TestStateFlowToDataStore:
    """Verifies that machine states propagate through the data store."""

    def test_state_flows_to_dashboard_data(self, store):
        """MachineState → HMI store → dashboard data should carry all fields."""
        state = {
            'machine_id': 'cnc1',
            'status': 'RUNNING',
            'spindle_speed': 8000.0,
            'feed_rate': 500.0,
            'spindle_load': 42.5,
            'current_line': 150,
            'axis_positions': [10.0, 20.0, -5.0],
        }
        store.on_state('cnc1', state)
        data = store.get_dashboard_data()

        assert data['machines']['cnc1']['status'] == 'RUNNING'
        assert data['machines']['cnc1']['spindle_speed'] == 8000.0
        assert data['machines']['cnc1']['axis_positions'] == [10.0, 20.0, -5.0]

    def test_kpis_flow_to_dashboard_data(self, store):
        """SystemKPIs → HMI store → dashboard data."""
        kpis = {
            'oee': 0.87,
            'availability': 0.92,
            'performance': 0.95,
            'quality': 0.99,
            'throughput': 42,
        }
        store.on_kpis(kpis)
        data = store.get_dashboard_data()

        assert data['kpis']['oee'] == 0.87
        assert data['kpis']['availability'] == 0.92

    def test_anomaly_flows_to_dashboard_data(self, store):
        """AnomalyAlert → HMI store → dashboard data."""
        anomaly = {
            'machine_id': 'cnc1',
            'anomaly_type': 'vibration_spike',
            'severity': 0.85,
            'description': 'Vibration exceeded threshold at 12.3 mm/s²',
        }
        store.on_anomaly(anomaly)
        data = store.get_dashboard_data()

        assert len(data['recent_anomalies']) == 1
        assert data['recent_anomalies'][0]['anomaly_type'] == 'vibration_spike'
        assert data['recent_anomalies'][0]['severity'] == 0.85

    def test_multi_machine_state_isolation(self, store):
        """States from different machines should be independent."""
        store.on_state('cnc1', {'status': 'RUNNING', 'spindle_speed': 8000})
        store.on_state('cnc2', {'status': 'IDLE', 'spindle_speed': 0})
        store.on_state('cnc3', {'status': 'PAUSED', 'spindle_speed': 5000})

        data = store.get_dashboard_data()
        assert len(data['machines']) == 3
        assert data['machines']['cnc1']['status'] == 'RUNNING'
        assert data['machines']['cnc2']['status'] == 'IDLE'
        assert data['machines']['cnc3']['status'] == 'PAUSED'

    def test_rapid_state_updates_last_write_wins(self, store):
        """Rapid updates to the same machine should keep only the latest."""
        for rpm in range(0, 10000, 100):
            store.on_state('cnc1', {'status': 'RUNNING', 'spindle_speed': rpm})

        data = store.get_dashboard_data()
        assert data['machines']['cnc1']['spindle_speed'] == 9900


# ---------------------------------------------------------------------------
# WebSocket Broadcast JSON Tests
# ---------------------------------------------------------------------------

class TestWebSocketBroadcastJSON:
    """Verifies that broadcast payloads are valid JSON with correct schema."""

    def test_broadcast_sends_valid_json(self, store, broadcaster):
        """Broadcast output should be valid JSON."""
        store.on_state('cnc1', {'status': 'RUNNING'})
        payload = broadcaster.broadcast()
        data = json.loads(payload)
        assert isinstance(data, dict)

    def test_broadcast_schema_has_required_keys(self, store, broadcaster):
        """Broadcast JSON must contain all dashboard sections."""
        store.on_state('cnc1', {'status': 'IDLE'})
        store.on_kpis({'oee': 0.85})
        store.on_anomaly({'type': 'test'})

        payload = broadcaster.broadcast()
        data = json.loads(payload)

        assert 'type' in data
        assert data['type'] == 'update'
        assert 'machines' in data
        assert 'kpis' in data
        assert 'fleet_health' in data
        assert 'recent_anomalies' in data

    def test_broadcast_machines_is_dict(self, store, broadcaster):
        """machines field should be a dict keyed by machine_id."""
        store.on_state('cnc1', {'status': 'RUNNING'})
        store.on_state('cnc2', {'status': 'IDLE'})
        payload = broadcaster.broadcast()
        data = json.loads(payload)

        assert isinstance(data['machines'], dict)
        assert 'cnc1' in data['machines']
        assert 'cnc2' in data['machines']

    def test_broadcast_anomalies_is_list(self, store, broadcaster):
        """recent_anomalies should be a list."""
        store.on_anomaly({'id': 1})
        store.on_anomaly({'id': 2})
        payload = broadcaster.broadcast()
        data = json.loads(payload)

        assert isinstance(data['recent_anomalies'], list)
        assert len(data['recent_anomalies']) == 2

    def test_broadcast_empty_store_valid_json(self, store, broadcaster):
        """Empty store should still produce valid JSON."""
        payload = broadcaster.broadcast()
        data = json.loads(payload)

        assert data['machines'] == {}
        assert data['kpis'] is None
        assert data['recent_anomalies'] == []

    def test_consecutive_broadcasts_reflect_updates(self, store, broadcaster):
        """Each broadcast should reflect the latest data."""
        store.on_state('cnc1', {'status': 'IDLE'})
        p1 = json.loads(broadcaster.broadcast())
        assert p1['machines']['cnc1']['status'] == 'IDLE'

        store.on_state('cnc1', {'status': 'RUNNING'})
        p2 = json.loads(broadcaster.broadcast())
        assert p2['machines']['cnc1']['status'] == 'RUNNING'


# ---------------------------------------------------------------------------
# Config Loader E2E Tests
# ---------------------------------------------------------------------------

class TestConfigLoaderE2E:
    """Verifies config loader precedence: env var > YAML > fallback."""

    def test_yaml_value_returned(self, config):
        """Known YAML value should be returned."""
        assert config.get_default('network', 'unity_tcp_port', 0) == 10000

    def test_yaml_nested_section(self, config):
        """Nested YAML section should resolve."""
        assert config.get_default('ros2', 'domain_id', 0) == 42

    def test_fallback_when_key_missing(self, config):
        """Missing key should return fallback."""
        assert config.get_default('network', 'nonexistent_key', 9999) == 9999

    def test_fallback_when_section_missing(self, config):
        """Missing section should return fallback."""
        assert config.get_default('missing_section', 'key', 'default') == 'default'

    def test_env_var_overrides_yaml(self, config, monkeypatch):
        """Environment variable should take precedence over YAML."""
        monkeypatch.setenv('MIRACLE_NETWORK_UNITY_TCP_PORT', '20000')
        result = config.get_default('network', 'unity_tcp_port', 0)
        assert result == 20000

    def test_env_var_coercion_int(self, config, monkeypatch):
        """Int fallback should coerce env string to int."""
        monkeypatch.setenv('MIRACLE_ROS2_DOMAIN_ID', '99')
        result = config.get_default('ros2', 'domain_id', 0)
        assert result == 99
        assert isinstance(result, int)

    def test_env_var_coercion_float(self, config, monkeypatch):
        """Float fallback should coerce env string to float."""
        monkeypatch.setenv('MIRACLE_NETWORK_TIMEOUT', '2.5')
        result = config.get_default('network', 'timeout', 1.0)
        assert result == 2.5
        assert isinstance(result, float)

    def test_env_var_coercion_bool_true(self, config, monkeypatch):
        """Bool fallback should coerce 'true' to True."""
        monkeypatch.setenv('MIRACLE_NETWORK_ENABLE_TLS', 'true')
        result = config.get_default('network', 'enable_tls', False)
        assert result is True

    def test_env_var_coercion_bool_false(self, config, monkeypatch):
        """Bool fallback should coerce 'false' to False."""
        monkeypatch.setenv('MIRACLE_NETWORK_ENABLE_TLS', 'false')
        result = config.get_default('network', 'enable_tls', True)
        assert result is False

    def test_machines_list_from_yaml(self, config):
        """List values in YAML should be returned as-is."""
        ids = config.get_default('machines', 'ids', [])
        assert ids == ['cnc1', 'cnc2', 'cnc3']


# ---------------------------------------------------------------------------
# Full Pipeline Integration Test
# ---------------------------------------------------------------------------

class TestFullPipelineIntegration:
    """Simulate a complete data pipeline: state → store → broadcast → verify."""

    def test_full_pipeline_state_to_broadcast(self, store, broadcaster):
        """Simulate: 3 machines → state updates → KPIs → anomaly → broadcast."""
        # Simulate machine states arriving
        store.on_state('cnc1', {
            'machine_id': 'cnc1',
            'status': 'RUNNING',
            'spindle_speed': 8000.0,
            'feed_rate': 500.0,
        })
        store.on_state('cnc2', {
            'machine_id': 'cnc2',
            'status': 'IDLE',
            'spindle_speed': 0.0,
            'feed_rate': 0.0,
        })
        store.on_state('cnc3', {
            'machine_id': 'cnc3',
            'status': 'PAUSED',
            'spindle_speed': 5000.0,
            'feed_rate': 250.0,
        })

        # Simulate KPIs
        store.on_kpis({
            'oee': 0.87,
            'availability': 0.92,
            'performance': 0.95,
            'quality': 0.99,
        })

        # Simulate fleet health
        store.on_health({
            'active_nodes': 12,
            'degraded_nodes': 1,
            'total_nodes': 15,
        })

        # Simulate anomaly
        store.on_anomaly({
            'machine_id': 'cnc1',
            'anomaly_type': 'tool_wear_critical',
            'severity': 0.92,
        })

        # Broadcast and verify
        payload = broadcaster.broadcast()
        data = json.loads(payload)

        # Verify complete schema
        assert data['type'] == 'update'
        assert len(data['machines']) == 3
        assert data['machines']['cnc1']['status'] == 'RUNNING'
        assert data['machines']['cnc2']['status'] == 'IDLE'
        assert data['machines']['cnc3']['status'] == 'PAUSED'
        assert data['kpis']['oee'] == 0.87
        assert data['fleet_health']['active_nodes'] == 12
        assert len(data['recent_anomalies']) == 1
        assert data['recent_anomalies'][0]['severity'] == 0.92

    def test_concurrent_pipeline_no_data_corruption(self, store, broadcaster):
        """Concurrent writes + broadcast should not corrupt data."""
        errors = []

        def write_states(prefix: str, count: int):
            try:
                for i in range(count):
                    store.on_state(f'{prefix}_{i % 3}', {
                        'status': 'RUNNING',
                        'spindle_speed': float(i * 100),
                    })
            except Exception as e:
                errors.append(e)

        def write_anomalies(count: int):
            try:
                for i in range(count):
                    store.on_anomaly({'id': i, 'severity': 0.5})
            except Exception as e:
                errors.append(e)

        def read_broadcasts(count: int):
            try:
                for _ in range(count):
                    payload = broadcaster.broadcast()
                    data = json.loads(payload)
                    assert isinstance(data['machines'], dict)
                    assert isinstance(data['recent_anomalies'], list)
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=write_states, args=('cnc', 200)),
            threading.Thread(target=write_anomalies, args=(100,)),
            threading.Thread(target=read_broadcasts, args=(50,)),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0, f"Pipeline errors: {errors}"
