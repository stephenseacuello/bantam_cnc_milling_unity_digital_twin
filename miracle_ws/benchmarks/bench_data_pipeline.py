"""
Performance benchmarks for the MIRACLE data pipeline.

Measures throughput and latency of:
- HMI data store operations
- JSON serialization of dashboard snapshots
- Config loader resolution
- Message routing throughput

Run: python3 -m pytest benchmarks/bench_data_pipeline.py -v
"""

import copy
import json
import os
import time
import threading
import pytest


class _HMIDataStore:
    """Extracted data store for benchmarking (mirrors hmi_bridge.py)."""
    def __init__(self, max_anomalies=50):
        self._latest_states = {}
        self._latest_kpis = None
        self._latest_health = None
        self._recent_anomalies = []
        self._max_anomalies = max_anomalies
        self._data_lock = threading.Lock()

    def on_state(self, machine_id, state_dict):
        with self._data_lock:
            self._latest_states[machine_id] = state_dict

    def on_kpis(self, kpi_dict):
        with self._data_lock:
            self._latest_kpis = kpi_dict

    def on_anomaly(self, anomaly_dict):
        with self._data_lock:
            self._recent_anomalies.append(anomaly_dict)
            if len(self._recent_anomalies) > self._max_anomalies:
                self._recent_anomalies = self._recent_anomalies[-self._max_anomalies:]

    def get_dashboard_data(self):
        with self._data_lock:
            return {
                'type': 'update',
                'machines': copy.deepcopy(self._latest_states),
                'kpis': copy.deepcopy(self._latest_kpis),
                'fleet_health': copy.deepcopy(self._latest_health),
                'recent_anomalies': copy.deepcopy(self._recent_anomalies),
            }


SAMPLE_STATE = {
    'machine_id': 'cnc1',
    'status': 'RUNNING',
    'spindle_speed': 8000.0,
    'feed_rate': 500.0,
    'spindle_load': 42.5,
    'current_line': 150,
    'axis_positions': [10.0, 20.0, -5.0],
    'program_name': 'pocket_50x50.nc',
}

SAMPLE_KPIS = {
    'oee': 0.87,
    'availability': 0.92,
    'performance': 0.95,
    'quality': 0.99,
    'throughput': 42,
}

SAMPLE_ANOMALY = {
    'machine_id': 'cnc1',
    'anomaly_type': 'vibration_spike',
    'severity': 0.85,
    'description': 'Vibration exceeded threshold',
}


class TestStateIngestionThroughput:
    """Benchmark: How many state updates/sec can the store handle?"""

    def test_single_thread_10k_states(self):
        store = _HMIDataStore()
        count = 10_000
        start = time.perf_counter()
        for i in range(count):
            store.on_state(f'cnc{i % 3 + 1}', {**SAMPLE_STATE, 'current_line': i})
        elapsed = time.perf_counter() - start
        rate = count / elapsed
        print(f"\n  Single-thread state ingestion: {rate:,.0f} ops/sec ({elapsed:.3f}s)")
        assert rate > 50_000, f"State ingestion too slow: {rate:.0f} ops/sec"

    def test_multi_thread_30k_states(self):
        store = _HMIDataStore()
        count_per_thread = 10_000
        errors = []

        def ingest(prefix, n):
            try:
                for i in range(n):
                    store.on_state(f'{prefix}_{i % 3}', {**SAMPLE_STATE, 'current_line': i})
            except Exception as e:
                errors.append(e)

        start = time.perf_counter()
        threads = [threading.Thread(target=ingest, args=(f't{t}', count_per_thread)) for t in range(3)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        elapsed = time.perf_counter() - start
        total = count_per_thread * 3
        rate = total / elapsed
        print(f"\n  Multi-thread state ingestion (3 threads): {rate:,.0f} ops/sec ({elapsed:.3f}s)")
        assert len(errors) == 0
        assert rate > 30_000


class TestDashboardSnapshotPerformance:
    """Benchmark: Dashboard snapshot + JSON serialization."""

    def test_snapshot_json_round_trip(self):
        store = _HMIDataStore()
        for i in range(3):
            store.on_state(f'cnc{i+1}', SAMPLE_STATE)
        store.on_kpis(SAMPLE_KPIS)
        for i in range(50):
            store.on_anomaly({**SAMPLE_ANOMALY, 'id': i})

        count = 5_000
        start = time.perf_counter()
        for _ in range(count):
            data = store.get_dashboard_data()
            json.dumps(data, default=str)
        elapsed = time.perf_counter() - start
        rate = count / elapsed
        print(f"\n  Snapshot+JSON serialization: {rate:,.0f} ops/sec ({elapsed:.3f}s)")
        assert rate > 1_000


class TestAnomalyBufferPerformance:
    """Benchmark: Anomaly ring buffer under pressure."""

    def test_anomaly_buffer_50k_inserts(self):
        store = _HMIDataStore(max_anomalies=50)
        count = 50_000
        start = time.perf_counter()
        for i in range(count):
            store.on_anomaly({'id': i, 'severity': 0.5})
        elapsed = time.perf_counter() - start
        rate = count / elapsed
        print(f"\n  Anomaly buffer throughput: {rate:,.0f} ops/sec ({elapsed:.3f}s)")
        assert rate > 100_000


class TestConfigResolutionPerformance:
    """Benchmark: Config resolution speed."""

    def test_config_resolution_10k_lookups(self):
        # Inline config resolver
        config = {
            'network': {'unity_tcp_port': 10000, 'websocket_port': 9090},
            'ros2': {'domain_id': 42},
        }

        def get_default(section, key, fallback=None):
            env_key = f"MIRACLE_{section.upper()}_{key.upper()}"
            env_val = os.environ.get(env_key)
            if env_val is not None:
                return env_val
            section_data = config.get(section, {})
            if isinstance(section_data, dict) and key in section_data:
                return section_data[key]
            return fallback

        count = 100_000
        start = time.perf_counter()
        for _ in range(count):
            get_default('network', 'unity_tcp_port', 0)
            get_default('ros2', 'domain_id', 0)
            get_default('missing', 'key', 'default')
        elapsed = time.perf_counter() - start
        rate = (count * 3) / elapsed
        print(f"\n  Config resolution: {rate:,.0f} lookups/sec ({elapsed:.3f}s)")
        assert rate > 200_000
