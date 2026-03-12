"""Tests for the FaultExecutor chaos engineering module.

Verifies safety guardrails, dry-run mode, fault lifecycle,
and all fault type handlers.
"""

import pytest

from miracle_resiliency.fault_executor import (
    FaultExecutor,
    FaultSpec,
    FaultType,
)


@pytest.fixture
def executor():
    """A FaultExecutor in dry_run mode for safe testing."""
    return FaultExecutor(dry_run=True)


class TestDryRunInjection:
    """Tests that dry_run mode accepts faults without executing commands."""

    def test_network_delay_dry_run(self, executor):
        spec = FaultSpec(FaultType.NETWORK_DELAY, target='eth0', duration_sec=10.0)
        assert executor.inject('f1', spec) is True
        assert executor.active_fault_count == 1

    def test_node_kill_dry_run(self, executor):
        spec = FaultSpec(FaultType.NODE_KILL, target='test_node', duration_sec=5.0)
        assert executor.inject('f2', spec) is True

    def test_cpu_stress_dry_run(self, executor):
        spec = FaultSpec(
            FaultType.CPU_STRESS, target='cpu',
            duration_sec=10.0, parameters={'cpu_percent': 90},
        )
        assert executor.inject('f3', spec) is True

    def test_memory_stress_dry_run(self, executor):
        spec = FaultSpec(
            FaultType.MEMORY_STRESS, target='mem',
            duration_sec=10.0, parameters={'memory_percent': 70},
        )
        assert executor.inject('f4', spec) is True

    def test_message_drop_dry_run(self, executor):
        spec = FaultSpec(
            FaultType.MESSAGE_DROP, target='/sensor/data',
            duration_sec=15.0, parameters={'drop_rate': 0.3},
        )
        assert executor.inject('f5', spec) is True


class TestSafetyGuardrails:
    """Tests for safety checks that prevent dangerous fault injection."""

    def test_excluded_node_rejected(self, executor):
        """Faults targeting excluded safety nodes must be rejected."""
        spec = FaultSpec(FaultType.NODE_KILL, target='safety_monitor', duration_sec=10.0)
        assert executor.inject('f1', spec) is False
        assert executor.active_fault_count == 0

    def test_excluded_e_stop_rejected(self, executor):
        spec = FaultSpec(FaultType.NODE_KILL, target='e_stop_controller', duration_sec=5.0)
        assert executor.inject('f2', spec) is False

    def test_custom_excluded_nodes(self):
        ex = FaultExecutor(excluded_nodes={'my_critical_node'}, dry_run=True)
        spec = FaultSpec(FaultType.NODE_KILL, target='my_critical_node', duration_sec=5.0)
        assert ex.inject('f1', spec) is False

    def test_max_duration_clamped(self, executor):
        """Duration exceeding MAX_DURATION should be clamped."""
        spec = FaultSpec(FaultType.NETWORK_DELAY, target='eth0', duration_sec=120.0)
        executor.inject('f1', spec)
        assert spec.duration_sec == 60.0

    def test_zero_duration_rejected(self, executor):
        spec = FaultSpec(FaultType.NETWORK_DELAY, target='eth0', duration_sec=0.0)
        assert executor.inject('f1', spec) is False

    def test_negative_duration_rejected(self, executor):
        spec = FaultSpec(FaultType.NETWORK_DELAY, target='eth0', duration_sec=-5.0)
        assert executor.inject('f1', spec) is False

    def test_custom_max_duration(self):
        ex = FaultExecutor(max_duration_sec=10.0, dry_run=True)
        spec = FaultSpec(FaultType.NETWORK_DELAY, target='eth0', duration_sec=30.0)
        ex.inject('f1', spec)
        assert spec.duration_sec == 10.0

    def test_max_duration_cannot_exceed_hard_limit(self):
        """Even if max_duration_sec > 60, it should be capped at 60."""
        ex = FaultExecutor(max_duration_sec=200.0, dry_run=True)
        spec = FaultSpec(FaultType.NETWORK_DELAY, target='eth0', duration_sec=100.0)
        ex.inject('f1', spec)
        assert spec.duration_sec == 60.0


class TestDuplicateFaultId:
    """Tests that duplicate fault IDs are rejected."""

    def test_duplicate_fault_id_rejected(self, executor):
        spec1 = FaultSpec(FaultType.NETWORK_DELAY, target='eth0', duration_sec=10.0)
        spec2 = FaultSpec(FaultType.CPU_STRESS, target='cpu', duration_sec=10.0)
        assert executor.inject('dup', spec1) is True
        assert executor.inject('dup', spec2) is False
        assert executor.active_fault_count == 1


class TestCleanup:
    """Tests for fault cleanup and removal."""

    def test_cleanup_active_fault(self, executor):
        spec = FaultSpec(FaultType.NETWORK_DELAY, target='eth0', duration_sec=30.0)
        executor.inject('f1', spec)
        assert executor.active_fault_count == 1
        result = executor.cleanup('f1')
        assert result is True
        assert executor.active_fault_count == 0

    def test_cleanup_nonexistent_fault(self, executor):
        assert executor.cleanup('nonexistent') is False

    def test_cleanup_already_cleaned(self, executor):
        spec = FaultSpec(FaultType.NETWORK_DELAY, target='eth0', duration_sec=10.0)
        executor.inject('f1', spec)
        executor.cleanup('f1')
        assert executor.cleanup('f1') is False

    def test_cleanup_all(self, executor):
        for i in range(5):
            spec = FaultSpec(FaultType.MESSAGE_DROP, target=f'/topic_{i}', duration_sec=10.0)
            executor.inject(f'f{i}', spec)
        assert executor.active_fault_count == 5
        count = executor.cleanup_all()
        assert count == 5
        assert executor.active_fault_count == 0

    def test_cleanup_all_empty(self, executor):
        assert executor.cleanup_all() == 0


class TestActiveFaultsProperty:
    """Tests for the active_faults read-only property."""

    def test_active_faults_returns_specs(self, executor):
        spec = FaultSpec(FaultType.CPU_STRESS, target='cpu', duration_sec=10.0)
        executor.inject('f1', spec)
        faults = executor.active_faults
        assert 'f1' in faults
        assert faults['f1'].fault_type == FaultType.CPU_STRESS

    def test_active_faults_empty_initially(self, executor):
        assert executor.active_faults == {}


class TestMessageDropParameters:
    """Tests for message drop parameter handling."""

    def test_drop_rate_clamped_to_range(self, executor):
        spec = FaultSpec(
            FaultType.MESSAGE_DROP, target='/topic',
            duration_sec=10.0, parameters={'drop_rate': 1.5},
        )
        executor.inject('f1', spec)
        assert spec.parameters['_effective_drop_rate'] == 1.0

    def test_negative_drop_rate_clamped(self, executor):
        spec = FaultSpec(
            FaultType.MESSAGE_DROP, target='/topic',
            duration_sec=10.0, parameters={'drop_rate': -0.5},
        )
        executor.inject('f1', spec)
        assert spec.parameters['_effective_drop_rate'] == 0.0

    def test_topic_stored_in_parameters(self, executor):
        spec = FaultSpec(
            FaultType.MESSAGE_DROP, target='/sensor/data',
            duration_sec=10.0,
        )
        executor.inject('f1', spec)
        assert spec.parameters['_topic'] == '/sensor/data'
