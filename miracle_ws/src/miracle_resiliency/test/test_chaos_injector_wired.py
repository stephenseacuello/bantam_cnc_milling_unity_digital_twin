"""Tests for ChaosInjectorNode integration with FaultExecutor.

Verifies that the chaos injector properly initializes and delegates
to the FaultExecutor for fault injection and cleanup.
"""

import pytest

from miracle_resiliency.fault_executor import (
    FaultExecutor,
    FaultSpec,
    FaultType,
)


class _FakeRequest:
    """Simulated InjectFault request for testing."""
    def __init__(self, target_node, fault_type, duration_sec=10.0, intensity=0.5):
        self.target_node = target_node
        self.fault_type = fault_type
        self.duration_sec = duration_sec
        self.intensity = intensity


class _FakeResponse:
    """Simulated InjectFault response for testing."""
    def __init__(self):
        self.success = False
        self.fault_id = ''
        self.message = ''


class _ChaosInjectorLogic:
    """Extracted inject/cleanup logic from ChaosInjectorNode.

    Mirrors the fault_executor integration without requiring ROS2.
    """

    FAULT_TYPE_MAP = {
        'network_delay': FaultType.NETWORK_DELAY,
        'node_kill': FaultType.NODE_KILL,
        'cpu_stress': FaultType.CPU_STRESS,
        'memory_stress': FaultType.MEMORY_STRESS,
        'message_drop': FaultType.MESSAGE_DROP,
    }

    def __init__(self, dry_run=True):
        self.fault_executor = FaultExecutor(dry_run=dry_run)
        self.active_faults = {}
        self._next_id = 0

    def inject(self, request):
        """Inject a fault, returning (success, fault_id)."""
        self._next_id += 1
        fault_id = f'f{self._next_id}'

        ft = self.FAULT_TYPE_MAP.get(request.fault_type)
        if ft is None:
            return False, ''

        fault_spec = FaultSpec(
            fault_type=ft,
            target=request.target_node,
            duration_sec=request.duration_sec,
            parameters={'intensity': request.intensity},
        )

        success = self.fault_executor.inject(fault_id, fault_spec)
        if success:
            self.active_faults[fault_id] = {
                'target': request.target_node,
                'type': request.fault_type,
            }
        return success, fault_id

    def remove(self, fault_id):
        """Remove a fault."""
        self.fault_executor.cleanup(fault_id)
        self.active_faults.pop(fault_id, None)


@pytest.fixture
def injector():
    return _ChaosInjectorLogic(dry_run=True)


class TestFaultExecutorInitialization:
    """Tests that the chaos injector properly initializes FaultExecutor."""

    def test_executor_created(self, injector):
        assert injector.fault_executor is not None
        assert isinstance(injector.fault_executor, FaultExecutor)

    def test_executor_dry_run_mode(self, injector):
        """Default initialization should use dry_run=True."""
        assert injector.fault_executor._dry_run is True

    def test_executor_non_dry_run(self):
        logic = _ChaosInjectorLogic(dry_run=False)
        assert logic.fault_executor._dry_run is False


class TestInjectCleanupLifecycle:
    """Tests for the inject -> cleanup lifecycle through the wired system."""

    def test_inject_network_delay(self, injector):
        req = _FakeRequest('eth0', 'network_delay', duration_sec=10.0)
        success, fault_id = injector.inject(req)
        assert success is True
        assert fault_id != ''
        assert injector.fault_executor.active_fault_count == 1

    def test_inject_then_cleanup(self, injector):
        req = _FakeRequest('eth0', 'network_delay', duration_sec=10.0)
        success, fault_id = injector.inject(req)
        assert success is True
        injector.remove(fault_id)
        assert injector.fault_executor.active_fault_count == 0
        assert fault_id not in injector.active_faults

    def test_inject_node_kill(self, injector):
        req = _FakeRequest('test_node', 'node_kill', duration_sec=5.0)
        success, fault_id = injector.inject(req)
        assert success is True

    def test_inject_cpu_stress(self, injector):
        req = _FakeRequest('cpu', 'cpu_stress', duration_sec=10.0, intensity=0.8)
        success, fault_id = injector.inject(req)
        assert success is True

    def test_inject_excluded_node_rejected(self, injector):
        req = _FakeRequest('safety_monitor', 'node_kill', duration_sec=5.0)
        success, fault_id = injector.inject(req)
        assert success is False

    def test_inject_unknown_fault_type(self, injector):
        req = _FakeRequest('node', 'unknown_type', duration_sec=5.0)
        success, fault_id = injector.inject(req)
        assert success is False

    def test_multiple_inject_and_cleanup_all(self, injector):
        for ft in ['network_delay', 'cpu_stress', 'message_drop']:
            req = _FakeRequest(f'target_{ft}', ft, duration_sec=10.0)
            injector.inject(req)
        assert injector.fault_executor.active_fault_count == 3
        count = injector.fault_executor.cleanup_all()
        assert count == 3
        assert injector.fault_executor.active_fault_count == 0

    def test_remove_nonexistent_fault(self, injector):
        """Removing a fault that doesn't exist should not raise."""
        injector.remove('nonexistent')  # Should not raise
