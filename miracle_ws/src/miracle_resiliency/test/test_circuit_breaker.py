"""Tests for CircuitBreaker pattern.

Covers closed-state pass-through, failure counting and tripping, open-state
rejection, half-open probe logic, reset, metrics tracking, the registry,
and edge cases.
"""

import sys
import time
from unittest.mock import MagicMock, patch

for mod in ['miracle_core.datatypes', 'miracle_core.constants',
            'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
            'rclpy.callback_groups',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_msgs', 'miracle_msgs.msg',
            'std_msgs', 'std_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_resiliency.recovery_orchestrator import (
    CircuitBreaker,
    CircuitBreakerConfig,
    CircuitBreakerMetrics,
    CircuitBreakerRegistry,
    CircuitBreakerState,
    CircuitOpenError,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_breaker(**overrides) -> CircuitBreaker:
    """Create a breaker with short thresholds suitable for testing."""
    defaults = dict(
        failure_threshold=3,
        reset_timeout_sec=1.0,
        half_open_max_calls=3,
        success_threshold=2,
    )
    defaults.update(overrides)
    return CircuitBreaker(CircuitBreakerConfig(**defaults))


def _failing_func():
    raise RuntimeError('boom')


def _succeeding_func(value=42):
    return value


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestCircuitBreakerClosedState:
    """Tests for normal (CLOSED) operation."""

    def test_successful_call_passes_through(self):
        cb = _make_breaker()
        result = cb.call(_succeeding_func, value=99)
        assert result == 99
        assert cb.get_state() == CircuitBreakerState.CLOSED

    def test_failures_below_threshold_stay_closed(self):
        cb = _make_breaker(failure_threshold=3)
        for _ in range(2):
            with pytest.raises(RuntimeError):
                cb.call(_failing_func)
        assert cb.get_state() == CircuitBreakerState.CLOSED

    def test_failures_at_threshold_trip_to_open(self):
        cb = _make_breaker(failure_threshold=3)
        for _ in range(3):
            with pytest.raises(RuntimeError):
                cb.call(_failing_func)
        assert cb.get_state() == CircuitBreakerState.OPEN


class TestCircuitBreakerOpenState:
    """Tests for OPEN state behaviour."""

    def test_open_state_rejects_calls(self):
        cb = _make_breaker(failure_threshold=1)
        with pytest.raises(RuntimeError):
            cb.call(_failing_func)
        # Now OPEN -- next call should raise CircuitOpenError
        with pytest.raises(CircuitOpenError):
            cb.call(_succeeding_func)

    def test_open_transitions_to_half_open_after_timeout(self):
        cb = _make_breaker(failure_threshold=1, reset_timeout_sec=0.05)
        with pytest.raises(RuntimeError):
            cb.call(_failing_func)
        assert cb.get_state() == CircuitBreakerState.OPEN
        time.sleep(0.1)
        assert cb.get_state() == CircuitBreakerState.HALF_OPEN


class TestCircuitBreakerHalfOpenState:
    """Tests for HALF_OPEN probe logic."""

    def test_half_open_success_threshold_closes_circuit(self):
        cb = _make_breaker(
            failure_threshold=1,
            reset_timeout_sec=0.05,
            success_threshold=2,
        )
        # Trip to OPEN
        with pytest.raises(RuntimeError):
            cb.call(_failing_func)
        time.sleep(0.1)
        # Now HALF_OPEN -- two successes should close it
        cb.call(_succeeding_func)
        cb.call(_succeeding_func)
        assert cb.get_state() == CircuitBreakerState.CLOSED

    def test_half_open_failure_reopens_circuit(self):
        cb = _make_breaker(
            failure_threshold=1,
            reset_timeout_sec=0.05,
            success_threshold=2,
        )
        # Trip to OPEN
        with pytest.raises(RuntimeError):
            cb.call(_failing_func)
        time.sleep(0.1)
        # Now HALF_OPEN -- a failure should reopen
        with pytest.raises(RuntimeError):
            cb.call(_failing_func)
        assert cb.get_state() == CircuitBreakerState.OPEN


class TestCircuitBreakerReset:
    """Tests for manual reset."""

    def test_reset_returns_to_closed(self):
        cb = _make_breaker(failure_threshold=1)
        with pytest.raises(RuntimeError):
            cb.call(_failing_func)
        assert cb.get_state() == CircuitBreakerState.OPEN
        cb.reset()
        assert cb.get_state() == CircuitBreakerState.CLOSED
        # Should be able to call again
        result = cb.call(_succeeding_func)
        assert result == 42


class TestCircuitBreakerMetrics:
    """Tests for metrics reporting."""

    def test_metrics_reflect_activity(self):
        cb = _make_breaker(failure_threshold=3)
        cb.call(_succeeding_func)
        cb.call(_succeeding_func)
        with pytest.raises(RuntimeError):
            cb.call(_failing_func)

        metrics = cb.get_metrics()
        assert metrics.state == 'CLOSED'
        assert metrics.success_count == 2
        assert metrics.failure_count == 1
        assert metrics.total_calls == 3
        assert metrics.last_failure_time is not None
        assert metrics.last_success_time is not None
        assert metrics.trips == 0

    def test_trip_count_increases(self):
        cb = _make_breaker(failure_threshold=1, reset_timeout_sec=0.05)
        # Trip once
        with pytest.raises(RuntimeError):
            cb.call(_failing_func)
        assert cb.get_metrics().trips == 1
        # Reset and trip again
        time.sleep(0.1)
        with pytest.raises(RuntimeError):
            cb.call(_failing_func)
        assert cb.get_metrics().trips == 2

    def test_state_changes_recorded(self):
        cb = _make_breaker(failure_threshold=1)
        with pytest.raises(RuntimeError):
            cb.call(_failing_func)
        metrics = cb.get_metrics()
        assert len(metrics.state_changes) >= 1
        ts, old, new = metrics.state_changes[0]
        assert old == 'CLOSED'
        assert new == 'OPEN'


class TestCircuitBreakerRegistry:
    """Tests for the registry of named circuit breakers."""

    def test_get_or_create_returns_same_instance(self):
        registry = CircuitBreakerRegistry()
        cb1 = registry.get_or_create('svc_a')
        cb2 = registry.get_or_create('svc_a')
        assert cb1 is cb2

    def test_get_or_create_different_names(self):
        registry = CircuitBreakerRegistry()
        cb1 = registry.get_or_create('svc_a')
        cb2 = registry.get_or_create('svc_b')
        assert cb1 is not cb2

    def test_get_all_metrics(self):
        registry = CircuitBreakerRegistry()
        cb_a = registry.get_or_create('svc_a')
        cb_b = registry.get_or_create('svc_b')
        cb_a.call(_succeeding_func)

        all_metrics = registry.get_all_metrics()
        assert 'svc_a' in all_metrics
        assert 'svc_b' in all_metrics
        assert all_metrics['svc_a'].total_calls == 1
        assert all_metrics['svc_b'].total_calls == 0

    def test_registry_respects_custom_config(self):
        registry = CircuitBreakerRegistry()
        config = CircuitBreakerConfig(failure_threshold=10)
        cb = registry.get_or_create('custom', config=config)
        assert cb._config.failure_threshold == 10


class TestCircuitBreakerEdgeCases:
    """Edge-case and integration-level tests."""

    def test_default_config_values(self):
        config = CircuitBreakerConfig()
        assert config.failure_threshold == 5
        assert config.reset_timeout_sec == 30.0
        assert config.half_open_max_calls == 3
        assert config.success_threshold == 2

    def test_call_propagates_original_exception(self):
        cb = _make_breaker()

        class CustomError(Exception):
            pass

        def raise_custom():
            raise CustomError('specific error')

        with pytest.raises(CustomError, match='specific error'):
            cb.call(raise_custom)

    def test_circuit_open_error_is_exception(self):
        assert issubclass(CircuitOpenError, Exception)
