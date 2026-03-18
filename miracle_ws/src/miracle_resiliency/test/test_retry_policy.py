"""Tests for the RetryPolicyManager.

Covers retry policy registration, backoff strategy computation,
execute_with_retry behaviour, timeout handling, exception filtering,
default policies, and aggregate statistics.
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
    RetryPolicy,
    RetryAttempt,
    RetryResult,
    RetryPolicyManager,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_manager() -> RetryPolicyManager:
    return RetryPolicyManager()


def _zero_delay_policy(name: str = 'test', max_retries: int = 3,
                       strategy: str = 'fixed',
                       retry_on: list | None = None) -> RetryPolicy:
    """Create a policy with zero delays for fast tests."""
    return RetryPolicy(
        name=name,
        max_retries=max_retries,
        base_delay_sec=0.0,
        max_delay_sec=0.0,
        backoff_strategy=strategy,
        retry_on_exceptions=retry_on or ['Exception'],
        timeout_sec=300.0,
    )


# ---------------------------------------------------------------------------
# 1. Default policies are pre-registered
# ---------------------------------------------------------------------------

class TestDefaultPolicies:
    def test_fast_retry_registered(self):
        mgr = _make_manager()
        policy = mgr.get_policy('FAST_RETRY')
        assert policy is not None
        assert policy.max_retries == 3
        assert policy.base_delay_sec == 0.5
        assert policy.backoff_strategy == 'exponential'

    def test_patient_retry_registered(self):
        mgr = _make_manager()
        policy = mgr.get_policy('PATIENT_RETRY')
        assert policy is not None
        assert policy.max_retries == 5
        assert policy.base_delay_sec == 2.0
        assert policy.backoff_strategy == 'exponential'

    def test_critical_retry_registered(self):
        mgr = _make_manager()
        policy = mgr.get_policy('CRITICAL_RETRY')
        assert policy is not None
        assert policy.max_retries == 10
        assert policy.base_delay_sec == 1.0
        assert policy.backoff_strategy == 'jitter'


# ---------------------------------------------------------------------------
# 2. Policy registration and lookup
# ---------------------------------------------------------------------------

class TestPolicyRegistration:
    def test_register_and_get(self):
        mgr = _make_manager()
        custom = RetryPolicy(name='CUSTOM', max_retries=7, base_delay_sec=0.1)
        mgr.register_policy(custom)
        assert mgr.get_policy('CUSTOM') is custom

    def test_get_nonexistent_returns_none(self):
        mgr = _make_manager()
        assert mgr.get_policy('DOES_NOT_EXIST') is None


# ---------------------------------------------------------------------------
# 3. compute_delay for each backoff strategy
# ---------------------------------------------------------------------------

class TestComputeDelay:
    def test_fixed_delay(self):
        policy = RetryPolicy(name='f', base_delay_sec=2.0, backoff_strategy='fixed')
        mgr = _make_manager()
        assert mgr.compute_delay(policy, 1) == 2.0
        assert mgr.compute_delay(policy, 5) == 2.0

    def test_linear_delay(self):
        policy = RetryPolicy(name='l', base_delay_sec=1.0, backoff_strategy='linear')
        mgr = _make_manager()
        assert mgr.compute_delay(policy, 1) == 1.0
        assert mgr.compute_delay(policy, 3) == 3.0
        assert mgr.compute_delay(policy, 5) == 5.0

    def test_exponential_delay(self):
        policy = RetryPolicy(name='e', base_delay_sec=1.0, max_delay_sec=60.0,
                             backoff_strategy='exponential')
        mgr = _make_manager()
        assert mgr.compute_delay(policy, 1) == 1.0   # 1 * 2^0
        assert mgr.compute_delay(policy, 2) == 2.0   # 1 * 2^1
        assert mgr.compute_delay(policy, 3) == 4.0   # 1 * 2^2
        assert mgr.compute_delay(policy, 4) == 8.0   # 1 * 2^3

    def test_exponential_capped_at_max_delay(self):
        policy = RetryPolicy(name='e', base_delay_sec=1.0, max_delay_sec=5.0,
                             backoff_strategy='exponential')
        mgr = _make_manager()
        # 2^9 = 512, but capped at 5.0
        assert mgr.compute_delay(policy, 10) == 5.0

    def test_jitter_delay_within_bounds(self):
        policy = RetryPolicy(name='j', base_delay_sec=1.0, max_delay_sec=100.0,
                             backoff_strategy='jitter')
        mgr = _make_manager()
        for attempt in range(1, 6):
            delay = mgr.compute_delay(policy, attempt)
            exp_base = 1.0 * (2 ** (attempt - 1))
            # jitter adds random(0, base_delay) = random(0, 1.0)
            assert delay >= exp_base
            assert delay <= exp_base + 1.0 + 1e-9


# ---------------------------------------------------------------------------
# 4. execute_with_retry -- success on first try
# ---------------------------------------------------------------------------

class TestExecuteSuccess:
    def test_immediate_success(self):
        mgr = _make_manager()
        policy = _zero_delay_policy()
        result = mgr.execute_with_retry(policy, lambda: 42)
        assert result.succeeded is True
        assert result.result == 42
        assert len(result.attempts) == 1
        assert result.attempts[0].succeeded is True
        assert result.policy_name == 'test'


# ---------------------------------------------------------------------------
# 5. execute_with_retry -- retries then succeeds
# ---------------------------------------------------------------------------

class TestRetryThenSucceed:
    def test_succeeds_after_two_failures(self):
        mgr = _make_manager()
        policy = _zero_delay_policy(max_retries=5)
        call_count = 0

        def flaky():
            nonlocal call_count
            call_count += 1
            if call_count < 3:
                raise RuntimeError('boom')
            return 'ok'

        result = mgr.execute_with_retry(policy, flaky)
        assert result.succeeded is True
        assert result.result == 'ok'
        assert len(result.attempts) == 3
        assert result.attempts[0].succeeded is False
        assert result.attempts[1].succeeded is False
        assert result.attempts[2].succeeded is True


# ---------------------------------------------------------------------------
# 6. execute_with_retry -- exhausts retries
# ---------------------------------------------------------------------------

class TestExhaustRetries:
    def test_all_retries_fail(self):
        mgr = _make_manager()
        policy = _zero_delay_policy(max_retries=2)

        def always_fails():
            raise ValueError('nope')

        result = mgr.execute_with_retry(policy, always_fails)
        assert result.succeeded is False
        assert result.result is None
        # 1 initial + 2 retries = 3 attempts
        assert len(result.attempts) == 3
        assert all(not a.succeeded for a in result.attempts)
        assert result.attempts[0].error_type == 'ValueError'


# ---------------------------------------------------------------------------
# 7. Exception filtering
# ---------------------------------------------------------------------------

class TestExceptionFiltering:
    def test_non_matching_exception_stops_retries(self):
        mgr = _make_manager()
        policy = _zero_delay_policy(
            max_retries=5,
            retry_on=['RuntimeError'],
        )

        def raises_type_error():
            raise TypeError('wrong type')

        result = mgr.execute_with_retry(policy, raises_type_error)
        assert result.succeeded is False
        # Should stop after the first attempt because TypeError is not in the list
        assert len(result.attempts) == 1

    def test_matching_exception_retries(self):
        mgr = _make_manager()
        policy = _zero_delay_policy(
            max_retries=3,
            retry_on=['RuntimeError'],
        )
        calls = 0

        def flaky_runtime():
            nonlocal calls
            calls += 1
            if calls < 3:
                raise RuntimeError('transient')
            return 'recovered'

        result = mgr.execute_with_retry(policy, flaky_runtime)
        assert result.succeeded is True
        assert len(result.attempts) == 3


# ---------------------------------------------------------------------------
# 8. get_retry_stats
# ---------------------------------------------------------------------------

class TestRetryStats:
    def test_empty_stats(self):
        mgr = _make_manager()
        stats = mgr.get_retry_stats()
        assert stats['total_executions'] == 0
        assert stats['success_rate'] == 0.0
        assert stats['avg_retries'] == 0.0

    def test_stats_after_executions(self):
        mgr = _make_manager()
        policy = _zero_delay_policy(max_retries=1)

        # One success (1 attempt)
        mgr.execute_with_retry(policy, lambda: 'ok')

        # One failure (2 attempts: initial + 1 retry)
        mgr.execute_with_retry(policy, MagicMock(side_effect=RuntimeError('bad')))

        stats = mgr.get_retry_stats()
        assert stats['total_executions'] == 2
        assert stats['successful_executions'] == 1
        assert stats['failed_executions'] == 1
        assert stats['success_rate'] == 0.5
        # total_attempts = 1 + 2 = 3, avg = 1.5
        assert stats['total_attempts'] == 3
        assert stats['avg_retries'] == 1.5


# ---------------------------------------------------------------------------
# 9. RetryResult dataclass
# ---------------------------------------------------------------------------

class TestRetryResultDataclass:
    def test_defaults(self):
        r = RetryResult(succeeded=True, result='x')
        assert r.succeeded is True
        assert r.result == 'x'
        assert r.attempts == []
        assert r.total_time_sec == 0.0
        assert r.policy_name == ''


# ---------------------------------------------------------------------------
# 10. Timeout enforcement
# ---------------------------------------------------------------------------

class TestTimeout:
    def test_timeout_stops_retries(self):
        mgr = _make_manager()
        policy = RetryPolicy(
            name='timeout_test',
            max_retries=100,
            base_delay_sec=0.0,
            max_delay_sec=0.0,
            backoff_strategy='fixed',
            timeout_sec=0.0,  # immediate timeout after first attempt
        )

        call_count = 0

        def slow_fail():
            nonlocal call_count
            call_count += 1
            raise RuntimeError('fail')

        result = mgr.execute_with_retry(policy, slow_fail)
        assert result.succeeded is False
        # With timeout_sec=0.0, after the first failed attempt the elapsed
        # time exceeds the timeout so no further attempts are made.
        assert len(result.attempts) <= 2
