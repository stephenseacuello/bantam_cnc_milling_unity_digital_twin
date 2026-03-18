"""Tests for the RateLimiter pattern.

Covers token bucket and sliding window rate limiters, the factory,
thread safety, burst handling, retry-after computation, and edge cases.
"""

import sys
import time
import threading
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
    RateLimitConfig,
    RateLimitResult,
    TokenBucketLimiter,
    SlidingWindowLimiter,
    RateLimiterFactory,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_token_config(**overrides) -> RateLimitConfig:
    defaults = dict(
        name='test_api',
        max_requests=10,
        window_sec=1.0,
        burst_size=10,
        strategy='token_bucket',
    )
    defaults.update(overrides)
    return RateLimitConfig(**defaults)


def _make_window_config(**overrides) -> RateLimitConfig:
    defaults = dict(
        name='test_api',
        max_requests=5,
        window_sec=1.0,
        burst_size=5,
        strategy='sliding_window',
    )
    defaults.update(overrides)
    return RateLimitConfig(**defaults)


# ---------------------------------------------------------------------------
# RateLimitConfig
# ---------------------------------------------------------------------------

class TestRateLimitConfig:
    """Tests for the RateLimitConfig dataclass."""

    def test_default_burst_size_matches_max_requests(self):
        cfg = RateLimitConfig(name='api', max_requests=20, window_sec=10.0)
        assert cfg.burst_size == 20

    def test_explicit_burst_size_is_preserved(self):
        cfg = RateLimitConfig(
            name='api', max_requests=20, window_sec=10.0, burst_size=50,
        )
        assert cfg.burst_size == 50


# ---------------------------------------------------------------------------
# TokenBucketLimiter
# ---------------------------------------------------------------------------

class TestTokenBucketLimiter:
    """Tests for the token-bucket rate limiter."""

    def test_allows_requests_up_to_burst(self):
        cfg = _make_token_config(max_requests=5, burst_size=5, window_sec=1.0)
        limiter = TokenBucketLimiter(cfg)
        results = [limiter.allow() for _ in range(5)]
        assert all(r.allowed for r in results)
        assert results[-1].remaining == 0

    def test_rejects_after_burst_exhausted(self):
        cfg = _make_token_config(max_requests=3, burst_size=3, window_sec=1.0)
        limiter = TokenBucketLimiter(cfg)
        for _ in range(3):
            limiter.allow()
        result = limiter.allow()
        assert not result.allowed
        assert result.remaining == 0
        assert result.retry_after_sec > 0

    def test_tokens_refill_over_time(self):
        cfg = _make_token_config(max_requests=10, burst_size=10, window_sec=1.0)
        limiter = TokenBucketLimiter(cfg)
        # Exhaust all tokens
        for _ in range(10):
            limiter.allow()
        assert limiter.get_remaining() == 0
        # Simulate time passing (refill rate = 10/sec, so 0.5s => ~5 tokens)
        with patch('miracle_resiliency.recovery_orchestrator.time') as mock_time:
            mock_time.time.return_value = time.time() + 0.5
            remaining = limiter.get_remaining()
            assert remaining >= 4  # at least 4 tokens after ~0.5s

    def test_result_contains_limit_name(self):
        cfg = _make_token_config(name='my_service')
        limiter = TokenBucketLimiter(cfg)
        result = limiter.allow()
        assert result.limit_name == 'my_service'

    def test_custom_name_override_in_allow(self):
        cfg = _make_token_config(name='default')
        limiter = TokenBucketLimiter(cfg)
        result = limiter.allow(name='override_name')
        assert result.limit_name == 'override_name'

    def test_thread_safety(self):
        """Multiple threads consuming tokens concurrently."""
        cfg = _make_token_config(max_requests=100, burst_size=100, window_sec=10.0)
        limiter = TokenBucketLimiter(cfg)
        allowed_count = [0]
        lock = threading.Lock()

        def consume():
            for _ in range(10):
                result = limiter.allow()
                if result.allowed:
                    with lock:
                        allowed_count[0] += 1

        threads = [threading.Thread(target=consume) for _ in range(10)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # Exactly 100 tokens were available; all should have been consumed
        assert allowed_count[0] == 100


# ---------------------------------------------------------------------------
# SlidingWindowLimiter
# ---------------------------------------------------------------------------

class TestSlidingWindowLimiter:
    """Tests for the sliding-window rate limiter."""

    def test_allows_up_to_max_requests(self):
        cfg = _make_window_config(max_requests=3, window_sec=1.0)
        limiter = SlidingWindowLimiter(cfg)
        for i in range(3):
            result = limiter.allow()
            assert result.allowed
        assert limiter.get_remaining() == 0

    def test_rejects_beyond_max_in_window(self):
        cfg = _make_window_config(max_requests=3, window_sec=1.0)
        limiter = SlidingWindowLimiter(cfg)
        for _ in range(3):
            limiter.allow()
        result = limiter.allow()
        assert not result.allowed
        assert result.retry_after_sec > 0
        assert result.remaining == 0

    def test_window_slides_to_allow_new_requests(self):
        cfg = _make_window_config(max_requests=2, window_sec=0.1)
        limiter = SlidingWindowLimiter(cfg)
        limiter.allow()
        limiter.allow()
        # Window is full
        assert not limiter.allow().allowed
        # Wait for window to expire
        time.sleep(0.15)
        result = limiter.allow()
        assert result.allowed

    def test_remaining_decreases_correctly(self):
        cfg = _make_window_config(max_requests=5, window_sec=1.0)
        limiter = SlidingWindowLimiter(cfg)
        assert limiter.get_remaining() == 5
        limiter.allow()
        assert limiter.get_remaining() == 4
        limiter.allow()
        assert limiter.get_remaining() == 3

    def test_thread_safety(self):
        """Concurrent threads cannot exceed max_requests within the window."""
        cfg = _make_window_config(max_requests=50, window_sec=10.0)
        limiter = SlidingWindowLimiter(cfg)
        allowed_count = [0]
        lock = threading.Lock()

        def consume():
            for _ in range(10):
                result = limiter.allow()
                if result.allowed:
                    with lock:
                        allowed_count[0] += 1

        threads = [threading.Thread(target=consume) for _ in range(10)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert allowed_count[0] == 50


# ---------------------------------------------------------------------------
# RateLimiterFactory
# ---------------------------------------------------------------------------

class TestRateLimiterFactory:
    """Tests for the rate limiter factory."""

    def test_create_token_bucket(self):
        cfg = _make_token_config()
        limiter = RateLimiterFactory.create(cfg)
        assert isinstance(limiter, TokenBucketLimiter)

    def test_create_sliding_window(self):
        cfg = _make_window_config()
        limiter = RateLimiterFactory.create(cfg)
        assert isinstance(limiter, SlidingWindowLimiter)

    def test_create_unknown_strategy_raises(self):
        cfg = RateLimitConfig(
            name='bad', max_requests=5, window_sec=1.0, strategy='unknown',
        )
        with pytest.raises(ValueError, match='Unknown rate limit strategy'):
            RateLimiterFactory.create(cfg)

    def test_get_or_create_returns_same_instance(self):
        factory = RateLimiterFactory()
        cfg = _make_token_config(name='singleton')
        limiter1 = factory.get_or_create('singleton', cfg)
        limiter2 = factory.get_or_create('singleton', cfg)
        assert limiter1 is limiter2

    def test_get_all_stats(self):
        factory = RateLimiterFactory()
        cfg_a = _make_token_config(name='api_a', max_requests=10, window_sec=1.0)
        cfg_b = _make_window_config(name='api_b', max_requests=5, window_sec=2.0)
        factory.get_or_create('api_a', cfg_a)
        factory.get_or_create('api_b', cfg_b)

        stats = factory.get_all_stats()
        assert 'api_a' in stats
        assert 'api_b' in stats
        assert stats['api_a']['max_requests'] == 10
        assert stats['api_a']['strategy'] == 'token_bucket'
        assert stats['api_b']['max_requests'] == 5
        assert stats['api_b']['strategy'] == 'sliding_window'
        assert stats['api_a']['remaining'] == 10
        assert stats['api_b']['remaining'] == 5

    def test_get_all_stats_reflects_consumption(self):
        factory = RateLimiterFactory()
        cfg = _make_token_config(name='svc', max_requests=5, burst_size=5)
        limiter = factory.get_or_create('svc', cfg)
        limiter.allow()
        limiter.allow()
        stats = factory.get_all_stats()
        assert stats['svc']['remaining'] == 3
