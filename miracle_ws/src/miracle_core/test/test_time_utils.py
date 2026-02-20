"""Tests for time conversion utilities.

Validates conversions between float seconds, builtin_interfaces/Time
messages, duration computation, and expiration checks.
Mocks rclpy and builtin_interfaces since we do not have a ROS2 runtime.
"""

import sys
from unittest.mock import MagicMock

import pytest

# --- Mock ROS2 time types ---

class _MockTimeMsg:
    """Stand-in for builtin_interfaces.msg.Time."""
    def __init__(self):
        self.sec = 0
        self.nanosec = 0


class _MockTime:
    """Stand-in for rclpy.time.Time that supports subtraction."""
    def __init__(self, nanoseconds=0):
        self._ns = nanoseconds

    @property
    def nanoseconds(self):
        return self._ns

    def __sub__(self, other):
        result = MagicMock()
        result.nanoseconds = self._ns - other._ns
        return result


class _MockDuration:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds


# Wire up mocks
_rclpy_time_mod = MagicMock()
_rclpy_time_mod.Time = _MockTime

_rclpy_duration_mod = MagicMock()
_rclpy_duration_mod.Duration = MagicMock()

_builtin_msg = MagicMock()
_builtin_msg.Time = _MockTimeMsg

sys.modules.setdefault('rclpy', MagicMock())
sys.modules['rclpy.time'] = _rclpy_time_mod
sys.modules['rclpy.duration'] = _rclpy_duration_mod
sys.modules['builtin_interfaces'] = MagicMock()
sys.modules['builtin_interfaces.msg'] = _builtin_msg

from miracle_core.time_utils import (
    time_to_float,
    float_to_time_msg,
    duration_between,
    is_expired,
)


class TestTimeToFloat:
    """Tests for time_to_float conversion."""

    def test_zero_time(self):
        """A zero TimeMsg should convert to 0.0."""
        msg = _MockTimeMsg()
        msg.sec = 0
        msg.nanosec = 0
        assert time_to_float(msg) == pytest.approx(0.0)

    def test_whole_seconds(self):
        """Whole seconds should convert exactly."""
        msg = _MockTimeMsg()
        msg.sec = 42
        msg.nanosec = 0
        assert time_to_float(msg) == pytest.approx(42.0)

    def test_fractional_seconds(self):
        """Nanoseconds should be correctly combined with seconds."""
        msg = _MockTimeMsg()
        msg.sec = 1
        msg.nanosec = 500_000_000  # 0.5s
        assert time_to_float(msg) == pytest.approx(1.5)

    def test_large_nanoseconds(self):
        """Near-maximum nanosecond values should convert correctly."""
        msg = _MockTimeMsg()
        msg.sec = 0
        msg.nanosec = 999_999_999
        assert time_to_float(msg) == pytest.approx(0.999999999, abs=1e-9)


class TestFloatToTimeMsg:
    """Tests for float_to_time_msg conversion."""

    def test_whole_seconds(self):
        """Integer seconds should produce zero nanoseconds."""
        msg = float_to_time_msg(10.0)
        assert msg.sec == 10
        assert msg.nanosec == 0

    def test_fractional_conversion(self):
        """Fractional seconds should split into sec and nanosec."""
        msg = float_to_time_msg(2.5)
        assert msg.sec == 2
        assert msg.nanosec == 500_000_000

    def test_zero_conversion(self):
        """Zero should produce both fields as zero."""
        msg = float_to_time_msg(0.0)
        assert msg.sec == 0
        assert msg.nanosec == 0

    def test_roundtrip(self):
        """Converting float -> TimeMsg -> float should preserve the value."""
        original = 123.456789
        msg = float_to_time_msg(original)
        recovered = time_to_float(msg)
        assert recovered == pytest.approx(original, abs=1e-3)


class TestDurationBetween:
    """Tests for duration_between computation."""

    def test_positive_duration(self):
        """Duration from an earlier to a later time should be positive."""
        start = _MockTime(nanoseconds=1_000_000_000)  # 1s
        end = _MockTime(nanoseconds=3_500_000_000)    # 3.5s
        result = duration_between(start, end)
        assert result == pytest.approx(2.5)

    def test_zero_duration(self):
        """Same start and end should yield zero."""
        t = _MockTime(nanoseconds=5_000_000_000)
        result = duration_between(t, t)
        assert result == pytest.approx(0.0)

    def test_negative_duration(self):
        """If end is before start, duration should be negative."""
        start = _MockTime(nanoseconds=5_000_000_000)
        end = _MockTime(nanoseconds=2_000_000_000)
        result = duration_between(start, end)
        assert result < 0


class TestIsExpired:
    """Tests for the is_expired timeout check."""

    def test_not_expired(self):
        """A recent timestamp within the timeout should not be expired."""
        ts = _MockTime(nanoseconds=9_000_000_000)     # 9s
        now = _MockTime(nanoseconds=10_000_000_000)    # 10s
        assert is_expired(ts, now, timeout_sec=5.0) is False

    def test_expired(self):
        """A timestamp beyond the timeout should be expired."""
        ts = _MockTime(nanoseconds=1_000_000_000)      # 1s
        now = _MockTime(nanoseconds=10_000_000_000)     # 10s
        assert is_expired(ts, now, timeout_sec=5.0) is True

    def test_exact_boundary_not_expired(self):
        """Exactly at the timeout boundary (elapsed == timeout) should NOT be expired."""
        ts = _MockTime(nanoseconds=5_000_000_000)
        now = _MockTime(nanoseconds=10_000_000_000)
        # elapsed = 5.0, timeout = 5.0 -> not expired (> vs >=)
        assert is_expired(ts, now, timeout_sec=5.0) is False
