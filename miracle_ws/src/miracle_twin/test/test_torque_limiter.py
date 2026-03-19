"""Tests for SpindleTorqueLimiter and associated dataclasses."""
import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import math
import pytest

from miracle_twin.cutting_sim_proxy import (
    SpindleTorqueLimiter,
    TorqueLimit,
    TorqueReading,
    TorqueStatus,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _limiter(max_torque=100.0, warning=80.0, critical=95.0, action='reduce_feed'):
    lim = SpindleTorqueLimiter()
    lim.set_limit(TorqueLimit(
        max_torque_nm=max_torque,
        warning_pct=warning,
        critical_pct=critical,
        action=action,
    ))
    return lim


def _reading(torque=50.0, rpm=3000.0, power=None, ts=1.0):
    if power is None:
        power = torque * rpm / 9549.0
    return TorqueReading(timestamp=ts, torque_nm=torque, rpm=rpm, power_kw=power)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestCalculateTorque:
    """calculate_torque: T = P * 9549 / RPM"""

    def test_basic(self):
        t = SpindleTorqueLimiter.calculate_torque(power_kw=5.0, rpm=3000.0)
        expected = 5.0 * 9549.0 / 3000.0
        assert math.isclose(t, expected, rel_tol=1e-6)

    def test_zero_rpm_returns_zero(self):
        assert SpindleTorqueLimiter.calculate_torque(power_kw=10.0, rpm=0) == 0.0


class TestCalculatePower:
    """calculate_power: P = T * RPM / 9549"""

    def test_basic(self):
        p = SpindleTorqueLimiter.calculate_power(torque_nm=15.0, rpm=6000.0)
        expected = 15.0 * 6000.0 / 9549.0
        assert math.isclose(p, expected, rel_tol=1e-6)

    def test_roundtrip(self):
        """Torque -> Power -> Torque should be identity."""
        rpm = 4500.0
        original_torque = 22.5
        power = SpindleTorqueLimiter.calculate_power(original_torque, rpm)
        recovered = SpindleTorqueLimiter.calculate_torque(power, rpm)
        assert math.isclose(recovered, original_torque, rel_tol=1e-9)


class TestSetLimit:
    def test_valid_limit(self):
        lim = SpindleTorqueLimiter()
        lim.set_limit(TorqueLimit(max_torque_nm=50.0))
        status = lim.get_status()
        assert status.limit_nm == 50.0

    def test_invalid_max_torque(self):
        lim = SpindleTorqueLimiter()
        with pytest.raises(ValueError, match="positive"):
            lim.set_limit(TorqueLimit(max_torque_nm=-1.0))

    def test_invalid_action(self):
        lim = SpindleTorqueLimiter()
        with pytest.raises(ValueError, match="Invalid action"):
            lim.set_limit(TorqueLimit(max_torque_nm=50.0, action='explode'))


class TestRecordAndStatus:
    def test_normal_status(self):
        lim = _limiter(max_torque=100.0)
        status = lim.record_reading(_reading(torque=50.0))
        assert status.status == 'normal'
        assert math.isclose(status.utilization_pct, 50.0, rel_tol=1e-4)
        assert status.recommended_feed_pct == 100.0

    def test_warning_status(self):
        lim = _limiter(max_torque=100.0, warning=80.0)
        status = lim.record_reading(_reading(torque=85.0))
        assert status.status == 'warning'
        assert status.recommended_feed_pct < 100.0

    def test_critical_status(self):
        lim = _limiter(max_torque=100.0, critical=95.0)
        status = lim.record_reading(_reading(torque=97.0))
        assert status.status == 'critical'

    def test_overload_status(self):
        lim = _limiter(max_torque=100.0)
        status = lim.record_reading(_reading(torque=110.0))
        assert status.status == 'overload'
        assert status.utilization_pct > 100.0

    def test_no_limit_set_returns_normal(self):
        lim = SpindleTorqueLimiter()
        status = lim.record_reading(_reading(torque=999.0))
        assert status.status == 'normal'

    def test_no_readings_returns_zero(self):
        lim = _limiter(max_torque=100.0)
        status = lim.get_status()
        assert status.current_torque_nm == 0.0
        assert status.status == 'normal'


class TestFeedReduction:
    def test_reduction_computed(self):
        lim = _limiter(max_torque=100.0)
        # Current torque is 90 Nm, want to get to 70% of limit (70 Nm)
        pct = lim.get_feed_reduction(current_torque=90.0, target_pct=70.0)
        expected = (70.0 / 90.0) * 100.0
        assert math.isclose(pct, round(expected, 4), rel_tol=1e-4)

    def test_no_limit_returns_100(self):
        lim = SpindleTorqueLimiter()
        assert lim.get_feed_reduction(current_torque=50.0, target_pct=50.0) == 100.0

    def test_zero_torque_returns_100(self):
        lim = _limiter(max_torque=100.0)
        assert lim.get_feed_reduction(current_torque=0.0, target_pct=50.0) == 100.0

    def test_clamped_to_100(self):
        lim = _limiter(max_torque=200.0)
        # target would exceed current -> clamped to 100
        pct = lim.get_feed_reduction(current_torque=10.0, target_pct=90.0)
        assert pct == 100.0


class TestHistory:
    def test_get_torque_history_returns_last_n(self):
        lim = _limiter()
        for i in range(20):
            lim.record_reading(_reading(torque=float(i), ts=float(i)))
        history = lim.get_torque_history(last_n=5)
        assert len(history) == 5
        assert history[0].torque_nm == 15.0
        assert history[-1].torque_nm == 19.0

    def test_history_fewer_than_n(self):
        lim = _limiter()
        lim.record_reading(_reading(torque=7.0))
        assert len(lim.get_torque_history(last_n=10)) == 1

    def test_peak_torque(self):
        lim = _limiter()
        lim.record_reading(_reading(torque=10.0, ts=1.0))
        lim.record_reading(_reading(torque=50.0, ts=2.0))
        lim.record_reading(_reading(torque=30.0, ts=3.0))
        peak = lim.get_peak_torque()
        assert peak is not None
        assert peak.torque_nm == 50.0
        assert peak.timestamp == 2.0

    def test_peak_torque_empty(self):
        lim = SpindleTorqueLimiter()
        assert lim.get_peak_torque() is None
