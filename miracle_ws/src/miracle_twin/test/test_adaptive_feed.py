"""Tests for AdaptiveFeedController PID-based feed override.

Tests force/power/MRR modes, safety limits, rate limiting,
history tracking, report generation, and zero-force handling.
"""

import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import pytest
from miracle_twin.cutting_sim_proxy import (
    AdaptiveFeedController,
    FeedControlMode,
    FeedOverrideReport,
)


@pytest.fixture
def controller():
    return AdaptiveFeedController()


# ---- Force constant mode ----

def test_force_constant_increases_feed_when_below_target(controller):
    """If measured force is below target, override should increase."""
    controller.set_mode(FeedControlMode.FORCE_CONSTANT, target_value=500.0)
    override = controller.update(300.0)  # under target
    assert override > 100.0


def test_force_constant_decreases_feed_when_above_target(controller):
    """If measured force exceeds target, override should decrease."""
    controller.set_mode(FeedControlMode.FORCE_CONSTANT, target_value=500.0)
    override = controller.update(700.0)  # over target
    assert override < 100.0


def test_force_constant_converges(controller):
    """Repeated updates at target should stabilise near 100%."""
    controller.set_mode(FeedControlMode.FORCE_CONSTANT, target_value=500.0)
    for _ in range(50):
        controller.update(500.0)  # right at target
    assert abs(controller.current_override - 100.0) < 1.0


# ---- Power constant mode ----

def test_power_constant_mode(controller):
    controller.set_mode(FeedControlMode.POWER_CONSTANT, target_value=2000.0)
    override = controller.update(1500.0)  # under target
    assert override > 100.0
    assert controller.mode == FeedControlMode.POWER_CONSTANT


# ---- MRR constant mode ----

def test_mrr_constant_mode(controller):
    controller.set_mode(FeedControlMode.MRR_CONSTANT, target_value=100.0)
    override = controller.update(120.0)  # over target
    assert override < 100.0
    assert controller.mode == FeedControlMode.MRR_CONSTANT


# ---- Safety limits ----

def test_min_override_clamp(controller):
    """Override should never go below min (10%)."""
    controller.set_mode(FeedControlMode.FORCE_CONSTANT, target_value=100.0)
    # Drive it down hard with very high measured values
    for _ in range(100):
        controller.update(10000.0)
    assert controller.current_override >= 10.0


def test_max_override_clamp(controller):
    """Override should never exceed max (150%)."""
    controller.set_mode(FeedControlMode.FORCE_CONSTANT, target_value=1000.0)
    # Drive it up hard with very low measured values
    for _ in range(100):
        controller.update(1.0)
    assert controller.current_override <= 150.0


# ---- Rate of change limiting ----

def test_rate_of_change_limit(controller):
    """Single step change should not exceed max_rate (5%)."""
    controller.set_mode(FeedControlMode.FORCE_CONSTANT, target_value=500.0)
    # Big error: 500 vs 100 -> huge adjustment, but rate-limited
    override = controller.update(100.0)
    assert abs(override - 100.0) <= 5.01  # within rate limit + float tolerance


# ---- History tracking ----

def test_override_history(controller):
    controller.set_mode(FeedControlMode.FORCE_CONSTANT, target_value=500.0)
    controller.update(400.0)
    controller.update(450.0)
    controller.update(500.0)
    assert len(controller._history) == 3
    assert controller._state.total_adjustments == 3


# ---- Report generation ----

def test_report_generation(controller):
    controller.set_mode(FeedControlMode.FORCE_CONSTANT, target_value=500.0)
    controller.update(400.0)
    controller.update(450.0)
    controller.update(500.0)
    controller.update(550.0)

    report = controller.report()
    assert isinstance(report, FeedOverrideReport)
    assert report.total_adjustments == 4
    assert report.mode == 'force_constant'
    assert report.min_override <= report.average_override <= report.max_override
    assert len(report.override_histogram) >= 1


def test_report_empty_history(controller):
    report = controller.report()
    assert report.total_adjustments == 0
    assert report.current_override == 100.0


# ---- Zero/edge cases ----

def test_zero_target_no_crash(controller):
    """Target of 0 should not cause division by zero."""
    controller.set_mode(FeedControlMode.FORCE_CONSTANT, target_value=0.0)
    override = controller.update(500.0)
    # Should return current override unchanged
    assert override == 100.0


def test_zero_measured_value(controller):
    """Zero measured value with positive target should increase override."""
    controller.set_mode(FeedControlMode.FORCE_CONSTANT, target_value=500.0)
    override = controller.update(0.0)
    assert override > 100.0


def test_reset(controller):
    controller.set_mode(FeedControlMode.POWER_CONSTANT, target_value=2000.0)
    controller.update(1500.0)
    controller.update(1800.0)
    controller.reset()
    assert controller.current_override == 100.0
    assert controller._state.total_adjustments == 0
    assert len(controller._history) == 0
