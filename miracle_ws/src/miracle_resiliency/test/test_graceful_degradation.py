"""Tests for GracefulDegradationManager.

Covers component registration, failure/recovery reporting, degradation
level computation, fallback activation/deactivation, capability queries,
can_operate checks, critical-component SHUTDOWN, and SystemStatus snapshots.
"""

import sys
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# Mock ROS2 / miracle dependencies so we can import without a ROS2 install.
# ---------------------------------------------------------------------------
for mod in [
    'miracle_core.datatypes', 'miracle_core.constants',
    'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
    'rclpy.callback_groups',
    'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
    'miracle_msgs', 'miracle_msgs.msg',
    'std_msgs', 'std_msgs.msg',
]:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_resiliency.recovery_orchestrator import (
    DegradationLevel,
    FallbackStrategy,
    GracefulDegradationManager,
    SystemStatus,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_manager_with_components():
    """Return a manager with three normal and one critical component."""
    mgr = GracefulDegradationManager()
    mgr.register_component('spindle', ['cutting', 'drilling'])
    mgr.register_component('coolant', ['cooling', 'chip_flush'])
    mgr.register_component('axis_x', ['movement_x'])
    mgr.register_component('safety_monitor', ['estop', 'monitoring'])
    return mgr


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestDegradationLevelComputation:
    """Degradation level depends on the number/type of failed components."""

    def test_normal_when_no_failures(self):
        mgr = _make_manager_with_components()
        assert mgr.get_degradation_level() == DegradationLevel.NORMAL

    def test_reduced_with_one_failure(self):
        mgr = _make_manager_with_components()
        mgr.report_failure('spindle', 'overheated')
        assert mgr.get_degradation_level() == DegradationLevel.REDUCED

    def test_reduced_with_two_failures(self):
        mgr = _make_manager_with_components()
        mgr.report_failure('spindle', 'overheated')
        mgr.report_failure('coolant', 'pump failure')
        assert mgr.get_degradation_level() == DegradationLevel.REDUCED

    def test_minimal_with_three_failures(self):
        mgr = _make_manager_with_components()
        mgr.register_component('axis_y', ['movement_y'])
        mgr.report_failure('spindle', 'overheated')
        mgr.report_failure('coolant', 'pump failure')
        mgr.report_failure('axis_x', 'encoder fault')
        assert mgr.get_degradation_level() == DegradationLevel.MINIMAL

    def test_emergency_with_five_failures(self):
        mgr = GracefulDegradationManager()
        for i in range(5):
            mgr.register_component(f'comp_{i}', [f'cap_{i}'])
        for i in range(5):
            mgr.report_failure(f'comp_{i}', 'broken')
        assert mgr.get_degradation_level() == DegradationLevel.EMERGENCY

    def test_shutdown_on_critical_component_failure(self):
        mgr = _make_manager_with_components()
        level = mgr.report_failure('safety_monitor', 'unresponsive')
        assert level == DegradationLevel.SHUTDOWN
        assert mgr.get_degradation_level() == DegradationLevel.SHUTDOWN


class TestRecovery:
    """Component recovery should restore capabilities and lower the level."""

    def test_recovery_restores_normal(self):
        mgr = _make_manager_with_components()
        mgr.report_failure('spindle', 'overheated')
        assert mgr.get_degradation_level() == DegradationLevel.REDUCED
        level = mgr.report_recovery('spindle')
        assert level == DegradationLevel.NORMAL

    def test_recovery_of_unknown_component_is_safe(self):
        mgr = GracefulDegradationManager()
        level = mgr.report_recovery('nonexistent')
        assert level == DegradationLevel.NORMAL


class TestCapabilities:
    """Available capabilities reflect only healthy components."""

    def test_all_capabilities_when_healthy(self):
        mgr = _make_manager_with_components()
        caps = mgr.get_available_capabilities()
        assert set(caps) == {
            'cutting', 'drilling', 'cooling', 'chip_flush',
            'movement_x', 'estop', 'monitoring',
        }

    def test_capabilities_reduced_after_failure(self):
        mgr = _make_manager_with_components()
        mgr.report_failure('spindle', 'fault')
        caps = mgr.get_available_capabilities()
        assert 'cutting' not in caps
        assert 'drilling' not in caps
        assert 'cooling' in caps

    def test_can_operate_true_when_caps_available(self):
        mgr = _make_manager_with_components()
        assert mgr.can_operate(['cutting', 'cooling']) is True

    def test_can_operate_false_when_cap_missing(self):
        mgr = _make_manager_with_components()
        mgr.report_failure('spindle', 'fault')
        assert mgr.can_operate(['cutting']) is False

    def test_can_operate_empty_list_always_true(self):
        mgr = GracefulDegradationManager()
        assert mgr.can_operate([]) is True


class TestFallbackStrategies:
    """Fallback activation and deactivation on failure/recovery."""

    def test_fallback_activated_on_failure(self):
        mgr = _make_manager_with_components()
        fb = FallbackStrategy(
            strategy_id='fb1',
            component='spindle',
            degradation_level=DegradationLevel.REDUCED,
            action='reduce_speed',
            description='Lower spindle speed',
            priority=1,
        )
        mgr.add_fallback(fb)
        mgr.report_failure('spindle', 'overheated')
        status = mgr.get_status()
        assert 'fb1' in status.active_fallbacks

    def test_fallback_deactivated_on_recovery(self):
        mgr = _make_manager_with_components()
        fb = FallbackStrategy(
            strategy_id='fb1',
            component='spindle',
            degradation_level=DegradationLevel.REDUCED,
            action='reduce_speed',
            description='Lower spindle speed',
            priority=1,
        )
        mgr.add_fallback(fb)
        mgr.report_failure('spindle', 'overheated')
        mgr.report_recovery('spindle')
        status = mgr.get_status()
        assert 'fb1' not in status.active_fallbacks

    def test_multiple_fallbacks_priority_order(self):
        mgr = _make_manager_with_components()
        fb_low = FallbackStrategy('fb_low', 'spindle', DegradationLevel.REDUCED,
                                  'slow_down', 'Low priority', priority=10)
        fb_high = FallbackStrategy('fb_high', 'spindle', DegradationLevel.REDUCED,
                                   'stop', 'High priority', priority=1)
        mgr.add_fallback(fb_low)
        mgr.add_fallback(fb_high)
        mgr.report_failure('spindle', 'fault')
        status = mgr.get_status()
        # Both should be active; high-priority first in insertion order
        assert status.active_fallbacks == ['fb_high', 'fb_low']


class TestSystemStatus:
    """get_status returns a coherent SystemStatus snapshot."""

    def test_status_snapshot_fields(self):
        mgr = _make_manager_with_components()
        mgr.report_failure('coolant', 'leak')
        status = mgr.get_status()
        assert isinstance(status, SystemStatus)
        assert status.current_level == DegradationLevel.REDUCED
        assert 'coolant' in status.failed_components
        assert 'cooling' not in status.available_capabilities
        assert status.timestamp > 0

    def test_status_normal_when_clean(self):
        mgr = _make_manager_with_components()
        status = mgr.get_status()
        assert status.current_level == DegradationLevel.NORMAL
        assert status.failed_components == []
        assert status.active_fallbacks == []
