"""Tests for AlarmSuppressionRuleManager.

Validates suppression rule management, pattern matching, shelving,
maintenance windows, duplicate suppression, and history tracking.
"""

import sys
import time
from unittest.mock import MagicMock

# Mock ROS2 and miracle_core sub-module dependencies before importing
for _mod in ['rclpy', 'rclpy.node', 'rclpy.callback_groups', 'rclpy.qos', 'rclpy.lifecycle',
             'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
             'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
             'miracle_core.exceptions',
             'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(_mod, MagicMock())

sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = type('FakeNode', (), {
    'CRITICALITY_HIGH': 'HIGH',
    'CRITICALITY_MEDIUM': 'MEDIUM',
    '__init__': lambda self, *a, **kw: None,
    'get_logger': lambda self: MagicMock(),
    'create_publisher': lambda self, *a, **kw: MagicMock(),
    'create_subscription': lambda self, *a, **kw: MagicMock(),
    'create_timer': lambda self, *a, **kw: MagicMock(),
    'declare_and_validate_parameters': lambda self, specs: {k: MagicMock(value=v['default']) for k, v in specs.items()},
    'get_parameter': lambda self, name: MagicMock(value=0),
})

sys.modules.pop('miracle_scada.alarm_manager', None)

import pytest

from miracle_scada.alarm_manager import (
    AlarmSuppressionRuleManager,
    SuppressionDecision,
    SuppressionRule,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_manager() -> AlarmSuppressionRuleManager:
    return AlarmSuppressionRuleManager()


def _make_shelve_rule(
    alarm_type_pattern: str = 'THERMAL*',
    start: float = 1000.0,
    end: float = 2000.0,
    rule_id: str = 'shelve_1',
) -> SuppressionRule:
    return SuppressionRule(
        rule_id=rule_id,
        name='Shelve thermal',
        rule_type='shelve',
        alarm_type_pattern=alarm_type_pattern,
        machine_pattern='*',
        start_time=start,
        end_time=end,
        is_active=True,
        created_by='operator_1',
        reason='Known false positive',
    )


def _make_maintenance_rule(
    machine_pattern: str = 'cnc_1',
    start: float = 5000.0,
    end: float = 6000.0,
    rule_id: str = 'maint_1',
) -> SuppressionRule:
    return SuppressionRule(
        rule_id=rule_id,
        name='Maintenance cnc_1',
        rule_type='maintenance_window',
        alarm_type_pattern='*',
        machine_pattern=machine_pattern,
        start_time=start,
        end_time=end,
        is_active=True,
        created_by='system',
        reason='Scheduled maintenance',
    )


# ---------------------------------------------------------------------------
# Rule CRUD
# ---------------------------------------------------------------------------

class TestRuleCrud:
    def test_add_and_get_active_rules(self):
        mgr = _make_manager()
        rule = _make_shelve_rule()
        mgr.add_rule(rule)
        active = mgr.get_active_rules()
        assert len(active) == 1
        assert active[0].rule_id == 'shelve_1'

    def test_remove_rule(self):
        mgr = _make_manager()
        mgr.add_rule(_make_shelve_rule())
        assert mgr.remove_rule('shelve_1') is True
        assert mgr.get_active_rules() == []

    def test_remove_nonexistent_rule_returns_false(self):
        mgr = _make_manager()
        assert mgr.remove_rule('no_such_rule') is False

    def test_inactive_rule_excluded_from_active(self):
        mgr = _make_manager()
        rule = _make_shelve_rule()
        rule.is_active = False
        mgr.add_rule(rule)
        assert mgr.get_active_rules() == []


# ---------------------------------------------------------------------------
# Pattern matching — shelve rules
# ---------------------------------------------------------------------------

class TestShelveEvaluation:
    def test_shelve_suppresses_matching_alarm(self):
        mgr = _make_manager()
        mgr.add_rule(_make_shelve_rule(alarm_type_pattern='THERMAL*', start=1000.0, end=2000.0))
        decision = mgr.evaluate('alarm_1', 'THERMAL_OVERTEMP', 'cnc_1', 0.8, 1500.0)
        assert decision.should_suppress is True
        assert decision.rule_id == 'shelve_1'

    def test_shelve_does_not_suppress_outside_window(self):
        mgr = _make_manager()
        mgr.add_rule(_make_shelve_rule(start=1000.0, end=2000.0))
        decision = mgr.evaluate('alarm_2', 'THERMAL_OVERTEMP', 'cnc_1', 0.8, 2500.0)
        assert decision.should_suppress is False

    def test_shelve_does_not_suppress_non_matching_type(self):
        mgr = _make_manager()
        mgr.add_rule(_make_shelve_rule(alarm_type_pattern='THERMAL*'))
        decision = mgr.evaluate('alarm_3', 'SAFETY_GUARD', 'cnc_1', 0.9, 1500.0)
        assert decision.should_suppress is False


# ---------------------------------------------------------------------------
# Maintenance window suppression
# ---------------------------------------------------------------------------

class TestMaintenanceWindow:
    def test_maintenance_suppresses_all_alarms_for_machine(self):
        mgr = _make_manager()
        mgr.add_rule(_make_maintenance_rule(machine_pattern='cnc_1', start=5000.0, end=6000.0))
        decision = mgr.evaluate('alarm_4', 'TOOL_WEAR', 'cnc_1', 0.5, 5500.0)
        assert decision.should_suppress is True
        assert 'maint_1' in decision.rule_id

    def test_maintenance_does_not_affect_other_machines(self):
        mgr = _make_manager()
        mgr.add_rule(_make_maintenance_rule(machine_pattern='cnc_1'))
        decision = mgr.evaluate('alarm_5', 'TOOL_WEAR', 'cnc_2', 0.5, 5500.0)
        assert decision.should_suppress is False


# ---------------------------------------------------------------------------
# Duplicate suppression
# ---------------------------------------------------------------------------

class TestDuplicateSuppression:
    def test_duplicate_rule_suppresses_after_threshold(self):
        mgr = _make_manager()
        dup_rule = SuppressionRule(
            rule_id='dup_1',
            name='Duplicate filter',
            rule_type='duplicate',
            alarm_type_pattern='VIBRATION*',
            machine_pattern='*',
            max_occurrences=3,
            window_sec=60.0,
            is_active=True,
            created_by='system',
            reason='Too many duplicates',
        )
        mgr.add_rule(dup_rule)

        base_time = 10000.0
        # First 2 alarms should NOT be suppressed (occurrences < max_occurrences)
        for i in range(2):
            d = mgr.evaluate(f'v_{i}', 'VIBRATION_HIGH', 'cnc_1', 0.6, base_time + i)
            assert d.should_suppress is False, f'alarm {i} should not be suppressed'

        # 3rd alarm triggers suppression (occurrences == max_occurrences)
        d = mgr.evaluate('v_2', 'VIBRATION_HIGH', 'cnc_1', 0.6, base_time + 2)
        assert d.should_suppress is True
        assert d.rule_id == 'dup_1'

        # 4th alarm also suppressed
        d = mgr.evaluate('v_3', 'VIBRATION_HIGH', 'cnc_1', 0.6, base_time + 3)
        assert d.should_suppress is True

    def test_duplicate_rule_resets_after_window(self):
        mgr = _make_manager()
        dup_rule = SuppressionRule(
            rule_id='dup_2',
            name='Duplicate filter short window',
            rule_type='duplicate',
            alarm_type_pattern='*',
            machine_pattern='*',
            max_occurrences=2,
            window_sec=10.0,
            is_active=True,
            created_by='system',
            reason='Short window dup',
        )
        mgr.add_rule(dup_rule)

        # Two alarms in-window
        mgr.evaluate('a1', 'THERMAL', 'cnc_1', 0.5, 100.0)
        mgr.evaluate('a2', 'THERMAL', 'cnc_1', 0.5, 105.0)
        # Third in-window: should suppress
        d = mgr.evaluate('a3', 'THERMAL', 'cnc_1', 0.5, 109.0)
        assert d.should_suppress is True

        # After window expires: should NOT suppress
        d = mgr.evaluate('a4', 'THERMAL', 'cnc_1', 0.5, 200.0)
        assert d.should_suppress is False


# ---------------------------------------------------------------------------
# Convenience methods
# ---------------------------------------------------------------------------

class TestConvenienceMethods:
    def test_shelve_alarm_type_creates_rule(self):
        mgr = _make_manager()
        rule = mgr.shelve_alarm_type('QUALITY*', 300.0, 'test', 'op_1')
        assert rule.rule_type == 'shelve'
        assert rule.alarm_type_pattern == 'QUALITY*'
        assert rule.created_by == 'op_1'
        # Rule should be in active rules
        active = mgr.get_active_rules()
        assert any(r.rule_id == rule.rule_id for r in active)

    def test_add_maintenance_window_creates_rule(self):
        mgr = _make_manager()
        rule = mgr.add_maintenance_window('cnc_3', 8000.0, 9000.0, 'PM')
        assert rule.rule_type == 'maintenance_window'
        assert rule.machine_pattern == 'cnc_3'
        assert rule.start_time == 8000.0
        assert rule.end_time == 9000.0

    def test_get_active_shelves(self):
        mgr = _make_manager()
        # Create a shelf that is still valid (end_time far in the future)
        rule = SuppressionRule(
            rule_id='shelve_future',
            name='Future shelve',
            rule_type='shelve',
            alarm_type_pattern='*',
            start_time=time.time() - 10,
            end_time=time.time() + 3600,
            is_active=True,
            created_by='op',
            reason='test',
        )
        mgr.add_rule(rule)
        shelves = mgr.get_active_shelves()
        assert len(shelves) == 1
        assert shelves[0].rule_id == 'shelve_future'

    def test_get_active_shelves_excludes_expired(self):
        mgr = _make_manager()
        rule = SuppressionRule(
            rule_id='shelve_expired',
            name='Expired shelve',
            rule_type='shelve',
            alarm_type_pattern='*',
            start_time=100.0,
            end_time=200.0,  # long in the past
            is_active=True,
            created_by='op',
            reason='test',
        )
        mgr.add_rule(rule)
        shelves = mgr.get_active_shelves()
        assert len(shelves) == 0


# ---------------------------------------------------------------------------
# Suppression history
# ---------------------------------------------------------------------------

class TestSuppressionHistory:
    def test_history_records_all_decisions(self):
        mgr = _make_manager()
        mgr.add_rule(_make_shelve_rule(start=0.0, end=9999.0))

        mgr.evaluate('a1', 'THERMAL_X', 'cnc_1', 0.5, 500.0)
        mgr.evaluate('a2', 'SAFETY', 'cnc_1', 0.9, 600.0)

        history = mgr.get_suppression_history()
        assert len(history) == 2
        # First alarm was suppressed (THERMAL* match), second was not
        assert history[0].should_suppress is True
        assert history[1].should_suppress is False

    def test_history_contains_timestamps(self):
        mgr = _make_manager()
        mgr.evaluate('a1', 'X', 'cnc_1', 0.1, 42.0)
        history = mgr.get_suppression_history()
        assert history[0].timestamp == 42.0


# ---------------------------------------------------------------------------
# Wildcard pattern matching
# ---------------------------------------------------------------------------

class TestPatternMatching:
    def test_star_matches_everything(self):
        mgr = _make_manager()
        rule = SuppressionRule(
            rule_id='catch_all',
            name='Catch all',
            rule_type='state_based',
            alarm_type_pattern='*',
            machine_pattern='*',
            is_active=True,
            reason='blanket suppress',
        )
        mgr.add_rule(rule)
        d = mgr.evaluate('a1', 'ANYTHING', 'any_machine', 0.1, 1.0)
        assert d.should_suppress is True

    def test_prefix_pattern(self):
        mgr = _make_manager()
        rule = SuppressionRule(
            rule_id='prefix_rule',
            name='Prefix',
            rule_type='state_based',
            alarm_type_pattern='THERMAL*',
            machine_pattern='cnc_*',
            is_active=True,
            reason='prefix test',
        )
        mgr.add_rule(rule)
        # Matching
        d = mgr.evaluate('a1', 'THERMAL_OVERTEMP', 'cnc_1', 0.5, 1.0)
        assert d.should_suppress is True
        # Non-matching alarm type
        d = mgr.evaluate('a2', 'SAFETY', 'cnc_1', 0.5, 2.0)
        assert d.should_suppress is False
        # Non-matching machine
        d = mgr.evaluate('a3', 'THERMAL_OVERTEMP', 'lathe_1', 0.5, 3.0)
        assert d.should_suppress is False
