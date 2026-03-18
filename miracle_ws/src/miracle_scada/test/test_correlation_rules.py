"""Tests for the AlarmCorrelationRuleEngine.

Validates rule management, alarm submission, condition matching,
built-in rules, and correlation history tracking.

Uses the same ROS2 mock pattern as test_capability_profiler.py.
"""

import sys
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

# Clear cached module so our mocks take effect
sys.modules.pop('miracle_scada.alert_correlator', None)

import pytest
import time

from miracle_scada.alert_correlator import (
    AlarmCorrelationRule,
    AlarmCorrelationRuleEngine,
    AlarmEvent,
    CorrelationMatch,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_alarm(
    alarm_id: str,
    alarm_type: str,
    source: str = 'cnc_1',
    severity: float = 0.8,
    timestamp: float = 100.0,
    properties: dict = None,
) -> AlarmEvent:
    """Create an AlarmEvent with sensible defaults."""
    return AlarmEvent(
        alarm_id=alarm_id,
        alarm_type=alarm_type,
        source=source,
        severity=severity,
        timestamp=timestamp,
        properties=properties or {},
    )


def _make_rule(
    rule_id: str = 'TEST_RULE',
    name: str = 'Test Rule',
    conditions: list = None,
    time_window_sec: float = 30.0,
    min_matches: int = 2,
    action: str = 'group',
    priority: int = 5,
    description: str = 'A test rule',
) -> AlarmCorrelationRule:
    """Create an AlarmCorrelationRule with sensible defaults."""
    if conditions is None:
        conditions = [
            {'field': 'alarm_type', 'operator': 'eq', 'value': 'temperature'},
            {'field': 'alarm_type', 'operator': 'eq', 'value': 'vibration'},
        ]
    return AlarmCorrelationRule(
        rule_id=rule_id,
        name=name,
        conditions=conditions,
        time_window_sec=time_window_sec,
        min_matches=min_matches,
        action=action,
        priority=priority,
        description=description,
    )


# ---------------------------------------------------------------------------
# Test: Built-in rules are registered
# ---------------------------------------------------------------------------

class TestBuiltinRules:
    def test_three_builtin_rules_present(self):
        """Engine starts with THERMAL_CASCADE, TOOL_FAILURE_CHAIN, POWER_SEQUENCE."""
        engine = AlarmCorrelationRuleEngine()
        rules = engine.get_rules()
        rule_ids = {r.rule_id for r in rules}
        assert 'THERMAL_CASCADE' in rule_ids
        assert 'TOOL_FAILURE_CHAIN' in rule_ids
        assert 'POWER_SEQUENCE' in rule_ids
        assert len(rules) == 3

    def test_builtin_rule_properties(self):
        """Verify specific properties of the THERMAL_CASCADE rule."""
        engine = AlarmCorrelationRuleEngine()
        rules = {r.rule_id: r for r in engine.get_rules()}
        tc = rules['THERMAL_CASCADE']
        assert tc.time_window_sec == 30.0
        assert tc.min_matches == 2
        assert tc.action == 'group'

        tfc = rules['TOOL_FAILURE_CHAIN']
        assert tfc.time_window_sec == 60.0
        assert tfc.action == 'escalate'

        ps = rules['POWER_SEQUENCE']
        assert ps.time_window_sec == 10.0
        assert ps.action == 'suppress_secondary'


# ---------------------------------------------------------------------------
# Test: Rule management (add / remove / get)
# ---------------------------------------------------------------------------

class TestRuleManagement:
    def test_add_custom_rule(self):
        """Adding a custom rule increases the rule count."""
        engine = AlarmCorrelationRuleEngine()
        custom = _make_rule(rule_id='CUSTOM_1', priority=50)
        engine.add_rule(custom)
        rules = engine.get_rules()
        assert len(rules) == 4
        assert any(r.rule_id == 'CUSTOM_1' for r in rules)

    def test_remove_rule(self):
        """Removing a rule by id reduces the rule count."""
        engine = AlarmCorrelationRuleEngine()
        assert engine.remove_rule('THERMAL_CASCADE') is True
        rules = engine.get_rules()
        assert len(rules) == 2
        assert not any(r.rule_id == 'THERMAL_CASCADE' for r in rules)

    def test_remove_nonexistent_rule(self):
        """Removing a rule that does not exist returns False."""
        engine = AlarmCorrelationRuleEngine()
        assert engine.remove_rule('DOES_NOT_EXIST') is False
        assert len(engine.get_rules()) == 3

    def test_get_rules_sorted_by_priority(self):
        """get_rules returns rules sorted by priority descending."""
        engine = AlarmCorrelationRuleEngine()
        engine.add_rule(_make_rule(rule_id='LOW', priority=1))
        engine.add_rule(_make_rule(rule_id='HIGH', priority=100))
        rules = engine.get_rules()
        priorities = [r.priority for r in rules]
        assert priorities == sorted(priorities, reverse=True)


# ---------------------------------------------------------------------------
# Test: Condition matching
# ---------------------------------------------------------------------------

class TestConditionMatching:
    def test_eq_operator(self):
        """'eq' operator matches when field equals value."""
        alarm = _make_alarm('a1', 'temperature')
        assert AlarmCorrelationRuleEngine._check_condition(
            alarm, {'field': 'alarm_type', 'operator': 'eq', 'value': 'temperature'}
        ) is True
        assert AlarmCorrelationRuleEngine._check_condition(
            alarm, {'field': 'alarm_type', 'operator': 'eq', 'value': 'vibration'}
        ) is False

    def test_ne_operator(self):
        """'ne' operator matches when field does not equal value."""
        alarm = _make_alarm('a1', 'temperature')
        assert AlarmCorrelationRuleEngine._check_condition(
            alarm, {'field': 'alarm_type', 'operator': 'ne', 'value': 'vibration'}
        ) is True

    def test_gt_lt_operators(self):
        """'gt' and 'lt' operators compare numerically."""
        alarm = _make_alarm('a1', 'temperature', severity=0.9)
        assert AlarmCorrelationRuleEngine._check_condition(
            alarm, {'field': 'severity', 'operator': 'gt', 'value': 0.5}
        ) is True
        assert AlarmCorrelationRuleEngine._check_condition(
            alarm, {'field': 'severity', 'operator': 'lt', 'value': 0.5}
        ) is False

    def test_contains_operator(self):
        """'contains' checks substring membership."""
        alarm = _make_alarm('a1', 'high_temperature_warning')
        assert AlarmCorrelationRuleEngine._check_condition(
            alarm, {'field': 'alarm_type', 'operator': 'contains', 'value': 'temperature'}
        ) is True

    def test_properties_lookup(self):
        """Condition can match against properties dict entries."""
        alarm = _make_alarm('a1', 'temperature', properties={'zone': 'spindle'})
        assert AlarmCorrelationRuleEngine._check_condition(
            alarm, {'field': 'zone', 'operator': 'eq', 'value': 'spindle'}
        ) is True

    def test_missing_field_returns_false(self):
        """Condition on a field that does not exist returns False."""
        alarm = _make_alarm('a1', 'temperature')
        assert AlarmCorrelationRuleEngine._check_condition(
            alarm, {'field': 'nonexistent', 'operator': 'eq', 'value': 'x'}
        ) is False


# ---------------------------------------------------------------------------
# Test: Thermal Cascade built-in rule fires correctly
# ---------------------------------------------------------------------------

class TestThermalCascadeRule:
    def test_thermal_cascade_match(self):
        """Temperature + vibration within 30s triggers THERMAL_CASCADE."""
        engine = AlarmCorrelationRuleEngine()
        t = 1000.0
        engine.submit_alarm(_make_alarm('a1', 'temperature', timestamp=t))
        matches = engine.submit_alarm(_make_alarm('a2', 'vibration', timestamp=t + 10))
        thermal = [m for m in matches if m.rule_id == 'THERMAL_CASCADE']
        assert len(thermal) == 1
        assert set(thermal[0].matched_alarms) == {'a1', 'a2'}
        assert thermal[0].action == 'group'

    def test_thermal_cascade_outside_window(self):
        """Temperature + vibration >30s apart does NOT trigger."""
        engine = AlarmCorrelationRuleEngine()
        t = 1000.0
        engine.submit_alarm(_make_alarm('a1', 'temperature', timestamp=t))
        matches = engine.submit_alarm(_make_alarm('a2', 'vibration', timestamp=t + 50))
        thermal = [m for m in matches if m.rule_id == 'THERMAL_CASCADE']
        assert len(thermal) == 0


# ---------------------------------------------------------------------------
# Test: Tool Failure Chain built-in rule
# ---------------------------------------------------------------------------

class TestToolFailureChainRule:
    def test_tool_failure_chain_match(self):
        """Force + wear + quality within 60s triggers TOOL_FAILURE_CHAIN."""
        engine = AlarmCorrelationRuleEngine()
        t = 2000.0
        engine.submit_alarm(_make_alarm('f1', 'force', timestamp=t))
        engine.submit_alarm(_make_alarm('w1', 'wear', timestamp=t + 20))
        matches = engine.submit_alarm(_make_alarm('q1', 'quality', timestamp=t + 40))
        chain = [m for m in matches if m.rule_id == 'TOOL_FAILURE_CHAIN']
        assert len(chain) == 1
        assert chain[0].action == 'escalate'
        assert chain[0].root_alarm_id == 'f1'  # earliest alarm


# ---------------------------------------------------------------------------
# Test: Power Sequence built-in rule
# ---------------------------------------------------------------------------

class TestPowerSequenceRule:
    def test_power_sequence_match(self):
        """Power + spindle + feed within 10s triggers POWER_SEQUENCE."""
        engine = AlarmCorrelationRuleEngine()
        t = 3000.0
        engine.submit_alarm(_make_alarm('p1', 'power', timestamp=t))
        engine.submit_alarm(_make_alarm('s1', 'spindle', timestamp=t + 3))
        matches = engine.submit_alarm(_make_alarm('fd1', 'feed', timestamp=t + 6))
        power = [m for m in matches if m.rule_id == 'POWER_SEQUENCE']
        assert len(power) == 1
        assert power[0].action == 'suppress_secondary'

    def test_power_sequence_outside_window(self):
        """Power + spindle + feed >10s apart does NOT trigger."""
        engine = AlarmCorrelationRuleEngine()
        t = 3000.0
        engine.submit_alarm(_make_alarm('p1', 'power', timestamp=t))
        engine.submit_alarm(_make_alarm('s1', 'spindle', timestamp=t + 5))
        matches = engine.submit_alarm(_make_alarm('fd1', 'feed', timestamp=t + 15))
        power = [m for m in matches if m.rule_id == 'POWER_SEQUENCE']
        assert len(power) == 0


# ---------------------------------------------------------------------------
# Test: Correlation history
# ---------------------------------------------------------------------------

class TestCorrelationHistory:
    def test_history_accumulates(self):
        """Every successful match is recorded in get_correlation_history."""
        engine = AlarmCorrelationRuleEngine()
        t = 4000.0
        engine.submit_alarm(_make_alarm('a1', 'temperature', timestamp=t))
        engine.submit_alarm(_make_alarm('a2', 'vibration', timestamp=t + 5))
        history = engine.get_correlation_history()
        assert len(history) >= 1
        assert all(isinstance(m, CorrelationMatch) for m in history)

    def test_empty_history_initially(self):
        """A fresh engine has no correlation history."""
        engine = AlarmCorrelationRuleEngine()
        assert engine.get_correlation_history() == []

    def test_evaluate_rules_does_not_record(self):
        """evaluate_rules returns matches but does not add to history."""
        engine = AlarmCorrelationRuleEngine()
        t = 5000.0
        engine.submit_alarm(_make_alarm('a1', 'temperature', timestamp=t))
        alarm2 = _make_alarm('a2', 'vibration', timestamp=t + 5)
        result = engine.evaluate_rules(alarm2)
        # evaluate_rules should not persist to history
        # Only the alarm submitted via submit_alarm('a1') could have generated history
        # but a single temperature alarm can't match THERMAL_CASCADE alone.
        # So history should still be empty.
        assert engine.get_correlation_history() == []


# ---------------------------------------------------------------------------
# Test: Custom rules and confidence
# ---------------------------------------------------------------------------

class TestCustomRuleAndConfidence:
    def test_custom_rule_match(self):
        """A user-defined rule can match submitted alarms."""
        engine = AlarmCorrelationRuleEngine()
        # Remove built-in rules for clarity
        for rid in ['THERMAL_CASCADE', 'TOOL_FAILURE_CHAIN', 'POWER_SEQUENCE']:
            engine.remove_rule(rid)

        engine.add_rule(_make_rule(
            rule_id='CUSTOM_SEVERITY',
            conditions=[
                {'field': 'severity', 'operator': 'gt', 'value': 0.7},
            ],
            min_matches=2,
            action='escalate',
            priority=15,
        ))

        t = 6000.0
        engine.submit_alarm(_make_alarm('h1', 'any', severity=0.9, timestamp=t))
        matches = engine.submit_alarm(_make_alarm('h2', 'other', severity=0.85, timestamp=t + 5))
        custom = [m for m in matches if m.rule_id == 'CUSTOM_SEVERITY']
        assert len(custom) == 1
        assert custom[0].confidence > 0.0
        assert custom[0].confidence <= 1.0

    def test_confidence_bounded(self):
        """Confidence never exceeds 1.0."""
        engine = AlarmCorrelationRuleEngine()
        t = 7000.0
        engine.submit_alarm(_make_alarm('x1', 'temperature', severity=1.0, timestamp=t))
        matches = engine.submit_alarm(_make_alarm('x2', 'vibration', severity=1.0, timestamp=t + 1))
        for m in matches:
            assert m.confidence <= 1.0
