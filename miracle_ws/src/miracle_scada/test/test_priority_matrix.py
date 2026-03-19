"""Tests for the ISA-18.2 Alarm Priority Matrix.

Uses the same mock pattern as test_capability_profiler.py — ROS2 modules are
stubbed so the pure dataclasses and classes can be tested without a ROS2
installation.
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

# Clear any previously cached import of alarm_manager
sys.modules.pop('miracle_scada.alarm_manager', None)

import pytest

from miracle_scada.alarm_manager import (
    AlarmPriority,
    PriorityAssignment,
    AlarmPriorityMatrix,
)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestAlarmPriorityDataclass:
    """Verify AlarmPriority dataclass construction."""

    def test_fields(self):
        p = AlarmPriority(
            priority_level=1,
            name='critical',
            color='red',
            response_time_sec=60.0,
            escalation_delay_sec=120.0,
        )
        assert p.priority_level == 1
        assert p.name == 'critical'
        assert p.color == 'red'
        assert p.response_time_sec == 60.0
        assert p.escalation_delay_sec == 120.0


class TestAlarmPriorityMatrixLookup:
    """Test get_priority lookups across the 4x4 matrix."""

    def setup_method(self):
        self.matrix = AlarmPriorityMatrix()

    def test_catastrophic_likely_is_critical(self):
        """C4 + L4 => priority level 1 (critical)."""
        p = self.matrix.get_priority(consequence=4, likelihood=4)
        assert p.priority_level == 1
        assert p.name == 'critical'

    def test_catastrophic_possible_is_critical(self):
        """C4 + L3 => priority level 1 (critical)."""
        p = self.matrix.get_priority(consequence=4, likelihood=3)
        assert p.priority_level == 1
        assert p.name == 'critical'

    def test_serious_likely_is_high(self):
        """C3 + L4 => priority level 2 (high)."""
        p = self.matrix.get_priority(consequence=3, likelihood=4)
        assert p.priority_level == 2
        assert p.name == 'high'

    def test_serious_possible_is_high(self):
        """C3 + L3 => priority level 2 (high)."""
        p = self.matrix.get_priority(consequence=3, likelihood=3)
        assert p.priority_level == 2

    def test_moderate_likely_is_medium(self):
        """C2 + L4 => priority level 3 (medium)."""
        p = self.matrix.get_priority(consequence=2, likelihood=4)
        assert p.priority_level == 3
        assert p.name == 'medium'

    def test_minor_rare_is_low(self):
        """C1 + L1 => priority level 4 (low)."""
        p = self.matrix.get_priority(consequence=1, likelihood=1)
        assert p.priority_level == 4
        assert p.name == 'low'

    def test_invalid_consequence_raises(self):
        with pytest.raises(ValueError, match='consequence must be 1'):
            self.matrix.get_priority(consequence=0, likelihood=2)

    def test_invalid_likelihood_raises(self):
        with pytest.raises(ValueError, match='likelihood must be 1'):
            self.matrix.get_priority(consequence=2, likelihood=5)


class TestAlarmPriorityMatrixAssignment:
    """Test assign_priority, get_assignment, get_all_assignments."""

    def setup_method(self):
        self.matrix = AlarmPriorityMatrix()

    def test_assign_and_retrieve(self):
        assignment = self.matrix.assign_priority(
            alarm_type='spindle_overload',
            consequence=4,
            likelihood=3,
            rationale='Catastrophic spindle failure with possible occurrence',
        )
        assert isinstance(assignment, PriorityAssignment)
        assert assignment.alarm_type == 'spindle_overload'
        assert assignment.assigned_priority.name == 'critical'
        assert assignment.rationale == 'Catastrophic spindle failure with possible occurrence'

        retrieved = self.matrix.get_assignment('spindle_overload')
        assert retrieved is assignment

    def test_get_assignment_missing_returns_none(self):
        assert self.matrix.get_assignment('nonexistent') is None

    def test_get_all_assignments_sorted(self):
        self.matrix.assign_priority('low_alarm', 1, 1, 'minor/rare')
        self.matrix.assign_priority('critical_alarm', 4, 4, 'catastrophic/likely')
        self.matrix.assign_priority('medium_alarm', 2, 3, 'moderate/possible')

        all_assignments = self.matrix.get_all_assignments()
        levels = [a.assigned_priority.priority_level for a in all_assignments]
        assert levels == sorted(levels), 'Assignments should be sorted by priority level (ascending)'
        assert all_assignments[0].alarm_type == 'critical_alarm'


class TestResponseTime:
    """Test get_response_time."""

    def setup_method(self):
        self.matrix = AlarmPriorityMatrix()

    def test_critical_response_time(self):
        assert self.matrix.get_response_time(1) == 60.0

    def test_low_response_time(self):
        assert self.matrix.get_response_time(4) == 7200.0

    def test_invalid_level_raises(self):
        with pytest.raises(ValueError, match='No priority definition'):
            self.matrix.get_response_time(99)


class TestRationalize:
    """Test rationalize method for inconsistency detection."""

    def setup_method(self):
        self.matrix = AlarmPriorityMatrix()

    def test_no_issues_when_consistent(self):
        """Same consequence + same likelihood => same priority => no issues."""
        self.matrix.assign_priority('alarm_a', 3, 3, 'serious/possible')
        self.matrix.assign_priority('alarm_b', 3, 3, 'serious/possible too')
        issues = self.matrix.rationalize()
        assert issues == []

    def test_detects_inconsistency(self):
        """Same consequence but different likelihoods yield different priorities."""
        self.matrix.assign_priority('alarm_x', 3, 1, 'serious/rare => medium')
        self.matrix.assign_priority('alarm_y', 3, 4, 'serious/likely => high')
        issues = self.matrix.rationalize()
        assert len(issues) == 1
        assert 'Consequence 3' in issues[0]
        assert 'alarm_x' in issues[0]
        assert 'alarm_y' in issues[0]

    def test_rationalize_with_external_assignments(self):
        """Pass explicit assignments instead of using internal state."""
        p1 = self.matrix.get_priority(4, 4)  # critical
        p2 = self.matrix.get_priority(4, 1)  # high
        external = [
            PriorityAssignment('ext_a', 4, 4, p1, ''),
            PriorityAssignment('ext_b', 4, 1, p2, ''),
        ]
        issues = self.matrix.rationalize(external)
        assert len(issues) == 1
        assert 'Consequence 4' in issues[0]
