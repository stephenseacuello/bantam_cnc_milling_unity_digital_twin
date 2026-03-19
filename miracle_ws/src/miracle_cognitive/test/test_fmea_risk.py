"""Tests for FMEARiskCalculator FMEA Risk Priority Number analysis.

Covers RPN calculation, risk level classification, failure mode registration,
report generation, critical mode filtering, action suggestions, and
before/after comparison.
"""

import sys
from unittest.mock import MagicMock

for mod in [
    'miracle_core.datatypes', 'miracle_core.constants',
    'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
    'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
    'miracle_msgs', 'miracle_msgs.msg',
]:
    sys.modules.setdefault(mod, MagicMock())

import pytest
from miracle_cognitive.knowledge.reasoning_engine import (
    FMEARiskCalculator,
    FailureMode,
    FMEAReport,
)


@pytest.fixture
def calculator():
    return FMEARiskCalculator()


@pytest.fixture
def sample_modes():
    """A set of representative failure modes spanning risk levels."""
    return [
        FailureMode(
            mode_id='FM-001',
            description='Spindle bearing wear',
            component='Spindle',
            severity=8,
            occurrence=4,
            detection=6,
            current_controls='Vibration monitoring',
        ),
        FailureMode(
            mode_id='FM-002',
            description='Coolant leak',
            component='Coolant system',
            severity=3,
            occurrence=2,
            detection=2,
            current_controls='Visual inspection',
        ),
        FailureMode(
            mode_id='FM-003',
            description='Tool breakage',
            component='Cutting tool',
            severity=9,
            occurrence=5,
            detection=7,
            current_controls='Force monitoring',
        ),
        FailureMode(
            mode_id='FM-004',
            description='Axis backlash',
            component='Ball screw',
            severity=6,
            occurrence=3,
            detection=5,
            current_controls='Periodic calibration',
        ),
    ]


# -- RPN calculation -------------------------------------------------------

class TestCalculateRPN:

    def test_basic_rpn(self, calculator):
        """RPN = severity * occurrence * detection."""
        assert calculator.calculate_rpn(5, 3, 4) == 60

    def test_maximum_rpn(self, calculator):
        assert calculator.calculate_rpn(10, 10, 10) == 1000

    def test_minimum_rpn(self, calculator):
        assert calculator.calculate_rpn(1, 1, 1) == 1

    def test_invalid_value_raises(self, calculator):
        with pytest.raises(ValueError):
            calculator.calculate_rpn(0, 5, 5)
        with pytest.raises(ValueError):
            calculator.calculate_rpn(5, 11, 5)
        with pytest.raises(ValueError):
            calculator.calculate_rpn(5, 5, 0)


# -- Risk level classification ---------------------------------------------

class TestRiskLevel:

    @pytest.mark.parametrize('rpn, expected', [
        (1, 'low'),
        (49, 'low'),
        (50, 'medium'),
        (100, 'medium'),
        (101, 'high'),
        (200, 'high'),
        (201, 'critical'),
        (1000, 'critical'),
    ])
    def test_risk_boundaries(self, calculator, rpn, expected):
        assert calculator.get_risk_level(rpn) == expected


# -- Failure mode registration ---------------------------------------------

class TestAddFailureMode:

    def test_auto_calculates_rpn(self, calculator):
        mode = FailureMode(
            mode_id='FM-T1', description='Test', component='X',
            severity=5, occurrence=4, detection=3,
        )
        result = calculator.add_failure_mode(mode)
        assert result.rpn == 60

    def test_mode_stored(self, calculator):
        mode = FailureMode(
            mode_id='FM-T2', description='Test2', component='Y',
            severity=2, occurrence=2, detection=2,
        )
        calculator.add_failure_mode(mode)
        assert 'FM-T2' in calculator._modes


# -- Report generation -----------------------------------------------------

class TestGetReport:

    def test_report_structure(self, calculator, sample_modes):
        for m in sample_modes:
            calculator.add_failure_mode(m)
        report = calculator.get_report()

        assert isinstance(report, FMEAReport)
        assert len(report.failure_modes) == 4
        assert report.total_rpn == sum(m.rpn for m in report.failure_modes)
        assert report.avg_rpn == report.total_rpn / 4
        assert report.timestamp > 0

    def test_modes_sorted_descending(self, calculator, sample_modes):
        for m in sample_modes:
            calculator.add_failure_mode(m)
        report = calculator.get_report()
        rpns = [m.rpn for m in report.failure_modes]
        assert rpns == sorted(rpns, reverse=True)

    def test_empty_report(self, calculator):
        report = calculator.get_report()
        assert report.total_rpn == 0
        assert report.avg_rpn == 0.0
        assert len(report.failure_modes) == 0


# -- Critical modes ---------------------------------------------------------

class TestCriticalModes:

    def test_default_threshold(self, calculator, sample_modes):
        for m in sample_modes:
            calculator.add_failure_mode(m)
        critical = calculator.get_critical_modes()
        # FM-001 RPN=192, FM-003 RPN=315 -> only FM-003 > 125 default and FM-001=192>125
        for m in critical:
            assert m.rpn > 125

    def test_custom_threshold(self, calculator, sample_modes):
        for m in sample_modes:
            calculator.add_failure_mode(m)
        critical = calculator.get_critical_modes(threshold=50)
        assert all(m.rpn > 50 for m in critical)

    def test_sorted_descending(self, calculator, sample_modes):
        for m in sample_modes:
            calculator.add_failure_mode(m)
        critical = calculator.get_critical_modes(threshold=0)
        rpns = [m.rpn for m in critical]
        assert rpns == sorted(rpns, reverse=True)


# -- Action suggestions ----------------------------------------------------

class TestSuggestAction:

    def test_suggests_detection_improvement(self, calculator):
        """When detection is the highest factor, suggest improving detection."""
        mode = FailureMode(
            mode_id='FM-D', description='Hard to detect',
            component='Sensor', severity=3, occurrence=2, detection=9,
        )
        calculator.add_failure_mode(mode)
        action = calculator.suggest_action(mode)
        assert 'detection' in action.lower()

    def test_suggests_occurrence_reduction(self, calculator):
        """When occurrence is the highest factor, suggest reducing occurrence."""
        mode = FailureMode(
            mode_id='FM-O', description='Frequent failure',
            component='Belt', severity=2, occurrence=9, detection=3,
        )
        calculator.add_failure_mode(mode)
        action = calculator.suggest_action(mode)
        assert 'occurrence' in action.lower()

    def test_suggests_severity_reduction(self, calculator):
        """When severity is the highest factor, suggest reducing severity."""
        mode = FailureMode(
            mode_id='FM-S', description='Catastrophic failure',
            component='Frame', severity=10, occurrence=2, detection=3,
        )
        calculator.add_failure_mode(mode)
        action = calculator.suggest_action(mode)
        assert 'severity' in action.lower()


# -- Before / after comparison ---------------------------------------------

class TestCompareBeforeAfter:

    def test_rpn_reduction(self, calculator):
        mode = FailureMode(
            mode_id='FM-C1', description='Compare test',
            component='Z', severity=8, occurrence=6, detection=5,
        )
        calculator.add_failure_mode(mode)
        result = calculator.compare_before_after('FM-C1', 4, 3, 2)

        assert result['before_rpn'] == 8 * 6 * 5  # 240
        assert result['after_rpn'] == 4 * 3 * 2   # 24
        assert result['rpn_reduction'] == 240 - 24
        assert result['reduction_pct'] == round((216 / 240) * 100, 2)
        assert result['before_risk'] == 'critical'
        assert result['after_risk'] == 'low'

    def test_unknown_mode_raises(self, calculator):
        with pytest.raises(KeyError):
            calculator.compare_before_after('NOPE', 1, 1, 1)

    def test_no_change(self, calculator):
        mode = FailureMode(
            mode_id='FM-NC', description='No change',
            component='A', severity=5, occurrence=5, detection=5,
        )
        calculator.add_failure_mode(mode)
        result = calculator.compare_before_after('FM-NC', 5, 5, 5)
        assert result['rpn_reduction'] == 0
        assert result['reduction_pct'] == 0.0
