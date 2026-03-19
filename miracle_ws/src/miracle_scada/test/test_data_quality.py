"""Tests for DataQualityScorer — sensor/process data quality assessment.

Uses the same mock pattern as test_capability_profiler.py — ROS2 modules are
stubbed so the pure dataclasses and classes can be tested without a ROS2
installation.
"""

import sys
import math
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

sys.modules.pop('miracle_scada.kpi_calculator', None)

import pytest

from miracle_scada.kpi_calculator import (
    DataQualityCheck,
    DataQualityReport,
    DataQualityScorer,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def scorer():
    return DataQualityScorer()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestCompleteness:
    """Completeness check — non-None/non-NaN percentage."""

    def test_all_valid(self, scorer):
        data = [1.0, 2.0, 3.0, 4.0, 5.0]
        report = scorer.assess(data, 'spindle_speed')
        completeness = next(c for c in report.checks if c.check_name == 'completeness')
        assert completeness.score == 100.0
        assert completeness.passed is True

    def test_with_none_and_nan(self, scorer):
        data = [1.0, None, 3.0, float('nan'), 5.0]
        report = scorer.assess(data, 'spindle_speed')
        completeness = next(c for c in report.checks if c.check_name == 'completeness')
        assert completeness.score == 60.0  # 3/5
        assert completeness.passed is False


class TestRangeValidity:
    """Range validity check — values within configured bounds."""

    def test_all_in_range(self, scorer):
        scorer.set_bounds('temp_sensor', 10.0, 50.0)
        data = [20.0, 30.0, 40.0, 25.0]
        report = scorer.assess(data, 'temp_sensor')
        rng = next(c for c in report.checks if c.check_name == 'range_validity')
        assert rng.score == 100.0
        assert rng.passed is True

    def test_out_of_range(self, scorer):
        scorer.set_bounds('temp_sensor', 10.0, 50.0)
        data = [5.0, 20.0, 55.0, 30.0]  # 2 of 4 out of range
        report = scorer.assess(data, 'temp_sensor')
        rng = next(c for c in report.checks if c.check_name == 'range_validity')
        assert rng.score == 50.0
        assert rng.passed is False


class TestStuckSensor:
    """Resolution / stuck sensor detection."""

    def test_stuck_sensor_detected(self, scorer):
        data = [42.0] * 20
        check = scorer.get_stuck_sensor_check(data)
        assert check.passed is False
        assert check.score == 0.0
        assert 'stuck' in check.details.lower()

    def test_healthy_sensor(self, scorer):
        data = [1.0, 2.0, 3.0, 4.0, 5.0]
        check = scorer.get_stuck_sensor_check(data)
        assert check.passed is True
        assert check.score > 0.0


class TestNoiseLevel:
    """Noise level — coefficient of variation assessment."""

    def test_low_noise(self, scorer):
        # Very low CV: values tightly clustered around 100
        data = [100.0, 100.1, 99.9, 100.05, 99.95]
        check = scorer.get_noise_level(data)
        assert check.passed is True
        assert check.score > 90.0

    def test_high_noise(self, scorer):
        # High CV: large spread relative to mean
        data = [1.0, 100.0, 1.0, 100.0, 1.0, 100.0]
        check = scorer.get_noise_level(data)
        assert check.passed is False
        assert check.score < 50.0


class TestFreshness:
    """Freshness check — data age within threshold."""

    def test_fresh_data(self, scorer):
        scorer.set_freshness_threshold(10.0)
        scorer.record_data_time('sensor_a', time.time())
        report = scorer.assess([1.0, 2.0], 'sensor_a')
        freshness = next(c for c in report.checks if c.check_name == 'freshness')
        assert freshness.passed is True
        assert freshness.score > 90.0

    def test_stale_data(self, scorer):
        scorer.set_freshness_threshold(5.0)
        # Record a timestamp far in the past
        scorer.record_data_time('sensor_b', time.time() - 100.0)
        check = scorer._check_freshness('sensor_b')
        assert check.passed is False
        assert check.score == 0.0


class TestConsistency:
    """Consistency check — sudden jumps detection."""

    def test_consistent_data(self, scorer):
        data = [10.0, 10.1, 9.9, 10.05, 9.95, 10.0, 10.02]
        report = scorer.assess(data, 'feed_rate')
        consistency = next(c for c in report.checks if c.check_name == 'consistency')
        assert consistency.passed is True
        assert consistency.score == 100.0

    def test_inconsistent_data(self, scorer):
        # A single extreme outlier in otherwise stable data
        data = [10.0] * 50 + [10000.0]
        report = scorer.assess(data, 'feed_rate')
        consistency = next(c for c in report.checks if c.check_name == 'consistency')
        assert consistency.score < 100.0


class TestOverallReport:
    """End-to-end assess() produces a valid DataQualityReport."""

    def test_report_structure(self, scorer):
        data = [1.0, 2.0, 3.0, 4.0, 5.0]
        report = scorer.assess(data, 'vibration')
        assert isinstance(report, DataQualityReport)
        assert 0.0 <= report.overall_score <= 100.0
        assert len(report.checks) == 6
        assert report.data_source == 'vibration'
        assert report.record_count == 5
        assert report.timestamp > 0

    def test_recommendations_on_bad_data(self, scorer):
        scorer.set_bounds('pressure', 0.0, 10.0)
        data = [None, None, 999.0, 999.0, 999.0]
        report = scorer.assess(data, 'pressure')
        # Should have at least one recommendation
        assert len(report.recommendations) > 0
