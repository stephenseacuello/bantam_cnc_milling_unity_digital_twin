"""Tests for MetricAggregationService, MetricPoint, and AggregatedMetric.

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

sys.modules.pop('miracle_scada.kpi_calculator', None)

import math
import pytest
from typing import List

from miracle_scada.kpi_calculator import (
    MetricPoint,
    AggregatedMetric,
    MetricAggregationService,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_service_with_points(
    name: str = "cpu_temp",
    values: List[float] = None,
    start_ts: float = 1000.0,
    step: float = 1.0,
) -> MetricAggregationService:
    """Create a service pre-loaded with points."""
    if values is None:
        values = [10.0, 20.0, 30.0, 40.0, 50.0]
    svc = MetricAggregationService()
    for i, v in enumerate(values):
        svc.record(MetricPoint(
            name=name,
            value=v,
            timestamp=start_ts + i * step,
            tags={"machine": "cnc_1"},
        ))
    return svc


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestMetricPointDataclass:
    """Verify MetricPoint dataclass basics."""

    def test_creation_defaults(self):
        pt = MetricPoint(name="temp", value=42.0, timestamp=100.0)
        assert pt.name == "temp"
        assert pt.value == 42.0
        assert pt.timestamp == 100.0
        assert pt.tags == {}

    def test_creation_with_tags(self):
        pt = MetricPoint(name="rpm", value=3000.0, timestamp=200.0,
                         tags={"axis": "x"})
        assert pt.tags == {"axis": "x"}


class TestRecordAndGetLatest:
    """Test record() and get_latest()."""

    def test_record_and_retrieve(self):
        svc = _make_service_with_points(values=[1.0, 2.0, 3.0])
        latest = svc.get_latest("cpu_temp", count=2)
        assert len(latest) == 2
        assert latest[0].value == 2.0
        assert latest[1].value == 3.0

    def test_get_latest_returns_all_when_count_exceeds(self):
        svc = _make_service_with_points(values=[5.0, 6.0])
        latest = svc.get_latest("cpu_temp", count=100)
        assert len(latest) == 2

    def test_get_latest_unknown_name(self):
        svc = MetricAggregationService()
        assert svc.get_latest("nonexistent") == []


class TestGetMetricNames:
    """Test get_metric_names()."""

    def test_returns_sorted_names(self):
        svc = MetricAggregationService()
        svc.record(MetricPoint("zeta", 1.0, 100.0))
        svc.record(MetricPoint("alpha", 2.0, 101.0))
        svc.record(MetricPoint("mu", 3.0, 102.0))
        assert svc.get_metric_names() == ["alpha", "mu", "zeta"]

    def test_empty_service(self):
        svc = MetricAggregationService()
        assert svc.get_metric_names() == []


class TestAggregate:
    """Test aggregate() bucketing and stats."""

    def test_single_bucket(self):
        values = [10.0, 20.0, 30.0, 40.0, 50.0]
        svc = _make_service_with_points(values=values, start_ts=0.0, step=1.0)
        results = svc.aggregate("cpu_temp", 0.0, 5.0, interval_sec=10.0)
        assert len(results) == 1
        agg = results[0]
        assert agg.count == 5
        assert agg.mean == pytest.approx(30.0)
        assert agg.min_val == 10.0
        assert agg.max_val == 50.0
        assert agg.sum_val == pytest.approx(150.0)
        # Population std dev of [10,20,30,40,50]
        expected_std = math.sqrt(200.0)
        assert agg.std_dev == pytest.approx(expected_std, abs=1e-6)

    def test_multiple_buckets(self):
        # 10 points at ts=0..9, values 0..9, two 5-sec buckets
        svc = _make_service_with_points(
            values=[float(i) for i in range(10)],
            start_ts=0.0,
            step=1.0,
        )
        results = svc.aggregate("cpu_temp", 0.0, 10.0, interval_sec=5.0)
        assert len(results) == 2
        # First bucket: ts [0,5) -> values 0,1,2,3,4
        assert results[0].count == 5
        assert results[0].mean == pytest.approx(2.0)
        # Second bucket: ts [5,10] -> values 5,6,7,8,9
        assert results[1].count == 5
        assert results[1].mean == pytest.approx(7.0)

    def test_empty_aggregate(self):
        svc = MetricAggregationService()
        assert svc.aggregate("nope", 0.0, 10.0, 1.0) == []


class TestGetPercentile:
    """Test get_percentile()."""

    def test_median(self):
        svc = _make_service_with_points(
            values=[10.0, 20.0, 30.0, 40.0, 50.0],
            start_ts=0.0,
        )
        p50 = svc.get_percentile("cpu_temp", 50.0, 0.0, 10.0)
        assert p50 == pytest.approx(30.0)

    def test_p99_close_to_max(self):
        svc = _make_service_with_points(
            values=[float(i) for i in range(1, 101)],
            start_ts=0.0,
            step=0.1,
        )
        p99 = svc.get_percentile("cpu_temp", 99.0, 0.0, 100.0)
        assert p99 >= 99.0  # should be very close to max

    def test_no_data_returns_zero(self):
        svc = MetricAggregationService()
        assert svc.get_percentile("x", 50.0, 0.0, 10.0) == 0.0


class TestGetRate:
    """Test get_rate() — rate of change computation."""

    def test_linear_increase(self):
        # value goes from 0 to 100 over 10 seconds -> rate = 10 per sec
        svc = _make_service_with_points(
            values=[0.0, 50.0, 100.0],
            start_ts=0.0,
            step=5.0,
        )
        rate = svc.get_rate("cpu_temp", 0.0, 10.0)
        assert rate == pytest.approx(10.0)

    def test_no_change(self):
        svc = _make_service_with_points(
            values=[5.0, 5.0, 5.0],
            start_ts=0.0,
            step=1.0,
        )
        assert svc.get_rate("cpu_temp", 0.0, 3.0) == pytest.approx(0.0)

    def test_single_point_returns_zero(self):
        svc = MetricAggregationService()
        svc.record(MetricPoint("x", 10.0, 1.0))
        assert svc.get_rate("x", 0.0, 5.0) == 0.0


class TestPrune:
    """Test prune() — removal of old data points."""

    def test_prune_removes_old_points(self):
        svc = _make_service_with_points(
            values=[1.0, 2.0, 3.0, 4.0, 5.0],
            start_ts=100.0,
            step=10.0,
        )
        # Points at ts 100, 110, 120, 130, 140
        # current_time=145, max_age=25 -> cutoff=120 -> keep >=120
        removed = svc.prune(max_age_sec=25.0, current_time=145.0)
        assert removed == 2  # ts 100 and 110 removed
        remaining = svc.get_latest("cpu_temp", count=100)
        assert len(remaining) == 3
        assert remaining[0].timestamp == 120.0

    def test_prune_removes_empty_metrics(self):
        svc = MetricAggregationService()
        svc.record(MetricPoint("old", 1.0, 10.0))
        svc.prune(max_age_sec=5.0, current_time=100.0)
        assert "old" not in svc.get_metric_names()

    def test_prune_no_removal(self):
        svc = _make_service_with_points(
            values=[1.0, 2.0],
            start_ts=100.0,
        )
        removed = svc.prune(max_age_sec=1000.0, current_time=105.0)
        assert removed == 0
