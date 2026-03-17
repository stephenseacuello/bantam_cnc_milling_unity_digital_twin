"""Tests for SPC ControlChartGenerator.

Tests X-bar/R charts, EWMA charts, Western Electric rules,
A2/D3/D4 constants, and edge cases.
"""

import sys
import math
from unittest.mock import MagicMock

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

import pytest
from miracle_scada.kpi_calculator import (
    ControlChartGenerator,
    ControlChartData,
    ControlChartType,
    EWMAChartData,
    RuleViolation,
    _A2, _D3, _D4,
)


# ---- X-bar chart control limits ----

def test_xbar_chart_control_limits():
    """Subgroups of size 5 with known data."""
    subgroups = [
        [10.0, 10.2, 9.8, 10.1, 9.9],
        [10.1, 10.3, 9.7, 10.0, 10.2],
        [9.9, 10.0, 10.1, 9.8, 10.0],
        [10.2, 10.0, 10.1, 10.3, 9.9],
        [10.0, 9.9, 10.0, 10.1, 10.0],
    ]
    chart = ControlChartGenerator.compute_xbar_r(subgroups)

    assert len(chart.subgroup_means) == 5
    assert len(chart.subgroup_ranges) == 5
    # CL = grand mean
    expected_cl = sum(chart.subgroup_means) / 5
    assert chart.x_bar_cl == pytest.approx(expected_cl, abs=0.001)
    # UCL > CL > LCL
    assert chart.x_bar_ucl > chart.x_bar_cl > chart.x_bar_lcl
    # A2 for n=5 is 0.577
    r_bar = sum(chart.subgroup_ranges) / 5
    assert chart.x_bar_ucl == pytest.approx(expected_cl + 0.577 * r_bar, abs=0.001)


# ---- R chart control limits ----

def test_r_chart_control_limits():
    subgroups = [
        [10.0, 10.5, 9.5, 10.2, 9.8],
        [10.1, 10.3, 9.7, 10.0, 10.4],
        [9.9, 10.0, 10.1, 9.6, 10.0],
    ]
    chart = ControlChartGenerator.compute_xbar_r(subgroups)
    r_bar = sum(chart.subgroup_ranges) / 3
    # D4 for n=5 is 2.114
    assert chart.r_ucl == pytest.approx(2.114 * r_bar, abs=0.001)
    # D3 for n=5 is 0.000
    assert chart.r_lcl == pytest.approx(0.0, abs=0.001)
    assert chart.r_cl == pytest.approx(r_bar, abs=0.001)


# ---- Out-of-control point detection ----

def test_out_of_control_detection():
    """One subgroup with a wildly different mean."""
    subgroups = [
        [10.0, 10.0, 10.0, 10.0, 10.0],
        [10.0, 10.0, 10.0, 10.0, 10.0],
        [10.0, 10.0, 10.0, 10.0, 10.0],
        [15.0, 15.0, 15.0, 15.0, 15.0],  # way out
        [10.0, 10.0, 10.0, 10.0, 10.0],
    ]
    chart = ControlChartGenerator.compute_xbar_r(subgroups)
    assert 3 in chart.out_of_control_points


# ---- Western Electric Rule 1 (beyond 3σ) ----

def test_western_electric_rule_1():
    subgroups = [
        [10.0, 10.0, 10.0, 10.0, 10.0],
        [10.0, 10.0, 10.0, 10.0, 10.0],
        [10.0, 10.0, 10.0, 10.0, 10.0],
        [20.0, 20.0, 20.0, 20.0, 20.0],  # extreme outlier
        [10.0, 10.0, 10.0, 10.0, 10.0],
    ]
    chart = ControlChartGenerator.compute_xbar_r(subgroups)
    rule1_violations = [v for v in chart.rule_violations if v.rule_number == 1]
    assert len(rule1_violations) >= 1
    violation_indices = {v.point_index for v in rule1_violations}
    assert 3 in violation_indices  # the extreme outlier must be flagged


# ---- Western Electric Rule 2 (9 consecutive same side) ----

def test_western_electric_rule_2():
    """9+ points all above the center line."""
    # Build subgroups where means are consistently above center
    subgroups = []
    for i in range(12):
        if i < 9:
            subgroups.append([10.5, 10.6, 10.4, 10.5, 10.5])  # above
        else:
            subgroups.append([9.5, 9.6, 9.4, 9.5, 9.5])  # below to set CL lower
    chart = ControlChartGenerator.compute_xbar_r(subgroups)
    rule2 = [v for v in chart.rule_violations if v.rule_number == 2]
    # The first 9 points are all above the grand mean
    assert len(rule2) >= 1


# ---- EWMA chart ----

def test_ewma_chart():
    data = [10.0 + 0.1 * i for i in range(20)]  # trending upward
    ewma = ControlChartGenerator.compute_ewma(data, lambda_param=0.2)
    assert len(ewma.ewma_values) == 20
    assert len(ewma.ucl_values) == 20
    assert len(ewma.lcl_values) == 20
    assert ewma.lambda_param == 0.2
    # EWMA should smooth the trend
    assert ewma.ewma_values[-1] > ewma.ewma_values[0]


def test_ewma_with_known_params():
    data = [10.0] * 10  # stable process
    ewma = ControlChartGenerator.compute_ewma(data, mu=10.0, sigma=1.0)
    # All EWMA values should be at or near mu
    for v in ewma.ewma_values:
        assert abs(v - 10.0) < 0.01
    assert len(ewma.out_of_control_points) == 0


# ---- A2/D3/D4 constants ----

def test_constants_valid_sizes():
    for n in range(2, 11):
        c = ControlChartGenerator.get_constants(n)
        assert c is not None
        assert 'A2' in c
        assert 'D3' in c
        assert 'D4' in c
        assert 'd2' in c
        assert c['A2'] > 0
        assert c['D4'] > 0


def test_constants_invalid_size():
    assert ControlChartGenerator.get_constants(1) is None
    assert ControlChartGenerator.get_constants(11) is None


def test_a2_decreases_with_subgroup_size():
    """A2 should decrease as subgroup size increases."""
    for n in range(2, 10):
        assert _A2[n] > _A2[n + 1]


# ---- Edge cases ----

def test_empty_subgroups():
    chart = ControlChartGenerator.compute_xbar_r([])
    assert len(chart.subgroup_means) == 0
    assert chart.x_bar_cl == 0.0


def test_single_subgroup():
    chart = ControlChartGenerator.compute_xbar_r([[10.0, 10.1, 9.9, 10.0, 10.0]])
    assert len(chart.subgroup_means) == 1
    assert chart.x_bar_cl == pytest.approx(10.0, abs=0.01)


def test_ewma_empty_data():
    ewma = ControlChartGenerator.compute_ewma([])
    assert len(ewma.ewma_values) == 0
    assert ewma.lambda_param == 0.2
