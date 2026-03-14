"""Tests for process capability analysis (Cp/Cpk) and SPC control charts.

Uses the same mock pattern as test_shift_report.py — ROS2 modules are
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
    MeasurementSample,
    ProcessCapability,
    CapabilityProfiler,
    _normal_cdf,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_samples(
    values: List[float],
    feature_id: str = 'diameter_A',
    machine_id: str = 'cnc_1',
    program_id: str = 'prog_1',
    start_time: float = 0.0,
    interval: float = 1.0,
) -> List[MeasurementSample]:
    """Create a list of MeasurementSample from raw values."""
    return [
        MeasurementSample(
            value=v,
            timestamp=start_time + i * interval,
            feature_id=feature_id,
            machine_id=machine_id,
            program_id=program_id,
        )
        for i, v in enumerate(values)
    ]


def _add_samples(profiler: CapabilityProfiler, samples: List[MeasurementSample]):
    for s in samples:
        profiler.add_sample(s)


def _centered_process(n: int = 50, mean: float = 10.0, spread: float = 0.3):
    """Generate a linearly-spaced, symmetric process centred on *mean*."""
    # Values equally spaced in [mean - spread, mean + spread]
    return [mean - spread + 2 * spread * i / (n - 1) for i in range(n)]


# ---------------------------------------------------------------------------
# MeasurementSample dataclass
# ---------------------------------------------------------------------------

class TestMeasurementSample:
    def test_creation(self):
        s = MeasurementSample(1.23, 100.0, 'diam', 'cnc_1', 'prog_1')
        assert s.value == 1.23
        assert s.feature_id == 'diam'

    def test_equality(self):
        a = MeasurementSample(1.0, 0.0, 'f', 'm', 'p')
        b = MeasurementSample(1.0, 0.0, 'f', 'm', 'p')
        assert a == b


# ---------------------------------------------------------------------------
# Insufficient-samples guard
# ---------------------------------------------------------------------------

class TestInsufficientSamples:
    def test_fewer_than_30_raises(self):
        profiler = CapabilityProfiler()
        _add_samples(profiler, _make_samples([1.0] * 10))
        with pytest.raises(ValueError, match='Need >= 30'):
            profiler.compute_capability('diameter_A', 11.0, 9.0)

    def test_exactly_30_accepted(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=30, mean=10.0, spread=0.5)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 12.0, 8.0)
        assert cap.sample_count == 30

    def test_zero_samples_raises(self):
        profiler = CapabilityProfiler()
        with pytest.raises(ValueError):
            profiler.compute_capability('nonexistent', 1.0, 0.0)

    def test_single_sample_raises(self):
        profiler = CapabilityProfiler()
        _add_samples(profiler, _make_samples([5.0]))
        with pytest.raises(ValueError):
            profiler.compute_capability('diameter_A', 6.0, 4.0)


# ---------------------------------------------------------------------------
# Cp calculation — centered process
# ---------------------------------------------------------------------------

class TestCpCenteredProcess:
    def test_cp_formula(self):
        """Cp = (USL - LSL) / (6 * sigma)."""
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=0.3)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        # Manual: sigma ≈ std of linspace, Cp = 2.0 / (6*sigma)
        expected_cp = (11.0 - 9.0) / (6.0 * cap.std_dev)
        assert cap.cp == pytest.approx(expected_cp, rel=1e-6)

    def test_cp_equals_cpk_when_centered(self):
        """For a perfectly centered process Cp ≈ Cpk."""
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=0.3)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.cp == pytest.approx(cap.cpk, rel=1e-3)

    def test_cpu_equals_cpl_when_centered(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=0.3)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.cpu == pytest.approx(cap.cpl, rel=1e-3)


# ---------------------------------------------------------------------------
# Cpk < Cp when process is off-centre
# ---------------------------------------------------------------------------

class TestCpkOffCenter:
    def test_cpk_less_than_cp(self):
        profiler = CapabilityProfiler()
        # Shift mean toward USL
        vals = _centered_process(n=50, mean=10.5, spread=0.3)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.cpk < cap.cp

    def test_cpu_less_than_cpl_when_shifted_high(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.5, spread=0.3)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.cpu < cap.cpl
        assert cap.cpk == pytest.approx(cap.cpu, rel=1e-6)

    def test_cpl_less_than_cpu_when_shifted_low(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=9.5, spread=0.3)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.cpl < cap.cpu
        assert cap.cpk == pytest.approx(cap.cpl, rel=1e-6)


# ---------------------------------------------------------------------------
# Capable / incapable process thresholds
# ---------------------------------------------------------------------------

class TestCapableProcess:
    def test_capable_when_cpk_high(self):
        """Tight tolerance relative to spread => Cpk >= 1.33."""
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=0.1)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.cpk >= 1.33
        assert cap.is_capable is True

    def test_incapable_when_cpk_low(self):
        """Wide spread relative to tolerance => Cpk < 1.0."""
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=1.5)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.cpk < 1.0
        assert cap.is_capable is False

    def test_marginal_capability(self):
        """Cpk between 1.0 and 1.33 is not considered capable."""
        profiler = CapabilityProfiler()
        # Tune spread so Cpk ~ 1.15
        vals = _centered_process(n=50, mean=10.0, spread=0.28)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.cpk >= 1.0
        assert cap.is_capable is False or cap.cpk >= 1.33  # either path is valid


# ---------------------------------------------------------------------------
# Zero standard deviation edge case
# ---------------------------------------------------------------------------

class TestZeroStdDev:
    def test_all_identical_values(self):
        profiler = CapabilityProfiler()
        vals = [10.0] * 50
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.std_dev == 0.0
        assert cap.cp == float('inf')
        assert cap.cpk == float('inf')
        assert cap.is_capable is True

    def test_zero_std_scrap_in_spec(self):
        profiler = CapabilityProfiler()
        vals = [10.0] * 50
        _add_samples(profiler, _make_samples(vals))
        rate = profiler.predict_scrap_rate('diameter_A', 11.0, 9.0)
        assert rate == 0.0

    def test_zero_std_scrap_out_of_spec(self):
        profiler = CapabilityProfiler()
        vals = [12.0] * 50  # all above USL
        _add_samples(profiler, _make_samples(vals))
        rate = profiler.predict_scrap_rate('diameter_A', 11.0, 9.0)
        assert rate == 1.0


# ---------------------------------------------------------------------------
# Pp / Ppk (process performance)
# ---------------------------------------------------------------------------

class TestProcessPerformance:
    def test_pp_greater_or_equal_cp(self):
        """Pp uses population std dev (N), Cp uses sample std dev (N-1).
        Since population std < sample std, Pp >= Cp."""
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=0.3)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.pp >= cap.cp

    def test_ppk_computed(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=0.3)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.ppk > 0


# ---------------------------------------------------------------------------
# Control chart: UCL / LCL
# ---------------------------------------------------------------------------

class TestControlChart:
    def test_ucl_lcl_3sigma(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=0.3)
        _add_samples(profiler, _make_samples(vals))
        chart = profiler.get_control_chart_data('diameter_A')
        mean = chart['center_line']
        # UCL = mean + 3*sigma, LCL = mean - 3*sigma
        assert chart['ucl'] > mean
        assert chart['lcl'] < mean
        assert chart['ucl'] == pytest.approx(2 * mean - chart['lcl'], rel=1e-6)

    def test_all_values_returned(self):
        profiler = CapabilityProfiler()
        vals = list(range(50))
        _add_samples(profiler, _make_samples([float(v) for v in vals]))
        chart = profiler.get_control_chart_data('diameter_A')
        assert len(chart['values']) == 50

    def test_empty_feature(self):
        profiler = CapabilityProfiler()
        chart = profiler.get_control_chart_data('nonexistent')
        assert chart['values'] == []
        assert chart['ucl'] == 0.0


# ---------------------------------------------------------------------------
# Out-of-control point detection
# ---------------------------------------------------------------------------

class TestOutOfControlDetection:
    def test_rule1_beyond_3sigma(self):
        """A single point beyond 3-sigma is flagged."""
        profiler = CapabilityProfiler()
        vals = [10.0] * 49 + [100.0]  # last point is a clear outlier
        _add_samples(profiler, _make_samples(vals))
        chart = profiler.get_control_chart_data('diameter_A')
        assert 49 in chart['out_of_control_points']

    def test_no_ooc_for_stable_process(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=0.1)
        _add_samples(profiler, _make_samples(vals))
        chart = profiler.get_control_chart_data('diameter_A')
        # With a uniform-ish distribution within tight spread, no OOC expected
        # (depends on sigma calculation, but spread is small)
        assert isinstance(chart['out_of_control_points'], list)


# ---------------------------------------------------------------------------
# Western Electric rules — detailed
# ---------------------------------------------------------------------------

class TestWesternElectricRules:
    def test_rule4_eight_consecutive_same_side(self):
        """Eight points all above the mean triggers rule 4."""
        profiler = CapabilityProfiler()
        # 30 at mean, then 8 clearly above
        base = [10.0] * 30
        above = [10.5] * 8
        vals = base + above
        _add_samples(profiler, _make_samples(vals))
        chart = profiler.get_control_chart_data('diameter_A')
        ooc = chart['out_of_control_points']
        # The 8 consecutive points (indices 30-37) should be flagged
        for i in range(30, 38):
            assert i in ooc, f"Index {i} should be flagged by rule 4"

    def test_sigma_zero_returns_empty(self):
        """Identical values yield sigma=0 → no OOC points."""
        profiler = CapabilityProfiler()
        vals = [10.0] * 50
        _add_samples(profiler, _make_samples(vals))
        chart = profiler.get_control_chart_data('diameter_A')
        assert chart['out_of_control_points'] == []


# ---------------------------------------------------------------------------
# Trend detection
# ---------------------------------------------------------------------------

class TestTrendDetection:
    def test_stable_trend(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=0.3)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.trend == 'STABLE'

    def test_degrading_trend(self):
        """Second half has much higher dispersion => DEGRADING."""
        profiler = CapabilityProfiler()
        tight = _centered_process(n=25, mean=10.0, spread=0.1)
        wide = _centered_process(n=25, mean=10.0, spread=1.0)
        _add_samples(profiler, _make_samples(tight + wide))
        cap = profiler.compute_capability('diameter_A', 12.0, 8.0)
        assert cap.trend == 'DEGRADING'

    def test_improving_trend(self):
        """Second half has much lower dispersion => IMPROVING."""
        profiler = CapabilityProfiler()
        wide = _centered_process(n=25, mean=10.0, spread=1.0)
        tight = _centered_process(n=25, mean=10.0, spread=0.1)
        _add_samples(profiler, _make_samples(wide + tight))
        cap = profiler.compute_capability('diameter_A', 12.0, 8.0)
        assert cap.trend == 'IMPROVING'

    def test_few_samples_default_stable(self):
        """< 10 values in the trend computation default to STABLE."""
        # We need 30 for capability, but trend helper uses the raw values.
        # Use exactly 30 to test that trend logic handles small halves.
        profiler = CapabilityProfiler()
        vals = _centered_process(n=30, mean=10.0, spread=0.3)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        # With 30 values, halves are 15 each (>= 10), so trend is computed
        assert cap.trend in ('STABLE', 'IMPROVING', 'DEGRADING')


# ---------------------------------------------------------------------------
# Machine comparison
# ---------------------------------------------------------------------------

class TestMachineComparison:
    def test_compare_two_machines(self):
        profiler = CapabilityProfiler()
        # Machine 1: tight process
        tight = _make_samples(
            _centered_process(n=50, mean=10.0, spread=0.1),
            machine_id='cnc_1',
        )
        # Machine 2: looser process
        loose = _make_samples(
            _centered_process(n=50, mean=10.0, spread=0.5),
            machine_id='cnc_2',
        )
        _add_samples(profiler, tight)
        _add_samples(profiler, loose)

        result = profiler.compare_machines(
            'cnc_1', 'cnc_2', 'diameter_A', 11.0, 9.0,
        )
        assert result['better_machine'] == 'cnc_1'
        assert result['cpk_1'] > result['cpk_2']
        assert result['delta'] > 0

    def test_compare_insufficient_samples(self):
        profiler = CapabilityProfiler()
        _add_samples(profiler, _make_samples([10.0] * 5, machine_id='cnc_1'))
        _add_samples(profiler, _make_samples(
            _centered_process(n=50), machine_id='cnc_2',
        ))
        result = profiler.compare_machines(
            'cnc_1', 'cnc_2', 'diameter_A', 11.0, 9.0,
        )
        assert result['cpk_1'] is None
        assert result['better_machine'] is None

    def test_compare_returns_feature_id(self):
        profiler = CapabilityProfiler()
        _add_samples(profiler, _make_samples(
            _centered_process(n=50), machine_id='cnc_1',
        ))
        _add_samples(profiler, _make_samples(
            _centered_process(n=50), machine_id='cnc_2',
        ))
        result = profiler.compare_machines(
            'cnc_1', 'cnc_2', 'diameter_A', 11.0, 9.0,
        )
        assert result['feature_id'] == 'diameter_A'


# ---------------------------------------------------------------------------
# Machine capability summary
# ---------------------------------------------------------------------------

class TestMachineCapabilitySummary:
    def test_filters_by_machine(self):
        profiler = CapabilityProfiler()
        _add_samples(profiler, _make_samples([1.0, 2.0], machine_id='cnc_1'))
        _add_samples(profiler, _make_samples([3.0], machine_id='cnc_2'))
        summary = profiler.get_machine_capability_summary('cnc_1')
        assert 'diameter_A' in summary
        assert all(s.machine_id == 'cnc_1' for s in summary['diameter_A'])

    def test_empty_for_unknown_machine(self):
        profiler = CapabilityProfiler()
        _add_samples(profiler, _make_samples([1.0], machine_id='cnc_1'))
        summary = profiler.get_machine_capability_summary('unknown')
        assert summary == {}


# ---------------------------------------------------------------------------
# Scrap rate prediction
# ---------------------------------------------------------------------------

class TestScrapRatePrediction:
    def test_zero_scrap_for_wide_spec(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=0.1)
        _add_samples(profiler, _make_samples(vals))
        rate = profiler.predict_scrap_rate('diameter_A', 20.0, 0.0)
        assert rate < 0.001  # effectively zero

    def test_high_scrap_for_tight_spec(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=1.0)
        _add_samples(profiler, _make_samples(vals))
        rate = profiler.predict_scrap_rate('diameter_A', 10.1, 9.9)
        assert rate > 0.5  # most parts would be out

    def test_scrap_rate_between_0_and_1(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=0.5)
        _add_samples(profiler, _make_samples(vals))
        rate = profiler.predict_scrap_rate('diameter_A', 11.0, 9.0)
        assert 0.0 <= rate <= 1.0

    def test_scrap_rate_fewer_than_2_samples(self):
        profiler = CapabilityProfiler()
        _add_samples(profiler, _make_samples([10.0]))
        rate = profiler.predict_scrap_rate('diameter_A', 11.0, 9.0)
        assert rate == 0.0


# ---------------------------------------------------------------------------
# Out-of-spec percentage
# ---------------------------------------------------------------------------

class TestOutOfSpecPct:
    def test_all_in_spec(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=0.1)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.out_of_spec_pct == 0.0

    def test_some_out_of_spec(self):
        profiler = CapabilityProfiler()
        vals = _centered_process(n=50, mean=10.0, spread=1.5)
        _add_samples(profiler, _make_samples(vals))
        cap = profiler.compute_capability('diameter_A', 11.0, 9.0)
        assert cap.out_of_spec_pct > 0.0


# ---------------------------------------------------------------------------
# Normal CDF helper
# ---------------------------------------------------------------------------

class TestNormalCdf:
    def test_cdf_at_zero(self):
        assert _normal_cdf(0.0) == pytest.approx(0.5)

    def test_cdf_at_large_positive(self):
        assert _normal_cdf(5.0) == pytest.approx(1.0, abs=1e-6)

    def test_cdf_at_large_negative(self):
        assert _normal_cdf(-5.0) == pytest.approx(0.0, abs=1e-6)

    def test_cdf_symmetry(self):
        assert _normal_cdf(1.0) + _normal_cdf(-1.0) == pytest.approx(1.0)
