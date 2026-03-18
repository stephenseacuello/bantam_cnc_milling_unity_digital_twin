"""Tests for the ConfidenceIntervalCalculator in explanation_generator.py."""

import math
import os
import sys
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Mock ROS2 dependencies before import.
# ---------------------------------------------------------------------------

for mod in ['miracle_core.datatypes', 'miracle_core.constants',
            'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

# Ensure the base class has required class attributes
_base = sys.modules['miracle_core.lifecycle_node_base']
if not hasattr(_base.MiracleLifecycleNode, 'CRITICALITY_LOW') or \
   isinstance(getattr(_base.MiracleLifecycleNode, 'CRITICALITY_LOW', None), MagicMock):
    _base.MiracleLifecycleNode.CRITICALITY_LOW = 'LOW'
    _base.MiracleLifecycleNode.CRITICALITY_HIGH = 'HIGH'
    _base.MiracleLifecycleNode.CRITICALITY_CRITICAL = 'CRITICAL'

# Force reimport so the module picks up our mocks
sys.modules.pop('miracle_cognitive.interface.explanation_generator', None)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_cognitive.interface.explanation_generator import (
    ConfidenceIntervalCalculator,
    IntervalResult,
    PredictionInterval,
)


@pytest.fixture
def calc():
    """Return a fresh ConfidenceIntervalCalculator instance."""
    return ConfidenceIntervalCalculator()


# ---------------------------------------------------------------------------
# 1. mean_interval — small sample (t-distribution)
# ---------------------------------------------------------------------------

class TestMeanInterval:
    def test_small_sample_t_distribution(self, calc):
        """Small sample (n < 30) should use t-distribution."""
        data = [10.0, 10.2, 9.8, 10.1, 9.9]
        result = calc.mean_interval(data, confidence=0.95)

        assert isinstance(result, IntervalResult)
        assert result.method == 't_distribution'
        assert result.n == 5
        assert result.confidence_level == 0.95
        assert result.lower < result.mean < result.upper
        assert result.margin_of_error > 0
        # The interval should contain the known population mean (10.0)
        assert result.lower < 10.0 < result.upper

    def test_large_sample_z_normal(self, calc):
        """Large sample (n >= 30) should use z-normal distribution."""
        # Generate 50 observations around 100.0
        data = [100.0 + (i % 5 - 2) * 0.1 for i in range(50)]
        result = calc.mean_interval(data, confidence=0.95)

        assert result.method == 'z_normal'
        assert result.n == 50
        assert result.lower < result.mean < result.upper

    def test_single_observation(self, calc):
        """Single observation should return degenerate interval."""
        result = calc.mean_interval([42.0], confidence=0.95)
        assert result.method == 'insufficient_data'
        assert result.lower == result.upper == 42.0
        assert result.margin_of_error == 0.0


# ---------------------------------------------------------------------------
# 2. proportion_interval — Wilson score
# ---------------------------------------------------------------------------

class TestProportionInterval:
    def test_wilson_score_basic(self, calc):
        """Wilson score interval for a moderate proportion."""
        result = calc.proportion_interval(successes=80, total=100, confidence=0.95)

        assert result.method == 'wilson_score'
        assert result.n == 100
        # Point estimate should be 0.80
        assert abs(result.mean - 0.80) < 1e-9
        # The interval should be reasonable
        assert 0.70 < result.lower < 0.80
        assert 0.80 < result.upper < 0.90

    def test_proportion_zero_total(self, calc):
        """Zero total should give degenerate result."""
        result = calc.proportion_interval(successes=0, total=0, confidence=0.95)
        assert result.mean == 0.0
        assert result.lower == 0.0
        assert result.upper == 0.0

    def test_proportion_extreme_values(self, calc):
        """Interval bounds should be clamped to [0, 1]."""
        result = calc.proportion_interval(successes=1, total=1, confidence=0.95)
        assert result.lower >= 0.0
        assert result.upper <= 1.0


# ---------------------------------------------------------------------------
# 3. variance_interval — chi-squared
# ---------------------------------------------------------------------------

class TestVarianceInterval:
    def test_variance_interval_basic(self, calc):
        """Variance interval should bracket the sample variance."""
        data = [10.0, 10.5, 9.5, 10.2, 9.8, 10.1, 9.9, 10.3]
        result = calc.variance_interval(data, confidence=0.95)

        assert result.method == 'chi_squared'
        assert result.n == 8
        # Lower bound < sample variance < upper bound
        assert result.lower < result.mean < result.upper
        assert result.margin_of_error > 0

    def test_variance_interval_insufficient_data(self, calc):
        """Single observation should return degenerate result."""
        result = calc.variance_interval([5.0], confidence=0.95)
        assert result.lower == 0.0
        assert result.upper == 0.0


# ---------------------------------------------------------------------------
# 4. prediction_interval
# ---------------------------------------------------------------------------

class TestPredictionInterval:
    def test_prediction_interval_basic(self, calc):
        """Prediction interval should be wider than a mean CI."""
        result = calc.prediction_interval(
            predicted=50.0, model_std_error=2.0, n=20, confidence=0.95,
        )

        assert isinstance(result, PredictionInterval)
        assert result.predicted_value == 50.0
        assert result.model_std_error == 2.0
        assert result.confidence_level == 0.95
        assert result.lower < 50.0 < result.upper
        # Width should be substantial for std_error=2.0
        assert (result.upper - result.lower) > 4.0

    def test_prediction_interval_large_n(self, calc):
        """Large n should use z-based interval (narrower)."""
        result_small = calc.prediction_interval(
            predicted=50.0, model_std_error=2.0, n=10, confidence=0.95,
        )
        result_large = calc.prediction_interval(
            predicted=50.0, model_std_error=2.0, n=100, confidence=0.95,
        )
        # Larger n -> slightly narrower interval
        width_small = result_small.upper - result_small.lower
        width_large = result_large.upper - result_large.lower
        assert width_large < width_small


# ---------------------------------------------------------------------------
# 5. tolerance_interval
# ---------------------------------------------------------------------------

class TestToleranceInterval:
    def test_tolerance_interval_basic(self, calc):
        """Tolerance interval should be wider than a mean CI."""
        data = [10.0 + (i % 7 - 3) * 0.2 for i in range(20)]
        mean_ci = calc.mean_interval(data, confidence=0.95)
        tol = calc.tolerance_interval(data, confidence=0.95, coverage=0.99)

        assert tol.method == 'tolerance_howe'
        # Tolerance interval should be wider
        tol_width = tol.upper - tol.lower
        ci_width = mean_ci.upper - mean_ci.lower
        assert tol_width > ci_width

    def test_tolerance_interval_single_obs(self, calc):
        """Single observation should give degenerate tolerance interval."""
        result = calc.tolerance_interval([7.0], confidence=0.95, coverage=0.99)
        assert result.lower == result.upper == 7.0


# ---------------------------------------------------------------------------
# 6. sample_size_needed
# ---------------------------------------------------------------------------

class TestSampleSizeNeeded:
    def test_basic_sample_size(self, calc):
        """Sample size for margin=0.5, std=1.0, 95% confidence ≈ 16."""
        n = calc.sample_size_needed(
            margin_of_error=0.5, confidence=0.95, estimated_std=1.0,
        )
        assert isinstance(n, int)
        # z_0.975 ≈ 1.96 -> n = ceil((1.96*1.0/0.5)^2) = ceil(15.37) = 16
        assert n == 16 or abs(n - 16) <= 1  # allow ±1 for approximation

    def test_tighter_margin_needs_more(self, calc):
        """Tighter margin of error requires more observations."""
        n_wide = calc.sample_size_needed(margin_of_error=1.0, confidence=0.95)
        n_tight = calc.sample_size_needed(margin_of_error=0.1, confidence=0.95)
        assert n_tight > n_wide

    def test_zero_margin(self, calc):
        """Zero or negative margin should return minimum sample size."""
        n = calc.sample_size_needed(margin_of_error=0.0)
        assert n == 2

    def test_higher_confidence_needs_more(self, calc):
        """Higher confidence level requires larger sample size."""
        n_95 = calc.sample_size_needed(
            margin_of_error=0.5, confidence=0.95, estimated_std=1.0,
        )
        n_99 = calc.sample_size_needed(
            margin_of_error=0.5, confidence=0.99, estimated_std=1.0,
        )
        assert n_99 > n_95


# ---------------------------------------------------------------------------
# 7. Internal PPF / quantile correctness
# ---------------------------------------------------------------------------

class TestInternalQuantiles:
    def test_z_ppf_symmetry(self, calc):
        """z_ppf should satisfy symmetry: z(p) = -z(1-p)."""
        z_975 = calc._z_ppf(0.975)
        z_025 = calc._z_ppf(0.025)
        assert abs(z_975 + z_025) < 0.01  # approx symmetric

    def test_z_ppf_known_value(self, calc):
        """z_ppf(0.975) should be approximately 1.96."""
        z = calc._z_ppf(0.975)
        assert abs(z - 1.96) < 0.02

    def test_t_ppf_approaches_z(self, calc):
        """t-quantile with large df should approach z-quantile."""
        t_val = calc._t_ppf(0.975, df=10000)
        z_val = calc._z_ppf(0.975)
        assert abs(t_val - z_val) < 0.05


# ---------------------------------------------------------------------------
# 8. Integration: end-to-end manufacturing scenario
# ---------------------------------------------------------------------------

class TestManufacturingScenario:
    def test_bearing_diameter_analysis(self, calc):
        """Simulate a bearing diameter quality analysis."""
        # Simulated bearing diameter measurements (nominal 25.000 mm)
        measurements = [
            25.002, 24.998, 25.001, 24.999, 25.003,
            24.997, 25.000, 25.002, 24.998, 25.001,
        ]

        # 1. Mean CI
        mean_ci = calc.mean_interval(measurements, confidence=0.95)
        assert mean_ci.method == 't_distribution'
        assert 24.99 < mean_ci.lower
        assert mean_ci.upper < 25.01

        # 2. Variance CI
        var_ci = calc.variance_interval(measurements, confidence=0.95)
        assert var_ci.lower > 0
        assert var_ci.upper > var_ci.lower

        # 3. Tolerance interval (99% of population within bounds, 95% conf)
        tol = calc.tolerance_interval(measurements, confidence=0.95, coverage=0.99)
        assert tol.lower < mean_ci.lower
        assert tol.upper > mean_ci.upper

        # 4. Sample size for tighter precision
        n_needed = calc.sample_size_needed(
            margin_of_error=0.0005,
            confidence=0.95,
            estimated_std=calc._std(measurements),
        )
        assert n_needed > len(measurements)

        # 5. Prediction interval for next part
        pred = calc.prediction_interval(
            predicted=mean_ci.mean,
            model_std_error=mean_ci.std,
            n=len(measurements),
            confidence=0.95,
        )
        assert pred.lower < pred.predicted_value < pred.upper
