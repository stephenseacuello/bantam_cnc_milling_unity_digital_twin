"""Tests for the HypothesisTestEngine statistical hypothesis testing engine."""

import math
import pytest
import sys
import os
from unittest.mock import MagicMock

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

import miracle_cognitive.interface.explanation_generator as _eg_module

HypothesisTestEngine = _eg_module.HypothesisTestEngine
HypothesisTest = _eg_module.HypothesisTest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_engine() -> HypothesisTestEngine:
    return HypothesisTestEngine()


def _generate_normal_like(mean: float, std: float, n: int, seed: int = 42) -> list:
    """Generate pseudo-normal samples using the Box-Muller transform.

    Uses a simple LCG for reproducibility without requiring ``random``.
    """
    # Linear congruential generator (Numerical Recipes parameters)
    state = seed
    def _lcg() -> float:
        nonlocal state
        state = (state * 1664525 + 1013904223) & 0xFFFFFFFF
        return state / 0xFFFFFFFF

    samples = []
    for _ in range(n):
        u1 = max(_lcg(), 1e-10)
        u2 = _lcg()
        z = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
        samples.append(mean + std * z)
    return samples


# ===========================================================================
# Dataclass tests
# ===========================================================================

class TestHypothesisTestDataclass:
    def test_fields(self):
        ht = HypothesisTest(
            test_name='t', null_hypothesis='H0', alternative_hypothesis='H1',
            test_statistic=1.5, p_value=0.04, reject_null=True,
            confidence_level=0.95, sample_size=30, conclusion='rejected',
        )
        assert ht.test_name == 't'
        assert ht.p_value == 0.04
        assert ht.reject_null is True
        assert ht.sample_size == 30


# ===========================================================================
# One-sample t-test
# ===========================================================================

class TestOneSampleTTest:
    def test_mean_at_target(self):
        """Data drawn around the target mean should NOT reject H0."""
        engine = _make_engine()
        data = _generate_normal_like(mean=10.0, std=1.0, n=50)
        result = engine.t_test_one_sample(data, mu0=10.0, alpha=0.05)

        assert result.test_name == 'one_sample_t_test'
        assert result.reject_null is False
        assert result.p_value > 0.05
        assert result.sample_size == 50
        assert result.confidence_level == pytest.approx(0.95)
        assert 'Fail to reject' in result.conclusion

    def test_mean_shifted(self):
        """Data with mean clearly shifted from target should reject H0."""
        engine = _make_engine()
        # Mean = 15, target = 10 -- big shift relative to std=1
        data = _generate_normal_like(mean=15.0, std=1.0, n=50)
        result = engine.t_test_one_sample(data, mu0=10.0, alpha=0.05)

        assert result.reject_null is True
        assert result.p_value < 0.05
        assert 'Reject H0' in result.conclusion

    def test_insufficient_data(self):
        engine = _make_engine()
        result = engine.t_test_one_sample([5.0], mu0=5.0)
        assert result.reject_null is False
        assert 'Insufficient' in result.conclusion


# ===========================================================================
# Two-sample t-test (Welch's)
# ===========================================================================

class TestTwoSampleTTest:
    def test_same_means(self):
        """Two samples from the same distribution should NOT reject H0."""
        engine = _make_engine()
        data1 = _generate_normal_like(mean=50.0, std=5.0, n=40, seed=1)
        data2 = _generate_normal_like(mean=50.0, std=5.0, n=40, seed=2)
        result = engine.t_test_two_sample(data1, data2, alpha=0.05)

        assert result.test_name == 'two_sample_t_test_welch'
        assert result.reject_null is False
        assert result.p_value > 0.05
        assert 'Fail to reject' in result.conclusion

    def test_different_means(self):
        """Two samples with clearly different means should reject H0."""
        engine = _make_engine()
        data1 = _generate_normal_like(mean=50.0, std=2.0, n=40, seed=10)
        data2 = _generate_normal_like(mean=60.0, std=2.0, n=40, seed=20)
        result = engine.t_test_two_sample(data1, data2, alpha=0.05)

        assert result.reject_null is True
        assert result.p_value < 0.05
        assert result.sample_size == 80
        assert 'Reject H0' in result.conclusion

    def test_insufficient_data(self):
        engine = _make_engine()
        result = engine.t_test_two_sample([1.0], [2.0, 3.0])
        assert result.reject_null is False
        assert 'Insufficient' in result.conclusion


# ===========================================================================
# F-test for equality of variances
# ===========================================================================

class TestFTestVariance:
    def test_equal_variances(self):
        """Two samples with equal variances should NOT reject H0."""
        engine = _make_engine()
        data1 = _generate_normal_like(mean=100.0, std=5.0, n=50, seed=100)
        data2 = _generate_normal_like(mean=100.0, std=5.0, n=50, seed=200)
        result = engine.f_test_variance(data1, data2, alpha=0.05)

        assert result.test_name == 'f_test_variance'
        assert result.reject_null is False
        assert result.p_value > 0.05
        assert 'Fail to reject' in result.conclusion

    def test_different_variances(self):
        """Two samples with very different variances should reject H0."""
        engine = _make_engine()
        data1 = _generate_normal_like(mean=100.0, std=1.0, n=50, seed=300)
        data2 = _generate_normal_like(mean=100.0, std=10.0, n=50, seed=400)
        result = engine.f_test_variance(data1, data2, alpha=0.05)

        assert result.reject_null is True
        assert result.p_value < 0.05
        assert 'Reject H0' in result.conclusion

    def test_f_statistic_positive(self):
        engine = _make_engine()
        data1 = _generate_normal_like(mean=0.0, std=3.0, n=30, seed=500)
        data2 = _generate_normal_like(mean=0.0, std=5.0, n=30, seed=600)
        result = engine.f_test_variance(data1, data2)
        assert result.test_statistic >= 1.0  # larger variance in numerator


# ===========================================================================
# Chi-squared goodness of fit
# ===========================================================================

class TestChiSquaredGoodnessOfFit:
    def test_matching_distribution(self):
        """Observed counts matching expected should NOT reject H0."""
        engine = _make_engine()
        observed = [50.0, 50.0, 50.0, 50.0]
        expected = [50.0, 50.0, 50.0, 50.0]
        result = engine.chi_squared_goodness_of_fit(observed, expected, alpha=0.05)

        assert result.test_name == 'chi_squared_goodness_of_fit'
        assert result.reject_null is False
        assert result.test_statistic == pytest.approx(0.0, abs=1e-10)
        assert result.p_value > 0.99
        assert 'Fail to reject' in result.conclusion

    def test_mismatched_distribution(self):
        """Observed counts far from expected should reject H0."""
        engine = _make_engine()
        observed = [90.0, 10.0, 60.0, 40.0]
        expected = [50.0, 50.0, 50.0, 50.0]
        result = engine.chi_squared_goodness_of_fit(observed, expected, alpha=0.05)

        assert result.reject_null is True
        assert result.p_value < 0.05
        assert 'Reject H0' in result.conclusion

    def test_mismatched_lengths(self):
        engine = _make_engine()
        result = engine.chi_squared_goodness_of_fit([10, 20], [10, 20, 30])
        assert result.reject_null is False
        assert 'Invalid input' in result.conclusion


# ===========================================================================
# Process shift detection
# ===========================================================================

class TestProcessShiftDetection:
    def test_no_shift(self):
        """Data consistent with baseline should NOT detect a shift."""
        engine = _make_engine()
        data = _generate_normal_like(mean=25.0, std=0.5, n=50, seed=700)
        result = engine.process_shift_detection(
            data, baseline_mean=25.0, baseline_std=0.5, alpha=0.05,
        )

        assert result.test_name == 'process_shift_detection'
        assert result.reject_null is False
        assert 'No significant process shift' in result.conclusion

    def test_significant_shift(self):
        """Data with mean far from baseline should detect a shift."""
        engine = _make_engine()
        data = _generate_normal_like(mean=28.0, std=0.5, n=50, seed=800)
        result = engine.process_shift_detection(
            data, baseline_mean=25.0, baseline_std=0.5, alpha=0.05,
        )

        assert result.reject_null is True
        assert 'Process shift detected' in result.conclusion
        assert 'sigma' in result.conclusion

    def test_shift_direction_above(self):
        engine = _make_engine()
        data = _generate_normal_like(mean=30.0, std=1.0, n=30, seed=900)
        result = engine.process_shift_detection(
            data, baseline_mean=25.0, baseline_std=1.0,
        )
        assert result.reject_null is True
        assert 'above' in result.conclusion

    def test_shift_direction_below(self):
        engine = _make_engine()
        data = _generate_normal_like(mean=20.0, std=1.0, n=30, seed=1000)
        result = engine.process_shift_detection(
            data, baseline_mean=25.0, baseline_std=1.0,
        )
        assert result.reject_null is True
        assert 'below' in result.conclusion


# ===========================================================================
# run_test dispatcher
# ===========================================================================

class TestRunTestDispatcher:
    def test_dispatch_one_sample_t(self):
        engine = _make_engine()
        data = _generate_normal_like(mean=10.0, std=1.0, n=30)
        result = engine.run_test('one_sample_t', data=data, mu0=10.0)
        assert result.test_name == 'one_sample_t_test'

    def test_dispatch_two_sample_t(self):
        engine = _make_engine()
        d1 = _generate_normal_like(mean=5.0, std=1.0, n=20, seed=1)
        d2 = _generate_normal_like(mean=5.0, std=1.0, n=20, seed=2)
        result = engine.run_test('two_sample_t', data1=d1, data2=d2)
        assert result.test_name == 'two_sample_t_test_welch'

    def test_dispatch_f_test(self):
        engine = _make_engine()
        d1 = _generate_normal_like(mean=0.0, std=1.0, n=20, seed=3)
        d2 = _generate_normal_like(mean=0.0, std=1.0, n=20, seed=4)
        result = engine.run_test('f_test', data1=d1, data2=d2)
        assert result.test_name == 'f_test_variance'

    def test_dispatch_chi_squared(self):
        engine = _make_engine()
        result = engine.run_test(
            'chi_squared', observed=[25, 25, 25, 25], expected=[25, 25, 25, 25],
        )
        assert result.test_name == 'chi_squared_goodness_of_fit'

    def test_dispatch_process_shift(self):
        engine = _make_engine()
        data = _generate_normal_like(mean=10.0, std=1.0, n=20)
        result = engine.run_test(
            'process_shift', data=data, baseline_mean=10.0, baseline_std=1.0,
        )
        assert result.test_name == 'process_shift_detection'

    def test_dispatch_unknown_raises(self):
        engine = _make_engine()
        with pytest.raises(ValueError, match='Unknown test_type'):
            engine.run_test('nonexistent_test')


# ===========================================================================
# CDF approximation sanity checks
# ===========================================================================

class TestCDFApproximations:
    def test_t_cdf_symmetry(self):
        """t-distribution CDF should be symmetric: CDF(-t) + CDF(t) ~= 1."""
        for df in [1, 5, 30]:
            for t_val in [0.5, 1.0, 2.0]:
                lo = HypothesisTestEngine._t_cdf(-t_val, df)
                hi = HypothesisTestEngine._t_cdf(t_val, df)
                assert lo + hi == pytest.approx(1.0, abs=1e-6)

    def test_t_cdf_at_zero(self):
        """CDF at t=0 should be 0.5."""
        assert HypothesisTestEngine._t_cdf(0.0, 10) == pytest.approx(0.5, abs=1e-6)

    def test_chi2_cdf_at_zero(self):
        assert HypothesisTestEngine._chi2_cdf(0.0, 5) == pytest.approx(0.0, abs=1e-10)

    def test_chi2_cdf_large_x(self):
        """For large x the CDF should approach 1."""
        assert HypothesisTestEngine._chi2_cdf(100.0, 3) > 0.999
