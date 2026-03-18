"""Tests for BayesianParameterEstimator."""

import sys
import math
import time
import pytest
from unittest.mock import MagicMock

# Mock ROS2 / MIRACLE dependencies before importing the target module.
for mod in [
    'miracle_core.datatypes', 'miracle_core.constants',
    'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
    'rclpy.callback_groups',
    'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
    'miracle_msgs', 'miracle_msgs.msg',
]:
    sys.modules.setdefault(mod, MagicMock())

# Ensure the base class has the CRITICALITY attributes expected at class-body
# evaluation time.
_base = sys.modules['miracle_core.lifecycle_node_base']
if not hasattr(_base.MiracleLifecycleNode, 'CRITICALITY_LOW') or \
   isinstance(getattr(_base.MiracleLifecycleNode, 'CRITICALITY_LOW', None), MagicMock):
    _base.MiracleLifecycleNode.CRITICALITY_LOW = 'LOW'
    _base.MiracleLifecycleNode.CRITICALITY_HIGH = 'HIGH'
    _base.MiracleLifecycleNode.CRITICALITY_CRITICAL = 'CRITICAL'

sys.modules.pop('miracle_cognitive.interface.explanation_generator', None)

import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_cognitive.interface.explanation_generator import (
    PriorDistribution,
    PosteriorEstimate,
    EstimationReport,
    BayesianParameterEstimator,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_estimator(**kwargs):
    return BayesianParameterEstimator(**kwargs)


def _feed_rate_prior(mean=500.0, std=50.0):
    return PriorDistribution(
        parameter_name='feed_rate',
        mean=mean,
        std=std,
        distribution_type='normal',
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestPriorDistribution:
    """Validate PriorDistribution dataclass constraints."""

    def test_valid_normal(self):
        p = PriorDistribution('feed_rate', mean=500.0, std=50.0)
        assert p.distribution_type == 'normal'
        assert p.lower_bound is None

    def test_valid_uniform(self):
        p = PriorDistribution(
            'depth_of_cut', mean=2.0, std=0.5,
            distribution_type='uniform', lower_bound=1.0, upper_bound=3.0,
        )
        assert p.distribution_type == 'uniform'

    def test_invalid_distribution_type(self):
        with pytest.raises(ValueError, match="distribution_type"):
            PriorDistribution('x', mean=0.0, std=1.0, distribution_type='beta')

    def test_non_positive_std_raises(self):
        with pytest.raises(ValueError, match="std must be positive"):
            PriorDistribution('x', mean=0.0, std=0.0)


class TestBayesianSetPriorAndReset:
    """set_prior / reset lifecycle."""

    def test_set_prior_stores(self):
        est = _make_estimator()
        prior = _feed_rate_prior()
        est.set_prior(prior)
        result = est.get_estimate('feed_rate')
        assert result.mean == pytest.approx(500.0)
        assert result.std == pytest.approx(50.0)
        assert result.n_observations == 0

    def test_reset_clears_observations(self):
        est = _make_estimator()
        est.set_prior(_feed_rate_prior())
        est.update('feed_rate', 520.0)
        est.update('feed_rate', 530.0)
        assert est.get_estimate('feed_rate').n_observations == 2
        est.reset('feed_rate')
        result = est.get_estimate('feed_rate')
        assert result.n_observations == 0
        assert result.mean == pytest.approx(500.0)

    def test_reset_unknown_raises(self):
        est = _make_estimator()
        with pytest.raises(KeyError):
            est.reset('nonexistent')


class TestBayesianUpdate:
    """Single and batch observation updates."""

    def test_update_unknown_parameter_raises(self):
        est = _make_estimator()
        with pytest.raises(KeyError):
            est.update('spindle_speed', 3000.0)

    def test_single_update_shifts_mean(self):
        est = _make_estimator()
        est.set_prior(_feed_rate_prior(mean=500.0, std=50.0))
        est.update('feed_rate', 600.0)
        result = est.get_estimate('feed_rate')
        # Posterior mean should be between prior (500) and observation (600)
        assert 500.0 < result.mean < 600.0
        assert result.n_observations == 1

    def test_batch_update_equivalent_to_sequential(self):
        """Batch update should give the same result as sequential updates."""
        est_seq = _make_estimator()
        est_seq.set_prior(_feed_rate_prior())
        observations = [510.0, 520.0, 530.0, 515.0]
        for obs in observations:
            est_seq.update('feed_rate', obs)

        est_batch = _make_estimator()
        est_batch.set_prior(_feed_rate_prior())
        est_batch.update_batch('feed_rate', observations)

        seq_est = est_seq.get_estimate('feed_rate')
        batch_est = est_batch.get_estimate('feed_rate')
        assert seq_est.mean == pytest.approx(batch_est.mean, abs=1e-9)
        assert seq_est.std == pytest.approx(batch_est.std, abs=1e-9)

    def test_batch_update_unknown_raises(self):
        est = _make_estimator()
        with pytest.raises(KeyError):
            est.update_batch('missing', [1.0, 2.0])


class TestConjugateUpdate:
    """Verify Normal-Normal conjugate update math."""

    def test_posterior_precision_increases(self):
        """More observations should decrease posterior std."""
        est = _make_estimator()
        est.set_prior(_feed_rate_prior(mean=500.0, std=50.0))
        std_0 = est.get_estimate('feed_rate').std
        est.update('feed_rate', 510.0)
        std_1 = est.get_estimate('feed_rate').std
        est.update('feed_rate', 510.0)
        std_2 = est.get_estimate('feed_rate').std
        assert std_0 > std_1 > std_2

    def test_many_observations_dominate_prior(self):
        """With many consistent observations, posterior should approach sample mean."""
        est = _make_estimator()
        est.set_prior(_feed_rate_prior(mean=500.0, std=50.0))
        sample_mean = 520.0
        est.update_batch('feed_rate', [sample_mean] * 1000)
        result = est.get_estimate('feed_rate')
        assert result.mean == pytest.approx(sample_mean, abs=0.5)

    def test_credible_interval_contains_mean(self):
        est = _make_estimator()
        est.set_prior(_feed_rate_prior())
        est.update_batch('feed_rate', [505.0, 510.0, 495.0])
        result = est.get_estimate('feed_rate')
        ci_low, ci_high = result.credible_interval_95
        assert ci_low < result.mean < ci_high

    def test_explicit_observation_noise(self):
        """Setting observation_noise_std should affect the update."""
        est_default = _make_estimator()
        est_default.set_prior(_feed_rate_prior(mean=500.0, std=50.0))
        est_default.update('feed_rate', 600.0)

        est_low_noise = _make_estimator(observation_noise_std=10.0)
        est_low_noise.set_prior(_feed_rate_prior(mean=500.0, std=50.0))
        est_low_noise.update('feed_rate', 600.0)

        # Lower observation noise => posterior trusts data more => closer to 600
        assert est_low_noise.get_estimate('feed_rate').mean > \
               est_default.get_estimate('feed_rate').mean


class TestGetEstimate:
    """get_estimate edge cases."""

    def test_no_prior_raises(self):
        est = _make_estimator()
        with pytest.raises(KeyError):
            est.get_estimate('unknown')

    def test_returns_posterior_estimate_type(self):
        est = _make_estimator()
        est.set_prior(_feed_rate_prior())
        result = est.get_estimate('feed_rate')
        assert isinstance(result, PosteriorEstimate)


class TestGetReport:
    """get_report across multiple parameters."""

    def test_empty_report(self):
        est = _make_estimator()
        report = est.get_report()
        assert isinstance(report, EstimationReport)
        assert report.total_observations == 0
        assert report.convergence_score == 0.0
        assert len(report.parameters) == 0

    def test_multi_parameter_report(self):
        est = _make_estimator()
        est.set_prior(PriorDistribution('feed_rate', 500.0, 50.0))
        est.set_prior(PriorDistribution('spindle_speed', 3000.0, 200.0))

        est.update_batch('feed_rate', [510.0, 520.0])
        est.update('spindle_speed', 3100.0)

        report = est.get_report()
        assert len(report.parameters) == 2
        assert report.total_observations == 3
        assert 'feed_rate' in report.parameters
        assert 'spindle_speed' in report.parameters
        assert 0.0 <= report.convergence_score <= 1.0
        assert report.timestamp > 0

    def test_convergence_increases_with_observations(self):
        est = _make_estimator()
        est.set_prior(_feed_rate_prior())
        report_0 = est.get_report()

        est.update_batch('feed_rate', [510.0] * 50)
        report_50 = est.get_report()

        assert report_50.convergence_score > report_0.convergence_score
