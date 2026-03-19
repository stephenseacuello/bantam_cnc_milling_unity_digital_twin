"""Tests for RegressionModelManager — multi-variable OLS regression."""

import sys
import math
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
    RegressionModel,
    PredictionResult,
    RegressionModelManager,
)


# ------------------------------------------------------------------ #
#  Helper: simple dataset (y = 2*x1 + 3*x2 + 5)                     #
# ------------------------------------------------------------------ #

def _make_linear_dataset(n: int = 50):
    """Generate a noise-free dataset: y = 2*x1 + 3*x2 + 5.

    x1 and x2 are linearly independent to avoid singular XtX.
    """
    x1 = [float(i) for i in range(n)]
    x2 = [float(i * i) * 0.01 + 1.0 for i in range(n)]  # quadratic, independent of x1
    y = [2.0 * x1[i] + 3.0 * x2[i] + 5.0 for i in range(n)]
    return {'x1': x1, 'x2': x2}, y


def _make_noisy_dataset(n: int = 80, seed: int = 42):
    """Generate a mildly noisy dataset: y ~ 1.5*speed + 0.8*feed + 10."""
    # Deterministic pseudo-random via LCG
    state = seed
    def _rand():
        nonlocal state
        state = (state * 1103515245 + 12345) & 0x7FFFFFFF
        return (state / 0x7FFFFFFF) - 0.5  # uniform in [-0.5, 0.5]

    # Make speed and feed linearly independent
    speed = [10.0 + 2.0 * i + _rand() * 3.0 for i in range(n)]
    feed = [0.5 + 0.05 * (i * i % 37) + _rand() * 1.0 for i in range(n)]
    y = [1.5 * speed[i] + 0.8 * feed[i] + 10.0 + _rand() * 0.5 for i in range(n)]
    return {'speed': speed, 'feed': feed}, y


# ------------------------------------------------------------------ #
#  Tests                                                              #
# ------------------------------------------------------------------ #

class TestRegressionModelManagerFit:
    """Test fitting models."""

    def test_fit_perfect_linear(self):
        """Fit a noise-free dataset and verify coefficients / R²."""
        mgr = RegressionModelManager()
        X, y = _make_linear_dataset()
        model = mgr.fit('m1', 'Linear Test', X, y, 'output')

        assert model.model_id == 'm1'
        assert model.name == 'Linear Test'
        assert model.target == 'output'
        assert model.n_samples == 50
        assert model.r_squared == pytest.approx(1.0, abs=1e-8)
        assert model.intercept == pytest.approx(5.0, abs=1e-6)
        # Features are sorted, so x1 first, x2 second
        assert model.features == ['x1', 'x2']
        assert model.coefficients[0] == pytest.approx(2.0, abs=1e-6)
        assert model.coefficients[1] == pytest.approx(3.0, abs=1e-6)

    def test_fit_stores_model(self):
        """Verify the model is stored and retrievable."""
        mgr = RegressionModelManager()
        X, y = _make_linear_dataset(20)
        mgr.fit('stored', 'Stored Model', X, y, 'val')
        retrieved = mgr.get_model('stored')
        assert retrieved.model_id == 'stored'

    def test_fit_empty_raises(self):
        """Fitting with empty y_data should raise."""
        mgr = RegressionModelManager()
        with pytest.raises(ValueError, match="must not be empty"):
            mgr.fit('bad', 'Bad', {'a': []}, [], 'target')


class TestRegressionModelManagerPredict:
    """Test prediction with fitted models."""

    def test_predict_perfect(self):
        """Predict on a perfectly fitted model."""
        mgr = RegressionModelManager()
        X, y = _make_linear_dataset()
        mgr.fit('m1', 'Linear', X, y, 'out')

        result = mgr.predict('m1', {'x1': 10.0, 'x2': 5.0})
        expected = 2.0 * 10.0 + 3.0 * 5.0 + 5.0  # 40
        assert result.predicted_value == pytest.approx(expected, abs=1e-4)
        assert result.model_id == 'm1'
        assert 'x1' in result.features_used
        assert 'x2' in result.features_used

    def test_predict_missing_feature_raises(self):
        """Predicting with a missing feature should raise KeyError."""
        mgr = RegressionModelManager()
        X, y = _make_linear_dataset()
        mgr.fit('m1', 'Linear', X, y, 'out')

        with pytest.raises(KeyError, match="Missing feature"):
            mgr.predict('m1', {'x1': 1.0})  # x2 missing

    def test_predict_confidence_interval(self):
        """Confidence interval should bracket the predicted value."""
        mgr = RegressionModelManager()
        X, y = _make_noisy_dataset()
        mgr.fit('noisy', 'Noisy', X, y, 'out')

        result = mgr.predict('noisy', {'speed': 20.0, 'feed': 1.0})
        lo, hi = result.confidence_interval
        assert lo <= result.predicted_value <= hi


class TestRegressionModelManagerEvaluate:
    """Test model evaluation."""

    def test_evaluate_perfect(self):
        """R² = 1, RMSE = 0, MAE = 0 on perfect data."""
        mgr = RegressionModelManager()
        X, y = _make_linear_dataset()
        mgr.fit('m1', 'Linear', X, y, 'out')

        metrics = mgr.evaluate('m1', X, y)
        assert metrics['r_squared'] == pytest.approx(1.0, abs=1e-8)
        assert metrics['rmse'] == pytest.approx(0.0, abs=1e-6)
        assert metrics['mae'] == pytest.approx(0.0, abs=1e-6)

    def test_evaluate_noisy(self):
        """Noisy data should still have high R² but RMSE > 0."""
        mgr = RegressionModelManager()
        X, y = _make_noisy_dataset()
        mgr.fit('noisy', 'Noisy', X, y, 'out')

        metrics = mgr.evaluate('noisy', X, y)
        assert metrics['r_squared'] > 0.99
        assert metrics['rmse'] > 0.0


class TestRegressionModelManagerUtilities:
    """Test utility methods."""

    def test_get_model_not_found(self):
        """Getting a non-existent model should raise KeyError."""
        mgr = RegressionModelManager()
        with pytest.raises(KeyError, match="No model found"):
            mgr.get_model('ghost')

    def test_get_all_models(self):
        """get_all_models returns every fitted model."""
        mgr = RegressionModelManager()
        X, y = _make_linear_dataset(20)
        mgr.fit('a', 'A', X, y, 't')
        mgr.fit('b', 'B', X, y, 't')
        mgr.fit('c', 'C', X, y, 't')
        assert len(mgr.get_all_models()) == 3

    def test_get_feature_coefficients(self):
        """Feature coefficients dict has correct keys and values."""
        mgr = RegressionModelManager()
        X, y = _make_linear_dataset()
        mgr.fit('m1', 'Linear', X, y, 'out')

        coeffs = mgr.get_feature_coefficients('m1')
        assert set(coeffs.keys()) == {'x1', 'x2'}
        assert coeffs['x1'] == pytest.approx(2.0, abs=1e-6)
        assert coeffs['x2'] == pytest.approx(3.0, abs=1e-6)

    def test_compare_models_ranking(self):
        """compare_models should rank models by R² descending."""
        mgr = RegressionModelManager()
        X_perf, y_perf = _make_linear_dataset()
        X_noisy, y_noisy = _make_noisy_dataset()

        mgr.fit('perfect', 'Perfect', X_perf, y_perf, 't')
        mgr.fit('noisy', 'Noisy', X_noisy, y_noisy, 't')

        ranked = mgr.compare_models(['noisy', 'perfect'])
        assert ranked[0].model_id == 'perfect'
        assert ranked[0].r_squared >= ranked[1].r_squared
