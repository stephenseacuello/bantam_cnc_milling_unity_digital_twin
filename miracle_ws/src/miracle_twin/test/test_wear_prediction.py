"""Tests for WearPredictionModel — tool wear prediction via regression and Taylor."""
import math
import sys
from unittest.mock import MagicMock

# Mock heavy ROS/miracle_core dependencies so we can test in isolation.
for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import pytest
from miracle_twin.cutting_sim_proxy import (
    WearDataPoint,
    WearPrediction,
    WearPredictionModel,
    WearTrend,
)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _make_point(
    t_min: float,
    flank: float,
    *,
    crater: float = 0.0,
    speed: float = 150.0,
    feed: float = 0.05,
    ts: float = 0.0,
) -> WearDataPoint:
    return WearDataPoint(
        timestamp=ts,
        cutting_time_min=t_min,
        flank_wear_mm=flank,
        crater_wear_mm=crater,
        cutting_speed_m_min=speed,
        feed_mm_tooth=feed,
    )


def _model_with_linear_data() -> WearPredictionModel:
    """Return a model loaded with perfectly linear wear data (slope=0.01 mm/min)."""
    model = WearPredictionModel()
    for t in range(0, 21):
        model.add_data_point(_make_point(float(t), 0.01 * t))
    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestAddAndReset:
    """Data management basics."""

    def test_add_data_point(self):
        model = WearPredictionModel()
        pt = _make_point(5.0, 0.05)
        model.add_data_point(pt)
        # Should have one data point — verify via get_wear_trend
        trend = model.get_wear_trend()
        assert trend.data_points == 1

    def test_reset_clears_data(self):
        model = _model_with_linear_data()
        model.reset()
        with pytest.raises(ValueError):
            model.get_wear_trend()


class TestPredictWear:
    """Wear prediction with linear regression."""

    def test_predict_at_known_time(self):
        model = _model_with_linear_data()
        pred = model.predict_wear(10.0)
        assert isinstance(pred, WearPrediction)
        assert pred.predicted_wear_mm == pytest.approx(0.10, abs=1e-6)
        assert pred.model_used == 'linear_regression'

    def test_predict_extrapolation(self):
        model = _model_with_linear_data()
        pred = model.predict_wear(30.0)
        # slope=0.01 => 0.01*30 = 0.30
        assert pred.predicted_wear_mm == pytest.approx(0.30, abs=1e-4)
        # Extrapolation should reduce confidence
        pred_interp = model.predict_wear(10.0)
        assert pred.confidence < pred_interp.confidence

    def test_predict_no_data_raises(self):
        model = WearPredictionModel()
        with pytest.raises(ValueError):
            model.predict_wear(5.0)

    def test_predicted_wear_non_negative(self):
        """Predicted wear should never be negative even with noisy data."""
        model = WearPredictionModel()
        model.add_data_point(_make_point(0.0, 0.0))
        model.add_data_point(_make_point(1.0, 0.001))
        pred = model.predict_wear(0.0)
        assert pred.predicted_wear_mm >= 0.0


class TestPredictRemainingLife:
    """Remaining-life estimation."""

    def test_remaining_life_linear(self):
        model = _model_with_linear_data()
        # Current max time=20, wear at 20 = 0.20, limit 0.30
        # Remaining = (0.30-0.0)/0.01 - 20 = 10 min
        remaining = model.predict_remaining_life(wear_limit_mm=0.3)
        assert remaining == pytest.approx(10.0, abs=0.5)

    def test_remaining_life_already_exceeded(self):
        model = _model_with_linear_data()
        remaining = model.predict_remaining_life(wear_limit_mm=0.05)
        assert remaining == 0.0

    def test_remaining_life_insufficient_data(self):
        model = WearPredictionModel()
        model.add_data_point(_make_point(1.0, 0.01))
        assert model.predict_remaining_life() == float('inf')


class TestGetWearTrend:
    """Wear trend / linear regression statistics."""

    def test_trend_statistics(self):
        model = _model_with_linear_data()
        trend = model.get_wear_trend()
        assert isinstance(trend, WearTrend)
        assert trend.slope == pytest.approx(0.01, abs=1e-6)
        assert trend.intercept == pytest.approx(0.0, abs=1e-6)
        assert trend.r_squared == pytest.approx(1.0, abs=1e-6)
        assert trend.data_points == 21

    def test_trend_no_data_raises(self):
        model = WearPredictionModel()
        with pytest.raises(ValueError):
            model.get_wear_trend()


class TestGetWearRate:
    """Wear rate from recent data."""

    def test_wear_rate_linear(self):
        model = _model_with_linear_data()
        rate = model.get_wear_rate()
        assert rate == pytest.approx(0.01, abs=1e-4)

    def test_wear_rate_single_point(self):
        model = WearPredictionModel()
        model.add_data_point(_make_point(5.0, 0.05))
        assert model.get_wear_rate() == 0.0


class TestTaylorToolLife:
    """Taylor tool-life equation V*T^n = C."""

    def test_known_values(self):
        # V=150, n=0.125, C=300 => T=(300/150)^(1/0.125)=2^8=256
        life = WearPredictionModel.taylor_tool_life(150.0, n=0.125, C=300.0)
        assert life == pytest.approx(256.0, rel=1e-6)

    def test_zero_speed(self):
        life = WearPredictionModel.taylor_tool_life(0.0)
        assert life == float('inf')

    def test_higher_speed_shorter_life(self):
        slow = WearPredictionModel.taylor_tool_life(100.0)
        fast = WearPredictionModel.taylor_tool_life(200.0)
        assert fast < slow
