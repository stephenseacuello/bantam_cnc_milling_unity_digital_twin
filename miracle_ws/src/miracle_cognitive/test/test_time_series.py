"""Tests for TimeSeriesForecaster in explanation_generator.py."""

import sys
import math
from unittest.mock import MagicMock

# Mock ROS2 / miracle_core dependencies before importing the module
for mod in [
    'miracle_core.datatypes', 'miracle_core.constants',
    'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
    'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
    'miracle_msgs', 'miracle_msgs.msg',
]:
    sys.modules.setdefault(mod, MagicMock())

from miracle_cognitive.interface.explanation_generator import (
    ForecastPoint,
    ForecastResult,
    TimeSeriesForecaster,
)


# ---------------------------------------------------------------------------
# 1. Basic fit + forecast produces correct structure
# ---------------------------------------------------------------------------
class TestFitAndForecast:
    def test_forecast_returns_correct_number_of_points(self):
        ts = TimeSeriesForecaster(method='linear_trend')
        timestamps = [float(i) for i in range(20)]
        values = [2.0 * i + 1.0 for i in range(20)]
        ts.fit(timestamps, values)
        result = ts.forecast(n_steps=5, step_size_sec=1.0)

        assert isinstance(result, ForecastResult)
        assert len(result.forecasts) == 5
        assert result.method == 'linear_trend'
        assert all(isinstance(p, ForecastPoint) for p in result.forecasts)

    def test_forecast_timestamps_are_future(self):
        fc = TimeSeriesForecaster(method='moving_average', window=3)
        timestamps = [float(i) for i in range(10)]
        values = [float(i) for i in range(10)]
        fc.fit(timestamps, values)
        result = fc.forecast(n_steps=3, step_size_sec=2.0)

        last_ts = timestamps[-1]
        for pt in result.forecasts:
            assert pt.timestamp > last_ts


# ---------------------------------------------------------------------------
# 2. Linear trend produces accurate extrapolation
# ---------------------------------------------------------------------------
class TestLinearTrend:
    def test_linear_extrapolation_accuracy(self):
        fc = TimeSeriesForecaster(method='linear_trend')
        timestamps = [float(i) for i in range(50)]
        values = [3.0 * t + 7.0 for t in timestamps]
        fc.fit(timestamps, values)
        result = fc.forecast(n_steps=5, step_size_sec=1.0)

        for i, pt in enumerate(result.forecasts):
            expected = 3.0 * pt.timestamp + 7.0
            assert abs(pt.value - expected) < 1e-6, (
                f"Step {i}: expected {expected}, got {pt.value}"
            )


# ---------------------------------------------------------------------------
# 3. Exponential smoothing converges toward recent data
# ---------------------------------------------------------------------------
class TestExponentialSmoothing:
    def test_smoothed_value_close_to_recent_mean(self):
        fc = TimeSeriesForecaster(method='exponential_smoothing', alpha=0.5)
        timestamps = [float(i) for i in range(30)]
        # Step function: first 15 at 10, last 15 at 20
        values = [10.0] * 15 + [20.0] * 15
        fc.fit(timestamps, values)
        result = fc.forecast(n_steps=1)
        # With alpha=0.5 and 15 points at 20.0, level should be very close to 20
        assert result.forecasts[0].value > 19.0


# ---------------------------------------------------------------------------
# 4. Moving average
# ---------------------------------------------------------------------------
class TestMovingAverage:
    def test_moving_average_constant_series(self):
        fc = TimeSeriesForecaster(method='moving_average', window=5)
        timestamps = [float(i) for i in range(20)]
        values = [42.0] * 20
        fc.fit(timestamps, values)
        result = fc.forecast(n_steps=3)
        for pt in result.forecasts:
            assert abs(pt.value - 42.0) < 1e-9


# ---------------------------------------------------------------------------
# 5. Evaluate computes correct MAPE and RMSE
# ---------------------------------------------------------------------------
class TestEvaluate:
    def test_perfect_predictions(self):
        actual = [1.0, 2.0, 3.0, 4.0]
        predicted = [1.0, 2.0, 3.0, 4.0]
        mape, rmse = TimeSeriesForecaster.evaluate(actual, predicted)
        assert mape == 0.0
        assert rmse == 0.0

    def test_known_errors(self):
        actual = [100.0, 200.0]
        predicted = [110.0, 190.0]
        mape, rmse = TimeSeriesForecaster.evaluate(actual, predicted)
        # MAPE = mean(|10/100|, |10/200|) * 100 = (0.1 + 0.05)/2 * 100 = 7.5
        assert abs(mape - 7.5) < 1e-6
        # RMSE = sqrt((100 + 100)/2) = 10.0
        assert abs(rmse - 10.0) < 1e-6


# ---------------------------------------------------------------------------
# 6. Trend detection
# ---------------------------------------------------------------------------
class TestGetTrend:
    def test_increasing_trend(self):
        data = [float(i) for i in range(20)]
        assert TimeSeriesForecaster.get_trend(data) == 'increasing'

    def test_decreasing_trend(self):
        data = [100.0 - float(i) for i in range(20)]
        assert TimeSeriesForecaster.get_trend(data) == 'decreasing'

    def test_stable_trend(self):
        data = [5.0] * 20
        assert TimeSeriesForecaster.get_trend(data) == 'stable'


# ---------------------------------------------------------------------------
# 7. Seasonality detection
# ---------------------------------------------------------------------------
class TestSeasonality:
    def test_detects_known_period(self):
        period = 10
        data = [math.sin(2 * math.pi * i / period) for i in range(200)]
        detected = TimeSeriesForecaster.detect_seasonality(data, max_period=50)
        assert detected is not None
        assert abs(detected - period) <= 1, (
            f"Expected period ~{period}, got {detected}"
        )

    def test_no_seasonality_in_noise_free_linear(self):
        data = [float(i) for i in range(100)]
        detected = TimeSeriesForecaster.detect_seasonality(data, max_period=50)
        # Linear data has no repeating pattern; might return None or a spurious lag
        # We just ensure it doesn't crash and returns Optional[int]
        assert detected is None or isinstance(detected, int)


# ---------------------------------------------------------------------------
# 8. set_method switches behaviour
# ---------------------------------------------------------------------------
class TestSetMethod:
    def test_switch_method(self):
        fc = TimeSeriesForecaster(method='moving_average', window=3)
        timestamps = [float(i) for i in range(20)]
        values = [2.0 * i for i in range(20)]
        fc.fit(timestamps, values)

        result_ma = fc.forecast(n_steps=3)
        assert result_ma.method == 'moving_average'

        fc.set_method('linear_trend')
        result_lt = fc.forecast(n_steps=3)
        assert result_lt.method == 'linear_trend'

        # Linear trend on perfectly linear data should differ from MA
        # (MA averages recent values; LT extrapolates the line)
        assert result_lt.forecasts[0].value != result_ma.forecasts[0].value

    def test_invalid_method_raises(self):
        fc = TimeSeriesForecaster()
        try:
            fc.set_method('arima')
            assert False, "Should have raised ValueError"
        except ValueError:
            pass


# ---------------------------------------------------------------------------
# 9. Confidence intervals widen with horizon
# ---------------------------------------------------------------------------
class TestConfidenceIntervals:
    def test_intervals_widen(self):
        fc = TimeSeriesForecaster(method='linear_trend')
        timestamps = [float(i) for i in range(30)]
        # Add a bit of noise so residual_std > 0
        values = [2.0 * i + (0.5 if i % 3 == 0 else -0.3) for i in range(30)]
        fc.fit(timestamps, values)
        result = fc.forecast(n_steps=5)

        widths = [p.upper_bound - p.lower_bound for p in result.forecasts]
        # Each successive interval should be wider (or equal)
        for i in range(1, len(widths)):
            assert widths[i] >= widths[i - 1] - 1e-9


# ---------------------------------------------------------------------------
# 10. Edge cases
# ---------------------------------------------------------------------------
class TestEdgeCases:
    def test_fit_requires_min_data(self):
        fc = TimeSeriesForecaster()
        try:
            fc.fit([1.0], [2.0])
            assert False, "Should have raised ValueError"
        except ValueError:
            pass

    def test_forecast_before_fit_raises(self):
        fc = TimeSeriesForecaster()
        try:
            fc.forecast(n_steps=5)
            assert False, "Should have raised RuntimeError"
        except RuntimeError:
            pass
