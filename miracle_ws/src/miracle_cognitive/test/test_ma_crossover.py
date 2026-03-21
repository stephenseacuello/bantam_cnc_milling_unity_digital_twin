"""Tests for MovingAverageCrossoverDetector."""

import math
import sys
from unittest.mock import MagicMock

# Mock ROS2/MIRACLE dependencies before importing the module under test.
for mod in ['miracle_core.datatypes', 'miracle_core.constants',
            'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_cognitive.interface.explanation_generator import (
    CrossoverEvent,
    MovingAverageCrossoverDetector,
    TrendAnalysis,
)


@pytest.fixture
def detector():
    return MovingAverageCrossoverDetector()


# ------------------------------------------------------------------ #
#  1. SMA basic computation                                          #
# ------------------------------------------------------------------ #
class TestCalculateSMA:
    def test_sma_basic(self, detector):
        data = [1.0, 2.0, 3.0, 4.0, 5.0]
        result = detector.calculate_sma(data, window=3)

        # First two entries should be NaN
        assert math.isnan(result[0])
        assert math.isnan(result[1])
        # SMA(3) of [1,2,3] = 2.0, [2,3,4] = 3.0, [3,4,5] = 4.0
        assert result[2] == pytest.approx(2.0)
        assert result[3] == pytest.approx(3.0)
        assert result[4] == pytest.approx(4.0)

    def test_sma_window_one(self, detector):
        data = [10.0, 20.0, 30.0]
        result = detector.calculate_sma(data, window=1)
        assert result == pytest.approx(data)

    def test_sma_invalid_window_raises(self, detector):
        with pytest.raises(ValueError):
            detector.calculate_sma([1.0, 2.0], window=0)


# ------------------------------------------------------------------ #
#  2. EMA basic computation                                          #
# ------------------------------------------------------------------ #
class TestCalculateEMA:
    def test_ema_basic(self, detector):
        data = [1.0, 2.0, 3.0, 4.0, 5.0]
        result = detector.calculate_ema(data, span=3)

        # First value is seeded with data[0]
        assert result[0] == pytest.approx(1.0)
        # k = 2/(3+1) = 0.5
        # ema[1] = 2*0.5 + 1*0.5 = 1.5
        assert result[1] == pytest.approx(1.5)
        # ema[2] = 3*0.5 + 1.5*0.5 = 2.25
        assert result[2] == pytest.approx(2.25)
        assert len(result) == len(data)

    def test_ema_empty_data(self, detector):
        assert detector.calculate_ema([], span=5) == []

    def test_ema_invalid_span_raises(self, detector):
        with pytest.raises(ValueError):
            detector.calculate_ema([1.0], span=0)


# ------------------------------------------------------------------ #
#  3. Golden cross detection                                         #
# ------------------------------------------------------------------ #
class TestDetectCrossovers:
    def test_golden_cross_detected(self, detector):
        """Build a series where the fast MA crosses above the slow MA."""
        # Start low then ramp up sharply — fast MA will overtake slow MA.
        data = [10.0] * 25 + [20.0 + i for i in range(15)]
        events = detector.detect_crossovers(data, fast_window=3, slow_window=10)

        golden = [e for e in events if e.crossover_type == 'golden_cross']
        assert len(golden) >= 1
        assert golden[0].signal_strength >= 0.0

    def test_death_cross_detected(self, detector):
        """Build a series where the fast MA crosses below the slow MA."""
        # Start high then drop sharply.
        data = [50.0] * 25 + [50.0 - 2 * i for i in range(15)]
        events = detector.detect_crossovers(data, fast_window=3, slow_window=10)

        death = [e for e in events if e.crossover_type == 'death_cross']
        assert len(death) >= 1

    def test_no_crossover_in_flat_series(self, detector):
        data = [5.0] * 40
        events = detector.detect_crossovers(data, fast_window=3, slow_window=10)
        assert events == []

    def test_fast_ge_slow_raises(self, detector):
        with pytest.raises(ValueError):
            detector.detect_crossovers([1.0] * 30, fast_window=10, slow_window=5)

    def test_custom_timestamps(self, detector):
        data = [10.0] * 25 + [20.0 + i for i in range(15)]
        ts = [float(100 + i) for i in range(len(data))]
        events = detector.detect_crossovers(data, fast_window=3, slow_window=10,
                                            timestamps=ts)
        if events:
            # Timestamps should come from the supplied list
            assert events[0].timestamp >= 100.0


# ------------------------------------------------------------------ #
#  4. Trend analysis                                                 #
# ------------------------------------------------------------------ #
class TestGetTrendAnalysis:
    def test_bullish_trend(self, detector):
        data = [10.0] * 25 + [20.0 + i for i in range(15)]
        analysis = detector.get_trend_analysis(data, fast_window=3,
                                               slow_window=10)

        assert isinstance(analysis, TrendAnalysis)
        assert analysis.current_trend == 'bullish'
        assert len(analysis.fast_ma) == len(data)
        assert len(analysis.slow_ma) == len(data)
        assert len(analysis.divergence) == len(data)

    def test_bearish_trend(self, detector):
        data = [50.0] * 25 + [50.0 - 2 * i for i in range(15)]
        analysis = detector.get_trend_analysis(data, fast_window=3,
                                               slow_window=10)
        assert analysis.current_trend == 'bearish'

    def test_neutral_trend_flat(self, detector):
        data = [5.0] * 40
        analysis = detector.get_trend_analysis(data, fast_window=3,
                                               slow_window=10)
        assert analysis.current_trend == 'neutral'


# ------------------------------------------------------------------ #
#  5. MACD calculation                                               #
# ------------------------------------------------------------------ #
class TestCalculateMACD:
    def test_macd_lengths(self, detector):
        data = [float(i) for i in range(50)]
        macd_line, signal_line, histogram = detector.calculate_macd(data)

        assert len(macd_line) == len(data)
        assert len(signal_line) == len(data)
        assert len(histogram) == len(data)

    def test_macd_histogram_relation(self, detector):
        data = [float(i) for i in range(50)]
        macd_line, signal_line, histogram = detector.calculate_macd(data)

        for m, s, h in zip(macd_line, signal_line, histogram):
            assert h == pytest.approx(m - s)


# ------------------------------------------------------------------ #
#  6. Signal strength                                                #
# ------------------------------------------------------------------ #
class TestGetSignalStrength:
    def test_signal_strength_basic(self, detector):
        fast = [10.0, 12.0, 15.0]
        slow = [10.0, 10.0, 10.0]
        strength = detector.get_signal_strength(fast, slow, 2)
        # |15 - 10| / |10| = 0.5
        assert strength == pytest.approx(0.5)

    def test_signal_strength_zero_slow(self, detector):
        fast = [5.0]
        slow = [0.0]
        assert detector.get_signal_strength(fast, slow, 0) == 0.0

    def test_signal_strength_out_of_bounds(self, detector):
        assert detector.get_signal_strength([1.0], [1.0], 5) == 0.0


# ------------------------------------------------------------------ #
#  7. CrossoverEvent dataclass                                       #
# ------------------------------------------------------------------ #
class TestCrossoverEventDataclass:
    def test_fields(self):
        evt = CrossoverEvent(
            index=42,
            timestamp=1000.0,
            crossover_type='golden_cross',
            fast_value=11.5,
            slow_value=10.0,
            signal_strength=0.15,
        )
        assert evt.index == 42
        assert evt.crossover_type == 'golden_cross'
        assert evt.signal_strength == pytest.approx(0.15)
