"""Tests for predictive fleet health data model and aggregation logic.

Validates RUL classification, health score coloring, anomaly countdown
formatting, fleet-level aggregation, and trend direction derivation
without any ROS2 or Unity dependencies.
"""

import math
import pytest
from dataclasses import dataclass, field
from typing import List, Optional


# ---------------------------------------------------------------------------
# Pure data model (mirrors C# PredictiveHealthData)
# ---------------------------------------------------------------------------

@dataclass
class PredictiveHealthData:
    tool_rul_minutes: float = math.inf
    next_anomaly_eta_seconds: float = math.inf
    next_anomaly_type: str = ""
    next_anomaly_block_index: int = 0
    overall_health_score: float = 1.0
    oee_score: float = 100.0
    trend_direction: str = "stable"  # "improving", "stable", "degrading"


# ---------------------------------------------------------------------------
# Pure helper functions (mirror C# logic)
# ---------------------------------------------------------------------------

def classify_rul_color(rul_minutes: float) -> str:
    """Return 'green', 'yellow', or 'red' based on RUL thresholds."""
    if math.isinf(rul_minutes) or rul_minutes > 60.0:
        return "green"
    if rul_minutes > 30.0:
        return "yellow"
    return "red"


def get_health_color_name(score: float) -> str:
    """Return a color band name for a 0-1 health score."""
    score = max(0.0, min(1.0, score))
    if score >= 0.7:
        return "green"
    if score >= 0.3:
        return "yellow"
    return "red"


def format_anomaly_countdown(anomaly_type: str, eta_seconds: float) -> Optional[str]:
    """Format an anomaly countdown string, or None if no anomaly predicted."""
    if not anomaly_type or math.isinf(eta_seconds) or eta_seconds < 0:
        return None
    total = round(eta_seconds)
    minutes = total // 60
    seconds = total % 60
    return f"{anomaly_type} in {minutes}:{seconds:02d}"


def compute_fleet_health(machines: List[PredictiveHealthData]) -> float:
    """Average overall_health_score across the fleet."""
    if not machines:
        return 1.0
    return sum(m.overall_health_score for m in machines) / len(machines)


def count_rul_critical(machines: List[PredictiveHealthData], threshold: float = 30.0) -> int:
    """Count machines with RUL below threshold minutes."""
    return sum(1 for m in machines if m.tool_rul_minutes < threshold)


def count_upcoming_anomalies(machines: List[PredictiveHealthData], horizon_seconds: float = 300.0) -> int:
    """Count machines with a predicted anomaly within horizon_seconds."""
    return sum(
        1 for m in machines
        if not math.isinf(m.next_anomaly_eta_seconds)
        and 0 <= m.next_anomaly_eta_seconds <= horizon_seconds
    )


def find_worst_machine(machines: List[PredictiveHealthData]) -> Optional[PredictiveHealthData]:
    """Return the machine with the lowest health score, or None if empty."""
    if not machines:
        return None
    return min(machines, key=lambda m: m.overall_health_score)


def derive_trend_direction(oee_history: List[float], window: int = 5) -> str:
    """Derive trend direction from recent OEE history.

    Compares average of last `window` samples to the previous `window`.
    """
    if len(oee_history) < window * 2:
        return "stable"
    recent = oee_history[-window:]
    previous = oee_history[-2 * window:-window]
    avg_recent = sum(recent) / len(recent)
    avg_previous = sum(previous) / len(previous)
    diff = avg_recent - avg_previous
    if diff > 1.0:
        return "improving"
    if diff < -1.0:
        return "degrading"
    return "stable"


def compute_health_from_components(
    spindle_load_norm: float,
    temperature_norm: float,
    power_norm: float,
    wear_norm: float,
    has_anomaly: bool = False,
) -> float:
    """Compute overall health score from normalized component metrics (0-1 each).

    Lower component values are healthier; health = 1 - worst_metric.
    Active anomaly forces score to 0.
    """
    if has_anomaly:
        return 0.0
    worst = max(spindle_load_norm, temperature_norm, power_norm, wear_norm)
    return max(0.0, min(1.0, 1.0 - worst))


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestRULClassification:
    """Test RUL color classification (green/yellow/red)."""

    def test_rul_green_high(self):
        assert classify_rul_color(90.0) == "green"

    def test_rul_green_boundary(self):
        assert classify_rul_color(60.1) == "green"

    def test_rul_yellow_boundary(self):
        assert classify_rul_color(60.0) == "yellow"

    def test_rul_yellow_mid(self):
        assert classify_rul_color(45.0) == "yellow"

    def test_rul_red_boundary(self):
        assert classify_rul_color(30.0) == "red"

    def test_rul_red_low(self):
        assert classify_rul_color(5.0) == "red"

    def test_rul_red_zero(self):
        assert classify_rul_color(0.0) == "red"

    def test_rul_infinity(self):
        assert classify_rul_color(math.inf) == "green"


class TestHealthColor:
    """Test health score to color mapping."""

    def test_perfect_health(self):
        assert get_health_color_name(1.0) == "green"

    def test_good_health(self):
        assert get_health_color_name(0.85) == "green"

    def test_green_boundary(self):
        assert get_health_color_name(0.7) == "green"

    def test_warning_zone(self):
        assert get_health_color_name(0.5) == "yellow"

    def test_yellow_boundary(self):
        assert get_health_color_name(0.3) == "yellow"

    def test_critical_zone(self):
        assert get_health_color_name(0.1) == "red"

    def test_zero_health(self):
        assert get_health_color_name(0.0) == "red"

    def test_clamped_above(self):
        assert get_health_color_name(1.5) == "green"

    def test_clamped_below(self):
        assert get_health_color_name(-0.5) == "red"


class TestAnomalyCountdown:
    """Test anomaly countdown formatting."""

    def test_normal_countdown(self):
        result = format_anomaly_countdown("CHATTER_RISK", 222.0)
        assert result == "CHATTER_RISK in 3:42"

    def test_short_countdown(self):
        result = format_anomaly_countdown("FORCE_WARNING", 45.0)
        assert result == "FORCE_WARNING in 0:45"

    def test_exact_minute(self):
        result = format_anomaly_countdown("VIBRATION", 120.0)
        assert result == "VIBRATION in 2:00"

    def test_no_anomaly_infinity(self):
        result = format_anomaly_countdown("", math.inf)
        assert result is None

    def test_no_anomaly_empty_type(self):
        result = format_anomaly_countdown("", 60.0)
        assert result is None

    def test_no_anomaly_inf_eta(self):
        result = format_anomaly_countdown("CHATTER", math.inf)
        assert result is None

    def test_negative_eta_returns_none(self):
        result = format_anomaly_countdown("CHATTER", -10.0)
        assert result is None

    def test_zero_eta(self):
        result = format_anomaly_countdown("FORCE_WARNING", 0.0)
        assert result == "FORCE_WARNING in 0:00"


class TestFleetAggregation:
    """Test fleet-level aggregation functions."""

    def test_fleet_health_single(self):
        machines = [PredictiveHealthData(overall_health_score=0.8)]
        assert compute_fleet_health(machines) == pytest.approx(0.8)

    def test_fleet_health_average(self):
        machines = [
            PredictiveHealthData(overall_health_score=0.9),
            PredictiveHealthData(overall_health_score=0.5),
            PredictiveHealthData(overall_health_score=0.7),
        ]
        assert compute_fleet_health(machines) == pytest.approx(0.7, abs=1e-6)

    def test_fleet_health_empty(self):
        assert compute_fleet_health([]) == 1.0

    def test_worst_machine(self):
        m1 = PredictiveHealthData(overall_health_score=0.9)
        m2 = PredictiveHealthData(overall_health_score=0.3)
        m3 = PredictiveHealthData(overall_health_score=0.7)
        worst = find_worst_machine([m1, m2, m3])
        assert worst is m2

    def test_worst_machine_empty(self):
        assert find_worst_machine([]) is None

    def test_rul_critical_count(self):
        machines = [
            PredictiveHealthData(tool_rul_minutes=10.0),
            PredictiveHealthData(tool_rul_minutes=45.0),
            PredictiveHealthData(tool_rul_minutes=5.0),
            PredictiveHealthData(tool_rul_minutes=90.0),
        ]
        assert count_rul_critical(machines) == 2

    def test_rul_critical_none(self):
        machines = [
            PredictiveHealthData(tool_rul_minutes=60.0),
            PredictiveHealthData(tool_rul_minutes=90.0),
        ]
        assert count_rul_critical(machines) == 0

    def test_upcoming_anomalies(self):
        machines = [
            PredictiveHealthData(next_anomaly_eta_seconds=120.0, next_anomaly_type="CHATTER"),
            PredictiveHealthData(next_anomaly_eta_seconds=600.0, next_anomaly_type="FORCE"),
            PredictiveHealthData(next_anomaly_eta_seconds=math.inf),
            PredictiveHealthData(next_anomaly_eta_seconds=60.0, next_anomaly_type="VIBRATION"),
        ]
        assert count_upcoming_anomalies(machines) == 2

    def test_upcoming_anomalies_none(self):
        machines = [
            PredictiveHealthData(next_anomaly_eta_seconds=math.inf),
            PredictiveHealthData(next_anomaly_eta_seconds=500.0, next_anomaly_type="X"),
        ]
        # 500 < 300? No. So only the inf one is excluded. 500 > 300 => excluded too.
        # Wait: 500 > 300, so not counted.
        assert count_upcoming_anomalies(machines) == 0


class TestTrendDirection:
    """Test OEE trend direction derivation."""

    def test_improving_trend(self):
        history = [70, 71, 72, 73, 74, 80, 81, 82, 83, 84]
        assert derive_trend_direction(history) == "improving"

    def test_degrading_trend(self):
        history = [80, 81, 82, 83, 84, 70, 71, 72, 73, 74]
        assert derive_trend_direction(history) == "degrading"

    def test_stable_trend(self):
        history = [80, 80, 80, 80, 80, 80, 80, 80, 80, 80]
        assert derive_trend_direction(history) == "stable"

    def test_insufficient_data(self):
        history = [80, 81, 82]
        assert derive_trend_direction(history) == "stable"

    def test_empty_history(self):
        assert derive_trend_direction([]) == "stable"


class TestHealthFromComponents:
    """Test health score calculation from component metrics."""

    def test_perfect_state(self):
        score = compute_health_from_components(0.0, 0.0, 0.0, 0.0)
        assert score == pytest.approx(1.0)

    def test_critical_state_all_max(self):
        score = compute_health_from_components(1.0, 1.0, 1.0, 1.0)
        assert score == pytest.approx(0.0)

    def test_anomaly_forces_zero(self):
        score = compute_health_from_components(0.1, 0.1, 0.1, 0.1, has_anomaly=True)
        assert score == pytest.approx(0.0)

    def test_single_high_metric(self):
        score = compute_health_from_components(0.0, 0.0, 0.0, 0.8)
        assert score == pytest.approx(0.2)

    def test_moderate_all(self):
        score = compute_health_from_components(0.3, 0.3, 0.3, 0.3)
        assert score == pytest.approx(0.7)


class TestMultipleMachinesIntegration:
    """Integration tests with multiple machines in different health states."""

    @pytest.fixture
    def mixed_fleet(self):
        return [
            PredictiveHealthData(
                tool_rul_minutes=90.0,
                overall_health_score=0.95,
                oee_score=88.0,
                trend_direction="improving",
                next_anomaly_eta_seconds=math.inf,
            ),
            PredictiveHealthData(
                tool_rul_minutes=20.0,
                overall_health_score=0.4,
                oee_score=55.0,
                trend_direction="degrading",
                next_anomaly_eta_seconds=120.0,
                next_anomaly_type="CHATTER_RISK",
                next_anomaly_block_index=42,
            ),
            PredictiveHealthData(
                tool_rul_minutes=50.0,
                overall_health_score=0.7,
                oee_score=72.0,
                trend_direction="stable",
                next_anomaly_eta_seconds=600.0,
                next_anomaly_type="FORCE_WARNING",
            ),
        ]

    def test_fleet_avg_health(self, mixed_fleet):
        avg = compute_fleet_health(mixed_fleet)
        expected = (0.95 + 0.4 + 0.7) / 3.0
        assert avg == pytest.approx(expected, abs=1e-6)

    def test_fleet_worst_is_degrading(self, mixed_fleet):
        worst = find_worst_machine(mixed_fleet)
        assert worst.overall_health_score == pytest.approx(0.4)
        assert worst.trend_direction == "degrading"

    def test_fleet_rul_critical(self, mixed_fleet):
        assert count_rul_critical(mixed_fleet) == 1

    def test_fleet_upcoming_anomalies(self, mixed_fleet):
        # Only the CHATTER_RISK at 120s is within 5 min
        assert count_upcoming_anomalies(mixed_fleet) == 1

    def test_fleet_degrades_when_any_critical(self, mixed_fleet):
        """Fleet health should be below 1.0 if any machine is critical."""
        avg = compute_fleet_health(mixed_fleet)
        assert avg < 1.0

    def test_rul_colors_across_fleet(self, mixed_fleet):
        colors = [classify_rul_color(m.tool_rul_minutes) for m in mixed_fleet]
        assert colors == ["green", "red", "yellow"]

    def test_health_colors_across_fleet(self, mixed_fleet):
        colors = [get_health_color_name(m.overall_health_score) for m in mixed_fleet]
        assert colors == ["green", "yellow", "green"]

    def test_anomaly_countdowns_across_fleet(self, mixed_fleet):
        countdowns = [
            format_anomaly_countdown(m.next_anomaly_type, m.next_anomaly_eta_seconds)
            for m in mixed_fleet
        ]
        assert countdowns[0] is None
        assert countdowns[1] == "CHATTER_RISK in 2:00"
        assert countdowns[2] == "FORCE_WARNING in 10:00"
