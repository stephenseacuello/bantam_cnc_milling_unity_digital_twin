"""Tests for predictive quality and predictive OEE KPI integration.

Tests the pure functions compute_predictive_quality, compute_predictive_oee,
_classify_quality_trend, and _recommend_quality_action without ROS2.
"""

import sys
from unittest.mock import MagicMock

# Mock ROS2 and miracle_core dependencies before importing kpi_calculator
for mod in ['rclpy', 'rclpy.node', 'rclpy.callback_groups', 'rclpy.qos', 'rclpy.lifecycle',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
            'miracle_core.exceptions',
            'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

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

import pytest
from miracle_scada.kpi_calculator import (
    compute_predictive_quality,
    compute_predictive_oee,
    _classify_quality_trend,
    _recommend_quality_action,
    PredictiveQualityMetrics,
    PredictiveOEE,
)


# ---------------------------------------------------------------------------
# Predictive Quality Tests
# ---------------------------------------------------------------------------

class TestComputePredictiveQuality:
    """Tests for compute_predictive_quality."""

    def test_no_anomaly_markers_gives_100_predicted(self):
        """No anomalies: predicted quality equals current quality."""
        result = compute_predictive_quality([], total_blocks=100, current_quality=100.0)
        assert result.predicted_quality_pct == 100.0
        assert result.anomaly_risk_count == 0

    def test_force_critical_blocks_reduce_quality_by_2_each(self):
        """Each force_exceedance block reduces quality by 2%."""
        markers = [{'type': 'force_exceedance'}, {'type': 'force_exceedance'}]
        result = compute_predictive_quality(markers, total_blocks=100, current_quality=100.0)
        assert result.predicted_quality_pct == pytest.approx(96.0)
        assert result.force_exceedance_blocks == 2

    def test_thermal_warning_blocks_reduce_quality_by_1_each(self):
        """Each thermal_warning block reduces quality by 1%."""
        markers = [{'type': 'thermal_warning'}, {'type': 'thermal_warning'}, {'type': 'thermal_warning'}]
        result = compute_predictive_quality(markers, total_blocks=100, current_quality=100.0)
        assert result.predicted_quality_pct == pytest.approx(97.0)
        assert result.thermal_risk_blocks == 3

    def test_surface_quality_blocks_reduce_quality_by_3_each(self):
        """Each surface_quality_risk block reduces quality by 3%."""
        markers = [{'type': 'surface_quality_risk'}]
        result = compute_predictive_quality(markers, total_blocks=100, current_quality=100.0)
        assert result.predicted_quality_pct == pytest.approx(97.0)
        assert result.surface_quality_risk_blocks == 1

    def test_combined_anomaly_types(self):
        """Mixed anomaly types apply cumulative penalties."""
        markers = [
            {'type': 'force_exceedance'},      # -2%
            {'type': 'thermal_warning'},        # -1%
            {'type': 'surface_quality_risk'},   # -3%
        ]
        result = compute_predictive_quality(markers, total_blocks=100, current_quality=100.0)
        assert result.predicted_quality_pct == pytest.approx(94.0)
        assert result.anomaly_risk_count == 3

    def test_scrap_probability_zero_when_quality_above_90(self):
        """Scrap probability is 0 when predicted quality >= 90%."""
        markers = [{'type': 'force_exceedance'}]  # -2% -> 98%
        result = compute_predictive_quality(markers, total_blocks=100, current_quality=100.0)
        assert result.predicted_scrap_probability == 0.0

    def test_scrap_probability_nonzero_when_quality_below_90(self):
        """Scrap probability = 1 - predicted_quality/100 when quality < 90%."""
        # 6 force blocks = -12% => 88%
        markers = [{'type': 'force_exceedance'}] * 6
        result = compute_predictive_quality(markers, total_blocks=100, current_quality=100.0)
        assert result.predicted_quality_pct == pytest.approx(88.0)
        assert result.predicted_scrap_probability == pytest.approx(0.12)

    def test_quality_capped_at_zero_minimum(self):
        """Quality never goes below 0%."""
        markers = [{'type': 'force_exceedance'}] * 60  # -120% penalty
        result = compute_predictive_quality(markers, total_blocks=100, current_quality=100.0)
        assert result.predicted_quality_pct == 0.0

    def test_predicted_quality_never_exceeds_current(self):
        """Predicted quality cannot be higher than current quality."""
        result = compute_predictive_quality([], total_blocks=100, current_quality=75.0)
        assert result.predicted_quality_pct == 75.0

    def test_predicted_quality_degrades_from_current(self):
        """Predicted quality degrades from current quality, not from 100%."""
        markers = [{'type': 'force_exceedance'}]  # -2%
        result = compute_predictive_quality(markers, total_blocks=100, current_quality=80.0)
        assert result.predicted_quality_pct == pytest.approx(78.0)

    def test_empty_marker_list(self):
        """Empty marker list produces clean metrics."""
        result = compute_predictive_quality([], total_blocks=50, current_quality=95.0)
        assert result.predicted_quality_pct == 95.0
        assert result.anomaly_risk_count == 0
        assert result.force_exceedance_blocks == 0
        assert result.thermal_risk_blocks == 0
        assert result.surface_quality_risk_blocks == 0
        assert result.predicted_scrap_probability == 0.0

    def test_unknown_marker_type_ignored(self):
        """Markers with unknown types do not affect quality."""
        markers = [{'type': 'vibration_anomaly'}, {'type': 'unknown'}]
        result = compute_predictive_quality(markers, total_blocks=100, current_quality=100.0)
        assert result.predicted_quality_pct == 100.0
        assert result.anomaly_risk_count == 0

    def test_scrap_probability_at_zero_quality(self):
        """Scrap probability is 1.0 when quality reaches 0%."""
        markers = [{'type': 'surface_quality_risk'}] * 40  # -120%
        result = compute_predictive_quality(markers, total_blocks=100, current_quality=100.0)
        assert result.predicted_quality_pct == 0.0
        assert result.predicted_scrap_probability == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# Quality Trend Classification Tests
# ---------------------------------------------------------------------------

class TestClassifyQualityTrend:
    """Tests for _classify_quality_trend."""

    def test_degrading_when_predicted_much_lower(self):
        assert _classify_quality_trend(95.0, 90.0) == 'degrading'

    def test_improving_when_predicted_higher(self):
        assert _classify_quality_trend(90.0, 95.0) == 'improving'

    def test_stable_when_close(self):
        assert _classify_quality_trend(95.0, 95.0) == 'stable'

    def test_stable_within_threshold(self):
        assert _classify_quality_trend(95.0, 94.5) == 'stable'

    def test_degrading_just_beyond_threshold(self):
        assert _classify_quality_trend(95.0, 93.0) == 'degrading'


# ---------------------------------------------------------------------------
# Recommended Action Tests
# ---------------------------------------------------------------------------

class TestRecommendQualityAction:
    """Tests for _recommend_quality_action."""

    def test_none_when_no_anomalies(self):
        metrics = PredictiveQualityMetrics(
            current_quality_pct=100.0, predicted_quality_pct=100.0,
            anomaly_risk_count=0, force_exceedance_blocks=0,
            thermal_risk_blocks=0, surface_quality_risk_blocks=0,
            predicted_scrap_probability=0.0, quality_trend='stable',
            recommended_action='',
        )
        assert _recommend_quality_action(metrics) == 'none'

    def test_adjust_parameters_when_anomalies_present_low_scrap(self):
        metrics = PredictiveQualityMetrics(
            current_quality_pct=100.0, predicted_quality_pct=97.0,
            anomaly_risk_count=2, force_exceedance_blocks=1,
            thermal_risk_blocks=1, surface_quality_risk_blocks=0,
            predicted_scrap_probability=0.03, quality_trend='degrading',
            recommended_action='',
        )
        assert _recommend_quality_action(metrics) == 'adjust_parameters'

    def test_tool_change_when_scrap_above_20_pct(self):
        metrics = PredictiveQualityMetrics(
            current_quality_pct=100.0, predicted_quality_pct=75.0,
            anomaly_risk_count=5, force_exceedance_blocks=5,
            thermal_risk_blocks=0, surface_quality_risk_blocks=0,
            predicted_scrap_probability=0.25, quality_trend='degrading',
            recommended_action='',
        )
        assert _recommend_quality_action(metrics) == 'tool_change'

    def test_abort_when_scrap_above_50_pct(self):
        metrics = PredictiveQualityMetrics(
            current_quality_pct=100.0, predicted_quality_pct=40.0,
            anomaly_risk_count=10, force_exceedance_blocks=10,
            thermal_risk_blocks=0, surface_quality_risk_blocks=0,
            predicted_scrap_probability=0.60, quality_trend='degrading',
            recommended_action='',
        )
        assert _recommend_quality_action(metrics) == 'abort'


# ---------------------------------------------------------------------------
# Predictive OEE Tests
# ---------------------------------------------------------------------------

class TestComputePredictiveOEE:
    """Tests for compute_predictive_oee."""

    def test_healthy_tool_low_risk(self):
        """Healthy tool (60+ min RUL) should have zero availability risk."""
        result = compute_predictive_oee(
            current_oee=0.85, tool_rul_min=120.0,
            anomaly_markers=[], feed_override_pct=100.0,
        )
        assert result.availability_risk == 0.0
        assert result.performance_risk == 0.0
        assert result.quality_risk == 0.0
        assert result.predicted_oee == pytest.approx(0.85)

    def test_worn_tool_high_availability_risk(self):
        """Tool near end of life (RUL ~0) should have high availability risk."""
        result = compute_predictive_oee(
            current_oee=0.85, tool_rul_min=0.0,
            anomaly_markers=[], feed_override_pct=100.0,
        )
        assert result.availability_risk == 1.0

    def test_active_feed_override_performance_risk(self):
        """Feed override at 70% means 30% performance risk."""
        result = compute_predictive_oee(
            current_oee=0.85, tool_rul_min=120.0,
            anomaly_markers=[], feed_override_pct=70.0,
        )
        assert result.performance_risk == pytest.approx(0.3)

    def test_full_feed_no_performance_risk(self):
        """Feed at 100% means zero performance risk."""
        result = compute_predictive_oee(
            current_oee=0.85, tool_rul_min=120.0,
            anomaly_markers=[], feed_override_pct=100.0,
        )
        assert result.performance_risk == 0.0

    def test_quality_risk_from_anomalies(self):
        """Multiple anomaly markers increase quality risk."""
        markers = [{'type': 'force_exceedance'}] * 5
        result = compute_predictive_oee(
            current_oee=0.85, tool_rul_min=120.0,
            anomaly_markers=markers, feed_override_pct=100.0,
        )
        assert result.quality_risk == pytest.approx(0.5)

    def test_quality_risk_capped_at_1(self):
        """Quality risk cannot exceed 1.0."""
        markers = [{'type': 'force_exceedance'}] * 20
        result = compute_predictive_oee(
            current_oee=0.85, tool_rul_min=120.0,
            anomaly_markers=markers, feed_override_pct=100.0,
        )
        assert result.quality_risk == 1.0

    def test_time_to_intervention_from_tool_rul(self):
        """When no critical anomalies, intervention time = tool RUL."""
        result = compute_predictive_oee(
            current_oee=0.85, tool_rul_min=30.0,
            anomaly_markers=[], feed_override_pct=100.0,
        )
        assert result.time_to_next_intervention_min == 30.0

    def test_time_to_intervention_from_critical_anomaly(self):
        """Critical anomaly sooner than tool RUL drives intervention time."""
        markers = [{'type': 'force_exceedance', 'severity': 'critical', 'time_from_now_min': 10.0}]
        result = compute_predictive_oee(
            current_oee=0.85, tool_rul_min=30.0,
            anomaly_markers=markers, feed_override_pct=100.0,
        )
        assert result.time_to_next_intervention_min == 10.0

    def test_predicted_oee_lower_with_risks(self):
        """Predicted OEE should be lower than current when risks are present."""
        markers = [{'type': 'force_exceedance'}] * 3
        result = compute_predictive_oee(
            current_oee=0.85, tool_rul_min=10.0,
            anomaly_markers=markers, feed_override_pct=80.0,
        )
        assert result.predicted_oee < result.current_oee

    def test_predicted_oee_never_negative(self):
        """Predicted OEE should never go below 0."""
        markers = [{'type': 'force_exceedance'}] * 20
        result = compute_predictive_oee(
            current_oee=0.85, tool_rul_min=0.0,
            anomaly_markers=markers, feed_override_pct=0.0,
        )
        assert result.predicted_oee >= 0.0

    def test_current_oee_preserved(self):
        """The current_oee field should be passed through unchanged."""
        result = compute_predictive_oee(
            current_oee=0.72, tool_rul_min=60.0,
            anomaly_markers=[], feed_override_pct=100.0,
        )
        assert result.current_oee == 0.72


# ---------------------------------------------------------------------------
# Integration / Edge Case Tests
# ---------------------------------------------------------------------------

class TestPredictiveIntegration:
    """Integration-level tests combining predictive quality and OEE."""

    def test_end_to_end_clean_run(self):
        """Clean run: no anomalies, full feed, healthy tool."""
        q = compute_predictive_quality([], 100, 100.0)
        oee = compute_predictive_oee(0.90, 120.0, [], 100.0)
        assert q.recommended_action == 'none'
        assert oee.predicted_oee == pytest.approx(0.90)
        assert q.quality_trend == 'stable'

    def test_end_to_end_degraded_run(self):
        """Degraded run with multiple anomaly types."""
        markers = [
            {'type': 'force_exceedance'},
            {'type': 'force_exceedance'},
            {'type': 'thermal_warning'},
            {'type': 'surface_quality_risk'},
            {'type': 'surface_quality_risk'},
        ]
        q = compute_predictive_quality(markers, 200, 98.0)
        # penalty = 2*2 + 1*1 + 2*3 = 11%
        assert q.predicted_quality_pct == pytest.approx(87.0)
        assert q.predicted_scrap_probability == pytest.approx(0.13)
        assert q.quality_trend == 'degrading'
        assert q.recommended_action == 'adjust_parameters'

    def test_severe_degradation_triggers_abort(self):
        """Severe quality loss triggers abort recommendation."""
        markers = [{'type': 'surface_quality_risk'}] * 11  # -33%
        q = compute_predictive_quality(markers, 100, 80.0)
        assert q.predicted_quality_pct == pytest.approx(47.0)
        assert q.predicted_scrap_probability == pytest.approx(0.53)
        assert q.recommended_action == 'abort'
