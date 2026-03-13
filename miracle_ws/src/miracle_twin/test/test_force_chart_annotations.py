"""
Tests for force chart anomaly annotation data model and severity classification.

These are pure data-model tests that mirror the C# AnomalyAnnotation struct
and the GetMarkerSymbol / GetSeverityColor utility methods from ForceChart.cs.
No ROS2 or Unity dependencies are required.
"""
import dataclasses
import pytest
from typing import List, Optional


# ── Data model mirroring C# AnomalyAnnotation ───────────────────────

@dataclasses.dataclass
class AnomalyAnnotation:
    """Python mirror of ForceChart.AnomalyAnnotation (C#)."""
    block_index: int = 0
    marker_type: str = ""
    severity: float = 0.0
    predicted_value: float = 0.0
    threshold: float = 0.0
    recommendation: str = ""
    chart_x: float = 0.0


# ── Utility functions mirroring C# static methods ───────────────────

_MARKER_SYMBOL_MAP = {
    "FORCE": "\u25B2",    # ▲
    "THERMAL": "\u25CF",  # ●
    "WEAR": "\u25C6",     # ◆
    "CHATTER": "\u2605",  # ★
    "SURFACE": "\u25A0",  # ■
    "TOOL": "\u26A0",     # ⚠
}
_DEFAULT_SYMBOL = "\u26A0"  # ⚠


def get_marker_symbol(marker_type: str) -> str:
    """Mirror of ForceChart.GetMarkerSymbol."""
    if not marker_type:
        return _DEFAULT_SYMBOL
    upper = marker_type.upper()
    for prefix, symbol in _MARKER_SYMBOL_MAP.items():
        if upper.startswith(prefix):
            return symbol
    return _DEFAULT_SYMBOL


def get_severity_color(severity: float) -> str:
    """
    Mirror of ForceChart.GetSeverityColor.
    Returns a color name string instead of UnityEngine.Color.
    < 0.5 -> 'yellow', >= 0.5 -> 'orange', >= 0.8 -> 'red'
    """
    severity = max(0.0, min(1.0, severity))
    if severity >= 0.8:
        return "red"
    if severity >= 0.5:
        return "orange"
    return "yellow"


def cap_annotations(annotations: List[AnomalyAnnotation],
                    max_count: int = 20) -> List[AnomalyAnnotation]:
    """Mirror of ForceChart.SetAnomalyAnnotations capping logic."""
    if not annotations:
        return []
    if len(annotations) > max_count:
        return sorted(annotations, key=lambda a: a.severity, reverse=True)[:max_count]
    return list(annotations)


def generate_recommendation(marker_type: str, predicted: float,
                            threshold: float) -> str:
    """Generate a human-readable recommendation for an anomaly."""
    if not marker_type:
        return ""
    upper = marker_type.upper()
    if upper.startswith("FORCE"):
        overshoot_pct = int((predicted - threshold) / threshold * 100) if threshold > 0 else 0
        return f"Reduce feed rate by {overshoot_pct}%"
    if upper.startswith("THERMAL"):
        return "Increase coolant flow or reduce cutting speed"
    if upper.startswith("WEAR"):
        return "Schedule tool change before VB_max exceeded"
    if upper.startswith("CHATTER"):
        return "Adjust spindle speed to avoid resonance"
    if upper.startswith("SURFACE"):
        return "Reduce step-over or increase finishing passes"
    if upper.startswith("TOOL"):
        return "Verify toolpath clearance at this block"
    return "Review process parameters"


# ═════════════════════════════════════════════════════════════════════
# Tests
# ═════════════════════════════════════════════════════════════════════


class TestGetMarkerSymbol:
    """Test GetMarkerSymbol mapping for all known types."""

    def test_force_critical(self):
        assert get_marker_symbol("FORCE_CRITICAL") == "\u25B2"

    def test_force_warning(self):
        assert get_marker_symbol("FORCE_WARNING") == "\u25B2"

    def test_thermal_warning(self):
        assert get_marker_symbol("THERMAL_WARNING") == "\u25CF"

    def test_thermal_critical(self):
        assert get_marker_symbol("THERMAL_CRITICAL") == "\u25CF"

    def test_wear_critical(self):
        assert get_marker_symbol("WEAR_CRITICAL") == "\u25C6"

    def test_chatter_high(self):
        assert get_marker_symbol("CHATTER_HIGH") == "\u2605"

    def test_surface_roughness(self):
        assert get_marker_symbol("SURFACE_ROUGHNESS") == "\u25A0"

    def test_tool_collision(self):
        assert get_marker_symbol("TOOL_COLLISION") == "\u26A0"

    def test_unknown_type_returns_default(self):
        assert get_marker_symbol("VIBRATION_EXCESS") == "\u26A0"

    def test_empty_string_returns_default(self):
        assert get_marker_symbol("") == "\u26A0"

    def test_none_returns_default(self):
        assert get_marker_symbol(None) == "\u26A0"

    def test_case_insensitive(self):
        assert get_marker_symbol("force_critical") == "\u25B2"
        assert get_marker_symbol("Thermal_Warning") == "\u25CF"


class TestGetSeverityColor:
    """Test GetSeverityColor thresholds and boundary values."""

    def test_zero_severity(self):
        assert get_severity_color(0.0) == "yellow"

    def test_low_severity(self):
        assert get_severity_color(0.3) == "yellow"

    def test_boundary_below_medium(self):
        assert get_severity_color(0.49) == "yellow"

    def test_boundary_at_medium(self):
        assert get_severity_color(0.5) == "orange"

    def test_medium_severity(self):
        assert get_severity_color(0.65) == "orange"

    def test_boundary_below_high(self):
        assert get_severity_color(0.79) == "orange"

    def test_boundary_at_high(self):
        assert get_severity_color(0.8) == "red"

    def test_high_severity(self):
        assert get_severity_color(0.95) == "red"

    def test_max_severity(self):
        assert get_severity_color(1.0) == "red"

    def test_clamp_negative(self):
        """Negative severity should clamp to 0 -> yellow."""
        assert get_severity_color(-0.5) == "yellow"

    def test_clamp_above_one(self):
        """Severity above 1.0 should clamp to 1.0 -> red."""
        assert get_severity_color(1.5) == "red"


class TestAnomalyAnnotationDataModel:
    """Test the AnomalyAnnotation data model with all fields populated."""

    def test_all_fields_populated(self):
        ann = AnomalyAnnotation(
            block_index=42,
            marker_type="FORCE_CRITICAL",
            severity=0.85,
            predicted_value=195.0,
            threshold=180.0,
            recommendation="Reduce feed by 20%",
            chart_x=150.0,
        )
        assert ann.block_index == 42
        assert ann.marker_type == "FORCE_CRITICAL"
        assert ann.severity == pytest.approx(0.85)
        assert ann.predicted_value == pytest.approx(195.0)
        assert ann.threshold == pytest.approx(180.0)
        assert ann.recommendation == "Reduce feed by 20%"
        assert ann.chart_x == pytest.approx(150.0)

    def test_default_values(self):
        ann = AnomalyAnnotation()
        assert ann.block_index == 0
        assert ann.marker_type == ""
        assert ann.severity == 0.0
        assert ann.predicted_value == 0.0
        assert ann.threshold == 0.0
        assert ann.recommendation == ""
        assert ann.chart_x == 0.0

    def test_thermal_annotation(self):
        ann = AnomalyAnnotation(
            block_index=100,
            marker_type="THERMAL_WARNING",
            severity=0.6,
            predicted_value=85.0,
            threshold=75.0,
            recommendation="Increase coolant flow",
        )
        assert get_marker_symbol(ann.marker_type) == "\u25CF"
        assert get_severity_color(ann.severity) == "orange"

    def test_wear_annotation(self):
        ann = AnomalyAnnotation(
            block_index=200,
            marker_type="WEAR_CRITICAL",
            severity=0.9,
            predicted_value=0.28,
            threshold=0.30,
        )
        assert get_marker_symbol(ann.marker_type) == "\u25C6"
        assert get_severity_color(ann.severity) == "red"


class TestAnnotationsCapping:
    """Test max annotations cap behavior."""

    def test_under_limit_returns_all(self):
        annotations = [
            AnomalyAnnotation(block_index=i, severity=i * 0.1)
            for i in range(5)
        ]
        result = cap_annotations(annotations, max_count=20)
        assert len(result) == 5

    def test_at_limit_returns_all(self):
        annotations = [
            AnomalyAnnotation(block_index=i, severity=i * 0.05)
            for i in range(20)
        ]
        result = cap_annotations(annotations, max_count=20)
        assert len(result) == 20

    def test_over_limit_caps_to_max(self):
        annotations = [
            AnomalyAnnotation(block_index=i, severity=i * 0.03)
            for i in range(30)
        ]
        result = cap_annotations(annotations, max_count=20)
        assert len(result) == 20

    def test_over_limit_keeps_highest_severity(self):
        annotations = [
            AnomalyAnnotation(block_index=i, severity=i * 0.04)
            for i in range(30)
        ]
        result = cap_annotations(annotations, max_count=5)
        assert len(result) == 5
        # Should keep indices 29, 28, 27, 26, 25 (highest severity)
        severities = [a.severity for a in result]
        assert severities == sorted(severities, reverse=True)
        assert result[0].severity == pytest.approx(29 * 0.04)

    def test_empty_list(self):
        result = cap_annotations([], max_count=20)
        assert result == []

    def test_none_input(self):
        result = cap_annotations(None, max_count=20)
        assert result == []


class TestRecommendationGeneration:
    """Test recommendation text generation for various anomaly types."""

    def test_force_recommendation(self):
        rec = generate_recommendation("FORCE_CRITICAL", 195.0, 180.0)
        assert "Reduce feed rate" in rec
        assert "8%" in rec  # (195-180)/180 = 8.3% -> int = 8

    def test_force_recommendation_large_overshoot(self):
        rec = generate_recommendation("FORCE_CRITICAL", 250.0, 180.0)
        assert "Reduce feed rate" in rec
        assert "38%" in rec  # (250-180)/180 = 38.8% -> int = 38

    def test_thermal_recommendation(self):
        rec = generate_recommendation("THERMAL_WARNING", 90.0, 75.0)
        assert "coolant" in rec.lower() or "cutting speed" in rec.lower()

    def test_wear_recommendation(self):
        rec = generate_recommendation("WEAR_CRITICAL", 0.28, 0.30)
        assert "tool change" in rec.lower()

    def test_chatter_recommendation(self):
        rec = generate_recommendation("CHATTER_HIGH", 0.85, 0.70)
        assert "spindle" in rec.lower()

    def test_surface_recommendation(self):
        rec = generate_recommendation("SURFACE_ROUGHNESS", 3.2, 1.6)
        assert "step-over" in rec.lower() or "finishing" in rec.lower()

    def test_tool_recommendation(self):
        rec = generate_recommendation("TOOL_COLLISION", 1.0, 0.0)
        assert "toolpath" in rec.lower() or "clearance" in rec.lower()

    def test_unknown_type_recommendation(self):
        rec = generate_recommendation("UNKNOWN_TYPE", 1.0, 0.5)
        assert rec == "Review process parameters"

    def test_empty_type_recommendation(self):
        rec = generate_recommendation("", 1.0, 0.5)
        assert rec == ""


class TestSeverityBoundaryValues:
    """Exhaustive boundary value tests for severity classification."""

    @pytest.mark.parametrize("severity,expected", [
        (0.0, "yellow"),
        (0.001, "yellow"),
        (0.25, "yellow"),
        (0.499, "yellow"),
        (0.5, "orange"),
        (0.501, "orange"),
        (0.6, "orange"),
        (0.799, "orange"),
        (0.8, "red"),
        (0.801, "red"),
        (0.9, "red"),
        (1.0, "red"),
    ])
    def test_boundary(self, severity, expected):
        assert get_severity_color(severity) == expected
