"""Tests for AdaptiveStepoverCalculator logic (Python mirror of C# implementation).

Validates scallop height, inverse scallop, optimal stepover, stepdown,
engagement angle, material removal rate, and edge cases.
"""

import math
import pytest
from dataclasses import dataclass
from typing import Dict, Tuple


# ---- Python mirror of C# types ----

@dataclass
class StepoverResult:
    stepoverMm: float = 0.0
    scallopHeightMm: float = 0.0
    effectiveWidth: float = 0.0
    cuspHeightMm: float = 0.0
    materialRemovalRate: float = 0.0


# ---- Python mirror of AdaptiveStepoverCalculator ----

OPERATION_RANGES: Dict[str, Tuple[float, float]] = {
    "finishing":      (0.05, 0.10),
    "semi_finishing": (0.25, 0.40),
    "roughing":      (0.50, 0.75),
}

MATERIAL_STEPDOWN_FACTORS: Dict[str, float] = {
    "aluminum":  1.0,
    "steel":     0.5,
    "stainless": 0.35,
    "titanium":  0.25,
    "cast_iron": 0.6,
    "brass":     0.8,
    "plastic":   1.2,
}


class AdaptiveStepoverCalculator:

    # ── Scallop height ──────────────────────────────────────────

    def CalculateScallopHeight(self, toolDiameter: float, stepover: float) -> float:
        """h = R - sqrt(R^2 - (ae/2)^2)"""
        if toolDiameter <= 0.0:
            raise ValueError("Tool diameter must be positive.")
        if stepover <= 0.0:
            return 0.0
        if stepover > toolDiameter:
            raise ValueError("Stepover cannot exceed tool diameter.")

        R = toolDiameter / 2.0
        half_ae = stepover / 2.0
        discriminant = R * R - half_ae * half_ae
        if discriminant < 0.0:
            discriminant = 0.0
        return R - math.sqrt(discriminant)

    # ── Inverse scallop ─────────────────────────────────────────

    def CalculateStepoverFromScallop(self, toolDiameter: float,
                                     targetScallopHeight: float) -> float:
        """ae = 2 * sqrt(2*R*h - h^2)"""
        if toolDiameter <= 0.0:
            raise ValueError("Tool diameter must be positive.")
        if targetScallopHeight <= 0.0:
            return 0.0

        R = toolDiameter / 2.0
        if targetScallopHeight > R:
            raise ValueError("Target scallop height cannot exceed tool radius.")

        inner = 2.0 * R * targetScallopHeight - targetScallopHeight ** 2
        if inner < 0.0:
            inner = 0.0
        return 2.0 * math.sqrt(inner)

    # ── Optimal stepover for surface finish ─────────────────────

    def CalculateOptimalStepover(self, toolDiameter: float,
                                 targetRa: float,
                                 operation: str) -> StepoverResult:
        if toolDiameter <= 0.0:
            raise ValueError("Tool diameter must be positive.")
        if targetRa <= 0.0:
            raise ValueError("Target Ra must be positive.")

        op = operation.lower()
        if op not in OPERATION_RANGES:
            raise ValueError(
                f"Unknown operation '{operation}'. "
                "Use finishing, semi_finishing, or roughing.")

        min_frac, max_frac = OPERATION_RANGES[op]

        # Convert Ra (um) to approximate scallop height (mm)
        scallop_target = targetRa * 4.0 / 1000.0

        stepover = self.CalculateStepoverFromScallop(toolDiameter, scallop_target)

        min_step = min_frac * toolDiameter
        max_step = max_frac * toolDiameter
        stepover = max(min_step, min(stepover, max_step))

        scallop = self.CalculateScallopHeight(toolDiameter, stepover)
        return StepoverResult(
            stepoverMm=stepover,
            scallopHeightMm=scallop,
            effectiveWidth=stepover,
            cuspHeightMm=scallop,
            materialRemovalRate=0.0,
        )

    # ── Stepdown recommendation ─────────────────────────────────

    def CalculateStepdown(self, toolDiameter: float,
                          material: str, operation: str) -> float:
        if toolDiameter <= 0.0:
            raise ValueError("Tool diameter must be positive.")

        mat = material.lower()
        op = operation.lower()

        mat_factor = MATERIAL_STEPDOWN_FACTORS.get(mat, 0.5)

        if op == "finishing":
            op_factor = 0.1
        elif op == "semi_finishing":
            op_factor = 0.4
        elif op == "roughing":
            op_factor = 1.0
        else:
            raise ValueError(f"Unknown operation '{operation}'.")

        return toolDiameter * mat_factor * op_factor

    # ── Engagement angle ────────────────────────────────────────

    def GetEngagementAngle(self, toolDiameter: float, stepover: float) -> float:
        """theta = arccos(1 - ae / R)  in degrees"""
        if toolDiameter <= 0.0:
            raise ValueError("Tool diameter must be positive.")
        if stepover <= 0.0:
            return 0.0
        if stepover > toolDiameter:
            raise ValueError("Stepover cannot exceed tool diameter.")

        R = toolDiameter / 2.0
        cos_val = 1.0 - stepover / R
        cos_val = max(-1.0, min(cos_val, 1.0))
        return math.degrees(math.acos(cos_val))

    # ── Material removal rate ───────────────────────────────────

    def CalculateMRR(self, stepover: float, stepdown: float,
                     feedMmPerMin: float, numFlutes: int) -> float:
        if stepover <= 0.0 or stepdown <= 0.0 or feedMmPerMin <= 0.0:
            return 0.0
        return stepover * stepdown * feedMmPerMin

    # ── Full result convenience ─────────────────────────────────

    def GetFullResult(self, toolDiameter: float, stepover: float,
                      stepdown: float, feedMmPerMin: float,
                      numFlutes: int) -> StepoverResult:
        scallop = self.CalculateScallopHeight(toolDiameter, stepover)
        mrr = self.CalculateMRR(stepover, stepdown, feedMmPerMin, numFlutes)
        return StepoverResult(
            stepoverMm=stepover,
            scallopHeightMm=scallop,
            effectiveWidth=stepover,
            cuspHeightMm=scallop,
            materialRemovalRate=mrr,
        )


# ======================================================================
#  Tests
# ======================================================================

@pytest.fixture
def calc():
    return AdaptiveStepoverCalculator()


# 1. Scallop height for known geometry
class TestScallopHeight:

    def test_scallop_height_known_value(self, calc):
        """10 mm ball-end, 5 mm stepover -> known scallop height."""
        h = calc.CalculateScallopHeight(10.0, 5.0)
        R = 5.0
        expected = R - math.sqrt(R**2 - (5.0 / 2.0)**2)
        assert h == pytest.approx(expected, abs=1e-6)

    def test_scallop_height_zero_stepover(self, calc):
        """Zero stepover produces zero scallop."""
        assert calc.CalculateScallopHeight(10.0, 0.0) == 0.0

    def test_scallop_height_full_diameter(self, calc):
        """Stepover == diameter: scallop equals the radius (full engagement)."""
        h = calc.CalculateScallopHeight(10.0, 10.0)
        assert h == pytest.approx(5.0, abs=1e-6)


# 2. Inverse scallop (stepover from scallop)
class TestStepoverFromScallop:

    def test_roundtrip(self, calc):
        """Compute scallop from stepover, then recover stepover."""
        original_stepover = 3.0
        toolDia = 12.0
        h = calc.CalculateScallopHeight(toolDia, original_stepover)
        recovered = calc.CalculateStepoverFromScallop(toolDia, h)
        assert recovered == pytest.approx(original_stepover, abs=1e-5)

    def test_zero_scallop_returns_zero(self, calc):
        """Zero target scallop => zero stepover."""
        assert calc.CalculateStepoverFromScallop(10.0, 0.0) == 0.0


# 3. Optimal stepover respects operation ranges
class TestOptimalStepover:

    def test_finishing_clamp(self, calc):
        """Finishing stepover stays within 5-10% of tool diameter."""
        toolDia = 10.0
        result = calc.CalculateOptimalStepover(toolDia, 0.8, "finishing")
        assert 0.05 * toolDia <= result.stepoverMm <= 0.10 * toolDia + 1e-9

    def test_roughing_clamp(self, calc):
        """Roughing stepover stays within 50-75% of tool diameter."""
        toolDia = 10.0
        result = calc.CalculateOptimalStepover(toolDia, 20.0, "roughing")
        assert 0.50 * toolDia - 1e-9 <= result.stepoverMm <= 0.75 * toolDia + 1e-9

    def test_unknown_operation_raises(self, calc):
        with pytest.raises(ValueError, match="Unknown operation"):
            calc.CalculateOptimalStepover(10.0, 1.0, "magic")


# 4. Stepdown by material and operation
class TestStepdown:

    def test_aluminum_roughing(self, calc):
        """12 mm tool, aluminum, roughing -> 12 * 1.0 * 1.0 = 12 mm."""
        ap = calc.CalculateStepdown(12.0, "aluminum", "roughing")
        assert ap == pytest.approx(12.0, abs=1e-6)

    def test_steel_finishing(self, calc):
        """12 mm tool, steel, finishing -> 12 * 0.5 * 0.1 = 0.6 mm."""
        ap = calc.CalculateStepdown(12.0, "steel", "finishing")
        assert ap == pytest.approx(0.6, abs=1e-6)

    def test_titanium_semi_finishing(self, calc):
        """10 mm tool, titanium, semi_finishing -> 10 * 0.25 * 0.4 = 1.0 mm."""
        ap = calc.CalculateStepdown(10.0, "titanium", "semi_finishing")
        assert ap == pytest.approx(1.0, abs=1e-6)

    def test_unknown_material_uses_default(self, calc):
        """Unknown material falls back to 0.5 factor."""
        ap = calc.CalculateStepdown(10.0, "unobtanium", "roughing")
        assert ap == pytest.approx(10.0 * 0.5 * 1.0, abs=1e-6)


# 5. Engagement angle
class TestEngagementAngle:

    def test_half_diameter_stepover(self, calc):
        """ae = R => cos = 0 => 90 degrees."""
        angle = calc.GetEngagementAngle(10.0, 5.0)
        assert angle == pytest.approx(90.0, abs=1e-4)

    def test_full_diameter_stepover(self, calc):
        """ae = D => cos = -1 => 180 degrees."""
        angle = calc.GetEngagementAngle(10.0, 10.0)
        assert angle == pytest.approx(180.0, abs=1e-4)

    def test_zero_stepover(self, calc):
        """Zero stepover returns zero engagement."""
        assert calc.GetEngagementAngle(10.0, 0.0) == 0.0


# 6. Material removal rate
class TestMRR:

    def test_basic_mrr(self, calc):
        """ae=2, ap=3, vf=500 => MRR = 3000 mm^3/min."""
        mrr = calc.CalculateMRR(2.0, 3.0, 500.0, 4)
        assert mrr == pytest.approx(3000.0, abs=1e-6)

    def test_zero_feed_returns_zero(self, calc):
        assert calc.CalculateMRR(2.0, 3.0, 0.0, 4) == 0.0


# 7. Full result
class TestFullResult:

    def test_full_result_fields(self, calc):
        result = calc.GetFullResult(10.0, 3.0, 2.0, 600.0, 4)
        expected_scallop = calc.CalculateScallopHeight(10.0, 3.0)
        assert result.stepoverMm == pytest.approx(3.0)
        assert result.scallopHeightMm == pytest.approx(expected_scallop, abs=1e-6)
        assert result.materialRemovalRate == pytest.approx(3.0 * 2.0 * 600.0, abs=1e-6)


# 8. Edge cases / validation
class TestEdgeCases:

    def test_negative_tool_diameter_raises(self, calc):
        with pytest.raises(ValueError):
            calc.CalculateScallopHeight(-1.0, 1.0)

    def test_stepover_exceeds_diameter_raises(self, calc):
        with pytest.raises(ValueError):
            calc.CalculateScallopHeight(10.0, 11.0)

    def test_scallop_exceeds_radius_raises(self, calc):
        with pytest.raises(ValueError):
            calc.CalculateStepoverFromScallop(10.0, 6.0)

    def test_negative_diameter_stepdown_raises(self, calc):
        with pytest.raises(ValueError):
            calc.CalculateStepdown(-5.0, "aluminum", "roughing")
