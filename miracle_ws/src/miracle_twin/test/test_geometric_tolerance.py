"""Tests for GeometricToleranceAnalyzer — GD&T analysis for cutting simulation.

Mirrors the Unity-side GeometricToleranceAnalyzer logic so that the Python
ROS 2 twin can validate tolerance predictions independently.
"""
import sys
import types
import math
import pytest

# ── ROS 2 / Unity shim mocking (standard pattern) ──────────────────
# Mock modules that are unavailable outside the ROS 2 workspace.
for mod_name in (
    "rclpy", "rclpy.node", "rclpy.qos",
    "std_msgs", "std_msgs.msg",
    "geometry_msgs", "geometry_msgs.msg",
    "sensor_msgs", "sensor_msgs.msg",
    "miracle_interfaces", "miracle_interfaces.msg", "miracle_interfaces.srv",
):
    sys.modules.setdefault(mod_name, types.ModuleType(mod_name))


# ── Portable re-implementation of the analyzer (matches C# logic) ──

class ToleranceType:
    POSITION = "POSITION"
    FLATNESS = "FLATNESS"
    PARALLELISM = "PARALLELISM"
    PERPENDICULARITY = "PERPENDICULARITY"
    CONCENTRICITY = "CONCENTRICITY"
    CIRCULARITY = "CIRCULARITY"
    CYLINDRICITY = "CYLINDRICITY"
    PROFILE = "PROFILE"
    RUNOUT = "RUNOUT"
    TOTAL_RUNOUT = "TOTAL_RUNOUT"


class ToleranceSpec:
    def __init__(
        self,
        tolerance_type="POSITION",
        feature_id="",
        nominal_value=0.0,
        tolerance_zone=0.0,
        datum_reference="",
        material_condition="RFS",
    ):
        self.tolerance_type = tolerance_type
        self.feature_id = feature_id
        self.nominal_value = nominal_value
        self.tolerance_zone = tolerance_zone
        self.datum_reference = datum_reference
        self.material_condition = material_condition


class ToleranceResult:
    def __init__(self):
        self.spec = None
        self.actual_deviation = 0.0
        self.is_in_tolerance = True
        self.percent_of_tolerance = 0.0
        self.risk_level = "LOW"
        self.contributing_factors = []
        self.predicted_drift_per_hour = 0.0


class GeometricToleranceAnalyzer:
    ALUMINUM_CTE = 11.7e-6
    REFERENCE_TEMP = 20.0
    DEFAULT_FEATURE_LENGTH = 50.0

    def __init__(self):
        self.specs = []

    def add_spec(self, spec):
        if spec is None:
            raise ValueError("spec must not be None")
        self.specs.append(spec)

    def analyze_feature(self, feature_id, tool_deflection, thermal_expansion, wear_compensation):
        spec = next((s for s in self.specs if s.feature_id == feature_id), None)
        if spec is None:
            raise ValueError(f"No spec found for feature '{feature_id}'")

        total_deviation = tool_deflection + thermal_expansion - wear_compensation
        abs_deviation = abs(total_deviation)

        half_zone = spec.tolerance_zone / 2.0
        if half_zone > 0:
            percent = (abs_deviation / half_zone) * 100.0
        else:
            percent = float("inf") if abs_deviation > 0 else 0.0

        if percent < 50:
            risk = "LOW"
        elif percent < 80:
            risk = "MEDIUM"
        elif percent < 100:
            risk = "HIGH"
        else:
            risk = "OUT_OF_SPEC"

        factors = []
        if abs(tool_deflection) > 0.001:
            factors.append(f"tool_deflection:{tool_deflection:.4f}mm")
        if abs(thermal_expansion) > 0.001:
            factors.append(f"thermal_expansion:{thermal_expansion:.4f}mm")
        if abs(wear_compensation) > 0.001:
            factors.append(f"wear_compensation:{wear_compensation:.4f}mm")

        result = ToleranceResult()
        result.spec = spec
        result.actual_deviation = total_deviation
        result.is_in_tolerance = percent <= 100.0
        result.percent_of_tolerance = percent
        result.risk_level = risk
        result.contributing_factors = factors
        result.predicted_drift_per_hour = 0.0
        return result

    def predict_tolerance_risk(self, current_temp, tool_wear_mm, spindle_runout):
        results = []
        delta_t = current_temp - self.REFERENCE_TEMP

        for spec in self.specs:
            feature_length = spec.nominal_value if spec.nominal_value > 0 else self.DEFAULT_FEATURE_LENGTH
            thermal_expansion = self.ALUMINUM_CTE * delta_t * feature_length
            tool_deflection = spindle_runout + tool_wear_mm * 0.1
            wear_compensation = 0.0

            result = self.analyze_feature(spec.feature_id, tool_deflection, thermal_expansion, wear_compensation)
            drift = abs(self.ALUMINUM_CTE * 2.0 * feature_length) + tool_wear_mm * 0.05
            result.predicted_drift_per_hour = drift
            results.append(result)

        return results

    def get_critical_features(self):
        results = self.predict_tolerance_risk(self.REFERENCE_TEMP, 0.0, 0.0)
        return [r for r in results if r.percent_of_tolerance >= 70.0]

    def estimate_compensation(self, feature_id):
        spec = next((s for s in self.specs if s.feature_id == feature_id), None)
        if spec is None:
            raise ValueError(f"No spec found for feature '{feature_id}'")
        feature_length = spec.nominal_value if spec.nominal_value > 0 else self.DEFAULT_FEATURE_LENGTH
        estimated_thermal = self.ALUMINUM_CTE * 5.0 * feature_length
        return estimated_thermal / 2.0


# ── Fixtures ────────────────────────────────────────────────────────

@pytest.fixture
def analyzer():
    return GeometricToleranceAnalyzer()


@pytest.fixture
def basic_spec():
    return ToleranceSpec(
        tolerance_type=ToleranceType.POSITION,
        feature_id="BORE_01",
        nominal_value=25.4,
        tolerance_zone=0.05,
        datum_reference="A",
        material_condition="MMC",
    )


@pytest.fixture
def analyzer_with_spec(analyzer, basic_spec):
    analyzer.add_spec(basic_spec)
    return analyzer


# ── Tests: In-tolerance analysis ────────────────────────────────────

class TestInToleranceAnalysis:
    def test_zero_deviation_is_in_tolerance(self, analyzer_with_spec):
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.0, 0.0, 0.0)
        assert result.is_in_tolerance is True
        assert result.percent_of_tolerance == 0.0
        assert result.risk_level == "LOW"

    def test_small_deflection_stays_in_tolerance(self, analyzer_with_spec):
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.005, 0.0, 0.0)
        assert result.is_in_tolerance is True
        assert result.percent_of_tolerance == pytest.approx(20.0, abs=0.5)

    def test_wear_compensation_reduces_deviation(self, analyzer_with_spec):
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.020, 0.0, 0.010)
        expected_dev = 0.010
        assert result.actual_deviation == pytest.approx(expected_dev, abs=1e-6)
        assert result.is_in_tolerance is True


# ── Tests: Out-of-tolerance detection ───────────────────────────────

class TestOutOfToleranceDetection:
    def test_large_deflection_exceeds_tolerance(self, analyzer_with_spec):
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.030, 0.0, 0.0)
        assert result.is_in_tolerance is False
        assert result.risk_level == "OUT_OF_SPEC"

    def test_combined_sources_exceed_tolerance(self, analyzer_with_spec):
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.015, 0.015, 0.0)
        assert result.is_in_tolerance is False
        assert result.percent_of_tolerance > 100.0

    def test_negative_deviation_out_of_tolerance(self, analyzer_with_spec):
        # Large wear compensation with no deflection → negative deviation
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.0, 0.0, 0.030)
        assert result.is_in_tolerance is False


# ── Tests: Risk level classification ────────────────────────────────

class TestRiskLevelClassification:
    def test_low_risk_below_50_percent(self, analyzer_with_spec):
        # 0.010 / 0.025 = 40%
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.010, 0.0, 0.0)
        assert result.risk_level == "LOW"

    def test_medium_risk_between_50_and_80(self, analyzer_with_spec):
        # 0.016 / 0.025 = 64%
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.016, 0.0, 0.0)
        assert result.risk_level == "MEDIUM"

    def test_high_risk_between_80_and_100(self, analyzer_with_spec):
        # 0.022 / 0.025 = 88%
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.022, 0.0, 0.0)
        assert result.risk_level == "HIGH"

    def test_out_of_spec_at_100_percent(self, analyzer_with_spec):
        # exactly at boundary: 0.025 / 0.025 = 100%
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.025, 0.0, 0.0)
        assert result.risk_level == "OUT_OF_SPEC"


# ── Tests: Thermal expansion contribution ──────────────────────────

class TestThermalExpansion:
    def test_thermal_expansion_increases_deviation(self, analyzer_with_spec):
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.0, 0.010, 0.0)
        assert result.actual_deviation == pytest.approx(0.010, abs=1e-6)
        assert result.is_in_tolerance is True

    def test_predicted_thermal_at_elevated_temp(self, analyzer_with_spec):
        results = analyzer_with_spec.predict_tolerance_risk(50.0, 0.0, 0.0)
        assert len(results) == 1
        # deltaT=30, length=25.4 → thermal = 11.7e-6 * 30 * 25.4 ≈ 0.00892 mm
        expected_thermal = 11.7e-6 * 30.0 * 25.4
        assert results[0].actual_deviation == pytest.approx(expected_thermal, rel=0.1)

    def test_no_thermal_at_reference_temp(self, analyzer_with_spec):
        results = analyzer_with_spec.predict_tolerance_risk(20.0, 0.0, 0.0)
        assert results[0].actual_deviation == pytest.approx(0.0, abs=1e-9)


# ── Tests: Tool deflection contribution ────────────────────────────

class TestToolDeflection:
    def test_deflection_appears_in_factors(self, analyzer_with_spec):
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.015, 0.0, 0.0)
        assert any("tool_deflection" in f for f in result.contributing_factors)

    def test_spindle_runout_contributes_to_deflection(self, analyzer_with_spec):
        results = analyzer_with_spec.predict_tolerance_risk(20.0, 0.0, 0.005)
        assert results[0].actual_deviation == pytest.approx(0.005, abs=1e-6)


# ── Tests: Combined deviation sources ──────────────────────────────

class TestCombinedDeviation:
    def test_all_sources_combined(self, analyzer_with_spec):
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.010, 0.008, 0.005)
        expected = 0.010 + 0.008 - 0.005
        assert result.actual_deviation == pytest.approx(expected, abs=1e-6)

    def test_compensation_can_zero_out_deviation(self, analyzer_with_spec):
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.010, 0.005, 0.015)
        assert result.actual_deviation == pytest.approx(0.0, abs=1e-6)
        assert result.risk_level == "LOW"


# ── Tests: Critical feature identification ─────────────────────────

class TestCriticalFeatures:
    def test_no_critical_at_ambient(self, analyzer_with_spec):
        critical = analyzer_with_spec.get_critical_features()
        assert len(critical) == 0

    def test_tight_tolerance_becomes_critical(self, analyzer):
        spec = ToleranceSpec(
            tolerance_type=ToleranceType.FLATNESS,
            feature_id="FACE_01",
            nominal_value=100.0,
            tolerance_zone=0.001,
            datum_reference="A",
            material_condition="RFS",
        )
        analyzer.add_spec(spec)
        # Even at reference temp with zero inputs, we expect zero deviation
        # so it should NOT be critical.  But if we call predict with elevated values...
        results = analyzer.predict_tolerance_risk(25.0, 0.1, 0.005)
        critical = [r for r in results if r.percent_of_tolerance >= 70.0]
        assert len(critical) >= 1
        assert critical[0].spec.feature_id == "FACE_01"


# ── Tests: Compensation estimation ─────────────────────────────────

class TestCompensation:
    def test_compensation_positive(self, analyzer_with_spec):
        comp = analyzer_with_spec.estimate_compensation("BORE_01")
        assert comp > 0.0

    def test_compensation_scales_with_feature_length(self, analyzer):
        short = ToleranceSpec(feature_id="SHORT", nominal_value=10.0, tolerance_zone=0.05)
        long_ = ToleranceSpec(feature_id="LONG", nominal_value=200.0, tolerance_zone=0.05)
        analyzer.add_spec(short)
        analyzer.add_spec(long_)
        comp_short = analyzer.estimate_compensation("SHORT")
        comp_long = analyzer.estimate_compensation("LONG")
        assert comp_long > comp_short

    def test_compensation_unknown_feature_raises(self, analyzer):
        with pytest.raises(ValueError, match="No spec found"):
            analyzer.estimate_compensation("NONEXISTENT")


# ── Tests: Multiple specs analysis ─────────────────────────────────

class TestMultipleSpecs:
    def test_predict_returns_result_per_spec(self, analyzer):
        for i in range(5):
            analyzer.add_spec(ToleranceSpec(
                feature_id=f"FEAT_{i:02d}",
                nominal_value=25.0 + i * 10,
                tolerance_zone=0.05,
            ))
        results = analyzer.predict_tolerance_risk(30.0, 0.05, 0.003)
        assert len(results) == 5

    def test_each_result_has_drift_estimate(self, analyzer):
        analyzer.add_spec(ToleranceSpec(feature_id="A", nominal_value=50.0, tolerance_zone=0.1))
        analyzer.add_spec(ToleranceSpec(feature_id="B", nominal_value=100.0, tolerance_zone=0.1))
        results = analyzer.predict_tolerance_risk(35.0, 0.1, 0.002)
        for r in results:
            assert r.predicted_drift_per_hour > 0.0


# ── Tests: Edge cases ──────────────────────────────────────────────

class TestEdgeCases:
    def test_zero_tolerance_zone_with_zero_deviation(self, analyzer):
        spec = ToleranceSpec(feature_id="ZERO_TOL", nominal_value=10.0, tolerance_zone=0.0)
        analyzer.add_spec(spec)
        result = analyzer.analyze_feature("ZERO_TOL", 0.0, 0.0, 0.0)
        assert result.percent_of_tolerance == 0.0
        assert result.is_in_tolerance is True

    def test_zero_tolerance_zone_with_nonzero_deviation(self, analyzer):
        spec = ToleranceSpec(feature_id="ZERO_TOL2", nominal_value=10.0, tolerance_zone=0.0)
        analyzer.add_spec(spec)
        result = analyzer.analyze_feature("ZERO_TOL2", 0.001, 0.001, 0.0)
        assert result.is_in_tolerance is False
        assert result.percent_of_tolerance == float("inf")

    def test_no_specs_predict_returns_empty(self, analyzer):
        results = analyzer.predict_tolerance_risk(30.0, 0.1, 0.01)
        assert results == []

    def test_unknown_feature_raises(self, analyzer):
        with pytest.raises(ValueError, match="No spec found"):
            analyzer.analyze_feature("MISSING", 0.0, 0.0, 0.0)

    def test_add_none_spec_raises(self, analyzer):
        with pytest.raises(ValueError):
            analyzer.add_spec(None)


# ── Tests: Percent of tolerance calculation ────────────────────────

class TestPercentOfTolerance:
    def test_exactly_half_zone(self, analyzer_with_spec):
        # half_zone = 0.025, deviation = 0.025 → 100%
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.025, 0.0, 0.0)
        assert result.percent_of_tolerance == pytest.approx(100.0, abs=0.1)

    def test_quarter_zone(self, analyzer_with_spec):
        # deviation = 0.0125 → 50%
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.0125, 0.0, 0.0)
        assert result.percent_of_tolerance == pytest.approx(50.0, abs=0.1)

    def test_double_zone(self, analyzer_with_spec):
        # deviation = 0.050 → 200%
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.050, 0.0, 0.0)
        assert result.percent_of_tolerance == pytest.approx(200.0, abs=0.1)


# ── Tests: Contributing factors listing ────────────────────────────

class TestContributingFactors:
    def test_no_factors_at_zero(self, analyzer_with_spec):
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.0, 0.0, 0.0)
        assert len(result.contributing_factors) == 0

    def test_all_three_factors(self, analyzer_with_spec):
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.005, 0.005, 0.003)
        assert len(result.contributing_factors) == 3
        labels = [f.split(":")[0] for f in result.contributing_factors]
        assert "tool_deflection" in labels
        assert "thermal_expansion" in labels
        assert "wear_compensation" in labels

    def test_factors_below_threshold_omitted(self, analyzer_with_spec):
        result = analyzer_with_spec.analyze_feature("BORE_01", 0.0005, 0.0, 0.0)
        assert len(result.contributing_factors) == 0


# ── Tests: Tolerance type coverage ─────────────────────────────────

class TestToleranceTypes:
    @pytest.mark.parametrize("ttype", [
        ToleranceType.POSITION,
        ToleranceType.FLATNESS,
        ToleranceType.PARALLELISM,
        ToleranceType.PERPENDICULARITY,
        ToleranceType.CONCENTRICITY,
        ToleranceType.CIRCULARITY,
        ToleranceType.CYLINDRICITY,
        ToleranceType.PROFILE,
        ToleranceType.RUNOUT,
        ToleranceType.TOTAL_RUNOUT,
    ])
    def test_all_tolerance_types_analyzable(self, analyzer, ttype):
        spec = ToleranceSpec(
            tolerance_type=ttype,
            feature_id=f"feat_{ttype}",
            nominal_value=30.0,
            tolerance_zone=0.05,
        )
        analyzer.add_spec(spec)
        result = analyzer.analyze_feature(f"feat_{ttype}", 0.010, 0.0, 0.0)
        assert result.is_in_tolerance is True
        assert result.spec.tolerance_type == ttype
