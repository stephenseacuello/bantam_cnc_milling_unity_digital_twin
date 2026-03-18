"""Tests for Gauge R&R (Repeatability & Reproducibility) Analyzer.

Validates the ANOVA-based GRR analysis, variance components, NDC
calculation, and measurement system assessment per AIAG MSA guidelines.

Uses the same ROS2 mock pattern as test_capability_profiler.py.
"""

import sys
from unittest.mock import MagicMock

# Mock ROS2 and miracle_core sub-module dependencies before importing
for _mod in ['rclpy', 'rclpy.node', 'rclpy.callback_groups', 'rclpy.qos', 'rclpy.lifecycle',
             'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
             'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
             'miracle_core.exceptions',
             'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(_mod, MagicMock())

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

import math
import pytest

from miracle_scada.kpi_calculator import (
    GRRStudyData,
    GRRResult,
    GaugeRRAnalyzer,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _build_study(
    parts: list,
    operators: list,
    trials: int,
    measurements: dict,
) -> GRRStudyData:
    """Convenience wrapper to create a GRRStudyData instance."""
    return GRRStudyData(
        parts=parts,
        operators=operators,
        trials=trials,
        measurements=measurements,
    )


def _perfect_study() -> GRRStudyData:
    """Study where every measurement is identical -> zero GRR."""
    parts = ['P1', 'P2', 'P3']
    operators = ['Op1', 'Op2']
    trials = 3
    # Each part has a distinct true value, but all measurements match exactly
    true_values = {'P1': 10.0, 'P2': 20.0, 'P3': 30.0}
    meas = {}
    for p in parts:
        for o in operators:
            for t in range(1, trials + 1):
                meas[(p, o, t)] = true_values[p]
    return _build_study(parts, operators, trials, meas)


def _noisy_study() -> GRRStudyData:
    """Study with operator bias and equipment noise -> non-zero GRR."""
    parts = ['P1', 'P2', 'P3']
    operators = ['Op1', 'Op2']
    trials = 3
    # Base values per part
    base = {'P1': 10.0, 'P2': 20.0, 'P3': 30.0}
    # Operator bias
    op_bias = {'Op1': 0.0, 'Op2': 0.5}
    # Small trial-level noise patterns (deterministic for reproducibility)
    noise = {1: -0.1, 2: 0.05, 3: 0.05}
    meas = {}
    for p in parts:
        for o in operators:
            for t in range(1, trials + 1):
                meas[(p, o, t)] = base[p] + op_bias[o] + noise[t]
    return _build_study(parts, operators, trials, meas)


def _high_grr_study() -> GRRStudyData:
    """Study with large measurement variation relative to part variation."""
    parts = ['P1', 'P2']
    operators = ['Op1', 'Op2']
    trials = 2
    # Parts are close together but measurements scatter widely
    meas = {
        ('P1', 'Op1', 1): 10.0, ('P1', 'Op1', 2): 12.0,
        ('P1', 'Op2', 1): 9.0,  ('P1', 'Op2', 2): 13.0,
        ('P2', 'Op1', 1): 11.0, ('P2', 'Op1', 2): 13.0,
        ('P2', 'Op2', 1): 10.0, ('P2', 'Op2', 2): 14.0,
    }
    return _build_study(parts, operators, trials, meas)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestAssessMeasurementSystem:
    """Tests for the static assessment classifier."""

    def test_acceptable_below_10(self):
        assert GaugeRRAnalyzer.assess_measurement_system(5.0) == 'acceptable'

    def test_acceptable_at_zero(self):
        assert GaugeRRAnalyzer.assess_measurement_system(0.0) == 'acceptable'

    def test_marginal_at_10(self):
        assert GaugeRRAnalyzer.assess_measurement_system(10.0) == 'marginal'

    def test_marginal_at_30(self):
        assert GaugeRRAnalyzer.assess_measurement_system(30.0) == 'marginal'

    def test_unacceptable_above_30(self):
        assert GaugeRRAnalyzer.assess_measurement_system(31.0) == 'unacceptable'


class TestCalculateNDC:
    """Tests for Number of Distinct Categories calculation."""

    def test_ndc_basic(self):
        # 1.41 * 10.0 / 2.0 = 7.05 -> 7
        assert GaugeRRAnalyzer.calculate_ndc(10.0, 2.0) == 7

    def test_ndc_minimum_is_one(self):
        # Very small PV relative to GRR
        assert GaugeRRAnalyzer.calculate_ndc(0.1, 10.0) >= 1

    def test_ndc_zero_grr(self):
        # GRR = 0 -> should return 1 (avoid division by zero)
        assert GaugeRRAnalyzer.calculate_ndc(5.0, 0.0) == 1

    def test_ndc_floors(self):
        # 1.41 * 5.0 / 3.0 = 2.35 -> 2
        assert GaugeRRAnalyzer.calculate_ndc(5.0, 3.0) == 2


class TestGetVarianceComponents:
    """Tests for the ANOVA variance decomposition."""

    def test_perfect_study_zero_repeatability(self):
        """Perfect measurements should yield zero equipment variance."""
        analyzer = GaugeRRAnalyzer()
        var_rep, var_repro, var_part, var_total = analyzer.get_variance_components(
            _perfect_study()
        )
        assert var_rep == pytest.approx(0.0, abs=1e-12)
        assert var_repro == pytest.approx(0.0, abs=1e-12)
        assert var_part > 0.0  # parts differ

    def test_noisy_study_nonzero_components(self):
        """A study with noise should produce non-zero repeatability."""
        analyzer = GaugeRRAnalyzer()
        var_rep, var_repro, var_part, var_total = analyzer.get_variance_components(
            _noisy_study()
        )
        assert var_rep > 0.0
        # Total should be sum of components
        assert var_total == pytest.approx(var_rep + var_repro + var_part, rel=1e-9)

    def test_variance_components_sum_to_total(self):
        """Variance components must sum to total variance."""
        analyzer = GaugeRRAnalyzer()
        var_rep, var_repro, var_part, var_total = analyzer.get_variance_components(
            _high_grr_study()
        )
        assert var_total == pytest.approx(var_rep + var_repro + var_part, rel=1e-9)


class TestAnalyze:
    """Integration tests for the full analyze() pipeline."""

    def test_perfect_study_acceptable(self):
        """Perfect measurements -> zero GRR% -> acceptable."""
        analyzer = GaugeRRAnalyzer()
        result = analyzer.analyze(_perfect_study())
        assert result.grr_pct == pytest.approx(0.0, abs=1e-9)
        assert result.acceptable is True
        assert result.assessment == 'acceptable'

    def test_noisy_study_result_fields(self):
        """Noisy study should populate all result fields correctly."""
        analyzer = GaugeRRAnalyzer()
        result = analyzer.analyze(_noisy_study())
        # GRR = sqrt(EV^2 + AV^2)
        expected_grr = math.sqrt(result.repeatability ** 2 + result.reproducibility ** 2)
        assert result.grr == pytest.approx(expected_grr, rel=1e-9)
        # TV = sqrt(GRR^2 + PV^2)
        expected_tv = math.sqrt(result.grr ** 2 + result.part_variation ** 2)
        assert result.total_variation == pytest.approx(expected_tv, rel=1e-9)
        # GRR% = GRR / TV * 100
        expected_pct = result.grr / result.total_variation * 100.0
        assert result.grr_pct == pytest.approx(expected_pct, rel=1e-9)
        # NDC must be >= 1
        assert result.ndc >= 1

    def test_high_grr_study_unacceptable(self):
        """A study dominated by measurement error should be unacceptable."""
        analyzer = GaugeRRAnalyzer()
        result = analyzer.analyze(_high_grr_study())
        # The high-noise study should produce a large GRR%
        assert result.grr_pct > 30.0
        assert result.acceptable is False
        assert result.assessment == 'unacceptable'

    def test_grr_result_dataclass_fields(self):
        """GRRResult should have all required fields."""
        analyzer = GaugeRRAnalyzer()
        result = analyzer.analyze(_noisy_study())
        for attr in [
            'repeatability', 'reproducibility', 'grr',
            'part_variation', 'total_variation', 'grr_pct',
            'ndc', 'acceptable', 'assessment',
        ]:
            assert hasattr(result, attr), f"Missing field: {attr}"

    def test_grr_study_data_dataclass_fields(self):
        """GRRStudyData should have all required fields."""
        study = _noisy_study()
        for attr in ['parts', 'operators', 'trials', 'measurements']:
            assert hasattr(study, attr), f"Missing field: {attr}"

    def test_single_operator_zero_reproducibility(self):
        """With only one operator reproducibility variance should be zero."""
        parts = ['P1', 'P2', 'P3']
        operators = ['Op1']
        trials = 3
        base = {'P1': 10.0, 'P2': 20.0, 'P3': 30.0}
        noise = {1: -0.05, 2: 0.03, 3: 0.02}
        meas = {}
        for p in parts:
            for o in operators:
                for t in range(1, trials + 1):
                    meas[(p, o, t)] = base[p] + noise[t]
        study = _build_study(parts, operators, trials, meas)
        analyzer = GaugeRRAnalyzer()
        result = analyzer.analyze(study)
        # With one operator, reproducibility should be zero
        assert result.reproducibility == pytest.approx(0.0, abs=1e-9)
