"""Tests for CausalImpactAnalyzer in the reasoning engine.

Covers recording, correlation computation, impact analysis, reporting,
most-impactful-change retrieval, edge cases, and direction classification.
"""

import sys
from unittest.mock import MagicMock

for mod in [
    'miracle_core.datatypes', 'miracle_core.constants',
    'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
    'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
    'miracle_msgs', 'miracle_msgs.msg',
]:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_cognitive.knowledge.reasoning_engine import (
    CausalImpactAnalyzer,
    ImpactReport,
    ImpactResult,
    OutcomeObservation,
    ParameterChange,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def analyzer():
    """Return a fresh CausalImpactAnalyzer instance."""
    return CausalImpactAnalyzer()


def _make_change(name='feed_rate', old=100.0, new=120.0, ts=0.0,
                 ctype='manual'):
    return ParameterChange(
        parameter_name=name,
        old_value=old,
        new_value=new,
        timestamp=ts,
        change_type=ctype,
    )


def _make_outcome(name='surface_roughness', value=1.0, ts=0.0):
    return OutcomeObservation(metric_name=name, value=value, timestamp=ts)


# ---------------------------------------------------------------------------
# 1. Recording changes and outcomes
# ---------------------------------------------------------------------------

class TestRecording:
    def test_record_change_stores_entry(self, analyzer):
        change = _make_change()
        analyzer.record_change(change)
        assert len(analyzer._changes) == 1
        assert analyzer._changes[0] is change

    def test_record_outcome_stores_entry(self, analyzer):
        obs = _make_outcome()
        analyzer.record_outcome(obs)
        assert len(analyzer._outcomes) == 1
        assert analyzer._outcomes[0] is obs


# ---------------------------------------------------------------------------
# 2. ParameterChange dataclass helpers
# ---------------------------------------------------------------------------

class TestParameterChangeHelpers:
    def test_magnitude(self):
        c = _make_change(old=100.0, new=80.0)
        assert c.magnitude == 20.0

    def test_direction_sign_positive(self):
        c = _make_change(old=100.0, new=120.0)
        assert c.direction_sign == 1.0

    def test_direction_sign_negative(self):
        c = _make_change(old=120.0, new=100.0)
        assert c.direction_sign == -1.0

    def test_direction_sign_zero(self):
        c = _make_change(old=100.0, new=100.0)
        assert c.direction_sign == 0.0


# ---------------------------------------------------------------------------
# 3. Pearson correlation helper
# ---------------------------------------------------------------------------

class TestPearsonCorrelation:
    def test_perfect_positive(self):
        r = CausalImpactAnalyzer._pearson([1, 2, 3], [10, 20, 30])
        assert abs(r - 1.0) < 1e-9

    def test_perfect_negative(self):
        r = CausalImpactAnalyzer._pearson([1, 2, 3], [30, 20, 10])
        assert abs(r - (-1.0)) < 1e-9

    def test_zero_variance_returns_zero(self):
        r = CausalImpactAnalyzer._pearson([5, 5, 5], [1, 2, 3])
        assert r == 0.0

    def test_too_few_values(self):
        assert CausalImpactAnalyzer._pearson([1], [2]) == 0.0
        assert CausalImpactAnalyzer._pearson([], []) == 0.0


# ---------------------------------------------------------------------------
# 4. analyze_impact
# ---------------------------------------------------------------------------

class TestAnalyzeImpact:
    def test_positive_correlation(self, analyzer):
        """Larger parameter increases should correlate with larger roughness deltas."""
        for i in range(5):
            ts = float(i * 100)
            jump = (i + 1) * 10.0  # varying magnitudes: 10, 20, 30, 40, 50
            analyzer.record_change(_make_change(
                name='feed_rate', old=100.0,
                new=100.0 + jump, ts=ts, ctype='manual',
            ))
            # Outcome delta proportional to the jump
            analyzer.record_outcome(_make_outcome(
                name='roughness', value=1.0, ts=ts + 1,
            ))
            analyzer.record_outcome(_make_outcome(
                name='roughness', value=1.0 + jump * 0.05, ts=ts + 5,
            ))

        result = analyzer.analyze_impact('feed_rate', 'roughness', window_sec=50.0)

        assert isinstance(result, ImpactResult)
        assert result.parameter_name == 'feed_rate'
        assert result.metric_name == 'roughness'
        assert result.direction == 'positive'
        assert result.confidence > 0.0
        assert result.impact_magnitude > 0.0

    def test_no_matching_data_returns_zero(self, analyzer):
        """No outcome observations means zero correlation and confidence."""
        analyzer.record_change(_make_change(ts=0.0))
        result = analyzer.analyze_impact('feed_rate', 'roughness', window_sec=10.0)

        assert result.correlation == 0.0
        assert result.confidence == 0.0
        assert result.impact_magnitude == 0.0
        assert result.direction == 'neutral'

    def test_negative_correlation(self, analyzer):
        """Larger param increases should correlate with larger metric drops."""
        for i in range(5):
            ts = float(i * 100)
            jump = (i + 1) * 5.0  # varying: 5, 10, 15, 20, 25
            analyzer.record_change(_make_change(
                name='coolant_flow', old=10.0, new=10.0 + jump,
                ts=ts, ctype='automatic',
            ))
            # Temperature drops proportionally to jump magnitude
            analyzer.record_outcome(_make_outcome(
                name='temperature', value=80.0, ts=ts + 1,
            ))
            analyzer.record_outcome(_make_outcome(
                name='temperature', value=80.0 - jump * 0.3, ts=ts + 5,
            ))

        result = analyzer.analyze_impact('coolant_flow', 'temperature', window_sec=50.0)
        assert result.direction == 'negative'
        assert result.correlation < -0.1


# ---------------------------------------------------------------------------
# 5. get_impact_report
# ---------------------------------------------------------------------------

class TestImpactReport:
    def test_report_structure(self, analyzer):
        analyzer.record_change(_make_change(name='speed', ts=0.0))
        analyzer.record_outcome(_make_outcome(name='quality', value=5.0, ts=0.5))
        analyzer.record_outcome(_make_outcome(name='quality', value=5.5, ts=1.5))

        report = analyzer.get_impact_report(window_sec=10.0)

        assert isinstance(report, ImpactReport)
        assert len(report.changes) == 1
        assert len(report.impacts) >= 1
        assert isinstance(report.summary, str)
        assert report.timestamp > 0

    def test_report_empty_analyzer(self, analyzer):
        report = analyzer.get_impact_report()
        assert report.impacts == []
        assert 'insufficient' in report.summary.lower() or 'No impacts' in report.summary


# ---------------------------------------------------------------------------
# 6. get_most_impactful_change
# ---------------------------------------------------------------------------

class TestMostImpactfulChange:
    def test_returns_none_when_empty(self, analyzer):
        assert analyzer.get_most_impactful_change() is None

    def test_selects_highest_correlation(self, analyzer):
        # Use two separate metrics so data doesn't cross-contaminate
        for i in range(5):
            ts = float(i * 1000)
            jump = (i + 1) * 10.0

            # Strong correlation: feed_rate -> roughness
            analyzer.record_change(_make_change(
                name='feed_rate', old=100.0, new=100.0 + jump,
                ts=ts, ctype='manual',
            ))
            analyzer.record_outcome(_make_outcome(
                name='roughness', value=1.0, ts=ts + 1,
            ))
            analyzer.record_outcome(_make_outcome(
                name='roughness', value=1.0 + jump * 0.05, ts=ts + 5,
            ))

            # Zero correlation: spindle_speed -> vibration (constant delta)
            analyzer.record_change(_make_change(
                name='spindle_speed', old=3000.0, new=3000.0 + jump,
                ts=ts + 500, ctype='automatic',
            ))
            analyzer.record_outcome(_make_outcome(
                name='vibration', value=3.0, ts=ts + 501,
            ))
            analyzer.record_outcome(_make_outcome(
                name='vibration', value=3.0, ts=ts + 505,
            ))

        best = analyzer.get_most_impactful_change()

        assert best is not None
        assert isinstance(best, ImpactResult)
        # The strong-correlation parameter should be selected
        assert best.parameter_name == 'feed_rate'


# ---------------------------------------------------------------------------
# 7. Lag computation
# ---------------------------------------------------------------------------

class TestLagComputation:
    def test_lag_reflects_observation_timing(self, analyzer):
        analyzer.record_change(_make_change(name='depth', ts=10.0))
        # Outcomes arrive 5 and 8 seconds after the change
        analyzer.record_outcome(_make_outcome(name='force', value=50.0, ts=15.0))
        analyzer.record_outcome(_make_outcome(name='force', value=60.0, ts=18.0))

        result = analyzer.analyze_impact('depth', 'force', window_sec=20.0)
        # Average observation time is (15+18)/2 = 16.5; lag = 16.5 - 10 = 6.5
        assert abs(result.lag_sec - 6.5) < 1e-6


# ---------------------------------------------------------------------------
# 8. Multiple change types
# ---------------------------------------------------------------------------

class TestChangeTypes:
    def test_all_change_types_accepted(self, analyzer):
        for ctype in ('manual', 'automatic', 'drift'):
            analyzer.record_change(_make_change(
                name='param', old=1.0, new=2.0, ts=0.0, ctype=ctype,
            ))
        assert len(analyzer._changes) == 3
        types = {c.change_type for c in analyzer._changes}
        assert types == {'manual', 'automatic', 'drift'}
