"""Tests for the AnomalyScoringModel in explanation_generator.py."""

import math
import sys
import os
import pytest
from unittest.mock import MagicMock

# Mock ROS2 and MIRACLE dependencies before import
for mod in ['miracle_core.datatypes', 'miracle_core.constants',
            'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
            'rclpy.callback_groups',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

# Ensure the base class has the CRITICALITY attrs
_base = sys.modules['miracle_core.lifecycle_node_base']
if not hasattr(_base.MiracleLifecycleNode, 'CRITICALITY_LOW') or \
   isinstance(getattr(_base.MiracleLifecycleNode, 'CRITICALITY_LOW', None), MagicMock):
    _base.MiracleLifecycleNode.CRITICALITY_LOW = 'LOW'
    _base.MiracleLifecycleNode.CRITICALITY_HIGH = 'HIGH'
    _base.MiracleLifecycleNode.CRITICALITY_CRITICAL = 'CRITICAL'

sys.modules.pop('miracle_cognitive.interface.explanation_generator', None)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_cognitive.interface.explanation_generator import (
    AnomalyScore,
    AnomalyReport,
    AnomalyScoringModel,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_trained_model():
    """Return a model trained on a simple 3-feature dataset."""
    model = AnomalyScoringModel()
    model.train({
        'temperature': [100.0, 102.0, 98.0, 101.0, 99.0, 100.5, 101.5, 99.5, 100.0, 100.0],
        'vibration':   [0.5, 0.6, 0.4, 0.55, 0.45, 0.5, 0.52, 0.48, 0.5, 0.5],
        'pressure':    [50.0, 51.0, 49.0, 50.5, 49.5, 50.0, 50.2, 49.8, 50.0, 50.0],
    })
    return model


# ---------------------------------------------------------------------------
# Test 1 — Training populates feature statistics
# ---------------------------------------------------------------------------

class TestTraining:
    def test_train_stores_stats(self):
        model = _make_trained_model()
        stats = model.get_feature_stats()

        assert 'temperature' in stats
        assert 'vibration' in stats
        assert 'pressure' in stats

        t = stats['temperature']
        assert t['count'] == 10
        assert 99.0 <= t['mean'] <= 101.0
        assert t['std'] > 0
        assert t['min'] == 98.0
        assert t['max'] == 102.0

    def test_train_empty_data_raises(self):
        model = AnomalyScoringModel()
        with pytest.raises(ValueError, match='empty'):
            model.train({})

    def test_train_insufficient_data_raises(self):
        model = AnomalyScoringModel()
        with pytest.raises(ValueError, match='at least 2'):
            model.train({'feat': [1.0]})


# ---------------------------------------------------------------------------
# Test 2 — Scoring a normal observation
# ---------------------------------------------------------------------------

class TestScoreNormal:
    def test_normal_observation_not_anomalous(self):
        model = _make_trained_model()
        report = model.score({
            'temperature': 100.0,
            'vibration': 0.5,
            'pressure': 50.0,
        })

        assert isinstance(report, AnomalyReport)
        assert report.overall_score < 25.0
        assert report.is_anomalous is False
        assert len(report.top_anomalies) == 0
        assert 'normal' in report.explanation.lower()


# ---------------------------------------------------------------------------
# Test 3 — Scoring a severe anomaly
# ---------------------------------------------------------------------------

class TestScoreSevereAnomaly:
    def test_severe_anomaly_detected(self):
        model = _make_trained_model()
        stats = model.get_feature_stats()
        # Push temperature way beyond 3 sigma
        extreme_temp = stats['temperature']['mean'] + 5 * stats['temperature']['std']

        report = model.score({
            'temperature': extreme_temp,
            'vibration': 0.5,
            'pressure': 50.0,
        })

        assert report.is_anomalous is True
        assert report.overall_score > 0.0

        temp_scores = [s for s in report.anomaly_scores if s.feature_name == 'temperature']
        assert len(temp_scores) == 1
        assert temp_scores[0].severity == 'severe'
        assert temp_scores[0].is_anomaly is True
        assert abs(temp_scores[0].z_score) > 3.0


# ---------------------------------------------------------------------------
# Test 4 — Severity classification thresholds
# ---------------------------------------------------------------------------

class TestSeverityClassification:
    def test_severity_levels(self):
        model = _make_trained_model()
        stats = model.get_feature_stats()
        mean = stats['temperature']['mean']
        std = stats['temperature']['std']

        # mild: |z| between 1.5 and 2.0
        report_mild = model.score({
            'temperature': mean + 1.7 * std,
            'vibration': 0.5,
            'pressure': 50.0,
        })
        temp_mild = [s for s in report_mild.anomaly_scores if s.feature_name == 'temperature'][0]
        assert temp_mild.severity == 'mild'

        # moderate: |z| between 2.0 and 3.0
        report_mod = model.score({
            'temperature': mean + 2.5 * std,
            'vibration': 0.5,
            'pressure': 50.0,
        })
        temp_mod = [s for s in report_mod.anomaly_scores if s.feature_name == 'temperature'][0]
        assert temp_mod.severity == 'moderate'

        # severe: |z| > 3.0
        report_sev = model.score({
            'temperature': mean + 3.5 * std,
            'vibration': 0.5,
            'pressure': 50.0,
        })
        temp_sev = [s for s in report_sev.anomaly_scores if s.feature_name == 'temperature'][0]
        assert temp_sev.severity == 'severe'


# ---------------------------------------------------------------------------
# Test 5 — Batch scoring
# ---------------------------------------------------------------------------

class TestScoreBatch:
    def test_batch_returns_correct_count(self):
        model = _make_trained_model()
        observations = [
            {'temperature': 100.0, 'vibration': 0.5, 'pressure': 50.0},
            {'temperature': 120.0, 'vibration': 0.5, 'pressure': 50.0},
            {'temperature': 100.0, 'vibration': 2.0, 'pressure': 50.0},
        ]
        reports = model.score_batch(observations)
        assert len(reports) == 3
        assert all(isinstance(r, AnomalyReport) for r in reports)

        # First should be normal, rest anomalous
        assert reports[0].is_anomalous is False
        assert reports[1].is_anomalous is True
        assert reports[2].is_anomalous is True


# ---------------------------------------------------------------------------
# Test 6 — Feature bounds enforcement
# ---------------------------------------------------------------------------

class TestFeatureBounds:
    def test_bounds_flag_anomaly(self):
        model = _make_trained_model()
        model.add_feature_bounds('pressure', 45.0, 55.0)

        # Within bounds — should be normal
        report_ok = model.score({
            'temperature': 100.0,
            'vibration': 0.5,
            'pressure': 50.0,
        })
        pres_ok = [s for s in report_ok.anomaly_scores if s.feature_name == 'pressure'][0]
        assert pres_ok.is_anomaly is False

        # Outside bounds — should be flagged
        report_bad = model.score({
            'temperature': 100.0,
            'vibration': 0.5,
            'pressure': 60.0,
        })
        pres_bad = [s for s in report_bad.anomaly_scores if s.feature_name == 'pressure'][0]
        assert pres_bad.is_anomaly is True
        assert pres_bad.severity == 'severe'

    def test_invalid_bounds_raises(self):
        model = AnomalyScoringModel()
        with pytest.raises(ValueError, match='less than'):
            model.add_feature_bounds('x', 10.0, 5.0)


# ---------------------------------------------------------------------------
# Test 7 — Scoring before training raises
# ---------------------------------------------------------------------------

class TestUntrained:
    def test_score_before_train_raises(self):
        model = AnomalyScoringModel()
        with pytest.raises(RuntimeError, match='trained'):
            model.score({'temperature': 100.0})


# ---------------------------------------------------------------------------
# Test 8 — Contribution percentages sum to ~100%
# ---------------------------------------------------------------------------

class TestContributions:
    def test_contributions_sum_to_100(self):
        model = _make_trained_model()
        stats = model.get_feature_stats()

        # Create an observation where at least one feature is anomalous
        report = model.score({
            'temperature': stats['temperature']['mean'] + 3.0 * stats['temperature']['std'],
            'vibration': stats['vibration']['mean'],
            'pressure': stats['pressure']['mean'],
        })

        total_contrib = sum(s.contribution_pct for s in report.anomaly_scores)
        assert abs(total_contrib - 100.0) < 0.1, (
            f'Contributions should sum to ~100%, got {total_contrib:.2f}%'
        )


# ---------------------------------------------------------------------------
# Test 9 — Unknown features in observation are ignored
# ---------------------------------------------------------------------------

class TestUnknownFeatures:
    def test_unknown_features_ignored(self):
        model = _make_trained_model()
        report = model.score({
            'temperature': 100.0,
            'unknown_sensor': 999.0,
        })

        feature_names = [s.feature_name for s in report.anomaly_scores]
        assert 'temperature' in feature_names
        assert 'unknown_sensor' not in feature_names


# ---------------------------------------------------------------------------
# Test 10 — Explanation text mentions top anomalies
# ---------------------------------------------------------------------------

class TestExplanation:
    def test_explanation_references_anomalous_features(self):
        model = _make_trained_model()
        stats = model.get_feature_stats()
        extreme_temp = stats['temperature']['mean'] + 5 * stats['temperature']['std']

        report = model.score({
            'temperature': extreme_temp,
            'vibration': 0.5,
            'pressure': 50.0,
        })

        assert 'temperature' in report.explanation
        assert 'severe' in report.explanation
