"""Tests for the FeatureImportanceRanker module."""

import math
import os
import sys
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Mock ROS2 / miracle_core dependencies before importing the module under test
# ---------------------------------------------------------------------------

for mod in [
    "miracle_core.datatypes",
    "miracle_core.constants",
    "rclpy",
    "rclpy.node",
    "rclpy.lifecycle",
    "rclpy.qos",
    "miracle_core.lifecycle_node_base",
    "miracle_core.qos_profiles",
    "miracle_msgs",
    "miracle_msgs.msg",
]:
    sys.modules.setdefault(mod, MagicMock())

sys.modules["miracle_core.lifecycle_node_base"].MiracleLifecycleNode = type(
    "FakeNode",
    (),
    {"__init__": lambda self, *a, **kw: None},
)

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..")
)

from miracle_cognitive.knowledge.reasoning_engine import (
    FeatureImportanceRanker,
    FeatureScore,
    ImportanceReport,
)


# ===========================================================================
# Helpers
# ===========================================================================


def _linear_data():
    """Create a simple dataset where feature_a is strongly correlated with
    the target and feature_b is weakly correlated."""
    n = 100
    feature_a = [float(i) for i in range(n)]
    feature_b = [float(i % 7) for i in range(n)]
    feature_c = [float(n - i) for i in range(n)]  # perfectly anti-correlated
    target = [2.0 * i + 1.0 for i in range(n)]
    data = {
        "feature_a": feature_a,
        "feature_b": feature_b,
        "feature_c": feature_c,
    }
    return data, target


# ===========================================================================
# Fixtures
# ===========================================================================


@pytest.fixture
def ranker():
    return FeatureImportanceRanker(method="correlation")


@pytest.fixture
def trained_ranker():
    r = FeatureImportanceRanker(method="correlation")
    data, target = _linear_data()
    r.train(data, target)
    return r


@pytest.fixture
def permutation_ranker():
    r = FeatureImportanceRanker(method="permutation", n_repeats=10, seed=42)
    data, target = _linear_data()
    r.train(data, target)
    return r


# ===========================================================================
# Tests
# ===========================================================================


class TestTrainValidation:
    """Test that train() validates its inputs properly."""

    def test_empty_data_raises(self, ranker):
        with pytest.raises(ValueError):
            ranker.train({}, [1.0, 2.0])

    def test_empty_target_raises(self, ranker):
        with pytest.raises(ValueError):
            ranker.train({"a": [1.0]}, [])

    def test_mismatched_lengths_raises(self, ranker):
        with pytest.raises(ValueError, match="samples"):
            ranker.train({"a": [1.0, 2.0]}, [1.0, 2.0, 3.0])


class TestRankFeatures:
    """Test rank_features() ordering and top_n filtering."""

    def test_rank_order(self, trained_ranker):
        ranked = trained_ranker.rank_features()
        # feature_a and feature_c have |r| = 1.0, feature_b is weaker
        names = [f.feature_name for f in ranked]
        assert "feature_b" == names[-1], (
            "feature_b should be ranked last (weakest correlation)"
        )

    def test_top_n_limits_results(self, trained_ranker):
        top1 = trained_ranker.rank_features(top_n=1)
        assert len(top1) == 1

    def test_rank_values_are_sequential(self, trained_ranker):
        ranked = trained_ranker.rank_features()
        ranks = [f.rank for f in ranked]
        assert ranks == [1, 2, 3]


class TestGetImportance:
    """Test get_importance() lookups."""

    def test_known_feature(self, trained_ranker):
        score = trained_ranker.get_importance("feature_a")
        assert isinstance(score, FeatureScore)
        assert score.feature_name == "feature_a"
        assert score.importance_score == pytest.approx(1.0, abs=0.01)

    def test_unknown_feature_raises(self, trained_ranker):
        with pytest.raises(KeyError, match="Unknown feature"):
            trained_ranker.get_importance("nonexistent")

    def test_not_trained_raises(self, ranker):
        with pytest.raises(RuntimeError, match="train"):
            ranker.get_importance("x")


class TestGetReport:
    """Test get_report() structure."""

    def test_report_structure(self, trained_ranker):
        report = trained_ranker.get_report("surface_roughness")
        assert isinstance(report, ImportanceReport)
        assert report.target_metric == "surface_roughness"
        assert report.method == "correlation"
        assert report.total_features == 3
        assert report.timestamp > 0
        assert len(report.features) == 3


class TestDirection:
    """Test that direction labels are correctly assigned."""

    def test_positive_direction(self, trained_ranker):
        score = trained_ranker.get_importance("feature_a")
        assert score.direction == "positive"

    def test_negative_direction(self, trained_ranker):
        score = trained_ranker.get_importance("feature_c")
        assert score.direction == "negative"


class TestCompareFeatures:
    """Test compare_features()."""

    def test_compare_returns_more_important(self, trained_ranker):
        result = trained_ranker.compare_features("feature_a", "feature_b")
        assert result["more_important"] == "feature_a"
        assert result["importance_difference"] >= 0.0
        assert "feature_a" in (result["feature_a"].feature_name,)
        assert "feature_b" in (result["feature_b"].feature_name,)


class TestIdentifyRedundant:
    """Test identify_redundant_features()."""

    def test_finds_redundant_pair(self, trained_ranker):
        # feature_a and feature_c are perfectly anti-correlated (|r| = 1.0)
        pairs = trained_ranker.identify_redundant_features(threshold=0.9)
        pair_names = [(a, b) for a, b, _ in pairs]
        found = any(
            ("feature_a" in (a, b) and "feature_c" in (a, b))
            for a, b in pair_names
        )
        assert found, "Expected feature_a and feature_c to be redundant"

    def test_high_threshold_filters(self, trained_ranker):
        # With threshold 0.999, only exact |r|=1 pairs survive
        pairs = trained_ranker.identify_redundant_features(threshold=0.999)
        for _, _, r in pairs:
            assert r > 0.999


class TestPermutationMethod:
    """Test that permutation importance produces sensible results."""

    def test_strong_feature_has_high_importance(self, permutation_ranker):
        score_a = permutation_ranker.get_importance("feature_a")
        score_b = permutation_ranker.get_importance("feature_b")
        assert score_a.importance_score > score_b.importance_score

    def test_permutation_report_method(self, permutation_ranker):
        report = permutation_ranker.get_report("target_metric")
        assert report.method == "permutation"


class TestMutualInformationMethod:
    """Test the mutual_information method."""

    def test_mi_strong_vs_weak(self):
        ranker = FeatureImportanceRanker(method="mutual_information", n_bins=10)
        data, target = _linear_data()
        ranker.train(data, target)
        score_a = ranker.get_importance("feature_a")
        score_b = ranker.get_importance("feature_b")
        # Strongly correlated feature should have higher MI
        assert score_a.importance_score > score_b.importance_score
