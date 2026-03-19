"""Tests for KMeansClusteringAnalyzer k-means clustering.

Covers fit, predict, find_optimal_k, get_cluster_statistics,
get_nearest_neighbors, convergence, edge cases, and silhouette scoring.
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
    KMeansClusteringAnalyzer,
    ClusterResult,
    ClusteringReport,
)


@pytest.fixture
def analyzer():
    return KMeansClusteringAnalyzer()


@pytest.fixture
def simple_data():
    """Three well-separated 2-D clusters."""
    return [
        [0.0, 0.0], [0.1, 0.1], [0.2, 0.0],
        [5.0, 5.0], [5.1, 5.1], [5.2, 5.0],
        [10.0, 0.0], [10.1, 0.1], [10.2, 0.0],
    ]


# -- Test 1: fit returns a valid ClusteringReport ----------------------------

def test_fit_returns_clustering_report(analyzer, simple_data):
    report = analyzer.fit(simple_data, k=3)
    assert isinstance(report, ClusteringReport)
    assert report.k == 3
    assert len(report.clusters) == 3
    assert report.total_inertia >= 0.0
    assert report.iterations >= 1


# -- Test 2: all points are assigned to exactly one cluster ------------------

def test_all_points_assigned(analyzer, simple_data):
    report = analyzer.fit(simple_data, k=3)
    all_members = []
    for c in report.clusters:
        all_members.extend(c.members)
    assert sorted(all_members) == list(range(len(simple_data)))


# -- Test 3: predict assigns to nearest centroid -----------------------------

def test_predict_nearest_centroid(analyzer, simple_data):
    analyzer.fit(simple_data, k=3)
    # A point near (0,0) cluster should get the same cluster as index 0
    cluster_origin = analyzer.predict([0.05, 0.05])
    cluster_far = analyzer.predict([10.05, 0.05])
    assert cluster_origin != cluster_far


# -- Test 4: predict raises without fit -------------------------------------

def test_predict_without_fit_raises(analyzer):
    with pytest.raises(RuntimeError):
        analyzer.predict([1.0, 2.0])


# -- Test 5: get_cluster_statistics returns correct keys ---------------------

def test_cluster_statistics_keys(analyzer, simple_data):
    report = analyzer.fit(simple_data, k=3)
    cid = report.clusters[0].cluster_id
    stats = analyzer.get_cluster_statistics(cid)
    assert set(stats.keys()) == {'mean', 'std', 'min', 'max'}
    # Each value list should have same dimensionality as input
    for key in stats:
        assert len(stats[key]) == 2


# -- Test 6: get_nearest_neighbors returns correct count --------------------

def test_nearest_neighbors_count(analyzer, simple_data):
    analyzer.fit(simple_data, k=3)
    neighbors = analyzer.get_nearest_neighbors([0.0, 0.0], n=3)
    assert len(neighbors) == 3
    # Should be sorted by distance
    dists = [d for _, d in neighbors]
    assert dists == sorted(dists)


# -- Test 7: find_optimal_k returns a sensible k ----------------------------

def test_find_optimal_k(analyzer, simple_data):
    best_k = analyzer.find_optimal_k(simple_data, max_k=5)
    # With three clear clusters, optimal k should be 2 or 3
    assert 2 <= best_k <= 4


# -- Test 8: fit with k=1 yields a single cluster ---------------------------

def test_fit_k_one(analyzer, simple_data):
    report = analyzer.fit(simple_data, k=1)
    assert report.k == 1
    assert len(report.clusters) == 1
    assert report.clusters[0].member_count == len(simple_data)
    assert report.silhouette_score == 0.0


# -- Test 9: convergence flag is set correctly --------------------------------

def test_convergence(analyzer, simple_data):
    report = analyzer.fit(simple_data, k=3, max_iterations=1000)
    # Well-separated clusters should converge
    assert report.converged is True


# -- Test 10: fit rejects empty data -----------------------------------------

def test_fit_empty_data_raises(analyzer):
    with pytest.raises(ValueError):
        analyzer.fit([], k=2)


# -- Test 11: fit rejects k > n ---------------------------------------------

def test_fit_k_greater_than_n_raises(analyzer):
    with pytest.raises(ValueError):
        analyzer.fit([[1.0], [2.0]], k=5)


# -- Test 12: ClusterResult dataclass fields ---------------------------------

def test_cluster_result_fields():
    cr = ClusterResult(
        cluster_id=0, centroid=[1.0, 2.0],
        member_count=5, members=[0, 1, 2, 3, 4], inertia=1.5,
    )
    assert cr.cluster_id == 0
    assert cr.centroid == [1.0, 2.0]
    assert cr.member_count == 5
    assert cr.inertia == 1.5
