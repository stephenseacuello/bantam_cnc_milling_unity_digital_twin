"""Tests for the OutlierDetectionEngine in explanation_generator.py."""

import sys
import math
from unittest.mock import MagicMock

import pytest

# Mock ROS2 / MIRACLE dependencies before importing the module under test.
for mod in [
    'miracle_core.datatypes', 'miracle_core.constants',
    'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
    'rclpy.callback_groups',
    'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
    'miracle_msgs', 'miracle_msgs.msg',
]:
    sys.modules.setdefault(mod, MagicMock())

from miracle_cognitive.interface.explanation_generator import (  # noqa: E402
    OutlierDetectionEngine,
    OutlierReport,
    OutlierResult,
)


@pytest.fixture
def engine():
    return OutlierDetectionEngine()


# ------------------------------------------------------------------ #
# 1. Z-Score — obvious outlier is detected
# ------------------------------------------------------------------ #
class TestZScore:
    def test_detects_obvious_outlier(self, engine):
        data = [10.0, 10.1, 9.9, 10.2, 9.8, 10.0, 100.0]
        report = engine.detect_zscore(data, threshold=2.0)
        assert isinstance(report, OutlierReport)
        assert report.method == 'zscore'
        assert report.total_points == 7
        assert report.outlier_count >= 1
        # The extreme value (100.0) must be flagged
        outlier_vals = [r.value for r in report.results if r.is_outlier]
        assert 100.0 in outlier_vals

    def test_no_outlier_in_uniform_data(self, engine):
        data = [5.0] * 10
        report = engine.detect_zscore(data)
        assert report.outlier_count == 0


# ------------------------------------------------------------------ #
# 2. IQR — outlier at both tails
# ------------------------------------------------------------------ #
class TestIQR:
    def test_detects_high_and_low_outliers(self, engine):
        data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 100, -80]
        report = engine.detect_iqr(data, multiplier=1.5)
        assert report.method == 'iqr'
        outlier_vals = {r.value for r in report.results if r.is_outlier}
        assert 100 in outlier_vals
        assert -80 in outlier_vals

    def test_small_data_returns_no_outliers(self, engine):
        # IQR needs >= 4 points to be meaningful
        report = engine.detect_iqr([1.0, 2.0])
        assert report.outlier_count == 0
        assert report.total_points == 2


# ------------------------------------------------------------------ #
# 3. Modified Z-Score (MAD)
# ------------------------------------------------------------------ #
class TestModifiedZScore:
    def test_flags_extreme_value(self, engine):
        data = [10, 10, 10, 10, 10, 10, 10, 10, 10, 500]
        report = engine.detect_modified_zscore(data, threshold=3.5)
        assert report.method == 'modified_zscore'
        assert report.outlier_count >= 1
        outlier_vals = [r.value for r in report.results if r.is_outlier]
        assert 500 in outlier_vals

    def test_direction_labels(self, engine):
        data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 100]
        report = engine.detect_modified_zscore(data, threshold=1.0)
        for r in report.results:
            if r.value == 100:
                assert r.direction == 'high'
            else:
                assert r.direction == 'low'


# ------------------------------------------------------------------ #
# 4. Grubbs' Test
# ------------------------------------------------------------------ #
class TestGrubbs:
    def test_single_outlier_detected(self, engine):
        data = [10, 11, 10, 9, 10, 11, 10, 9, 10, 50]
        report = engine.detect_grubbs(data, alpha=0.05)
        assert report.method == 'grubbs'
        # At most one outlier per Grubbs' test
        assert report.outlier_count <= 1
        if report.outlier_count == 1:
            outlier = [r for r in report.results if r.is_outlier][0]
            assert outlier.value == 50

    def test_too_few_points(self, engine):
        report = engine.detect_grubbs([1.0, 2.0])
        assert report.outlier_count == 0


# ------------------------------------------------------------------ #
# 5. Dispatcher (detect)
# ------------------------------------------------------------------ #
class TestDispatcher:
    def test_dispatch_zscore(self, engine):
        data = [1, 2, 3, 4, 5]
        report = engine.detect(data, method='zscore')
        assert report.method == 'zscore'

    def test_dispatch_iqr(self, engine):
        data = list(range(20))
        report = engine.detect(data, method='iqr')
        assert report.method == 'iqr'

    def test_unknown_method_raises(self, engine):
        with pytest.raises(ValueError, match="Unknown method"):
            engine.detect([1, 2, 3], method='magic')


# ------------------------------------------------------------------ #
# 6. remove_outliers
# ------------------------------------------------------------------ #
class TestRemoveOutliers:
    def test_removes_extreme_values(self, engine):
        data = [10.0, 10.1, 9.9, 10.2, 9.8, 10.0, 100.0]
        clean = engine.remove_outliers(data, method='zscore', threshold=2.0)
        assert 100.0 not in clean
        assert len(clean) < len(data)


# ------------------------------------------------------------------ #
# 7. get_clean_statistics
# ------------------------------------------------------------------ #
class TestCleanStatistics:
    def test_mean_and_std_after_cleaning(self, engine):
        data = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 1000.0]
        stats = engine.get_clean_statistics(data, method='zscore', threshold=2.0)
        assert 'mean' in stats
        assert 'std' in stats
        # After removing the outlier, mean should be close to 10.0
        assert abs(stats['mean'] - 10.0) < 1.0
        assert stats['std'] < 1.0

    def test_empty_after_cleaning(self, engine):
        # Edge case: all points identical, none removed
        stats = engine.get_clean_statistics([5.0, 5.0, 5.0])
        assert stats['mean'] == 5.0
        assert stats['std'] == 0.0


# ------------------------------------------------------------------ #
# 8. OutlierResult / OutlierReport dataclass fields
# ------------------------------------------------------------------ #
class TestDataclasses:
    def test_outlier_result_fields(self):
        r = OutlierResult(
            index=0, value=42.0, score=3.1, method='zscore',
            is_outlier=True, direction='high',
        )
        assert r.index == 0
        assert r.value == 42.0
        assert r.direction == 'high'

    def test_outlier_report_pct(self, engine):
        data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 100]
        report = engine.detect_zscore(data, threshold=2.0)
        if report.outlier_count > 0:
            expected_pct = report.outlier_count / report.total_points * 100.0
            assert abs(report.outlier_pct - expected_pct) < 1e-9
