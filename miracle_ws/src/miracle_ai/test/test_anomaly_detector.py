"""Tests for anomaly detection logic.

Tests the statistical z-score computation, ensemble fusion, anomaly
classification, and recommendation logic extracted from
AnomalyDetectorNode. Mocks the ROS2 node and numpy-dependent logic.
"""

import sys
from unittest.mock import MagicMock

import pytest
import numpy as np

# --- Mock all ROS2 dependencies ---
sys.modules.setdefault('rclpy', MagicMock())
sys.modules['rclpy.lifecycle'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.qos'] = MagicMock()
sys.modules['rclpy.duration'] = MagicMock()
sys.modules['rclpy.callback_groups'] = MagicMock()
sys.modules['rcl_interfaces'] = MagicMock()
sys.modules['rcl_interfaces.msg'] = MagicMock()
sys.modules['builtin_interfaces'] = MagicMock()
sys.modules['builtin_interfaces.msg'] = MagicMock()
sys.modules['miracle_core.lifecycle_node_base'] = MagicMock()
sys.modules['miracle_core.qos_profiles'] = MagicMock()
sys.modules['miracle_core.heartbeat_mixin'] = MagicMock()
sys.modules['miracle_msgs'] = MagicMock()
sys.modules['miracle_msgs.msg'] = MagicMock()


class _AnomalyDetectionLogic:
    """Extracted pure detection logic from AnomalyDetectorNode.

    This helper isolates the statistical detection, ensemble fusion,
    classification, and recommendation logic for unit testing without
    requiring a full ROS2 lifecycle.
    """

    def __init__(self, stat_sigma=3.0, ensemble_threshold=0.6, ae_threshold=0.5):
        self.stat_sigma = stat_sigma
        self.ensemble_threshold = ensemble_threshold
        self.ae_threshold = ae_threshold
        self.feature_mean = None
        self.feature_std = None

    def set_statistics(self, mean: np.ndarray, std: np.ndarray):
        """Set running mean and std for detection."""
        self.feature_mean = mean
        self.feature_std = std

    def statistical_detect(self, features: np.ndarray):
        """Statistical anomaly detection using z-scores."""
        if self.feature_mean is None:
            return 0.0, []
        z_scores = np.abs((features - self.feature_mean) / self.feature_std)
        max_z = np.max(z_scores)
        score = min(1.0, max_z / (self.stat_sigma * 2))
        top_indices = np.argsort(z_scores)[-5:]
        contributions = z_scores[top_indices].tolist()
        return score, contributions

    def isolation_forest_detect(self, features: np.ndarray) -> float:
        """Simplified Isolation Forest detection."""
        if self.feature_mean is None:
            return 0.0
        distances = np.abs(features - self.feature_mean) / self.feature_std
        avg_distance = np.mean(distances)
        return min(1.0, avg_distance / 5.0)

    def ensemble_fusion(self, scores: dict) -> float:
        """Weighted ensemble fusion."""
        weights = {'statistical': 0.4, 'isolation_forest': 0.3, 'autoencoder': 0.3}
        return sum(scores[k] * weights[k] for k in scores)

    def classify_anomaly(self, features: np.ndarray) -> str:
        """Classify anomaly type from feature z-score patterns."""
        if self.feature_mean is None:
            return 'UNKNOWN'
        z_scores = np.abs((features - self.feature_mean) / self.feature_std)
        imu_anomaly = np.mean(z_scores[:20]) if len(z_scores) > 20 else 0
        current_anomaly = np.mean(z_scores[20:29]) if len(z_scores) > 29 else 0
        audio_anomaly = np.mean(z_scores[29:39]) if len(z_scores) > 39 else 0
        max_group = max(
            ('TOOL_WEAR', current_anomaly),
            ('CHATTER', audio_anomaly),
            ('COLLISION', imu_anomaly),
            key=lambda x: x[1],
        )
        return max_group[0] if max_group[1] > 2.0 else 'UNKNOWN'

    @staticmethod
    def get_recommendation(anomaly_type: str, severity: float) -> str:
        """Get recommended action for anomaly type."""
        recommendations = {
            'TOOL_WEAR': 'Inspect tool and consider replacement',
            'CHATTER': 'Reduce feed rate or adjust spindle speed',
            'COLLISION': 'Immediate stop - check for collision',
            'THERMAL': 'Check coolant system and reduce cutting speed',
        }
        base = recommendations.get(anomaly_type, 'Investigate anomaly source')
        if severity > 0.8:
            return f"URGENT: {base}"
        return base


@pytest.fixture
def detector():
    """Return an anomaly detection logic instance with preset statistics."""
    d = _AnomalyDetectionLogic(stat_sigma=3.0, ensemble_threshold=0.6)
    mean = np.zeros(50)
    std = np.ones(50)
    d.set_statistics(mean, std)
    return d


class TestStatisticalDetection:
    """Tests for the z-score-based statistical anomaly detection."""

    def test_normal_features_low_score(self, detector):
        """Features close to the mean should produce a low anomaly score."""
        features = np.zeros(50) + 0.1
        score, _ = detector.statistical_detect(features)
        assert score < 0.1, f"Score should be low for normal features, got {score}"

    def test_anomalous_features_high_score(self, detector):
        """Features far from the mean should produce a high anomaly score."""
        features = np.zeros(50) + 10.0  # 10-sigma away
        score, _ = detector.statistical_detect(features)
        assert score > 0.5, f"Score should be high for anomalous features, got {score}"

    def test_score_capped_at_one(self, detector):
        """Score should never exceed 1.0 regardless of deviation."""
        features = np.zeros(50) + 1000.0
        score, _ = detector.statistical_detect(features)
        assert score <= 1.0, f"Score must be capped at 1.0, got {score}"

    def test_contributions_are_returned(self, detector):
        """Top contributing features should be returned as a list."""
        features = np.zeros(50)
        features[10] = 5.0  # one outlier feature
        _, contributions = detector.statistical_detect(features)
        assert len(contributions) > 0, "Contributions list should not be empty"
        assert max(contributions) > 0, "At least one contribution should be positive"

    def test_no_statistics_returns_zero(self):
        """Without trained statistics, detection should return 0."""
        d = _AnomalyDetectionLogic()
        score, contribs = d.statistical_detect(np.zeros(10))
        assert score == 0.0
        assert contribs == []


class TestIsolationForestDetection:
    """Tests for the simplified Isolation Forest detection."""

    def test_normal_features_low_score(self, detector):
        """Near-mean features should yield a low IF score."""
        features = np.zeros(50)
        score = detector.isolation_forest_detect(features)
        assert score < 0.1

    def test_outlier_features_high_score(self, detector):
        """Far-from-mean features should yield a higher IF score."""
        features = np.ones(50) * 8.0
        score = detector.isolation_forest_detect(features)
        assert score > 0.3, f"IF score should be elevated for outliers, got {score}"

    def test_no_statistics_returns_zero(self):
        """Without statistics, IF detection should return 0."""
        d = _AnomalyDetectionLogic()
        assert d.isolation_forest_detect(np.zeros(10)) == 0.0


class TestEnsembleFusion:
    """Tests for weighted ensemble score fusion."""

    def test_all_zeros_gives_zero(self, detector):
        """All detector scores at zero should produce a zero ensemble score."""
        scores = {'statistical': 0.0, 'isolation_forest': 0.0, 'autoencoder': 0.0}
        assert detector.ensemble_fusion(scores) == pytest.approx(0.0)

    def test_all_ones_gives_one(self, detector):
        """All detector scores at 1.0 should produce 1.0 (weights sum to 1)."""
        scores = {'statistical': 1.0, 'isolation_forest': 1.0, 'autoencoder': 1.0}
        assert detector.ensemble_fusion(scores) == pytest.approx(1.0)

    def test_weighted_average(self, detector):
        """Fusion should be a weighted average with weights 0.4, 0.3, 0.3."""
        scores = {'statistical': 1.0, 'isolation_forest': 0.0, 'autoencoder': 0.0}
        expected = 0.4
        assert detector.ensemble_fusion(scores) == pytest.approx(expected)

    def test_threshold_crossing(self, detector):
        """Score above ensemble_threshold should indicate an anomaly."""
        scores = {'statistical': 0.8, 'isolation_forest': 0.7, 'autoencoder': 0.5}
        fused = detector.ensemble_fusion(scores)
        assert fused > detector.ensemble_threshold, (
            f"Fused score {fused} should exceed threshold {detector.ensemble_threshold}"
        )


class TestAnomalyClassification:
    """Tests for anomaly type classification based on feature groups."""

    def test_collision_detected_from_imu(self, detector):
        """Anomalous IMU features (indices 0-19) should classify as COLLISION."""
        features = np.zeros(50)
        features[:20] = 5.0  # IMU features anomalous
        result = detector.classify_anomaly(features)
        assert result == 'COLLISION'

    def test_tool_wear_detected_from_current(self, detector):
        """Anomalous current features (indices 20-28) should classify as TOOL_WEAR."""
        features = np.zeros(50)
        features[20:29] = 5.0  # Current features anomalous
        result = detector.classify_anomaly(features)
        assert result == 'TOOL_WEAR'

    def test_chatter_detected_from_audio(self, detector):
        """Anomalous audio features (indices 29-38) should classify as CHATTER."""
        features = np.zeros(50)
        features[29:39] = 5.0  # Audio features anomalous
        result = detector.classify_anomaly(features)
        assert result == 'CHATTER'

    def test_unknown_for_mild_anomaly(self, detector):
        """Mild anomalies (z < 2.0 in all groups) should classify as UNKNOWN."""
        features = np.ones(50) * 1.5  # mildly elevated across all groups
        result = detector.classify_anomaly(features)
        assert result == 'UNKNOWN'

    def test_unknown_without_statistics(self):
        """Without statistics, classification should return UNKNOWN."""
        d = _AnomalyDetectionLogic()
        assert d.classify_anomaly(np.zeros(50)) == 'UNKNOWN'


class TestRecommendations:
    """Tests for the recommendation engine."""

    def test_tool_wear_recommendation(self):
        """TOOL_WEAR should recommend tool inspection."""
        rec = _AnomalyDetectionLogic.get_recommendation('TOOL_WEAR', 0.5)
        assert 'tool' in rec.lower()

    def test_collision_recommendation(self):
        """COLLISION should recommend immediate stop."""
        rec = _AnomalyDetectionLogic.get_recommendation('COLLISION', 0.5)
        assert 'stop' in rec.lower() or 'collision' in rec.lower()

    def test_urgent_prefix_for_high_severity(self):
        """Severity > 0.8 should add URGENT prefix."""
        rec = _AnomalyDetectionLogic.get_recommendation('TOOL_WEAR', 0.95)
        assert rec.startswith('URGENT:')

    def test_unknown_type_gives_generic(self):
        """Unknown anomaly type should give a generic investigation recommendation."""
        rec = _AnomalyDetectionLogic.get_recommendation('UNKNOWN_TYPE', 0.5)
        assert 'investigate' in rec.lower()
