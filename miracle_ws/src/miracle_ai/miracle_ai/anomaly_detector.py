"""
Anomaly Detection Node.

Multi-model anomaly detection using Isolation Forest, Autoencoder,
and statistical methods. Fuses results with confidence weighting.
"""

from typing import Any, Dict, List, Optional
from collections import deque
import numpy as np
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import FusedSensorData, AnomalyAlert


class AnomalyDetectorNode(MiracleLifecycleNode):
    """Multi-model anomaly detection with confidence fusion.

    Parameters:
        machine_id (str): Machine identifier.
        detection_rate_hz (float): Detection frequency.
        isolation_forest_contamination (float): IF contamination param.
        autoencoder_threshold (float): AE reconstruction error threshold.
        statistical_sigma (float): Sigma for statistical detection.
        ensemble_threshold (float): Minimum ensemble score for alert.
        warmup_samples (int): Samples before detection starts.

    Subscribed Topics:
        ~/sensor/fused (FusedSensorData): Fused sensor features.

    Published Topics:
        ~/anomaly (AnomalyAlert): Detected anomaly alerts.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'anomaly_detector',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._sensor_sub = None
        self._anomaly_pub = None
        self._machine_id: str = ''

        # Detection models (lightweight implementations)
        self._feature_buffer: deque = deque(maxlen=1000)
        self._feature_mean: Optional[np.ndarray] = None
        self._feature_std: Optional[np.ndarray] = None
        self._warmup: int = 50
        self._sample_count: int = 0

        # Thresholds
        self._stat_sigma: float = 3.0
        self._ensemble_threshold: float = 0.6
        self._ae_threshold: float = 0.5
        self._if_contamination: float = 0.05

        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure anomaly detector."""
        params = self.declare_and_validate_parameters({
            'machine_id': {'default': 'cnc1', 'type': str},
            'isolation_forest_contamination': {
                'default': 0.05,
                'type': float,
                'range': (0.001, 0.5),
            },
            'autoencoder_threshold': {
                'default': 0.5,
                'type': float,
                'range': (0.01, 10.0),
            },
            'statistical_sigma': {
                'default': 3.0,
                'type': float,
                'range': (1.0, 6.0),
            },
            'ensemble_threshold': {
                'default': 0.6,
                'type': float,
                'range': (0.1, 1.0),
            },
            'warmup_samples': {
                'default': 50,
                'type': int,
                'range': (10, 10000),
            },
        })

        self._machine_id = params['machine_id']
        self._stat_sigma = params['statistical_sigma']
        self._ensemble_threshold = params['ensemble_threshold']
        self._ae_threshold = params['autoencoder_threshold']
        self._if_contamination = params['isolation_forest_contamination']
        self._warmup = params['warmup_samples']

        self._anomaly_pub = self.create_publisher(
            AnomalyAlert,
            f'/miracle/{self._machine_id}/anomaly',
            QoSProfiles.alert(),
        )

        self._sensor_sub = self.create_subscription(
            FusedSensorData,
            f'/miracle/{self._machine_id}/sensor/fused',
            self._on_sensor_data,
            QoSProfiles.sensor_data(),
        )

        self.get_logger().info(
            f"Anomaly detector configured for {self._machine_id}"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate anomaly detection."""
        self.get_logger().info("Anomaly detector activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate anomaly detection."""
        self._feature_buffer.clear()
        self._feature_mean = None
        self._feature_std = None
        self._sample_count = 0
        return TransitionCallbackReturn.SUCCESS

    def _on_sensor_data(self, msg: FusedSensorData) -> None:
        """Process fused sensor data for anomaly detection."""
        features = np.array(msg.feature_vector)
        if len(features) == 0:
            return

        with self._lock:
            self._feature_buffer.append(features)
            self._sample_count += 1

            # Warmup phase: collect statistics
            if self._sample_count < self._warmup:
                if self._sample_count % 10 == 0:
                    self.get_logger().debug(
                        f"Warmup: {self._sample_count}/{self._warmup}"
                    )
                return

            # Update running statistics
            buffer_arr = np.array(list(self._feature_buffer))
            self._feature_mean = np.mean(buffer_arr, axis=0)
            self._feature_std = np.std(buffer_arr, axis=0) + 1e-10

        # Run detection models
        scores = {}
        contributions = {}

        # 1. Statistical detection (Mahalanobis-like)
        stat_score, stat_contribs = self._statistical_detect(features)
        scores['statistical'] = stat_score
        contributions['statistical'] = stat_contribs

        # 2. Isolation Forest approximation
        if_score = self._isolation_forest_detect(features)
        scores['isolation_forest'] = if_score

        # 3. Autoencoder approximation (reconstruction error)
        ae_score = self._autoencoder_detect(features)
        scores['autoencoder'] = ae_score

        # Ensemble fusion (weighted average)
        weights = {'statistical': 0.4, 'isolation_forest': 0.3, 'autoencoder': 0.3}
        ensemble_score = sum(
            scores[k] * weights[k] for k in scores
        )

        if ensemble_score > self._ensemble_threshold:
            anomaly_type = self._classify_anomaly(features, contributions.get('statistical', []))
            self._publish_anomaly(
                anomaly_type=anomaly_type,
                confidence=ensemble_score,
                severity=min(1.0, ensemble_score),
                contributions=stat_contribs,
                scores=scores,
            )

    def _statistical_detect(self, features: np.ndarray) -> tuple:
        """Statistical anomaly detection using z-scores."""
        if self._feature_mean is None:
            return 0.0, []

        z_scores = np.abs((features - self._feature_mean) / self._feature_std)
        max_z = np.max(z_scores)
        score = min(1.0, max_z / (self._stat_sigma * 2))

        # Feature contributions (top anomalous features)
        top_indices = np.argsort(z_scores)[-5:]
        contributions = z_scores[top_indices].tolist()

        return score, contributions

    def _isolation_forest_detect(self, features: np.ndarray) -> float:
        """Simplified Isolation Forest detection.

        Uses path length approximation based on feature distribution.
        """
        if self._feature_mean is None:
            return 0.0

        # Approximate anomaly score based on feature distance
        distances = np.abs(features - self._feature_mean) / self._feature_std
        avg_distance = np.mean(distances)

        # Map to 0-1 score
        score = min(1.0, avg_distance / 5.0)
        return score

    def _autoencoder_detect(self, features: np.ndarray) -> float:
        """Simplified autoencoder detection using PCA reconstruction.

        In production, this would use a trained neural network.
        """
        if self._feature_mean is None or len(self._feature_buffer) < 20:
            return 0.0

        # PCA-based reconstruction error
        buffer_arr = np.array(list(self._feature_buffer))
        centered = buffer_arr - self._feature_mean

        # Use top-k principal components
        try:
            cov = np.cov(centered.T)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)

            # Keep top 80% variance components
            total_var = np.sum(eigenvalues)
            cumsum = np.cumsum(eigenvalues[::-1]) / total_var
            k = max(1, np.searchsorted(cumsum, 0.8) + 1)

            top_vecs = eigenvectors[:, -k:]

            # Project and reconstruct
            sample = features - self._feature_mean
            projected = top_vecs @ (top_vecs.T @ sample)
            reconstruction_error = np.linalg.norm(sample - projected)

            # Normalize
            score = min(1.0, reconstruction_error / (self._ae_threshold * 10))
            return score

        except Exception:
            return 0.0

    def _classify_anomaly(
        self, features: np.ndarray, contributions: List[float]
    ) -> str:
        """Classify the type of anomaly based on feature pattern."""
        if self._feature_mean is None:
            return 'UNKNOWN'

        z_scores = np.abs((features - self._feature_mean) / self._feature_std)

        # Heuristic classification based on which feature groups are anomalous
        # Assumes feature vector layout: [imu(20), current(9), audio(10), vision(10)]
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

    def _publish_anomaly(
        self,
        anomaly_type: str,
        confidence: float,
        severity: float,
        contributions: List[float],
        scores: Dict[str, float],
    ) -> None:
        """Publish anomaly alert."""
        msg = AnomalyAlert()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.anomaly_type = anomaly_type
        msg.confidence = confidence
        msg.severity = severity
        msg.contributing_factors = [
            f"{k}: {v:.3f}" for k, v in scores.items()
        ]
        msg.feature_contributions = contributions
        msg.recommended_action = self._get_recommendation(anomaly_type, severity)
        msg.requires_immediate_stop = severity > 0.9

        self._anomaly_pub.publish(msg)
        self.get_logger().warn(
            f"Anomaly detected: {anomaly_type} "
            f"(confidence={confidence:.2f}, severity={severity:.2f})"
        )

    @staticmethod
    def _get_recommendation(anomaly_type: str, severity: float) -> str:
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


def main(args=None):
    """Entry point for the anomaly detector node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = AnomalyDetectorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
