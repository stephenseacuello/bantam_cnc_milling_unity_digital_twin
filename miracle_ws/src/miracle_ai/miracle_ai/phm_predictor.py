"""
Prognostics and Health Management (PHM) Predictor Node.

Predicts Remaining Useful Life (RUL) for machine components
using degradation modeling and trend analysis.
"""

from typing import Any, Dict, List, Optional
from collections import deque
import numpy as np
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import FusedSensorData, PHMPrediction, MachineState


class PHMPredictorNode(MiracleLifecycleNode):
    """Predicts remaining useful life using degradation modeling.

    Parameters:
        machine_id (str): Machine identifier.
        prediction_interval_sec (float): Prediction update interval.
        health_window_size (int): Samples for health index computation.
        degradation_model (str): Model type (linear, exponential).

    Subscribed Topics:
        ~/sensor/fused (FusedSensorData): Fused sensor data.
        ~/state (MachineState): Machine state.

    Published Topics:
        ~/phm_prediction (PHMPrediction): RUL predictions.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'phm_predictor',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._sensor_sub = None
        self._state_sub = None
        self._prediction_pub = None
        self._predict_timer = None
        self._machine_id: str = ''

        self._health_history: deque = deque(maxlen=1000)
        self._feature_history: deque = deque(maxlen=500)
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure PHM predictor."""
        params = self.declare_and_validate_parameters({
            'machine_id': {'default': 'cnc1', 'type': str},
            'prediction_interval_sec': {
                'default': 60.0,
                'type': float,
                'range': (1.0, 3600.0),
            },
            'health_window_size': {
                'default': 100,
                'type': int,
                'range': (10, 10000),
            },
            'degradation_model': {
                'default': 'exponential',
                'type': str,
                'choices': ['linear', 'exponential'],
            },
        })

        self._machine_id = params['machine_id']

        self._sensor_sub = self.create_subscription(
            FusedSensorData,
            f'/miracle/{self._machine_id}/sensor/fused',
            self._on_sensor_data,
            QoSProfiles.sensor_data(),
        )

        self._prediction_pub = self.create_publisher(
            PHMPrediction,
            f'/miracle/{self._machine_id}/phm_prediction',
            QoSProfiles.state_data(),
        )

        self.get_logger().info(f"PHM predictor configured for {self._machine_id}")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate PHM prediction."""
        interval = self.get_parameter('prediction_interval_sec').value
        self._predict_timer = self.create_timer(
            interval,
            self._run_prediction,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("PHM predictor activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate PHM prediction."""
        if self._predict_timer is not None:
            self._predict_timer.cancel()
            self._predict_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_sensor_data(self, msg: FusedSensorData) -> None:
        """Collect sensor data for health computation."""
        features = np.array(msg.feature_vector)
        if len(features) > 0:
            with self._lock:
                self._feature_history.append(features)

    def _compute_health_index(self) -> float:
        """Compute current health index (0=failed, 1=perfect)."""
        with self._lock:
            if len(self._feature_history) < 10:
                return 1.0

            recent = np.array(list(self._feature_history))

        # Health index based on feature stability
        variance = np.mean(np.var(recent, axis=0))
        # Higher variance = degradation
        health = max(0.0, 1.0 - variance / 100.0)
        return min(1.0, health)

    def _predict_rul(self, health_values: List[float]) -> tuple:
        """Predict remaining useful life from health trend.

        Returns:
            Tuple of (rul_hours, confidence).
        """
        if len(health_values) < 5:
            return 1000.0, 0.1

        x = np.arange(len(health_values), dtype=float)
        y = np.array(health_values)

        model_type = self.get_parameter('degradation_model').value

        if model_type == 'exponential' and np.all(y > 0):
            # Fit exponential: h(t) = a * exp(b * t)
            log_y = np.log(y + 1e-10)
            coeffs = np.polyfit(x, log_y, 1)
            rate = coeffs[0]

            if rate >= 0:
                # Not degrading
                return 1000.0, 0.3

            # Time to reach failure threshold (health=0.2)
            threshold = 0.2
            current_health = y[-1]
            if current_health <= threshold:
                return 0.0, 0.9

            steps_to_failure = (np.log(threshold) - np.log(current_health)) / rate
            hours_per_step = 1.0 / 60.0  # Assume 1 min per step
            rul = max(0.0, steps_to_failure * hours_per_step)

        else:
            # Linear: h(t) = a + b * t
            coeffs = np.polyfit(x, y, 1)
            rate = coeffs[0]

            if rate >= 0:
                return 1000.0, 0.3

            threshold = 0.2
            current_health = y[-1]
            if current_health <= threshold:
                return 0.0, 0.9

            steps = (threshold - current_health) / rate
            rul = max(0.0, steps / 60.0)

        # Confidence based on data quantity and fit quality
        residuals = y - np.polyval(coeffs, x)
        r_squared = 1 - np.sum(residuals**2) / np.sum((y - np.mean(y))**2 + 1e-10)
        confidence = max(0.1, min(0.95, r_squared * len(health_values) / 100))

        return rul, confidence

    def _run_prediction(self) -> None:
        """Execute periodic prediction."""
        health = self._compute_health_index()
        self._health_history.append(health)

        if len(self._health_history) < 5:
            return

        health_values = list(self._health_history)
        rul, confidence = self._predict_rul(health_values)

        msg = PHMPrediction()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.component = 'spindle_bearing'
        msg.prediction_type = 'RUL'
        msg.remaining_useful_life_hours = rul
        msg.confidence = confidence
        msg.health_index = health
        msg.trend_data = health_values[-10:]

        if rul < 24.0:
            msg.recommended_action = 'Schedule immediate maintenance'
        elif rul < 168.0:
            msg.recommended_action = 'Plan maintenance within one week'
        else:
            msg.recommended_action = 'Continue normal operation'

        self._prediction_pub.publish(msg)
        self.get_logger().debug(
            f"PHM: health={health:.2f}, RUL={rul:.1f}h, conf={confidence:.2f}"
        )


def main(args=None):
    """Entry point for the PHM predictor node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = PHMPredictorNode()
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
