"""
Prediction Validation System.

Compares digital twin predictions against actual sensor data and tracks
accuracy over time, computing RMSE, bias, R-squared, and drift rate
to detect model degradation.
"""

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState


@dataclass
class PredictionSample:
    """A single prediction-vs-actual comparison sample."""
    timestamp: float
    machine_id: str
    metric: str  # 'force', 'temperature', 'wear', 'power'
    predicted: float
    actual: float
    error_pct: float


@dataclass
class AccuracyMetrics:
    """Aggregated accuracy metrics for a single prediction metric."""
    metric: str
    sample_count: int = 0
    mean_error_pct: float = 0.0
    max_error_pct: float = 0.0
    rmse: float = 0.0
    bias: float = 0.0
    r_squared: float = 0.0
    drift_rate: float = 0.0
    last_updated: float = 0.0


class PredictionValidatorNode(MiracleLifecycleNode):
    """Validates digital twin predictions against actual sensor data.

    Subscribes to MachineState for actual sensor readings and compares
    them against registered predictions from the PredictionRunner.
    Computes accuracy metrics including RMSE, bias, R-squared, and
    drift rate to detect model degradation over time.

    Parameters:
        drift_threshold_pct (float): Mean error threshold for drift warning (default 15%).
        history_size (int): Maximum number of samples retained (default 5000).
        comparison_window_sec (float): Max age of prediction for comparison (default 10.0).
        accuracy_interval_sec (float): How often to recompute accuracy (default 30.0).
    """

    CRITICALITY_MEDIUM = 'MEDIUM'

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'prediction_validator',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._lock = threading.Lock()
        self._samples: List[PredictionSample] = []
        self._accuracy: Dict[str, AccuracyMetrics] = {}
        self._pending_predictions: Dict[str, Dict[str, float]] = {}
        self._prediction_timestamps: Dict[str, float] = {}
        self._machine_state_sub = None
        self._accuracy_timer = None

        # Configuration defaults (overridden in _do_configure)
        self._drift_threshold_pct: float = 15.0
        self._history_size: int = 5000
        self._comparison_window_sec: float = 10.0
        self._accuracy_interval_sec: float = 30.0

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure the prediction validator."""
        params = self.declare_and_validate_parameters({
            'drift_threshold_pct': {
                'default': 15.0,
                'type': float,
                'range': (1.0, 100.0),
            },
            'history_size': {
                'default': 5000,
                'type': int,
                'range': (100, 100000),
            },
            'comparison_window_sec': {
                'default': 10.0,
                'type': float,
                'range': (1.0, 120.0),
            },
            'accuracy_interval_sec': {
                'default': 30.0,
                'type': float,
                'range': (5.0, 600.0),
            },
        })

        self._drift_threshold_pct = params.get('drift_threshold_pct', 15.0)
        self._history_size = params.get('history_size', 5000)
        self._comparison_window_sec = params.get('comparison_window_sec', 10.0)
        self._accuracy_interval_sec = params.get('accuracy_interval_sec', 30.0)

        self._machine_state_sub = self.create_subscription(
            MachineState,
            '/miracle/machine_state',
            self._on_machine_state,
            QoSProfiles.state_data(),
        )

        self._accuracy_timer = self.create_timer(
            self._accuracy_interval_sec,
            self._compute_all_accuracy,
        )

        self.get_logger().info("Prediction validator configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate the prediction validator."""
        self.get_logger().info("Prediction validator activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate the prediction validator."""
        return TransitionCallbackReturn.SUCCESS

    def register_prediction(
        self,
        machine_id: str,
        predictions: Dict[str, float],
    ) -> None:
        """Register latest predictions for a machine.

        Called by PredictionRunner after running a simulation. The predictions
        dict maps metric names to predicted values, e.g.
        {'force': 250.0, 'temperature': 45.0, 'power': 1200.0}.

        Args:
            machine_id: Target machine identifier.
            predictions: Dict mapping metric name to predicted value.
        """
        with self._lock:
            self._pending_predictions[machine_id] = dict(predictions)
            self._prediction_timestamps[machine_id] = time.time()

    def _on_machine_state(self, msg: MachineState) -> None:
        """Compare incoming actual data against registered predictions."""
        now = time.time()
        machine_id = msg.machine_id

        with self._lock:
            if machine_id not in self._pending_predictions:
                return

            pred_time = self._prediction_timestamps.get(machine_id, 0.0)
            if now - pred_time > self._comparison_window_sec:
                # Prediction is stale; discard it
                del self._pending_predictions[machine_id]
                del self._prediction_timestamps[machine_id]
                return

            predictions = self._pending_predictions.pop(machine_id)
            self._prediction_timestamps.pop(machine_id, None)

        # Extract actual values from MachineState
        actuals = self._extract_actuals(msg)

        with self._lock:
            for metric, predicted in predictions.items():
                if metric not in actuals:
                    continue
                actual = actuals[metric]

                if actual == 0.0:
                    error_pct = 0.0 if predicted == 0.0 else 100.0
                else:
                    error_pct = abs(predicted - actual) / abs(actual) * 100.0

                sample = PredictionSample(
                    timestamp=now,
                    machine_id=machine_id,
                    metric=metric,
                    predicted=predicted,
                    actual=actual,
                    error_pct=error_pct,
                )
                self._samples.append(sample)

            # Trim history
            if len(self._samples) > self._history_size:
                self._samples = self._samples[-self._history_size:]

    @staticmethod
    def _extract_actuals(msg: MachineState) -> Dict[str, float]:
        """Extract actual metric values from a MachineState message.

        Metric extraction:
        - force = sqrt(Fx^2 + Fy^2 + Fz^2)
        - temperature = spindle_temp
        - power = spindle_load * 10
        """
        fx = getattr(msg, 'cutting_force_x', 0.0)
        fy = getattr(msg, 'cutting_force_y', 0.0)
        fz = getattr(msg, 'cutting_force_z', 0.0)
        force = math.sqrt(fx ** 2 + fy ** 2 + fz ** 2)

        temperature = getattr(msg, 'spindle_temp', 0.0)
        spindle_load = getattr(msg, 'spindle_load', 0.0)
        power = spindle_load * 10.0

        return {
            'force': force,
            'temperature': temperature,
            'power': power,
        }

    def _compute_all_accuracy(self) -> None:
        """Recompute accuracy metrics for all tracked metrics."""
        with self._lock:
            metrics_set = set(s.metric for s in self._samples)
            for metric in metrics_set:
                metric_samples = [s for s in self._samples if s.metric == metric]
                if not metric_samples:
                    continue
                accuracy = self._compute_accuracy(metric, metric_samples)
                self._accuracy[metric] = accuracy

                if accuracy.mean_error_pct > self._drift_threshold_pct:
                    self.get_logger().warn(
                        f"Prediction drift warning for '{metric}': "
                        f"mean error {accuracy.mean_error_pct:.1f}% "
                        f"exceeds threshold {self._drift_threshold_pct:.1f}%"
                    )

    @staticmethod
    def _compute_accuracy(
        metric: str,
        samples: List[PredictionSample],
    ) -> AccuracyMetrics:
        """Compute accuracy metrics from a list of samples for one metric.

        Calculates mean error, max error, RMSE, bias, R-squared, and
        drift rate (via linear regression of error over time).
        """
        n = len(samples)
        if n == 0:
            return AccuracyMetrics(metric=metric)

        errors_pct = [s.error_pct for s in samples]
        mean_error = sum(errors_pct) / n
        max_error = max(errors_pct)

        # RMSE (of predicted vs actual)
        squared_errors = [(s.predicted - s.actual) ** 2 for s in samples]
        rmse = math.sqrt(sum(squared_errors) / n)

        # Bias (mean signed error: predicted - actual)
        signed_errors = [s.predicted - s.actual for s in samples]
        bias = sum(signed_errors) / n

        # R-squared
        actual_mean = sum(s.actual for s in samples) / n
        ss_tot = sum((s.actual - actual_mean) ** 2 for s in samples)
        ss_res = sum((s.actual - s.predicted) ** 2 for s in samples)
        if ss_tot > 0:
            r_squared = 1.0 - ss_res / ss_tot
        else:
            r_squared = 1.0 if ss_res == 0.0 else 0.0

        # Drift rate via linear regression of error_pct over time
        drift_rate = PredictionValidatorNode._compute_drift_rate(samples)

        return AccuracyMetrics(
            metric=metric,
            sample_count=n,
            mean_error_pct=mean_error,
            max_error_pct=max_error,
            rmse=rmse,
            bias=bias,
            r_squared=r_squared,
            drift_rate=drift_rate,
            last_updated=time.time(),
        )

    @staticmethod
    def _compute_drift_rate(samples: List[PredictionSample]) -> float:
        """Compute drift rate as slope of error_pct over time via OLS.

        Returns the slope in units of error_pct per second.
        """
        n = len(samples)
        if n < 2:
            return 0.0

        t0 = samples[0].timestamp
        times = [s.timestamp - t0 for s in samples]
        errors = [s.error_pct for s in samples]

        t_mean = sum(times) / n
        e_mean = sum(errors) / n

        numerator = sum((t - t_mean) * (e - e_mean) for t, e in zip(times, errors))
        denominator = sum((t - t_mean) ** 2 for t in times)

        if denominator == 0.0:
            return 0.0

        return numerator / denominator

    @property
    def accuracy(self) -> Dict[str, AccuracyMetrics]:
        """Return the current accuracy metrics dict (metric -> AccuracyMetrics)."""
        with self._lock:
            return dict(self._accuracy)

    @property
    def samples(self) -> List[PredictionSample]:
        """Return a copy of the sample history."""
        with self._lock:
            return list(self._samples)

    def get_accuracy_summary(self) -> str:
        """Return a human-readable summary of prediction accuracy.

        Returns:
            Multi-line string summarizing accuracy per metric,
            or 'No prediction data yet.' if no samples exist.
        """
        with self._lock:
            if not self._accuracy:
                return "No prediction data yet."

            lines = ["Prediction Accuracy Summary:"]
            for metric, acc in sorted(self._accuracy.items()):
                lines.append(
                    f"  {metric}: samples={acc.sample_count}, "
                    f"mean_err={acc.mean_error_pct:.1f}%, "
                    f"max_err={acc.max_error_pct:.1f}%, "
                    f"RMSE={acc.rmse:.2f}, "
                    f"bias={acc.bias:.2f}, "
                    f"R²={acc.r_squared:.3f}, "
                    f"drift={acc.drift_rate:.4f}%/s"
                )
            return '\n'.join(lines)

    def set_validator(self, validator: Any) -> None:
        """No-op for API compatibility; this node IS the validator."""
        pass
