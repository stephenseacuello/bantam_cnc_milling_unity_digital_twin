"""Tests for the Prediction Validator."""

import pytest
import sys
import os
import math
import time
import importlib

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from unittest.mock import MagicMock
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# Minimal stubs for ROS2 / miracle types so tests run without ROS2.
# ---------------------------------------------------------------------------


@dataclass
class _FakeTime:
    sec: int = 0
    nanosec: int = 0


@dataclass
class _FakeMachineState:
    machine_id: str = 'cnc1'
    cutting_force_x: float = 0.0
    cutting_force_y: float = 0.0
    cutting_force_z: float = 0.0
    spindle_load: float = 0.0
    spindle_temp: float = 25.0
    spindle_speed: float = 0.0
    feed_rate: float = 0.0


# Stub miracle_msgs
_miracle_msgs_mod = type(sys)('miracle_msgs')
_miracle_msgs_msg_mod = type(sys)('miracle_msgs.msg')
_miracle_msgs_msg_mod.MachineState = _FakeMachineState
sys.modules.setdefault('miracle_msgs', _miracle_msgs_mod)
sys.modules.setdefault('miracle_msgs.msg', _miracle_msgs_msg_mod)
# Ensure our fakes are applied even if another test loaded first
_existing_msg_mod = sys.modules['miracle_msgs.msg']
_existing_msg_mod.MachineState = _FakeMachineState


class _FakeTransitionCallbackReturn:
    SUCCESS = 0
    FAILURE = 1
    ERROR = 2


# Stub rclpy
_rclpy_mod = type(sys)('rclpy')
_rclpy_lifecycle_mod = type(sys)('rclpy.lifecycle')
_rclpy_lifecycle_mod.TransitionCallbackReturn = _FakeTransitionCallbackReturn
sys.modules.setdefault('rclpy', _rclpy_mod)
sys.modules.setdefault('rclpy.lifecycle', _rclpy_lifecycle_mod)
sys.modules.setdefault('rclpy.executors', type(sys)('rclpy.executors'))
sys.modules.setdefault('rclpy.callback_groups', type(sys)('rclpy.callback_groups'))
sys.modules.setdefault('rclpy.action', type(sys)('rclpy.action'))
sys.modules.setdefault('rclpy.action.server', type(sys)('rclpy.action.server'))


# Stub miracle_core
class _FakeMiracleLifecycleNode:
    CRITICALITY_MEDIUM = 'MEDIUM'

    def __init__(self, name, **kwargs):
        self._name = name
        self.service_callback_group = None
        self._logger = MagicMock()

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.nanoseconds = 1000000000
        clock.now.return_value.to_msg.return_value = _FakeTime()
        return clock

    def get_logger(self):
        return self._logger

    def declare_and_validate_parameters(self, params):
        return {k: v.get('default') for k, v in params.items()}

    def create_publisher(self, msg_type, topic, qos):
        return MagicMock()

    def create_subscription(self, msg_type, topic, callback, qos):
        return MagicMock()

    def create_timer(self, period, callback):
        return MagicMock()


class _FakeQoSProfiles:
    @staticmethod
    def state_data():
        return MagicMock()


_miracle_core_mod = type(sys)('miracle_core')
_miracle_core_lifecycle_mod = type(sys)('miracle_core.lifecycle_node_base')
_miracle_core_lifecycle_mod.MiracleLifecycleNode = _FakeMiracleLifecycleNode
_miracle_core_qos_mod = type(sys)('miracle_core.qos_profiles')
_miracle_core_qos_mod.QoSProfiles = _FakeQoSProfiles
_miracle_core_heartbeat_mod = type(sys)('miracle_core.heartbeat_mixin')
_miracle_core_heartbeat_mod.HeartbeatMixin = type('HeartbeatMixin', (), {})
_miracle_core_param_mod = type(sys)('miracle_core.parameter_validation')
_miracle_core_param_mod.declare_and_validate_parameters = lambda *a, **k: {}
_miracle_core_param_mod.create_parameter_callback = lambda *a, **k: lambda *a: None
_miracle_core_exc_mod = type(sys)('miracle_core.exceptions')
_miracle_core_exc_mod.ConfigurationError = type('ConfigurationError', (Exception,), {})

sys.modules.setdefault('miracle_core.lifecycle_node_base', _miracle_core_lifecycle_mod)
sys.modules.setdefault('miracle_core.qos_profiles', _miracle_core_qos_mod)
sys.modules.setdefault('miracle_core.heartbeat_mixin', _miracle_core_heartbeat_mod)
sys.modules.setdefault('miracle_core.parameter_validation', _miracle_core_param_mod)
sys.modules.setdefault('miracle_core.exceptions', _miracle_core_exc_mod)

# Ensure our stubs are applied
sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = _FakeMiracleLifecycleNode
sys.modules['miracle_core.qos_profiles'].QoSProfiles = _FakeQoSProfiles

# Force reimport to pick up stubs
if 'miracle_twin.prediction_validator' in sys.modules:
    del sys.modules['miracle_twin.prediction_validator']

from miracle_twin.prediction_validator import (
    PredictionValidatorNode,
    PredictionSample,
    AccuracyMetrics,
)


# ---------------------------------------------------------------------------
# Helper to build a validator with configuration applied
# ---------------------------------------------------------------------------

def _make_validator(**kwargs) -> PredictionValidatorNode:
    """Create a PredictionValidatorNode with defaults configured."""
    node = PredictionValidatorNode()
    node._do_configure()
    for k, v in kwargs.items():
        setattr(node, f'_{k}', v)
    return node


def _make_machine_state(
    machine_id='cnc1',
    fx=60.0, fy=80.0, fz=0.0,
    spindle_temp=45.0,
    spindle_load=120.0,
) -> _FakeMachineState:
    return _FakeMachineState(
        machine_id=machine_id,
        cutting_force_x=fx,
        cutting_force_y=fy,
        cutting_force_z=fz,
        spindle_temp=spindle_temp,
        spindle_load=spindle_load,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestPredictionValidator:

    def test_register_prediction_and_compare(self):
        """Register a prediction and verify it gets compared to actual data."""
        node = _make_validator()
        # force actual = sqrt(60^2 + 80^2) = 100
        node.register_prediction('cnc1', {'force': 100.0})
        msg = _make_machine_state(machine_id='cnc1', fx=60.0, fy=80.0, fz=0.0)
        node._on_machine_state(msg)

        samples = node.samples
        assert len(samples) == 1
        assert samples[0].metric == 'force'
        assert samples[0].predicted == 100.0
        assert samples[0].actual == pytest.approx(100.0)
        assert samples[0].error_pct == pytest.approx(0.0, abs=0.01)

    def test_error_percentage_calculation(self):
        """Predicted=150, actual=100 should give 50% error."""
        node = _make_validator()
        node.register_prediction('cnc1', {'force': 150.0})
        # actual force = sqrt(60^2+80^2) = 100
        msg = _make_machine_state(machine_id='cnc1', fx=60.0, fy=80.0, fz=0.0)
        node._on_machine_state(msg)

        samples = node.samples
        assert len(samples) == 1
        assert samples[0].error_pct == pytest.approx(50.0, abs=0.1)

    def test_accuracy_metrics_computation(self):
        """Verify mean, max, RMSE, bias computation."""
        node = _make_validator()

        # Inject samples directly for controlled testing
        now = time.time()
        with node._lock:
            node._samples = [
                PredictionSample(now, 'cnc1', 'force', 110.0, 100.0, 10.0),
                PredictionSample(now + 1, 'cnc1', 'force', 120.0, 100.0, 20.0),
                PredictionSample(now + 2, 'cnc1', 'force', 130.0, 100.0, 30.0),
            ]

        node._compute_all_accuracy()
        acc = node.accuracy['force']

        assert acc.sample_count == 3
        assert acc.mean_error_pct == pytest.approx(20.0, abs=0.01)
        assert acc.max_error_pct == pytest.approx(30.0, abs=0.01)

        # RMSE = sqrt((10^2 + 20^2 + 30^2) / 3)
        expected_rmse = math.sqrt((100 + 400 + 900) / 3)
        assert acc.rmse == pytest.approx(expected_rmse, abs=0.01)

        # Bias = mean(10, 20, 30) = 20
        assert acc.bias == pytest.approx(20.0, abs=0.01)

    def test_r_squared_perfect_predictions(self):
        """Perfect predictions should yield R^2 = 1.0."""
        node = _make_validator()
        now = time.time()
        with node._lock:
            node._samples = [
                PredictionSample(now, 'cnc1', 'force', 100.0, 100.0, 0.0),
                PredictionSample(now + 1, 'cnc1', 'force', 200.0, 200.0, 0.0),
                PredictionSample(now + 2, 'cnc1', 'force', 300.0, 300.0, 0.0),
            ]

        node._compute_all_accuracy()
        acc = node.accuracy['force']
        assert acc.r_squared == pytest.approx(1.0, abs=0.001)

    def test_drift_rate_increasing_error(self):
        """Increasing error over time should produce positive drift rate."""
        node = _make_validator()
        now = time.time()
        with node._lock:
            node._samples = [
                PredictionSample(now, 'cnc1', 'force', 110.0, 100.0, 10.0),
                PredictionSample(now + 10, 'cnc1', 'force', 120.0, 100.0, 20.0),
                PredictionSample(now + 20, 'cnc1', 'force', 130.0, 100.0, 30.0),
                PredictionSample(now + 30, 'cnc1', 'force', 140.0, 100.0, 40.0),
            ]

        node._compute_all_accuracy()
        acc = node.accuracy['force']
        assert acc.drift_rate > 0.0

    def test_drift_warning_when_threshold_exceeded(self):
        """Mean error > drift_threshold_pct should trigger a warning."""
        node = _make_validator(drift_threshold_pct=10.0)
        now = time.time()
        with node._lock:
            node._samples = [
                PredictionSample(now, 'cnc1', 'force', 120.0, 100.0, 20.0),
                PredictionSample(now + 1, 'cnc1', 'force', 125.0, 100.0, 25.0),
            ]

        # The warn method should be called
        node._compute_all_accuracy()
        logger = node.get_logger()
        logger.warn.assert_called()
        call_args = logger.warn.call_args[0][0]
        assert 'drift warning' in call_args.lower() or 'drift' in call_args.lower()

    def test_history_size_trimming(self):
        """Samples should be trimmed to history_size."""
        node = _make_validator(history_size=5000)
        now = time.time()

        # Manually add more than 5000 samples
        with node._lock:
            node._samples = [
                PredictionSample(now + i, 'cnc1', 'force', 100.0 + i, 100.0,
                                 float(i))
                for i in range(5050)
            ]

        # Register and trigger a new comparison to invoke trimming
        node.register_prediction('cnc1', {'force': 100.0})
        msg = _make_machine_state(machine_id='cnc1', fx=60.0, fy=80.0, fz=0.0)
        node._on_machine_state(msg)

        assert len(node.samples) <= 5000

    def test_stale_prediction_ignored(self):
        """Predictions older than comparison_window_sec should be discarded."""
        node = _make_validator(comparison_window_sec=10.0)

        # Register a prediction and then artificially age it
        node.register_prediction('cnc1', {'force': 100.0})
        with node._lock:
            node._prediction_timestamps['cnc1'] = time.time() - 15.0

        msg = _make_machine_state(machine_id='cnc1', fx=60.0, fy=80.0, fz=0.0)
        node._on_machine_state(msg)

        assert len(node.samples) == 0

    def test_zero_actual_handling(self):
        """When actual is 0 and predicted is non-zero, error_pct should be 100%."""
        node = _make_validator()
        node.register_prediction('cnc1', {'force': 50.0})
        msg = _make_machine_state(
            machine_id='cnc1', fx=0.0, fy=0.0, fz=0.0,
            spindle_temp=0.0, spindle_load=0.0,
        )
        node._on_machine_state(msg)

        force_samples = [s for s in node.samples if s.metric == 'force']
        assert len(force_samples) == 1
        assert force_samples[0].error_pct == 100.0

    def test_zero_actual_and_zero_predicted(self):
        """When both actual and predicted are 0, error_pct should be 0."""
        node = _make_validator()
        node.register_prediction('cnc1', {'force': 0.0})
        msg = _make_machine_state(
            machine_id='cnc1', fx=0.0, fy=0.0, fz=0.0,
            spindle_temp=0.0, spindle_load=0.0,
        )
        node._on_machine_state(msg)

        force_samples = [s for s in node.samples if s.metric == 'force']
        assert len(force_samples) == 1
        assert force_samples[0].error_pct == 0.0

    def test_accuracy_summary_format(self):
        """Summary should contain metric names and key statistics."""
        node = _make_validator()
        now = time.time()
        with node._lock:
            node._samples = [
                PredictionSample(now, 'cnc1', 'force', 110.0, 100.0, 10.0),
                PredictionSample(now, 'cnc1', 'temperature', 50.0, 45.0, 11.1),
            ]

        node._compute_all_accuracy()
        summary = node.get_accuracy_summary()

        assert 'Prediction Accuracy Summary:' in summary
        assert 'force' in summary
        assert 'temperature' in summary
        assert 'mean_err=' in summary
        assert 'RMSE=' in summary
        assert 'R²=' in summary  # Using the actual unicode char
        assert 'drift=' in summary

    def test_multiple_metrics_tracked_independently(self):
        """Different metrics should have independent accuracy stats."""
        node = _make_validator()
        now = time.time()
        with node._lock:
            node._samples = [
                PredictionSample(now, 'cnc1', 'force', 110.0, 100.0, 10.0),
                PredictionSample(now, 'cnc1', 'force', 120.0, 100.0, 20.0),
                PredictionSample(now, 'cnc1', 'temperature', 50.0, 45.0, 11.1),
            ]

        node._compute_all_accuracy()
        acc = node.accuracy

        assert 'force' in acc
        assert 'temperature' in acc
        assert acc['force'].sample_count == 2
        assert acc['temperature'].sample_count == 1
        assert acc['force'].mean_error_pct != acc['temperature'].mean_error_pct

    def test_empty_samples_summary(self):
        """Empty samples should return 'No prediction data yet.'."""
        node = _make_validator()
        assert node.get_accuracy_summary() == "No prediction data yet."

    def test_metric_extraction_force(self):
        """Force extraction: sqrt(fx^2+fy^2+fz^2)."""
        msg = _make_machine_state(fx=30.0, fy=40.0, fz=0.0)
        actuals = PredictionValidatorNode._extract_actuals(msg)
        assert actuals['force'] == pytest.approx(50.0, abs=0.01)

    def test_metric_extraction_temperature(self):
        """Temperature extraction from spindle_temp."""
        msg = _make_machine_state(spindle_temp=72.5)
        actuals = PredictionValidatorNode._extract_actuals(msg)
        assert actuals['temperature'] == pytest.approx(72.5)

    def test_metric_extraction_power(self):
        """Power extraction: spindle_load * 10."""
        msg = _make_machine_state(spindle_load=50.0)
        actuals = PredictionValidatorNode._extract_actuals(msg)
        assert actuals['power'] == pytest.approx(500.0)

    def test_thread_safety_concurrent_register(self):
        """Multiple concurrent registrations should not corrupt state."""
        import threading

        node = _make_validator()
        errors = []

        def register_many(mid, n):
            try:
                for i in range(n):
                    node.register_prediction(mid, {'force': float(i)})
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=register_many, args=(f'cnc{t}', 100))
            for t in range(4)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0

    def test_prediction_consumed_after_comparison(self):
        """After comparison, the pending prediction should be consumed."""
        node = _make_validator()
        node.register_prediction('cnc1', {'force': 100.0})
        msg = _make_machine_state(machine_id='cnc1')
        node._on_machine_state(msg)

        # Second message should not produce a second sample
        node._on_machine_state(msg)
        assert len(node.samples) == 1
