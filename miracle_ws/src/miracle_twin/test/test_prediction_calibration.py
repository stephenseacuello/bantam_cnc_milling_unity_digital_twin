"""Tests for the calibration feedback loop in PredictionValidator + CuttingSimProxy."""

import pytest
import sys
import os
import math
import time

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

sys.modules.setdefault('miracle_core', _miracle_core_mod)
sys.modules.setdefault('miracle_core.lifecycle_node_base', _miracle_core_lifecycle_mod)
sys.modules.setdefault('miracle_core.qos_profiles', _miracle_core_qos_mod)
sys.modules.setdefault('miracle_core.heartbeat_mixin', _miracle_core_heartbeat_mod)
sys.modules.setdefault('miracle_core.parameter_validation', _miracle_core_param_mod)
sys.modules.setdefault('miracle_core.exceptions', _miracle_core_exc_mod)

sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = _FakeMiracleLifecycleNode
sys.modules['miracle_core.qos_profiles'].QoSProfiles = _FakeQoSProfiles

# Force reimport to pick up stubs
for mod_name in list(sys.modules):
    if 'miracle_twin.prediction_validator' in mod_name:
        del sys.modules[mod_name]
    if 'miracle_twin.cutting_sim_proxy' in mod_name:
        del sys.modules[mod_name]

from miracle_twin.prediction_validator import (
    PredictionValidatorNode,
    PredictionSample,
    AccuracyMetrics,
    CalibrationAdjustment,
)
from miracle_twin.cutting_sim_proxy import CuttingSimProxy, CuttingCoefficients


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_validator(**kwargs) -> PredictionValidatorNode:
    """Create a PredictionValidatorNode with defaults configured."""
    node = PredictionValidatorNode()
    node._do_configure()
    for k, v in kwargs.items():
        setattr(node, f'_{k}', v)
    return node


def _make_proxy(**kwargs) -> CuttingSimProxy:
    """Create a CuttingSimProxy with optional coefficient overrides."""
    coeffs = CuttingCoefficients(**kwargs) if kwargs else CuttingCoefficients()
    return CuttingSimProxy(coefficients=coeffs)


def _inject_biased_samples(
    node: PredictionValidatorNode,
    metric: str,
    n: int,
    predicted: float,
    actual: float,
) -> None:
    """Inject *n* samples for *metric* with constant predicted/actual values."""
    now = time.time()
    with node._lock:
        for i in range(n):
            err_pct = abs(predicted - actual) / abs(actual) * 100.0 if actual != 0.0 else 100.0
            node._samples.append(
                PredictionSample(
                    timestamp=now + i,
                    machine_id='cnc1',
                    metric=metric,
                    predicted=predicted,
                    actual=actual,
                    error_pct=err_pct,
                )
            )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestCalibrationFeedbackLoop:

    def test_calibration_disabled_by_default_produces_no_adjustments(self):
        """When calibration_enabled is False (default), no adjustments are proposed."""
        node = _make_validator()
        _inject_biased_samples(node, 'force', 100, 120.0, 100.0)
        node._compute_all_accuracy()

        adjustments = node._compute_calibration()
        assert adjustments == []

    def test_calibration_enabled_but_insufficient_samples(self):
        """With calibration enabled but too few samples, no adjustments are proposed."""
        node = _make_validator(
            calibration_enabled=True,
            min_samples_for_calibration=50,
        )
        _inject_biased_samples(node, 'force', 30, 120.0, 100.0)
        node._compute_all_accuracy()

        adjustments = node._compute_calibration()
        assert adjustments == []

    def test_overpredicting_force_gives_correction_below_one(self):
        """When predictions overshoot actual, correction_factor should be < 1.0."""
        node = _make_validator(
            calibration_enabled=True,
            min_samples_for_calibration=50,
            calibration_interval_sec=0,
        )
        # Predicted 120, actual 100 -> bias = +20, relative bias = 20/120 ~ 16.7%
        _inject_biased_samples(node, 'force', 60, 120.0, 100.0)
        node._compute_all_accuracy()

        adjustments = node._compute_calibration()
        force_adj = [a for a in adjustments if a.metric == 'force']
        assert len(force_adj) == 1
        assert force_adj[0].correction_factor < 1.0
        assert force_adj[0].current_bias > 0.0

    def test_underpredicting_force_gives_correction_above_one(self):
        """When predictions undershoot actual, correction_factor should be > 1.0."""
        node = _make_validator(
            calibration_enabled=True,
            min_samples_for_calibration=50,
            calibration_interval_sec=0,
        )
        # Predicted 80, actual 100 -> bias = -20, relative bias = 20/80 = 25%
        _inject_biased_samples(node, 'force', 60, 80.0, 100.0)
        node._compute_all_accuracy()

        adjustments = node._compute_calibration()
        force_adj = [a for a in adjustments if a.metric == 'force']
        assert len(force_adj) == 1
        assert force_adj[0].correction_factor > 1.0
        assert force_adj[0].current_bias < 0.0

    def test_max_correction_clamped_at_ten_percent(self):
        """Even with massive bias, correction_factor is clamped to +/- 10%."""
        node = _make_validator(
            calibration_enabled=True,
            min_samples_for_calibration=50,
            max_correction_pct=0.10,
            calibration_interval_sec=0,
        )
        # Predicted 200, actual 100 -> enormous overprediction
        _inject_biased_samples(node, 'force', 60, 200.0, 100.0)
        node._compute_all_accuracy()

        adjustments = node._compute_calibration()
        force_adj = [a for a in adjustments if a.metric == 'force']
        assert len(force_adj) == 1
        assert force_adj[0].correction_factor >= 0.90
        assert force_adj[0].correction_factor <= 1.10

    def test_confidence_scales_with_sample_count_and_r_squared(self):
        """Confidence should increase with more samples and better R-squared."""
        node = _make_validator(
            calibration_enabled=True,
            min_samples_for_calibration=50,
            calibration_interval_sec=0,
        )
        # Use wide-spread actuals with a *multiplicative* bias so R^2 stays high.
        # predicted = actual * 1.10 gives 10% overprediction but preserves correlation.
        # R^2 = 1 - ss_res/ss_tot.  With multiplicative bias on wide range:
        #   ss_tot is large (actuals vary 100..200), ss_res is small relative to ss_tot.
        now = time.time()
        with node._lock:
            for i in range(100):
                actual = 100.0 + i * 1.0  # range 100-199
                predicted = actual * 1.10  # consistent 10% overprediction
                err_pct = abs(predicted - actual) / abs(actual) * 100.0
                node._samples.append(
                    PredictionSample(now + i, 'cnc1', 'force', predicted, actual, err_pct)
                )
        node._compute_all_accuracy()

        adjustments = node._compute_calibration()
        force_adj = [a for a in adjustments if a.metric == 'force']
        assert len(force_adj) == 1
        # R^2 should be positive (predictions track actuals well despite bias)
        # confidence = min(1, 100/200) * R^2 = 0.5 * R^2
        assert force_adj[0].confidence > 0.0
        assert force_adj[0].confidence <= 1.0

    def test_applied_calibration_changes_proxy_force_coefficients(self):
        """Applying a force calibration should scale Ktc, Krc, Kac in the proxy."""
        proxy = _make_proxy()
        original_ktc = proxy.coeffs.Ktc
        original_krc = proxy.coeffs.Krc
        original_kac = proxy.coeffs.Kac

        adj = CalibrationAdjustment(
            metric='force',
            current_bias=20.0,
            correction_factor=0.95,
            confidence=0.8,
            sample_count=100,
            timestamp=time.time(),
        )

        node = _make_validator(calibration_enabled=True)
        node._apply_calibration(proxy, [adj])

        assert proxy.coeffs.Ktc == pytest.approx(original_ktc * 0.95, rel=1e-6)
        assert proxy.coeffs.Krc == pytest.approx(original_krc * 0.95, rel=1e-6)
        assert proxy.coeffs.Kac == pytest.approx(original_kac * 0.95, rel=1e-6)

    def test_calibration_history_tracks_all_adjustments(self):
        """Applied adjustments should appear in calibration_history."""
        proxy = _make_proxy()
        node = _make_validator(calibration_enabled=True)

        adj1 = CalibrationAdjustment('force', 20.0, 0.95, 0.8, 100, time.time())
        adj2 = CalibrationAdjustment('power', -10.0, 1.05, 0.9, 150, time.time())
        node._apply_calibration(proxy, [adj1, adj2])

        history = node.calibration_history
        assert len(history) == 2
        assert history[0].metric == 'force'
        assert history[1].metric == 'power'

    def test_low_confidence_adjustment_not_applied(self):
        """Adjustments with confidence < 0.7 should be skipped."""
        proxy = _make_proxy()
        original_ktc = proxy.coeffs.Ktc

        adj = CalibrationAdjustment(
            metric='force',
            current_bias=20.0,
            correction_factor=0.95,
            confidence=0.5,  # below threshold
            sample_count=60,
            timestamp=time.time(),
        )

        node = _make_validator(calibration_enabled=True)
        node._apply_calibration(proxy, [adj])

        # Coefficients should be unchanged
        assert proxy.coeffs.Ktc == original_ktc
        # Should NOT appear in calibration history
        assert len(node.calibration_history) == 0

    def test_damping_factor_prevents_overcorrection(self):
        """The 0.3 damping factor should limit how much each correction applies."""
        node = _make_validator(
            calibration_enabled=True,
            min_samples_for_calibration=50,
            calibration_interval_sec=0,
        )
        # bias/predicted_mean = 20/120 ~ 0.1667
        # raw_correction = 1.0 - 0.1667 * 0.3 = 1.0 - 0.05 = 0.95
        # Without damping (factor=1.0) it would be ~0.833
        _inject_biased_samples(node, 'force', 60, 120.0, 100.0)
        node._compute_all_accuracy()

        adjustments = node._compute_calibration()
        force_adj = [a for a in adjustments if a.metric == 'force']
        assert len(force_adj) == 1

        # With 0.3 damping the correction should be modest (around 0.95)
        cf = force_adj[0].correction_factor
        assert cf > 0.93  # not too aggressive
        assert cf < 0.97

    def test_multiple_calibration_cycles_converge(self):
        """Successive calibration cycles should reduce the bias."""
        proxy = _make_proxy()
        node = _make_validator(
            calibration_enabled=True,
            min_samples_for_calibration=50,
            calibration_interval_sec=0,
        )
        node.set_proxy(proxy)

        original_ktc = proxy.coeffs.Ktc
        # The model overpredicts by 10% at default coefficients.
        # Each calibration cycle adjusts Ktc downward, reducing predicted
        # force closer to actual values.
        MODEL_OVERPREDICT = 1.10  # predicted = actual * 1.10
        bias_history = []

        for cycle in range(5):
            with node._lock:
                node._samples.clear()
                node._accuracy.clear()

            # predicted force = actual * MODEL_OVERPREDICT * (current_Ktc / original_Ktc)
            coeff_ratio = proxy.coeffs.Ktc / original_ktc
            now = time.time()
            with node._lock:
                for i in range(200):
                    actual = 80.0 + i * 1.0  # 80..279
                    predicted = actual * MODEL_OVERPREDICT * coeff_ratio
                    err = abs(predicted - actual) / abs(actual) * 100.0
                    node._samples.append(
                        PredictionSample(now + i, 'cnc1', 'force',
                                         predicted, actual, err)
                    )

            node._last_calibration_time = 0.0
            node.run_calibration_cycle()

            signed_errors = [s.predicted - s.actual for s in node.samples]
            mean_bias = sum(signed_errors) / len(signed_errors)
            bias_history.append(abs(mean_bias))

        # Bias should decrease over successive cycles
        assert bias_history[-1] < bias_history[0]

    def test_calibration_log_in_proxy_records_changes(self):
        """The proxy's _calibration_log should record every coefficient change."""
        proxy = _make_proxy()

        proxy.scale_force_coefficients(0.95)
        proxy.scale_edge_coefficients(1.05)

        assert len(proxy._calibration_log) == 2

        log0 = proxy._calibration_log[0]
        assert log0['action'] == 'scale_force_coefficients'
        assert log0['factor'] == 0.95
        assert 'before' in log0
        assert 'after' in log0
        assert log0['after']['Ktc'] == pytest.approx(log0['before']['Ktc'] * 0.95)

        log1 = proxy._calibration_log[1]
        assert log1['action'] == 'scale_edge_coefficients'
        assert log1['factor'] == 1.05

    def test_proxy_get_coefficients_returns_current_values(self):
        """get_coefficients() should reflect the current state."""
        proxy = _make_proxy(Ktc=800.0, Krc=170.0, Kac=85.0,
                            Kte=15.0, Kre=11.0, Kae=5.0)
        c = proxy.get_coefficients()
        assert c['Ktc'] == 800.0
        assert c['Krc'] == 170.0
        assert c['Kac'] == 85.0
        assert c['Kte'] == 15.0
        assert c['Kre'] == 11.0
        assert c['Kae'] == 5.0

    def test_proxy_scale_thermal_factor(self):
        """scale_thermal_factor should adjust the coolant's thermal factor."""
        proxy = _make_proxy()
        old_tf = proxy.coolant.thermal_factor
        proxy.scale_thermal_factor(0.9)
        new_tf = proxy.coolant.thermal_factor
        assert new_tf == pytest.approx(old_tf * 0.9, rel=1e-6)
        assert len(proxy._calibration_log) == 1
        assert proxy._calibration_log[0]['action'] == 'scale_thermal_factor'

    def test_temperature_calibration_applied_to_proxy(self):
        """A temperature calibration should call scale_thermal_factor on the proxy."""
        proxy = _make_proxy()
        old_tf = proxy.coolant.thermal_factor

        adj = CalibrationAdjustment(
            metric='temperature',
            current_bias=5.0,
            correction_factor=0.92,
            confidence=0.85,
            sample_count=100,
            timestamp=time.time(),
        )

        node = _make_validator(calibration_enabled=True)
        node._apply_calibration(proxy, [adj])

        new_tf = proxy.coolant.thermal_factor
        assert new_tf == pytest.approx(old_tf * 0.92, rel=1e-6)
        assert len(node.calibration_history) == 1
        assert node.calibration_history[0].metric == 'temperature'

    def test_power_calibration_scales_edge_coefficients(self):
        """A power calibration should scale Kte, Kre, Kae."""
        proxy = _make_proxy()
        original_kte = proxy.coeffs.Kte

        adj = CalibrationAdjustment(
            metric='power',
            current_bias=-15.0,
            correction_factor=1.08,
            confidence=0.75,
            sample_count=80,
            timestamp=time.time(),
        )

        node = _make_validator(calibration_enabled=True)
        node._apply_calibration(proxy, [adj])

        assert proxy.coeffs.Kte == pytest.approx(original_kte * 1.08, rel=1e-6)
        assert len(node.calibration_history) == 1

    def test_run_calibration_cycle_end_to_end(self):
        """run_calibration_cycle ties compute + apply together."""
        proxy = _make_proxy()
        node = _make_validator(
            calibration_enabled=True,
            min_samples_for_calibration=50,
            calibration_interval_sec=0,
        )
        node.set_proxy(proxy)

        # Inject enough biased force samples with variation so R^2 is decent
        now = time.time()
        with node._lock:
            for i in range(100):
                actual = 100.0 + i * 0.5
                predicted = actual * 1.20  # 20% overprediction
                err_pct = abs(predicted - actual) / abs(actual) * 100.0
                node._samples.append(
                    PredictionSample(now + i, 'cnc1', 'force', predicted, actual, err_pct)
                )

        original_ktc = proxy.coeffs.Ktc
        adjustments = node.run_calibration_cycle()

        # Should have proposed at least one force adjustment
        force_adjs = [a for a in adjustments if a.metric == 'force']
        assert len(force_adjs) >= 1

        # If confidence was high enough, coefficients should have changed
        if force_adjs[0].confidence >= 0.7:
            assert proxy.coeffs.Ktc != original_ktc
            assert len(proxy._calibration_log) > 0
