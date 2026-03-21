"""Tests for SensorFusionEngine — weighted sensor fusion, disagreement
detection, reliability scoring, and calibration.

Uses the same mock pattern as test_capability_profiler.py so ROS2 modules are
stubbed out.
"""

import sys
from unittest.mock import MagicMock

# Mock ROS2 and miracle_core sub-module dependencies before importing
for _mod in ['rclpy', 'rclpy.node', 'rclpy.callback_groups', 'rclpy.qos', 'rclpy.lifecycle',
             'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
             'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
             'miracle_core.exceptions',
             'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(_mod, MagicMock())

sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = type('FakeNode', (), {
    'CRITICALITY_HIGH': 'HIGH',
    'CRITICALITY_MEDIUM': 'MEDIUM',
    '__init__': lambda self, *a, **kw: None,
    'get_logger': lambda self: MagicMock(),
    'create_publisher': lambda self, *a, **kw: MagicMock(),
    'create_subscription': lambda self, *a, **kw: MagicMock(),
    'create_timer': lambda self, *a, **kw: MagicMock(),
    'declare_and_validate_parameters': lambda self, specs: {k: MagicMock(value=v['default']) for k, v in specs.items()},
    'get_parameter': lambda self, name: MagicMock(value=0),
})

sys.modules.pop('miracle_scada.kpi_calculator', None)

import math
import pytest

from miracle_scada.kpi_calculator import (
    SensorReading,
    FusedEstimate,
    SensorConfig,
    SensorFusionEngine,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _engine_with_two_sensors() -> SensorFusionEngine:
    """Return an engine with two temperature sensors registered."""
    engine = SensorFusionEngine()
    engine.register_sensor(SensorConfig(
        sensor_id='temp_1', sensor_type='temperature',
        weight=1.0, max_uncertainty=5.0, calibration_offset=0.0,
    ))
    engine.register_sensor(SensorConfig(
        sensor_id='temp_2', sensor_type='temperature',
        weight=1.0, max_uncertainty=5.0, calibration_offset=0.0,
    ))
    return engine


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestSensorFusionEngine:

    def test_register_and_submit(self):
        """Readings from a registered sensor are stored."""
        engine = _engine_with_two_sensors()
        engine.submit_reading(SensorReading(
            sensor_id='temp_1', value=25.0, uncertainty=1.0,
            timestamp=1.0, sensor_type='temperature',
        ))
        assert len(engine._readings['temp_1']) == 1

    def test_fused_estimate_single_sensor(self):
        """Fusing with one sensor returns that sensor's value."""
        engine = _engine_with_two_sensors()
        engine.submit_reading(SensorReading(
            sensor_id='temp_1', value=25.0, uncertainty=1.0,
            timestamp=1.0, sensor_type='temperature',
        ))
        est = engine.get_fused_estimate('temperature')
        assert est is not None
        assert math.isclose(est.value, 25.0, abs_tol=1e-6)
        assert est.contributing_sensors == ['temp_1']

    def test_fused_estimate_inverse_variance_weighting(self):
        """Lower-uncertainty sensors dominate the fused value."""
        engine = _engine_with_two_sensors()
        # temp_1: value=20, uncertainty=1 (high precision)
        engine.submit_reading(SensorReading(
            sensor_id='temp_1', value=20.0, uncertainty=1.0,
            timestamp=1.0, sensor_type='temperature',
        ))
        # temp_2: value=30, uncertainty=3 (low precision)
        engine.submit_reading(SensorReading(
            sensor_id='temp_2', value=30.0, uncertainty=3.0,
            timestamp=1.0, sensor_type='temperature',
        ))
        est = engine.get_fused_estimate('temperature')
        assert est is not None
        # Weight for temp_1: 1/1 = 1, weight for temp_2: 1/9 ~ 0.111
        # Fused = (1*20 + 0.111*30) / (1 + 0.111) ~ 21.0
        assert est.value < 25.0, "Fused value should be closer to the more precise sensor"
        assert est.value > 20.0
        assert len(est.contributing_sensors) == 2

    def test_fused_estimate_none_for_unknown_type(self):
        """Returns None when no readings exist for the requested type."""
        engine = _engine_with_two_sensors()
        assert engine.get_fused_estimate('pressure') is None

    def test_reject_high_uncertainty_reading(self):
        """Readings exceeding max_uncertainty are rejected."""
        engine = _engine_with_two_sensors()
        engine.submit_reading(SensorReading(
            sensor_id='temp_1', value=25.0, uncertainty=10.0,  # > max 5.0
            timestamp=1.0, sensor_type='temperature',
        ))
        assert len(engine._readings['temp_1']) == 0

    def test_detect_sensor_disagreement(self):
        """A sensor far from the fused estimate is flagged."""
        engine = _engine_with_two_sensors()
        engine.register_sensor(SensorConfig(
            sensor_id='temp_3', sensor_type='temperature',
            weight=1.0, max_uncertainty=100.0, calibration_offset=0.0,
        ))
        engine.submit_reading(SensorReading(
            sensor_id='temp_1', value=25.0, uncertainty=1.0,
            timestamp=1.0, sensor_type='temperature',
        ))
        engine.submit_reading(SensorReading(
            sensor_id='temp_2', value=25.5, uncertainty=1.0,
            timestamp=1.0, sensor_type='temperature',
        ))
        engine.submit_reading(SensorReading(
            sensor_id='temp_3', value=50.0, uncertainty=1.0,
            timestamp=1.0, sensor_type='temperature',
        ))
        disagreements = engine.detect_sensor_disagreement('temperature', threshold=2.0)
        ids = [d['sensor_id'] for d in disagreements]
        assert 'temp_3' in ids

    def test_sensor_reliability_perfect(self):
        """An unused sensor has reliability 1.0."""
        engine = _engine_with_two_sensors()
        assert engine.get_sensor_reliability('temp_1') == 1.0

    def test_sensor_reliability_degrades(self):
        """Reliability degrades when a sensor consistently deviates."""
        engine = _engine_with_two_sensors()
        # Submit agreeing readings to build history
        for _ in range(5):
            engine.submit_reading(SensorReading(
                sensor_id='temp_1', value=25.0, uncertainty=1.0,
                timestamp=1.0, sensor_type='temperature',
            ))
            engine.submit_reading(SensorReading(
                sensor_id='temp_2', value=35.0, uncertainty=1.0,
                timestamp=1.0, sensor_type='temperature',
            ))
            engine.get_fused_estimate('temperature')  # triggers reliability tracking

        r1 = engine.get_sensor_reliability('temp_1')
        r2 = engine.get_sensor_reliability('temp_2')
        # Both deviate from the fused mean symmetrically, so both < 1.0
        assert r1 < 1.0
        assert r2 < 1.0

    def test_calibrate_sensor_applies_offset(self):
        """calibrate_sensor adds offset and future readings use it."""
        engine = _engine_with_two_sensors()
        engine.calibrate_sensor('temp_1', offset=2.0)
        engine.submit_reading(SensorReading(
            sensor_id='temp_1', value=25.0, uncertainty=1.0,
            timestamp=1.0, sensor_type='temperature',
        ))
        # Stored value should be 25.0 + 2.0 = 27.0
        assert math.isclose(engine._readings['temp_1'][0].value, 27.0, abs_tol=1e-6)

    def test_calibrate_unregistered_sensor_raises(self):
        """Calibrating an unregistered sensor raises ValueError."""
        engine = SensorFusionEngine()
        with pytest.raises(ValueError):
            engine.calibrate_sensor('unknown', offset=1.0)

    def test_fused_uncertainty_decreases_with_more_sensors(self):
        """Fused uncertainty decreases when more sensors contribute."""
        engine = SensorFusionEngine()
        for i in range(5):
            sid = f'temp_{i}'
            engine.register_sensor(SensorConfig(
                sensor_id=sid, sensor_type='temperature',
                weight=1.0, max_uncertainty=5.0, calibration_offset=0.0,
            ))
            engine.submit_reading(SensorReading(
                sensor_id=sid, value=25.0, uncertainty=2.0,
                timestamp=1.0, sensor_type='temperature',
            ))

        est_all = engine.get_fused_estimate('temperature')
        # With 5 identical sensors at uncertainty=2, fused uncertainty = 1/sqrt(5/4) < 2
        assert est_all is not None
        assert est_all.uncertainty < 2.0
