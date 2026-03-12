"""Tests for predictive anomaly markers in prediction_runner and cutting_sim_proxy."""
import sys
from unittest.mock import MagicMock

# Mock ROS2 and miracle_core submodules ONLY
for mod in ['rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.action.server',
            'rclpy.callback_groups', 'rclpy.qos', 'rclpy.lifecycle',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
            'miracle_core.exceptions',
            'miracle_msgs', 'miracle_msgs.msg', 'miracle_msgs.srv',
            'miracle_msgs.action']:
    sys.modules.setdefault(mod, MagicMock())

# Force-set attributes on whatever module is installed
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

# Ensure msg/srv/action modules have required attributes
for _mod_name in ('miracle_msgs.action', 'miracle_msgs.srv', 'miracle_msgs.msg'):
    _mod = sys.modules[_mod_name]
    for _attr in ('RunPrediction', 'RunPredictionSrv', 'PredictionResult',
                  'AnomalyAlert', 'MachineState', 'FeedOverride',
                  'StabilityRecommendation', 'InferredAction', 'OperatorFeedback'):
        if not hasattr(_mod, _attr):
            setattr(_mod, _attr, MagicMock())

# Force reimport target modules
sys.modules.pop('miracle_twin.prediction_runner', None)
sys.modules.pop('miracle_twin.cutting_sim_proxy', None)

import pytest
from miracle_twin.cutting_sim_proxy import (
    BlockPrediction,
    CuttingSimProxy,
    GCodeBlock,
    SimulationResult,
)
from miracle_twin.prediction_runner import AnomalyMarker, PredictionRunnerNode


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_sim_result(
    block_predictions=None,
    rul_hours=100.0,
):
    """Build a SimulationResult with given block predictions."""
    bps = block_predictions or []
    return SimulationResult(
        block_predictions=bps,
        total_cutting_time_min=10.0,
        final_wear_mm=0.05,
        remaining_useful_life_hours=rul_hours,
        confidence=0.85,
        health_index=0.9,
        trend_data=[0.9],
        recommended_action='Continue normal operation',
    )


def _make_blocks(n=1, rpm=8000.0, feed=500.0):
    """Create n identical GCodeBlocks."""
    return [
        GCodeBlock(
            feed_rate_mmpm=feed,
            spindle_rpm=rpm,
            axial_depth_mm=1.5,
            radial_depth_mm=3.175,
            length_mm=50.0,
        )
        for _ in range(n)
    ]


# ===========================================================================
# Test AnomalyMarker dataclass
# ===========================================================================

class TestAnomalyMarkerDataclass:
    def test_create_anomaly_marker(self):
        m = AnomalyMarker(
            block_index=0,
            gcode_line='G1 F500 S8000 (block 0)',
            marker_type='FORCE_CRITICAL',
            severity=0.8,
            predicted_value=200.0,
            threshold=180.0,
            recommendation='Reduce feed rate',
        )
        assert m.block_index == 0
        assert m.marker_type == 'FORCE_CRITICAL'
        assert m.severity == 0.8
        assert m.predicted_value == 200.0
        assert m.threshold == 180.0


# ===========================================================================
# Test _generate_recommendation
# ===========================================================================

class TestGenerateRecommendation:
    def test_force_critical_recommendation(self):
        rec = PredictionRunnerNode._generate_recommendation('FORCE_CRITICAL', 250.0, 180.0)
        assert 'feed rate' in rec.lower() or 'depth' in rec.lower()
        assert 'exceeded by' in rec.lower()

    def test_thermal_warning_recommendation(self):
        rec = PredictionRunnerNode._generate_recommendation('THERMAL_WARNING', 20.0, 15.0)
        assert 'coolant' in rec.lower() or 'speed' in rec.lower()

    def test_wear_accelerated_recommendation(self):
        rec = PredictionRunnerNode._generate_recommendation('WEAR_ACCELERATED', 0.01, 0.005)
        assert 'tool' in rec.lower()

    def test_chatter_risk_recommendation(self):
        rec = PredictionRunnerNode._generate_recommendation('CHATTER_RISK', 1.0, 0.8)
        assert 'spindle' in rec.lower() or 'speed' in rec.lower()

    def test_tool_end_of_life_recommendation(self):
        rec = PredictionRunnerNode._generate_recommendation('TOOL_END_OF_LIFE', 10.0, 30.0)
        assert 'replace' in rec.lower() or 'tool' in rec.lower()

    def test_surface_quality_recommendation(self):
        rec = PredictionRunnerNode._generate_recommendation('SURFACE_QUALITY_WARNING', 4.0, 3.2)
        assert 'feed' in rec.lower() or 'nose' in rec.lower()

    def test_unknown_marker_type(self):
        rec = PredictionRunnerNode._generate_recommendation('UNKNOWN_TYPE', 100.0, 50.0)
        assert 'review' in rec.lower()


# ===========================================================================
# Test _identify_anomaly_risks — Force markers
# ===========================================================================

class TestForceAnomalyMarkers:
    def test_force_critical_above_threshold(self):
        """Force >= 100% of threshold -> FORCE_CRITICAL."""
        bp = BlockPrediction(peak_force_n=200.0)
        sim = _make_sim_result([bp])
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(
            blocks, sim, force_threshold=180.0,
        )
        types = [m.marker_type for m in markers]
        assert 'FORCE_CRITICAL' in types

    def test_force_warning_at_80_percent(self):
        """Force >= 80% and < 100% of threshold -> FORCE_WARNING."""
        bp = BlockPrediction(peak_force_n=150.0)  # 150/180 = 83%
        sim = _make_sim_result([bp])
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(
            blocks, sim, force_threshold=180.0,
        )
        types = [m.marker_type for m in markers]
        assert 'FORCE_WARNING' in types
        assert 'FORCE_CRITICAL' not in types

    def test_no_force_marker_below_80_percent(self):
        """Force < 80% of threshold -> no force marker."""
        bp = BlockPrediction(peak_force_n=100.0)  # 100/180 = 55%
        sim = _make_sim_result([bp])
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(
            blocks, sim, force_threshold=180.0,
        )
        force_markers = [m for m in markers if 'FORCE' in m.marker_type]
        assert len(force_markers) == 0


# ===========================================================================
# Test _identify_anomaly_risks — Temperature markers
# ===========================================================================

class TestThermalAnomalyMarkers:
    def test_thermal_critical_above_25c(self):
        bp = BlockPrediction(temperature_rise_c=30.0)
        sim = _make_sim_result([bp])
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        types = [m.marker_type for m in markers]
        assert 'THERMAL_CRITICAL' in types

    def test_thermal_warning_between_15_and_25c(self):
        bp = BlockPrediction(temperature_rise_c=20.0)
        sim = _make_sim_result([bp])
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        types = [m.marker_type for m in markers]
        assert 'THERMAL_WARNING' in types
        assert 'THERMAL_CRITICAL' not in types

    def test_no_thermal_marker_below_15c(self):
        bp = BlockPrediction(temperature_rise_c=10.0)
        sim = _make_sim_result([bp])
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        thermal = [m for m in markers if 'THERMAL' in m.marker_type]
        assert len(thermal) == 0


# ===========================================================================
# Test _identify_anomaly_risks — Wear markers
# ===========================================================================

class TestWearAnomalyMarkers:
    def test_wear_accelerated_above_threshold(self):
        """Wear rate > 0.005 mm/block -> WEAR_ACCELERATED."""
        bp = BlockPrediction(wear_after_block_mm=0.01)  # first block, prev=0 -> rate=0.01
        sim = _make_sim_result([bp])
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        types = [m.marker_type for m in markers]
        assert 'WEAR_ACCELERATED' in types

    def test_no_wear_marker_below_threshold(self):
        bp = BlockPrediction(wear_after_block_mm=0.003)  # rate=0.003 < 0.005
        sim = _make_sim_result([bp])
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        wear = [m for m in markers if m.marker_type == 'WEAR_ACCELERATED']
        assert len(wear) == 0


# ===========================================================================
# Test _identify_anomaly_risks — Chatter risk
# ===========================================================================

class TestChatterAnomalyMarkers:
    def test_chatter_risk_at_unstable_rpm(self):
        """RPM near stability lobe boundary -> CHATTER_RISK."""
        # unstable_rpm = 60 * 250 / (k+1) for k=1 -> 7500
        bp = BlockPrediction(peak_force_n=50.0)
        sim = _make_sim_result([bp])
        blocks = _make_blocks(1, rpm=7500.0)
        markers = PredictionRunnerNode._identify_anomaly_risks(
            blocks, sim, chatter_threshold=0.8,
        )
        types = [m.marker_type for m in markers]
        assert 'CHATTER_RISK' in types

    def test_no_chatter_at_stable_rpm(self):
        bp = BlockPrediction(peak_force_n=50.0)
        sim = _make_sim_result([bp])
        blocks = _make_blocks(1, rpm=8000.0)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        chatter = [m for m in markers if m.marker_type == 'CHATTER_RISK']
        assert len(chatter) == 0


# ===========================================================================
# Test _identify_anomaly_risks — Surface quality
# ===========================================================================

class TestSurfaceQualityMarkers:
    def test_surface_quality_warning_above_threshold(self):
        bp = BlockPrediction(surface_ra_um=4.0)
        sim = _make_sim_result([bp])
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        types = [m.marker_type for m in markers]
        assert 'SURFACE_QUALITY_WARNING' in types

    def test_no_surface_marker_below_threshold(self):
        bp = BlockPrediction(surface_ra_um=2.0)
        sim = _make_sim_result([bp])
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        surface = [m for m in markers if m.marker_type == 'SURFACE_QUALITY_WARNING']
        assert len(surface) == 0


# ===========================================================================
# Test _identify_anomaly_risks — Tool end of life
# ===========================================================================

class TestToolEndOfLifeMarkers:
    def test_tool_end_of_life_below_30_min(self):
        bp = BlockPrediction(peak_force_n=50.0)
        sim = _make_sim_result([bp], rul_hours=0.3)  # 18 min < 30 min
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        types = [m.marker_type for m in markers]
        assert 'TOOL_END_OF_LIFE' in types

    def test_no_tool_eol_above_30_min(self):
        bp = BlockPrediction(peak_force_n=50.0)
        sim = _make_sim_result([bp], rul_hours=2.0)  # 120 min > 30 min
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        eol = [m for m in markers if m.marker_type == 'TOOL_END_OF_LIFE']
        assert len(eol) == 0


# ===========================================================================
# Test severity calculation
# ===========================================================================

class TestSeverityCalculation:
    def test_force_critical_severity_scales(self):
        """Higher force -> higher severity."""
        bp_low = BlockPrediction(peak_force_n=185.0)
        bp_high = BlockPrediction(peak_force_n=270.0)
        sim_low = _make_sim_result([bp_low])
        sim_high = _make_sim_result([bp_high])
        blocks = _make_blocks(1)

        markers_low = PredictionRunnerNode._identify_anomaly_risks(blocks, sim_low)
        markers_high = PredictionRunnerNode._identify_anomaly_risks(blocks, sim_high)

        crit_low = [m for m in markers_low if m.marker_type == 'FORCE_CRITICAL']
        crit_high = [m for m in markers_high if m.marker_type == 'FORCE_CRITICAL']
        assert len(crit_low) > 0
        assert len(crit_high) > 0
        assert crit_high[0].severity > crit_low[0].severity

    def test_severity_bounded_0_to_1(self):
        """All severity values should be in [0, 1]."""
        bp = BlockPrediction(
            peak_force_n=500.0,
            temperature_rise_c=100.0,
            wear_after_block_mm=0.1,
            surface_ra_um=10.0,
        )
        sim = _make_sim_result([bp], rul_hours=0.01)
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        for m in markers:
            assert 0.0 <= m.severity <= 1.0, f"{m.marker_type} severity={m.severity}"


# ===========================================================================
# Test safe blocks produce no markers
# ===========================================================================

class TestSafeBlocks:
    def test_no_markers_for_safe_blocks(self):
        """Low-value block predictions should produce zero markers."""
        bp = BlockPrediction(
            peak_force_n=50.0,
            temperature_rise_c=5.0,
            wear_after_block_mm=0.001,
            surface_ra_um=1.0,
        )
        sim = _make_sim_result([bp], rul_hours=10.0)
        blocks = _make_blocks(1, rpm=8000.0)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        assert len(markers) == 0


# ===========================================================================
# Test multiple markers on same block
# ===========================================================================

class TestMultipleMarkersPerBlock:
    def test_multiple_markers_on_same_block(self):
        """A block exceeding multiple thresholds should get multiple markers."""
        bp = BlockPrediction(
            peak_force_n=200.0,       # FORCE_CRITICAL
            temperature_rise_c=30.0,  # THERMAL_CRITICAL
            wear_after_block_mm=0.01, # WEAR_ACCELERATED
            surface_ra_um=5.0,        # SURFACE_QUALITY_WARNING
        )
        sim = _make_sim_result([bp], rul_hours=0.1)  # TOOL_END_OF_LIFE
        blocks = _make_blocks(1)
        markers = PredictionRunnerNode._identify_anomaly_risks(blocks, sim)
        types = {m.marker_type for m in markers}
        assert 'FORCE_CRITICAL' in types
        assert 'THERMAL_CRITICAL' in types
        assert 'WEAR_ACCELERATED' in types
        assert 'SURFACE_QUALITY_WARNING' in types
        assert 'TOOL_END_OF_LIFE' in types
        assert len(markers) >= 5


# ===========================================================================
# Test get_block_anomaly_indicators on CuttingSimProxy
# ===========================================================================

class TestGetBlockAnomalyIndicators:
    def test_returns_all_indicator_keys(self):
        proxy = CuttingSimProxy()
        result = {
            'peak_force_n': 150.0,
            'temperature_rise_c': 20.0,
            'wear_after_block_mm': 0.05,
            'surface_ra_um': 2.5,
            'spindle_rpm': 8000.0,
        }
        indicators = proxy.get_block_anomaly_indicators(result)
        assert 'peak_force_ratio' in indicators
        assert 'temp_rise_ratio' in indicators
        assert 'wear_rate' in indicators
        assert 'chatter_risk_score' in indicators
        assert 'surface_ra_um' in indicators

    def test_force_ratio_calculation(self):
        proxy = CuttingSimProxy()
        result = {'peak_force_n': 90.0}
        indicators = proxy.get_block_anomaly_indicators(result, force_threshold=180.0)
        assert abs(indicators['peak_force_ratio'] - 0.5) < 0.01

    def test_temp_rise_ratio_calculation(self):
        proxy = CuttingSimProxy()
        result = {'temperature_rise_c': 12.5}
        indicators = proxy.get_block_anomaly_indicators(result, temp_threshold=25.0)
        assert abs(indicators['temp_rise_ratio'] - 0.5) < 0.01

    def test_defaults_for_missing_keys(self):
        proxy = CuttingSimProxy()
        indicators = proxy.get_block_anomaly_indicators({})
        assert indicators['peak_force_ratio'] == 0.0
        assert indicators['temp_rise_ratio'] == 0.0
        assert indicators['wear_rate'] == 0.0
        assert indicators['chatter_risk_score'] == 0.0
        assert indicators['surface_ra_um'] == 0.0

    def test_chatter_risk_passed_through(self):
        proxy = CuttingSimProxy()
        # RPM=7500 is near stability lobe boundary (60*250/2 = 7500)
        result = {'spindle_rpm': 7500.0}
        indicators = proxy.get_block_anomaly_indicators(result)
        assert indicators['chatter_risk_score'] > 0.0
