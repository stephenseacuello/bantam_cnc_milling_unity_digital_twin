"""Tests for prediction-based anomaly marker scheduling in the Adaptive Controller."""

import pytest
import sys
import os
from unittest.mock import MagicMock
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# Mock ROS 2 and miracle dependencies before importing the module under test
# ---------------------------------------------------------------------------
for mod in ['rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.callback_groups',
            'rclpy.qos', 'rclpy.lifecycle',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
            'miracle_core.exceptions',
            'miracle_msgs', 'miracle_msgs.msg', 'miracle_msgs.srv',
            'miracle_msgs.action']:
    sys.modules.setdefault(mod, MagicMock())

sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = type('FakeNode', (), {
    'CRITICALITY_HIGH': 'HIGH',
    'CRITICALITY_MEDIUM': 'MEDIUM',
    'CRITICALITY_LOW': 'LOW',
    '__init__': lambda self, *a, **kw: None,
    'get_logger': lambda self: MagicMock(),
    'create_publisher': lambda self, *a, **kw: MagicMock(),
    'create_subscription': lambda self, *a, **kw: MagicMock(),
    'create_timer': lambda self, *a, **kw: MagicMock(),
    'declare_and_validate_parameters': lambda self, specs: {k: MagicMock(value=v['default']) for k, v in specs.items()},
    'get_parameter': lambda self, name: MagicMock(value=0),
})

for _mod_name in ('miracle_msgs.action', 'miracle_msgs.srv', 'miracle_msgs.msg'):
    _mod = sys.modules[_mod_name]
    for _attr in ('RunPrediction', 'PredictionResult', 'AnomalyAlert', 'MachineState',
                  'FeedOverride', 'StabilityRecommendation', 'InferredAction', 'OperatorFeedback'):
        if not hasattr(_mod, _attr):
            setattr(_mod, _attr, MagicMock())

sys.modules.pop('miracle_twin.adaptive_controller', None)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_twin.adaptive_controller import (
    AdaptiveControllerNode,
    PredictiveOverride,
    PreemptiveAction,
    AdaptiveState,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

@dataclass
class FakeMarker:
    """Simulates an anomaly marker from the prediction pipeline."""
    marker_type: str
    block_index: int
    severity: float = 0.7


def make_controller() -> AdaptiveControllerNode:
    """Create a controller instance with mocked ROS infrastructure."""
    node = AdaptiveControllerNode()
    node._predictive_overrides = []
    node._target_feed_override = 100.0
    node._target_spindle_override = 100.0
    return node


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestPredictiveOverrideDataclass:
    def test_creation(self):
        ov = PredictiveOverride(
            trigger_block_index=10,
            marker_type='FORCE_CRITICAL',
            predicted_severity=0.9,
            feed_override_pct=70.0,
            speed_override_pct=100.0,
            reason='test reason',
            expires_at_block=13,
            confidence=0.9,
        )
        assert ov.trigger_block_index == 10
        assert ov.marker_type == 'FORCE_CRITICAL'
        assert ov.feed_override_pct == 70.0
        assert ov.speed_override_pct == 100.0
        assert ov.expires_at_block == 13
        assert ov.confidence == 0.9

    def test_defaults_are_explicit(self):
        """PredictiveOverride has no defaults — all fields required."""
        with pytest.raises(TypeError):
            PredictiveOverride()  # type: ignore[call-arg]


class TestProcessAnomalyMarkers:
    def test_force_critical_generates_70pct_feed(self):
        ctrl = make_controller()
        markers = [FakeMarker('FORCE_CRITICAL', block_index=50, severity=0.7)]
        overrides = ctrl._process_anomaly_markers(markers)
        assert len(overrides) == 1
        assert overrides[0].feed_override_pct == 70.0
        assert overrides[0].speed_override_pct == 100.0

    def test_force_warning_generates_85pct_feed(self):
        ctrl = make_controller()
        markers = [FakeMarker('FORCE_WARNING', block_index=30, severity=0.6)]
        overrides = ctrl._process_anomaly_markers(markers)
        assert len(overrides) == 1
        assert overrides[0].feed_override_pct == 85.0

    def test_chatter_risk_generates_speed_override(self):
        ctrl = make_controller()
        markers = [FakeMarker('CHATTER_RISK', block_index=40, severity=0.7)]
        overrides = ctrl._process_anomaly_markers(markers)
        assert len(overrides) == 1
        assert overrides[0].speed_override_pct == 90.0
        assert overrides[0].feed_override_pct == 100.0  # feed unchanged

    def test_thermal_critical_generates_feed_and_speed_override(self):
        ctrl = make_controller()
        markers = [FakeMarker('THERMAL_CRITICAL', block_index=20, severity=0.8)]
        overrides = ctrl._process_anomaly_markers(markers)
        assert len(overrides) == 1
        assert overrides[0].feed_override_pct == 75.0
        assert overrides[0].speed_override_pct == 90.0

    def test_thermal_warning_generates_90pct_feed(self):
        ctrl = make_controller()
        markers = [FakeMarker('THERMAL_WARNING', block_index=15, severity=0.5)]
        overrides = ctrl._process_anomaly_markers(markers)
        assert len(overrides) == 1
        assert overrides[0].feed_override_pct == 90.0

    def test_tool_end_of_life_generates_60pct_feed(self):
        ctrl = make_controller()
        markers = [FakeMarker('TOOL_END_OF_LIFE', block_index=100, severity=0.9)]
        overrides = ctrl._process_anomaly_markers(markers)
        assert len(overrides) == 1
        assert overrides[0].feed_override_pct == 60.0
        assert 'tool change' in overrides[0].reason.lower()

    def test_wear_accelerated_generates_80pct_feed(self):
        ctrl = make_controller()
        markers = [FakeMarker('WEAR_ACCELERATED', block_index=60, severity=0.6)]
        overrides = ctrl._process_anomaly_markers(markers)
        assert len(overrides) == 1
        assert overrides[0].feed_override_pct == 80.0

    def test_surface_quality_warning_generates_85pct_feed(self):
        ctrl = make_controller()
        markers = [FakeMarker('SURFACE_QUALITY_WARNING', block_index=25, severity=0.5)]
        overrides = ctrl._process_anomaly_markers(markers)
        assert len(overrides) == 1
        assert overrides[0].feed_override_pct == 85.0

    def test_no_markers_no_overrides(self):
        ctrl = make_controller()
        overrides = ctrl._process_anomaly_markers([])
        assert overrides == []

    def test_unknown_marker_type_ignored(self):
        ctrl = make_controller()
        markers = [FakeMarker('UNKNOWN_TYPE', block_index=10, severity=0.9)]
        overrides = ctrl._process_anomaly_markers(markers)
        assert overrides == []


class TestOverrideBlockRange:
    def test_force_critical_applies_3_blocks_before(self):
        ctrl = make_controller()
        markers = [FakeMarker('FORCE_CRITICAL', block_index=50, severity=0.7)]
        overrides = ctrl._process_anomaly_markers(markers)
        ov = overrides[0]
        start = getattr(ov, '_start_block', ov.trigger_block_index)
        # 3 blocks before block 50 = block 47
        assert start == 47
        assert ov.expires_at_block == 50  # post_blocks=0

    def test_force_warning_applies_2_blocks_before(self):
        ctrl = make_controller()
        markers = [FakeMarker('FORCE_WARNING', block_index=30, severity=0.6)]
        overrides = ctrl._process_anomaly_markers(markers)
        ov = overrides[0]
        start = getattr(ov, '_start_block', ov.trigger_block_index)
        assert start == 28  # 2 blocks before block 30

    def test_chatter_risk_spans_5_blocks(self):
        ctrl = make_controller()
        markers = [FakeMarker('CHATTER_RISK', block_index=40, severity=0.5)]
        overrides = ctrl._process_anomaly_markers(markers)
        ov = overrides[0]
        start = getattr(ov, '_start_block', ov.trigger_block_index)
        # pre=2, post=3 -> start=38, end=43
        assert start == 38
        assert ov.expires_at_block == 43
        total_blocks = ov.expires_at_block - start + 1
        assert total_blocks == 6  # 2+1+3

    def test_override_expires_after_affected_range(self):
        ctrl = make_controller()
        markers = [FakeMarker('THERMAL_CRITICAL', block_index=20, severity=0.5)]
        overrides = ctrl._process_anomaly_markers(markers)
        ov = overrides[0]
        # post_blocks = 1 for THERMAL_CRITICAL
        assert ov.expires_at_block == 21

    def test_high_severity_gets_earlier_preemptive_range(self):
        ctrl = make_controller()
        # severity=0.9 > 0.8 -> extra preemptive block
        markers_high = [FakeMarker('FORCE_CRITICAL', block_index=50, severity=0.9)]
        overrides_high = ctrl._process_anomaly_markers(markers_high)
        start_high = getattr(overrides_high[0], '_start_block', 0)

        markers_low = [FakeMarker('FORCE_CRITICAL', block_index=50, severity=0.5)]
        overrides_low = ctrl._process_anomaly_markers(markers_low)
        start_low = getattr(overrides_low[0], '_start_block', 0)

        # High severity should start 1 block earlier
        assert start_high < start_low
        assert start_high == start_low - 1


class TestGetOverrideForBlock:
    def test_returns_override_within_range(self):
        ctrl = make_controller()
        markers = [FakeMarker('FORCE_CRITICAL', block_index=50, severity=0.7)]
        ctrl.update_predictions(markers, current_block=40)
        # Block 48 should be within range (starts at 47, expires at 50)
        ov = ctrl.get_override_for_block(48)
        assert ov is not None
        assert ov.feed_override_pct <= 70.0

    def test_returns_none_outside_range(self):
        ctrl = make_controller()
        markers = [FakeMarker('FORCE_CRITICAL', block_index=50, severity=0.7)]
        ctrl.update_predictions(markers, current_block=40)
        # Block 40 should be outside the range (override starts at 47)
        ov = ctrl.get_override_for_block(40)
        assert ov is None

    def test_returns_none_after_expiry(self):
        ctrl = make_controller()
        markers = [FakeMarker('FORCE_CRITICAL', block_index=50, severity=0.7)]
        ctrl.update_predictions(markers, current_block=40)
        # Block 55 is well past expires_at_block=50
        ov = ctrl.get_override_for_block(55)
        assert ov is None


class TestMergeOverlappingOverrides:
    def test_overlapping_takes_minimum_feed(self):
        ctrl = make_controller()
        # Two markers with overlapping block ranges
        markers = [
            FakeMarker('FORCE_WARNING', block_index=50, severity=0.6),   # feed=85%, blocks 48-50
            FakeMarker('FORCE_CRITICAL', block_index=52, severity=0.7),  # feed=70%, blocks 49-52
        ]
        overrides = ctrl._process_anomaly_markers(markers)
        merged = ctrl._merge_overlapping_overrides(overrides)

        # In the overlapping region (blocks 49-50), feed should be min(85, 70)=70
        # Check by querying the merged overrides
        feed_at_49 = 100.0
        for m in merged:
            start = getattr(m, '_start_block', m.trigger_block_index)
            if start <= 49 <= m.expires_at_block:
                feed_at_49 = min(feed_at_49, m.feed_override_pct)
        assert feed_at_49 == 70.0

    def test_non_overlapping_stay_separate(self):
        ctrl = make_controller()
        markers = [
            FakeMarker('FORCE_CRITICAL', block_index=10, severity=0.7),  # blocks 7-10
            FakeMarker('FORCE_WARNING', block_index=50, severity=0.6),   # blocks 48-50
        ]
        overrides = ctrl._process_anomaly_markers(markers)
        merged = ctrl._merge_overlapping_overrides(overrides)
        # Should produce at least 2 separate override segments
        assert len(merged) >= 2


class TestUpdatePredictions:
    def test_clears_old_overrides(self):
        ctrl = make_controller()
        # First prediction
        markers1 = [FakeMarker('FORCE_CRITICAL', block_index=50, severity=0.7)]
        ctrl.update_predictions(markers1, current_block=40)
        assert len(ctrl._predictive_overrides) > 0

        # Second prediction replaces the first
        markers2 = [FakeMarker('THERMAL_WARNING', block_index=80, severity=0.5)]
        ctrl.update_predictions(markers2, current_block=70)
        # Old FORCE_CRITICAL overrides at block 50 should be gone
        ov = ctrl.get_override_for_block(48)
        assert ov is None

    def test_filters_markers_behind_current_block(self):
        ctrl = make_controller()
        markers = [
            FakeMarker('FORCE_CRITICAL', block_index=10, severity=0.9),  # behind
            FakeMarker('FORCE_WARNING', block_index=50, severity=0.6),   # ahead
        ]
        ctrl.update_predictions(markers, current_block=30)
        # Block 10 marker should have been filtered out
        ov = ctrl.get_override_for_block(8)
        assert ov is None
        # Block 50 marker should be present
        ov = ctrl.get_override_for_block(48)
        assert ov is not None

    def test_multiple_markers_different_blocks(self):
        ctrl = make_controller()
        markers = [
            FakeMarker('FORCE_CRITICAL', block_index=50, severity=0.7),
            FakeMarker('THERMAL_CRITICAL', block_index=80, severity=0.8),
        ]
        ctrl.update_predictions(markers, current_block=40)
        # Both should be present
        ov50 = ctrl.get_override_for_block(48)
        assert ov50 is not None
        ov80 = ctrl.get_override_for_block(79)
        assert ov80 is not None


class TestOverrideConfidence:
    def test_confidence_matches_marker_severity(self):
        ctrl = make_controller()
        markers = [FakeMarker('FORCE_CRITICAL', block_index=50, severity=0.75)]
        overrides = ctrl._process_anomaly_markers(markers)
        assert overrides[0].confidence == 0.75

    def test_low_severity_still_produces_override(self):
        ctrl = make_controller()
        markers = [FakeMarker('FORCE_CRITICAL', block_index=50, severity=0.3)]
        overrides = ctrl._process_anomaly_markers(markers)
        assert len(overrides) == 1
        assert overrides[0].confidence == 0.3
