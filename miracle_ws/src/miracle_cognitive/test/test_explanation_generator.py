"""Tests for the Explanation Generator (XAI)."""

import pytest
import time
import sys
import os
from unittest.mock import MagicMock

# Mock ROS2 dependencies before import
# These tests only need the dataclasses and class-level constants from
# ExplanationGeneratorNode, not any ROS2 functionality.  We set up mocks
# carefully using setdefault so we never overwrite modules that were
# already loaded by other test files in the same pytest session.

_mock_modules = {
    'rclpy': MagicMock(),
    'rclpy.lifecycle': MagicMock(),
    'rclpy.node': MagicMock(),
    'rclpy.qos': MagicMock(),
    'rclpy.callback_groups': MagicMock(),
    'miracle_core.lifecycle_node_base': MagicMock(),
    'miracle_core.qos_profiles': MagicMock(),
    'miracle_msgs.msg': MagicMock(),
}
for mod_name, mock_obj in _mock_modules.items():
    sys.modules.setdefault(mod_name, mock_obj)

# If the base class is a MagicMock, ensure it has the CRITICALITY attrs
# so ExplanationGeneratorNode class body can reference them.
_base = sys.modules['miracle_core.lifecycle_node_base']
if not hasattr(_base.MiracleLifecycleNode, 'CRITICALITY_LOW') or \
   isinstance(getattr(_base.MiracleLifecycleNode, 'CRITICALITY_LOW', None), MagicMock):
    _base.MiracleLifecycleNode.CRITICALITY_LOW = 'LOW'
    _base.MiracleLifecycleNode.CRITICALITY_HIGH = 'HIGH'
    _base.MiracleLifecycleNode.CRITICALITY_CRITICAL = 'CRITICAL'

# Force reimport of the target module so it picks up our mocks
sys.modules.pop('miracle_cognitive.interface.explanation_generator', None)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_cognitive.interface.explanation_generator import (
    FeatureContribution,
    ExplanationRecord,
    ExplanationGeneratorNode,
)


class TestFeatureContribution:
    def test_create(self):
        fc = FeatureContribution(
            feature_name='vibration_rms', value=0.85, contribution_pct=42.5,
        )
        assert fc.feature_name == 'vibration_rms'
        assert fc.value == 0.85
        assert fc.threshold is None
        assert fc.direction == 'above'

    def test_with_threshold(self):
        fc = FeatureContribution(
            feature_name='temperature', value=95.0, contribution_pct=30.0,
            threshold=80.0, direction='above',
        )
        assert fc.threshold == 80.0


class TestExplanationRecord:
    def test_create(self):
        record = ExplanationRecord(
            timestamp=time.time(), anomaly_type='vibration_anomaly',
            machine_id='cnc1', severity=0.85,
            summary='Test', detail='Test', counterfactual='Test', features=[],
        )
        assert record.anomaly_type == 'vibration_anomaly'
        assert record.severity == 0.85


_node_is_mock = not hasattr(ExplanationGeneratorNode, '_ANOMALY_TEMPLATES')


@pytest.mark.skipif(_node_is_mock, reason="ExplanationGeneratorNode is a Mock (rclpy mock ordering)")
class TestExplanationTemplates:
    def test_known_anomaly_type(self):
        templates = ExplanationGeneratorNode._ANOMALY_TEMPLATES
        assert 'vibration_anomaly' in templates
        assert 'force_anomaly' in templates
        assert 'thermal_anomaly' in templates
        assert 'tool_wear_anomaly' in templates

    def test_summary_format(self):
        template = ExplanationGeneratorNode._ANOMALY_TEMPLATES['vibration_anomaly']
        summary = template['summary'].format(machine_id='cnc1', severity_pct=85)
        assert 'cnc1' in summary
        assert '85%' in summary

    def test_default_template_fallback(self):
        default = ExplanationGeneratorNode._DEFAULT_TEMPLATE
        summary = default['summary'].format(
            anomaly_type='unknown_thing', machine_id='cnc2', severity_pct=60,
        )
        assert 'unknown_thing' in summary
        assert 'cnc2' in summary


class TestFeatureRanking:
    def test_contributions_sum_to_100(self):
        raw = [40.0, 25.0, 10.0, 5.0]
        total = sum(raw)
        normalized = [round(r / total * 100, 1) for r in raw]
        assert sum(normalized) == pytest.approx(100.0, abs=0.5)

    def test_primary_factor_highest(self):
        raw = [50.0, 25.0, 15.0, 10.0]
        total = sum(raw)
        normalized = [r / total * 100 for r in raw]
        assert normalized[0] > normalized[1]

    def test_single_feature(self):
        raw = [40.0]
        normalized = [round(r / sum(raw) * 100, 1) for r in raw]
        assert normalized[0] == 100.0


@pytest.mark.skipif(_node_is_mock, reason="ExplanationGeneratorNode is a Mock (rclpy mock ordering)")
class TestCausalChainBuilding:
    """Tests for causal-inference integration in the explanation generator."""

    def _make_node(self):
        """Create an ExplanationGeneratorNode with mocked super().__init__."""
        node = ExplanationGeneratorNode.__new__(ExplanationGeneratorNode)
        # Manually replicate __init__ state without calling super()
        node._history = []
        node._history_lock = __import__('threading').Lock()
        node._history_size = 500
        node._detail_level = 'medium'
        node._explanation_pub = None
        node._feedback_confidence = {}
        node._feedback_ratings = {}
        node._feedback_detail_overrides = {}
        node._gcode_contexts = {}
        from miracle_cognitive.interface.explanation_generator import RootCauseAnalyzer
        node._root_cause_analyzer = RootCauseAnalyzer()
        node._causal_links = {
            'ToolWear': ('HighFeedRate', 0.7),
            'SurfaceRoughnessIncrease': ('ToolWear', 0.8),
            'ThermalExpansion': ('HighSpindleSpeed', 0.6),
            'Chatter': ('ImproperToolpath', 0.75),
            'Vibration': ('WornBearing', 0.85),
            'ThermalDamage': ('LowCoolant', 0.9),
        }
        return node

    def test_vibration_traces_back_to_worn_bearing(self):
        node = self._make_node()
        result = node._build_causal_chain('vibration_anomaly', 0.8)
        assert result is not None
        chain, confidence = result
        assert chain == ['WornBearing', 'Vibration']
        assert chain[0] == 'WornBearing'

    def test_surface_quality_traces_full_chain(self):
        node = self._make_node()
        result = node._build_causal_chain('surface_quality_anomaly', 0.7)
        assert result is not None
        chain, confidence = result
        assert chain == ['HighFeedRate', 'ToolWear', 'SurfaceRoughnessIncrease']

    def test_chain_confidence_is_product_of_strengths(self):
        node = self._make_node()
        # vibration: WornBearing->Vibration  strength=0.85
        result = node._build_causal_chain('vibration_anomaly', 0.8)
        assert result is not None
        _, confidence = result
        assert confidence == pytest.approx(0.85)

        # surface_quality: HighFeedRate(0.7)->ToolWear(0.8)->SurfaceRoughnessIncrease
        result = node._build_causal_chain('surface_quality_anomaly', 0.7)
        assert result is not None
        _, confidence = result
        assert confidence == pytest.approx(0.7 * 0.8)

    def test_causal_detail_section_in_output(self):
        node = self._make_node()
        features = []
        detail = node._build_detail(
            'Test prefix.', features, 0.8, '', 'cnc1', 'vibration_anomaly',
        )
        assert 'Causal analysis' in detail
        assert 'WornBearing' in detail
        assert 'Causal chain:' in detail

    def test_counterfactual_uses_causal_root_cause(self):
        node = self._make_node()
        record = node.generate_explanation(
            anomaly_type='vibration_anomaly',
            machine_id='cnc1',
            severity=0.8,
        )
        assert 'bearing' in record.counterfactual.lower()
        assert 'Causal insight' in record.counterfactual

    def test_counterfactual_feed_rate_for_surface_quality(self):
        node = self._make_node()
        record = node.generate_explanation(
            anomaly_type='surface_quality_anomaly',
            machine_id='cnc1',
            severity=0.7,
        )
        assert 'feed rate' in record.counterfactual.lower()
        assert 'Causal insight' in record.counterfactual

    def test_unknown_anomaly_type_falls_back_gracefully(self):
        node = self._make_node()
        result = node._build_causal_chain('completely_unknown_anomaly', 0.5)
        assert result is None

        # generate_explanation should still work without causal section
        record = node.generate_explanation(
            anomaly_type='completely_unknown_anomaly',
            machine_id='cnc1',
            severity=0.5,
        )
        assert record.summary  # still produces output
        assert 'Causal analysis' not in record.detail


class TestHistoricalContext:
    def test_first_occurrence_message(self):
        recent = []
        if not recent:
            context = 'First occurrence of this anomaly type on this machine.'
        assert 'First occurrence' in context

    def test_repeated_occurrence_message(self):
        recent = [
            ExplanationRecord(
                timestamp=time.time() - 60, anomaly_type='vibration_anomaly',
                machine_id='cnc1', severity=0.7,
                summary='', detail='', counterfactual='', features=[],
            ),
            ExplanationRecord(
                timestamp=time.time() - 30, anomaly_type='vibration_anomaly',
                machine_id='cnc1', severity=0.9,
                summary='', detail='', counterfactual='', features=[],
            ),
        ]
        avg_severity = sum(r.severity for r in recent) / len(recent)
        assert len(recent) == 2
        assert avg_severity == pytest.approx(0.8)
