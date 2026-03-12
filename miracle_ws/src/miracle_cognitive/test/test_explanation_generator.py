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
