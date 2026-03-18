"""Tests for the NaturalLanguageExplainer (NLG) module."""

import pytest
import sys
import os
import time
from unittest.mock import MagicMock

# Mock ROS2 dependencies before import — same pattern as test_explanation_generator.py
for mod in ['miracle_core.datatypes', 'miracle_core.constants',
            'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
            'rclpy.callback_groups',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

_base = sys.modules['miracle_core.lifecycle_node_base']
if not hasattr(_base.MiracleLifecycleNode, 'CRITICALITY_LOW') or \
   isinstance(getattr(_base.MiracleLifecycleNode, 'CRITICALITY_LOW', None), MagicMock):
    _base.MiracleLifecycleNode.CRITICALITY_LOW = 'LOW'
    _base.MiracleLifecycleNode.CRITICALITY_HIGH = 'HIGH'
    _base.MiracleLifecycleNode.CRITICALITY_CRITICAL = 'CRITICAL'

sys.modules.pop('miracle_cognitive.interface.explanation_generator', None)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_cognitive.interface.explanation_generator import (
    ExplanationContext,
    NLGExplanation,
    NaturalLanguageExplainer,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def explainer():
    """Return a default NaturalLanguageExplainer (operator audience)."""
    return NaturalLanguageExplainer(audience='operator')


@pytest.fixture
def context():
    """Return a sample ExplanationContext."""
    return ExplanationContext(
        event_type='alarm',
        severity=0.75,
        parameters={'spindle_speed': 8000, 'feed_rate': 500},
        timestamp=time.time(),
        machine_id='cnc1',
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestSetAudience:
    """Tests for audience switching."""

    def test_default_audience_is_operator(self, explainer):
        assert explainer.audience == 'operator'

    def test_set_audience_engineer(self, explainer):
        explainer.set_audience('engineer')
        assert explainer.audience == 'engineer'

    def test_set_audience_manager(self, explainer):
        explainer.set_audience('manager')
        assert explainer.audience == 'manager'

    def test_invalid_audience_raises(self, explainer):
        with pytest.raises(ValueError, match="Invalid audience"):
            explainer.set_audience('customer')


class TestExplainAlarm:
    """Tests for alarm explanations across template types."""

    @pytest.mark.parametrize('alarm_type', [
        'CHATTER', 'TOOL_WEAR', 'THERMAL_DRIFT',
        'FORCE_OVERLOAD', 'SURFACE_QUALITY', 'COOLANT_LOW',
    ])
    def test_known_alarm_returns_valid_explanation(self, explainer, context, alarm_type):
        result = explainer.explain_alarm(alarm_type, 0.8, context)
        assert isinstance(result, NLGExplanation)
        assert result.audience == 'operator'
        assert len(result.title) > 0
        assert len(result.summary) > 0
        assert len(result.details) > 0
        assert len(result.recommendations) > 0
        assert 0.0 <= result.confidence <= 1.0

    def test_unknown_alarm_uses_default_template(self, explainer, context):
        result = explainer.explain_alarm('UNKNOWN_ALARM', 0.5, context)
        assert 'UNKNOWN_ALARM' in result.summary
        assert result.confidence > 0

    def test_high_severity_uses_critical_urgency(self, explainer, context):
        result = explainer.explain_alarm('CHATTER', 0.95, context)
        assert 'CRITICAL' in result.title

    def test_low_severity_uses_low_urgency(self, explainer, context):
        result = explainer.explain_alarm('TOOL_WEAR', 0.35, context)
        assert 'LOW' in result.title

    def test_alarm_without_context(self, explainer):
        result = explainer.explain_alarm('CHATTER', 0.6)
        assert 'unknown' in result.summary
        assert result.audience == 'operator'

    def test_engineer_gets_more_detail(self, context):
        op = NaturalLanguageExplainer(audience='operator')
        eng = NaturalLanguageExplainer(audience='engineer')
        op_result = op.explain_alarm('TOOL_WEAR', 0.7, context)
        eng_result = eng.explain_alarm('TOOL_WEAR', 0.7, context)
        assert len(eng_result.details) >= len(op_result.details)

    def test_manager_gets_fewer_recommendations(self, context):
        mgr = NaturalLanguageExplainer(audience='manager')
        result = mgr.explain_alarm('FORCE_OVERLOAD', 0.8, context)
        # Manager should receive at most 1 recommendation
        assert len(result.recommendations) <= 1


class TestExplainProcessChange:
    """Tests for process change explanations."""

    def test_increase_wording(self, explainer):
        result = explainer.explain_process_change(
            parameter='feed_rate', old_value=500.0, new_value=600.0,
            reason='adaptive optimisation',
        )
        assert 'increased' in result.summary
        assert 'feed_rate' in result.title
        assert result.confidence == 0.85

    def test_decrease_wording(self, explainer):
        result = explainer.explain_process_change(
            parameter='spindle_speed', old_value=10000.0, new_value=8000.0,
            reason='chatter mitigation',
        )
        assert 'decreased' in result.summary

    def test_percentage_in_summary(self, explainer):
        result = explainer.explain_process_change(
            parameter='depth_of_cut', old_value=2.0, new_value=1.0,
            reason='force limit',
        )
        assert '50.0%' in result.summary


class TestExplainPrediction:
    """Tests for prediction explanations."""

    def test_high_confidence_wording(self, explainer):
        result = explainer.explain_prediction(
            prediction_type='tool_life',
            predicted_value=45.0,
            confidence=0.92,
            factors=['cutting_speed', 'feed_rate'],
        )
        assert 'high' in result.summary
        assert result.confidence == 0.92

    def test_low_confidence_adds_extra_recommendation(self, explainer):
        result = explainer.explain_prediction(
            prediction_type='surface_roughness',
            predicted_value=1.6,
            confidence=0.35,
            factors=['tool_wear'],
        )
        assert any('tentative' in r.lower() for r in result.recommendations)

    def test_engineer_includes_factors(self):
        eng = NaturalLanguageExplainer(audience='engineer')
        result = eng.explain_prediction(
            prediction_type='vibration_amplitude',
            predicted_value=0.85,
            confidence=0.7,
            factors=['spindle_speed', 'depth_of_cut'],
        )
        detail_text = ' '.join(result.details)
        assert 'spindle_speed' in detail_text
        assert 'depth_of_cut' in detail_text


class TestExplainRecommendation:
    """Tests for recommendation explanations."""

    def test_basic_recommendation(self, explainer, context):
        result = explainer.explain_recommendation(
            action='replace tool insert',
            expected_benefit='restore surface finish quality',
            risk='continued quality degradation and potential scrap',
            context=context,
        )
        assert 'replace tool insert' in result.summary
        assert 'cnc1' in result.summary
        assert 'replace tool insert' in result.recommendations

    def test_recommendation_without_context(self, explainer):
        result = explainer.explain_recommendation(
            action='inspect coolant pump',
            expected_benefit='restore cooling flow',
            risk='thermal damage to workpiece',
        )
        assert 'unknown' in result.summary

    def test_confidence_scales_with_severity(self, explainer):
        ctx_low = ExplanationContext(
            event_type='alarm', severity=0.2,
            parameters={}, timestamp=time.time(), machine_id='cnc1',
        )
        ctx_high = ExplanationContext(
            event_type='alarm', severity=0.9,
            parameters={}, timestamp=time.time(), machine_id='cnc1',
        )
        low = explainer.explain_recommendation(
            action='a', expected_benefit='b', risk='c', context=ctx_low,
        )
        high = explainer.explain_recommendation(
            action='a', expected_benefit='b', risk='c', context=ctx_high,
        )
        assert high.confidence > low.confidence


class TestExplanationContextDataclass:
    """Tests for the ExplanationContext dataclass."""

    def test_fields(self):
        ctx = ExplanationContext(
            event_type='alarm', severity=0.5,
            parameters={'key': 'val'}, timestamp=1000.0, machine_id='cnc2',
        )
        assert ctx.event_type == 'alarm'
        assert ctx.severity == 0.5
        assert ctx.parameters == {'key': 'val'}
        assert ctx.timestamp == 1000.0
        assert ctx.machine_id == 'cnc2'


class TestNLGExplanationDataclass:
    """Tests for the NLGExplanation dataclass."""

    def test_fields(self):
        exp = NLGExplanation(
            title='Test', summary='Sum', details=['d1'],
            recommendations=['r1'], confidence=0.9, audience='engineer',
        )
        assert exp.title == 'Test'
        assert exp.summary == 'Sum'
        assert exp.details == ['d1']
        assert exp.recommendations == ['r1']
        assert exp.confidence == 0.9
        assert exp.audience == 'engineer'
