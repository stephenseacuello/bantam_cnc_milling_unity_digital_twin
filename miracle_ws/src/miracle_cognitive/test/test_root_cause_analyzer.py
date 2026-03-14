"""Tests for RootCauseAnalyzer and related dataclasses."""

import pytest
import time
import sys
import os
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# Mock ROS2 dependencies before import.
# Use setdefault to avoid overwriting modules already loaded by other tests
# in the same pytest session.
# ---------------------------------------------------------------------------

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

# Ensure the base class has required class attributes
_base = sys.modules['miracle_core.lifecycle_node_base']
if not hasattr(_base.MiracleLifecycleNode, 'CRITICALITY_LOW') or \
   isinstance(getattr(_base.MiracleLifecycleNode, 'CRITICALITY_LOW', None), MagicMock):
    _base.MiracleLifecycleNode.CRITICALITY_LOW = 'LOW'
    _base.MiracleLifecycleNode.CRITICALITY_HIGH = 'HIGH'
    _base.MiracleLifecycleNode.CRITICALITY_CRITICAL = 'CRITICAL'

# Force reimport so the module picks up our mocks
sys.modules.pop('miracle_cognitive.interface.explanation_generator', None)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import miracle_cognitive.interface.explanation_generator as _eg_module

EvidenceItem = _eg_module.EvidenceItem
RootCauseCandidate = _eg_module.RootCauseCandidate
RootCauseAnalyzer = _eg_module.RootCauseAnalyzer
ExplanationGeneratorNode = _eg_module.ExplanationGeneratorNode
ExplanationRecord = _eg_module.ExplanationRecord

# Guard: skip node-level tests if ExplanationGeneratorNode is a Mock
_node_is_mock = not hasattr(ExplanationGeneratorNode, '_ANOMALY_TEMPLATES')


# ---------------------------------------------------------------------------
# Helper to create an ExplanationGeneratorNode without calling super().__init__
# ---------------------------------------------------------------------------

def _make_node():
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


def _make_history_record(anomaly_type='vibration_anomaly', severity=0.7,
                         machine_id='cnc1', ts_offset=0):
    return ExplanationRecord(
        timestamp=time.time() - ts_offset,
        anomaly_type=anomaly_type,
        machine_id=machine_id,
        severity=severity,
        summary='test', detail='test', counterfactual='test', features=[],
    )


# ===========================================================================
# EvidenceItem dataclass tests
# ===========================================================================

class TestEvidenceItem:
    def test_create_supporting_evidence(self):
        ev = EvidenceItem(
            source='sensor', description='Force increasing', strength=0.8,
            supports=True, timestamp=time.time(),
        )
        assert ev.source == 'sensor'
        assert ev.supports is True
        assert 0.0 <= ev.strength <= 1.0

    def test_create_contradicting_evidence(self):
        ev = EvidenceItem(
            source='operator', description='Tool looks fine', strength=0.9,
            supports=False, timestamp=time.time(),
        )
        assert ev.supports is False

    def test_evidence_sources(self):
        for src in ('sensor', 'model', 'history', 'operator'):
            ev = EvidenceItem(source=src, description='d', strength=0.5,
                              supports=True, timestamp=0.0)
            assert ev.source == src


# ===========================================================================
# RootCauseCandidate dataclass tests
# ===========================================================================

class TestRootCauseCandidate:
    def test_defaults(self):
        rc = RootCauseCandidate(
            cause_id='TOOL_WEAR', description='desc', probability=0.5,
        )
        assert rc.evidence == []
        assert rc.supporting_count == 0
        assert rc.contradicting_count == 0
        assert rc.net_score == 0.0
        assert rc.mechanism == ''
        assert rc.verification_steps == []

    def test_full_construction(self):
        ev = EvidenceItem('sensor', 'desc', 0.8, True, 0.0)
        rc = RootCauseCandidate(
            cause_id='CHATTER', description='chatter', probability=0.3,
            evidence=[ev], supporting_count=1, contradicting_count=0,
            net_score=0.8, mechanism='regen chatter',
            verification_steps=['FFT', 'tap test'],
        )
        assert len(rc.evidence) == 1
        assert rc.verification_steps == ['FFT', 'tap test']


# ===========================================================================
# RootCauseAnalyzer tests
# ===========================================================================

class TestRootCauseAnalyzerBasics:
    def test_empty_anomaly_data(self):
        rca = RootCauseAnalyzer()
        candidates = rca.analyze_root_cause({}, [])
        # Should still return candidates (based on priors alone)
        assert len(candidates) == len(RootCauseAnalyzer.CAUSE_TEMPLATES)
        # Probabilities should sum to ~1
        total = sum(c.probability for c in candidates)
        assert total == pytest.approx(1.0, abs=0.01)

    def test_candidates_sorted_by_probability(self):
        rca = RootCauseAnalyzer()
        candidates = rca.analyze_root_cause({'force_trend': 'increasing'}, [])
        probs = [c.probability for c in candidates]
        assert probs == sorted(probs, reverse=True)

    def test_all_templates_have_mechanisms(self):
        rca = RootCauseAnalyzer()
        for cause_id, template in rca.CAUSE_TEMPLATES.items():
            assert template['mechanism'], f'{cause_id} has empty mechanism'
            assert len(template['mechanism']) > 20, f'{cause_id} mechanism too short'

    def test_all_templates_have_verification_steps(self):
        rca = RootCauseAnalyzer()
        for cause_id, template in rca.CAUSE_TEMPLATES.items():
            assert len(template['verification']) >= 2, \
                f'{cause_id} needs at least 2 verification steps'

    def test_all_templates_have_indicators(self):
        rca = RootCauseAnalyzer()
        for cause_id, template in rca.CAUSE_TEMPLATES.items():
            assert len(template['indicators']) >= 2, \
                f'{cause_id} needs at least 2 indicators'


class TestToolWearDetection:
    def test_matching_evidence_boosts_probability(self):
        rca = RootCauseAnalyzer()
        anomaly_data = {
            'force_trend': 'increasing',
            'vibration_trend': 'increasing',
            'tool_wear_vb': 0.85,
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        tool_wear = next(c for c in candidates if c.cause_id == 'TOOL_WEAR')
        assert tool_wear.supporting_count >= 3
        assert tool_wear.net_score > 0

    def test_tool_wear_ranked_high_with_full_match(self):
        rca = RootCauseAnalyzer()
        anomaly_data = {
            'force_trend': 'increasing',
            'vibration_trend': 'increasing',
            'tool_wear_vb': 0.9,
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        # Tool wear should be among top 3 with its full evidence match
        top_ids = [c.cause_id for c in candidates[:3]]
        assert 'TOOL_WEAR' in top_ids

    def test_contradicting_evidence_lowers_score(self):
        rca = RootCauseAnalyzer()
        # force_trend is 'decreasing' — contradicts TOOL_WEAR expectation of 'increasing'
        anomaly_data = {
            'force_trend': 'decreasing',
            'vibration_trend': 'decreasing',
            'tool_wear_vb': 0.1,  # low, doesn't meet > 0.7 threshold
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        tool_wear = next(c for c in candidates if c.cause_id == 'TOOL_WEAR')
        assert tool_wear.contradicting_count >= 2


class TestChatterDetection:
    def test_chatter_evidence_from_vibration(self):
        rca = RootCauseAnalyzer()
        anomaly_data = {
            'vibration_pattern': 'periodic',
            'frequency_type': 'non_harmonic',
            'vibration_amplitude': 0.9,
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        chatter = next(c for c in candidates if c.cause_id == 'CHATTER')
        assert chatter.supporting_count >= 2
        assert chatter.net_score > 0

    def test_chatter_ranked_above_baseline_with_evidence(self):
        rca = RootCauseAnalyzer()
        no_evidence = rca.analyze_root_cause({}, [])
        chatter_base = next(c for c in no_evidence if c.cause_id == 'CHATTER')

        with_evidence = rca.analyze_root_cause({
            'vibration_pattern': 'periodic',
            'frequency_type': 'non_harmonic',
            'vibration_amplitude': 0.95,
        }, [])
        chatter_ev = next(c for c in with_evidence if c.cause_id == 'CHATTER')
        assert chatter_ev.probability > chatter_base.probability


class TestThermalDrift:
    def test_thermal_drift_correlation(self):
        rca = RootCauseAnalyzer()
        anomaly_data = {
            'position_drift': 'gradual',
            'temperature_correlation': 'positive',
            'temperature_trend': 'increasing',
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        thermal = next(c for c in candidates if c.cause_id == 'THERMAL_DRIFT')
        assert thermal.supporting_count >= 3

    def test_thermal_drift_ranked_high_with_full_match(self):
        rca = RootCauseAnalyzer()
        anomaly_data = {
            'position_drift': 'gradual',
            'temperature_correlation': 'positive',
            'temperature_trend': 'increasing',
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        top_ids = [c.cause_id for c in candidates[:3]]
        assert 'THERMAL_DRIFT' in top_ids


class TestCompetingCauses:
    def test_multiple_causes_ranked_correctly(self):
        rca = RootCauseAnalyzer()
        # Evidence that strongly supports TOOL_WEAR but not CHATTER
        anomaly_data = {
            'force_trend': 'increasing',
            'vibration_trend': 'increasing',
            'tool_wear_vb': 0.9,
            'vibration_pattern': 'aperiodic',  # contradicts CHATTER
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        tool_wear = next(c for c in candidates if c.cause_id == 'TOOL_WEAR')
        chatter = next(c for c in candidates if c.cause_id == 'CHATTER')
        assert tool_wear.probability > chatter.probability

    def test_spindle_vs_chatter_differentiation(self):
        rca = RootCauseAnalyzer()
        # High-frequency vibration + runout favors SPINDLE_BEARING
        anomaly_data = {
            'vibration_frequency': 'high',
            'runout': 0.85,
            'vibration_pattern': 'periodic',
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        spindle = next(c for c in candidates if c.cause_id == 'SPINDLE_BEARING')
        # Spindle should have good evidence
        assert spindle.supporting_count >= 2


class TestEvidenceScoring:
    def test_supporting_increases_net_score(self):
        rca = RootCauseAnalyzer()
        anomaly_data = {'force_trend': 'increasing'}
        candidates = rca.analyze_root_cause(anomaly_data, [])
        tool_wear = next(c for c in candidates if c.cause_id == 'TOOL_WEAR')
        assert tool_wear.net_score > 0

    def test_contradicting_decreases_net_score(self):
        rca = RootCauseAnalyzer()
        # Provide values that contradict TOOL_WEAR
        anomaly_data = {
            'force_trend': 'decreasing',
            'vibration_trend': 'stable',
            'tool_wear_vb': 0.05,
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        tool_wear = next(c for c in candidates if c.cause_id == 'TOOL_WEAR')
        assert tool_wear.net_score < 0

    def test_evidence_items_have_correct_sources(self):
        rca = RootCauseAnalyzer()
        anomaly_data = {'force_trend': 'increasing'}
        candidates = rca.analyze_root_cause(anomaly_data, [])
        tool_wear = next(c for c in candidates if c.cause_id == 'TOOL_WEAR')
        sensor_evidence = [e for e in tool_wear.evidence if e.source == 'sensor']
        assert len(sensor_evidence) >= 1

    def test_net_score_is_supporting_minus_contradicting(self):
        rca = RootCauseAnalyzer()
        # Mix of supporting and contradicting
        anomaly_data = {
            'force_trend': 'increasing',     # supports TOOL_WEAR
            'vibration_trend': 'decreasing',  # contradicts TOOL_WEAR
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        tool_wear = next(c for c in candidates if c.cause_id == 'TOOL_WEAR')
        sup = sum(e.strength for e in tool_wear.evidence if e.supports)
        con = sum(e.strength for e in tool_wear.evidence if not e.supports)
        assert tool_wear.net_score == pytest.approx(sup - con, abs=0.01)


class TestOperatorEvidence:
    def test_operator_evidence_updates_probability(self):
        rca = RootCauseAnalyzer()
        # Baseline without operator evidence
        base = rca.analyze_root_cause({}, [])
        base_tw = next(c for c in base if c.cause_id == 'TOOL_WEAR')

        # Add strong operator evidence supporting TOOL_WEAR
        rca.add_operator_evidence('TOOL_WEAR', EvidenceItem(
            source='operator', description='Tool visibly worn',
            strength=0.95, supports=True, timestamp=time.time(),
        ))
        updated = rca.analyze_root_cause({}, [])
        updated_tw = next(c for c in updated if c.cause_id == 'TOOL_WEAR')
        assert updated_tw.probability > base_tw.probability

    def test_operator_contradicting_evidence_lowers_probability(self):
        rca = RootCauseAnalyzer()
        base = rca.analyze_root_cause({}, [])
        base_tw = next(c for c in base if c.cause_id == 'TOOL_WEAR')

        rca.add_operator_evidence('TOOL_WEAR', EvidenceItem(
            source='operator', description='Tool is brand new',
            strength=0.95, supports=False, timestamp=time.time(),
        ))
        updated = rca.analyze_root_cause({}, [])
        updated_tw = next(c for c in updated if c.cause_id == 'TOOL_WEAR')
        assert updated_tw.probability < base_tw.probability

    def test_multiple_operator_evidence_accumulates(self):
        rca = RootCauseAnalyzer()
        rca.add_operator_evidence('CHATTER', EvidenceItem(
            source='operator', description='Chatter marks on workpiece',
            strength=0.8, supports=True, timestamp=time.time(),
        ))
        rca.add_operator_evidence('CHATTER', EvidenceItem(
            source='operator', description='Chatter audible',
            strength=0.7, supports=True, timestamp=time.time(),
        ))
        candidates = rca.analyze_root_cause({}, [])
        chatter = next(c for c in candidates if c.cause_id == 'CHATTER')
        op_evidence = [e for e in chatter.evidence if e.source == 'operator']
        assert len(op_evidence) == 2


class TestVerificationPlan:
    def test_known_cause_returns_steps(self):
        rca = RootCauseAnalyzer()
        steps = rca.get_verification_plan('TOOL_WEAR')
        assert len(steps) >= 2
        assert any('wear' in s.lower() or 'flank' in s.lower() for s in steps)

    def test_unknown_cause_returns_empty(self):
        rca = RootCauseAnalyzer()
        steps = rca.get_verification_plan('NONEXISTENT_CAUSE')
        assert steps == []

    def test_each_cause_has_verification_plan(self):
        rca = RootCauseAnalyzer()
        for cause_id in rca.CAUSE_TEMPLATES:
            steps = rca.get_verification_plan(cause_id)
            assert len(steps) >= 2, f'{cause_id} needs verification steps'


class TestHistoryBoost:
    def test_history_boosts_recurring_cause(self):
        rca = RootCauseAnalyzer()
        history = [
            _make_history_record('tool_wear_anomaly', 0.7, ts_offset=60),
            _make_history_record('tool_wear_anomaly', 0.8, ts_offset=30),
            _make_history_record('tool_wear_anomaly', 0.75, ts_offset=10),
        ]
        no_hist = rca.analyze_root_cause({}, [])
        with_hist = rca.analyze_root_cause({}, history)

        tw_no = next(c for c in no_hist if c.cause_id == 'TOOL_WEAR')
        tw_yes = next(c for c in with_hist if c.cause_id == 'TOOL_WEAR')
        assert tw_yes.probability > tw_no.probability

    def test_history_evidence_source_is_history(self):
        rca = RootCauseAnalyzer()
        history = [_make_history_record('vibration_anomaly', 0.8)]
        candidates = rca.analyze_root_cause({}, history)
        chatter = next(c for c in candidates if c.cause_id == 'CHATTER')
        hist_ev = [e for e in chatter.evidence if e.source == 'history']
        assert len(hist_ev) >= 1


class TestProgrammingError:
    def test_block_consistency_detection(self):
        rca = RootCauseAnalyzer()
        anomaly_data = {
            'block_consistency': 'true',
            'gcode_block': 'repeating',
            'anomaly_recurrence': 'same_location',
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        prog_err = next(c for c in candidates if c.cause_id == 'PROGRAMMING_ERROR')
        assert prog_err.supporting_count >= 3

    def test_programming_error_ranked_high_with_evidence(self):
        rca = RootCauseAnalyzer()
        anomaly_data = {
            'block_consistency': 'true',
            'gcode_block': 'repeating',
            'anomaly_recurrence': 'same_location',
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        top_ids = [c.cause_id for c in candidates[:3]]
        assert 'PROGRAMMING_ERROR' in top_ids


class TestUnknownPattern:
    def test_unknown_anomaly_low_confidence(self):
        rca = RootCauseAnalyzer()
        # Random fields that don't match any template well
        anomaly_data = {
            'unknown_field': 'random_value',
            'another_field': 42,
        }
        candidates = rca.analyze_root_cause(anomaly_data, [])
        # All candidates should have relatively low probability
        top = candidates[0]
        assert top.probability < 0.3  # No strong evidence for any cause

    def test_probabilities_still_sum_to_one(self):
        rca = RootCauseAnalyzer()
        candidates = rca.analyze_root_cause({'xyz': 'abc'}, [])
        total = sum(c.probability for c in candidates)
        assert total == pytest.approx(1.0, abs=0.01)


# ===========================================================================
# Integration with ExplanationGeneratorNode
# ===========================================================================

@pytest.mark.skipif(_node_is_mock,
                    reason="ExplanationGeneratorNode is a Mock (rclpy mock ordering)")
class TestNodeIntegration:
    """Integration tests using manually constructed node (bypasses MagicMock base)."""

    def test_node_has_root_cause_analyzer(self):
        node = _make_node()
        assert hasattr(node, '_root_cause_analyzer')
        assert isinstance(node._root_cause_analyzer, RootCauseAnalyzer)

    def test_generate_detail_explanation_returns_string(self):
        node = _make_node()
        detail = node._generate_detail_explanation(
            'vibration_anomaly', 'cnc1', 0.8,
            {'vibration_pattern': 'periodic', 'frequency_type': 'non_harmonic'},
        )
        assert isinstance(detail, str)
        assert 'Root cause analysis' in detail

    def test_detail_includes_verification_plan(self):
        node = _make_node()
        detail = node._generate_detail_explanation(
            'force_anomaly', 'cnc1', 0.7,
            {'force_trend': 'increasing', 'tool_wear_vb': 0.9},
        )
        assert 'Verification plan' in detail

    def test_build_detail_includes_root_cause_section(self):
        node = _make_node()
        features = []
        detail = node._build_detail(
            'Test prefix.', features, 0.8, '', 'cnc1', 'vibration_anomaly',
            anomaly_data={'vibration_pattern': 'periodic'},
        )
        assert 'Root cause analysis' in detail

    def test_generate_detail_with_empty_data(self):
        node = _make_node()
        detail = node._generate_detail_explanation(
            'vibration_anomaly', 'cnc1', 0.5, {},
        )
        # Should still produce output (from priors)
        assert 'Root cause analysis' in detail

    def test_generate_detail_with_none_data(self):
        node = _make_node()
        detail = node._generate_detail_explanation(
            'vibration_anomaly', 'cnc1', 0.5, None,
        )
        assert 'Root cause analysis' in detail


class TestNodeIntegrationStandalone:
    """Integration tests that work even when ExplanationGeneratorNode is a Mock.

    These tests exercise _generate_detail_explanation logic through a
    standalone function call pattern using the RootCauseAnalyzer directly.
    """

    def _generate_detail(self, anomaly_type, machine_id, severity,
                         anomaly_data=None, history=None):
        """Replicate _generate_detail_explanation logic standalone."""
        if anomaly_data is None:
            anomaly_data = {}
        if history is None:
            history = []

        rca = RootCauseAnalyzer()
        candidates = rca.analyze_root_cause(anomaly_data, history)
        if not candidates:
            return ''

        lines = ['\nRoot cause analysis (ranked by probability):']
        for i, candidate in enumerate(candidates[:5], 1):
            lines.append(
                f'  {i}. [{candidate.cause_id}] {candidate.description} '
                f'(probability: {candidate.probability:.1%})'
            )
            lines.append(
                f'     Evidence: {candidate.supporting_count} supporting, '
                f'{candidate.contradicting_count} contradicting '
                f'(net score: {candidate.net_score:+.2f})'
            )
            lines.append(f'     Mechanism: {candidate.mechanism}')

        top = candidates[0]
        lines.append(f'\nVerification plan for top cause ({top.cause_id}):')
        for step in top.verification_steps:
            lines.append(f'  - {step}')

        return '\n'.join(lines)

    def test_detail_explanation_returns_string(self):
        detail = self._generate_detail(
            'vibration_anomaly', 'cnc1', 0.8,
            {'vibration_pattern': 'periodic', 'frequency_type': 'non_harmonic'},
        )
        assert isinstance(detail, str)
        assert 'Root cause analysis' in detail

    def test_detail_includes_verification_plan(self):
        detail = self._generate_detail(
            'force_anomaly', 'cnc1', 0.7,
            {'force_trend': 'increasing', 'tool_wear_vb': 0.9},
        )
        assert 'Verification plan' in detail

    def test_detail_includes_mechanism(self):
        detail = self._generate_detail(
            'thermal_anomaly', 'cnc1', 0.6,
            {'temperature_trend': 'increasing'},
        )
        assert 'Mechanism:' in detail

    def test_detail_with_empty_data(self):
        detail = self._generate_detail('vibration_anomaly', 'cnc1', 0.5, {})
        assert 'Root cause analysis' in detail

    def test_detail_with_none_data(self):
        detail = self._generate_detail('vibration_anomaly', 'cnc1', 0.5, None)
        assert 'Root cause analysis' in detail

    def test_detail_shows_probability(self):
        detail = self._generate_detail(
            'vibration_anomaly', 'cnc1', 0.8,
            {'force_trend': 'increasing'},
        )
        assert 'probability:' in detail
