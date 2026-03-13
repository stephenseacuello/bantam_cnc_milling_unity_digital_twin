"""Tests for the Anomaly Pattern Library."""

import json
import os
import sys
import tempfile
import time

import pytest
from unittest.mock import MagicMock

# Stub out ROS2 and project dependencies
for mod in (
    'rclpy', 'rclpy.lifecycle', 'rclpy.node', 'rclpy.qos',
    'rclpy.parameter', 'rclpy.callback_groups', 'rclpy.executors',
    'std_msgs', 'std_msgs.msg',
):
    sys.modules.setdefault(mod, MagicMock())

# Mock miracle_core and miracle_msgs submodules (NOT top-level).
# Never replace top-level miracle_core — it's eagerly imported as a real package.
sys.modules.setdefault('miracle_msgs', MagicMock())

import miracle_core  # noqa: E402
import miracle_msgs  # noqa: E402

for attr in ('lifecycle_node_base', 'qos_profiles'):
    sub = MagicMock()
    sys.modules.setdefault(f'miracle_core.{attr}', sub)
    setattr(miracle_core, attr, sys.modules[f'miracle_core.{attr}'])

for attr in ('msg',):
    sub = MagicMock()
    sys.modules.setdefault(f'miracle_msgs.{attr}', sub)
    setattr(miracle_msgs, attr, sys.modules[f'miracle_msgs.{attr}'])

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_scada.alert_correlator import (  # noqa: E402
    AlertEntry,
    AnomalyPattern,
    AnomalyPatternLibrary,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_correlated_alert(**overrides):
    """Return a dict that mimics a CorrelatedAlert message."""
    base = {
        'category': 'tool_degradation_chain',
        'contributing_alert_ids': [
            'vibration_anomaly@1000',
            'force_anomaly@1001',
        ],
        'root_cause_hypothesis': 'Progressive tool wear',
        'recommended_actions': ['Inspect tool wear', 'Reduce feed rate'],
        'machine_id': 'cnc1',
        'confidence': 0.85,
    }
    base.update(overrides)
    return base


def _make_alert_entry(alert_type, machine_id='cnc1', severity=0.8, ts=None):
    return AlertEntry(
        alert_type=alert_type,
        machine_id=machine_id,
        severity=severity,
        message='test',
        timestamp=ts if ts is not None else time.time(),
    )


# ===================================================================
# AnomalyPattern dataclass
# ===================================================================

class TestAnomalyPattern:
    def test_default_fields(self):
        p = AnomalyPattern()
        assert isinstance(p.pattern_id, str) and len(p.pattern_id) > 0
        assert p.occurrence_count == 0
        assert p.confidence == 0.0

    def test_to_dict_roundtrip(self):
        p = AnomalyPattern(name='test', occurrence_count=3, confidence=0.7)
        d = p.to_dict()
        p2 = AnomalyPattern.from_dict(d)
        assert p2.name == 'test'
        assert p2.occurrence_count == 3
        assert p2.confidence == 0.7

    def test_unique_ids(self):
        ids = {AnomalyPattern().pattern_id for _ in range(20)}
        assert len(ids) == 20


# ===================================================================
# New pattern creation
# ===================================================================

class TestRecordPattern:
    def test_creates_new_pattern(self):
        lib = AnomalyPatternLibrary()
        alert = _make_correlated_alert()
        p = lib.record_pattern(alert)
        assert p.occurrence_count == 1
        assert p.name == 'tool_degradation_chain'
        assert 'vibration_anomaly' in p.signature['alert_types']
        assert 'force_anomaly' in p.signature['alert_types']
        assert len(lib.patterns) == 1

    def test_new_pattern_has_machine_id(self):
        lib = AnomalyPatternLibrary()
        p = lib.record_pattern(_make_correlated_alert(machine_id='cnc5'))
        assert 'cnc5' in p.machine_ids

    def test_new_pattern_confidence_starts_at_half(self):
        lib = AnomalyPatternLibrary()
        p = lib.record_pattern(_make_correlated_alert())
        assert p.confidence == 0.5

    def test_new_pattern_sets_timestamps(self):
        before = time.time()
        lib = AnomalyPatternLibrary()
        p = lib.record_pattern(_make_correlated_alert())
        after = time.time()
        assert before <= p.first_seen <= after
        assert before <= p.last_seen <= after


# ===================================================================
# Occurrence count increment + confidence growth
# ===================================================================

class TestOccurrenceAndConfidence:
    def test_occurrence_count_increments(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert())
        lib.record_pattern(_make_correlated_alert())
        lib.record_pattern(_make_correlated_alert())
        assert len(lib.patterns) == 1
        assert lib.patterns[0].occurrence_count == 3

    def test_confidence_grows_with_occurrences(self):
        lib = AnomalyPatternLibrary()
        for _ in range(10):
            lib.record_pattern(_make_correlated_alert())
        p = lib.patterns[0]
        assert p.confidence > 0.5
        # 0.5 + 0.05*10 = 1.0
        assert p.confidence == 1.0

    def test_confidence_capped_at_one(self):
        lib = AnomalyPatternLibrary()
        for _ in range(50):
            lib.record_pattern(_make_correlated_alert())
        assert lib.patterns[0].confidence <= 1.0

    def test_last_seen_updates(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert())
        first_last = lib.patterns[0].last_seen
        time.sleep(0.01)
        lib.record_pattern(_make_correlated_alert())
        assert lib.patterns[0].last_seen >= first_last


# ===================================================================
# Pattern matching (high / low similarity)
# ===================================================================

class TestMatchPattern:
    def test_high_similarity_match(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert())

        alerts = [
            _make_alert_entry('vibration_anomaly', ts=100.0),
            _make_alert_entry('force_anomaly', ts=101.0),
        ]
        matches = lib.match_pattern(alerts)
        assert len(matches) >= 1
        assert matches[0][1] > 0.7  # high similarity

    def test_low_similarity_no_strong_match(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert())

        alerts = [
            _make_alert_entry('unknown_type_a', ts=100.0),
            _make_alert_entry('unknown_type_b', ts=200.0),
        ]
        matches = lib.match_pattern(alerts)
        # All matches should be below threshold
        for _, score in matches:
            assert score < 0.7

    def test_match_returns_sorted_by_score(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert())
        lib.record_pattern(_make_correlated_alert(
            category='thermal_cascade',
            contributing_alert_ids=['thermal_anomaly@200', 'dimensional_drift@201'],
        ))

        alerts = [
            _make_alert_entry('vibration_anomaly', ts=100.0),
            _make_alert_entry('force_anomaly', ts=101.0),
        ]
        matches = lib.match_pattern(alerts)
        scores = [s for _, s in matches]
        assert scores == sorted(scores, reverse=True)

    def test_match_empty_library(self):
        lib = AnomalyPatternLibrary()
        alerts = [_make_alert_entry('vibration_anomaly')]
        assert lib.match_pattern(alerts) == []

    def test_match_empty_sequence(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert())
        assert lib.match_pattern([]) == []


# ===================================================================
# Machine-specific queries
# ===================================================================

class TestMachineQueries:
    def test_get_pattern_for_machine(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert(machine_id='cnc1'))
        lib.record_pattern(_make_correlated_alert(
            category='thermal_cascade',
            contributing_alert_ids=['thermal_anomaly@300'],
            machine_id='cnc2',
        ))
        results = lib.get_pattern_for_machine('cnc1')
        assert len(results) == 1
        assert results[0].name == 'tool_degradation_chain'

    def test_machine_id_accumulated(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert(machine_id='cnc1'))
        lib.record_pattern(_make_correlated_alert(machine_id='cnc2'))
        p = lib.patterns[0]
        assert 'cnc1' in p.machine_ids
        assert 'cnc2' in p.machine_ids

    def test_no_patterns_for_unknown_machine(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert(machine_id='cnc1'))
        assert lib.get_pattern_for_machine('cnc99') == []


# ===================================================================
# Top patterns
# ===================================================================

class TestTopPatterns:
    def test_top_patterns_ordering(self):
        lib = AnomalyPatternLibrary()
        # Record pattern A once
        lib.record_pattern(_make_correlated_alert(
            category='rare_pattern',
            contributing_alert_ids=['type_x@1'],
        ))
        # Record pattern B many times
        for _ in range(5):
            lib.record_pattern(_make_correlated_alert())

        top = lib.get_top_patterns(n=2)
        assert len(top) == 2
        assert top[0].occurrence_count >= top[1].occurrence_count

    def test_top_patterns_limit(self):
        lib = AnomalyPatternLibrary()
        for i in range(5):
            lib.record_pattern(_make_correlated_alert(
                category=f'pattern_{i}',
                contributing_alert_ids=[f'type_{i}@{i}'],
            ))
        assert len(lib.get_top_patterns(n=3)) == 3

    def test_top_patterns_empty_library(self):
        lib = AnomalyPatternLibrary()
        assert lib.get_top_patterns() == []


# ===================================================================
# JSONL persistence round-trip
# ===================================================================

class TestPersistence:
    def test_save_and_load(self, tmp_path):
        path = str(tmp_path / 'patterns.jsonl')
        lib = AnomalyPatternLibrary(persistence_path=path)
        lib.record_pattern(_make_correlated_alert())
        lib.record_pattern(_make_correlated_alert(
            category='thermal_cascade',
            contributing_alert_ids=['thermal_anomaly@300'],
            machine_id='cnc2',
        ))

        lib2 = AnomalyPatternLibrary(persistence_path=path)
        assert len(lib2.patterns) == 2
        assert lib2.patterns[0].name == lib.patterns[0].name

    def test_load_nonexistent_file(self, tmp_path):
        path = str(tmp_path / 'nonexistent.jsonl')
        lib = AnomalyPatternLibrary(persistence_path=path)
        assert len(lib.patterns) == 0

    def test_persistence_updates_on_record(self, tmp_path):
        path = str(tmp_path / 'patterns.jsonl')
        lib = AnomalyPatternLibrary(persistence_path=path)
        lib.record_pattern(_make_correlated_alert())

        # File should exist with one line
        with open(path) as f:
            lines = [l for l in f if l.strip()]
        assert len(lines) == 1
        data = json.loads(lines[0])
        assert data['name'] == 'tool_degradation_chain'

    def test_persistence_roundtrip_fields(self, tmp_path):
        path = str(tmp_path / 'patterns.jsonl')
        lib = AnomalyPatternLibrary(persistence_path=path)
        for _ in range(3):
            lib.record_pattern(_make_correlated_alert(machine_id='cnc1'))

        lib2 = AnomalyPatternLibrary(persistence_path=path)
        p = lib2.patterns[0]
        assert p.occurrence_count == 3
        assert 'cnc1' in p.machine_ids
        assert p.confidence > 0.5


# ===================================================================
# Pattern merging
# ===================================================================

class TestMergePatterns:
    def test_merge_combines_counts(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert())
        lib.record_pattern(_make_correlated_alert(
            category='thermal_cascade',
            contributing_alert_ids=['thermal_anomaly@300'],
        ))
        id1 = lib.patterns[0].pattern_id
        id2 = lib.patterns[1].pattern_id
        merged = lib.merge_patterns(id1, id2)
        assert merged is not None
        assert merged.occurrence_count == 2
        assert len(lib.patterns) == 1

    def test_merge_unions_machine_ids(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert(machine_id='cnc1'))
        lib.record_pattern(_make_correlated_alert(
            category='thermal_cascade',
            contributing_alert_ids=['thermal_anomaly@300'],
            machine_id='cnc2',
        ))
        id1 = lib.patterns[0].pattern_id
        id2 = lib.patterns[1].pattern_id
        merged = lib.merge_patterns(id1, id2)
        assert 'cnc1' in merged.machine_ids
        assert 'cnc2' in merged.machine_ids

    def test_merge_unions_alert_types(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert())
        lib.record_pattern(_make_correlated_alert(
            category='thermal_cascade',
            contributing_alert_ids=['thermal_anomaly@300'],
        ))
        id1 = lib.patterns[0].pattern_id
        id2 = lib.patterns[1].pattern_id
        merged = lib.merge_patterns(id1, id2)
        types = merged.signature['alert_types']
        assert 'vibration_anomaly' in types
        assert 'thermal_anomaly' in types

    def test_merge_invalid_id_returns_none(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert())
        result = lib.merge_patterns(lib.patterns[0].pattern_id, 'bogus-id')
        assert result is None

    def test_merge_keeps_earliest_first_seen(self):
        lib = AnomalyPatternLibrary()
        lib.record_pattern(_make_correlated_alert())
        time.sleep(0.01)
        lib.record_pattern(_make_correlated_alert(
            category='other',
            contributing_alert_ids=['other_type@400'],
        ))
        id1 = lib.patterns[0].pattern_id
        id2 = lib.patterns[1].pattern_id
        earliest = min(lib.patterns[0].first_seen, lib.patterns[1].first_seen)
        merged = lib.merge_patterns(id1, id2)
        assert merged.first_seen == earliest


# ===================================================================
# Temporal order matching
# ===================================================================

class TestTemporalOrder:
    def test_same_order_high_score(self):
        sig1 = {
            'alert_types': ['a', 'b', 'c'],
            'temporal_order': ['a', 'b', 'c'],
            'time_window_sec': 30.0,
            'min_count': 3,
        }
        sig2 = {
            'alert_types': ['a', 'b', 'c'],
            'temporal_order': ['a', 'b', 'c'],
            'time_window_sec': 30.0,
            'min_count': 3,
        }
        score = AnomalyPatternLibrary._compute_signature_similarity(sig1, sig2)
        assert score == 1.0

    def test_reversed_order_lower_score(self):
        sig1 = {
            'alert_types': ['a', 'b', 'c'],
            'temporal_order': ['a', 'b', 'c'],
            'time_window_sec': 30.0,
            'min_count': 3,
        }
        sig2 = {
            'alert_types': ['a', 'b', 'c'],
            'temporal_order': ['c', 'b', 'a'],
            'time_window_sec': 30.0,
            'min_count': 3,
        }
        score = AnomalyPatternLibrary._compute_signature_similarity(sig1, sig2)
        # Types identical (0.5*1.0) but order differs (only 'b' matches -> 0.3*1/3)
        assert score < 1.0

    def test_empty_signatures(self):
        sig1 = {'alert_types': [], 'temporal_order': [], 'time_window_sec': 0, 'min_count': 0}
        sig2 = {'alert_types': [], 'temporal_order': [], 'time_window_sec': 0, 'min_count': 0}
        score = AnomalyPatternLibrary._compute_signature_similarity(sig1, sig2)
        assert score == 1.0

    def test_disjoint_types_zero_type_score(self):
        sig1 = {
            'alert_types': ['a', 'b'],
            'temporal_order': ['a', 'b'],
            'time_window_sec': 30.0,
            'min_count': 2,
        }
        sig2 = {
            'alert_types': ['x', 'y'],
            'temporal_order': ['x', 'y'],
            'time_window_sec': 30.0,
            'min_count': 2,
        }
        score = AnomalyPatternLibrary._compute_signature_similarity(sig1, sig2)
        # type_score=0, order_score=0, window_score=1.0 => 0.2
        assert score == pytest.approx(0.2, abs=0.01)

    def test_time_window_proximity(self):
        sig_base = {
            'alert_types': ['a'],
            'temporal_order': ['a'],
            'time_window_sec': 30.0,
            'min_count': 1,
        }
        sig_close = dict(sig_base, time_window_sec=31.0)
        sig_far = dict(sig_base, time_window_sec=300.0)
        score_close = AnomalyPatternLibrary._compute_signature_similarity(sig_base, sig_close)
        score_far = AnomalyPatternLibrary._compute_signature_similarity(sig_base, sig_far)
        assert score_close > score_far
