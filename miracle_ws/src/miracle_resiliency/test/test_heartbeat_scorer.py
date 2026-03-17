"""Tests for HeartbeatHealthScorer.

Covers heartbeat recording, missed heartbeat tracking, score computation,
score formula weights, trend detection, sorting, degrading node filtering,
sliding window enforcement, and edge cases.
"""

import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants',
            'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
            'rclpy.callback_groups',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_msgs', 'miracle_msgs.msg',
            'std_msgs', 'std_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_resiliency.recovery_orchestrator import (
    HeartbeatHealthScorer,
    HeartbeatRecord,
    NodeHealthScore,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def scorer():
    """Return a fresh HeartbeatHealthScorer instance."""
    return HeartbeatHealthScorer()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestHeartbeatRecord:
    """Verify HeartbeatRecord dataclass construction."""

    def test_dataclass_fields(self):
        rec = HeartbeatRecord(
            node_id='sensor_1',
            timestamp=1000.0,
            sequence_number=42,
            payload_size=128,
            round_trip_ms=5.5,
        )
        assert rec.node_id == 'sensor_1'
        assert rec.timestamp == 1000.0
        assert rec.sequence_number == 42
        assert rec.payload_size == 128
        assert rec.round_trip_ms == 5.5


class TestRecordHeartbeat:
    """Verify that heartbeats are recorded and affect scores."""

    def test_single_heartbeat_produces_high_score(self, scorer):
        scorer.record_heartbeat('node_a', 1000.0, 1, 5.0)
        score = scorer.get_score('node_a')
        assert score.node_id == 'node_a'
        assert score.overall_score > 80.0
        assert score.availability_pct == 100.0
        assert score.missed_beats == 0
        assert score.last_seen == 1000.0

    def test_multiple_heartbeats_low_latency(self, scorer):
        for i in range(20):
            scorer.record_heartbeat('node_b', 1000.0 + i, i, 2.0)
        score = scorer.get_score('node_b')
        assert score.availability_pct == 100.0
        assert score.avg_latency_ms == pytest.approx(2.0, abs=0.01)
        assert score.jitter_ms == pytest.approx(0.0, abs=0.01)
        assert score.overall_score > 90.0


class TestRecordMissed:
    """Verify that missed heartbeats reduce availability and score."""

    def test_missed_reduces_availability(self, scorer):
        for i in range(5):
            scorer.record_heartbeat('node_c', 1000.0 + i, i, 3.0)
        for i in range(5):
            scorer.record_missed('node_c', 1005.0 + i)
        score = scorer.get_score('node_c')
        assert score.availability_pct == pytest.approx(50.0, abs=0.01)
        assert score.missed_beats == 5

    def test_all_missed_gives_zero_availability(self, scorer):
        for i in range(10):
            scorer.record_missed('node_d', 1000.0 + i)
        score = scorer.get_score('node_d')
        assert score.availability_pct == 0.0
        assert score.overall_score == 0.0
        assert score.trend == 'critical'


class TestScoreFormula:
    """Verify the weighted score formula components."""

    def test_perfect_score_components(self, scorer):
        # All heartbeats received, zero latency, zero jitter
        for i in range(20):
            scorer.record_heartbeat('perfect', 1000.0 + i, i, 0.0)
        score = scorer.get_score('perfect')
        # availability=100 * 0.4 = 40
        # latency=100 * 0.3 = 30
        # jitter=100 * 0.2 = 20
        # trend_bonus=75 (stable) * 0.1 = 7.5
        assert score.overall_score == pytest.approx(97.5, abs=0.1)

    def test_high_latency_reduces_score(self, scorer):
        for i in range(10):
            scorer.record_heartbeat('slow', 1000.0 + i, i, 40.0)
        score = scorer.get_score('slow')
        # latency_score = max(0, 100 - 40*2) = 20
        assert score.avg_latency_ms == pytest.approx(40.0, abs=0.01)
        # Overall should be lower than perfect
        assert score.overall_score < 90.0

    def test_high_jitter_reduces_score(self, scorer):
        # Alternate between 0ms and 40ms RTT -> high jitter
        for i in range(20):
            rtt = 0.0 if i % 2 == 0 else 40.0
            scorer.record_heartbeat('jittery', 1000.0 + i, i, rtt)
        score = scorer.get_score('jittery')
        assert score.jitter_ms > 10.0
        # Jitter component penalised
        assert score.overall_score < 95.0


class TestGetAllScores:
    """Verify sorting and completeness of get_all_scores."""

    def test_sorted_worst_first(self, scorer):
        # 'bad' node has many misses
        for i in range(2):
            scorer.record_heartbeat('bad', 1000.0 + i, i, 5.0)
        for i in range(8):
            scorer.record_missed('bad', 1002.0 + i)

        # 'good' node is healthy
        for i in range(10):
            scorer.record_heartbeat('good', 1000.0 + i, i, 1.0)

        scores = scorer.get_all_scores()
        assert len(scores) == 2
        assert scores[0].node_id == 'bad'
        assert scores[1].node_id == 'good'
        assert scores[0].overall_score < scores[1].overall_score


class TestGetDegradingNodes:
    """Verify filtering by threshold."""

    def test_default_threshold(self, scorer):
        # 'healthy' node
        for i in range(20):
            scorer.record_heartbeat('healthy', 1000.0 + i, i, 1.0)
        # 'sick' node with many misses
        for i in range(2):
            scorer.record_heartbeat('sick', 1000.0 + i, i, 1.0)
        for i in range(18):
            scorer.record_missed('sick', 1002.0 + i)
        degrading = scorer.get_degrading_nodes()
        node_ids = [s.node_id for s in degrading]
        assert 'sick' in node_ids
        assert 'healthy' not in node_ids

    def test_custom_threshold(self, scorer):
        for i in range(10):
            scorer.record_heartbeat('mediocre', 1000.0 + i, i, 20.0)
        for i in range(5):
            scorer.record_missed('mediocre', 1010.0 + i)
        degrading = scorer.get_degrading_nodes(threshold=95.0)
        node_ids = [s.node_id for s in degrading]
        assert 'mediocre' in node_ids


class TestSlidingWindow:
    """Verify that only the last N heartbeats are retained."""

    def test_window_enforced(self):
        scorer = HeartbeatHealthScorer(window_size=10)
        # Record 25 heartbeats
        for i in range(25):
            scorer.record_heartbeat('win_node', 1000.0 + i, i, 3.0)
        # Internal window should hold at most 10
        assert len(scorer._heartbeats['win_node']) == 10
        # The oldest kept should be sequence 15
        assert scorer._heartbeats['win_node'][0].sequence_number == 15


class TestTrendDetection:
    """Verify trend classification based on score history."""

    def test_stable_trend(self, scorer):
        for i in range(20):
            scorer.record_heartbeat('stable_node', 1000.0 + i, i, 5.0)
        # Build up score history by calling get_score multiple times
        for _ in range(25):
            scorer.get_score('stable_node')
        score = scorer.get_score('stable_node')
        assert score.trend == 'stable'

    def test_critical_when_only_misses(self, scorer):
        for i in range(5):
            scorer.record_missed('dead_node', 1000.0 + i)
        score = scorer.get_score('dead_node')
        assert score.trend == 'critical'
        assert score.overall_score == 0.0


class TestNodeHealthScoreDataclass:
    """Verify NodeHealthScore dataclass construction."""

    def test_fields(self):
        nhs = NodeHealthScore(
            node_id='test',
            overall_score=85.0,
            availability_pct=95.0,
            avg_latency_ms=10.0,
            jitter_ms=2.5,
            missed_beats=3,
            trend='stable',
            last_seen=1234.5,
        )
        assert nhs.node_id == 'test'
        assert nhs.overall_score == 85.0
        assert nhs.trend == 'stable'
        assert nhs.missed_beats == 3
