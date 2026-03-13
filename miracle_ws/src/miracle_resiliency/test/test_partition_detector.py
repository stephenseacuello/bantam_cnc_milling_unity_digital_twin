"""Tests for NetworkPartition and PartitionDetector.

Covers normal heartbeats, failure thresholds, partition classification
(FULL, PARTIAL, INTERMITTENT), recovery recommendations, node health
tracking, partition history, clearing, reachability, and edge cases.
"""

import sys
import types

import pytest

# ---------------------------------------------------------------------------
# ROS2 / miracle_core / miracle_msgs mocking
# ---------------------------------------------------------------------------
# Force-set attributes on the *existing* miracle_core and miracle_msgs
# submodules so that importing recovery_orchestrator succeeds without a
# real ROS2 installation.

def _ensure_module(fqn):
    """Return sys.modules[fqn], creating a bare types.ModuleType if needed."""
    if fqn not in sys.modules:
        sys.modules[fqn] = types.ModuleType(fqn)
    return sys.modules[fqn]


# --- rclpy stubs ---
_rclpy = _ensure_module('rclpy')
_rclpy_lifecycle = _ensure_module('rclpy.lifecycle')
_rclpy_cb = _ensure_module('rclpy.callback_groups')


class _TransitionCallbackReturn:
    SUCCESS = 0
    FAILURE = 1
    ERROR = 2


_rclpy_lifecycle.TransitionCallbackReturn = _TransitionCallbackReturn


class _ReentrantCallbackGroup:
    pass


_rclpy_cb.ReentrantCallbackGroup = _ReentrantCallbackGroup

# --- miracle_core stubs (force-set on submodules) ---
_mc = _ensure_module('miracle_core')
_mc_ln = _ensure_module('miracle_core.lifecycle_node_base')
_mc_qos = _ensure_module('miracle_core.qos_profiles')


class _MiracleLifecycleNode:
    CRITICALITY_HIGH = 'HIGH'

    def __init__(self, *a, **kw):
        pass


_mc_ln.MiracleLifecycleNode = _MiracleLifecycleNode


class _QoSProfiles:
    @staticmethod
    def alert():
        return None

    @staticmethod
    def state_data():
        return None


_mc_qos.QoSProfiles = _QoSProfiles

# --- miracle_msgs stubs (force-set on submodule) ---
_mm = _ensure_module('miracle_msgs')
_mm_msg = _ensure_module('miracle_msgs.msg')

_mm_msg.RecoveryRequest = type('RecoveryRequest', (), {})
_mm_msg.NodeFailure = type('NodeFailure', (), {})
_mm_msg.Heartbeat = type('Heartbeat', (), {})

# --- miracle_resiliency.lifecycle_client stub ---
_lc = _ensure_module('miracle_resiliency.lifecycle_client')


class _LifecycleClient:
    def __init__(self, *a, **kw):
        pass


class _LifecycleClientError(Exception):
    pass


class _LifecycleTransition:
    DEACTIVATE = 0
    CLEANUP = 1
    CONFIGURE = 2
    ACTIVATE = 3


_lc.LifecycleClient = _LifecycleClient
_lc.LifecycleClientError = _LifecycleClientError
_lc.LifecycleTransition = _LifecycleTransition

# ---------------------------------------------------------------------------
# Now we can safely import the module under test.
# ---------------------------------------------------------------------------
from miracle_resiliency.recovery_orchestrator import (  # noqa: E402
    NetworkPartition,
    PartitionDetector,
    SAFETY_NODES,
)


# ===========================================================================
# Fixtures
# ===========================================================================

@pytest.fixture
def detector():
    """A PartitionDetector with short timeout for fast tests."""
    return PartitionDetector(heartbeat_timeout_sec=1.0, min_failures=3)


@pytest.fixture
def detector_default():
    """A PartitionDetector with default parameters."""
    return PartitionDetector()


# ===========================================================================
# NetworkPartition dataclass
# ===========================================================================

class TestNetworkPartition:
    def test_fields(self):
        p = NetworkPartition(
            partition_id='p1',
            detected_at=100.0,
            affected_nodes=['a', 'b'],
            reachable_nodes=['a'],
            unreachable_nodes=['b'],
            partition_type='PARTIAL',
            estimated_severity=0.5,
            recovery_strategy='continue_degraded',
        )
        assert p.partition_id == 'p1'
        assert p.detected_at == 100.0
        assert p.partition_type == 'PARTIAL'
        assert p.estimated_severity == 0.5

    def test_equality(self):
        kwargs = dict(
            partition_id='x', detected_at=0.0, affected_nodes=[],
            reachable_nodes=[], unreachable_nodes=[],
            partition_type='FULL', estimated_severity=1.0,
            recovery_strategy='escalate_to_operator',
        )
        assert NetworkPartition(**kwargs) == NetworkPartition(**kwargs)


# ===========================================================================
# Normal heartbeats -> no partition
# ===========================================================================

class TestNormalHeartbeats:
    def test_no_nodes_no_partition(self, detector):
        assert detector.check_partitions(10.0) == []

    def test_all_healthy_no_partition(self, detector):
        detector.record_heartbeat('node_a', 1.0)
        detector.record_heartbeat('node_b', 1.0)
        assert detector.check_partitions(2.0) == []

    def test_heartbeat_resets_failures(self, detector):
        detector.record_heartbeat_failure('n1', 1.0)
        detector.record_heartbeat_failure('n1', 2.0)
        # Two failures, below min_failures=3
        detector.record_heartbeat('n1', 3.0)
        assert detector.check_partitions(4.0) == []


# ===========================================================================
# Single node timeout -- not yet a partition (below min_failures)
# ===========================================================================

class TestBelowThreshold:
    def test_single_failure_not_partition(self, detector):
        detector.record_heartbeat('n1', 1.0)
        detector.record_heartbeat_failure('n1', 2.0)
        assert detector.check_partitions(3.0) == []

    def test_two_failures_not_partition(self, detector):
        detector.record_heartbeat('n1', 1.0)
        detector.record_heartbeat_failure('n1', 2.0)
        detector.record_heartbeat_failure('n1', 3.0)
        assert detector.check_partitions(4.0) == []


# ===========================================================================
# Multiple failures -> unreachable
# ===========================================================================

class TestUnreachable:
    def test_three_failures_triggers_partition(self, detector):
        detector.record_heartbeat('n1', 1.0)
        detector.record_heartbeat_failure('n1', 2.0)
        detector.record_heartbeat_failure('n1', 3.0)
        detector.record_heartbeat_failure('n1', 4.0)
        parts = detector.check_partitions(5.0)
        assert len(parts) == 1
        assert 'n1' in parts[0].unreachable_nodes

    def test_failure_count_exact_threshold(self, detector):
        """Exactly min_failures consecutive failures."""
        for i in range(3):
            detector.record_heartbeat_failure('x', float(i))
        parts = detector.check_partitions(10.0)
        assert len(parts) == 1

    def test_node_recovers_after_failures(self, detector):
        for i in range(3):
            detector.record_heartbeat_failure('n1', float(i))
        assert len(detector.check_partitions(5.0)) == 1
        # Node recovers
        detector.record_heartbeat('n1', 6.0)
        assert detector.check_partitions(7.0) == []


# ===========================================================================
# Simultaneous loss -> FULL partition
# ===========================================================================

class TestFullPartition:
    def test_all_nodes_unreachable(self, detector):
        for node in ['a', 'b', 'c']:
            for i in range(3):
                detector.record_heartbeat_failure(node, float(i))
        parts = detector.check_partitions(10.0)
        assert len(parts) == 1
        assert parts[0].partition_type == 'FULL'
        assert parts[0].estimated_severity == 1.0

    def test_full_partition_recovery_strategy(self, detector):
        for node in ['a', 'b']:
            for i in range(3):
                detector.record_heartbeat_failure(node, float(i))
        parts = detector.check_partitions(10.0)
        assert parts[0].recovery_strategy == 'escalate_to_operator'


# ===========================================================================
# Partial loss -> PARTIAL partition
# ===========================================================================

class TestPartialPartition:
    def test_some_unreachable(self, detector):
        detector.record_heartbeat('healthy', 1.0)
        for i in range(3):
            detector.record_heartbeat_failure('sick', float(i))
        parts = detector.check_partitions(5.0)
        assert len(parts) == 1
        assert parts[0].partition_type == 'PARTIAL'
        assert 'healthy' in parts[0].reachable_nodes
        assert 'sick' in parts[0].unreachable_nodes

    def test_partial_severity(self, detector):
        detector.record_heartbeat('ok1', 1.0)
        detector.record_heartbeat('ok2', 1.0)
        for i in range(3):
            detector.record_heartbeat_failure('bad', float(i))
        parts = detector.check_partitions(5.0)
        # 1 out of 3 unreachable
        assert 0.3 < parts[0].estimated_severity < 0.4

    def test_partial_non_critical_recovery(self, detector):
        detector.record_heartbeat('ok', 1.0)
        for i in range(3):
            detector.record_heartbeat_failure('non_critical_node', float(i))
        parts = detector.check_partitions(5.0)
        assert parts[0].recovery_strategy == 'continue_degraded'


# ===========================================================================
# Flapping node -> INTERMITTENT
# ===========================================================================

class TestIntermittentPartition:
    def test_flapping_detected(self, detector):
        # Node has had successful heartbeats AND failures
        detector.record_heartbeat('flapper', 1.0)
        detector.record_heartbeat_failure('flapper', 2.0)
        detector.record_heartbeat_failure('flapper', 3.0)
        detector.record_heartbeat_failure('flapper', 4.0)
        parts = detector.check_partitions(5.0)
        assert len(parts) == 1
        assert parts[0].partition_type == 'INTERMITTENT'

    def test_intermittent_recovery_strategy(self, detector):
        detector.record_heartbeat('flapper', 1.0)
        for i in range(3):
            detector.record_heartbeat_failure('flapper', 2.0 + i)
        parts = detector.check_partitions(6.0)
        assert parts[0].recovery_strategy == 'increase_heartbeat_frequency'


# ===========================================================================
# Recovery recommendations per type
# ===========================================================================

class TestRecoveryRecommendations:
    def test_full_recommendation(self, detector):
        p = NetworkPartition(
            partition_id='t', detected_at=0.0,
            affected_nodes=['a'], reachable_nodes=[],
            unreachable_nodes=['a'], partition_type='FULL',
            estimated_severity=1.0, recovery_strategy='escalate_to_operator',
        )
        assert detector.get_recommended_recovery(p) == 'escalate_to_operator'

    def test_partial_safety_recommendation(self, detector):
        safety = list(SAFETY_NODES)[0]
        p = NetworkPartition(
            partition_id='t', detected_at=0.0,
            affected_nodes=[safety, 'ok'], reachable_nodes=['ok'],
            unreachable_nodes=[safety], partition_type='PARTIAL',
            estimated_severity=0.5, recovery_strategy='emergency_stop',
        )
        assert detector.get_recommended_recovery(p) == 'emergency_stop'

    def test_partial_non_critical_recommendation(self, detector):
        p = NetworkPartition(
            partition_id='t', detected_at=0.0,
            affected_nodes=['x', 'ok'], reachable_nodes=['ok'],
            unreachable_nodes=['x'], partition_type='PARTIAL',
            estimated_severity=0.5, recovery_strategy='continue_degraded',
        )
        assert detector.get_recommended_recovery(p) == 'continue_degraded'

    def test_intermittent_recommendation(self, detector):
        p = NetworkPartition(
            partition_id='t', detected_at=0.0,
            affected_nodes=['a'], reachable_nodes=[],
            unreachable_nodes=['a'], partition_type='INTERMITTENT',
            estimated_severity=0.5,
            recovery_strategy='increase_heartbeat_frequency',
        )
        assert detector.get_recommended_recovery(p) == 'increase_heartbeat_frequency'


# ===========================================================================
# Safety node partition -> emergency_stop
# ===========================================================================

class TestSafetyNodePartition:
    def test_safety_node_triggers_emergency(self, detector):
        safety = 'emergency_stop'
        detector.record_heartbeat('ok', 1.0)
        for i in range(3):
            detector.record_heartbeat_failure(safety, float(i))
        parts = detector.check_partitions(5.0)
        assert len(parts) == 1
        assert parts[0].recovery_strategy == 'emergency_stop'

    def test_safety_monitor_triggers_emergency(self, detector):
        detector.record_heartbeat('ok', 1.0)
        for i in range(3):
            detector.record_heartbeat_failure('safety_monitor', float(i))
        parts = detector.check_partitions(5.0)
        assert parts[0].recovery_strategy == 'emergency_stop'


# ===========================================================================
# Node health tracking
# ===========================================================================

class TestNodeHealth:
    def test_empty_health(self, detector):
        assert detector.get_node_health() == {}

    def test_healthy_node(self, detector):
        detector.record_heartbeat('n1', 1.0)
        health = detector.get_node_health()
        assert health['n1']['status'] == 'reachable'
        assert health['n1']['last_seen'] == 1.0
        assert health['n1']['failure_count'] == 0
        assert health['n1']['uptime_pct'] == 100.0

    def test_unhealthy_node(self, detector):
        for i in range(3):
            detector.record_heartbeat_failure('n1', float(i))
        health = detector.get_node_health()
        assert health['n1']['status'] == 'unreachable'
        assert health['n1']['failure_count'] == 3
        assert health['n1']['uptime_pct'] == 0.0

    def test_mixed_health(self, detector):
        detector.record_heartbeat('n1', 1.0)
        detector.record_heartbeat_failure('n1', 2.0)
        health = detector.get_node_health()
        assert health['n1']['failure_count'] == 1
        # 1 success out of 2 checks = 50%
        assert health['n1']['uptime_pct'] == 50.0


# ===========================================================================
# Partition history
# ===========================================================================

class TestPartitionHistory:
    def test_empty_history(self, detector):
        assert detector.get_partition_history() == []

    def test_history_accumulates(self, detector):
        for i in range(3):
            detector.record_heartbeat_failure('n1', float(i))
        detector.check_partitions(10.0)
        # Recover and fail again
        detector.record_heartbeat('n1', 11.0)
        for i in range(3):
            detector.record_heartbeat_failure('n1', 12.0 + i)
        detector.check_partitions(20.0)
        assert len(detector.get_partition_history()) == 2

    def test_history_is_copy(self, detector):
        for i in range(3):
            detector.record_heartbeat_failure('n1', float(i))
        detector.check_partitions(10.0)
        h = detector.get_partition_history()
        h.clear()
        assert len(detector.get_partition_history()) == 1


# ===========================================================================
# Clear partition
# ===========================================================================

class TestClearPartition:
    def test_clear_removes_from_active(self, detector):
        for i in range(3):
            detector.record_heartbeat_failure('n1', float(i))
        parts = detector.check_partitions(10.0)
        pid = parts[0].partition_id
        detector.clear_partition(pid)
        # History still has it, but active does not
        assert len(detector.get_partition_history()) == 1
        assert pid not in detector._active_partitions

    def test_clear_nonexistent(self, detector):
        # Should not raise
        detector.clear_partition('nonexistent')


# ===========================================================================
# Reachability check
# ===========================================================================

class TestReachability:
    def test_unknown_node_is_reachable(self, detector):
        assert detector.is_node_reachable('unknown') is True

    def test_healthy_node_reachable(self, detector):
        detector.record_heartbeat('n1', 1.0)
        assert detector.is_node_reachable('n1') is True

    def test_failed_node_unreachable(self, detector):
        for i in range(3):
            detector.record_heartbeat_failure('n1', float(i))
        assert detector.is_node_reachable('n1') is False

    def test_recovered_node_reachable(self, detector):
        for i in range(3):
            detector.record_heartbeat_failure('n1', float(i))
        assert detector.is_node_reachable('n1') is False
        detector.record_heartbeat('n1', 5.0)
        assert detector.is_node_reachable('n1') is True


# ===========================================================================
# Edge cases
# ===========================================================================

class TestEdgeCases:
    def test_single_node_full_partition(self, detector):
        for i in range(3):
            detector.record_heartbeat_failure('only', float(i))
        parts = detector.check_partitions(10.0)
        assert len(parts) == 1
        assert parts[0].partition_type == 'FULL'

    def test_default_timeout_values(self, detector_default):
        assert detector_default._heartbeat_timeout_sec == 5.0
        assert detector_default._min_failures == 3

    def test_custom_min_failures(self):
        d = PartitionDetector(heartbeat_timeout_sec=1.0, min_failures=5)
        for i in range(4):
            d.record_heartbeat_failure('n1', float(i))
        assert d.check_partitions(10.0) == []
        d.record_heartbeat_failure('n1', 5.0)
        assert len(d.check_partitions(11.0)) == 1

    def test_many_nodes_severity(self, detector):
        """10 nodes, 3 unreachable -> severity = 0.3"""
        # 7 healthy nodes
        for i in range(7):
            detector.record_heartbeat(f'good_{i}', 1.0)
        # 3 bad nodes (no prior heartbeat, so not intermittent)
        for bad in ['bad_0', 'bad_1', 'bad_2']:
            for j in range(3):
                detector.record_heartbeat_failure(bad, 2.0 + j)
        parts = detector.check_partitions(10.0)
        assert len(parts) == 1
        assert parts[0].estimated_severity == 0.3

    def test_partition_id_is_unique(self, detector):
        for i in range(3):
            detector.record_heartbeat_failure('n1', float(i))
        p1 = detector.check_partitions(10.0)
        detector.record_heartbeat('n1', 11.0)
        for i in range(3):
            detector.record_heartbeat_failure('n1', 12.0 + i)
        p2 = detector.check_partitions(20.0)
        assert p1[0].partition_id != p2[0].partition_id

    def test_affected_nodes_sorted(self, detector):
        detector.record_heartbeat('z_node', 1.0)
        detector.record_heartbeat('a_node', 1.0)
        for i in range(3):
            detector.record_heartbeat_failure('m_node', float(i))
        parts = detector.check_partitions(10.0)
        assert parts[0].affected_nodes == sorted(parts[0].affected_nodes)
        assert parts[0].reachable_nodes == sorted(parts[0].reachable_nodes)
        assert parts[0].unreachable_nodes == sorted(parts[0].unreachable_nodes)
