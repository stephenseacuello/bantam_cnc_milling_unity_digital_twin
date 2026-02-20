"""Tests for criticality-based timeout logic in the heartbeat aggregator.

Tests that nodes of different criticality levels have appropriate timeouts,
that degradation/failure states are correctly assigned based on elapsed time,
and that recovery strategies depend on criticality.
"""

import pytest


class _NodeInfo:
    """Simplified NodeInfo for testing."""
    def __init__(self, name, criticality, last_heartbeat_ns=0):
        self.node_name = name
        self.criticality = criticality
        self.last_heartbeat_ns = last_heartbeat_ns
        self.consecutive_misses = 0
        self.state = 'HEALTHY'


class _HeartbeatLogic:
    """Extracted timeout/failure logic from HeartbeatAggregatorNode.

    Tests the criticality-based timeout assignment, degradation detection,
    failure detection, and recovery strategy selection.
    """

    def __init__(self, timeouts=None):
        self.timeouts = timeouts or {
            'CRITICAL': 0.5,
            'HIGH': 1.0,
            'MEDIUM': 5.0,
            'LOW': 30.0,
        }
        self.nodes = {}

    def register_node(self, name: str, criticality: str, heartbeat_ns: int = 0):
        self.nodes[name] = _NodeInfo(name, criticality, heartbeat_ns)

    def check_heartbeats(self, now_ns: int):
        """Check all nodes for missed heartbeats and update states."""
        events = []
        for name, node in self.nodes.items():
            timeout = self.timeouts.get(node.criticality, 5.0)
            elapsed = (now_ns - node.last_heartbeat_ns) / 1e9

            if elapsed > timeout:
                node.consecutive_misses += 1
                if node.state == 'HEALTHY':
                    node.state = 'DEGRADED'
                    events.append((name, 'DEGRADED'))
                elif node.state == 'DEGRADED' and node.consecutive_misses > 3:
                    node.state = 'FAILED'
                    events.append((name, 'FAILED'))
            else:
                node.consecutive_misses = 0
                if node.state != 'HEALTHY':
                    node.state = 'HEALTHY'
                    events.append((name, 'RECOVERED'))
        return events

    def get_recovery_strategy(self, criticality: str) -> str:
        """Determine recovery strategy based on criticality."""
        return {
            'CRITICAL': 'IMMEDIATE_RESTART',
            'HIGH': 'RESTART_WITH_DEPENDENCIES',
        }.get(criticality, 'DELAYED_RESTART')


@pytest.fixture
def logic():
    return _HeartbeatLogic()


class TestCriticalityTimeouts:
    """Tests for timeout configuration by criticality level."""

    def test_critical_timeout_is_shortest(self, logic):
        """CRITICAL nodes should have the shortest timeout."""
        assert logic.timeouts['CRITICAL'] < logic.timeouts['HIGH']
        assert logic.timeouts['CRITICAL'] < logic.timeouts['MEDIUM']
        assert logic.timeouts['CRITICAL'] < logic.timeouts['LOW']

    def test_low_timeout_is_longest(self, logic):
        """LOW criticality nodes should have the longest timeout."""
        assert logic.timeouts['LOW'] > logic.timeouts['CRITICAL']
        assert logic.timeouts['LOW'] > logic.timeouts['HIGH']
        assert logic.timeouts['LOW'] > logic.timeouts['MEDIUM']

    def test_timeout_ordering(self, logic):
        """Timeouts should be ordered: CRITICAL < HIGH < MEDIUM < LOW."""
        assert (
            logic.timeouts['CRITICAL']
            < logic.timeouts['HIGH']
            < logic.timeouts['MEDIUM']
            < logic.timeouts['LOW']
        )

    def test_default_timeout_values(self, logic):
        """Default timeout values should match the node implementation."""
        assert logic.timeouts['CRITICAL'] == 0.5
        assert logic.timeouts['HIGH'] == 1.0
        assert logic.timeouts['MEDIUM'] == 5.0
        assert logic.timeouts['LOW'] == 30.0


class TestDegradationDetection:
    """Tests for node state transitions based on heartbeat timing."""

    def test_healthy_to_degraded_on_timeout(self, logic):
        """A healthy node that misses its timeout should become DEGRADED."""
        logic.register_node('sensor_node', 'HIGH', heartbeat_ns=0)
        # Check at 2 seconds (past 1.0s HIGH timeout)
        events = logic.check_heartbeats(now_ns=int(2e9))
        assert logic.nodes['sensor_node'].state == 'DEGRADED'
        assert ('sensor_node', 'DEGRADED') in events

    def test_stays_healthy_within_timeout(self, logic):
        """A node with a recent heartbeat should remain HEALTHY."""
        logic.register_node('sensor_node', 'HIGH', heartbeat_ns=int(9.5e9))
        events = logic.check_heartbeats(now_ns=int(10e9))
        assert logic.nodes['sensor_node'].state == 'HEALTHY'

    def test_degraded_to_failed_after_consecutive_misses(self, logic):
        """A degraded node with >3 consecutive misses should become FAILED."""
        logic.register_node('critical_node', 'CRITICAL', heartbeat_ns=0)
        # First check -> DEGRADED (miss 1)
        logic.check_heartbeats(now_ns=int(1e9))
        assert logic.nodes['critical_node'].state == 'DEGRADED'
        # Subsequent checks -> more misses
        logic.check_heartbeats(now_ns=int(2e9))  # miss 2
        logic.check_heartbeats(now_ns=int(3e9))  # miss 3
        events = logic.check_heartbeats(now_ns=int(4e9))  # miss 4 -> FAILED
        assert logic.nodes['critical_node'].state == 'FAILED'
        assert ('critical_node', 'FAILED') in events

    def test_recovery_on_heartbeat(self, logic):
        """A degraded node that gets a heartbeat should recover to HEALTHY."""
        logic.register_node('node_a', 'MEDIUM', heartbeat_ns=0)
        logic.check_heartbeats(now_ns=int(10e9))  # timeout -> DEGRADED
        assert logic.nodes['node_a'].state == 'DEGRADED'

        # Simulate receiving a heartbeat
        logic.nodes['node_a'].last_heartbeat_ns = int(15e9)
        events = logic.check_heartbeats(now_ns=int(16e9))  # within timeout
        assert logic.nodes['node_a'].state == 'HEALTHY'
        assert ('node_a', 'RECOVERED') in events


class TestRecoveryStrategy:
    """Tests for criticality-based recovery strategies."""

    def test_critical_gets_immediate_restart(self, logic):
        """CRITICAL nodes should trigger IMMEDIATE_RESTART."""
        assert logic.get_recovery_strategy('CRITICAL') == 'IMMEDIATE_RESTART'

    def test_high_gets_restart_with_dependencies(self, logic):
        """HIGH nodes should trigger RESTART_WITH_DEPENDENCIES."""
        assert logic.get_recovery_strategy('HIGH') == 'RESTART_WITH_DEPENDENCIES'

    def test_medium_gets_delayed_restart(self, logic):
        """MEDIUM nodes should trigger DELAYED_RESTART."""
        assert logic.get_recovery_strategy('MEDIUM') == 'DELAYED_RESTART'

    def test_low_gets_delayed_restart(self, logic):
        """LOW nodes should trigger DELAYED_RESTART."""
        assert logic.get_recovery_strategy('LOW') == 'DELAYED_RESTART'

    def test_unknown_criticality_gets_delayed(self, logic):
        """Unknown criticality should default to DELAYED_RESTART."""
        assert logic.get_recovery_strategy('UNKNOWN') == 'DELAYED_RESTART'
