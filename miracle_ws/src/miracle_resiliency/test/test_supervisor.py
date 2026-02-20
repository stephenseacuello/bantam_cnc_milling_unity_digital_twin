"""Tests for Erlang-style restart strategies in the root supervisor.

Tests the three restart strategies (one_for_one, one_for_all,
rest_for_one), restart budget enforcement, and state transitions.
"""

from enum import Enum

import pytest


class RestartStrategy(Enum):
    ONE_FOR_ONE = 'one_for_one'
    ONE_FOR_ALL = 'one_for_all'
    REST_FOR_ONE = 'rest_for_one'


class _SupervisedChild:
    """Simplified supervised child for testing."""
    def __init__(self, name, criticality='MEDIUM', max_restarts=5, window_sec=300.0):
        self.name = name
        self.criticality = criticality
        self.restart_count = 0
        self.max_restarts = max_restarts
        self.restart_window_sec = window_sec
        self.restart_timestamps = []
        self.state = 'RUNNING'


class _Supervisor:
    """Extracted supervision logic from SupervisorRootNode.

    Implements restart strategy selection, restart budget enforcement,
    and the three Erlang-style strategies.
    """

    def __init__(self, strategy=RestartStrategy.ONE_FOR_ONE, max_restarts=5, window=300.0):
        self.strategy = strategy
        self.max_restarts = max_restarts
        self.window = window
        self.children = {}
        self.restart_log = []  # Track which nodes were restarted

    def add_child(self, name: str, criticality: str = 'MEDIUM'):
        self.children[name] = _SupervisedChild(
            name, criticality, self.max_restarts, self.window
        )

    def handle_failure(self, node_name: str, now: float) -> str:
        """Handle a node failure. Returns the action taken."""
        if node_name not in self.children:
            self.children[node_name] = _SupervisedChild(
                node_name, 'MEDIUM', self.max_restarts, self.window
            )

        child = self.children[node_name]
        child.state = 'FAILED'

        # Check restart budget
        child.restart_timestamps = [
            t for t in child.restart_timestamps
            if now - t < child.restart_window_sec
        ]

        if len(child.restart_timestamps) >= child.max_restarts:
            child.state = 'ABANDONED'
            return 'ABANDONED'

        child.restart_timestamps.append(now)
        child.restart_count += 1

        # Apply strategy
        if self.strategy == RestartStrategy.ONE_FOR_ONE:
            self._restart_one(node_name)
            return 'RESTART_ONE'
        elif self.strategy == RestartStrategy.ONE_FOR_ALL:
            self._restart_all()
            return 'RESTART_ALL'
        elif self.strategy == RestartStrategy.REST_FOR_ONE:
            self._restart_rest(node_name)
            return 'RESTART_REST'
        return 'UNKNOWN'

    def _restart_one(self, name: str):
        """Restart only the failed node."""
        self.restart_log.append(name)
        self.children[name].state = 'RUNNING'

    def _restart_all(self):
        """Restart all supervised children."""
        for name in self.children:
            self.restart_log.append(name)
            self.children[name].state = 'RUNNING'

    def _restart_rest(self, failed_name: str):
        """Restart the failed node and all nodes registered after it."""
        found = False
        for name in self.children:
            if name == failed_name:
                found = True
            if found:
                self.restart_log.append(name)
                self.children[name].state = 'RUNNING'


@pytest.fixture
def one_for_one():
    s = _Supervisor(RestartStrategy.ONE_FOR_ONE, max_restarts=3, window=60.0)
    s.add_child('node_a', 'CRITICAL')
    s.add_child('node_b', 'HIGH')
    s.add_child('node_c', 'MEDIUM')
    return s


@pytest.fixture
def one_for_all():
    s = _Supervisor(RestartStrategy.ONE_FOR_ALL, max_restarts=3, window=60.0)
    s.add_child('node_a', 'CRITICAL')
    s.add_child('node_b', 'HIGH')
    s.add_child('node_c', 'MEDIUM')
    return s


@pytest.fixture
def rest_for_one():
    s = _Supervisor(RestartStrategy.REST_FOR_ONE, max_restarts=3, window=60.0)
    s.add_child('node_a', 'CRITICAL')
    s.add_child('node_b', 'HIGH')
    s.add_child('node_c', 'MEDIUM')
    return s


class TestOneForOneStrategy:
    """Tests for the one-for-one restart strategy."""

    def test_only_failed_node_restarted(self, one_for_one):
        """Only the failed node should be restarted."""
        result = one_for_one.handle_failure('node_b', now=100.0)
        assert result == 'RESTART_ONE'
        assert 'node_b' in one_for_one.restart_log
        assert 'node_a' not in one_for_one.restart_log
        assert 'node_c' not in one_for_one.restart_log

    def test_failed_node_returns_to_running(self, one_for_one):
        """After restart, the failed node should be RUNNING."""
        one_for_one.handle_failure('node_a', now=100.0)
        assert one_for_one.children['node_a'].state == 'RUNNING'

    def test_other_nodes_unaffected(self, one_for_one):
        """Nodes that did not fail should remain RUNNING."""
        one_for_one.handle_failure('node_b', now=100.0)
        assert one_for_one.children['node_a'].state == 'RUNNING'
        assert one_for_one.children['node_c'].state == 'RUNNING'


class TestOneForAllStrategy:
    """Tests for the one-for-all restart strategy."""

    def test_all_nodes_restarted(self, one_for_all):
        """All supervised nodes should be restarted when one fails."""
        result = one_for_all.handle_failure('node_b', now=100.0)
        assert result == 'RESTART_ALL'
        assert 'node_a' in one_for_all.restart_log
        assert 'node_b' in one_for_all.restart_log
        assert 'node_c' in one_for_all.restart_log

    def test_all_nodes_running_after_restart(self, one_for_all):
        """All nodes should be RUNNING after a one-for-all restart."""
        one_for_all.handle_failure('node_a', now=100.0)
        for child in one_for_all.children.values():
            assert child.state == 'RUNNING'


class TestRestForOneStrategy:
    """Tests for the rest-for-one restart strategy."""

    def test_failed_and_later_nodes_restarted(self, rest_for_one):
        """The failed node and all nodes after it should be restarted."""
        result = rest_for_one.handle_failure('node_b', now=100.0)
        assert result == 'RESTART_REST'
        assert 'node_a' not in rest_for_one.restart_log, (
            "node_a (before node_b) should not be restarted"
        )
        assert 'node_b' in rest_for_one.restart_log
        assert 'node_c' in rest_for_one.restart_log

    def test_first_node_failure_restarts_all(self, rest_for_one):
        """If the first node fails, all subsequent nodes should restart too."""
        rest_for_one.handle_failure('node_a', now=100.0)
        assert set(rest_for_one.restart_log) == {'node_a', 'node_b', 'node_c'}

    def test_last_node_only_restarts_itself(self, rest_for_one):
        """If the last node fails, only it should be restarted."""
        rest_for_one.handle_failure('node_c', now=100.0)
        assert rest_for_one.restart_log == ['node_c']


class TestRestartBudget:
    """Tests for restart budget enforcement."""

    def test_abandoned_after_max_restarts(self, one_for_one):
        """A node should be ABANDONED after exceeding max_restarts within the window."""
        # max_restarts = 3, window = 60s
        one_for_one.handle_failure('node_a', now=1.0)   # restart 1
        one_for_one.handle_failure('node_a', now=2.0)   # restart 2
        one_for_one.handle_failure('node_a', now=3.0)   # restart 3
        result = one_for_one.handle_failure('node_a', now=4.0)  # exceeds budget
        assert result == 'ABANDONED'
        assert one_for_one.children['node_a'].state == 'ABANDONED'

    def test_budget_resets_after_window(self, one_for_one):
        """Restarts outside the time window should not count against the budget."""
        one_for_one.handle_failure('node_a', now=1.0)    # restart 1
        one_for_one.handle_failure('node_a', now=2.0)    # restart 2
        one_for_one.handle_failure('node_a', now=3.0)    # restart 3
        # Now at now=100 (beyond 60s window), old timestamps should expire
        result = one_for_one.handle_failure('node_a', now=100.0)
        assert result == 'RESTART_ONE', (
            "Old timestamps should have expired; restart should be allowed"
        )

    def test_restart_count_increments(self, one_for_one):
        """Each restart should increment the child's restart_count."""
        one_for_one.handle_failure('node_b', now=1.0)
        one_for_one.handle_failure('node_b', now=2.0)
        assert one_for_one.children['node_b'].restart_count == 2

    def test_unknown_node_auto_registered(self, one_for_one):
        """A failure for an unregistered node should auto-register it."""
        result = one_for_one.handle_failure('new_node', now=100.0)
        assert result == 'RESTART_ONE'
        assert 'new_node' in one_for_one.children
