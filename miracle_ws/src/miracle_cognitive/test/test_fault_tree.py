"""Tests for FaultTreeAnalyzer fault tree analysis.

Tests gate logic (AND, OR, VOTING), tree evaluation, minimal cut sets,
Birnbaum importance analysis, and critical path identification.
"""

import sys
from unittest.mock import MagicMock

for mod in [
    'miracle_core.datatypes', 'miracle_core.constants',
    'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
    'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
    'miracle_msgs', 'miracle_msgs.msg',
]:
    sys.modules.setdefault(mod, MagicMock())

import pytest
from miracle_cognitive.knowledge.reasoning_engine import (
    FaultTreeAnalyzer,
    FaultTree,
    FaultTreeNode,
    FaultTreeNodeType,
    FaultTreeGateType,
)


@pytest.fixture
def analyzer():
    return FaultTreeAnalyzer()


# ---- helpers ----

def _event(node_id: str, name: str, probability: float) -> FaultTreeNode:
    """Shorthand for creating a basic event node."""
    return FaultTreeNode(
        node_id=node_id,
        name=name,
        node_type=FaultTreeNodeType.EVENT,
        probability=probability,
    )


def _gate(node_id: str, name: str, gate_type: FaultTreeGateType,
          children: list, voting_k: int = 0) -> FaultTreeNode:
    """Shorthand for creating a gate node."""
    return FaultTreeNode(
        node_id=node_id,
        name=name,
        node_type=FaultTreeNodeType.GATE,
        gate_type=gate_type,
        children=children,
        voting_k=voting_k,
    )


# ---- tests ----

class TestANDGate:
    """AND gate probability = product of children."""

    def test_and_gate_probability(self, analyzer):
        nodes = {
            'top': _gate('top', 'Top Event', FaultTreeGateType.AND, ['e1', 'e2']),
            'e1': _event('e1', 'Sensor Failure', 0.1),
            'e2': _event('e2', 'Actuator Failure', 0.2),
        }
        tree = analyzer.build_tree('top', nodes)
        prob = analyzer.evaluate(tree)
        assert prob == pytest.approx(0.1 * 0.2, abs=1e-9)


class TestORGate:
    """OR gate probability = 1 - product of (1 - child_prob)."""

    def test_or_gate_probability(self, analyzer):
        nodes = {
            'top': _gate('top', 'Top Event', FaultTreeGateType.OR, ['e1', 'e2']),
            'e1': _event('e1', 'Power Failure', 0.1),
            'e2': _event('e2', 'Coolant Failure', 0.2),
        }
        tree = analyzer.build_tree('top', nodes)
        prob = analyzer.evaluate(tree)
        expected = 1.0 - (1.0 - 0.1) * (1.0 - 0.2)
        assert prob == pytest.approx(expected, abs=1e-9)


class TestNestedGates:
    """Nested structure: AND gate under an OR gate."""

    def test_nested_and_under_or(self, analyzer):
        #     OR(top)
        #    /       \
        #  AND(g1)    e3
        #  /    \
        # e1    e2
        nodes = {
            'top': _gate('top', 'System Failure', FaultTreeGateType.OR, ['g1', 'e3']),
            'g1': _gate('g1', 'Dual Sensor Failure', FaultTreeGateType.AND, ['e1', 'e2']),
            'e1': _event('e1', 'Sensor A', 0.3),
            'e2': _event('e2', 'Sensor B', 0.4),
            'e3': _event('e3', 'Controller Failure', 0.05),
        }
        tree = analyzer.build_tree('top', nodes)
        prob = analyzer.evaluate(tree)

        p_g1 = 0.3 * 0.4  # AND
        expected = 1.0 - (1.0 - p_g1) * (1.0 - 0.05)  # OR
        assert prob == pytest.approx(expected, abs=1e-9)


class TestSingleEvent:
    """A tree with only a single event (degenerate case)."""

    def test_single_event_tree(self, analyzer):
        nodes = {
            'e1': _event('e1', 'Catastrophic Bearing Failure', 0.001),
        }
        tree = analyzer.build_tree('e1', nodes)
        prob = analyzer.evaluate(tree)
        assert prob == pytest.approx(0.001, abs=1e-12)


class TestMinimalCutSets:
    """Minimal cut set identification."""

    def test_cut_sets_or_gate(self, analyzer):
        # OR gate: each child is its own minimal cut set
        nodes = {
            'top': _gate('top', 'Top', FaultTreeGateType.OR, ['e1', 'e2', 'e3']),
            'e1': _event('e1', 'A', 0.1),
            'e2': _event('e2', 'B', 0.2),
            'e3': _event('e3', 'C', 0.3),
        }
        tree = analyzer.build_tree('top', nodes)
        mcs = analyzer.get_minimal_cut_sets(tree)
        assert len(mcs) == 3
        assert {'e1'} in mcs
        assert {'e2'} in mcs
        assert {'e3'} in mcs

    def test_cut_sets_and_gate(self, analyzer):
        # AND gate: single cut set = all children
        nodes = {
            'top': _gate('top', 'Top', FaultTreeGateType.AND, ['e1', 'e2']),
            'e1': _event('e1', 'A', 0.1),
            'e2': _event('e2', 'B', 0.2),
        }
        tree = analyzer.build_tree('top', nodes)
        mcs = analyzer.get_minimal_cut_sets(tree)
        assert len(mcs) == 1
        assert {'e1', 'e2'} in mcs

    def test_cut_sets_nested(self, analyzer):
        # OR(AND(e1,e2), e3) -> MCS: {e1,e2}, {e3}
        nodes = {
            'top': _gate('top', 'Top', FaultTreeGateType.OR, ['g1', 'e3']),
            'g1': _gate('g1', 'Sub', FaultTreeGateType.AND, ['e1', 'e2']),
            'e1': _event('e1', 'A', 0.3),
            'e2': _event('e2', 'B', 0.4),
            'e3': _event('e3', 'C', 0.05),
        }
        tree = analyzer.build_tree('top', nodes)
        mcs = analyzer.get_minimal_cut_sets(tree)
        assert len(mcs) == 2
        assert {'e1', 'e2'} in mcs
        assert {'e3'} in mcs


class TestImportanceAnalysis:
    """Birnbaum importance identifies the most critical basic event."""

    def test_importance_most_critical(self, analyzer):
        #     OR(top)
        #    /       \
        #  AND(g1)    e3(0.5)
        #  /    \
        # e1(0.1) e2(0.9)
        nodes = {
            'top': _gate('top', 'Top', FaultTreeGateType.OR, ['g1', 'e3']),
            'g1': _gate('g1', 'Sub', FaultTreeGateType.AND, ['e1', 'e2']),
            'e1': _event('e1', 'Rare Fault', 0.1),
            'e2': _event('e2', 'Common Fault', 0.9),
            'e3': _event('e3', 'Controller Bug', 0.5),
        }
        tree = analyzer.build_tree('top', nodes)
        importances = analyzer.importance_analysis(tree)

        # e3 is in a single-event OR branch: I_B(e3) = P(top|e3=1) - P(top|e3=0)
        # When e3=1, OR top = 1.0.  When e3=0, top = P(g1) = 0.1*0.9 = 0.09
        # So I_B(e3) = 1.0 - 0.09 = 0.91
        assert importances['e3'] == pytest.approx(0.91, abs=1e-6)

        # e3 should be the most important event
        most_critical = max(importances, key=importances.get)
        assert most_critical == 'e3'

    def test_importance_and_gate_symmetry(self, analyzer):
        # AND(e1=0.5, e2=0.5) -> I_B(e1) = I_B(e2) = 0.5
        nodes = {
            'top': _gate('top', 'Top', FaultTreeGateType.AND, ['e1', 'e2']),
            'e1': _event('e1', 'A', 0.5),
            'e2': _event('e2', 'B', 0.5),
        }
        tree = analyzer.build_tree('top', nodes)
        importances = analyzer.importance_analysis(tree)
        # I_B(e1) = P(top|e1=1) - P(top|e1=0) = 0.5 - 0.0 = 0.5
        assert importances['e1'] == pytest.approx(0.5, abs=1e-9)
        assert importances['e2'] == pytest.approx(0.5, abs=1e-9)


class TestVotingGate:
    """VOTING(k/n) gate: at least k of n children must fail."""

    def test_voting_2_of_3(self, analyzer):
        # 2-of-3 voting gate with identical probabilities for easy verification
        p = 0.1
        nodes = {
            'top': _gate('top', 'Redundant System', FaultTreeGateType.VOTING,
                         ['e1', 'e2', 'e3'], voting_k=2),
            'e1': _event('e1', 'Channel A', p),
            'e2': _event('e2', 'Channel B', p),
            'e3': _event('e3', 'Channel C', p),
        }
        tree = analyzer.build_tree('top', nodes)
        prob = analyzer.evaluate(tree)

        # P(at least 2 of 3) = C(3,2)*p^2*(1-p) + C(3,3)*p^3
        expected = 3 * p**2 * (1 - p) + p**3
        assert prob == pytest.approx(expected, abs=1e-9)

    def test_voting_2_of_3_unequal(self, analyzer):
        # Non-identical probabilities
        nodes = {
            'top': _gate('top', 'Voting', FaultTreeGateType.VOTING,
                         ['e1', 'e2', 'e3'], voting_k=2),
            'e1': _event('e1', 'A', 0.1),
            'e2': _event('e2', 'B', 0.2),
            'e3': _event('e3', 'C', 0.3),
        }
        tree = analyzer.build_tree('top', nodes)
        prob = analyzer.evaluate(tree)

        # Enumerate all subsets of size >= 2
        p1, p2, p3 = 0.1, 0.2, 0.3
        q1, q2, q3 = 1 - p1, 1 - p2, 1 - p3
        # Exactly 2 fail:
        expected = (p1 * p2 * q3 +
                    p1 * q2 * p3 +
                    q1 * p2 * p3 +
                    # All 3 fail:
                    p1 * p2 * p3)
        assert prob == pytest.approx(expected, abs=1e-9)


class TestCriticalPath:
    """Critical path: root to leaf through highest-probability children."""

    def test_critical_path_simple(self, analyzer):
        #     OR(top)
        #    /       \
        #  e1(0.9)   e2(0.1)
        nodes = {
            'top': _gate('top', 'Top', FaultTreeGateType.OR, ['e1', 'e2']),
            'e1': _event('e1', 'Dominant Failure', 0.9),
            'e2': _event('e2', 'Rare Failure', 0.1),
        }
        tree = analyzer.build_tree('top', nodes)
        path = analyzer.get_critical_path(tree)
        assert path == ['top', 'e1']

    def test_critical_path_nested(self, analyzer):
        #       OR(top)
        #      /       \
        #   AND(g1)     e3(0.01)
        #   /    \
        # e1(0.8) e2(0.9)
        nodes = {
            'top': _gate('top', 'Top', FaultTreeGateType.OR, ['g1', 'e3']),
            'g1': _gate('g1', 'Sub', FaultTreeGateType.AND, ['e1', 'e2']),
            'e1': _event('e1', 'A', 0.8),
            'e2': _event('e2', 'B', 0.9),
            'e3': _event('e3', 'C', 0.01),
        }
        tree = analyzer.build_tree('top', nodes)
        path = analyzer.get_critical_path(tree)
        # AND(0.8, 0.9)=0.72 > 0.01, so g1 is chosen, then e2 (0.9 > 0.8)
        assert path == ['top', 'g1', 'e2']


class TestBuildTreeValidation:
    """Build-tree validation catches structural errors."""

    def test_missing_root(self, analyzer):
        with pytest.raises(ValueError, match="Root node"):
            analyzer.build_tree('missing', {})

    def test_missing_child(self, analyzer):
        nodes = {
            'top': _gate('top', 'Top', FaultTreeGateType.AND, ['missing']),
        }
        with pytest.raises(ValueError, match="not found"):
            analyzer.build_tree('top', nodes)

    def test_gate_without_children(self, analyzer):
        nodes = {
            'top': FaultTreeNode(
                node_id='top', name='Top',
                node_type=FaultTreeNodeType.GATE,
                gate_type=FaultTreeGateType.AND,
                children=[],
            ),
        }
        with pytest.raises(ValueError, match="no children"):
            analyzer.build_tree('top', nodes)
