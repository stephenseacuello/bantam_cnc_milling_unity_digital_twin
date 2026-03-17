"""Tests for ManufacturingKnowledgeGraph entity-relationship graph.

Tests node/edge operations, path finding, transitive inference,
query by type, recommendations, and graph statistics.
"""

import sys
from unittest.mock import MagicMock

for mod in [
    'miracle_core.datatypes', 'miracle_core.constants',
    'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
    'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
    'miracle_msgs', 'miracle_msgs.msg',
    'std_msgs', 'std_msgs.msg',
]:
    sys.modules.setdefault(mod, MagicMock())

import pytest
from miracle_cognitive.knowledge.reasoning_engine import (
    ManufacturingKnowledgeGraph,
    KnowledgeNode,
    KnowledgeEdge,
    EntityType,
    RelationshipType,
)


@pytest.fixture
def graph():
    return ManufacturingKnowledgeGraph()


@pytest.fixture
def populated_graph(graph):
    """Graph with a small manufacturing scenario."""
    nodes = [
        KnowledgeNode('machine_1', EntityType.MACHINE, 'Haas VF-2'),
        KnowledgeNode('tool_1', EntityType.TOOL, 'HSS End Mill 10mm'),
        KnowledgeNode('mat_1', EntityType.MATERIAL, '6061-T6'),
        KnowledgeNode('proc_1', EntityType.PROCESS, 'Slot Milling'),
        KnowledgeNode('param_1', EntityType.PARAMETER, 'Feed 200mm/min',
                       properties={'feed': 200, 'material': '6061-T6'}),
        KnowledgeNode('outcome_1', EntityType.OUTCOME, 'Good Surface Finish'),
        KnowledgeNode('fail_1', EntityType.FAILURE, 'Chatter'),
    ]
    for n in nodes:
        graph.add_node(n)

    edges = [
        KnowledgeEdge('machine_1', 'tool_1', RelationshipType.USES, 1.0),
        KnowledgeEdge('proc_1', 'tool_1', RelationshipType.REQUIRES, 0.9),
        KnowledgeEdge('proc_1', 'mat_1', RelationshipType.AFFECTS, 0.8),
        KnowledgeEdge('param_1', 'mat_1', RelationshipType.OPTIMAL_FOR, 0.95),
        KnowledgeEdge('param_1', 'outcome_1', RelationshipType.PRODUCES, 0.85),
        KnowledgeEdge('fail_1', 'outcome_1', RelationshipType.AFFECTS, 0.7,
                       ['observed in job J-100']),
    ]
    for e in edges:
        graph.add_edge(e)
    return graph


# ---- Node & edge basics ----

def test_add_nodes_and_edges(graph):
    n1 = KnowledgeNode('a', EntityType.MACHINE, 'Machine A')
    n2 = KnowledgeNode('b', EntityType.TOOL, 'Tool B')
    assert graph.add_node(n1) is True
    assert graph.add_node(n2) is True
    assert graph.add_edge(KnowledgeEdge('a', 'b', RelationshipType.USES)) is True
    assert graph.node_count == 2
    assert graph.edge_count == 1


def test_duplicate_node_rejected(graph):
    n = KnowledgeNode('x', EntityType.MACHINE, 'X')
    assert graph.add_node(n) is True
    assert graph.add_node(n) is False  # duplicate
    assert graph.node_count == 1


def test_edge_requires_existing_nodes(graph):
    graph.add_node(KnowledgeNode('a', EntityType.MACHINE, 'A'))
    edge = KnowledgeEdge('a', 'missing', RelationshipType.USES)
    assert graph.add_edge(edge) is False


# ---- Path finding ----

def test_find_path(populated_graph):
    path = populated_graph.find_path('machine_1', 'outcome_1')
    # machine_1 -> tool_1 is a dead end for outcome, but
    # there should be no path since machine_1 doesn't connect to outcome_1 directly
    # Let's add a connecting edge
    populated_graph.add_edge(
        KnowledgeEdge('tool_1', 'proc_1', RelationshipType.AFFECTS, 0.8))
    populated_graph.add_edge(
        KnowledgeEdge('proc_1', 'outcome_1', RelationshipType.PRODUCES, 0.9))
    path = populated_graph.find_path('machine_1', 'outcome_1')
    assert path is not None
    assert path[0] == 'machine_1'
    assert path[-1] == 'outcome_1'
    assert len(path) >= 3


def test_find_path_no_connection(populated_graph):
    # outcome_1 has no outgoing edges to machine_1
    path = populated_graph.find_path('outcome_1', 'machine_1')
    assert path is None


def test_find_path_same_node(populated_graph):
    path = populated_graph.find_path('machine_1', 'machine_1')
    assert path == ['machine_1']


# ---- Neighbours ----

def test_get_neighbors_outgoing(populated_graph):
    neighbors = populated_graph.get_neighbors('machine_1')
    assert len(neighbors) == 1
    assert neighbors[0].node_id == 'tool_1'


def test_get_neighbors_by_relationship(populated_graph):
    neighbors = populated_graph.get_neighbors(
        'proc_1', relationship=RelationshipType.REQUIRES)
    assert len(neighbors) == 1
    assert neighbors[0].node_id == 'tool_1'


def test_get_neighbors_incoming(populated_graph):
    neighbors = populated_graph.get_neighbors(
        'tool_1', direction='incoming')
    ids = {n.node_id for n in neighbors}
    assert 'machine_1' in ids
    assert 'proc_1' in ids


# ---- Transitive inference ----

def test_transitive_inference(graph):
    """A CAUSES B and B CAUSES C => infer A CAUSES C."""
    graph.add_node(KnowledgeNode('a', EntityType.FAILURE, 'Vibration'))
    graph.add_node(KnowledgeNode('b', EntityType.FAILURE, 'Surface Marks'))
    graph.add_node(KnowledgeNode('c', EntityType.OUTCOME, 'Scrap Part'))
    graph.add_edge(KnowledgeEdge('a', 'b', RelationshipType.CAUSES, 0.9))
    graph.add_edge(KnowledgeEdge('b', 'c', RelationshipType.CAUSES, 0.8))

    new_edges = graph.infer_relationships(RelationshipType.CAUSES)
    assert len(new_edges) == 1
    inferred = new_edges[0]
    assert inferred.source_id == 'a'
    assert inferred.target_id == 'c'
    assert abs(inferred.weight - 0.72) < 0.01  # 0.9 * 0.8
    assert 'inferred' in inferred.evidence[0]


def test_transitive_inference_long_chain(graph):
    """A->B->C->D, should infer A->C, A->D, B->D."""
    for nid in ['a', 'b', 'c', 'd']:
        graph.add_node(KnowledgeNode(nid, EntityType.FAILURE, nid.upper()))
    graph.add_edge(KnowledgeEdge('a', 'b', RelationshipType.CAUSES, 0.9))
    graph.add_edge(KnowledgeEdge('b', 'c', RelationshipType.CAUSES, 0.8))
    graph.add_edge(KnowledgeEdge('c', 'd', RelationshipType.CAUSES, 0.7))

    new_edges = graph.infer_relationships(RelationshipType.CAUSES)
    inferred_pairs = {(e.source_id, e.target_id) for e in new_edges}
    assert ('a', 'c') in inferred_pairs
    assert ('a', 'd') in inferred_pairs
    assert ('b', 'd') in inferred_pairs


# ---- Query by type ----

def test_query_by_type(populated_graph):
    tools = populated_graph.query_by_type(EntityType.TOOL)
    assert len(tools) == 1
    assert tools[0].name == 'HSS End Mill 10mm'

    failures = populated_graph.query_by_type(EntityType.FAILURE)
    assert len(failures) == 1
    assert failures[0].name == 'Chatter'


# ---- Recommendations ----

def test_query_recommendations(populated_graph):
    """param_1 is OPTIMAL_FOR mat_1, so querying with node_id context should find it."""
    recs = populated_graph.query_recommendations(
        EntityType.PARAMETER, {'material': 'mat_1'})
    assert len(recs) >= 1
    node, score = recs[0]
    assert node.node_id == 'param_1'
    assert score == pytest.approx(0.95)


def test_query_recommendations_by_property(populated_graph):
    """Context matches node properties too (mat_1 has no 'material' prop but name matches)."""
    # Add a material node with a matching property
    populated_graph.add_node(
        KnowledgeNode('mat_2', EntityType.MATERIAL, '304-SS',
                       properties={'grade': 'austenitic'}))
    param2 = KnowledgeNode('param_2', EntityType.PARAMETER, 'Feed 100mm/min')
    populated_graph.add_node(param2)
    populated_graph.add_edge(
        KnowledgeEdge('param_2', 'mat_2', RelationshipType.OPTIMAL_FOR, 0.88))
    recs = populated_graph.query_recommendations(
        EntityType.PARAMETER, {'grade': 'austenitic'})
    assert len(recs) >= 1
    assert recs[0][0].node_id == 'param_2'


# ---- Statistics ----

def test_graph_statistics(populated_graph):
    assert populated_graph.node_count == 7
    assert populated_graph.edge_count == 6
    assert populated_graph.avg_degree == pytest.approx(6 / 7, abs=0.01)
    assert populated_graph.connected_components() >= 1


def test_empty_graph_statistics(graph):
    assert graph.node_count == 0
    assert graph.edge_count == 0
    assert graph.avg_degree == 0.0
    assert graph.connected_components() == 0
