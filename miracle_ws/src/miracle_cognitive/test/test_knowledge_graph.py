"""Tests for RDF triple store operations in the knowledge graph.

Tests adding triples, querying by subject/predicate/object patterns,
wildcard queries, confidence tracking, and removal.
"""

import pytest


class _KnowledgeGraph:
    """Extracted triple store logic from KnowledgeGraphNode.

    Provides add, query, and remove operations on (subject, predicate,
    object, confidence) tuples without any ROS2 dependencies.
    """

    def __init__(self, max_triples=100000):
        self.triples = []  # List of (s, p, o, confidence)
        self.max_triples = max_triples

    def add_triple(self, subject, predicate, obj, confidence=1.0):
        """Add a triple to the knowledge graph."""
        if len(self.triples) >= self.max_triples:
            self.triples.pop(0)
        self.triples.append((subject, predicate, obj, confidence))

    def query(self, subject='', predicate='', obj=''):
        """Query triples matching pattern (empty string = wildcard)."""
        results = []
        for s, p, o, c in self.triples:
            if subject and s != subject:
                continue
            if predicate and p != predicate:
                continue
            if obj and o != obj:
                continue
            results.append((s, p, o, c))
        return results

    def remove_triple(self, subject, predicate, obj):
        """Remove all matching triples."""
        self.triples = [
            (s, p, o, c) for s, p, o, c in self.triples
            if not (s == subject and p == predicate and o == obj)
        ]


@pytest.fixture
def kg():
    """Return a knowledge graph with some base manufacturing triples."""
    g = _KnowledgeGraph()
    g.add_triple('CNCMachine', 'isA', 'ManufacturingEquipment', 1.0)
    g.add_triple('ManufacturingEquipment', 'isA', 'PhysicalAsset', 1.0)
    g.add_triple('Spindle', 'isPartOf', 'CNCMachine', 1.0)
    g.add_triple('CuttingTool', 'isUsedBy', 'Spindle', 1.0)
    g.add_triple('ToolWear', 'affects', 'SurfaceQuality', 0.9)
    g.add_triple('Chatter', 'affects', 'SurfaceQuality', 0.85)
    return g


class TestAddTriple:
    """Tests for adding triples to the knowledge graph."""

    def test_add_increases_count(self, kg):
        """Adding a triple should increase the total count."""
        initial = len(kg.triples)
        kg.add_triple('NewEntity', 'hasProperty', 'Value', 1.0)
        assert len(kg.triples) == initial + 1

    def test_add_preserves_data(self, kg):
        """Added triple should be retrievable."""
        kg.add_triple('Drill', 'isA', 'CuttingTool', 0.95)
        results = kg.query(subject='Drill')
        assert len(results) == 1
        assert results[0] == ('Drill', 'isA', 'CuttingTool', 0.95)

    def test_max_triples_eviction(self):
        """When max_triples is reached, the oldest triple should be evicted."""
        g = _KnowledgeGraph(max_triples=3)
        g.add_triple('A', 'r', 'B')
        g.add_triple('C', 'r', 'D')
        g.add_triple('E', 'r', 'F')
        g.add_triple('G', 'r', 'H')  # Should evict (A, r, B)
        assert len(g.triples) == 3
        subjects = [t[0] for t in g.triples]
        assert 'A' not in subjects, "Oldest triple should be evicted"
        assert 'G' in subjects, "Newest triple should be present"

    def test_confidence_stored(self, kg):
        """Confidence value should be stored with the triple."""
        kg.add_triple('Sensor', 'reads', 'Temperature', 0.75)
        results = kg.query(subject='Sensor')
        assert results[0][3] == 0.75

    def test_duplicate_triples_allowed(self, kg):
        """Adding the same triple twice should result in two entries."""
        kg.add_triple('X', 'r', 'Y')
        kg.add_triple('X', 'r', 'Y')
        results = kg.query(subject='X', predicate='r', obj='Y')
        assert len(results) == 2


class TestQueryTriples:
    """Tests for querying the triple store."""

    def test_query_by_subject(self, kg):
        """Querying by subject should return all triples with that subject."""
        results = kg.query(subject='CNCMachine')
        assert len(results) == 1
        assert results[0][2] == 'ManufacturingEquipment'

    def test_query_by_predicate(self, kg):
        """Querying by predicate should return all matching triples."""
        results = kg.query(predicate='affects')
        assert len(results) == 2  # ToolWear and Chatter

    def test_query_by_object(self, kg):
        """Querying by object should return all triples with that object."""
        results = kg.query(obj='SurfaceQuality')
        assert len(results) == 2

    def test_wildcard_query_returns_all(self, kg):
        """An empty query (all wildcards) should return all triples."""
        results = kg.query()
        assert len(results) == len(kg.triples)

    def test_combined_pattern_match(self, kg):
        """Querying by subject AND predicate should narrow results."""
        results = kg.query(subject='ToolWear', predicate='affects')
        assert len(results) == 1
        assert results[0][2] == 'SurfaceQuality'

    def test_no_match_returns_empty(self, kg):
        """A query that matches nothing should return an empty list."""
        results = kg.query(subject='NonExistent')
        assert results == []

    def test_full_pattern_match(self, kg):
        """Specifying all three fields should match exactly."""
        results = kg.query('Spindle', 'isPartOf', 'CNCMachine')
        assert len(results) == 1


class TestRemoveTriple:
    """Tests for removing triples from the knowledge graph."""

    def test_remove_existing_triple(self, kg):
        """Removing an existing triple should decrease the count."""
        initial = len(kg.triples)
        kg.remove_triple('Spindle', 'isPartOf', 'CNCMachine')
        assert len(kg.triples) == initial - 1

    def test_removed_triple_not_queryable(self, kg):
        """A removed triple should no longer appear in query results."""
        kg.remove_triple('CuttingTool', 'isUsedBy', 'Spindle')
        results = kg.query(subject='CuttingTool')
        assert len(results) == 0

    def test_remove_nonexistent_triple(self, kg):
        """Removing a non-existent triple should be a no-op."""
        initial = len(kg.triples)
        kg.remove_triple('Ghost', 'haunts', 'Machine')
        assert len(kg.triples) == initial

    def test_remove_preserves_others(self, kg):
        """Removing one triple should not affect other triples."""
        kg.remove_triple('ToolWear', 'affects', 'SurfaceQuality')
        results = kg.query(predicate='affects')
        assert len(results) == 1  # Chatter still present
        assert results[0][0] == 'Chatter'
