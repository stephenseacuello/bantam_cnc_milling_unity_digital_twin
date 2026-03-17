"""Tests for NetworkTopologyMapper, NetworkNode, NetworkLink, TopologySnapshot.

Covers node/link management, status updates, snapshot generation,
shortest-path routing, articulation-point detection, node removal,
dependency queries, and unhealthy link detection.
"""

import sys
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# Mock all ROS2 / miracle_core / miracle_msgs modules so the import of
# recovery_orchestrator succeeds without a real ROS2 installation.
# ---------------------------------------------------------------------------
for mod in ['miracle_core.datatypes', 'miracle_core.constants',
            'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
            'rclpy.callback_groups',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_msgs', 'miracle_msgs.msg',
            'std_msgs', 'std_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

# Ensure lifecycle_client stub is available
_lc = sys.modules.setdefault(
    'miracle_resiliency.lifecycle_client', MagicMock(),
)

import pytest  # noqa: E402

from miracle_resiliency.recovery_orchestrator import (  # noqa: E402
    NetworkNode,
    NetworkLink,
    TopologySnapshot,
    NetworkTopologyMapper,
)


# ===========================================================================
# Helpers
# ===========================================================================

def _make_node(node_id, node_type='ros2_node', ip='10.0.0.1',
               hostname='host', is_online=True):
    return NetworkNode(
        node_id=node_id,
        node_type=node_type,
        ip_address=ip,
        hostname=hostname,
        is_online=is_online,
    )


def _make_link(src, tgt, protocol='DDS', healthy=True,
               packet_loss=0.0):
    return NetworkLink(
        source_id=src,
        target_id=tgt,
        protocol=protocol,
        is_healthy=healthy,
        packet_loss_pct=packet_loss,
    )


def _build_chain_topology():
    """Build a chain: A -- B -- C -- D

    B and C are articulation points.
    """
    mapper = NetworkTopologyMapper()
    for nid in ['A', 'B', 'C', 'D']:
        mapper.add_node(_make_node(nid))
    mapper.add_link(_make_link('A', 'B'))
    mapper.add_link(_make_link('B', 'C'))
    mapper.add_link(_make_link('C', 'D'))
    return mapper


@pytest.fixture
def mapper():
    return NetworkTopologyMapper()


@pytest.fixture
def chain_mapper():
    return _build_chain_topology()


# ===========================================================================
# Test: Add nodes and links
# ===========================================================================

class TestAddNodesAndLinks:
    def test_add_single_node(self, mapper):
        n = _make_node('sensor_1', 'sensor')
        mapper.add_node(n)
        snap = mapper.get_snapshot()
        assert snap.total_nodes == 1
        assert 'sensor_1' in snap.nodes

    def test_add_multiple_nodes(self, mapper):
        for i in range(5):
            mapper.add_node(_make_node(f'n{i}'))
        assert mapper.get_snapshot().total_nodes == 5

    def test_add_link_updates_connections(self, mapper):
        mapper.add_node(_make_node('a'))
        mapper.add_node(_make_node('b'))
        mapper.add_link(_make_link('a', 'b'))
        assert 'b' in mapper.get_node_dependencies('a')
        assert 'a' in mapper.get_node_dependencies('b')

    def test_add_link_appears_in_snapshot(self, mapper):
        mapper.add_node(_make_node('x'))
        mapper.add_node(_make_node('y'))
        mapper.add_link(_make_link('x', 'y', protocol='OPC-UA'))
        snap = mapper.get_snapshot()
        assert len(snap.links) == 1
        assert snap.links[0].protocol == 'OPC-UA'


# ===========================================================================
# Test: Update node online status
# ===========================================================================

class TestUpdateNodeStatus:
    def test_mark_offline(self, mapper):
        mapper.add_node(_make_node('plc_1', 'plc'))
        mapper.update_node_status('plc_1', is_online=False, latency_ms=999.0)
        snap = mapper.get_snapshot()
        assert snap.nodes['plc_1'].is_online is False
        assert snap.nodes['plc_1'].latency_ms == 999.0

    def test_mark_online_updates_last_seen(self, mapper):
        mapper.add_node(_make_node('hmi_1', 'hmi'))
        mapper.update_node_status('hmi_1', is_online=True, latency_ms=2.5)
        node = mapper._nodes['hmi_1']
        assert node.is_online is True
        assert node.last_seen > 0.0

    def test_update_nonexistent_node_no_error(self, mapper):
        # Should silently do nothing
        mapper.update_node_status('ghost', is_online=False, latency_ms=0.0)


# ===========================================================================
# Test: Snapshot generation with counts
# ===========================================================================

class TestSnapshotGeneration:
    def test_empty_topology(self, mapper):
        snap = mapper.get_snapshot()
        assert snap.total_nodes == 0
        assert snap.online_nodes == 0
        assert snap.unhealthy_links == 0

    def test_online_count(self, mapper):
        mapper.add_node(_make_node('a', is_online=True))
        mapper.add_node(_make_node('b', is_online=True))
        mapper.add_node(_make_node('c', is_online=False))
        snap = mapper.get_snapshot()
        assert snap.total_nodes == 3
        assert snap.online_nodes == 2

    def test_unhealthy_link_count(self, mapper):
        mapper.add_node(_make_node('a'))
        mapper.add_node(_make_node('b'))
        mapper.add_node(_make_node('c'))
        mapper.add_link(_make_link('a', 'b', healthy=True))
        mapper.add_link(_make_link('b', 'c', healthy=False))
        snap = mapper.get_snapshot()
        assert snap.unhealthy_links == 1

    def test_snapshot_timestamp_is_positive(self, mapper):
        mapper.add_node(_make_node('n'))
        snap = mapper.get_snapshot()
        assert snap.timestamp > 0


# ===========================================================================
# Test: Find path between nodes
# ===========================================================================

class TestGetPath:
    def test_direct_neighbours(self, mapper):
        mapper.add_node(_make_node('a'))
        mapper.add_node(_make_node('b'))
        mapper.add_link(_make_link('a', 'b'))
        path = mapper.get_path('a', 'b')
        assert path == ['a', 'b']

    def test_multi_hop_path(self, chain_mapper):
        path = chain_mapper.get_path('A', 'D')
        assert path == ['A', 'B', 'C', 'D']

    def test_path_to_self(self, chain_mapper):
        assert chain_mapper.get_path('B', 'B') == ['B']

    def test_no_path_returns_empty(self, mapper):
        mapper.add_node(_make_node('x'))
        mapper.add_node(_make_node('y'))
        # No link between them
        assert mapper.get_path('x', 'y') == []

    def test_nonexistent_source(self, mapper):
        mapper.add_node(_make_node('a'))
        assert mapper.get_path('ghost', 'a') == []


# ===========================================================================
# Test: Critical node detection (articulation points)
# ===========================================================================

class TestFindCriticalNodes:
    def test_chain_articulation_points(self, chain_mapper):
        """In A--B--C--D, B and C are articulation points."""
        critical = chain_mapper.find_critical_nodes()
        assert 'B' in critical
        assert 'C' in critical
        # A and D are leaf nodes, not articulation points
        assert 'A' not in critical
        assert 'D' not in critical

    def test_fully_connected_no_critical(self, mapper):
        """A triangle has no articulation points."""
        for nid in ['a', 'b', 'c']:
            mapper.add_node(_make_node(nid))
        mapper.add_link(_make_link('a', 'b'))
        mapper.add_link(_make_link('b', 'c'))
        mapper.add_link(_make_link('a', 'c'))
        assert mapper.find_critical_nodes() == []

    def test_star_topology_center_is_critical(self, mapper):
        """Hub-and-spoke: center is the only articulation point."""
        mapper.add_node(_make_node('hub'))
        for i in range(4):
            spoke = f'spoke_{i}'
            mapper.add_node(_make_node(spoke))
            mapper.add_link(_make_link('hub', spoke))
        critical = mapper.find_critical_nodes()
        assert critical == ['hub']

    def test_empty_topology_no_critical(self, mapper):
        assert mapper.find_critical_nodes() == []


# ===========================================================================
# Test: Node removal
# ===========================================================================

class TestRemoveNode:
    def test_remove_decreases_count(self, chain_mapper):
        chain_mapper.remove_node('B')
        snap = chain_mapper.get_snapshot()
        assert snap.total_nodes == 3
        assert 'B' not in snap.nodes

    def test_remove_cleans_links(self, chain_mapper):
        chain_mapper.remove_node('B')
        snap = chain_mapper.get_snapshot()
        for lk in snap.links:
            assert lk.source_id != 'B'
            assert lk.target_id != 'B'

    def test_remove_cleans_connections(self, chain_mapper):
        chain_mapper.remove_node('B')
        assert 'B' not in chain_mapper.get_node_dependencies('A')
        assert 'B' not in chain_mapper.get_node_dependencies('C')

    def test_remove_nonexistent_no_error(self, mapper):
        mapper.remove_node('nope')  # should not raise


# ===========================================================================
# Test: Node dependencies
# ===========================================================================

class TestNodeDependencies:
    def test_direct_connections(self, chain_mapper):
        deps = chain_mapper.get_node_dependencies('B')
        assert sorted(deps) == ['A', 'C']

    def test_leaf_node_single_dependency(self, chain_mapper):
        deps = chain_mapper.get_node_dependencies('A')
        assert deps == ['B']

    def test_nonexistent_node_returns_empty(self, mapper):
        assert mapper.get_node_dependencies('ghost') == []


# ===========================================================================
# Test: Unhealthy link detection
# ===========================================================================

class TestUnhealthyLinks:
    def test_all_healthy(self, mapper):
        mapper.add_node(_make_node('a'))
        mapper.add_node(_make_node('b'))
        mapper.add_link(_make_link('a', 'b', healthy=True))
        assert mapper.get_snapshot().unhealthy_links == 0

    def test_single_unhealthy_link(self, mapper):
        mapper.add_node(_make_node('a'))
        mapper.add_node(_make_node('b'))
        mapper.add_link(_make_link('a', 'b', healthy=False))
        assert mapper.get_snapshot().unhealthy_links == 1

    def test_multiple_unhealthy_links(self, mapper):
        for nid in ['a', 'b', 'c', 'd']:
            mapper.add_node(_make_node(nid))
        mapper.add_link(_make_link('a', 'b', healthy=False))
        mapper.add_link(_make_link('b', 'c', healthy=True))
        mapper.add_link(_make_link('c', 'd', healthy=False))
        snap = mapper.get_snapshot()
        assert snap.unhealthy_links == 2
        assert len(snap.links) == 3
