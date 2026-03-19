"""Tests for DependencyGraphManager."""

import sys
from unittest.mock import MagicMock

for mod in [
    'miracle_core.datatypes', 'miracle_core.constants',
    'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
    'rclpy.callback_groups',
    'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
    'miracle_msgs', 'miracle_msgs.msg',
    'std_msgs', 'std_msgs.msg',
]:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_resiliency.recovery_orchestrator import (
    DependencyGraphManager,
    DependencyAnalysis,
    ServiceNode,
)


# -- helpers ----------------------------------------------------------------

def _make_node(sid, name=None, version='1.0', deps=None, critical=False, order=0):
    return ServiceNode(
        service_id=sid,
        service_name=name or sid,
        version=version,
        dependencies=deps or [],
        is_critical=critical,
        startup_order=order,
    )


# -- tests ------------------------------------------------------------------


class TestServiceNodeDataclass:
    """Verify ServiceNode defaults and construction."""

    def test_defaults(self):
        node = ServiceNode(service_id='a', service_name='A', version='1.0')
        assert node.dependencies == []
        assert node.is_critical is False
        assert node.startup_order == 0

    def test_custom_values(self):
        node = ServiceNode(
            service_id='b', service_name='B', version='2.1',
            dependencies=['a'], is_critical=True, startup_order=5,
        )
        assert node.dependencies == ['a']
        assert node.is_critical is True
        assert node.startup_order == 5


class TestRegisterAndRemove:
    """Register / remove services and verify graph state."""

    def test_register_service(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('a'))
        assert 'a' in mgr._nodes
        assert 'a' in mgr._edges

    def test_remove_service(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('a'))
        mgr.register_service(_make_node('b', deps=['a']))
        mgr.remove_service('a')
        assert 'a' not in mgr._nodes
        # 'a' should also be removed from b's dependency list
        assert 'a' not in mgr._edges.get('b', [])

    def test_remove_nonexistent_service(self):
        mgr = DependencyGraphManager()
        # Should not raise
        mgr.remove_service('nonexistent')


class TestStartupOrder:
    """Topological startup ordering (dependencies first)."""

    def test_linear_chain(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('db'))
        mgr.register_service(_make_node('cache', deps=['db']))
        mgr.register_service(_make_node('api', deps=['cache']))
        order = mgr.get_startup_order()
        assert order.index('db') < order.index('cache')
        assert order.index('cache') < order.index('api')

    def test_empty_graph(self):
        mgr = DependencyGraphManager()
        assert mgr.get_startup_order() == []

    def test_independent_services(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('a'))
        mgr.register_service(_make_node('b'))
        mgr.register_service(_make_node('c'))
        order = mgr.get_startup_order()
        assert sorted(order) == ['a', 'b', 'c']


class TestShutdownOrder:
    """Shutdown order is the reverse of startup order."""

    def test_shutdown_reverses_startup(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('db'))
        mgr.register_service(_make_node('cache', deps=['db']))
        mgr.register_service(_make_node('api', deps=['cache']))
        startup = mgr.get_startup_order()
        shutdown = mgr.get_shutdown_order()
        assert shutdown == list(reversed(startup))


class TestImpactAnalysis:
    """Transitive impact analysis."""

    def test_direct_dependents(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('db'))
        mgr.register_service(_make_node('cache', deps=['db']))
        mgr.register_service(_make_node('api', deps=['cache']))
        impact = mgr.get_impact_analysis('db')
        assert 'cache' in impact
        assert 'api' in impact

    def test_no_impact(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('a'))
        mgr.register_service(_make_node('b'))
        assert mgr.get_impact_analysis('a') == []

    def test_diamond_dependency(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('db'))
        mgr.register_service(_make_node('auth', deps=['db']))
        mgr.register_service(_make_node('cache', deps=['db']))
        mgr.register_service(_make_node('api', deps=['auth', 'cache']))
        impact = mgr.get_impact_analysis('db')
        assert 'auth' in impact
        assert 'cache' in impact
        assert 'api' in impact


class TestCircularDependencies:
    """Cycle detection."""

    def test_no_cycles(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('a'))
        mgr.register_service(_make_node('b', deps=['a']))
        assert mgr.detect_circular_dependencies() == []

    def test_simple_cycle(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('a', deps=['b']))
        mgr.register_service(_make_node('b', deps=['a']))
        cycles = mgr.detect_circular_dependencies()
        assert len(cycles) >= 1
        # The cycle should contain both a and b
        flat = [sid for cycle in cycles for sid in cycle]
        assert 'a' in flat
        assert 'b' in flat

    def test_three_node_cycle(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('a', deps=['c']))
        mgr.register_service(_make_node('b', deps=['a']))
        mgr.register_service(_make_node('c', deps=['b']))
        cycles = mgr.detect_circular_dependencies()
        assert len(cycles) >= 1


class TestCriticalPath:
    """Longest dependency chain."""

    def test_linear_chain_is_critical(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('db'))
        mgr.register_service(_make_node('cache', deps=['db']))
        mgr.register_service(_make_node('api', deps=['cache']))
        mgr.register_service(_make_node('monitor'))
        critical = mgr.get_critical_path()
        assert len(critical) == 3
        assert critical == ['db', 'cache', 'api']

    def test_empty_graph(self):
        mgr = DependencyGraphManager()
        assert mgr.get_critical_path() == []


class TestOrphanServices:
    """Services with no deps and no dependents."""

    def test_orphan_detected(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('db'))
        mgr.register_service(_make_node('cache', deps=['db']))
        mgr.register_service(_make_node('orphan'))
        orphans = mgr.get_orphan_services()
        assert 'orphan' in orphans
        assert 'db' not in orphans  # db is depended upon
        assert 'cache' not in orphans  # cache has deps

    def test_no_orphans(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('a'))
        mgr.register_service(_make_node('b', deps=['a']))
        orphans = mgr.get_orphan_services()
        assert orphans == []


class TestFullAnalysis:
    """get_full_analysis returns a DependencyAnalysis."""

    def test_full_analysis_structure(self):
        mgr = DependencyGraphManager()
        mgr.register_service(_make_node('db'))
        mgr.register_service(_make_node('cache', deps=['db']))
        mgr.register_service(_make_node('api', deps=['cache']))
        mgr.register_service(_make_node('logger'))

        analysis = mgr.get_full_analysis()
        assert isinstance(analysis, DependencyAnalysis)
        assert analysis.startup_order[0] == 'db'
        assert analysis.circular_dependencies == []
        assert 'logger' in analysis.orphan_services
        assert len(analysis.critical_path) == 3
        assert analysis.max_depth == 3
