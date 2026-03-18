"""Tests for ServiceMeshMonitor.

Covers service registration and removal, dependency management,
health check recording with status derivation, health report generation,
critical path identification, availability calculation, cascading failure
detection, and edge cases.
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
    ServiceMeshMonitor,
    ServiceEndpoint,
    ServiceDependency,
    MeshHealthReport,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_endpoint(
    service_id: str,
    name: str = '',
    protocol: str = 'DDS',
) -> ServiceEndpoint:
    """Create a ServiceEndpoint with sensible defaults."""
    return ServiceEndpoint(
        service_id=service_id,
        service_name=name or service_id,
        endpoint_url=f'http://localhost/{service_id}',
        protocol=protocol,
    )


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def monitor():
    """Return a fresh ServiceMeshMonitor instance."""
    return ServiceMeshMonitor()


@pytest.fixture
def populated_monitor(monitor):
    """Return a monitor with three services and two dependencies."""
    svc_a = _make_endpoint('svc_a', 'Service A')
    svc_b = _make_endpoint('svc_b', 'Service B')
    svc_c = _make_endpoint('svc_c', 'Service C')
    monitor.register_service(svc_a)
    monitor.register_service(svc_b)
    monitor.register_service(svc_c)
    # A depends on B (critical), B depends on C (non-critical)
    monitor.add_dependency(ServiceDependency(
        source_service='svc_a', target_service='svc_b', is_critical=True,
    ))
    monitor.add_dependency(ServiceDependency(
        source_service='svc_b', target_service='svc_c', is_critical=False,
    ))
    return monitor


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestServiceEndpointDataclass:
    """Verify ServiceEndpoint dataclass construction and defaults."""

    def test_required_fields(self):
        ep = ServiceEndpoint(
            service_id='id_1',
            service_name='Sensor Fusion',
            endpoint_url='http://localhost:8080',
            protocol='DDS',
        )
        assert ep.service_id == 'id_1'
        assert ep.service_name == 'Sensor Fusion'
        assert ep.endpoint_url == 'http://localhost:8080'
        assert ep.protocol == 'DDS'

    def test_default_values(self):
        ep = _make_endpoint('test')
        assert ep.health_status == 'unknown'
        assert ep.last_check == 0.0
        assert ep.response_time_ms == 0.0
        assert ep.error_count == 0
        assert ep.success_count == 0


class TestRegisterAndRemoveService:
    """Verify service registration and removal."""

    def test_register_service(self, monitor):
        ep = _make_endpoint('svc_x')
        monitor.register_service(ep)
        assert 'svc_x' in monitor._services

    def test_remove_existing_service(self, monitor):
        ep = _make_endpoint('svc_y')
        monitor.register_service(ep)
        assert monitor.remove_service('svc_y') is True
        assert 'svc_y' not in monitor._services

    def test_remove_nonexistent_service(self, monitor):
        assert monitor.remove_service('ghost') is False

    def test_remove_cleans_dependencies(self, populated_monitor):
        populated_monitor.remove_service('svc_b')
        # Dependencies involving svc_b should be removed
        for dep in populated_monitor._dependencies:
            assert dep.source_service != 'svc_b'
            assert dep.target_service != 'svc_b'


class TestAddDependency:
    """Verify dependency registration."""

    def test_add_dependency(self, monitor):
        dep = ServiceDependency(
            source_service='a', target_service='b',
            is_critical=True, timeout_ms=3000.0, retry_count=5,
        )
        monitor.add_dependency(dep)
        assert len(monitor._dependencies) == 1
        assert monitor._dependencies[0].is_critical is True
        assert monitor._dependencies[0].timeout_ms == 3000.0
        assert monitor._dependencies[0].retry_count == 5


class TestRecordHealthCheck:
    """Verify health check recording and status derivation."""

    def test_healthy_check(self, monitor):
        ep = _make_endpoint('svc_h')
        monitor.register_service(ep)
        monitor.record_health_check('svc_h', True, 10.0)
        assert monitor._services['svc_h'].health_status == 'healthy'
        assert monitor._services['svc_h'].success_count == 1
        assert monitor._services['svc_h'].error_count == 0

    def test_degraded_on_slow_response(self, monitor):
        ep = _make_endpoint('svc_slow')
        monitor.register_service(ep)
        monitor.record_health_check('svc_slow', True, 600.0)
        assert monitor._services['svc_slow'].health_status == 'degraded'

    def test_unhealthy_on_high_error_ratio(self, monitor):
        ep = _make_endpoint('svc_err')
        monitor.register_service(ep)
        # Record enough failures to push error ratio >= 0.5
        monitor.record_health_check('svc_err', False, 10.0)
        monitor.record_health_check('svc_err', False, 10.0)
        assert monitor._services['svc_err'].health_status == 'unhealthy'

    def test_ignores_unknown_service(self, monitor):
        # Should not raise
        monitor.record_health_check('nonexistent', True, 5.0)

    def test_degraded_on_some_errors(self, monitor):
        ep = _make_endpoint('svc_mixed')
        monitor.register_service(ep)
        # 4 successes, 1 failure -> error_ratio = 0.2 < 0.5, but > 0
        for _ in range(4):
            monitor.record_health_check('svc_mixed', True, 10.0)
        monitor.record_health_check('svc_mixed', False, 10.0)
        assert monitor._services['svc_mixed'].health_status == 'degraded'


class TestGetHealthReport:
    """Verify MeshHealthReport generation."""

    def test_empty_mesh_report(self, monitor):
        report = monitor.get_health_report()
        assert report.total_services == 0
        assert report.overall_health_pct == 0.0
        assert report.critical_path_healthy is True

    def test_all_healthy_report(self, populated_monitor):
        for sid in ['svc_a', 'svc_b', 'svc_c']:
            populated_monitor.record_health_check(sid, True, 5.0)
        report = populated_monitor.get_health_report()
        assert report.total_services == 3
        assert report.healthy_count == 3
        assert report.degraded_count == 0
        assert report.unhealthy_count == 0
        assert report.overall_health_pct == pytest.approx(100.0, abs=0.01)
        assert report.critical_path_healthy is True
        assert report.avg_response_ms == pytest.approx(5.0, abs=0.01)

    def test_mixed_health_report(self, populated_monitor):
        populated_monitor.record_health_check('svc_a', True, 5.0)
        populated_monitor.record_health_check('svc_b', True, 600.0)  # degraded
        populated_monitor.record_health_check('svc_c', False, 5.0)
        populated_monitor.record_health_check('svc_c', False, 5.0)   # unhealthy
        report = populated_monitor.get_health_report()
        assert report.healthy_count == 1
        assert report.degraded_count == 1
        assert report.unhealthy_count == 1
        assert report.overall_health_pct == pytest.approx(100.0 / 3.0, abs=0.1)

    def test_slowest_service_identified(self, populated_monitor):
        populated_monitor.record_health_check('svc_a', True, 10.0)
        populated_monitor.record_health_check('svc_b', True, 200.0)
        populated_monitor.record_health_check('svc_c', True, 50.0)
        report = populated_monitor.get_health_report()
        assert report.slowest_service == 'svc_b'

    def test_critical_path_unhealthy(self, populated_monitor):
        # svc_b is on critical path; make it unhealthy
        populated_monitor.record_health_check('svc_a', True, 5.0)
        populated_monitor.record_health_check('svc_b', False, 5.0)
        populated_monitor.record_health_check('svc_b', False, 5.0)
        populated_monitor.record_health_check('svc_c', True, 5.0)
        report = populated_monitor.get_health_report()
        assert report.critical_path_healthy is False


class TestGetCriticalPathServices:
    """Verify critical path identification."""

    def test_critical_dependencies(self, populated_monitor):
        critical = populated_monitor.get_critical_path_services()
        # svc_a -> svc_b is critical, so both should appear
        assert 'svc_a' in critical
        assert 'svc_b' in critical
        # svc_c is not on the critical path
        assert 'svc_c' not in critical

    def test_no_critical_dependencies(self, monitor):
        monitor.register_service(_make_endpoint('x'))
        monitor.register_service(_make_endpoint('y'))
        monitor.add_dependency(ServiceDependency(
            source_service='x', target_service='y', is_critical=False,
        ))
        assert monitor.get_critical_path_services() == []


class TestGetServiceAvailability:
    """Verify per-service availability calculation."""

    def test_full_availability(self, monitor):
        monitor.register_service(_make_endpoint('svc_avail'))
        for _ in range(10):
            monitor.record_health_check('svc_avail', True, 5.0)
        assert monitor.get_service_availability('svc_avail') == 100.0

    def test_partial_availability(self, monitor):
        monitor.register_service(_make_endpoint('svc_part'))
        for _ in range(3):
            monitor.record_health_check('svc_part', True, 5.0)
        monitor.record_health_check('svc_part', False, 5.0)
        # 3 / 4 = 75%
        assert monitor.get_service_availability('svc_part') == 75.0

    def test_unknown_service_availability(self, monitor):
        assert monitor.get_service_availability('ghost') == 0.0

    def test_no_checks_availability(self, monitor):
        monitor.register_service(_make_endpoint('fresh'))
        assert monitor.get_service_availability('fresh') == 0.0


class TestDetectCascadingFailure:
    """Verify cascading failure detection."""

    def test_no_unhealthy_services(self, populated_monitor):
        for sid in ['svc_a', 'svc_b', 'svc_c']:
            populated_monitor.record_health_check(sid, True, 5.0)
        assert populated_monitor.detect_cascading_failure() == []

    def test_single_unhealthy_with_dependents(self, populated_monitor):
        # Make svc_b unhealthy; svc_a depends on svc_b
        populated_monitor.record_health_check('svc_a', True, 5.0)
        populated_monitor.record_health_check('svc_b', False, 5.0)
        populated_monitor.record_health_check('svc_b', False, 5.0)
        populated_monitor.record_health_check('svc_c', True, 5.0)
        at_risk = populated_monitor.detect_cascading_failure()
        assert 'svc_a' in at_risk
        # svc_b is the root cause, not at-risk
        assert 'svc_b' not in at_risk

    def test_transitive_cascading_failure(self, monitor):
        # Chain: svc_1 -> svc_2 -> svc_3
        for sid in ['svc_1', 'svc_2', 'svc_3']:
            monitor.register_service(_make_endpoint(sid))
        monitor.add_dependency(ServiceDependency(
            source_service='svc_1', target_service='svc_2',
        ))
        monitor.add_dependency(ServiceDependency(
            source_service='svc_2', target_service='svc_3',
        ))
        # Make the leaf (svc_3) unhealthy
        monitor.record_health_check('svc_1', True, 5.0)
        monitor.record_health_check('svc_2', True, 5.0)
        monitor.record_health_check('svc_3', False, 5.0)
        monitor.record_health_check('svc_3', False, 5.0)
        at_risk = monitor.detect_cascading_failure()
        # svc_2 depends on svc_3 (at risk), svc_1 depends on svc_2 (transitive)
        assert 'svc_1' in at_risk
        assert 'svc_2' in at_risk
        assert 'svc_3' not in at_risk

    def test_no_dependents_on_unhealthy(self, monitor):
        # svc_leaf is unhealthy but nothing depends on it
        monitor.register_service(_make_endpoint('svc_leaf'))
        monitor.record_health_check('svc_leaf', False, 5.0)
        monitor.record_health_check('svc_leaf', False, 5.0)
        assert monitor.detect_cascading_failure() == []
