"""Tests for HealthCheckManager."""
import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants',
            'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
            'rclpy.callback_groups',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_msgs', 'miracle_msgs.msg',
            'std_msgs', 'std_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

import time
import pytest

from miracle_resiliency.recovery_orchestrator import (
    HealthCheck,
    HealthCheckConfig,
    HealthCheckManager,
    SystemHealthReport,
)


def _make_check(service_id: str, status: str = 'healthy',
                ts: float | None = None, latency: float = 1.0,
                name: str = 'ping', message: str = 'ok') -> HealthCheck:
    return HealthCheck(
        service_id=service_id,
        check_name=name,
        status=status,
        message=message,
        timestamp=ts if ts is not None else time.time(),
        latency_ms=latency,
        details={},
    )


class TestHealthCheckDataclasses:
    """Verify dataclass defaults and construction."""

    def test_health_check_fields(self):
        hc = HealthCheck(
            service_id='svc1', check_name='ping', status='healthy',
            message='ok', timestamp=1000.0, latency_ms=2.5,
        )
        assert hc.service_id == 'svc1'
        assert hc.status == 'healthy'
        assert hc.details == {}

    def test_health_check_config_defaults(self):
        cfg = HealthCheckConfig(service_id='svc1')
        assert cfg.check_interval_sec == 30.0
        assert cfg.timeout_sec == 5.0
        assert cfg.healthy_threshold == 3
        assert cfg.unhealthy_threshold == 2

    def test_system_health_report_defaults(self):
        rpt = SystemHealthReport(timestamp=0.0, overall_status='healthy')
        assert rpt.services == {}
        assert rpt.healthy_count == 0
        assert rpt.total_count == 0
        assert rpt.uptime_pct == 100.0


class TestHealthCheckManagerRegistration:
    """Test service registration and basic recording."""

    def test_register_and_record(self):
        mgr = HealthCheckManager()
        cfg = HealthCheckConfig(service_id='svc_a')
        mgr.register_service(cfg)

        check = _make_check('svc_a')
        mgr.record_check(check)

        result = mgr.get_service_health('svc_a')
        assert result is not None
        assert result.status == 'healthy'

    def test_record_unregistered_raises(self):
        mgr = HealthCheckManager()
        with pytest.raises(KeyError, match="not registered"):
            mgr.record_check(_make_check('unknown'))

    def test_get_service_health_no_history(self):
        mgr = HealthCheckManager()
        mgr.register_service(HealthCheckConfig(service_id='empty'))
        assert mgr.get_service_health('empty') is None

    def test_get_service_health_unknown(self):
        mgr = HealthCheckManager()
        assert mgr.get_service_health('nonexistent') is None


class TestSystemReport:
    """Test aggregate report generation."""

    def _setup_mgr(self):
        mgr = HealthCheckManager()
        for sid in ['a', 'b', 'c']:
            mgr.register_service(HealthCheckConfig(service_id=sid))
        return mgr

    def test_all_healthy(self):
        mgr = self._setup_mgr()
        for sid in ['a', 'b', 'c']:
            mgr.record_check(_make_check(sid, 'healthy'))

        rpt = mgr.get_system_report()
        assert rpt.overall_status == 'healthy'
        assert rpt.healthy_count == 3
        assert rpt.total_count == 3
        assert rpt.uptime_pct == pytest.approx(100.0)

    def test_one_unhealthy(self):
        mgr = self._setup_mgr()
        mgr.record_check(_make_check('a', 'healthy'))
        mgr.record_check(_make_check('b', 'unhealthy'))
        mgr.record_check(_make_check('c', 'healthy'))

        rpt = mgr.get_system_report()
        assert rpt.overall_status == 'degraded'
        assert rpt.healthy_count == 2

    def test_all_unhealthy(self):
        mgr = self._setup_mgr()
        for sid in ['a', 'b', 'c']:
            mgr.record_check(_make_check(sid, 'unhealthy'))

        rpt = mgr.get_system_report()
        assert rpt.overall_status == 'unhealthy'
        assert rpt.healthy_count == 0


class TestIsSystemHealthy:
    """Test is_system_healthy and get_unhealthy_services."""

    def test_healthy_system(self):
        mgr = HealthCheckManager()
        mgr.register_service(HealthCheckConfig(service_id='s1'))
        mgr.record_check(_make_check('s1', 'healthy'))
        assert mgr.is_system_healthy() is True
        assert mgr.get_unhealthy_services() == []

    def test_degraded_still_healthy(self):
        mgr = HealthCheckManager()
        mgr.register_service(HealthCheckConfig(service_id='s1'))
        mgr.record_check(_make_check('s1', 'degraded'))
        assert mgr.is_system_healthy() is True

    def test_unhealthy_system(self):
        mgr = HealthCheckManager()
        mgr.register_service(HealthCheckConfig(service_id='s1'))
        mgr.register_service(HealthCheckConfig(service_id='s2'))
        mgr.record_check(_make_check('s1', 'healthy'))
        mgr.record_check(_make_check('s2', 'unhealthy'))
        assert mgr.is_system_healthy() is False
        assert mgr.get_unhealthy_services() == ['s2']


class TestUptime:
    """Test uptime calculation."""

    def test_full_uptime(self):
        mgr = HealthCheckManager()
        mgr.register_service(HealthCheckConfig(service_id='u1'))
        now = time.time()
        for i in range(5):
            mgr.record_check(_make_check('u1', 'healthy', ts=now - i))
        assert mgr.get_uptime('u1', window_sec=60) == pytest.approx(100.0)

    def test_partial_uptime(self):
        mgr = HealthCheckManager()
        mgr.register_service(HealthCheckConfig(service_id='u1'))
        now = time.time()
        # 3 healthy, 1 unhealthy within window
        mgr.record_check(_make_check('u1', 'healthy', ts=now - 3))
        mgr.record_check(_make_check('u1', 'healthy', ts=now - 2))
        mgr.record_check(_make_check('u1', 'unhealthy', ts=now - 1))
        mgr.record_check(_make_check('u1', 'healthy', ts=now))
        assert mgr.get_uptime('u1', window_sec=60) == pytest.approx(75.0)

    def test_no_checks_returns_100(self):
        mgr = HealthCheckManager()
        mgr.register_service(HealthCheckConfig(service_id='u1'))
        assert mgr.get_uptime('u1', window_sec=60) == pytest.approx(100.0)


class TestFlapDetection:
    """Test flap detection marks service as degraded."""

    def test_flapping_service(self):
        mgr = HealthCheckManager()
        mgr.register_service(HealthCheckConfig(service_id='flap'))
        now = time.time()
        # Produce more than 3 status transitions in 60 seconds
        statuses = ['healthy', 'unhealthy', 'healthy', 'unhealthy', 'healthy']
        for i, s in enumerate(statuses):
            mgr.record_check(_make_check('flap', s, ts=now - (len(statuses) - i)))

        result = mgr.get_service_health('flap')
        assert result is not None
        assert result.status == 'degraded'
        assert 'flap-detected' in result.message

    def test_stable_service_not_flapping(self):
        mgr = HealthCheckManager()
        mgr.register_service(HealthCheckConfig(service_id='stable'))
        now = time.time()
        for i in range(5):
            mgr.record_check(_make_check('stable', 'healthy', ts=now - i))

        result = mgr.get_service_health('stable')
        assert result is not None
        assert result.status == 'healthy'
        assert 'flap-detected' not in result.message
