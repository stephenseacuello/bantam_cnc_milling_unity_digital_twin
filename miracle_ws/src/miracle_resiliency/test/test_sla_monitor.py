"""Tests for SLAMonitor.

Covers SLA definition and registration, measurement recording with
compliance evaluation, compliance percentage calculation, report generation,
violation retrieval, at-risk detection, trend analysis, and edge cases.
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
    SLAMonitor,
    SLADefinition,
    SLAMeasurement,
    SLAReport,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_definition(
    sla_id: str = 'sla-1',
    service_id: str = 'svc-spindle',
    metric_name: str = 'latency_ms',
    target_value: float = 100.0,
    comparison: str = 'lt',
    measurement_window_sec: float = 3600.0,
    min_compliance_pct: float = 99.0,
) -> SLADefinition:
    return SLADefinition(
        sla_id=sla_id,
        service_id=service_id,
        metric_name=metric_name,
        target_value=target_value,
        comparison=comparison,
        measurement_window_sec=measurement_window_sec,
        min_compliance_pct=min_compliance_pct,
    )


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def monitor() -> SLAMonitor:
    return SLAMonitor()


@pytest.fixture
def populated_monitor(monitor: SLAMonitor) -> SLAMonitor:
    """Monitor with one SLA and 100 compliant measurements."""
    defn = _make_definition(min_compliance_pct=95.0)
    monitor.define_sla(defn)
    for i in range(100):
        monitor.record_measurement('sla-1', value=50.0, timestamp=float(i))
    return monitor


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestDefineSLA:
    """Tests for SLA registration."""

    def test_define_sla_stores_definition(self, monitor: SLAMonitor) -> None:
        defn = _make_definition()
        monitor.define_sla(defn)
        # Compliance should be 100% for an SLA with no measurements
        assert monitor.get_compliance('sla-1') == 100.0

    def test_define_sla_rejects_invalid_comparison(self, monitor: SLAMonitor) -> None:
        defn = _make_definition(comparison='invalid')
        with pytest.raises(ValueError, match="Invalid comparison"):
            monitor.define_sla(defn)


class TestRecordMeasurement:
    """Tests for measurement recording and compliance evaluation."""

    def test_compliant_measurement_lt(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(_make_definition(comparison='lt', target_value=100.0))
        m = monitor.record_measurement('sla-1', value=50.0, timestamp=1.0)
        assert m.compliant is True
        assert m.value == 50.0

    def test_non_compliant_measurement_lt(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(_make_definition(comparison='lt', target_value=100.0))
        m = monitor.record_measurement('sla-1', value=150.0, timestamp=1.0)
        assert m.compliant is False

    def test_comparison_gt(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(_make_definition(comparison='gt', target_value=50.0))
        assert monitor.record_measurement('sla-1', 100.0, 1.0).compliant is True
        assert monitor.record_measurement('sla-1', 30.0, 2.0).compliant is False

    def test_comparison_lte(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(_make_definition(comparison='lte', target_value=100.0))
        assert monitor.record_measurement('sla-1', 100.0, 1.0).compliant is True
        assert monitor.record_measurement('sla-1', 100.1, 2.0).compliant is False

    def test_comparison_gte(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(_make_definition(comparison='gte', target_value=50.0))
        assert monitor.record_measurement('sla-1', 50.0, 1.0).compliant is True
        assert monitor.record_measurement('sla-1', 49.9, 2.0).compliant is False

    def test_comparison_eq(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(_make_definition(comparison='eq', target_value=42.0))
        assert monitor.record_measurement('sla-1', 42.0, 1.0).compliant is True
        assert monitor.record_measurement('sla-1', 43.0, 2.0).compliant is False

    def test_record_for_unknown_sla_raises(self, monitor: SLAMonitor) -> None:
        with pytest.raises(KeyError, match="No SLA defined"):
            monitor.record_measurement('nonexistent', 1.0, 1.0)


class TestGetCompliance:
    """Tests for compliance percentage computation."""

    def test_full_compliance(self, populated_monitor: SLAMonitor) -> None:
        assert populated_monitor.get_compliance('sla-1') == 100.0

    def test_partial_compliance(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(_make_definition(measurement_window_sec=1000.0))
        # 90 compliant, 10 non-compliant
        for i in range(90):
            monitor.record_measurement('sla-1', 50.0, float(i))
        for i in range(90, 100):
            monitor.record_measurement('sla-1', 150.0, float(i))
        assert monitor.get_compliance('sla-1') == 90.0

    def test_no_measurements_returns_100(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(_make_definition())
        assert monitor.get_compliance('sla-1') == 100.0

    def test_window_filters_old_measurements(self, monitor: SLAMonitor) -> None:
        """Measurements outside the window should not count."""
        monitor.define_sla(
            _make_definition(measurement_window_sec=100.0)
        )
        # Old violation at t=0
        monitor.record_measurement('sla-1', 999.0, 0.0)
        # Recent compliant measurements at t=900..909
        for i in range(10):
            monitor.record_measurement('sla-1', 50.0, 900.0 + i)
        # The old violation should be outside the window
        assert monitor.get_compliance('sla-1') == 100.0


class TestGetReport:
    """Tests for SLA report generation."""

    def test_meeting_status(self, populated_monitor: SLAMonitor) -> None:
        report = populated_monitor.get_report('sla-1')
        assert isinstance(report, SLAReport)
        assert report.sla_id == 'sla-1'
        assert report.compliance_pct == 100.0
        assert report.total_measurements == 100
        assert report.violations == 0
        assert report.current_status == 'meeting'

    def test_breached_status(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(
            _make_definition(min_compliance_pct=99.0, measurement_window_sec=1000.0)
        )
        # 5 compliant, 95 non-compliant -> 5% compliance
        for i in range(5):
            monitor.record_measurement('sla-1', 50.0, float(i))
        for i in range(5, 100):
            monitor.record_measurement('sla-1', 999.0, float(i))
        report = monitor.get_report('sla-1')
        assert report.current_status == 'breached'
        assert report.violations == 95

    def test_at_risk_status(self, monitor: SLAMonitor) -> None:
        """Compliance within 2% of threshold -> at_risk."""
        monitor.define_sla(
            _make_definition(min_compliance_pct=95.0, measurement_window_sec=1000.0)
        )
        # 96 compliant out of 100 -> 96% compliance.
        # Threshold is 95%, so within 2% (96 <= 95+2=97).
        for i in range(96):
            monitor.record_measurement('sla-1', 50.0, float(i))
        for i in range(96, 100):
            monitor.record_measurement('sla-1', 999.0, float(i))
        report = monitor.get_report('sla-1')
        assert report.current_status == 'at_risk'


class TestGetAllReports:
    """Tests for bulk report generation."""

    def test_returns_reports_for_all_slas(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(_make_definition(sla_id='a'))
        monitor.define_sla(_make_definition(sla_id='b'))
        monitor.define_sla(_make_definition(sla_id='c'))
        reports = monitor.get_all_reports()
        assert len(reports) == 3
        ids = {r.sla_id for r in reports}
        assert ids == {'a', 'b', 'c'}


class TestGetViolations:
    """Tests for violation retrieval."""

    def test_returns_only_violations(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(_make_definition(measurement_window_sec=1000.0))
        monitor.record_measurement('sla-1', 50.0, 1.0)   # compliant
        monitor.record_measurement('sla-1', 200.0, 2.0)   # violation
        monitor.record_measurement('sla-1', 50.0, 3.0)    # compliant
        monitor.record_measurement('sla-1', 300.0, 4.0)   # violation
        violations = monitor.get_violations('sla-1')
        assert len(violations) == 2
        assert all(not v.compliant for v in violations)

    def test_last_n_limits_results(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(_make_definition(measurement_window_sec=1000.0))
        for i in range(10):
            monitor.record_measurement('sla-1', 999.0, float(i))
        violations = monitor.get_violations('sla-1', last_n=3)
        assert len(violations) == 3

    def test_violations_for_unknown_sla_raises(self, monitor: SLAMonitor) -> None:
        with pytest.raises(KeyError):
            monitor.get_violations('missing')


class TestIsAtRisk:
    """Tests for at-risk detection."""

    def test_not_at_risk_when_well_above_threshold(
        self, populated_monitor: SLAMonitor
    ) -> None:
        # 100% compliance, threshold 99% -> not at risk (100 > 101)
        assert populated_monitor.is_at_risk('sla-1') is False

    def test_at_risk_when_within_two_percent(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(
            _make_definition(min_compliance_pct=95.0, measurement_window_sec=1000.0)
        )
        # 96 compliant / 100 -> 96% compliance; threshold+2 = 97%, 96 < 97
        for i in range(96):
            monitor.record_measurement('sla-1', 50.0, float(i))
        for i in range(96, 100):
            monitor.record_measurement('sla-1', 999.0, float(i))
        assert monitor.is_at_risk('sla-1') is True

    def test_at_risk_when_breached(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(
            _make_definition(min_compliance_pct=99.0, measurement_window_sec=1000.0)
        )
        # All violations -> 0% compliance
        for i in range(10):
            monitor.record_measurement('sla-1', 999.0, float(i))
        assert monitor.is_at_risk('sla-1') is True


class TestTrendAnalysis:
    """Tests for trend computation in reports."""

    def test_improving_trend(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(
            _make_definition(min_compliance_pct=50.0, measurement_window_sec=1000.0)
        )
        # Older half: mostly violations
        for i in range(10):
            monitor.record_measurement('sla-1', 999.0, float(i))
        # Newer half: all compliant
        for i in range(10, 20):
            monitor.record_measurement('sla-1', 50.0, float(i))
        report = monitor.get_report('sla-1')
        assert report.trend == 'improving'

    def test_degrading_trend(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(
            _make_definition(min_compliance_pct=50.0, measurement_window_sec=1000.0)
        )
        # Older half: all compliant
        for i in range(10):
            monitor.record_measurement('sla-1', 50.0, float(i))
        # Newer half: mostly violations
        for i in range(10, 20):
            monitor.record_measurement('sla-1', 999.0, float(i))
        report = monitor.get_report('sla-1')
        assert report.trend == 'degrading'

    def test_stable_trend_few_measurements(self, monitor: SLAMonitor) -> None:
        monitor.define_sla(_make_definition(measurement_window_sec=1000.0))
        # Fewer than 4 measurements -> always stable
        monitor.record_measurement('sla-1', 50.0, 1.0)
        report = monitor.get_report('sla-1')
        assert report.trend == 'stable'
