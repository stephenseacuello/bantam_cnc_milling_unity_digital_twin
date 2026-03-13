"""Tests for shift handoff report generation.

Tests the ShiftEvent, ShiftReport, and ShiftReportGenerator classes
from kpi_calculator.py without any ROS2 dependency.
"""

import json
import sys
from unittest.mock import MagicMock

# Mock ROS2 and miracle_core dependencies before importing kpi_calculator
for mod in ['rclpy', 'rclpy.node', 'rclpy.callback_groups', 'rclpy.qos', 'rclpy.lifecycle',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
            'miracle_core.exceptions',
            'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = type('FakeNode', (), {
    'CRITICALITY_HIGH': 'HIGH',
    'CRITICALITY_MEDIUM': 'MEDIUM',
    '__init__': lambda self, *a, **kw: None,
    'get_logger': lambda self: MagicMock(),
    'create_publisher': lambda self, *a, **kw: MagicMock(),
    'create_subscription': lambda self, *a, **kw: MagicMock(),
    'create_timer': lambda self, *a, **kw: MagicMock(),
    'declare_and_validate_parameters': lambda self, specs: {k: MagicMock(value=v['default']) for k, v in specs.items()},
    'get_parameter': lambda self, name: MagicMock(value=0),
})

sys.modules.pop('miracle_scada.kpi_calculator', None)

import pytest
from miracle_scada.kpi_calculator import (
    ShiftEvent,
    ShiftReport,
    ShiftReportGenerator,
    MachineMetrics,
    OEESnapshot,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_event(
    timestamp=100.0, event_type='ALARM', machine_id='cnc1',
    severity='INFO', description='test event', data=None,
):
    return ShiftEvent(
        timestamp=timestamp,
        event_type=event_type,
        machine_id=machine_id,
        severity=severity,
        description=description,
        data=data or {},
    )


def _make_oee_snapshot(timestamp=100.0, oee=0.85, availability=0.95,
                       performance=0.92, quality=0.97, machine_id='fleet'):
    return OEESnapshot(
        timestamp=timestamp,
        oee=oee,
        availability=availability,
        performance=performance,
        quality=quality,
        machine_id=machine_id,
    )


def _make_machine_metrics(total_jobs=10, good_jobs=9, defect_jobs=1,
                          tool_life_used=0.5, tool_life_total=1.0):
    m = MachineMetrics()
    m.total_jobs = total_jobs
    m.good_jobs = good_jobs
    m.defect_jobs = defect_jobs
    m.tool_life_used = tool_life_used
    m.tool_life_total = tool_life_total
    return m


# ---------------------------------------------------------------------------
# ShiftEvent Tests
# ---------------------------------------------------------------------------

class TestShiftEvent:

    def test_create_event_with_defaults(self):
        e = ShiftEvent(
            timestamp=100.0, event_type='ALARM',
            machine_id='cnc1', severity='INFO', description='test',
        )
        assert e.timestamp == 100.0
        assert e.event_type == 'ALARM'
        assert e.data == {}

    def test_create_event_with_data(self):
        e = _make_event(data={'key': 'value'})
        assert e.data == {'key': 'value'}

    def test_event_types_are_valid_strings(self):
        for etype in ['ALARM', 'TOOL_CHANGE', 'PROGRAM_START', 'PROGRAM_END',
                       'OVERRIDE', 'ANOMALY', 'MAINTENANCE', 'CALIBRATION']:
            e = _make_event(event_type=etype)
            assert e.event_type == etype

    def test_severity_levels(self):
        for sev in ['INFO', 'WARNING', 'CRITICAL']:
            e = _make_event(severity=sev)
            assert e.severity == sev


# ---------------------------------------------------------------------------
# Event Recording Tests
# ---------------------------------------------------------------------------

class TestEventRecording:

    def test_record_single_event(self):
        gen = ShiftReportGenerator()
        e = _make_event()
        gen.record_event(e)
        assert len(gen.events) == 1
        assert gen.events[0] is e

    def test_record_multiple_events(self):
        gen = ShiftReportGenerator()
        for i in range(5):
            gen.record_event(_make_event(timestamp=float(i)))
        assert len(gen.events) == 5

    def test_events_property_returns_copy(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event())
        events = gen.events
        events.append(_make_event())
        assert len(gen.events) == 1  # original not modified

    def test_record_events_different_types(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(event_type='ALARM'))
        gen.record_event(_make_event(event_type='ANOMALY'))
        gen.record_event(_make_event(event_type='TOOL_CHANGE'))
        assert len(gen.events) == 3


# ---------------------------------------------------------------------------
# Report Generation Tests
# ---------------------------------------------------------------------------

class TestReportGeneration:

    def test_basic_report_fields(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(timestamp=50.0))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert report.operator_id == 'op1'
        assert report.shift_start == 0.0
        assert report.shift_end == 100.0
        assert report.machine_ids == ['cnc1']
        assert report.shift_id  # non-empty UUID

    def test_events_filtered_by_time_window(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(timestamp=50.0, event_type='ALARM'))
        gen.record_event(_make_event(timestamp=150.0, event_type='ALARM'))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert len(report.alarms) == 1
        assert report.alarms[0].timestamp == 50.0

    def test_alarm_categorisation(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(timestamp=10.0, event_type='ALARM', severity='CRITICAL'))
        gen.record_event(_make_event(timestamp=20.0, event_type='ANOMALY'))
        gen.record_event(_make_event(timestamp=30.0, event_type='MAINTENANCE'))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert len(report.alarms) == 1
        assert len(report.anomalies) == 1
        assert len(report.maintenance_performed) == 1

    def test_tool_change_count(self):
        gen = ShiftReportGenerator()
        for i in range(3):
            gen.record_event(_make_event(timestamp=float(i * 10), event_type='TOOL_CHANGE'))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert report.tool_changes == 3

    def test_parts_and_scrap_from_metrics(self):
        gen = ShiftReportGenerator()
        metrics = {
            'cnc1': _make_machine_metrics(total_jobs=20, good_jobs=18, defect_jobs=2),
        }
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
            machine_metrics=metrics,
        )
        assert report.total_parts_produced == 20
        assert report.total_scrap_count == 2

    def test_multi_machine_parts_aggregation(self):
        gen = ShiftReportGenerator()
        metrics = {
            'cnc1': _make_machine_metrics(total_jobs=10, defect_jobs=1),
            'cnc2': _make_machine_metrics(total_jobs=15, defect_jobs=3),
        }
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1', 'cnc2'],
            machine_metrics=metrics,
        )
        assert report.total_parts_produced == 25
        assert report.total_scrap_count == 4


# ---------------------------------------------------------------------------
# OEE Summary Tests
# ---------------------------------------------------------------------------

class TestOEESummary:

    def test_oee_summary_from_machine_snapshots(self):
        gen = ShiftReportGenerator()
        machine_snaps = {
            'cnc1': [
                _make_oee_snapshot(timestamp=50.0, oee=0.80, availability=0.90,
                                   performance=0.90, quality=0.99, machine_id='cnc1'),
                _make_oee_snapshot(timestamp=60.0, oee=0.82, availability=0.92,
                                   performance=0.91, quality=0.98, machine_id='cnc1'),
            ],
        }
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
            machine_oee_snapshots=machine_snaps,
        )
        oee = report.oee_summary['cnc1']
        assert abs(oee['overall'] - 0.81) < 0.01
        assert abs(oee['availability'] - 0.91) < 0.01

    def test_oee_summary_from_fleet_snapshots(self):
        gen = ShiftReportGenerator()
        fleet_snaps = [
            _make_oee_snapshot(timestamp=50.0, oee=0.85),
            _make_oee_snapshot(timestamp=60.0, oee=0.87),
        ]
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1', 'cnc2'],
            oee_snapshots=fleet_snaps,
        )
        # Both machines get fleet average
        assert abs(report.oee_summary['cnc1']['overall'] - 0.86) < 0.01
        assert abs(report.oee_summary['cnc2']['overall'] - 0.86) < 0.01

    def test_oee_summary_no_snapshots(self):
        gen = ShiftReportGenerator()
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert report.oee_summary['cnc1']['overall'] == 0.0

    def test_oee_summary_snapshots_outside_window(self):
        gen = ShiftReportGenerator()
        machine_snaps = {
            'cnc1': [
                _make_oee_snapshot(timestamp=200.0, oee=0.90, machine_id='cnc1'),
            ],
        }
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
            machine_oee_snapshots=machine_snaps,
        )
        assert report.oee_summary['cnc1']['overall'] == 0.0


# ---------------------------------------------------------------------------
# Pending Issues Tests
# ---------------------------------------------------------------------------

class TestPendingIssues:

    def test_critical_alarm_creates_pending_issue(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(
            timestamp=50.0, event_type='ALARM',
            severity='CRITICAL', description='spindle overheat',
        ))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert any('spindle overheat' in i for i in report.pending_issues)

    def test_warning_alarm_creates_pending_issue(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(
            timestamp=50.0, event_type='ALARM',
            severity='WARNING', description='coolant low',
        ))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert any('coolant low' in i for i in report.pending_issues)

    def test_info_alarm_no_pending_issue(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(
            timestamp=50.0, event_type='ALARM',
            severity='INFO', description='routine check',
        ))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert not any('routine check' in i for i in report.pending_issues)

    def test_low_tool_rul_pending_issue(self):
        gen = ShiftReportGenerator()
        metrics = {
            'cnc1': _make_machine_metrics(tool_life_used=0.85, tool_life_total=1.0),
        }
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
            machine_metrics=metrics,
        )
        assert any('remaining life' in i for i in report.pending_issues)

    def test_calibration_drift_pending_issue(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(
            timestamp=50.0, event_type='CALIBRATION',
            machine_id='cnc1', description='axis calibration',
            data={'drift': 0.08},
        ))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert any('drift' in i.lower() for i in report.pending_issues)

    def test_recurring_anomaly_pending_issue(self):
        gen = ShiftReportGenerator()
        for _ in range(2):
            gen.record_event(_make_event(
                timestamp=50.0, event_type='ANOMALY',
                description='vibration spike',
            ))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert any('recurring' in i.lower() for i in report.pending_issues)


# ---------------------------------------------------------------------------
# Recommendations Tests
# ---------------------------------------------------------------------------

class TestRecommendations:

    def test_tool_approaching_eol_recommendation(self):
        gen = ShiftReportGenerator()
        metrics = {
            'cnc1': _make_machine_metrics(tool_life_used=0.75, tool_life_total=1.0),
        }
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
            machine_metrics=metrics,
        )
        assert any('tool change' in r.lower() for r in report.recommendations)

    def test_recurring_anomaly_recommendation(self):
        gen = ShiftReportGenerator()
        for _ in range(3):
            gen.record_event(_make_event(
                timestamp=50.0, event_type='ANOMALY',
                description='thermal drift',
            ))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert any('recurring' in r.lower() for r in report.recommendations)

    def test_low_oee_recommendation(self):
        gen = ShiftReportGenerator()
        machine_snaps = {
            'cnc1': [
                _make_oee_snapshot(timestamp=50.0, oee=0.50, availability=0.70,
                                   performance=0.80, quality=0.90, machine_id='cnc1'),
            ],
        }
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
            machine_oee_snapshots=machine_snaps,
        )
        assert any('low oee' in r.lower() for r in report.recommendations)

    def test_high_scrap_rate_recommendation(self):
        gen = ShiftReportGenerator()
        metrics = {
            'cnc1': _make_machine_metrics(total_jobs=10, good_jobs=8, defect_jobs=2),
        }
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
            machine_metrics=metrics,
        )
        assert any('scrap rate' in r.lower() for r in report.recommendations)


# ---------------------------------------------------------------------------
# Highlight Selection Tests
# ---------------------------------------------------------------------------

class TestHighlightSelection:

    def test_critical_event_highlight(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(
            timestamp=50.0, event_type='ALARM',
            severity='CRITICAL', description='emergency stop',
        ))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert 'CRITICAL' in report.highlight
        assert 'emergency stop' in report.highlight

    def test_excellent_oee_highlight(self):
        gen = ShiftReportGenerator()
        machine_snaps = {
            'cnc1': [
                _make_oee_snapshot(timestamp=50.0, oee=0.92, availability=0.96,
                                   performance=0.97, quality=0.99, machine_id='cnc1'),
            ],
        }
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
            machine_oee_snapshots=machine_snaps,
        )
        assert 'Excellent OEE' in report.highlight

    def test_default_highlight_with_events(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(timestamp=50.0, event_type='PROGRAM_START', severity='INFO'))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert 'events' in report.highlight.lower()

    def test_quiet_shift_highlight(self):
        gen = ShiftReportGenerator()
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert 'quiet' in report.highlight.lower()


# ---------------------------------------------------------------------------
# Text Export Tests
# ---------------------------------------------------------------------------

class TestTextExport:

    def test_export_contains_header(self):
        gen = ShiftReportGenerator()
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        text = gen.export_report_text(report)
        assert '# Shift Handoff Report' in text

    def test_export_contains_operator(self):
        gen = ShiftReportGenerator()
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op_alpha', machine_ids=['cnc1'],
        )
        text = gen.export_report_text(report)
        assert 'op_alpha' in text

    def test_export_contains_production_summary(self):
        gen = ShiftReportGenerator()
        metrics = {'cnc1': _make_machine_metrics(total_jobs=25, defect_jobs=3)}
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
            machine_metrics=metrics,
        )
        text = gen.export_report_text(report)
        assert 'Parts Produced' in text
        assert '25' in text
        assert 'Scrap Count' in text

    def test_export_alarm_section(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(
            timestamp=50.0, event_type='ALARM',
            severity='WARNING', description='coolant pressure low',
        ))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        text = gen.export_report_text(report)
        assert '## Alarms' in text
        assert 'coolant pressure low' in text

    def test_export_recommendations_section(self):
        gen = ShiftReportGenerator()
        metrics = {
            'cnc1': _make_machine_metrics(tool_life_used=0.85, tool_life_total=1.0),
        }
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
            machine_metrics=metrics,
        )
        text = gen.export_report_text(report)
        assert '## Recommendations' in text


# ---------------------------------------------------------------------------
# JSON Export Tests
# ---------------------------------------------------------------------------

class TestJSONExport:

    def test_json_round_trip(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(timestamp=50.0, event_type='ALARM'))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        json_str = gen.export_report_json(report)
        data = json.loads(json_str)
        assert data['operator_id'] == 'op1'
        assert data['total_parts_produced'] == 0
        assert len(data['alarms']) == 1

    def test_json_contains_all_keys(self):
        gen = ShiftReportGenerator()
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        json_str = gen.export_report_json(report)
        data = json.loads(json_str)
        expected_keys = {
            'shift_id', 'shift_start', 'shift_end', 'operator_id',
            'machine_ids', 'oee_summary', 'total_parts_produced',
            'total_scrap_count', 'tool_changes', 'alarms', 'anomalies',
            'maintenance_performed', 'pending_issues', 'recommendations',
            'highlight',
        }
        assert set(data.keys()) == expected_keys

    def test_json_event_structure(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(
            timestamp=50.0, event_type='ANOMALY', machine_id='cnc2',
            severity='WARNING', description='vibration',
        ))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc2'],
        )
        data = json.loads(gen.export_report_json(report))
        anomaly = data['anomalies'][0]
        assert anomaly['timestamp'] == 50.0
        assert anomaly['event_type'] == 'ANOMALY'
        assert anomaly['machine_id'] == 'cnc2'
        assert anomaly['severity'] == 'WARNING'

    def test_json_valid_format(self):
        gen = ShiftReportGenerator()
        for i in range(5):
            gen.record_event(_make_event(timestamp=float(i * 10), event_type='ALARM'))
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        json_str = gen.export_report_json(report)
        # Should not raise
        data = json.loads(json_str)
        assert isinstance(data, dict)


# ---------------------------------------------------------------------------
# Shift Comparison Tests
# ---------------------------------------------------------------------------

class TestShiftComparison:

    def _make_report(self, gen, oee=0.85, parts=10, scrap=1, alarms=0, tool_changes=1):
        machine_snaps = {
            'cnc1': [
                _make_oee_snapshot(timestamp=50.0, oee=oee, availability=0.95,
                                   performance=0.92, quality=0.97, machine_id='cnc1'),
            ],
        }
        metrics = {'cnc1': _make_machine_metrics(total_jobs=parts, defect_jobs=scrap)}
        for i in range(alarms):
            gen.record_event(_make_event(
                timestamp=50.0 + i, event_type='ALARM', severity='WARNING',
            ))
        for i in range(tool_changes):
            gen.record_event(_make_event(
                timestamp=50.0 + i, event_type='TOOL_CHANGE',
            ))
        return gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
            machine_metrics=metrics,
            machine_oee_snapshots=machine_snaps,
        )

    def test_comparison_improvement(self):
        gen = ShiftReportGenerator()
        r1 = self._make_report(gen, oee=0.80, parts=10, scrap=2, alarms=3)
        gen2 = ShiftReportGenerator()
        r2 = self._make_report(gen2, oee=0.90, parts=15, scrap=1, alarms=1)
        comp = gen.get_shift_comparison(r1, r2)
        assert comp['oee_delta'] > 0
        assert comp['parts_delta'] == 5
        assert comp['scrap_delta'] == -1
        assert comp['alarm_count_delta'] == -2

    def test_comparison_degradation(self):
        gen = ShiftReportGenerator()
        r1 = self._make_report(gen, oee=0.90, parts=20, scrap=1, alarms=1)
        gen2 = ShiftReportGenerator()
        r2 = self._make_report(gen2, oee=0.70, parts=12, scrap=4, alarms=5)
        comp = gen.get_shift_comparison(r1, r2)
        assert comp['oee_delta'] < 0
        assert comp['parts_delta'] < 0
        assert comp['scrap_delta'] > 0
        assert comp['alarm_count_delta'] > 0

    def test_comparison_contains_shift_ids(self):
        gen = ShiftReportGenerator()
        r1 = self._make_report(gen, oee=0.85)
        gen2 = ShiftReportGenerator()
        r2 = self._make_report(gen2, oee=0.85)
        comp = gen.get_shift_comparison(r1, r2)
        assert comp['shift1_id'] == r1.shift_id
        assert comp['shift2_id'] == r2.shift_id

    def test_comparison_identical_shifts(self):
        gen = ShiftReportGenerator()
        r1 = self._make_report(gen, oee=0.85, parts=10, scrap=1)
        gen2 = ShiftReportGenerator()
        r2 = self._make_report(gen2, oee=0.85, parts=10, scrap=1)
        comp = gen.get_shift_comparison(r1, r2)
        assert abs(comp['oee_delta']) < 0.001
        assert comp['parts_delta'] == 0
        assert comp['scrap_delta'] == 0


# ---------------------------------------------------------------------------
# Empty Shift Tests
# ---------------------------------------------------------------------------

class TestEmptyShift:

    def test_empty_shift_no_events(self):
        gen = ShiftReportGenerator()
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert report.total_parts_produced == 0
        assert report.total_scrap_count == 0
        assert report.tool_changes == 0
        assert len(report.alarms) == 0
        assert len(report.anomalies) == 0
        assert len(report.maintenance_performed) == 0

    def test_empty_shift_has_highlight(self):
        gen = ShiftReportGenerator()
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1'],
        )
        assert report.highlight  # should have a default highlight

    def test_empty_shift_oee_zeroed(self):
        gen = ShiftReportGenerator()
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1', 'cnc2'],
        )
        for mid in ['cnc1', 'cnc2']:
            assert report.oee_summary[mid]['overall'] == 0.0


# ---------------------------------------------------------------------------
# Multi-Machine Tests
# ---------------------------------------------------------------------------

class TestMultiMachine:

    def test_multi_machine_report(self):
        gen = ShiftReportGenerator()
        gen.record_event(_make_event(timestamp=50.0, machine_id='cnc1', event_type='ALARM'))
        gen.record_event(_make_event(timestamp=60.0, machine_id='cnc2', event_type='ANOMALY'))
        gen.record_event(_make_event(timestamp=70.0, machine_id='cnc3', event_type='TOOL_CHANGE'))

        metrics = {
            'cnc1': _make_machine_metrics(total_jobs=10, defect_jobs=0),
            'cnc2': _make_machine_metrics(total_jobs=8, defect_jobs=2),
            'cnc3': _make_machine_metrics(total_jobs=12, defect_jobs=1),
        }

        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1', 'cnc2', 'cnc3'],
            machine_metrics=metrics,
        )
        assert report.total_parts_produced == 30
        assert report.total_scrap_count == 3
        assert len(report.alarms) == 1
        assert len(report.anomalies) == 1
        assert report.tool_changes == 1
        assert set(report.machine_ids) == {'cnc1', 'cnc2', 'cnc3'}

    def test_multi_machine_oee_per_machine(self):
        gen = ShiftReportGenerator()
        machine_snaps = {
            'cnc1': [_make_oee_snapshot(timestamp=50.0, oee=0.90, machine_id='cnc1')],
            'cnc2': [_make_oee_snapshot(timestamp=50.0, oee=0.70, machine_id='cnc2')],
        }
        report = gen.generate_report(
            shift_start=0.0, shift_end=100.0,
            operator_id='op1', machine_ids=['cnc1', 'cnc2'],
            machine_oee_snapshots=machine_snaps,
        )
        assert abs(report.oee_summary['cnc1']['overall'] - 0.90) < 0.01
        assert abs(report.oee_summary['cnc2']['overall'] - 0.70) < 0.01
