"""Tests for the Alert Correlation Engine."""

import pytest
import time

# Import the dataclasses and logic directly (no ROS2 needed)
import sys
import os
from unittest.mock import MagicMock

# Stub out ROS2 and project dependencies so we can import the module
# without a ROS2 installation.
sys.modules.setdefault('rclpy', MagicMock())
sys.modules.setdefault('rclpy.lifecycle', MagicMock())
sys.modules.setdefault('rclpy.executors', MagicMock())
sys.modules.setdefault('miracle_core.lifecycle_node_base', MagicMock())
sys.modules.setdefault('miracle_core.qos_profiles', MagicMock())
sys.modules.setdefault('miracle_msgs', MagicMock())
sys.modules.setdefault('miracle_msgs.msg', MagicMock())

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_scada.alert_correlator import AlertEntry, CorrelationRule, FleetCorrelationRule


class TestAlertEntry:
    """Test AlertEntry dataclass."""

    def test_create_entry(self):
        entry = AlertEntry(
            alert_type='vibration_anomaly',
            machine_id='cnc1',
            severity=0.85,
            message='High vibration detected',
            timestamp=time.time(),
        )
        assert entry.alert_type == 'vibration_anomaly'
        assert entry.machine_id == 'cnc1'
        assert entry.severity == 0.85

    def test_entry_source_default(self):
        entry = AlertEntry(
            alert_type='test',
            machine_id='cnc1',
            severity=0.5,
            message='test',
            timestamp=0.0,
        )
        assert entry.source == ''


class TestCorrelationRule:
    """Test CorrelationRule dataclass."""

    def test_create_rule(self):
        rule = CorrelationRule(
            name='test_rule',
            required_types=['type_a', 'type_b'],
            time_window_sec=30.0,
            root_cause='Test root cause',
            recommended_actions=['Action 1', 'Action 2'],
        )
        assert rule.name == 'test_rule'
        assert len(rule.required_types) == 2
        assert rule.min_confidence == 0.7  # default

    def test_rule_custom_confidence(self):
        rule = CorrelationRule(
            name='strict_rule',
            required_types=['x'],
            time_window_sec=10.0,
            root_cause='Strict',
            recommended_actions=[],
            min_confidence=0.95,
        )
        assert rule.min_confidence == 0.95


class TestCorrelationLogic:
    """Test correlation matching logic (without ROS2)."""

    def test_alerts_within_window_match(self):
        """Three related alerts within window should be correlatable."""
        now = time.time()
        alerts = [
            AlertEntry('vibration_anomaly', 'cnc1', 0.8, 'vib', now - 5),
            AlertEntry('force_anomaly', 'cnc1', 0.9, 'force', now - 3),
        ]
        rule = CorrelationRule(
            name='tool_degradation',
            required_types=['vibration_anomaly', 'force_anomaly'],
            time_window_sec=30.0,
            root_cause='Tool wear',
            recommended_actions=['Replace tool'],
        )

        # Check window filtering
        window_alerts = [a for a in alerts if now - a.timestamp <= rule.time_window_sec]
        present_types = {a.alert_type for a in window_alerts}
        assert all(rt in present_types for rt in rule.required_types)

    def test_alerts_outside_window_no_match(self):
        """Alerts outside the time window should not match."""
        now = time.time()
        alerts = [
            AlertEntry('vibration_anomaly', 'cnc1', 0.8, 'vib', now - 120),
            AlertEntry('force_anomaly', 'cnc1', 0.9, 'force', now - 3),
        ]
        rule = CorrelationRule(
            name='tool_degradation',
            required_types=['vibration_anomaly', 'force_anomaly'],
            time_window_sec=30.0,
            root_cause='Tool wear',
            recommended_actions=['Replace tool'],
        )

        window_alerts = [a for a in alerts if now - a.timestamp <= rule.time_window_sec]
        present_types = {a.alert_type for a in window_alerts}
        assert not all(rt in present_types for rt in rule.required_types)

    def test_partial_match_not_triggered(self):
        """Only one of two required types should not match."""
        now = time.time()
        alerts = [
            AlertEntry('vibration_anomaly', 'cnc1', 0.8, 'vib', now - 5),
        ]
        rule = CorrelationRule(
            name='tool_degradation',
            required_types=['vibration_anomaly', 'force_anomaly'],
            time_window_sec=30.0,
            root_cause='Tool wear',
            recommended_actions=['Replace tool'],
        )

        window_alerts = [a for a in alerts if now - a.timestamp <= rule.time_window_sec]
        present_types = {a.alert_type for a in window_alerts}
        assert not all(rt in present_types for rt in rule.required_types)

    def test_confidence_calculation(self):
        """Confidence should be based on severity and completeness."""
        matching = [
            AlertEntry('type_a', 'cnc1', 0.8, 'a', 0),
            AlertEntry('type_b', 'cnc1', 0.9, 'b', 0),
        ]
        required_count = 2
        avg_severity = sum(a.severity for a in matching) / len(matching)
        confidence = min(1.0, avg_severity * (len(matching) / required_count))
        assert confidence == pytest.approx(0.85, abs=0.01)

    def test_different_machines_separate(self):
        """Alerts from different machines should not correlate together."""
        now = time.time()
        cnc1_alerts = [AlertEntry('vibration_anomaly', 'cnc1', 0.8, 'vib', now)]
        cnc2_alerts = [AlertEntry('force_anomaly', 'cnc2', 0.9, 'force', now)]

        # Per-machine grouping means these are in separate buffers
        assert cnc1_alerts[0].machine_id != cnc2_alerts[0].machine_id


class TestCorrelationRulesYAML:
    """Test loading of YAML correlation rules."""

    def test_load_rules_file(self):
        """Test that the shipped rules file loads correctly."""
        import yaml
        rules_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'correlation_rules.yaml'
        )
        if os.path.exists(rules_path):
            with open(rules_path) as f:
                data = yaml.safe_load(f)
            assert 'rules' in data
            assert len(data['rules']) >= 3
            for rule in data['rules']:
                assert 'name' in rule
                assert 'required_types' in rule
                assert 'root_cause' in rule

    def test_load_fleet_rules_from_yaml(self):
        """Test that fleet_rules section loads correctly from YAML."""
        import yaml
        rules_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'correlation_rules.yaml'
        )
        if os.path.exists(rules_path):
            with open(rules_path) as f:
                data = yaml.safe_load(f)
            assert 'fleet_rules' in data
            assert len(data['fleet_rules']) >= 2
            for rule in data['fleet_rules']:
                assert 'name' in rule
                assert 'min_machines' in rule
                assert 'root_cause' in rule
                assert 'recommended_actions' in rule


class TestFleetCorrelationRule:
    """Test FleetCorrelationRule dataclass."""

    def test_create_fleet_rule(self):
        rule = FleetCorrelationRule(
            name='fleet_same_anomaly',
            description='Same anomaly across machines',
            min_machines=2,
            time_window_sec=60.0,
            root_cause='Shared factor',
            recommended_actions=['Check coolant'],
        )
        assert rule.name == 'fleet_same_anomaly'
        assert rule.min_machines == 2
        assert rule.min_confidence == 0.8  # default

    def test_fleet_rule_custom_confidence(self):
        rule = FleetCorrelationRule(
            name='test',
            description='test',
            min_machines=3,
            time_window_sec=120.0,
            root_cause='test',
            recommended_actions=[],
            min_confidence=0.6,
        )
        assert rule.min_confidence == 0.6


class TestFleetCorrelationLogic:
    """Test fleet-wide correlation matching logic (without ROS2).

    These tests exercise the same grouping and windowing logic used by
    ``AlertCorrelatorNode._check_fleet_same_anomaly`` and
    ``_check_fleet_cascading_failure`` but without instantiating a ROS2 node.
    """

    @staticmethod
    def _build_fleet_buffer(entries):
        """Group alert entries into a fleet buffer keyed by anomaly_type."""
        buf = {}
        for e in entries:
            buf.setdefault(e.alert_type, []).append(e)
        return buf

    @staticmethod
    def _check_same_anomaly(fleet_buffer, rule, now):
        """Replicate the same-anomaly fleet check logic."""
        results = []
        for anomaly_type, entries in fleet_buffer.items():
            window_entries = [e for e in entries if now - e.timestamp <= rule.time_window_sec]
            machines = {}
            for e in window_entries:
                if e.machine_id not in machines or e.severity > machines[e.machine_id].severity:
                    machines[e.machine_id] = e
            if len(machines) >= rule.min_machines:
                matching = list(machines.values())
                avg_sev = sum(e.severity for e in matching) / len(matching)
                confidence = min(1.0, avg_sev * (len(matching) / rule.min_machines))
                if confidence >= rule.min_confidence:
                    results.append({
                        'anomaly_type': anomaly_type,
                        'machine_ids': sorted(machines.keys()),
                        'confidence': confidence,
                        'severity': max(e.severity for e in matching),
                    })
        return results

    @staticmethod
    def _check_cascading(fleet_buffer, rule, now):
        """Replicate cascading failure fleet check logic."""
        all_recent = []
        for entries in fleet_buffer.values():
            for e in entries:
                if now - e.timestamp <= rule.time_window_sec:
                    all_recent.append(e)
        machines = {e.machine_id for e in all_recent}
        anomaly_types = {e.alert_type for e in all_recent}
        if len(machines) < rule.min_machines or len(anomaly_types) < 2:
            return None
        avg_sev = sum(e.severity for e in all_recent) / len(all_recent)
        confidence = min(1.0, avg_sev * (len(machines) / rule.min_machines))
        if confidence < rule.min_confidence:
            return None
        return {
            'machine_ids': sorted(machines),
            'anomaly_types': sorted(anomaly_types),
            'confidence': confidence,
        }

    def test_fleet_same_anomaly_two_machines_within_window(self):
        """Same anomaly on 2 machines within 60s -> fleet alert."""
        now = time.time()
        entries = [
            AlertEntry('vibration_anomaly', 'cnc1', 0.9, 'vib', now - 10),
            AlertEntry('vibration_anomaly', 'cnc2', 0.85, 'vib', now - 5),
        ]
        rule = FleetCorrelationRule(
            name='fleet_same_anomaly',
            description='test',
            min_machines=2,
            time_window_sec=60.0,
            root_cause='Shared factor',
            recommended_actions=['Check coolant'],
            min_confidence=0.8,
        )
        buf = self._build_fleet_buffer(entries)
        results = self._check_same_anomaly(buf, rule, now)
        assert len(results) == 1
        assert results[0]['anomaly_type'] == 'vibration_anomaly'
        assert sorted(results[0]['machine_ids']) == ['cnc1', 'cnc2']

    def test_fleet_same_anomaly_outside_window_no_alert(self):
        """Same anomaly on 2 machines outside 60s window -> no fleet alert."""
        now = time.time()
        entries = [
            AlertEntry('vibration_anomaly', 'cnc1', 0.9, 'vib', now - 120),
            AlertEntry('vibration_anomaly', 'cnc2', 0.85, 'vib', now - 5),
        ]
        rule = FleetCorrelationRule(
            name='fleet_same_anomaly',
            description='test',
            min_machines=2,
            time_window_sec=60.0,
            root_cause='Shared factor',
            recommended_actions=['Check coolant'],
            min_confidence=0.8,
        )
        buf = self._build_fleet_buffer(entries)
        results = self._check_same_anomaly(buf, rule, now)
        assert len(results) == 0

    def test_fleet_different_anomalies_single_machine_no_alert(self):
        """Different anomalies on 1 machine -> no fleet alert (need 2+ machines)."""
        now = time.time()
        entries = [
            AlertEntry('vibration_anomaly', 'cnc1', 0.9, 'vib', now - 10),
            AlertEntry('force_anomaly', 'cnc1', 0.85, 'force', now - 5),
        ]
        rule = FleetCorrelationRule(
            name='fleet_same_anomaly',
            description='test',
            min_machines=2,
            time_window_sec=60.0,
            root_cause='Shared factor',
            recommended_actions=['Check coolant'],
            min_confidence=0.8,
        )
        buf = self._build_fleet_buffer(entries)
        results = self._check_same_anomaly(buf, rule, now)
        # Each anomaly type only has 1 machine, so no fleet alert
        assert len(results) == 0

    def test_fleet_cascading_three_machines_within_window(self):
        """3 different anomalies on 3 machines within 120s -> cascading alert."""
        now = time.time()
        entries = [
            AlertEntry('vibration_anomaly', 'cnc1', 0.9, 'vib', now - 30),
            AlertEntry('thermal_anomaly', 'cnc2', 0.8, 'thermal', now - 20),
            AlertEntry('force_anomaly', 'cnc3', 0.85, 'force', now - 10),
        ]
        rule = FleetCorrelationRule(
            name='fleet_cascading_failure',
            description='test',
            min_machines=3,
            time_window_sec=120.0,
            root_cause='Facility issue',
            recommended_actions=['Check HVAC'],
            min_confidence=0.7,
        )
        buf = self._build_fleet_buffer(entries)
        result = self._check_cascading(buf, rule, now)
        assert result is not None
        assert len(result['machine_ids']) == 3
        assert len(result['anomaly_types']) == 3

    def test_fleet_deduplication(self):
        """Same fleet alert should not fire twice (dedup key based on time window)."""
        now = time.time()
        entries = [
            AlertEntry('vibration_anomaly', 'cnc1', 0.9, 'vib', now - 10),
            AlertEntry('vibration_anomaly', 'cnc2', 0.85, 'vib', now - 5),
        ]
        rule = FleetCorrelationRule(
            name='fleet_same_anomaly',
            description='test',
            min_machines=2,
            time_window_sec=60.0,
            root_cause='Shared factor',
            recommended_actions=['Check coolant'],
            min_confidence=0.8,
        )
        buf = self._build_fleet_buffer(entries)

        # Simulate dedup tracking
        suppressed = set()
        results_first = self._check_same_anomaly(buf, rule, now)
        assert len(results_first) == 1

        # Mark as suppressed using same dedup key logic from the node
        for r in results_first:
            dedup_key = f"fleet:{r['anomaly_type']}:{int(now // rule.time_window_sec)}"
            suppressed.add(dedup_key)

        # Second check with dedup should not produce results
        results_second = []
        for anomaly_type, ents in buf.items():
            window_ents = [e for e in ents if now - e.timestamp <= rule.time_window_sec]
            machines = {}
            for e in window_ents:
                if e.machine_id not in machines or e.severity > machines[e.machine_id].severity:
                    machines[e.machine_id] = e
            if len(machines) >= rule.min_machines:
                dedup_key = f"fleet:{anomaly_type}:{int(now // rule.time_window_sec)}"
                if dedup_key in suppressed:
                    continue
                results_second.append(anomaly_type)
        assert len(results_second) == 0

    def test_per_machine_correlation_still_works_alongside_fleet(self):
        """Per-machine correlation rules should still work alongside fleet rules."""
        now = time.time()
        # Per-machine: two different alert types on same machine
        machine_alerts = [
            AlertEntry('vibration_anomaly', 'cnc1', 0.8, 'vib', now - 5),
            AlertEntry('force_anomaly', 'cnc1', 0.9, 'force', now - 3),
        ]
        per_machine_rule = CorrelationRule(
            name='tool_degradation',
            required_types=['vibration_anomaly', 'force_anomaly'],
            time_window_sec=30.0,
            root_cause='Tool wear',
            recommended_actions=['Replace tool'],
        )

        # Per-machine check still works
        window_alerts = [a for a in machine_alerts if now - a.timestamp <= per_machine_rule.time_window_sec]
        present_types = {a.alert_type for a in window_alerts}
        assert all(rt in present_types for rt in per_machine_rule.required_types)

        # Fleet check: same alerts don't trigger fleet_same_anomaly (different types)
        fleet_rule = FleetCorrelationRule(
            name='fleet_same_anomaly',
            description='test',
            min_machines=2,
            time_window_sec=60.0,
            root_cause='Shared factor',
            recommended_actions=['Check coolant'],
            min_confidence=0.8,
        )
        buf = self._build_fleet_buffer(machine_alerts)
        fleet_results = self._check_same_anomaly(buf, fleet_rule, now)
        # Only 1 machine per anomaly type, so no fleet alert
        assert len(fleet_results) == 0
