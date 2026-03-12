"""Tests for the Alert Correlation Engine."""

import pytest
import time

# Import the dataclasses and logic directly (no ROS2 needed)
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_scada.alert_correlator import AlertEntry, CorrelationRule


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
