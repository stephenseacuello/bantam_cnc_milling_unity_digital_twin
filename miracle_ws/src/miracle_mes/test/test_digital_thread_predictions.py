"""Tests for Digital Thread prediction accuracy tracking and calibration logging.

Validates record_prediction, record_prediction_comparison,
record_calibration_event, get_prediction_accuracy_history,
get_calibration_history, and compute_model_accuracy_summary.
"""

import sys
import threading
import time
from unittest.mock import MagicMock

import pytest

# Mock ROS2 and miracle_core dependencies before importing the module
for mod in ['rclpy', 'rclpy.node', 'rclpy.callback_groups', 'rclpy.qos', 'rclpy.lifecycle',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
            'miracle_core.exceptions',
            'miracle_msgs', 'miracle_msgs.msg', 'miracle_msgs.srv']:
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
sys.modules.pop('miracle_mes.digital_thread', None)

from miracle_mes.digital_thread import DigitalThreadNode  # noqa: E402


@pytest.fixture
def dt():
    """Create a DigitalThreadNode instance for testing."""
    node = DigitalThreadNode()
    return node


class TestRecordPrediction:
    """Tests for record_prediction."""

    def test_creates_entry(self, dt):
        dt.record_prediction(
            machine_id='cnc1', program_name='prog1', block_index=0,
            predicted_force=100.0, predicted_temp=45.0,
            predicted_wear=0.1, predicted_rul_min=120.0,
        )
        assert len(dt._entries) == 1
        entry = dt._entries[0]
        assert entry['entry_type'] == 'PREDICTION_RECORDED'
        assert entry['machine_id'] == 'cnc1'
        assert entry['program_name'] == 'prog1'
        assert entry['block_index'] == 0
        assert entry['predicted_force'] == 100.0
        assert entry['predicted_temp'] == 45.0
        assert entry['predicted_wear'] == 0.1
        assert entry['predicted_rul_min'] == 120.0
        assert 'timestamp' in entry

    def test_prediction_with_anomaly_markers(self, dt):
        markers = ['chatter_onset', 'tool_wear_high']
        dt.record_prediction(
            machine_id='cnc1', program_name='prog1', block_index=5,
            predicted_force=150.0, predicted_temp=55.0,
            predicted_wear=0.3, predicted_rul_min=60.0,
            anomaly_markers=markers,
        )
        entry = dt._entries[0]
        assert entry['anomaly_markers'] == markers

    def test_prediction_default_anomaly_markers_empty(self, dt):
        dt.record_prediction(
            machine_id='cnc1', program_name='prog1', block_index=0,
            predicted_force=100.0, predicted_temp=45.0,
            predicted_wear=0.1, predicted_rul_min=120.0,
        )
        entry = dt._entries[0]
        assert entry['anomaly_markers'] == []

    def test_multiple_predictions_accumulate(self, dt):
        for i in range(5):
            dt.record_prediction(
                machine_id='cnc1', program_name='prog1', block_index=i,
                predicted_force=100.0 + i, predicted_temp=45.0,
                predicted_wear=0.1, predicted_rul_min=120.0,
            )
        predictions = [e for e in dt._entries if e['entry_type'] == 'PREDICTION_RECORDED']
        assert len(predictions) == 5


class TestRecordPredictionComparison:
    """Tests for record_prediction_comparison."""

    def test_creates_comparison_entry(self, dt):
        dt.record_prediction_comparison(
            machine_id='cnc1', program_name='prog1', block_index=0,
            predicted_force=100.0, actual_force=105.0,
            predicted_temp=45.0, actual_temp=46.0,
            force_error_pct=5.0, temp_error_pct=2.22,
        )
        assert len(dt._entries) == 1
        entry = dt._entries[0]
        assert entry['entry_type'] == 'PREDICTION_COMPARED'
        assert entry['predicted_force'] == 100.0
        assert entry['actual_force'] == 105.0
        assert entry['force_error_pct'] == 5.0
        assert entry['temp_error_pct'] == 2.22

    def test_comparison_with_zero_predicted_force(self, dt):
        """Edge case: predicted force is zero."""
        dt.record_prediction_comparison(
            machine_id='cnc1', program_name='prog1', block_index=0,
            predicted_force=0.0, actual_force=5.0,
            predicted_temp=0.0, actual_temp=2.0,
            force_error_pct=100.0, temp_error_pct=100.0,
        )
        entry = dt._entries[0]
        assert entry['predicted_force'] == 0.0
        assert entry['actual_force'] == 5.0
        assert entry['force_error_pct'] == 100.0

    def test_comparison_with_negative_error(self, dt):
        """Negative error percentage (over-prediction)."""
        dt.record_prediction_comparison(
            machine_id='cnc1', program_name='prog1', block_index=0,
            predicted_force=110.0, actual_force=100.0,
            predicted_temp=50.0, actual_temp=45.0,
            force_error_pct=-10.0, temp_error_pct=-11.11,
        )
        entry = dt._entries[0]
        assert entry['force_error_pct'] == -10.0


class TestRecordCalibrationEvent:
    """Tests for record_calibration_event."""

    def test_creates_calibration_entry(self, dt):
        adjustments = {'force_scale': 1.05, 'temp_offset': -0.5}
        dt.record_calibration_event(
            machine_id='cnc1', tool_id='T01',
            calibration_type='auto',
            adjustments=adjustments,
            reason='force drift detected',
            blocks_analyzed=50,
        )
        assert len(dt._entries) == 1
        entry = dt._entries[0]
        assert entry['entry_type'] == 'CALIBRATION_APPLIED'
        assert entry['machine_id'] == 'cnc1'
        assert entry['tool_id'] == 'T01'
        assert entry['calibration_type'] == 'auto'
        assert entry['reason'] == 'force drift detected'
        assert entry['blocks_analyzed'] == 50

    def test_calibration_includes_all_adjustments(self, dt):
        adjustments = {
            'force_scale': 1.05,
            'temp_offset': -0.5,
            'wear_factor': 0.98,
        }
        dt.record_calibration_event(
            machine_id='cnc1', tool_id='T01',
            calibration_type='manual',
            adjustments=adjustments,
            reason='operator override',
            blocks_analyzed=100,
        )
        entry = dt._entries[0]
        assert entry['adjustments'] == adjustments
        assert entry['adjustments']['force_scale'] == 1.05
        assert entry['adjustments']['temp_offset'] == -0.5
        assert entry['adjustments']['wear_factor'] == 0.98


class TestGetPredictionAccuracyHistory:
    """Tests for get_prediction_accuracy_history."""

    def _add_comparisons(self, dt, machine_id, program_name, count, force_err=2.0, temp_err=1.0):
        for i in range(count):
            dt.record_prediction_comparison(
                machine_id=machine_id, program_name=program_name, block_index=i,
                predicted_force=100.0, actual_force=100.0 + force_err,
                predicted_temp=45.0, actual_temp=45.0 + temp_err,
                force_error_pct=force_err, temp_error_pct=temp_err,
            )

    def test_returns_comparisons(self, dt):
        self._add_comparisons(dt, 'cnc1', 'prog1', 3)
        history = dt.get_prediction_accuracy_history()
        assert len(history) == 3
        assert all(h['entry_type'] == 'PREDICTION_COMPARED' for h in history)

    def test_filter_by_machine(self, dt):
        self._add_comparisons(dt, 'cnc1', 'prog1', 3)
        self._add_comparisons(dt, 'cnc2', 'prog1', 2)
        history = dt.get_prediction_accuracy_history(machine_id='cnc1')
        assert len(history) == 3

    def test_filter_by_program(self, dt):
        self._add_comparisons(dt, 'cnc1', 'prog1', 3)
        self._add_comparisons(dt, 'cnc1', 'prog2', 4)
        history = dt.get_prediction_accuracy_history(program_name='prog2')
        assert len(history) == 4

    def test_filter_by_machine_and_program(self, dt):
        self._add_comparisons(dt, 'cnc1', 'prog1', 3)
        self._add_comparisons(dt, 'cnc1', 'prog2', 2)
        self._add_comparisons(dt, 'cnc2', 'prog1', 4)
        history = dt.get_prediction_accuracy_history(machine_id='cnc1', program_name='prog1')
        assert len(history) == 3

    def test_last_n_limit(self, dt):
        self._add_comparisons(dt, 'cnc1', 'prog1', 20)
        history = dt.get_prediction_accuracy_history(last_n=5)
        assert len(history) == 5

    def test_empty_history(self, dt):
        history = dt.get_prediction_accuracy_history()
        assert history == []


class TestGetCalibrationHistory:
    """Tests for get_calibration_history."""

    def test_returns_calibration_events(self, dt):
        dt.record_calibration_event('cnc1', 'T01', 'auto', {'force_scale': 1.0}, 'test', 10)
        dt.record_calibration_event('cnc1', 'T02', 'manual', {'temp_offset': 0.1}, 'test2', 20)
        history = dt.get_calibration_history()
        assert len(history) == 2

    def test_filter_by_machine(self, dt):
        dt.record_calibration_event('cnc1', 'T01', 'auto', {}, 'r1', 10)
        dt.record_calibration_event('cnc2', 'T01', 'auto', {}, 'r2', 10)
        history = dt.get_calibration_history(machine_id='cnc1')
        assert len(history) == 1

    def test_filter_by_tool(self, dt):
        dt.record_calibration_event('cnc1', 'T01', 'auto', {}, 'r1', 10)
        dt.record_calibration_event('cnc1', 'T02', 'auto', {}, 'r2', 10)
        history = dt.get_calibration_history(tool_id='T02')
        assert len(history) == 1
        assert history[0]['tool_id'] == 'T02'

    def test_empty_calibration_history(self, dt):
        history = dt.get_calibration_history()
        assert history == []


class TestComputeModelAccuracySummary:
    """Tests for compute_model_accuracy_summary."""

    def test_empty_returns_defaults(self, dt):
        summary = dt.compute_model_accuracy_summary('cnc1')
        assert summary['total_comparisons'] == 0
        assert summary['mean_force_error_pct'] == 0.0
        assert summary['mean_temp_error_pct'] == 0.0
        assert summary['force_r_squared'] == 0.0
        assert summary['calibrations_applied'] == 0
        assert summary['accuracy_trend'] == 'stable'

    def test_good_predictions(self, dt):
        """Nearly perfect predictions should yield high R-squared."""
        for i in range(20):
            force = 100.0 + i * 2
            temp = 40.0 + i * 0.5
            dt.record_prediction_comparison(
                machine_id='cnc1', program_name='prog1', block_index=i,
                predicted_force=force, actual_force=force + 0.1,
                predicted_temp=temp, actual_temp=temp + 0.05,
                force_error_pct=0.1, temp_error_pct=0.125,
            )
        summary = dt.compute_model_accuracy_summary('cnc1')
        assert summary['total_comparisons'] == 20
        assert summary['mean_force_error_pct'] < 1.0
        assert summary['mean_temp_error_pct'] < 1.0
        assert summary['force_r_squared'] > 0.99
        assert summary['accuracy_trend'] == 'stable'

    def test_drifting_predictions(self, dt):
        """Predictions that degrade over time should show 'degrading' trend."""
        for i in range(20):
            # First 10: low error; last 10: high error
            err = 1.0 if i < 10 else 20.0
            dt.record_prediction_comparison(
                machine_id='cnc1', program_name='prog1', block_index=i,
                predicted_force=100.0, actual_force=100.0 + err,
                predicted_temp=45.0, actual_temp=45.0 + err * 0.5,
                force_error_pct=err, temp_error_pct=err * 0.5,
            )
        summary = dt.compute_model_accuracy_summary('cnc1')
        assert summary['accuracy_trend'] == 'degrading'

    def test_improving_predictions(self, dt):
        """Predictions that improve over time should show 'improving' trend."""
        for i in range(20):
            # First 10: high error; last 10: low error
            err = 20.0 if i < 10 else 1.0
            dt.record_prediction_comparison(
                machine_id='cnc1', program_name='prog1', block_index=i,
                predicted_force=100.0, actual_force=100.0 + err,
                predicted_temp=45.0, actual_temp=45.0 + err * 0.5,
                force_error_pct=err, temp_error_pct=err * 0.5,
            )
        summary = dt.compute_model_accuracy_summary('cnc1')
        assert summary['accuracy_trend'] == 'improving'

    def test_calibrations_counted(self, dt):
        """Calibrations applied count is included in summary."""
        dt.record_calibration_event('cnc1', 'T01', 'auto', {}, 'drift', 10)
        dt.record_calibration_event('cnc1', 'T02', 'auto', {}, 'drift', 20)
        dt.record_calibration_event('cnc2', 'T01', 'auto', {}, 'drift', 10)
        summary = dt.compute_model_accuracy_summary('cnc1')
        assert summary['calibrations_applied'] == 2

    def test_accuracy_trend_stable_with_few_points(self, dt):
        """Fewer than 6 comparison points should always be 'stable'."""
        for i in range(4):
            dt.record_prediction_comparison(
                machine_id='cnc1', program_name='prog1', block_index=i,
                predicted_force=100.0, actual_force=200.0,
                predicted_temp=45.0, actual_temp=90.0,
                force_error_pct=100.0, temp_error_pct=100.0,
            )
        summary = dt.compute_model_accuracy_summary('cnc1')
        assert summary['accuracy_trend'] == 'stable'


class TestEntryTypeConstants:
    """Verify the new entry type constants exist."""

    def test_prediction_recorded_constant(self):
        assert DigitalThreadNode.PREDICTION_RECORDED == 'PREDICTION_RECORDED'

    def test_prediction_compared_constant(self):
        assert DigitalThreadNode.PREDICTION_COMPARED == 'PREDICTION_COMPARED'

    def test_calibration_applied_constant(self):
        assert DigitalThreadNode.CALIBRATION_APPLIED == 'CALIBRATION_APPLIED'

    def test_calibration_reverted_constant(self):
        assert DigitalThreadNode.CALIBRATION_REVERTED == 'CALIBRATION_REVERTED'


class TestHashChainIntegrity:
    """Verify that prediction and calibration entries are part of the hash chain."""

    def test_prediction_entries_have_hash(self, dt):
        dt.record_prediction(
            machine_id='cnc1', program_name='prog1', block_index=0,
            predicted_force=100.0, predicted_temp=45.0,
            predicted_wear=0.1, predicted_rul_min=120.0,
        )
        entry = dt._entries[0]
        assert 'hash' in entry
        assert 'previous_hash' in entry
        assert len(entry['hash']) == 64

    def test_calibration_entries_have_hash(self, dt):
        dt.record_calibration_event('cnc1', 'T01', 'auto', {}, 'test', 10)
        entry = dt._entries[0]
        assert 'hash' in entry
        assert len(entry['hash']) == 64

    def test_chain_integrity_with_mixed_entries(self, dt):
        dt.record_prediction(
            machine_id='cnc1', program_name='prog1', block_index=0,
            predicted_force=100.0, predicted_temp=45.0,
            predicted_wear=0.1, predicted_rul_min=120.0,
        )
        dt.record_prediction_comparison(
            machine_id='cnc1', program_name='prog1', block_index=0,
            predicted_force=100.0, actual_force=102.0,
            predicted_temp=45.0, actual_temp=46.0,
            force_error_pct=2.0, temp_error_pct=2.22,
        )
        dt.record_calibration_event('cnc1', 'T01', 'auto', {'force_scale': 1.02}, 'drift', 10)
        assert dt.verify_genealogy_integrity()
