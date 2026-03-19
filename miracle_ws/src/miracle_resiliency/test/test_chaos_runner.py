"""Tests for ChaosEngineeringRunner.

Verifies experiment lifecycle (create, run, abort), hypothesis validation,
pre-defined templates, and edge-case handling.
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
    ChaosEngineeringRunner,
    ChaosExperiment,
    ExperimentResult,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def runner():
    return ChaosEngineeringRunner()


@pytest.fixture
def basic_experiment(runner):
    """Create a simple latency experiment and return (runner, experiment)."""
    exp = runner.create_experiment(
        name='Test Latency',
        target='spindle_controller',
        fault_type='latency',
        magnitude=0.5,
        duration=5.0,
        hypothesis='RPM stays within tolerance',
    )
    return runner, exp


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestCreateExperiment:
    def test_create_returns_pending_experiment(self, runner):
        exp = runner.create_experiment(
            name='Latency Check',
            target='spindle_controller',
            fault_type='latency',
            magnitude=0.4,
            duration=10.0,
            hypothesis='RPM deviation < 5%',
        )
        assert isinstance(exp, ChaosExperiment)
        assert exp.status == 'pending'
        assert exp.name == 'Latency Check'
        assert exp.target_component == 'spindle_controller'
        assert exp.fault_type == 'latency'
        assert exp.magnitude == 0.4
        assert exp.duration_sec == 10.0

    def test_create_rejects_invalid_fault_type(self, runner):
        with pytest.raises(ValueError, match='Invalid fault_type'):
            runner.create_experiment(
                name='Bad',
                target='x',
                fault_type='fire',
                magnitude=1.0,
                duration=1.0,
                hypothesis='n/a',
            )


class TestRunExperiment:
    def test_run_completes_with_no_checker(self, basic_experiment):
        runner, exp = basic_experiment
        result = runner.run_experiment(exp.experiment_id)
        assert isinstance(result, ExperimentResult)
        assert result.experiment_id == exp.experiment_id
        assert result.hypothesis_validated is True  # empty dicts -> within tolerance
        assert result.duration_sec >= 0
        assert result.steady_state_before == {}
        assert result.steady_state_after == {}

    def test_run_with_steady_state_checker(self, runner):
        exp = runner.create_experiment(
            name='Checker Test',
            target='sensor_hub',
            fault_type='shutdown',
            magnitude=1.0,
            duration=1.0,
            hypothesis='Sensor fallback works',
        )
        call_count = 0

        def checker():
            nonlocal call_count
            call_count += 1
            return {'rpm': 3000, 'temp': 55.0}

        result = runner.run_experiment(exp.experiment_id, steady_state_checker=checker)
        assert call_count == 2  # before + after
        assert result.steady_state_before == {'rpm': 3000, 'temp': 55.0}
        assert result.hypothesis_validated is True

    def test_run_already_completed_raises(self, basic_experiment):
        runner, exp = basic_experiment
        runner.run_experiment(exp.experiment_id)
        with pytest.raises(RuntimeError, match='cannot be run'):
            runner.run_experiment(exp.experiment_id)


class TestAbortExperiment:
    def test_abort_pending_experiment(self, basic_experiment):
        runner, exp = basic_experiment
        aborted = runner.abort_experiment(exp.experiment_id)
        assert aborted.status == 'aborted'
        result = runner.get_results(exp.experiment_id)
        assert result.rollback_performed is True
        assert result.hypothesis_validated is False

    def test_abort_already_aborted_raises(self, basic_experiment):
        runner, exp = basic_experiment
        runner.abort_experiment(exp.experiment_id)
        with pytest.raises(RuntimeError, match='Cannot abort'):
            runner.abort_experiment(exp.experiment_id)


class TestGetResults:
    def test_get_results_after_run(self, basic_experiment):
        runner, exp = basic_experiment
        runner.run_experiment(exp.experiment_id)
        result = runner.get_results(exp.experiment_id)
        assert result.experiment_id == exp.experiment_id

    def test_get_results_missing_raises(self, runner):
        with pytest.raises(KeyError, match='No results'):
            runner.get_results('nonexistent-id')


class TestExperimentHistory:
    def test_history_records_completed_experiments(self, runner):
        e1 = runner.create_experiment(
            name='E1', target='a', fault_type='error',
            magnitude=0.5, duration=1.0, hypothesis='h1',
        )
        e2 = runner.create_experiment(
            name='E2', target='b', fault_type='latency',
            magnitude=0.3, duration=2.0, hypothesis='h2',
        )
        runner.run_experiment(e1.experiment_id)
        runner.run_experiment(e2.experiment_id)

        history = runner.get_experiment_history()
        assert len(history) == 2
        assert history[0].name == 'E1'
        assert history[1].name == 'E2'
        assert all(e.status == 'completed' for e in history)


class TestValidateHypothesis:
    def test_within_tolerance(self):
        before = {'rpm': 3000, 'temp': 60.0}
        after = {'rpm': 3090, 'temp': 63.0}  # 3% and 5% change
        assert ChaosEngineeringRunner.validate_hypothesis(before, after, tolerance_pct=10.0)

    def test_exceeds_tolerance(self):
        before = {'rpm': 3000, 'temp': 60.0}
        after = {'rpm': 3600, 'temp': 60.0}  # 20% change in RPM
        assert not ChaosEngineeringRunner.validate_hypothesis(before, after, tolerance_pct=10.0)

    def test_empty_dicts_pass(self):
        assert ChaosEngineeringRunner.validate_hypothesis({}, {})

    def test_non_numeric_keys_ignored(self):
        before = {'label': 'alpha', 'rpm': 100}
        after = {'label': 'beta', 'rpm': 105}
        assert ChaosEngineeringRunner.validate_hypothesis(before, after, tolerance_pct=10.0)

    def test_zero_before_value(self):
        # If before is zero and after is non-zero, hypothesis fails
        assert not ChaosEngineeringRunner.validate_hypothesis(
            {'x': 0}, {'x': 5}, tolerance_pct=10.0,
        )
        # Both zero is fine
        assert ChaosEngineeringRunner.validate_hypothesis(
            {'x': 0}, {'x': 0}, tolerance_pct=10.0,
        )


class TestPredefinedExperiments:
    @pytest.mark.parametrize('template_name', [
        'SPINDLE_LATENCY', 'SENSOR_DROPOUT', 'DATABASE_SLOW', 'NETWORK_JITTER',
    ])
    def test_predefined_templates_create_valid_experiments(self, runner, template_name):
        template = getattr(ChaosEngineeringRunner, template_name)
        exp = runner.create_experiment(**template)
        assert exp.status == 'pending'
        assert exp.name == template['name']
        result = runner.run_experiment(exp.experiment_id)
        assert result.experiment_id == exp.experiment_id


class TestChaosExperimentDataclass:
    def test_invalid_status_raises(self):
        with pytest.raises(ValueError, match='Invalid status'):
            ChaosExperiment(
                experiment_id='x', name='X', target_component='t',
                fault_type='latency', magnitude=1.0, duration_sec=1.0,
                steady_state_hypothesis='h', status='bogus',
            )
