"""Tests for RecoveryOrchestratorNode."""
import pytest
from miracle_resiliency.recovery_orchestrator import (
    RecoveryAction, RecoveryOrchestratorNode, topological_sort,
    DEFAULT_DEPENDENCIES,
)


class TestRecoveryAction:
    """Test RecoveryAction dataclass."""

    def test_default_status(self):
        action = RecoveryAction(failed_node='test', strategy='IMMEDIATE_RESTART')
        assert action.status == 'PENDING'
        assert action.attempt == 0
        assert action.max_retries == 3

    def test_custom_values(self):
        action = RecoveryAction(
            failed_node='node1', strategy='DELAYED_RESTART',
            attempt=2, max_retries=5, status='IN_PROGRESS'
        )
        assert action.failed_node == 'node1'
        assert action.attempt == 2
        assert action.max_retries == 5


class TestTopologicalSort:
    """Test dependency-ordered restart."""

    def test_no_dependencies(self):
        result = topological_sort('standalone_node', {})
        assert result == ['standalone_node']

    def test_single_dependency(self):
        deps = {'B': ['A']}
        result = topological_sort('B', deps)
        assert result.index('A') < result.index('B')

    def test_chain_dependency(self):
        deps = {'C': ['B'], 'B': ['A']}
        result = topological_sort('C', deps)
        assert result.index('A') < result.index('B')
        assert result.index('B') < result.index('C')

    def test_diamond_dependency(self):
        deps = {'D': ['B', 'C'], 'B': ['A'], 'C': ['A']}
        result = topological_sort('D', deps)
        assert result.index('A') < result.index('B')
        assert result.index('A') < result.index('C')
        assert result.index('B') < result.index('D')

    def test_default_deps_phm_predictor(self):
        result = topological_sort('phm_predictor', DEFAULT_DEPENDENCIES)
        # sensor_fusion and anomaly_detector must come before phm_predictor
        assert 'sensor_fusion' in result
        assert 'anomaly_detector' in result
        assert result.index('sensor_fusion') < result.index('phm_predictor')

    def test_prediction_runner_deps(self):
        result = topological_sort('prediction_runner', DEFAULT_DEPENDENCIES)
        assert result.index('phm_predictor') < result.index('prediction_runner')
        assert result.index('twin_sync') < result.index('prediction_runner')
