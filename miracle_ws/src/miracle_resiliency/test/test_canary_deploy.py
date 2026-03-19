"""Tests for CanaryDeploymentManager, DeploymentConfig, and DeploymentStatus."""
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
    CanaryDeploymentManager,
    DeploymentConfig,
    DeploymentStatus,
)


@pytest.fixture
def manager():
    """Return a fresh CanaryDeploymentManager."""
    return CanaryDeploymentManager()


@pytest.fixture
def sample_config():
    """A deployment config with relaxed thresholds for easy testing."""
    return DeploymentConfig(
        deployment_id='deploy-1',
        name='firmware-update',
        target_version='2.0.0',
        canary_pct=20.0,
        max_error_rate=10.0,
        min_success_count=3,
        rollback_on_failure=True,
    )


@pytest.fixture
def nodes():
    return [f'node-{i}' for i in range(10)]


# -------------------------------------------------------------------
# Tests
# -------------------------------------------------------------------

class TestDeploymentConfigDefaults:
    """Verify DeploymentConfig default values."""

    def test_defaults(self):
        cfg = DeploymentConfig(
            deployment_id='d1', name='test', target_version='1.0',
        )
        assert cfg.canary_pct == 10.0
        assert cfg.max_error_rate == 5.0
        assert cfg.min_success_count == 10
        assert cfg.rollback_on_failure is True


class TestStartDeployment:
    """start_deployment selects canary nodes correctly."""

    def test_canary_node_selection(self, manager, sample_config, nodes):
        status = manager.start_deployment(sample_config, nodes)

        assert status.phase == 'canary'
        # 20% of 10 nodes = 2 canary nodes
        assert len(status.canary_nodes) == 2
        assert len(status.stable_nodes) == 8
        assert status.error_count == 0
        assert status.success_count == 0
        assert status.current_pct == 20.0

    def test_duplicate_deployment_raises(self, manager, sample_config, nodes):
        manager.start_deployment(sample_config, nodes)
        with pytest.raises(ValueError, match='already exists'):
            manager.start_deployment(sample_config, nodes)

    def test_minimum_one_canary_node(self, manager):
        """Even with a tiny canary_pct, at least one node is selected."""
        cfg = DeploymentConfig(
            deployment_id='d-small', name='tiny', target_version='1.0',
            canary_pct=1.0,
        )
        status = manager.start_deployment(cfg, ['n1', 'n2', 'n3'])
        assert len(status.canary_nodes) >= 1


class TestRecordResult:
    """record_result tracks successes and errors."""

    def test_success_increments(self, manager, sample_config, nodes):
        manager.start_deployment(sample_config, nodes)
        manager.record_result('deploy-1', 'node-0', success=True)
        status = manager.get_status('deploy-1')
        assert status.success_count == 1
        assert status.error_count == 0

    def test_error_increments(self, manager, sample_config, nodes):
        manager.start_deployment(sample_config, nodes)
        manager.record_result('deploy-1', 'node-0', success=False, error_msg='timeout')
        status = manager.get_status('deploy-1')
        assert status.error_count == 1

    def test_auto_rollback_on_high_error_rate(self, manager, nodes):
        """When error rate exceeds threshold after min results, auto-rollback."""
        cfg = DeploymentConfig(
            deployment_id='deploy-auto',
            name='auto-rb',
            target_version='3.0',
            canary_pct=50.0,
            max_error_rate=10.0,
            min_success_count=5,
            rollback_on_failure=True,
        )
        manager.start_deployment(cfg, nodes)

        # Record 3 successes + 1 error -> total=4, still < min_success_count=5
        for _ in range(3):
            manager.record_result('deploy-auto', 'node-0', True)
        manager.record_result('deploy-auto', 'node-1', False, 'fail')

        status = manager.get_status('deploy-auto')
        assert status.phase == 'canary'  # not rolled back yet (total < min_success_count)

        # One more error: total=5 >= 5, error_rate=2/5=40% > 10% -> rollback
        manager.record_result('deploy-auto', 'node-2', False, 'fail2')
        status = manager.get_status('deploy-auto')
        assert status.phase == 'rolled_back'


class TestEvaluateCanary:
    """evaluate_canary checks health criteria."""

    def test_healthy_canary(self, manager, sample_config, nodes):
        manager.start_deployment(sample_config, nodes)
        # Record enough successes to meet min_success_count=3
        for i in range(4):
            manager.record_result('deploy-1', f'node-{i}', True)

        assert manager.evaluate_canary('deploy-1') is True

    def test_unhealthy_not_enough_successes(self, manager, sample_config, nodes):
        manager.start_deployment(sample_config, nodes)
        manager.record_result('deploy-1', 'node-0', True)
        # Only 1 success; need 3
        assert manager.evaluate_canary('deploy-1') is False

    def test_unhealthy_high_error_rate(self, manager, nodes):
        cfg = DeploymentConfig(
            deployment_id='d-err', name='err-test', target_version='1.0',
            max_error_rate=5.0, min_success_count=2,
            rollback_on_failure=False,
        )
        manager.start_deployment(cfg, nodes)
        # 2 successes, 2 errors -> 50% error rate
        manager.record_result('d-err', 'n0', True)
        manager.record_result('d-err', 'n1', True)
        manager.record_result('d-err', 'n2', False, 'e1')
        manager.record_result('d-err', 'n3', False, 'e2')
        assert manager.evaluate_canary('d-err') is False


class TestPromote:
    """promote moves all nodes to the new version."""

    def test_promote_completes(self, manager, sample_config, nodes):
        manager.start_deployment(sample_config, nodes)
        status = manager.promote('deploy-1')
        assert status.phase == 'complete'
        assert status.current_pct == 100.0
        assert len(status.stable_nodes) == 0
        assert len(status.canary_nodes) == 10

    def test_promote_after_rollback_raises(self, manager, sample_config, nodes):
        manager.start_deployment(sample_config, nodes)
        manager.rollback('deploy-1')
        with pytest.raises(ValueError, match='Cannot promote'):
            manager.promote('deploy-1')


class TestRollback:
    """rollback reverts the deployment."""

    def test_rollback_moves_to_rolled_back(self, manager, sample_config, nodes):
        manager.start_deployment(sample_config, nodes)
        status = manager.rollback('deploy-1')
        assert status.phase == 'rolled_back'
        assert status.current_pct == 0.0
        assert len(status.canary_nodes) == 0
        # All nodes back on stable
        assert len(status.stable_nodes) == 10


class TestGetStatus:
    """get_status returns a copy of the current status."""

    def test_unknown_deployment_raises(self, manager):
        with pytest.raises(KeyError, match='No deployment'):
            manager.get_status('nonexistent')


class TestDeploymentHistory:
    """get_deployment_history tracks completed/rolled-back deployments."""

    def test_history_after_promote(self, manager, sample_config, nodes):
        manager.start_deployment(sample_config, nodes)
        manager.promote('deploy-1')
        history = manager.get_deployment_history()
        assert len(history) == 1
        assert history[0].phase == 'complete'

    def test_history_after_rollback(self, manager, sample_config, nodes):
        manager.start_deployment(sample_config, nodes)
        manager.rollback('deploy-1')
        history = manager.get_deployment_history()
        assert len(history) == 1
        assert history[0].phase == 'rolled_back'

    def test_multiple_deployments_in_history(self, manager, nodes):
        for i in range(3):
            cfg = DeploymentConfig(
                deployment_id=f'd-{i}', name=f'deploy-{i}',
                target_version=f'{i}.0',
            )
            manager.start_deployment(cfg, nodes)
            manager.promote(f'd-{i}')

        history = manager.get_deployment_history()
        assert len(history) == 3
