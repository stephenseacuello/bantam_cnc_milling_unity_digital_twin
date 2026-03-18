"""Tests for ConfigVersionManager, ConfigVersion, and ConfigDiff."""
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
    ConfigVersion,
    ConfigDiff,
    ConfigVersionManager,
)


@pytest.fixture
def manager():
    """Return a fresh ConfigVersionManager."""
    return ConfigVersionManager()


@pytest.fixture
def sample_config():
    return {'spindle_speed': 12000, 'feed_rate': 500, 'coolant': True}


class TestConfigVersion:
    """Test ConfigVersion dataclass."""

    def test_defaults(self):
        cv = ConfigVersion(
            version_id='v1', node_id='n1', config_data={},
            timestamp=1.0, author='alice', description='init',
        )
        assert cv.is_active is False

    def test_active_flag(self):
        cv = ConfigVersion(
            version_id='v2', node_id='n1', config_data={'a': 1},
            timestamp=2.0, author='bob', description='update',
            is_active=True,
        )
        assert cv.is_active is True


class TestSaveVersion:
    """Test saving new configuration versions."""

    def test_save_returns_config_version(self, manager, sample_config):
        v = manager.save_version('spindle', sample_config, 'alice', 'initial')
        assert isinstance(v, ConfigVersion)
        assert v.node_id == 'spindle'
        assert v.author == 'alice'
        assert v.description == 'initial'
        assert v.is_active is False
        assert v.config_data == sample_config

    def test_save_generates_unique_ids(self, manager, sample_config):
        v1 = manager.save_version('spindle', sample_config, 'a', 'd1')
        v2 = manager.save_version('spindle', sample_config, 'a', 'd2')
        assert v1.version_id != v2.version_id

    def test_save_deep_copies_config(self, manager):
        cfg = {'key': [1, 2, 3]}
        v = manager.save_version('node', cfg, 'a', 'd')
        cfg['key'].append(4)
        assert v.config_data == {'key': [1, 2, 3]}


class TestActivateVersion:
    """Test activating and deactivating versions."""

    def test_activate_sets_active(self, manager, sample_config):
        v = manager.save_version('n1', sample_config, 'a', 'd')
        result = manager.activate_version('n1', v.version_id)
        assert result.is_active is True
        assert manager.get_active_config('n1') is result

    def test_activate_deactivates_previous(self, manager, sample_config):
        v1 = manager.save_version('n1', sample_config, 'a', 'd1')
        v2 = manager.save_version('n1', sample_config, 'a', 'd2')
        manager.activate_version('n1', v1.version_id)
        manager.activate_version('n1', v2.version_id)
        assert v1.is_active is False
        assert v2.is_active is True

    def test_activate_unknown_node_raises(self, manager):
        with pytest.raises(KeyError):
            manager.activate_version('ghost', 'no-such-id')

    def test_activate_unknown_version_raises(self, manager, sample_config):
        manager.save_version('n1', sample_config, 'a', 'd')
        with pytest.raises(KeyError):
            manager.activate_version('n1', 'bad-id')


class TestGetActiveConfig:
    """Test retrieving the active configuration."""

    def test_no_active_returns_none(self, manager, sample_config):
        manager.save_version('n1', sample_config, 'a', 'd')
        assert manager.get_active_config('n1') is None

    def test_returns_active(self, manager, sample_config):
        v = manager.save_version('n1', sample_config, 'a', 'd')
        manager.activate_version('n1', v.version_id)
        active = manager.get_active_config('n1')
        assert active is not None
        assert active.version_id == v.version_id

    def test_unknown_node_returns_none(self, manager):
        assert manager.get_active_config('nonexistent') is None


class TestVersionHistory:
    """Test version history retrieval."""

    def test_history_sorted_by_timestamp(self, manager):
        import time as _time
        v1 = manager.save_version('n1', {'a': 1}, 'a', 'first')
        _time.sleep(0.01)
        v2 = manager.save_version('n1', {'a': 2}, 'a', 'second')
        history = manager.get_version_history('n1')
        assert len(history) == 2
        assert history[0].version_id == v1.version_id
        assert history[1].version_id == v2.version_id

    def test_empty_history(self, manager):
        assert manager.get_version_history('nobody') == []


class TestDiffVersions:
    """Test diffing two configuration versions."""

    def test_diff_detects_added_removed_changed(self, manager):
        v1 = manager.save_version('n', {'a': 1, 'b': 2}, 'a', 'd1')
        v2 = manager.save_version('n', {'b': 99, 'c': 3}, 'a', 'd2')
        diff = manager.diff_versions(v1.version_id, v2.version_id)
        assert isinstance(diff, ConfigDiff)
        assert diff.added_keys == ['c']
        assert diff.removed_keys == ['a']
        assert diff.changed_keys == ['b']
        assert diff.changes['b'] == (2, 99)

    def test_diff_identical_configs(self, manager):
        cfg = {'x': 10}
        v1 = manager.save_version('n', cfg, 'a', 'd1')
        v2 = manager.save_version('n', cfg, 'a', 'd2')
        diff = manager.diff_versions(v1.version_id, v2.version_id)
        assert diff.added_keys == []
        assert diff.removed_keys == []
        assert diff.changed_keys == []
        assert diff.changes == {}

    def test_diff_unknown_version_raises(self, manager):
        v = manager.save_version('n', {}, 'a', 'd')
        with pytest.raises(KeyError):
            manager.diff_versions(v.version_id, 'nonexistent')


class TestRollback:
    """Test rollback to a previous version."""

    def test_rollback_activates_old_version(self, manager, sample_config):
        v1 = manager.save_version('n1', sample_config, 'a', 'v1')
        v2 = manager.save_version('n1', {'override': True}, 'a', 'v2')
        manager.activate_version('n1', v2.version_id)
        assert manager.get_active_config('n1').version_id == v2.version_id

        manager.rollback('n1', v1.version_id)
        active = manager.get_active_config('n1')
        assert active.version_id == v1.version_id
        assert v2.is_active is False


class TestGetAllNodes:
    """Test listing all managed nodes."""

    def test_returns_sorted_nodes(self, manager):
        manager.save_version('zeta', {}, 'a', 'd')
        manager.save_version('alpha', {}, 'a', 'd')
        manager.save_version('mu', {}, 'a', 'd')
        assert manager.get_all_nodes() == ['alpha', 'mu', 'zeta']

    def test_empty_when_no_configs(self, manager):
        assert manager.get_all_nodes() == []


class TestExportImport:
    """Test export and import of configuration versions."""

    def test_round_trip(self, manager, sample_config):
        v = manager.save_version('n1', sample_config, 'alice', 'initial')
        manager.activate_version('n1', v.version_id)
        exported = manager.export_config('n1')

        # Import into a fresh manager
        mgr2 = ConfigVersionManager()
        imported = mgr2.import_config('n1', exported)
        assert len(imported) == 1
        assert imported[0].version_id == v.version_id
        assert imported[0].config_data == sample_config
        assert imported[0].is_active is True
        assert mgr2.get_active_config('n1') is not None

    def test_export_unknown_node_raises(self, manager):
        with pytest.raises(KeyError):
            manager.export_config('ghost')

    def test_export_is_json_serializable(self, manager, sample_config):
        manager.save_version('n1', sample_config, 'a', 'd')
        exported = manager.export_config('n1')
        import json
        serialized = json.dumps(exported)
        assert isinstance(serialized, str)

    def test_import_replaces_existing(self, manager):
        manager.save_version('n1', {'old': True}, 'a', 'old')
        data = {
            'versions': [
                {
                    'version_id': 'imported-v1',
                    'node_id': 'n1',
                    'config_data': {'new': True},
                    'timestamp': 100.0,
                    'author': 'importer',
                    'description': 'imported version',
                    'is_active': False,
                },
            ],
        }
        imported = manager.import_config('n1', data)
        assert len(imported) == 1
        history = manager.get_version_history('n1')
        assert len(history) == 1
        assert history[0].config_data == {'new': True}
