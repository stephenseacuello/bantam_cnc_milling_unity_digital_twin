"""Tests for DataRetentionManager, RetentionPolicy, RetentionRecord, and RetentionAction.

Covers: default policies, custom policies, record registration, evaluation,
storage summary, action application, edge cases.
"""

import sys
import time
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants',
            'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_msgs', 'miracle_msgs.msg',
            'std_msgs', 'std_msgs.msg',
            'cryptography', 'cryptography.fernet']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_security.secure_storage import (
    DataRetentionManager,
    RetentionAction,
    RetentionPolicy,
    RetentionRecord,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

SECONDS_PER_DAY = 86400


def _make_record(
    record_id: str,
    category: str = "SENSOR_DATA",
    age_days: int = 0,
    size_bytes: int = 1024,
    current_time: float = 1_000_000_000.0,
) -> RetentionRecord:
    """Create a RetentionRecord whose created_at is *age_days* before *current_time*."""
    return RetentionRecord(
        record_id=record_id,
        category=category,
        created_at=current_time - age_days * SECONDS_PER_DAY,
        size_bytes=size_bytes,
    )


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def manager():
    return DataRetentionManager()


# ===================================================================
# Default policies
# ===================================================================

class TestDefaultPolicies:
    """Verify the five built-in default policies are loaded correctly."""

    def test_default_policies_loaded(self, manager):
        expected = {"AUDIT_LOG", "SENSOR_DATA", "ALARM_HISTORY",
                    "JOB_RECORDS", "CALIBRATION"}
        for cat in expected:
            assert manager.get_policy(cat) is not None

    def test_audit_log_retention(self, manager):
        policy = manager.get_policy("AUDIT_LOG")
        assert policy.retention_days == 2555
        assert policy.requires_approval is True
        assert policy.delete_after_days == 0  # never delete

    def test_sensor_data_thresholds(self, manager):
        policy = manager.get_policy("SENSOR_DATA")
        assert policy.archive_after_days == 90
        assert policy.delete_after_days == 365

    def test_calibration_ten_year_retention(self, manager):
        policy = manager.get_policy("CALIBRATION")
        assert policy.retention_days == 3650


# ===================================================================
# add_policy / get_policy
# ===================================================================

class TestPolicyManagement:
    """Tests for adding and retrieving policies."""

    def test_add_custom_policy(self, manager):
        custom = RetentionPolicy(
            category="CUSTOM_DATA",
            retention_days=100,
            archive_after_days=30,
            delete_after_days=100,
            description="test custom policy",
        )
        manager.add_policy(custom)
        fetched = manager.get_policy("CUSTOM_DATA")
        assert fetched is not None
        assert fetched.retention_days == 100
        assert fetched.archive_after_days == 30
        assert fetched.description == "test custom policy"

    def test_override_existing_policy(self, manager):
        new_sensor = RetentionPolicy(
            category="SENSOR_DATA",
            retention_days=999,
            archive_after_days=10,
            delete_after_days=500,
        )
        manager.add_policy(new_sensor)
        assert manager.get_policy("SENSOR_DATA").retention_days == 999

    def test_get_nonexistent_policy_returns_none(self, manager):
        assert manager.get_policy("DOES_NOT_EXIST") is None


# ===================================================================
# register_record
# ===================================================================

class TestRegisterRecord:
    """Tests for record registration."""

    def test_register_single_record(self, manager):
        record = _make_record("r1")
        manager.register_record(record)
        summary = manager.get_storage_summary()
        assert summary["total_records"] == 1

    def test_register_multiple_categories(self, manager):
        manager.register_record(_make_record("r1", category="SENSOR_DATA"))
        manager.register_record(_make_record("r2", category="AUDIT_LOG"))
        manager.register_record(_make_record("r3", category="CALIBRATION"))
        summary = manager.get_storage_summary()
        assert summary["total_records"] == 3
        assert summary["records_by_category"]["SENSOR_DATA"] == 1
        assert summary["records_by_category"]["AUDIT_LOG"] == 1


# ===================================================================
# evaluate_records
# ===================================================================

class TestEvaluateRecords:
    """Tests for record evaluation and action generation."""

    def test_fresh_record_retained(self, manager):
        now = 1_000_000_000.0
        manager.register_record(_make_record("r1", age_days=1, current_time=now))
        actions = manager.evaluate_records(now)
        assert len(actions) == 1
        assert actions[0].action == 'retain'
        assert actions[0].record_id == "r1"

    def test_sensor_data_archived_after_90_days(self, manager):
        now = 1_000_000_000.0
        manager.register_record(
            _make_record("r1", category="SENSOR_DATA", age_days=100, current_time=now)
        )
        actions = manager.evaluate_records(now)
        assert len(actions) == 1
        assert actions[0].action == 'archive'

    def test_sensor_data_deleted_after_365_days(self, manager):
        now = 1_000_000_000.0
        manager.register_record(
            _make_record("r1", category="SENSOR_DATA", age_days=400, current_time=now)
        )
        actions = manager.evaluate_records(now)
        assert len(actions) == 1
        assert actions[0].action == 'delete'

    def test_audit_log_never_deleted(self, manager):
        now = 1_000_000_000.0
        # 10 years old — still retained because delete_after_days == 0
        manager.register_record(
            _make_record("r1", category="AUDIT_LOG", age_days=3650, current_time=now)
        )
        actions = manager.evaluate_records(now)
        assert len(actions) == 1
        assert actions[0].action == 'retain'

    def test_already_deleted_records_skipped(self, manager):
        now = 1_000_000_000.0
        record = _make_record("r1", category="SENSOR_DATA", age_days=400,
                              current_time=now)
        record.is_deleted = True
        record.deleted_at = now - 1000
        manager.register_record(record)
        actions = manager.evaluate_records(now)
        assert len(actions) == 0

    def test_already_archived_not_re_archived(self, manager):
        """A record already archived should get 'retain', not 'archive' again."""
        now = 1_000_000_000.0
        record = _make_record("r1", category="SENSOR_DATA", age_days=100,
                              current_time=now)
        record.is_archived = True
        record.archived_at = now - 10 * SECONDS_PER_DAY
        manager.register_record(record)
        actions = manager.evaluate_records(now)
        assert len(actions) == 1
        assert actions[0].action == 'retain'

    def test_no_policy_category_retained_by_default(self, manager):
        now = 1_000_000_000.0
        manager.register_record(
            _make_record("r1", category="UNKNOWN_CAT", age_days=9999,
                         current_time=now)
        )
        actions = manager.evaluate_records(now)
        assert len(actions) == 1
        assert actions[0].action == 'retain'
        assert 'no policy' in actions[0].reason

    def test_multiple_records_mixed_actions(self, manager):
        now = 1_000_000_000.0
        manager.register_record(
            _make_record("fresh", category="SENSOR_DATA", age_days=10,
                         current_time=now)
        )
        manager.register_record(
            _make_record("archive_me", category="SENSOR_DATA", age_days=100,
                         current_time=now)
        )
        manager.register_record(
            _make_record("delete_me", category="SENSOR_DATA", age_days=400,
                         current_time=now)
        )
        actions = manager.evaluate_records(now)
        action_map = {a.record_id: a.action for a in actions}
        assert action_map["fresh"] == 'retain'
        assert action_map["archive_me"] == 'archive'
        assert action_map["delete_me"] == 'delete'


# ===================================================================
# get_storage_summary
# ===================================================================

class TestStorageSummary:
    """Tests for storage summary statistics."""

    def test_empty_summary(self, manager):
        summary = manager.get_storage_summary()
        assert summary["total_records"] == 0
        assert summary["total_size"] == 0
        assert summary["records_by_category"] == {}
        assert summary["pending_archive"] == 0
        assert summary["pending_delete"] == 0

    def test_summary_with_records(self, manager):
        manager.register_record(_make_record("r1", size_bytes=500))
        manager.register_record(_make_record("r2", size_bytes=300))
        summary = manager.get_storage_summary()
        assert summary["total_records"] == 2
        assert summary["total_size"] == 800

    def test_summary_pending_counts(self, manager):
        """Archived records should not count toward pending_archive."""
        r1 = _make_record("r1")
        r2 = _make_record("r2")
        r2.is_archived = True
        r2.archived_at = time.time()
        manager.register_record(r1)
        manager.register_record(r2)
        summary = manager.get_storage_summary()
        # r1 is not archived/deleted -> pending_archive + 1
        # r2 is archived but not deleted -> not pending_archive
        assert summary["pending_archive"] == 1
        # Both are not deleted -> pending_delete == 2
        assert summary["pending_delete"] == 2


# ===================================================================
# apply_actions
# ===================================================================

class TestApplyActions:
    """Tests for applying retention actions to records."""

    def test_apply_archive_action(self, manager):
        now = 1_000_000_000.0
        manager.register_record(_make_record("r1", current_time=now))
        actions = [RetentionAction(
            record_id="r1", action='archive',
            reason='test', timestamp=now,
        )]
        manager.apply_actions(actions)
        summary = manager.get_storage_summary()
        # After archiving, r1 should not be pending_archive
        assert summary["pending_archive"] == 0

    def test_apply_delete_action(self, manager):
        now = 1_000_000_000.0
        manager.register_record(_make_record("r1", current_time=now))
        actions = [RetentionAction(
            record_id="r1", action='delete',
            reason='test', timestamp=now,
        )]
        manager.apply_actions(actions)
        summary = manager.get_storage_summary()
        assert summary["pending_delete"] == 0

    def test_apply_retain_action_noop(self, manager):
        now = 1_000_000_000.0
        manager.register_record(_make_record("r1", current_time=now))
        actions = [RetentionAction(
            record_id="r1", action='retain',
            reason='test', timestamp=now,
        )]
        manager.apply_actions(actions)
        summary = manager.get_storage_summary()
        assert summary["pending_archive"] == 1  # unchanged

    def test_apply_action_unknown_record_ignored(self, manager):
        """Applying an action for a non-existent record_id should not raise."""
        actions = [RetentionAction(
            record_id="nonexistent", action='delete',
            reason='test', timestamp=time.time(),
        )]
        manager.apply_actions(actions)  # should not raise

    def test_full_lifecycle(self, manager):
        """Register -> evaluate -> apply archive -> evaluate -> apply delete."""
        now = 1_000_000_000.0
        manager.register_record(
            _make_record("r1", category="SENSOR_DATA", age_days=100,
                         current_time=now)
        )

        # First evaluation: should archive
        actions = manager.evaluate_records(now)
        assert actions[0].action == 'archive'
        manager.apply_actions(actions)

        # After archive, re-evaluate at same time: should retain
        actions = manager.evaluate_records(now)
        assert actions[0].action == 'retain'

        # Advance time past delete threshold
        future = now + 300 * SECONDS_PER_DAY  # total age ~400 days
        actions = manager.evaluate_records(future)
        assert actions[0].action == 'delete'
        manager.apply_actions(actions)

        # After deletion, should no longer appear in evaluation
        actions = manager.evaluate_records(future)
        assert len(actions) == 0
