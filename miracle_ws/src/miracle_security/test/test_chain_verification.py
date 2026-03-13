"""Tests for chain verification, tamper detection, and related SecureStorage features.

Covers: verify_chain_detailed, export_audit_range, get_chain_statistics,
compact_chain, rotate_encryption_key, and edge cases.
"""

import os
import struct
import time

import pytest

pytest.importorskip("cryptography", reason="cryptography package required")

from miracle_security.secure_storage import SecureStorage, TamperReport


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def storage_dir(tmp_path):
    return str(tmp_path / "audit")


@pytest.fixture
def storage(storage_dir):
    return SecureStorage(storage_dir)


def _populate(storage, n=5):
    """Write *n* entries with simple data and return the storage."""
    for i in range(n):
        storage.write_entry({"index": i, "msg": f"event_{i}"})
    return storage


# ===================================================================
# verify_chain_detailed
# ===================================================================

class TestVerifyChainDetailed:
    """Tests for the detailed chain verification report."""

    def test_empty_chain_is_valid(self, storage):
        report = storage.verify_chain_detailed()
        assert report.is_valid is True
        assert report.broken_link_index is None
        assert report.total_entries == 0
        assert report.verified_entries == 0
        assert report.verification_time_ms >= 0

    def test_single_entry_valid(self, storage):
        storage.write_entry({"event": "only"})
        report = storage.verify_chain_detailed()
        assert report.is_valid is True
        assert report.verified_entries == 1
        assert report.total_entries == 1

    def test_multi_entry_valid(self, storage):
        _populate(storage, 10)
        report = storage.verify_chain_detailed()
        assert report.is_valid is True
        assert report.verified_entries == 10
        assert report.total_entries == 10
        assert report.broken_link_index is None

    def test_tampered_middle_entry_detected(self, storage):
        _populate(storage, 5)
        # Tamper with entry at index 2
        storage._entries[2].prev_hash = b'\xde\xad' * 16
        report = storage.verify_chain_detailed()
        assert report.is_valid is False
        assert report.broken_link_index == 2
        assert report.verified_entries == 2

    def test_tampered_first_entry_detected(self, storage):
        _populate(storage, 3)
        storage._entries[0].prev_hash = b'\x01' * 32
        report = storage.verify_chain_detailed()
        assert report.is_valid is False
        assert report.broken_link_index == 0
        assert report.verified_entries == 0

    def test_tampered_last_entry_detected(self, storage):
        _populate(storage, 4)
        storage._entries[-1].prev_hash = b'\xab' * 32
        report = storage.verify_chain_detailed()
        assert report.is_valid is False
        assert report.broken_link_index == 3

    def test_report_contains_expected_and_actual_hash(self, storage):
        _populate(storage, 3)
        real_prev = storage._entries[1].prev_hash
        fake_prev = b'\xff' * 32
        storage._entries[1].prev_hash = fake_prev
        report = storage.verify_chain_detailed()
        assert report.is_valid is False
        assert report.actual_hash == fake_prev.hex()
        # expected_hash should be the entry_hash of entry 0
        assert report.expected_hash == storage._entries[0].entry_hash.hex()

    def test_verification_time_is_positive(self, storage):
        _populate(storage, 20)
        report = storage.verify_chain_detailed()
        assert report.verification_time_ms >= 0

    def test_report_is_tamper_report_instance(self, storage):
        report = storage.verify_chain_detailed()
        assert isinstance(report, TamperReport)

    def test_missing_entry_breaks_chain(self, storage):
        """Removing an entry from the in-memory list should break continuity."""
        _populate(storage, 5)
        # Remove entry at index 2 (seq 3)
        del storage._entries[2]
        report = storage.verify_chain_detailed()
        assert report.is_valid is False
        # The entry now at index 2 (was index 3) has prev_hash pointing to
        # the removed entry's hash, not the hash of the entry now at index 1
        assert report.broken_link_index == 2


# ===================================================================
# export_audit_range
# ===================================================================

class TestExportAuditRange:
    """Tests for compliance range export."""

    def test_full_range(self, storage):
        _populate(storage, 5)
        entries = storage.export_audit_range(1, 5)
        assert len(entries) == 5

    def test_partial_range(self, storage):
        _populate(storage, 10)
        entries = storage.export_audit_range(3, 7)
        assert len(entries) == 5
        assert entries[0]["seq"] == 3
        assert entries[-1]["seq"] == 7

    def test_single_entry_range(self, storage):
        _populate(storage, 5)
        entries = storage.export_audit_range(3, 3)
        assert len(entries) == 1
        assert entries[0]["seq"] == 3

    def test_empty_range_returns_empty(self, storage):
        _populate(storage, 5)
        entries = storage.export_audit_range(10, 20)
        assert entries == []

    def test_range_on_empty_chain(self, storage):
        entries = storage.export_audit_range(1, 10)
        assert entries == []

    def test_range_preserves_data(self, storage):
        storage.write_entry({"key": "value_1"})
        storage.write_entry({"key": "value_2"})
        entries = storage.export_audit_range(1, 2)
        assert entries[0]["key"] == "value_1"
        assert entries[1]["key"] == "value_2"


# ===================================================================
# get_chain_statistics
# ===================================================================

class TestGetChainStatistics:
    """Tests for chain statistics."""

    def test_empty_stats(self, storage):
        stats = storage.get_chain_statistics()
        assert stats["total_entries"] == 0
        assert stats["chain_length"] == 0
        assert stats["first_entry_time"] is None
        assert stats["last_entry_time"] is None
        assert stats["size_bytes"] == 0

    def test_stats_after_writes(self, storage):
        _populate(storage, 3)
        stats = storage.get_chain_statistics()
        assert stats["total_entries"] == 3
        assert stats["chain_length"] == 3
        assert stats["first_entry_time"] is not None
        assert stats["last_entry_time"] is not None
        assert stats["first_entry_time"] <= stats["last_entry_time"]
        assert stats["size_bytes"] > 0

    def test_size_bytes_grows(self, storage):
        storage.write_entry({"a": 1})
        s1 = storage.get_chain_statistics()["size_bytes"]
        storage.write_entry({"b": 2})
        s2 = storage.get_chain_statistics()["size_bytes"]
        assert s2 > s1


# ===================================================================
# compact_chain
# ===================================================================

class TestCompactChain:
    """Tests for chain compaction / archival."""

    def test_compact_keeps_last_n(self, storage):
        _populate(storage, 10)
        archived = storage.compact_chain(keep_last_n=3)
        assert archived == 7
        assert storage.entry_count == 3

    def test_compact_moves_files_to_archive(self, storage_dir):
        s = SecureStorage(storage_dir)
        _populate(s, 5)
        s.compact_chain(keep_last_n=2)
        from pathlib import Path
        archive = Path(storage_dir) / "archive"
        assert archive.exists()
        archived_files = list(archive.glob("entry_*.bin"))
        assert len(archived_files) == 3

    def test_compact_keep_all(self, storage):
        _populate(storage, 5)
        archived = storage.compact_chain(keep_last_n=5)
        assert archived == 0
        assert storage.entry_count == 5

    def test_compact_keep_more_than_total(self, storage):
        _populate(storage, 3)
        archived = storage.compact_chain(keep_last_n=100)
        assert archived == 0
        assert storage.entry_count == 3

    def test_compact_keep_zero(self, storage):
        _populate(storage, 4)
        archived = storage.compact_chain(keep_last_n=0)
        assert archived == 4
        assert storage.entry_count == 0

    def test_compact_negative_raises(self, storage):
        with pytest.raises(ValueError):
            storage.compact_chain(keep_last_n=-1)

    def test_compact_empty_chain(self, storage):
        archived = storage.compact_chain(keep_last_n=5)
        assert archived == 0


# ===================================================================
# rotate_encryption_key
# ===================================================================

class TestRotateEncryptionKey:
    """Tests for key rotation."""

    def test_roundtrip_after_rotation(self, storage):
        _populate(storage, 5)
        original_data = storage.read_entries()
        new_key = os.urandom(32)
        storage.rotate_encryption_key(new_key)
        assert storage.read_entries() == original_data

    def test_persisted_data_readable_after_rotation(self, storage_dir):
        s1 = SecureStorage(storage_dir)
        _populate(s1, 3)
        new_key = os.urandom(32)
        s1.rotate_encryption_key(new_key)

        # Reload from disk with new key
        s2 = SecureStorage(storage_dir, encryption_key=new_key)
        assert s2.entry_count == 3
        assert s2.verify_chain() is True
        entries = s2.read_entries()
        assert entries[0]["index"] == 0

    def test_chain_valid_after_rotation(self, storage):
        _populate(storage, 5)
        new_key = os.urandom(32)
        storage.rotate_encryption_key(new_key)
        assert storage.verify_chain() is True

    def test_detailed_report_valid_after_rotation(self, storage):
        _populate(storage, 5)
        new_key = os.urandom(32)
        storage.rotate_encryption_key(new_key)
        report = storage.verify_chain_detailed()
        assert report.is_valid is True
        assert report.verified_entries == 5

    def test_invalid_key_length_raises(self, storage):
        with pytest.raises(ValueError, match="16, 24, or 32"):
            storage.rotate_encryption_key(b"short")

    def test_multiple_rotations(self, storage):
        _populate(storage, 3)
        original = storage.read_entries()
        for _ in range(3):
            storage.rotate_encryption_key(os.urandom(32))
        assert storage.read_entries() == original
        assert storage.verify_chain() is True


# ===================================================================
# Large chain / performance
# ===================================================================

class TestLargeChain:
    """Performance-oriented tests for larger chains."""

    def test_large_chain_verification(self, storage):
        _populate(storage, 100)
        report = storage.verify_chain_detailed()
        assert report.is_valid is True
        assert report.verified_entries == 100
        # Should complete in a reasonable time
        assert report.verification_time_ms < 10_000

    def test_large_chain_tamper_early(self, storage):
        _populate(storage, 50)
        storage._entries[5].prev_hash = b'\x00' * 32
        report = storage.verify_chain_detailed()
        assert report.is_valid is False
        assert report.broken_link_index == 5
        # Should short-circuit — only 5 verified before finding the break
        assert report.verified_entries == 5
