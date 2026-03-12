"""Tests for AES-256-GCM encrypted, Ed25519 signed secure audit storage.

Validates encryption roundtrips, hash chain integrity, tamper detection,
and persistence across SecureStorage instances.
"""

import json
import os
import struct

import pytest

pytest.importorskip("cryptography", reason="cryptography package required")

from miracle_security.secure_storage import SecureStorage


@pytest.fixture
def storage_dir(tmp_path):
    """Provide a temporary directory for secure storage."""
    return str(tmp_path / "audit")


@pytest.fixture
def storage(storage_dir):
    """Create a fresh SecureStorage instance."""
    return SecureStorage(storage_dir)


class TestEncryptDecrypt:
    """Tests for AES-256-GCM encrypt/decrypt roundtrip."""

    def test_roundtrip_simple(self, storage):
        """Encrypting then decrypting should return original plaintext."""
        plaintext = b"hello secure world"
        blob = storage._encrypt(plaintext)
        assert storage._decrypt(blob) == plaintext

    def test_roundtrip_empty(self, storage):
        """Empty plaintext should roundtrip correctly."""
        blob = storage._encrypt(b"")
        assert storage._decrypt(blob) == b""

    def test_roundtrip_large(self, storage):
        """Large payloads should roundtrip correctly."""
        plaintext = os.urandom(1024 * 64)
        blob = storage._encrypt(plaintext)
        assert storage._decrypt(blob) == plaintext

    def test_ciphertext_differs_from_plaintext(self, storage):
        """Encrypted blob should not contain the plaintext."""
        plaintext = b"sensitive data that must be hidden"
        blob = storage._encrypt(plaintext)
        assert plaintext not in blob

    def test_nonce_uniqueness(self, storage):
        """Two encryptions of the same plaintext should produce different blobs."""
        plaintext = b"same input"
        blob1 = storage._encrypt(plaintext)
        blob2 = storage._encrypt(plaintext)
        assert blob1 != blob2


class TestWriteAndRead:
    """Tests for write_entry and read_entries."""

    def test_write_single_entry(self, storage):
        """Writing a single entry should be readable."""
        entry = storage.write_entry({"event": "login", "user": "admin"})
        assert entry.sequence == 1
        assert entry.data["event"] == "login"

    def test_read_entries(self, storage):
        """Written entries should be readable in order."""
        storage.write_entry({"event": "login"})
        storage.write_entry({"event": "logout"})
        storage.write_entry({"event": "config_change"})

        entries = storage.read_entries()
        assert len(entries) == 3
        assert entries[0]["event"] == "login"
        assert entries[1]["event"] == "logout"
        assert entries[2]["event"] == "config_change"

    def test_read_entries_with_start_seq(self, storage):
        """read_entries should filter by start sequence."""
        for i in range(5):
            storage.write_entry({"index": i})

        entries = storage.read_entries(start_seq=3)
        assert len(entries) == 3
        assert entries[0]["seq"] == 3

    def test_read_entries_with_limit(self, storage):
        """read_entries should respect the limit parameter."""
        for i in range(10):
            storage.write_entry({"index": i})

        entries = storage.read_entries(limit=3)
        assert len(entries) == 3

    def test_entry_count(self, storage):
        """entry_count should track the number of entries."""
        assert storage.entry_count == 0
        storage.write_entry({"a": 1})
        storage.write_entry({"b": 2})
        assert storage.entry_count == 2

    def test_entries_have_metadata(self, storage):
        """Each entry should include seq and ts metadata."""
        storage.write_entry({"event": "test"})
        entries = storage.read_entries()
        assert "seq" in entries[0]
        assert "ts" in entries[0]
        assert entries[0]["seq"] == 1


class TestVerifyChain:
    """Tests for hash chain verification."""

    def test_empty_chain_valid(self, storage):
        """An empty storage should verify as valid."""
        assert storage.verify_chain() is True

    def test_single_entry_valid(self, storage):
        """A single entry chain should verify."""
        storage.write_entry({"event": "test"})
        assert storage.verify_chain() is True

    def test_multi_entry_chain_valid(self, storage):
        """A multi-entry chain should verify."""
        for i in range(20):
            storage.write_entry({"index": i, "severity": "LOW"})
        assert storage.verify_chain() is True

    def test_first_entry_uses_genesis(self, storage):
        """First entry's prev_hash should be the genesis hash."""
        storage.write_entry({"event": "first"})
        assert storage._entries[0].prev_hash == SecureStorage.CHAIN_GENESIS


class TestTamperDetection:
    """Tests for detecting tampered entries."""

    def test_tampered_file_detected(self, storage_dir):
        """Modifying an entry file on disk should break verification on reload."""
        storage = SecureStorage(storage_dir)
        storage.write_entry({"event": "one"})
        storage.write_entry({"event": "two"})
        storage.write_entry({"event": "three"})

        # Tamper with the second entry file on disk
        entry_file = os.path.join(storage_dir, "entry_00000002.bin")
        data = bytearray(open(entry_file, "rb").read())
        # Flip a byte in the encrypted payload area (after header: 32+32+8+64 = 136)
        if len(data) > 140:
            data[140] ^= 0xFF
        with open(entry_file, "wb") as f:
            f.write(bytes(data))

        # Reload — the tampered entry should be skipped
        reloaded = SecureStorage(storage_dir)
        # The tampered entry (and any subsequent entries that depend on it)
        # may be dropped, so we should have fewer than 3
        assert reloaded.entry_count < 3

    def test_tampered_in_memory_detected(self, storage):
        """Modifying an entry's prev_hash in memory should break verify_chain."""
        storage.write_entry({"event": "one"})
        storage.write_entry({"event": "two"})
        storage.write_entry({"event": "three"})

        # Tamper with the second entry's prev_hash
        storage._entries[1].prev_hash = b'\xff' * 32

        assert storage.verify_chain() is False


class TestPersistence:
    """Tests for write-reload-verify cycle."""

    def test_persist_and_reload(self, storage_dir):
        """Entries written by one instance should be loadable by another."""
        s1 = SecureStorage(storage_dir)
        s1.write_entry({"event": "alpha"})
        s1.write_entry({"event": "beta"})
        s1.write_entry({"event": "gamma"})

        # Create a new instance from the same directory (same keys on disk)
        s2 = SecureStorage(storage_dir)
        assert s2.entry_count == 3
        assert s2.verify_chain() is True

        entries = s2.read_entries()
        assert entries[0]["event"] == "alpha"
        assert entries[2]["event"] == "gamma"

    def test_append_after_reload(self, storage_dir):
        """New entries after reload should extend the existing chain."""
        s1 = SecureStorage(storage_dir)
        s1.write_entry({"event": "first"})

        s2 = SecureStorage(storage_dir)
        s2.write_entry({"event": "second"})

        s3 = SecureStorage(storage_dir)
        assert s3.entry_count == 2
        assert s3.verify_chain() is True
        entries = s3.read_entries()
        assert entries[0]["event"] == "first"
        assert entries[1]["event"] == "second"

    def test_keys_persisted(self, storage_dir):
        """Keys should be auto-generated and persisted across instances."""
        s1 = SecureStorage(storage_dir)
        key1 = s1._aes_key

        s2 = SecureStorage(storage_dir)
        key2 = s2._aes_key

        assert key1 == key2
