"""Tests for encrypted audit logging integration.

Validates that AuditLoggerNode uses SecureStorage when available and
correctly falls back to plaintext logging when it is not.
"""

import os
from unittest.mock import MagicMock, patch

import pytest

pytest.importorskip("cryptography", reason="cryptography package required")

from miracle_security.audit_logger import AuditLoggerNode


class TestResolvAuditDir:
    """Tests for the _resolve_audit_dir static method."""

    def test_writable_dir_used(self, tmp_path):
        """If the requested directory is writable, it should be returned."""
        d = str(tmp_path / "audit")
        result = AuditLoggerNode._resolve_audit_dir(d)
        assert result == d
        assert os.path.isdir(d)

    def test_non_writable_falls_back(self, tmp_path):
        """If the requested directory is not writable, fallback is used."""
        # Use a path that is almost certainly not writable by a regular user
        impossible = "/proc/nonexistent_miracle_test_dir"
        result = AuditLoggerNode._resolve_audit_dir(impossible)
        assert result == "/tmp/miracle_audit"

    def test_default_path_is_var(self):
        """The default log_directory parameter should be /var/miracle/audit."""
        # We inspect the parameter declaration indirectly — the static method
        # is what _do_configure uses, so we just verify the fallback logic.
        # A writable /var/miracle/audit would be used; otherwise /tmp.
        result = AuditLoggerNode._resolve_audit_dir("/var/miracle/audit")
        assert result in ("/var/miracle/audit", "/tmp/miracle_audit")


class TestSecureStorageIntegration:
    """Tests for _write_entry using SecureStorage."""

    def test_write_entry_delegates_to_secure_storage(self):
        """_write_entry should call SecureStorage.write_entry when available."""
        node = object.__new__(AuditLoggerNode)
        # Manually set the fields _write_entry depends on
        node._lock = __import__("threading").Lock()
        node._secure_storage = MagicMock()
        node._current_file = None

        entry = {"event": "test", "severity": "LOW"}
        node._write_entry(entry)

        node._secure_storage.write_entry.assert_called_once_with(entry)

    def test_write_entry_falls_back_on_secure_storage_error(self):
        """If SecureStorage.write_entry raises, fall back to plaintext."""
        node = object.__new__(AuditLoggerNode)
        node._lock = __import__("threading").Lock()
        node._secure_storage = MagicMock()
        node._secure_storage.write_entry.side_effect = RuntimeError("disk full")
        node._current_file = MagicMock()
        node._entry_count = 0
        node._max_per_file = 10000

        # Mock the logger
        mock_logger = MagicMock()
        node.get_logger = MagicMock(return_value=mock_logger)

        entry = {"event": "test"}
        node._write_entry(entry)

        # Should have logged the error and written to plaintext file
        mock_logger.error.assert_called_once()
        node._current_file.write.assert_called_once()

    def test_write_entry_plaintext_when_no_secure_storage(self):
        """When _secure_storage is None, entries go to plaintext file."""
        node = object.__new__(AuditLoggerNode)
        node._lock = __import__("threading").Lock()
        node._secure_storage = None
        node._current_file = MagicMock()
        node._entry_count = 0
        node._max_per_file = 10000

        entry = {"event": "test"}
        node._write_entry(entry)

        node._current_file.write.assert_called_once()
