"""Tests for hash chain tamper evidence in the audit logger.

Validates the audit logger's hash chain mechanism that provides
tamper-evident logging for security compliance. Tests chain creation,
verification, and tamper detection.
"""

import hashlib
import json

import pytest


class _AuditChain:
    """Extracted hash chain logic from AuditLoggerNode.

    Replicates the chain_hash computation used in _on_alert for
    tamper-evident logging.
    """

    def __init__(self):
        self.entries = []
        self._last_hash = '0' * 64

    def add_entry(self, entry: dict) -> dict:
        """Add an entry and compute its chain hash."""
        chain_input = f"{self._last_hash}:{json.dumps(entry, sort_keys=True)}"
        entry_with_hash = dict(entry)
        entry_with_hash['chain_hash'] = hashlib.sha256(chain_input.encode()).hexdigest()
        self._last_hash = entry_with_hash['chain_hash']
        self.entries.append(entry_with_hash)
        return entry_with_hash

    def verify_chain(self) -> bool:
        """Verify the integrity of the full audit chain."""
        prev_hash = '0' * 64
        for entry in self.entries:
            entry_copy = {k: v for k, v in entry.items() if k != 'chain_hash'}
            chain_input = f"{prev_hash}:{json.dumps(entry_copy, sort_keys=True)}"
            expected = hashlib.sha256(chain_input.encode()).hexdigest()
            if entry['chain_hash'] != expected:
                return False
            prev_hash = entry['chain_hash']
        return True


@pytest.fixture
def chain():
    return _AuditChain()


class TestChainCreation:
    """Tests for building the audit hash chain."""

    def test_first_entry_uses_genesis_hash(self, chain):
        """First entry should chain from the genesis hash (64 zeros)."""
        entry = {'alert_id': 'ACL-000001', 'severity': 'HIGH'}
        result = chain.add_entry(entry)
        genesis = '0' * 64
        expected_input = f"{genesis}:{json.dumps(entry, sort_keys=True)}"
        expected_hash = hashlib.sha256(expected_input.encode()).hexdigest()
        assert result['chain_hash'] == expected_hash

    def test_chain_links_entries(self, chain):
        """Each entry's hash should depend on the previous entry's hash."""
        e1 = chain.add_entry({'alert_id': '001', 'severity': 'LOW'})
        e2_data = {'alert_id': '002', 'severity': 'MEDIUM'}
        e2 = chain.add_entry(e2_data)

        expected_input = f"{e1['chain_hash']}:{json.dumps(e2_data, sort_keys=True)}"
        expected = hashlib.sha256(expected_input.encode()).hexdigest()
        assert e2['chain_hash'] == expected

    def test_hash_is_64_hex_chars(self, chain):
        """Each chain hash should be a 64-character hex string (SHA-256)."""
        entry = chain.add_entry({'test': 'data'})
        assert len(entry['chain_hash']) == 64
        assert all(c in '0123456789abcdef' for c in entry['chain_hash'])

    def test_deterministic_hashes(self):
        """Same inputs should always produce the same chain hashes."""
        c1 = _AuditChain()
        c2 = _AuditChain()
        e1 = c1.add_entry({'x': 1})
        e2 = c2.add_entry({'x': 1})
        assert e1['chain_hash'] == e2['chain_hash']


class TestChainVerification:
    """Tests for verify_chain integrity."""

    def test_empty_chain_valid(self, chain):
        """An empty chain should verify as valid."""
        assert chain.verify_chain() is True

    def test_single_entry_valid(self, chain):
        """A single-entry chain should verify successfully."""
        chain.add_entry({'alert_id': 'test', 'description': 'test event'})
        assert chain.verify_chain() is True

    def test_multi_entry_chain_valid(self, chain):
        """A multi-entry chain should verify successfully."""
        for i in range(20):
            chain.add_entry({'alert_id': f'ACL-{i:06d}', 'severity': 'LOW', 'index': i})
        assert chain.verify_chain() is True

    def test_tampered_data_detected(self, chain):
        """Modifying entry data after hashing should break verification."""
        chain.add_entry({'alert_id': '001', 'severity': 'LOW'})
        chain.add_entry({'alert_id': '002', 'severity': 'MEDIUM'})
        chain.add_entry({'alert_id': '003', 'severity': 'HIGH'})

        # Tamper with the second entry's data
        chain.entries[1]['severity'] = 'TAMPERED'

        assert chain.verify_chain() is False, (
            "Tampering with entry data should be detected"
        )

    def test_tampered_hash_detected(self, chain):
        """Directly modifying an entry's hash should break verification."""
        chain.add_entry({'alert_id': '001'})
        chain.add_entry({'alert_id': '002'})

        chain.entries[0]['chain_hash'] = 'a' * 64

        assert chain.verify_chain() is False, (
            "Tampering with a hash should break the chain"
        )

    def test_removed_entry_detected(self, chain):
        """Removing an entry from the middle should break verification."""
        chain.add_entry({'alert_id': '001'})
        chain.add_entry({'alert_id': '002'})
        chain.add_entry({'alert_id': '003'})

        # Remove the middle entry
        del chain.entries[1]

        assert chain.verify_chain() is False, (
            "Removing an entry from the chain should be detected"
        )
