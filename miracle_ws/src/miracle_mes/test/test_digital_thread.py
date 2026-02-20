"""Tests for Digital Thread hash chain integrity verification.

Validates the blockchain-like hash chain used to ensure tamper evidence
in the manufacturing digital thread. Tests chain creation, verification,
and tamper detection.
"""

import hashlib
import json

import pytest


class _DigitalThreadEntry:
    """Simplified entry for the digital thread chain."""
    def __init__(self, entry_id='', data_json='', hash_value=''):
        self.entry_id = entry_id
        self.data_json = data_json
        self.hash_value = hash_value
        self.previous_entry_id = ''


class _DigitalThreadChain:
    """Extracted hash chain logic from DigitalThreadNode.

    Replicates the chain creation and verification logic so it can be
    tested without a ROS2 lifecycle.
    """

    def __init__(self):
        self.chain = []
        self._last_hash = '0' * 64

    def add_entry(self, entry_id: str, data: dict) -> _DigitalThreadEntry:
        """Add a new entry to the chain."""
        entry = _DigitalThreadEntry()
        entry.entry_id = entry_id
        entry.data_json = json.dumps(data, default=str)
        entry.previous_entry_id = self.chain[-1].entry_id if self.chain else ''

        hash_input = f"{entry.entry_id}{self._last_hash}{entry.data_json}"
        entry.hash_value = hashlib.sha256(hash_input.encode()).hexdigest()

        self.chain.append(entry)
        self._last_hash = entry.hash_value
        return entry

    def verify_chain_integrity(self) -> bool:
        """Verify the entire chain's hash integrity."""
        prev_hash = '0' * 64
        for entry in self.chain:
            hash_input = f"{entry.entry_id}{prev_hash}{entry.data_json}"
            expected = hashlib.sha256(hash_input.encode()).hexdigest()
            if entry.hash_value != expected:
                return False
            prev_hash = entry.hash_value
        return True


@pytest.fixture
def thread():
    return _DigitalThreadChain()


class TestChainCreation:
    """Tests for building the hash chain."""

    def test_first_entry_uses_genesis_hash(self, thread):
        """The first entry should link to the genesis hash (64 zeros)."""
        entry = thread.add_entry('entry_001', {'status': 'STARTED'})
        genesis_hash = '0' * 64
        expected_input = f"entry_001{genesis_hash}{entry.data_json}"
        expected_hash = hashlib.sha256(expected_input.encode()).hexdigest()
        assert entry.hash_value == expected_hash, (
            "First entry hash should be based on genesis hash"
        )

    def test_second_entry_links_to_first(self, thread):
        """The second entry's hash should include the first entry's hash."""
        e1 = thread.add_entry('entry_001', {'status': 'STARTED'})
        e2 = thread.add_entry('entry_002', {'status': 'RUNNING'})
        expected_input = f"entry_002{e1.hash_value}{e2.data_json}"
        expected_hash = hashlib.sha256(expected_input.encode()).hexdigest()
        assert e2.hash_value == expected_hash, (
            "Second entry hash must chain from the first entry's hash"
        )

    def test_previous_entry_id_tracking(self, thread):
        """Each entry should record the ID of the previous entry."""
        e1 = thread.add_entry('aaa', {'x': 1})
        e2 = thread.add_entry('bbb', {'x': 2})
        assert e1.previous_entry_id == '', "First entry should have no previous"
        assert e2.previous_entry_id == 'aaa', "Second entry should point to first"

    def test_deterministic_hashes(self, thread):
        """Same inputs should always produce the same hash."""
        thread.add_entry('id1', {'a': 1})
        h1 = thread.chain[0].hash_value

        thread2 = _DigitalThreadChain()
        thread2.add_entry('id1', {'a': 1})
        h2 = thread2.chain[0].hash_value

        assert h1 == h2, "Identical inputs should produce identical hashes"


class TestChainVerification:
    """Tests for verify_chain_integrity."""

    def test_empty_chain_is_valid(self, thread):
        """An empty chain should verify as valid."""
        assert thread.verify_chain_integrity() is True

    def test_single_entry_valid(self, thread):
        """A single-entry chain should verify successfully."""
        thread.add_entry('entry_001', {'status': 'ok'})
        assert thread.verify_chain_integrity() is True

    def test_multi_entry_chain_valid(self, thread):
        """A multi-entry chain should verify successfully."""
        for i in range(10):
            thread.add_entry(f'entry_{i:03d}', {'step': i, 'value': i * 1.5})
        assert thread.verify_chain_integrity() is True

    def test_tampered_data_detected(self, thread):
        """Modifying an entry's data_json should break chain integrity."""
        thread.add_entry('e1', {'status': 'STARTED'})
        thread.add_entry('e2', {'status': 'RUNNING'})
        thread.add_entry('e3', {'status': 'COMPLETED'})

        # Tamper with the second entry
        thread.chain[1].data_json = '{"status": "HACKED"}'

        assert thread.verify_chain_integrity() is False, (
            "Tampering with entry data should be detected"
        )

    def test_tampered_hash_detected(self, thread):
        """Directly modifying an entry's hash should break integrity."""
        thread.add_entry('e1', {'a': 1})
        thread.add_entry('e2', {'a': 2})

        thread.chain[0].hash_value = 'deadbeef' * 8

        assert thread.verify_chain_integrity() is False, (
            "Tampering with entry hash should be detected"
        )

    def test_tampered_entry_id_detected(self, thread):
        """Modifying an entry's ID should break chain integrity."""
        thread.add_entry('original_id', {'data': 'value'})

        thread.chain[0].entry_id = 'modified_id'

        assert thread.verify_chain_integrity() is False, (
            "Tampering with entry ID should be detected"
        )
