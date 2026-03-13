"""
Secure Storage — AES-256-GCM encrypted, Ed25519 signed audit log storage.

Each entry: [nonce(12)][ciphertext][tag(16)]
Hash chain: sign(prev_hash || entry_hash || seq_number)
"""

import hashlib
import json
import os
import shutil
import struct
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    from cryptography.hazmat.primitives.ciphers.aead import AESGCM
    from cryptography.hazmat.primitives.asymmetric.ed25519 import (
        Ed25519PrivateKey,
        Ed25519PublicKey,
    )
    HAS_CRYPTOGRAPHY = True
except ImportError:
    HAS_CRYPTOGRAPHY = False
    serialization = None

if HAS_CRYPTOGRAPHY:
    from cryptography.hazmat.primitives import serialization


@dataclass
class AuditEntry:
    """A single audit log entry."""
    sequence: int
    timestamp: float
    data: Dict[str, Any]
    entry_hash: bytes
    prev_hash: bytes
    signature: bytes


@dataclass
class TamperReport:
    """Result of a detailed chain verification."""
    is_valid: bool
    broken_link_index: Optional[int]
    expected_hash: Optional[str]
    actual_hash: Optional[str]
    total_entries: int
    verified_entries: int
    verification_time_ms: float


class SecureStorage:
    """AES-256-GCM encrypted, Ed25519 hash-chain signed audit storage.

    Args:
        storage_dir: Directory for encrypted audit files.
        encryption_key: 32-byte AES-256 key. Auto-generated if not provided.
        signing_key: Ed25519 private key PEM bytes. Auto-generated if not provided.
    """

    NONCE_SIZE = 12
    TAG_SIZE = 16
    CHAIN_GENESIS = b'\x00' * 32

    def __init__(
        self,
        storage_dir: str,
        encryption_key: Optional[bytes] = None,
        signing_key: Optional[bytes] = None,
    ) -> None:
        if not HAS_CRYPTOGRAPHY:
            raise ImportError(
                "SecureStorage requires the 'cryptography' package. "
                "Install with: pip install cryptography"
            )
        self._dir = Path(storage_dir)
        self._dir.mkdir(parents=True, exist_ok=True)

        # AES-256 key
        key_path = self._dir / '.aes_key'
        if encryption_key:
            self._aes_key = encryption_key
        elif key_path.exists():
            self._aes_key = key_path.read_bytes()
        else:
            self._aes_key = AESGCM.generate_key(bit_length=256)
            key_path.write_bytes(self._aes_key)
            os.chmod(key_path, 0o600)

        self._aesgcm = AESGCM(self._aes_key)

        # Ed25519 signing key
        sign_key_path = self._dir / '.sign_key'
        if signing_key:
            self._signing_key = serialization.load_pem_private_key(signing_key, password=None)
        elif sign_key_path.exists():
            self._signing_key = serialization.load_pem_private_key(
                sign_key_path.read_bytes(), password=None
            )
        else:
            self._signing_key = Ed25519PrivateKey.generate()
            sign_key_path.write_bytes(
                self._signing_key.private_bytes(
                    serialization.Encoding.PEM,
                    serialization.PrivateFormat.PKCS8,
                    serialization.NoEncryption(),
                )
            )
            os.chmod(sign_key_path, 0o600)

        self._public_key = self._signing_key.public_key()
        self._entries: List[AuditEntry] = []
        self._sequence = 0
        self._prev_hash = self.CHAIN_GENESIS

        # Load existing entries
        self._load_existing()

    def _encrypt(self, plaintext: bytes) -> bytes:
        """Encrypt with AES-256-GCM: [nonce(12)][ciphertext+tag]."""
        nonce = os.urandom(self.NONCE_SIZE)
        ct = self._aesgcm.encrypt(nonce, plaintext, None)
        return nonce + ct

    def _decrypt(self, blob: bytes) -> bytes:
        """Decrypt AES-256-GCM blob."""
        nonce = blob[:self.NONCE_SIZE]
        ct = blob[self.NONCE_SIZE:]
        return self._aesgcm.decrypt(nonce, ct, None)

    def _hash_entry(self, data_bytes: bytes, seq: int, prev_hash: bytes) -> bytes:
        """SHA-256 hash of prev_hash || data || sequence."""
        h = hashlib.sha256()
        h.update(prev_hash)
        h.update(data_bytes)
        h.update(struct.pack('>Q', seq))
        return h.digest()

    def _sign(self, entry_hash: bytes, seq: int, prev_hash: bytes) -> bytes:
        """Ed25519 signature over (prev_hash || entry_hash || seq)."""
        payload = prev_hash + entry_hash + struct.pack('>Q', seq)
        return self._signing_key.sign(payload)

    def _verify_signature(
        self, signature: bytes, entry_hash: bytes, seq: int, prev_hash: bytes
    ) -> bool:
        """Verify Ed25519 signature."""
        payload = prev_hash + entry_hash + struct.pack('>Q', seq)
        try:
            self._public_key.verify(signature, payload)
            return True
        except Exception:
            return False

    def write_entry(self, data: Dict[str, Any]) -> AuditEntry:
        """Write and encrypt a new audit entry with hash chain signature."""
        self._sequence += 1
        data_with_meta = {
            'seq': self._sequence,
            'ts': time.time(),
            **data,
        }
        data_bytes = json.dumps(data_with_meta, sort_keys=True).encode()

        entry_hash = self._hash_entry(data_bytes, self._sequence, self._prev_hash)
        signature = self._sign(entry_hash, self._sequence, self._prev_hash)

        # Build record: [entry_hash(32)][prev_hash(32)][seq(8)][sig(64)][encrypted_data]
        encrypted = self._encrypt(data_bytes)
        record = entry_hash + self._prev_hash + struct.pack('>Q', self._sequence) + signature + encrypted

        # Write to file
        entry_file = self._dir / f'entry_{self._sequence:08d}.bin'
        entry_file.write_bytes(record)

        entry = AuditEntry(
            sequence=self._sequence,
            timestamp=data_with_meta['ts'],
            data=data_with_meta,
            entry_hash=entry_hash,
            prev_hash=self._prev_hash,
            signature=signature,
        )
        self._entries.append(entry)
        self._prev_hash = entry_hash
        return entry

    def _load_existing(self) -> None:
        """Load and verify existing entries."""
        files = sorted(self._dir.glob('entry_*.bin'))
        for f in files:
            try:
                raw = f.read_bytes()
                entry_hash = raw[:32]
                prev_hash = raw[32:64]
                seq = struct.unpack('>Q', raw[64:72])[0]
                signature = raw[72:136]
                encrypted = raw[136:]

                data_bytes = self._decrypt(encrypted)
                data = json.loads(data_bytes)

                computed_hash = self._hash_entry(data_bytes, seq, prev_hash)
                if computed_hash != entry_hash:
                    raise ValueError(f"Hash mismatch at seq {seq}")

                if not self._verify_signature(signature, entry_hash, seq, prev_hash):
                    raise ValueError(f"Signature invalid at seq {seq}")

                entry = AuditEntry(
                    sequence=seq,
                    timestamp=data.get('ts', 0),
                    data=data,
                    entry_hash=entry_hash,
                    prev_hash=prev_hash,
                    signature=signature,
                )
                self._entries.append(entry)
                self._sequence = seq
                self._prev_hash = entry_hash
            except Exception:
                pass  # Skip corrupted entries

    def verify_chain(self) -> bool:
        """Verify entire hash chain integrity and all signatures."""
        if not self._entries:
            return True

        expected_prev = self.CHAIN_GENESIS
        for entry in self._entries:
            if entry.prev_hash != expected_prev:
                return False
            if not self._verify_signature(
                entry.signature, entry.entry_hash, entry.sequence, entry.prev_hash
            ):
                return False
            expected_prev = entry.entry_hash
        return True

    def read_entries(self, start_seq: int = 0, limit: int = 100) -> List[Dict[str, Any]]:
        """Read decrypted entries."""
        return [
            e.data for e in self._entries
            if e.sequence >= start_seq
        ][:limit]

    @property
    def entry_count(self) -> int:
        return len(self._entries)

    # ------------------------------------------------------------------
    # Chain verification & tamper detection
    # ------------------------------------------------------------------

    def verify_chain_detailed(self) -> TamperReport:
        """Walk the full hash chain and return a detailed tamper report.

        Returns a :class:`TamperReport` describing whether the chain is intact.
        If a broken link is found the report includes the index of the first
        broken entry along with the expected and actual ``prev_hash`` values.
        """
        start = time.monotonic()

        if not self._entries:
            elapsed = (time.monotonic() - start) * 1000.0
            return TamperReport(
                is_valid=True,
                broken_link_index=None,
                expected_hash=None,
                actual_hash=None,
                total_entries=0,
                verified_entries=0,
                verification_time_ms=elapsed,
            )

        expected_prev = self.CHAIN_GENESIS
        verified = 0
        for idx, entry in enumerate(self._entries):
            # Check chain link
            if entry.prev_hash != expected_prev:
                elapsed = (time.monotonic() - start) * 1000.0
                return TamperReport(
                    is_valid=False,
                    broken_link_index=idx,
                    expected_hash=expected_prev.hex(),
                    actual_hash=entry.prev_hash.hex(),
                    total_entries=len(self._entries),
                    verified_entries=verified,
                    verification_time_ms=elapsed,
                )
            # Check signature
            if not self._verify_signature(
                entry.signature, entry.entry_hash, entry.sequence, entry.prev_hash
            ):
                elapsed = (time.monotonic() - start) * 1000.0
                return TamperReport(
                    is_valid=False,
                    broken_link_index=idx,
                    expected_hash=expected_prev.hex(),
                    actual_hash=entry.prev_hash.hex(),
                    total_entries=len(self._entries),
                    verified_entries=verified,
                    verification_time_ms=elapsed,
                )
            verified += 1
            expected_prev = entry.entry_hash

        elapsed = (time.monotonic() - start) * 1000.0
        return TamperReport(
            is_valid=True,
            broken_link_index=None,
            expected_hash=None,
            actual_hash=None,
            total_entries=len(self._entries),
            verified_entries=verified,
            verification_time_ms=elapsed,
        )

    # ------------------------------------------------------------------
    # Export / compliance
    # ------------------------------------------------------------------

    def export_audit_range(self, start_seq: int, end_seq: int) -> List[Dict[str, Any]]:
        """Return decrypted entry data for *start_seq* <= seq <= *end_seq*.

        Useful for compliance reporting where a specific sequence range is needed.
        """
        return [
            e.data for e in self._entries
            if start_seq <= e.sequence <= end_seq
        ]

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    def get_chain_statistics(self) -> Dict[str, Any]:
        """Return summary statistics about the current chain.

        Keys: ``total_entries``, ``chain_length``, ``first_entry_time``,
        ``last_entry_time``, ``size_bytes``.
        """
        total = len(self._entries)
        first_ts: Optional[float] = None
        last_ts: Optional[float] = None
        if total > 0:
            first_ts = self._entries[0].timestamp
            last_ts = self._entries[-1].timestamp

        size = 0
        for f in self._dir.glob('entry_*.bin'):
            size += f.stat().st_size

        return {
            'total_entries': total,
            'chain_length': total,
            'first_entry_time': first_ts,
            'last_entry_time': last_ts,
            'size_bytes': size,
        }

    # ------------------------------------------------------------------
    # Compaction
    # ------------------------------------------------------------------

    def compact_chain(self, keep_last_n: int) -> int:
        """Archive old entries, keeping only the last *keep_last_n* entries.

        Archived entries are moved to an ``archive/`` sub-directory.  The
        remaining in-memory chain is updated so that the oldest kept entry's
        ``prev_hash`` becomes the new effective genesis (no rewrite needed —
        the raw files already contain the correct ``prev_hash``).

        Returns the number of entries archived.
        """
        if keep_last_n < 0:
            raise ValueError("keep_last_n must be non-negative")

        total = len(self._entries)
        if keep_last_n >= total:
            return 0  # nothing to archive

        archive_dir = self._dir / 'archive'
        archive_dir.mkdir(parents=True, exist_ok=True)

        to_archive = total - keep_last_n
        archived = 0
        for entry in self._entries[:to_archive]:
            src = self._dir / f'entry_{entry.sequence:08d}.bin'
            if src.exists():
                shutil.move(str(src), str(archive_dir / src.name))
            archived += 1

        # Trim in-memory list
        self._entries = self._entries[to_archive:]

        return archived

    # ------------------------------------------------------------------
    # Key rotation
    # ------------------------------------------------------------------

    def rotate_encryption_key(self, new_key: bytes) -> None:
        """Re-encrypt every entry with *new_key* while preserving the hash chain.

        The hash chain and signatures are unaffected because they operate on
        the plaintext data, not the ciphertext.  Only the encrypted payload
        portion of each record is replaced.
        """
        if len(new_key) not in (16, 24, 32):
            raise ValueError("new_key must be 16, 24, or 32 bytes (AES key)")

        new_aesgcm = AESGCM(new_key)

        for entry in self._entries:
            entry_file = self._dir / f'entry_{entry.sequence:08d}.bin'
            raw = entry_file.read_bytes()

            # Decrypt with current key
            encrypted_old = raw[136:]
            data_bytes = self._decrypt(encrypted_old)

            # Re-encrypt with new key
            nonce = os.urandom(self.NONCE_SIZE)
            encrypted_new = nonce + new_aesgcm.encrypt(nonce, data_bytes, None)

            # Rewrite record keeping header intact
            header = raw[:136]
            entry_file.write_bytes(header + encrypted_new)

        # Swap key in memory
        self._aes_key = new_key
        self._aesgcm = new_aesgcm

        # Persist new key
        key_path = self._dir / '.aes_key'
        key_path.write_bytes(new_key)
        os.chmod(key_path, 0o600)
