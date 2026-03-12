"""
Secure Storage — AES-256-GCM encrypted, Ed25519 signed audit log storage.

Each entry: [nonce(12)][ciphertext][tag(16)]
Hash chain: sign(prev_hash || entry_hash || seq_number)
"""

import hashlib
import json
import os
import struct
import time
from dataclasses import dataclass
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
