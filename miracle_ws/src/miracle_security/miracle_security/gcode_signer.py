"""
G-Code Signer — Ed25519 signing and verification for G-code programs.

Signs .nc files by computing SHA256 of content and signing with Ed25519.
Appends signature as a comment block: ; MIRACLE_SIG:<base64_signature>

Usage:
    from miracle_security.gcode_signer import GCodeSigner
    signer = GCodeSigner()
    signer.generate_keypair('/path/to/keyfile')
    signer.sign_file('/path/to/program.nc', '/path/to/keyfile')
    result = signer.verify_text(gcode_text, public_key_bytes)
"""

import base64
import hashlib
import os
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

try:
    from cryptography.hazmat.primitives.asymmetric.ed25519 import (
        Ed25519PrivateKey, Ed25519PublicKey,
    )
    from cryptography.hazmat.primitives import serialization
    from cryptography.exceptions import InvalidSignature
    HAS_CRYPTOGRAPHY = True
except ImportError:
    HAS_CRYPTOGRAPHY = False


class SignatureResult(Enum):
    """Result of G-code signature verification."""
    VALID = "valid"
    INVALID = "invalid"
    MISSING = "missing"


@dataclass
class SignatureInfo:
    """Information extracted from a signature block."""
    signature_b64: str
    content_hash_hex: str = ""
    signer_id: str = ""
    timestamp: str = ""


SIGNATURE_PREFIX = "; MIRACLE_SIG:"
HASH_PREFIX = "; MIRACLE_HASH:"
SIGNER_PREFIX = "; MIRACLE_SIGNER:"
TIMESTAMP_PREFIX = "; MIRACLE_SIGNED_AT:"
SIG_BLOCK_START = "; --- MIRACLE SIGNATURE BLOCK ---"
SIG_BLOCK_END = "; --- END SIGNATURE BLOCK ---"


class GCodeSigner:
    """Ed25519 G-code signing and verification."""

    def generate_keypair(self, key_path: str) -> Tuple[bytes, bytes]:
        """Generate an Ed25519 key pair and save to files.

        Args:
            key_path: Base path for key files. Creates {key_path}.key (private)
                     and {key_path}.pub (public).

        Returns:
            Tuple of (private_key_pem, public_key_pem).
        """
        if not HAS_CRYPTOGRAPHY:
            raise RuntimeError("cryptography library required: pip install cryptography")

        private_key = Ed25519PrivateKey.generate()
        public_key = private_key.public_key()

        private_pem = private_key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.PKCS8,
            encryption_algorithm=serialization.NoEncryption(),
        )
        public_pem = public_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo,
        )

        # Write key files
        private_path = f"{key_path}.key"
        public_path = f"{key_path}.pub"

        os.makedirs(os.path.dirname(key_path) or '.', exist_ok=True)

        with open(private_path, 'wb') as f:
            f.write(private_pem)
        os.chmod(private_path, 0o600)

        with open(public_path, 'wb') as f:
            f.write(public_pem)

        return private_pem, public_pem

    def sign_text(
        self, gcode_text: str, private_key_pem: bytes,
        signer_id: str = "miracle",
    ) -> str:
        """Sign G-code text and return the signed version with appended signature block.

        Args:
            gcode_text: The G-code program text.
            private_key_pem: Ed25519 private key in PEM format.
            signer_id: Identifier for the signer.

        Returns:
            G-code text with appended signature block.
        """
        if not HAS_CRYPTOGRAPHY:
            raise RuntimeError("cryptography library required")

        # Strip any existing signature block
        clean_text = self._strip_signature_block(gcode_text)

        # Compute SHA256 of clean content
        content_hash = hashlib.sha256(clean_text.encode('utf-8')).digest()
        content_hash_hex = content_hash.hex()

        # Sign with Ed25519
        private_key = serialization.load_pem_private_key(private_key_pem, password=None)
        signature = private_key.sign(content_hash)
        sig_b64 = base64.b64encode(signature).decode('ascii')

        timestamp = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

        # Append signature block (includes SHA256 hash for clients without Ed25519)
        sig_block = (
            f"\n{SIG_BLOCK_START}\n"
            f"{SIGNATURE_PREFIX}{sig_b64}\n"
            f"{HASH_PREFIX}{content_hash_hex}\n"
            f"{SIGNER_PREFIX}{signer_id}\n"
            f"{TIMESTAMP_PREFIX}{timestamp}\n"
            f"{SIG_BLOCK_END}\n"
        )

        return clean_text + sig_block

    def sign_file(
        self, nc_path: str, key_path: str,
        signer_id: str = "miracle",
    ) -> str:
        """Sign a .nc file in-place.

        Args:
            nc_path: Path to the G-code file.
            key_path: Base path to key files (looks for {key_path}.key).
            signer_id: Identifier for the signer.

        Returns:
            Path to the signed file.
        """
        with open(nc_path, 'r') as f:
            gcode_text = f.read()

        private_path = f"{key_path}.key"
        with open(private_path, 'rb') as f:
            private_key_pem = f.read()

        signed = self.sign_text(gcode_text, private_key_pem, signer_id)

        with open(nc_path, 'w') as f:
            f.write(signed)

        return nc_path

    def verify_text(
        self, gcode_text: str, public_key_pem: bytes,
    ) -> SignatureResult:
        """Verify the signature of a G-code program.

        Args:
            gcode_text: The signed G-code text.
            public_key_pem: Ed25519 public key in PEM format.

        Returns:
            SignatureResult.VALID, INVALID, or MISSING.
        """
        if not HAS_CRYPTOGRAPHY:
            # Without cryptography library, can't verify
            return SignatureResult.MISSING

        sig_info = self._extract_signature(gcode_text)
        if sig_info is None:
            return SignatureResult.MISSING

        # Get clean content (without signature block)
        clean_text = self._strip_signature_block(gcode_text)
        content_hash = hashlib.sha256(clean_text.encode('utf-8')).digest()

        try:
            signature = base64.b64decode(sig_info.signature_b64)
            public_key = serialization.load_pem_public_key(public_key_pem)
            public_key.verify(signature, content_hash)
            return SignatureResult.VALID
        except (InvalidSignature, Exception):
            return SignatureResult.INVALID

    def verify_file(self, nc_path: str, key_path: str) -> SignatureResult:
        """Verify a signed .nc file.

        Args:
            nc_path: Path to the G-code file.
            key_path: Base path to key files (looks for {key_path}.pub).

        Returns:
            SignatureResult.
        """
        with open(nc_path, 'r') as f:
            gcode_text = f.read()

        public_path = f"{key_path}.pub"
        with open(public_path, 'rb') as f:
            public_key_pem = f.read()

        return self.verify_text(gcode_text, public_key_pem)

    def _extract_signature(self, text: str) -> Optional[SignatureInfo]:
        """Extract signature info from G-code text."""
        info = SignatureInfo(signature_b64="")

        for line in text.split('\n'):
            line = line.strip()
            if line.startswith(SIGNATURE_PREFIX):
                info.signature_b64 = line[len(SIGNATURE_PREFIX):]
            elif line.startswith(HASH_PREFIX):
                info.content_hash_hex = line[len(HASH_PREFIX):]
            elif line.startswith(SIGNER_PREFIX):
                info.signer_id = line[len(SIGNER_PREFIX):]
            elif line.startswith(TIMESTAMP_PREFIX):
                info.timestamp = line[len(TIMESTAMP_PREFIX):]

        if not info.signature_b64:
            return None
        return info

    def _strip_signature_block(self, text: str) -> str:
        """Remove signature block from G-code text."""
        lines = text.split('\n')
        result = []
        in_sig_block = False

        for line in lines:
            stripped = line.strip()
            if stripped == SIG_BLOCK_START:
                in_sig_block = True
                continue
            if stripped == SIG_BLOCK_END:
                in_sig_block = False
                continue
            if in_sig_block:
                continue
            result.append(line)

        # Remove trailing empty lines
        while result and result[-1].strip() == '':
            result.pop()

        return '\n'.join(result)
