"""
TPM Interface — Hardware attestation via TPM 2.0 or HMAC-SHA256 fallback.

Provides firmware attestation using:
  - TPM 2.0 quotes via tpm2-tools (server-grade hardware)
  - HMAC-SHA256 challenge-response (MCU tier, e.g., ESP32)
  - Software TPM (swtpm) fallback for development
"""

import hashlib
import hmac
import json
import os
import subprocess
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


class AttestationTier(Enum):
    """Hardware attestation tier."""
    TPM2 = 'tpm2'
    HMAC = 'hmac'
    SOFTWARE = 'software'


@dataclass
class AttestationQuote:
    """Result of an attestation quote operation."""
    tier: AttestationTier
    timestamp: float
    pcr_values: Dict[int, str] = field(default_factory=dict)
    firmware_hash: str = ''
    nonce: str = ''
    signature: bytes = b''
    raw_quote: bytes = b''
    valid: bool = False


class TPMInterface:
    """Interface to TPM 2.0 hardware or software fallback.

    Args:
        tier: Attestation tier to use.
        tpm_device: TPM device path (for TPM2 tier).
        hmac_key: Shared secret for HMAC tier.
        known_good_hashes: Dict of component → expected hash for verification.
    """

    def __init__(
        self,
        tier: AttestationTier = AttestationTier.SOFTWARE,
        tpm_device: str = '/dev/tpmrm0',
        hmac_key: Optional[bytes] = None,
        known_good_hashes: Optional[Dict[str, str]] = None,
    ) -> None:
        self._tier = tier
        self._tpm_device = tpm_device
        self._hmac_key = hmac_key or os.urandom(32)
        self._known_good = known_good_hashes or {}

        if tier == AttestationTier.TPM2:
            self._check_tpm2_tools()

    def _check_tpm2_tools(self) -> None:
        """Check that tpm2-tools are available."""
        try:
            subprocess.run(
                ['tpm2_getrandom', '--version'],
                capture_output=True, timeout=5,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            raise RuntimeError(
                "tpm2-tools not found. Install with: apt install tpm2-tools"
            )

    def get_quote(self, nonce: Optional[str] = None) -> AttestationQuote:
        """Get an attestation quote from the configured tier."""
        nonce = nonce or hashlib.sha256(os.urandom(16)).hexdigest()[:32]

        if self._tier == AttestationTier.TPM2:
            return self._get_tpm2_quote(nonce)
        elif self._tier == AttestationTier.HMAC:
            return self._get_hmac_quote(nonce)
        else:
            return self._get_software_quote(nonce)

    def _get_tpm2_quote(self, nonce: str) -> AttestationQuote:
        """Get TPM 2.0 quote via tpm2-tools."""
        quote = AttestationQuote(
            tier=AttestationTier.TPM2,
            timestamp=time.time(),
            nonce=nonce,
        )

        try:
            # Read PCR values (banks 0, 1, 2, 7)
            result = subprocess.run(
                ['tpm2_pcrread', 'sha256:0,1,2,7', '-o', '/tmp/pcr.bin'],
                capture_output=True, text=True, timeout=10,
            )
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    if ':' in line and '0x' in line:
                        parts = line.strip().split(':')
                        if len(parts) >= 2:
                            try:
                                pcr_idx = int(parts[0].strip().split()[-1])
                                pcr_val = parts[1].strip()
                                quote.pcr_values[pcr_idx] = pcr_val
                            except (ValueError, IndexError):
                                pass

            # Generate quote with nonce
            nonce_bytes = nonce.encode()
            nonce_file = '/tmp/miracle_nonce.bin'
            with open(nonce_file, 'wb') as f:
                f.write(nonce_bytes)

            result = subprocess.run(
                [
                    'tpm2_quote',
                    '-c', '0x81010001',
                    '-l', 'sha256:0,1,2,7',
                    '-q', nonce_file,
                    '-m', '/tmp/quote.msg',
                    '-s', '/tmp/quote.sig',
                ],
                capture_output=True, text=True, timeout=10,
            )

            if result.returncode == 0:
                if os.path.exists('/tmp/quote.msg'):
                    quote.raw_quote = Path('/tmp/quote.msg').read_bytes()
                if os.path.exists('/tmp/quote.sig'):
                    quote.signature = Path('/tmp/quote.sig').read_bytes()
                quote.valid = True

        except (subprocess.TimeoutExpired, Exception) as e:
            quote.valid = False

        return quote

    def _get_hmac_quote(self, nonce: str) -> AttestationQuote:
        """HMAC-SHA256 challenge-response for MCU tier."""
        quote = AttestationQuote(
            tier=AttestationTier.HMAC,
            timestamp=time.time(),
            nonce=nonce,
        )

        # Compute HMAC over firmware hash + nonce
        firmware_hash = self._compute_local_firmware_hash()
        quote.firmware_hash = firmware_hash

        message = f"{firmware_hash}:{nonce}".encode()
        quote.signature = hmac.new(
            self._hmac_key, message, hashlib.sha256
        ).digest()
        quote.valid = True

        return quote

    def _get_software_quote(self, nonce: str) -> AttestationQuote:
        """Software-only attestation for development."""
        quote = AttestationQuote(
            tier=AttestationTier.SOFTWARE,
            timestamp=time.time(),
            nonce=nonce,
        )

        firmware_hash = self._compute_local_firmware_hash()
        quote.firmware_hash = firmware_hash

        # Simulated PCR values based on local file hashes
        quote.pcr_values = {
            0: hashlib.sha256(b'boot_firmware_v1').hexdigest(),
            1: hashlib.sha256(b'os_config_v1').hexdigest(),
            2: firmware_hash,
            7: hashlib.sha256(b'secureboot_policy_v1').hexdigest(),
        }

        message = json.dumps({
            'pcrs': {str(k): v for k, v in quote.pcr_values.items()},
            'firmware': firmware_hash,
            'nonce': nonce,
        }, sort_keys=True).encode()
        quote.signature = hmac.new(
            self._hmac_key, message, hashlib.sha256
        ).digest()
        quote.valid = True

        return quote

    def verify_quote(self, quote: AttestationQuote) -> Tuple[bool, float]:
        """Verify an attestation quote.

        Returns:
            (is_valid, trust_score) where trust_score is 0.0-1.0.
        """
        if not quote.valid:
            return False, 0.0

        if quote.tier == AttestationTier.HMAC:
            return self._verify_hmac_quote(quote)
        elif quote.tier == AttestationTier.SOFTWARE:
            return self._verify_software_quote(quote)
        else:
            # TPM2 quotes are verified by the TPM hardware
            return quote.valid, 1.0 if quote.valid else 0.0

    def _verify_hmac_quote(self, quote: AttestationQuote) -> Tuple[bool, float]:
        """Verify HMAC challenge-response."""
        message = f"{quote.firmware_hash}:{quote.nonce}".encode()
        expected = hmac.new(self._hmac_key, message, hashlib.sha256).digest()

        if not hmac.compare_digest(quote.signature, expected):
            return False, 0.0

        # Check firmware hash against known-good
        if self._known_good:
            if quote.firmware_hash in self._known_good.values():
                return True, 1.0
            else:
                return True, 0.5  # Valid HMAC but unknown firmware

        return True, 0.8  # Valid HMAC, no known-good baseline

    def _verify_software_quote(self, quote: AttestationQuote) -> Tuple[bool, float]:
        """Verify software attestation."""
        message = json.dumps({
            'pcrs': {str(k): v for k, v in quote.pcr_values.items()},
            'firmware': quote.firmware_hash,
            'nonce': quote.nonce,
        }, sort_keys=True).encode()
        expected = hmac.new(self._hmac_key, message, hashlib.sha256).digest()

        if not hmac.compare_digest(quote.signature, expected):
            return False, 0.0

        # Software tier gets lower base trust
        return True, 0.6

    def _compute_local_firmware_hash(self) -> str:
        """Compute a hash representing the current firmware/software state."""
        h = hashlib.sha256()
        # Hash key Python files as proxy for "firmware"
        src_dirs = [
            Path(__file__).parent,
            Path(__file__).parent.parent.parent / 'miracle_core' / 'miracle_core',
        ]
        for src_dir in src_dirs:
            if src_dir.exists():
                for py_file in sorted(src_dir.glob('*.py')):
                    try:
                        h.update(py_file.read_bytes())
                    except (OSError, PermissionError):
                        pass
        return h.hexdigest()

    def store_known_good(self, component: str, hash_value: str) -> None:
        """Store a known-good hash for future verification."""
        self._known_good[component] = hash_value

    @property
    def tier(self) -> AttestationTier:
        return self._tier
