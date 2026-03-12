"""Tests for G-code signing and verification."""
import os
import tempfile
import pytest

pytest.importorskip("cryptography", reason="cryptography package required")

from miracle_security.gcode_signer import (
    GCodeSigner, SignatureResult, SIGNATURE_PREFIX, SIG_BLOCK_START,
)

SAMPLE_GCODE = """G21 ; mm mode
G90 ; absolute
G0 X0 Y0 Z10
G1 Z-1 F500
G1 X50 Y0 F1000
G1 X50 Y50
G1 X0 Y50
G1 X0 Y0
G0 Z10
M30"""


class TestGCodeSigner:
    """Test Ed25519 signing operations."""

    @pytest.fixture
    def signer(self):
        return GCodeSigner()

    @pytest.fixture
    def keypair(self, signer):
        with tempfile.TemporaryDirectory() as tmpdir:
            key_path = os.path.join(tmpdir, 'test_key')
            private_pem, public_pem = signer.generate_keypair(key_path)
            yield key_path, private_pem, public_pem

    def test_generate_keypair_creates_files(self, signer):
        with tempfile.TemporaryDirectory() as tmpdir:
            key_path = os.path.join(tmpdir, 'test')
            signer.generate_keypair(key_path)
            assert os.path.exists(f'{key_path}.key')
            assert os.path.exists(f'{key_path}.pub')

    def test_sign_text_appends_signature_block(self, signer, keypair):
        _, private_pem, _ = keypair
        signed = signer.sign_text(SAMPLE_GCODE, private_pem)
        assert SIG_BLOCK_START in signed
        assert SIGNATURE_PREFIX in signed

    def test_verify_valid_signature(self, signer, keypair):
        _, private_pem, public_pem = keypair
        signed = signer.sign_text(SAMPLE_GCODE, private_pem)
        result = signer.verify_text(signed, public_pem)
        assert result == SignatureResult.VALID

    def test_verify_tampered_content(self, signer, keypair):
        _, private_pem, public_pem = keypair
        signed = signer.sign_text(SAMPLE_GCODE, private_pem)
        tampered = signed.replace('X50', 'X999')
        result = signer.verify_text(tampered, public_pem)
        assert result == SignatureResult.INVALID

    def test_verify_missing_signature(self, signer, keypair):
        _, _, public_pem = keypair
        result = signer.verify_text(SAMPLE_GCODE, public_pem)
        assert result == SignatureResult.MISSING

    def test_verify_wrong_key(self, signer, keypair):
        _, private_pem, _ = keypair
        signed = signer.sign_text(SAMPLE_GCODE, private_pem)
        # Generate a different key pair
        with tempfile.TemporaryDirectory() as tmpdir:
            other_path = os.path.join(tmpdir, 'other')
            _, other_public = signer.generate_keypair(other_path)
            result = signer.verify_text(signed, other_public)
            assert result == SignatureResult.INVALID

    def test_sign_file_round_trip(self, signer, keypair):
        key_path, _, _ = keypair
        with tempfile.NamedTemporaryFile(mode='w', suffix='.nc', delete=False) as f:
            f.write(SAMPLE_GCODE)
            nc_path = f.name
        try:
            signer.sign_file(nc_path, key_path)
            result = signer.verify_file(nc_path, key_path)
            assert result == SignatureResult.VALID
        finally:
            os.unlink(nc_path)

    def test_strip_signature_block(self, signer, keypair):
        _, private_pem, _ = keypair
        signed = signer.sign_text(SAMPLE_GCODE, private_pem)
        stripped = signer._strip_signature_block(signed)
        assert SIG_BLOCK_START not in stripped
        assert SIGNATURE_PREFIX not in stripped

    def test_double_sign_replaces(self, signer, keypair):
        _, private_pem, public_pem = keypair
        signed1 = signer.sign_text(SAMPLE_GCODE, private_pem)
        signed2 = signer.sign_text(signed1, private_pem)
        # Should still verify (old sig stripped before re-signing)
        result = signer.verify_text(signed2, public_pem)
        assert result == SignatureResult.VALID
        # Should only have one signature block
        assert signed2.count(SIG_BLOCK_START) == 1
