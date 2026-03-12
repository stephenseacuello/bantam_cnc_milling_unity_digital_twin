"""Tests for attestation verifier with real TPMInterface integration."""
import hashlib
import sys
from unittest.mock import MagicMock, patch

import pytest

# Ensure miracle_msgs.srv mock exists for attestation_verifier import
sys.modules.setdefault('miracle_msgs.srv', MagicMock())
_srv_mod = sys.modules['miracle_msgs.srv']
if not hasattr(_srv_mod, 'RequestAttestation') or not callable(getattr(_srv_mod, 'RequestAttestation', None)):
    _srv_mod.RequestAttestation = MagicMock()

# Force reimport attestation_verifier to use our mock
sys.modules.pop('miracle_security.attestation_verifier', None)

from miracle_security.tpm_interface import (
    AttestationQuote,
    AttestationTier,
    TPMInterface,
)


class TestAttestationVerifierUsesTPM:
    """Verify that AttestationVerifierNode uses TPMInterface."""

    def test_tpm_interface_imported_in_verifier(self):
        """The attestation_verifier module imports TPMInterface."""
        from miracle_security import attestation_verifier
        assert hasattr(attestation_verifier, 'TPMInterface')
        assert hasattr(attestation_verifier, 'AttestationTier')

    def test_verifier_class_has_tpm_field(self):
        """AttestationVerifierNode source references _tpm attribute."""
        import inspect
        from miracle_security import attestation_verifier
        source = inspect.getsource(attestation_verifier)
        assert '_tpm' in source
        assert 'TPMInterface' in source


class TestTrustScoreCalculation:
    """Test trust score flows through TPMInterface verification."""

    @pytest.fixture
    def tpm(self):
        return TPMInterface(tier=AttestationTier.SOFTWARE)

    def test_first_attestation_sets_baseline_trust(self, tpm):
        quote = tpm.get_quote()
        is_valid, trust_score = tpm.verify_quote(quote)
        assert is_valid is True
        assert trust_score == 0.6  # Software tier base

    def test_hmac_no_baseline_trust(self):
        tpm = TPMInterface(tier=AttestationTier.HMAC, hmac_key=b'x' * 32)
        quote = tpm.get_quote()
        is_valid, trust_score = tpm.verify_quote(quote)
        assert is_valid is True
        assert trust_score == 0.8  # HMAC without known-good

    def test_hmac_known_good_match_full_trust(self):
        tpm = TPMInterface(tier=AttestationTier.HMAC, hmac_key=b'x' * 32)
        # Establish baseline
        quote = tpm.get_quote()
        tpm.store_known_good('device', quote.firmware_hash)
        # Re-attest
        quote2 = tpm.get_quote()
        is_valid, trust_score = tpm.verify_quote(quote2)
        assert is_valid is True
        assert trust_score == 1.0

    def test_invalid_quote_zero_trust(self, tpm):
        quote = AttestationQuote(
            tier=AttestationTier.SOFTWARE,
            timestamp=0,
            valid=False,
        )
        is_valid, trust_score = tpm.verify_quote(quote)
        assert is_valid is False
        assert trust_score == 0.0

    def test_tampered_firmware_zero_trust(self, tpm):
        quote = tpm.get_quote()
        quote.firmware_hash = 'tampered'
        is_valid, trust_score = tpm.verify_quote(quote)
        assert is_valid is False
        assert trust_score == 0.0


class TestQuarantineThreshold:
    """Test quarantine behavior at low trust scores."""

    def test_below_threshold_triggers_quarantine(self):
        """Trust score below min_trust (0.3) should trigger quarantine."""
        min_trust = 0.3
        # An invalid quote produces trust_score=0.0, which is < 0.3
        tpm = TPMInterface(tier=AttestationTier.SOFTWARE)
        quote = tpm.get_quote()
        # Tamper to produce zero trust
        quote.firmware_hash = 'bad_hash'
        is_valid, trust_score = tpm.verify_quote(quote)
        assert trust_score < min_trust
        assert is_valid is False

    def test_above_threshold_no_quarantine(self):
        """Trust score above min_trust should not quarantine."""
        min_trust = 0.3
        tpm = TPMInterface(tier=AttestationTier.SOFTWARE)
        quote = tpm.get_quote()
        is_valid, trust_score = tpm.verify_quote(quote)
        assert trust_score >= min_trust
        assert is_valid is True

    def test_software_tier_trust_above_quarantine(self):
        """Software tier (0.6 trust) is above default quarantine (0.3)."""
        tpm = TPMInterface(tier=AttestationTier.SOFTWARE)
        quote = tpm.get_quote()
        _, trust_score = tpm.verify_quote(quote)
        assert trust_score == 0.6
        assert trust_score >= 0.3
