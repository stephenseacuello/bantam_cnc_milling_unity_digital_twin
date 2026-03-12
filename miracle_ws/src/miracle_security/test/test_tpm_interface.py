"""Tests for TPM interface and attestation tiers."""
import pytest
from miracle_security.tpm_interface import (
    AttestationQuote,
    AttestationTier,
    TPMInterface,
)


class TestSoftwareTierQuote:
    """Test software tier quote generation and verification."""

    @pytest.fixture
    def tpm(self):
        return TPMInterface(tier=AttestationTier.SOFTWARE)

    def test_quote_has_correct_tier(self, tpm):
        quote = tpm.get_quote()
        assert quote.tier == AttestationTier.SOFTWARE

    def test_quote_is_valid(self, tpm):
        quote = tpm.get_quote()
        assert quote.valid is True

    def test_quote_has_firmware_hash(self, tpm):
        quote = tpm.get_quote()
        assert len(quote.firmware_hash) == 64  # SHA-256 hex digest

    def test_quote_has_pcr_values(self, tpm):
        quote = tpm.get_quote()
        assert len(quote.pcr_values) == 4
        assert all(idx in quote.pcr_values for idx in [0, 1, 2, 7])

    def test_quote_has_signature(self, tpm):
        quote = tpm.get_quote()
        assert len(quote.signature) == 32  # HMAC-SHA256

    def test_quote_has_nonce(self, tpm):
        quote = tpm.get_quote(nonce='test_nonce_123')
        assert quote.nonce == 'test_nonce_123'

    def test_quote_has_timestamp(self, tpm):
        quote = tpm.get_quote()
        assert quote.timestamp > 0

    def test_verify_valid_quote(self, tpm):
        quote = tpm.get_quote()
        is_valid, trust_score = tpm.verify_quote(quote)
        assert is_valid is True
        assert trust_score == 0.6  # Software tier base trust

    def test_verify_tampered_quote_fails(self, tpm):
        quote = tpm.get_quote()
        quote.firmware_hash = 'tampered_hash'
        is_valid, trust_score = tpm.verify_quote(quote)
        assert is_valid is False
        assert trust_score == 0.0

    def test_verify_invalid_quote(self, tpm):
        quote = AttestationQuote(
            tier=AttestationTier.SOFTWARE,
            timestamp=0,
            valid=False,
        )
        is_valid, trust_score = tpm.verify_quote(quote)
        assert is_valid is False
        assert trust_score == 0.0


class TestHMACTierQuote:
    """Test HMAC tier quote generation and verification."""

    @pytest.fixture
    def hmac_key(self):
        return b'test_shared_secret_32bytes______'

    @pytest.fixture
    def tpm(self, hmac_key):
        return TPMInterface(tier=AttestationTier.HMAC, hmac_key=hmac_key)

    def test_quote_has_correct_tier(self, tpm):
        quote = tpm.get_quote()
        assert quote.tier == AttestationTier.HMAC

    def test_quote_is_valid(self, tpm):
        quote = tpm.get_quote()
        assert quote.valid is True

    def test_quote_has_firmware_hash(self, tpm):
        quote = tpm.get_quote()
        assert len(quote.firmware_hash) == 64

    def test_verify_valid_quote(self, tpm):
        quote = tpm.get_quote()
        is_valid, trust_score = tpm.verify_quote(quote)
        assert is_valid is True
        assert trust_score == 0.8  # No known-good baseline

    def test_verify_tampered_signature_fails(self, tpm):
        quote = tpm.get_quote()
        quote.signature = b'\x00' * 32
        is_valid, trust_score = tpm.verify_quote(quote)
        assert is_valid is False
        assert trust_score == 0.0

    def test_verify_with_wrong_key(self, hmac_key):
        tpm1 = TPMInterface(tier=AttestationTier.HMAC, hmac_key=hmac_key)
        tpm2 = TPMInterface(
            tier=AttestationTier.HMAC, hmac_key=b'different_key_______________xxxx'
        )
        quote = tpm1.get_quote()
        is_valid, trust_score = tpm2.verify_quote(quote)
        assert is_valid is False
        assert trust_score == 0.0


class TestKnownGoodHashes:
    """Test known-good hash matching."""

    def test_known_good_match_gives_full_trust(self):
        tpm = TPMInterface(tier=AttestationTier.HMAC, hmac_key=b'k' * 32)
        # Get a quote to learn the firmware hash
        quote = tpm.get_quote()
        firmware_hash = quote.firmware_hash
        # Store it as known good
        tpm.store_known_good('controller', firmware_hash)
        # Re-verify — now known-good matches
        quote2 = tpm.get_quote()
        is_valid, trust_score = tpm.verify_quote(quote2)
        assert is_valid is True
        assert trust_score == 1.0

    def test_known_good_mismatch_gives_lower_trust(self):
        tpm = TPMInterface(
            tier=AttestationTier.HMAC,
            hmac_key=b'k' * 32,
            known_good_hashes={'controller': 'some_other_hash'},
        )
        quote = tpm.get_quote()
        is_valid, trust_score = tpm.verify_quote(quote)
        assert is_valid is True
        assert trust_score == 0.5  # Valid HMAC but unknown firmware

    def test_store_known_good(self):
        tpm = TPMInterface(tier=AttestationTier.SOFTWARE)
        tpm.store_known_good('device_a', 'abc123')
        assert tpm._known_good['device_a'] == 'abc123'


class TestNonceUniqueness:
    """Test that nonces are unique across quotes."""

    def test_auto_generated_nonces_are_unique(self):
        tpm = TPMInterface(tier=AttestationTier.SOFTWARE)
        nonces = set()
        for _ in range(50):
            quote = tpm.get_quote()
            nonces.add(quote.nonce)
        assert len(nonces) == 50

    def test_explicit_nonce_is_used(self):
        tpm = TPMInterface(tier=AttestationTier.SOFTWARE)
        quote = tpm.get_quote(nonce='my_custom_nonce')
        assert quote.nonce == 'my_custom_nonce'


class TestTierProperty:
    """Test tier property accessor."""

    def test_software_tier(self):
        tpm = TPMInterface(tier=AttestationTier.SOFTWARE)
        assert tpm.tier == AttestationTier.SOFTWARE

    def test_hmac_tier(self):
        tpm = TPMInterface(tier=AttestationTier.HMAC)
        assert tpm.tier == AttestationTier.HMAC
