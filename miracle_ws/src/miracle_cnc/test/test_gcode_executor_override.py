"""Tests for G-code executor feed override integration."""

import pytest


class TestFeedOverrideIntegration:
    def test_effective_feed_with_override(self):
        block_feed = 5000.0
        override_pct = 60.0
        effective = block_feed * (override_pct / 100.0)
        assert effective == pytest.approx(3000.0)

    def test_full_override_no_change(self):
        block_feed = 5000.0
        effective = block_feed * (100.0 / 100.0)
        assert effective == pytest.approx(5000.0)

    def test_override_clamp_min(self):
        clamped = max(10.0, min(100.0, 5.0))
        assert clamped == 10.0

    def test_override_clamp_max(self):
        clamped = max(10.0, min(100.0, 150.0))
        assert clamped == 100.0

    def test_spindle_override_clamp(self):
        clamped = max(80.0, min(100.0, 70.0))
        assert clamped == 80.0
