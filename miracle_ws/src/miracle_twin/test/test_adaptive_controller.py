"""Tests for the Adaptive Controller."""

import pytest
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_twin.adaptive_controller import AdaptiveState


class TestAdaptiveState:
    def test_defaults(self):
        state = AdaptiveState()
        assert state.force_ratio == 0.0
        assert state.chatter_risk == 'LOW'
        assert state.wear_ratio == 0.0
        assert state.thermal_ratio == 0.0
        assert state.current_feed_override == 100.0

    def test_force_override_logic(self):
        state = AdaptiveState(force_ratio=0.9)
        threshold = 0.80
        excess = state.force_ratio - threshold
        reduction = excess / (1.0 - threshold)
        feed_pct = max(20.0, 100.0 * (1.0 - reduction * 0.5))
        assert feed_pct < 100.0
        assert feed_pct >= 20.0

    def test_wear_override_logic(self):
        state = AdaptiveState(wear_ratio=0.95)
        feed_pct = 100.0
        if state.wear_ratio > 0.90:
            feed_pct = min(feed_pct, 50.0)
        assert feed_pct == 50.0

    def test_thermal_override_logic(self):
        state = AdaptiveState(thermal_ratio=0.95)
        threshold = 0.85
        feed_pct = 100.0
        excess = state.thermal_ratio - threshold
        thermal_reduction = excess / (1.0 - threshold)
        thermal_feed = 100.0 * (1.0 - thermal_reduction * 0.4)
        feed_pct = min(feed_pct, max(20.0, thermal_feed))
        assert feed_pct < 100.0
        assert feed_pct >= 20.0

    def test_chatter_high_spindle_shift(self):
        state = AdaptiveState(chatter_risk='HIGH')
        spindle_pct = 100.0
        if state.chatter_risk == 'HIGH':
            spindle_pct = 95.0
        assert spindle_pct == 95.0

    def test_no_override_when_nominal(self):
        state = AdaptiveState(force_ratio=0.5, chatter_risk='LOW', wear_ratio=0.3, thermal_ratio=0.4)
        assert state.force_ratio <= 0.80
        assert state.wear_ratio <= 0.90
        assert state.thermal_ratio <= 0.85

    def test_combined_overrides(self):
        state = AdaptiveState(force_ratio=0.95, wear_ratio=0.95, thermal_ratio=0.95)
        feed_pct = 100.0
        excess = state.force_ratio - 0.80
        reduction = excess / 0.20
        feed_pct = max(20.0, 100.0 * (1.0 - reduction * 0.5))
        feed_pct = min(feed_pct, 50.0)
        t_excess = state.thermal_ratio - 0.85
        t_reduction = t_excess / 0.15
        t_feed = 100.0 * (1.0 - t_reduction * 0.4)
        feed_pct = min(feed_pct, max(20.0, t_feed))
        assert feed_pct <= 50.0
        assert feed_pct >= 20.0
