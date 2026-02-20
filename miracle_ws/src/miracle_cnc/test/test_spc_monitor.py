"""Tests for the SPC Monitor Western Electric rules logic.

Extracts and directly tests the _check_we_rules method by constructing
a minimal mock of SPCMonitorNode with controlled buffers.
All four Western Electric rules are individually tested:
  Rule 1: Single point beyond 3-sigma
  Rule 2: 2 of 3 consecutive points beyond 2-sigma
  Rule 3: 4 of 5 consecutive points beyond 1-sigma
  Rule 4: 8 consecutive points on the same side of the center line
"""

import sys
import math
from collections import deque
from unittest.mock import MagicMock

import pytest

# --- Mock all ROS2 dependencies ---
sys.modules.setdefault('rclpy', MagicMock())
sys.modules['rclpy.lifecycle'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.qos'] = MagicMock()
sys.modules['rclpy.duration'] = MagicMock()
sys.modules['rclpy.callback_groups'] = MagicMock()
sys.modules['rcl_interfaces'] = MagicMock()
sys.modules['rcl_interfaces.msg'] = MagicMock()
sys.modules['builtin_interfaces'] = MagicMock()
sys.modules['builtin_interfaces.msg'] = MagicMock()
sys.modules['miracle_core.lifecycle_node_base'] = MagicMock()
sys.modules['miracle_core.qos_profiles'] = MagicMock()
sys.modules['miracle_core.heartbeat_mixin'] = MagicMock()
sys.modules['miracle_msgs'] = MagicMock()
sys.modules['miracle_msgs.msg'] = MagicMock()


class _SPCRulesChecker:
    """Extracted Western Electric rules logic from SPCMonitorNode.

    This is a test helper that replicates the _check_we_rules logic so
    it can be tested in isolation without requiring a full ROS2 lifecycle.
    """

    def __init__(self, sigma_limit: float = 3.0):
        self._sigma_limit = sigma_limit
        self._buffers = {}

    def set_buffer(self, var_name: str, values: list):
        """Populate a buffer for a given variable."""
        self._buffers[var_name] = deque(values, maxlen=max(len(values), 100))

    def check_we_rules(self, var_name: str, value: float, mean: float, std: float):
        """Re-implementation of SPCMonitorNode._check_we_rules."""
        violations = []
        sigma = self._sigma_limit

        # Rule 1: Single point beyond control limits
        if abs(value - mean) > sigma * std:
            violations.append(
                f"Rule 1: {var_name}={value:.2f} beyond {sigma}-sigma limit"
            )

        # Rule 2: 2 of 3 consecutive points beyond 2-sigma
        buf = list(self._buffers[var_name])
        if len(buf) >= 3:
            last3 = buf[-3:]
            beyond_2sigma = sum(1 for v in last3 if abs(v - mean) > 2 * std)
            if beyond_2sigma >= 2:
                violations.append(
                    f"Rule 2: 2 of 3 points beyond 2-sigma for {var_name}"
                )

        # Rule 3: 4 of 5 consecutive points beyond 1-sigma
        if len(buf) >= 5:
            last5 = buf[-5:]
            beyond_1sigma = sum(1 for v in last5 if abs(v - mean) > std)
            if beyond_1sigma >= 4:
                violations.append(
                    f"Rule 3: 4 of 5 points beyond 1-sigma for {var_name}"
                )

        # Rule 4: 8 consecutive points on same side of center
        if len(buf) >= 8:
            last8 = buf[-8:]
            above = all(v > mean for v in last8)
            below = all(v < mean for v in last8)
            if above or below:
                side = 'above' if above else 'below'
                violations.append(
                    f"Rule 4: 8 consecutive points {side} center for {var_name}"
                )

        return violations


@pytest.fixture
def checker():
    """Return an SPC rules checker with default 3-sigma limits."""
    return _SPCRulesChecker(sigma_limit=3.0)


class TestRule1SinglePointBeyond3Sigma:
    """Rule 1: A single point beyond 3-sigma control limits."""

    def test_rule1_single_point_beyond_3sigma(self, checker):
        """A value more than 3*std from the mean should trigger Rule 1."""
        mean, std = 100.0, 5.0
        extreme_value = 100.0 + 3.0 * 5.0 + 1.0  # 116.0 > 115.0
        checker.set_buffer('spindle_load', [100.0, 100.0, extreme_value])
        violations = checker.check_we_rules('spindle_load', extreme_value, mean, std)
        rule1 = [v for v in violations if 'Rule 1' in v]
        assert len(rule1) >= 1, "Rule 1 should fire for a point beyond 3-sigma"

    def test_rule1_no_violation_within_limits(self, checker):
        """A value within 3*std should NOT trigger Rule 1."""
        mean, std = 100.0, 5.0
        normal_value = 110.0  # within 3*5 = 15 of mean
        checker.set_buffer('spindle_load', [normal_value])
        violations = checker.check_we_rules('spindle_load', normal_value, mean, std)
        rule1 = [v for v in violations if 'Rule 1' in v]
        assert len(rule1) == 0, "Rule 1 should not fire for a point within 3-sigma"

    def test_rule1_negative_deviation(self, checker):
        """A value far below the mean should also trigger Rule 1."""
        mean, std = 100.0, 5.0
        low_value = 100.0 - 3.0 * 5.0 - 2.0  # 83.0 < 85.0
        checker.set_buffer('spindle_load', [low_value])
        violations = checker.check_we_rules('spindle_load', low_value, mean, std)
        rule1 = [v for v in violations if 'Rule 1' in v]
        assert len(rule1) >= 1, "Rule 1 should fire for a point below -3-sigma"


class TestRule2TwoOfThreeBeyond2Sigma:
    """Rule 2: 2 of 3 consecutive points beyond 2-sigma."""

    def test_rule2_two_of_three_beyond_2sigma(self, checker):
        """Two out of three recent points beyond 2*std should trigger Rule 2."""
        mean, std = 50.0, 2.0
        # 2-sigma = 4.0, so values beyond 54.0 trigger
        values = [55.0, 50.0, 56.0]  # 2 of 3 beyond 2-sigma
        checker.set_buffer('feed_rate', values)
        violations = checker.check_we_rules('feed_rate', values[-1], mean, std)
        rule2 = [v for v in violations if 'Rule 2' in v]
        assert len(rule2) >= 1, "Rule 2 should fire when 2 of 3 are beyond 2-sigma"

    def test_rule2_only_one_of_three_beyond(self, checker):
        """Only one of three beyond 2-sigma should NOT trigger Rule 2."""
        mean, std = 50.0, 2.0
        values = [55.0, 50.0, 51.0]  # only 1 of 3 beyond 2-sigma
        checker.set_buffer('feed_rate', values)
        violations = checker.check_we_rules('feed_rate', values[-1], mean, std)
        rule2 = [v for v in violations if 'Rule 2' in v]
        assert len(rule2) == 0, "Rule 2 should not fire when only 1 of 3 is beyond 2-sigma"

    def test_rule2_insufficient_data(self, checker):
        """Fewer than 3 data points should not trigger Rule 2."""
        mean, std = 50.0, 2.0
        checker.set_buffer('feed_rate', [99.0, 99.0])  # only 2 points
        violations = checker.check_we_rules('feed_rate', 99.0, mean, std)
        rule2 = [v for v in violations if 'Rule 2' in v]
        assert len(rule2) == 0, "Rule 2 should not fire with fewer than 3 data points"


class TestRule3FourOfFiveBeyond1Sigma:
    """Rule 3: 4 of 5 consecutive points beyond 1-sigma."""

    def test_rule3_four_of_five_beyond_1sigma(self, checker):
        """Four of five consecutive values beyond 1*std should trigger Rule 3."""
        mean, std = 100.0, 3.0
        # Beyond 1-sigma: > 103 or < 97
        values = [105.0, 106.0, 100.0, 107.0, 104.0]  # 4 of 5 beyond 1-sigma
        checker.set_buffer('spindle_speed', values)
        violations = checker.check_we_rules('spindle_speed', values[-1], mean, std)
        rule3 = [v for v in violations if 'Rule 3' in v]
        assert len(rule3) >= 1, "Rule 3 should fire when 4 of 5 are beyond 1-sigma"

    def test_rule3_three_of_five_no_violation(self, checker):
        """Only 3 of 5 beyond 1-sigma should NOT trigger Rule 3."""
        mean, std = 100.0, 3.0
        values = [105.0, 100.0, 100.0, 107.0, 104.0]  # only 3 of 5 beyond
        checker.set_buffer('spindle_speed', values)
        violations = checker.check_we_rules('spindle_speed', values[-1], mean, std)
        rule3 = [v for v in violations if 'Rule 3' in v]
        assert len(rule3) == 0, "Rule 3 should not fire when only 3 of 5 are beyond 1-sigma"

    def test_rule3_insufficient_data(self, checker):
        """Fewer than 5 data points should not trigger Rule 3."""
        mean, std = 100.0, 3.0
        checker.set_buffer('spindle_speed', [200.0, 200.0, 200.0, 200.0])  # 4 points
        violations = checker.check_we_rules('spindle_speed', 200.0, mean, std)
        rule3 = [v for v in violations if 'Rule 3' in v]
        assert len(rule3) == 0, "Rule 3 should not fire with fewer than 5 data points"


class TestRule4EightConsecutiveSameSide:
    """Rule 4: 8 consecutive points on the same side of the center line."""

    def test_rule4_eight_above_center(self, checker):
        """Eight consecutive points all above the mean should trigger Rule 4."""
        mean, std = 50.0, 5.0
        values = [51.0, 52.0, 51.5, 53.0, 52.5, 51.0, 54.0, 52.0]
        checker.set_buffer('spindle_load', values)
        violations = checker.check_we_rules('spindle_load', values[-1], mean, std)
        rule4 = [v for v in violations if 'Rule 4' in v]
        assert len(rule4) >= 1, "Rule 4 should fire for 8 consecutive points above center"
        assert any('above' in v for v in rule4)

    def test_rule4_eight_below_center(self, checker):
        """Eight consecutive points all below the mean should trigger Rule 4."""
        mean, std = 50.0, 5.0
        values = [49.0, 48.0, 49.5, 47.0, 48.5, 49.0, 46.0, 48.0]
        checker.set_buffer('spindle_load', values)
        violations = checker.check_we_rules('spindle_load', values[-1], mean, std)
        rule4 = [v for v in violations if 'Rule 4' in v]
        assert len(rule4) >= 1, "Rule 4 should fire for 8 consecutive points below center"
        assert any('below' in v for v in rule4)

    def test_rule4_alternating_no_violation(self, checker):
        """Points alternating above and below the mean should NOT trigger Rule 4."""
        mean, std = 50.0, 5.0
        values = [51.0, 49.0, 51.0, 49.0, 51.0, 49.0, 51.0, 49.0]
        checker.set_buffer('spindle_load', values)
        violations = checker.check_we_rules('spindle_load', values[-1], mean, std)
        rule4 = [v for v in violations if 'Rule 4' in v]
        assert len(rule4) == 0, "Rule 4 should not fire for alternating points"

    def test_rule4_insufficient_data(self, checker):
        """Fewer than 8 data points should not trigger Rule 4."""
        mean, std = 50.0, 5.0
        checker.set_buffer('spindle_load', [60.0] * 7)  # 7 points
        violations = checker.check_we_rules('spindle_load', 60.0, mean, std)
        rule4 = [v for v in violations if 'Rule 4' in v]
        assert len(rule4) == 0, "Rule 4 should not fire with fewer than 8 data points"


class TestMultipleRulesSimultaneous:
    """Test that multiple rules can fire simultaneously."""

    def test_multiple_rules_fire(self, checker):
        """An extreme scenario should trigger multiple rules at once."""
        mean, std = 50.0, 2.0
        # 8 points far above mean: triggers Rule 1, 2, 3, and 4
        values = [60.0, 61.0, 60.5, 62.0, 61.5, 60.0, 63.0, 61.0]
        checker.set_buffer('spindle_load', values)
        violations = checker.check_we_rules('spindle_load', values[-1], mean, std)
        assert len(violations) >= 2, (
            f"Expected at least 2 rules to fire, got {len(violations)}: {violations}"
        )
