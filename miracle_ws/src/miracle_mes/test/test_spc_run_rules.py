"""Tests for SPCRunRulesEngine, RunRuleViolation, and SPCAnalysis.

Validates the SPC Run Rules Engine including:
- Rule 1: Point beyond 3 sigma
- Rule 2: 9 consecutive points same side of mean
- Rule 3: 6 consecutive increasing or decreasing
- Rule 4: 14 consecutive alternating up/down
- Rule 5: 2 of 3 points beyond 2 sigma (same side)
- Rule 6: 4 of 5 points beyond 1 sigma (same side)
- Rule 7: 15 consecutive within 1 sigma (stratification)
- Rule 8: 8 consecutive beyond 1 sigma both sides (mixture)
- Full analysis with in_control detection
- Rule description lookup
"""

import os
import sys
import time
from types import ModuleType
from typing import Any, Dict, List, Optional
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Ensure the miracle_mes source directory is on sys.path
# ---------------------------------------------------------------------------
_MES_SRC = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir)
)
if _MES_SRC not in sys.path:
    sys.path.insert(0, _MES_SRC)


# ---------------------------------------------------------------------------
# Stub modules (ROS2 / miracle_core / miracle_msgs)
# ---------------------------------------------------------------------------

# Pre-import miracle_mes so it is the real package, not a stub module.
import miracle_mes  # noqa: E402, F401


def _ensure_module(dotted_name: str) -> ModuleType:
    parts = dotted_name.split('.')
    for i in range(1, len(parts) + 1):
        partial = '.'.join(parts[:i])
        if partial not in sys.modules:
            sys.modules[partial] = ModuleType(partial)
    return sys.modules[dotted_name]


# rclpy stubs
_rclpy = _ensure_module('rclpy')
_rclpy_lifecycle = _ensure_module('rclpy.lifecycle')
_rclpy_lifecycle.TransitionCallbackReturn = MagicMock()
_rclpy_lifecycle.TransitionCallbackReturn.SUCCESS = 'SUCCESS'
_rclpy_action = _ensure_module('rclpy.action')
_rclpy_action.ActionServer = MagicMock()
_rclpy_action.CancelResponse = MagicMock()
_rclpy_action.GoalResponse = MagicMock()
_rclpy_action_server = _ensure_module('rclpy.action.server')
_rclpy_action_server.ServerGoalHandle = MagicMock()
_rclpy_cbg = _ensure_module('rclpy.callback_groups')
_rclpy_cbg.ReentrantCallbackGroup = MagicMock()

# miracle_core stubs
_mc = _ensure_module('miracle_core')
_mc_lifecycle = _ensure_module('miracle_core.lifecycle_node_base')
_mc_qos = _ensure_module('miracle_core.qos_profiles')


class _StubLifecycleNode:
    CRITICALITY_MEDIUM = 'MEDIUM'

    def __init__(self, *a, **kw):
        self.service_callback_group = MagicMock()
        self._logger = MagicMock()

    def get_logger(self):
        return self._logger

    def declare_and_validate_parameters(self, spec):
        return {k: v['default'] for k, v in spec.items()}

    def get_machine_ids(self, params):
        return params.get('machine_ids', 'cnc1,cnc2,cnc3').split(',')

    def create_subscription(self, *a, **kw):
        return MagicMock()

    def create_multi_machine_subscriptions(self, *a, **kw):
        return MagicMock()

    def create_publisher(self, *a, **kw):
        return MagicMock()

    def create_service(self, *a, **kw):
        return MagicMock()

    def create_timer(self, *a, **kw):
        return MagicMock()

    def get_parameter(self, name):
        m = MagicMock()
        m.value = 1.0
        return m

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.nanoseconds = int(time.time() * 1e9)
        clock.now.return_value.to_msg.return_value = MagicMock()
        return clock


_mc_lifecycle.MiracleLifecycleNode = _StubLifecycleNode

_mc_qos.QoSProfiles = MagicMock()
_mc_qos.QoSProfiles.alert.return_value = MagicMock()
_mc_qos.QoSProfiles.state_data.return_value = MagicMock()
_mc_qos.QoSProfiles.logging.return_value = MagicMock()

# miracle_msgs stubs
_msgs = _ensure_module('miracle_msgs')
_msgs_msg = _ensure_module('miracle_msgs.msg')


class _StubMsg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattr__(self, name):
        return ''


_msgs_msg.DigitalThreadEntry = _StubMsg
_msgs_msg.JobStatus = _StubMsg
_msgs_msg.AnomalyAlert = _StubMsg
_msgs_msg.MachineState = _StubMsg

# ---------------------------------------------------------------------------
# Now import the production code
# ---------------------------------------------------------------------------
from miracle_mes.digital_thread import (  # noqa: E402
    RunRuleViolation,
    SPCAnalysis,
    SPCRunRulesEngine,
)


# ===================================================================
# Tests: Rule 1 — Point beyond 3 sigma
# ===================================================================

class TestRule1BeyondThreeSigma:
    def test_detects_point_above_3_sigma(self):
        engine = SPCRunRulesEngine()
        # mean=10, std=1 -> UCL=13.  A point at 13.5 violates rule 1.
        data = [10.0, 10.2, 9.8, 10.1, 13.5]
        violations = engine.check_rule(1, data, mean=10.0, std=1.0)
        assert len(violations) == 1
        assert violations[0].rule_number == 1
        assert violations[0].start_index == 4
        assert violations[0].severity == 'action'
        assert violations[0].values == [13.5]

    def test_detects_point_below_3_sigma(self):
        engine = SPCRunRulesEngine()
        data = [10.0, 10.2, 6.0, 10.1, 9.9]
        violations = engine.check_rule(1, data, mean=10.0, std=1.0)
        assert len(violations) == 1
        assert violations[0].start_index == 2
        assert violations[0].values == [6.0]

    def test_no_violation_within_limits(self):
        engine = SPCRunRulesEngine()
        data = [10.0, 10.5, 9.5, 10.1, 9.9]
        violations = engine.check_rule(1, data, mean=10.0, std=1.0)
        assert len(violations) == 0


# ===================================================================
# Tests: Rule 2 — 9 consecutive points same side
# ===================================================================

class TestRule2NineConsecutiveSameSide:
    def test_detects_nine_above_mean(self):
        engine = SPCRunRulesEngine()
        # 9 points all above mean=10
        data = [10.1, 10.2, 10.3, 10.1, 10.4, 10.5, 10.2, 10.3, 10.1]
        violations = engine.check_rule(2, data, mean=10.0, std=1.0)
        assert len(violations) >= 1
        v = violations[0]
        assert v.rule_number == 2
        assert v.start_index == 0
        assert v.end_index == 8

    def test_no_violation_with_mixed_sides(self):
        engine = SPCRunRulesEngine()
        # Points alternate around the mean
        data = [10.1, 9.9, 10.1, 9.9, 10.1, 9.9, 10.1, 9.9, 10.1]
        violations = engine.check_rule(2, data, mean=10.0, std=1.0)
        assert len(violations) == 0


# ===================================================================
# Tests: Rule 3 — 6 consecutive increasing or decreasing
# ===================================================================

class TestRule3SixConsecutiveTrend:
    def test_detects_six_increasing(self):
        engine = SPCRunRulesEngine()
        data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        violations = engine.check_rule(3, data, mean=3.5, std=1.0)
        assert len(violations) >= 1
        assert violations[0].rule_number == 3
        assert violations[0].severity == 'warning'

    def test_detects_six_decreasing(self):
        engine = SPCRunRulesEngine()
        data = [6.0, 5.0, 4.0, 3.0, 2.0, 1.0]
        violations = engine.check_rule(3, data, mean=3.5, std=1.0)
        assert len(violations) >= 1

    def test_no_violation_short_trend(self):
        engine = SPCRunRulesEngine()
        data = [1.0, 2.0, 3.0, 4.0, 5.0, 4.0]
        violations = engine.check_rule(3, data, mean=3.0, std=1.0)
        assert len(violations) == 0


# ===================================================================
# Tests: Rule 4 — 14 consecutive alternating
# ===================================================================

class TestRule4FourteenAlternating:
    def test_detects_alternating_pattern(self):
        engine = SPCRunRulesEngine()
        data = [1.0, 3.0, 1.0, 3.0, 1.0, 3.0, 1.0, 3.0,
                1.0, 3.0, 1.0, 3.0, 1.0, 3.0]
        violations = engine.check_rule(4, data, mean=2.0, std=1.0)
        assert len(violations) >= 1
        assert violations[0].rule_number == 4
        assert violations[0].severity == 'warning'

    def test_no_violation_insufficient_length(self):
        engine = SPCRunRulesEngine()
        data = [1.0, 3.0, 1.0, 3.0, 1.0, 3.0]
        violations = engine.check_rule(4, data, mean=2.0, std=1.0)
        assert len(violations) == 0


# ===================================================================
# Tests: Rule 5 — 2 of 3 beyond 2 sigma (same side)
# ===================================================================

class TestRule5TwoOfThreeBeyondTwoSigma:
    def test_detects_two_above_2_sigma(self):
        engine = SPCRunRulesEngine()
        # mean=10, std=1, 2sigma = 12.  Points at 12.5, 12.3 violate.
        data = [10.0, 12.5, 12.3]
        violations = engine.check_rule(5, data, mean=10.0, std=1.0)
        assert len(violations) >= 1
        assert violations[0].rule_number == 5

    def test_no_violation_single_outlier(self):
        engine = SPCRunRulesEngine()
        data = [10.0, 12.5, 10.0]
        violations = engine.check_rule(5, data, mean=10.0, std=1.0)
        assert len(violations) == 0


# ===================================================================
# Tests: Rule 6 — 4 of 5 beyond 1 sigma (same side)
# ===================================================================

class TestRule6FourOfFiveBeyondOneSigma:
    def test_detects_four_above_1_sigma(self):
        engine = SPCRunRulesEngine()
        # mean=10, std=1, 1sigma=11.  Four out of five above 11.
        data = [11.5, 11.2, 10.5, 11.3, 11.1]
        violations = engine.check_rule(6, data, mean=10.0, std=1.0)
        assert len(violations) >= 1
        assert violations[0].rule_number == 6

    def test_no_violation_only_two_beyond(self):
        engine = SPCRunRulesEngine()
        data = [11.5, 10.0, 10.0, 11.3, 10.0]
        violations = engine.check_rule(6, data, mean=10.0, std=1.0)
        assert len(violations) == 0


# ===================================================================
# Tests: Rule 7 — 15 consecutive within 1 sigma (stratification)
# ===================================================================

class TestRule7FifteenWithinOneSigma:
    def test_detects_stratification(self):
        engine = SPCRunRulesEngine()
        # 15 points all within 1 sigma of mean=10, std=2
        data = [10.0 + 0.1 * i for i in range(15)]  # 10.0 .. 11.4
        # With std=2, 1sigma band is [8, 12].  All points inside.
        violations = engine.check_rule(7, data, mean=10.0, std=2.0)
        assert len(violations) >= 1
        assert violations[0].rule_number == 7
        assert violations[0].severity == 'warning'

    def test_no_violation_when_point_outside_1_sigma(self):
        engine = SPCRunRulesEngine()
        data = [10.0] * 14 + [15.0]  # last point beyond 1 sigma
        violations = engine.check_rule(7, data, mean=10.0, std=2.0)
        assert len(violations) == 0


# ===================================================================
# Tests: Rule 8 — 8 consecutive beyond 1 sigma both sides (mixture)
# ===================================================================

class TestRule8EightBeyondOneSigmaMixture:
    def test_detects_mixture(self):
        engine = SPCRunRulesEngine()
        # mean=10, std=1.  Points all beyond 1 sigma, on both sides.
        data = [11.5, 8.5, 11.5, 8.5, 11.5, 8.5, 11.5, 8.5]
        violations = engine.check_rule(8, data, mean=10.0, std=1.0)
        assert len(violations) >= 1
        assert violations[0].rule_number == 8
        assert violations[0].severity == 'action'

    def test_no_violation_when_all_same_side(self):
        engine = SPCRunRulesEngine()
        # All above 1 sigma but only one side -> not mixture
        data = [11.5, 11.6, 11.7, 11.8, 11.5, 11.6, 11.7, 11.8]
        violations = engine.check_rule(8, data, mean=10.0, std=1.0)
        assert len(violations) == 0


# ===================================================================
# Tests: Full analysis (analyze method)
# ===================================================================

class TestAnalyze:
    def test_in_control_data(self):
        engine = SPCRunRulesEngine()
        # Well-behaved data with low variation, no patterns
        data = [10.1, 9.9, 10.0, 10.05, 9.95]
        result = engine.analyze(data, target_mean=10.0, target_std=1.0)
        assert isinstance(result, SPCAnalysis)
        assert result.mean == 10.0
        assert result.std == 1.0
        assert result.ucl == 13.0
        assert result.lcl == 7.0
        assert result.in_control is True
        assert result.total_points == 5
        assert len(result.violations) == 0

    def test_out_of_control_data(self):
        engine = SPCRunRulesEngine()
        # One point far beyond 3 sigma
        data = [10.0, 10.1, 9.9, 10.0, 20.0]
        result = engine.analyze(data, target_mean=10.0, target_std=1.0)
        assert result.in_control is False
        assert len(result.violations) > 0
        rule_numbers = {v.rule_number for v in result.violations}
        assert 1 in rule_numbers

    def test_empty_data(self):
        engine = SPCRunRulesEngine()
        result = engine.analyze([], target_mean=10.0, target_std=1.0)
        assert result.total_points == 0
        assert result.in_control is True

    def test_analyze_computes_mean_and_std(self):
        engine = SPCRunRulesEngine()
        data = [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0]
        result = engine.analyze(data)
        expected_mean = sum(data) / len(data)
        assert abs(result.mean - expected_mean) < 1e-9
        assert result.total_points == 8


# ===================================================================
# Tests: Rule description lookup
# ===================================================================

class TestRuleDescription:
    def test_all_rule_descriptions(self):
        engine = SPCRunRulesEngine()
        for rule_num in range(1, 9):
            desc = engine.get_rule_description(rule_num)
            assert isinstance(desc, str)
            assert len(desc) > 0

    def test_invalid_rule_number(self):
        engine = SPCRunRulesEngine()
        with pytest.raises(ValueError):
            engine.get_rule_description(0)
        with pytest.raises(ValueError):
            engine.get_rule_description(9)

    def test_check_rule_invalid_number(self):
        engine = SPCRunRulesEngine()
        with pytest.raises(ValueError):
            engine.check_rule(99, [1.0, 2.0], 1.5, 0.5)
