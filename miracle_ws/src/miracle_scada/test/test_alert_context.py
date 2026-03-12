"""Tests for G-code context enrichment in the Alert Correlation Engine."""

import json
import pytest
import time
import sys
import os
from unittest.mock import MagicMock

# Stub out ROS2 and project dependencies so we can import the module
# without a ROS2 installation.
for mod in ['rclpy', 'rclpy.node', 'rclpy.callback_groups',
            'rclpy.qos', 'rclpy.lifecycle',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_core.heartbeat_mixin',
            'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = type('FakeNode', (), {
    'CRITICALITY_HIGH': 'HIGH',
    '__init__': lambda self, *a, **kw: None,
    'get_logger': lambda self: MagicMock(),
    'create_publisher': lambda self, *a, **kw: MagicMock(),
    'create_subscription': lambda self, *a, **kw: MagicMock(),
    'create_timer': lambda self, *a, **kw: MagicMock(),
    'declare_and_validate_parameters': lambda self, specs: {k: MagicMock(value=v['default']) for k, v in specs.items()},
    'get_parameter': lambda self, name: MagicMock(value=0),
})

# Clear cached module to force re-import with our stubs
sys.modules.pop('miracle_scada.alert_correlator', None)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from miracle_scada.alert_correlator import (
    GCodeContext,
    AlertEntry,
    CorrelationRule,
    AlertCorrelatorNode,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_context(**overrides) -> GCodeContext:
    """Create a GCodeContext with sensible defaults, allowing overrides."""
    defaults = dict(
        program_name='part_001.nc',
        block_index=42,
        gcode_line='G01 X10 Y20 F500',
        operation_type='LINEAR_CUT',
        feed_rate=500.0,
        spindle_rpm=12000.0,
        depth_of_cut=2.5,
        tool_id='T01',
        elapsed_program_pct=35.0,
    )
    defaults.update(overrides)
    return GCodeContext(**defaults)


def _make_node() -> AlertCorrelatorNode:
    """Create an AlertCorrelatorNode without ROS2 lifecycle."""
    node = AlertCorrelatorNode.__new__(AlertCorrelatorNode)
    # Manually initialize the attributes that __init__ sets
    node._alert_buffer = {}
    node._buffer_lock = __import__('threading').Lock()
    node._rules = []
    node._fleet_rules = []
    node._correlation_window = 30.0
    node._fleet_window = 60.0
    node._correlated_pub = MagicMock()
    node._check_timer = None
    node._correlation_counter = 0
    node._suppressed_ids = set()
    node._fleet_buffer = {}
    node._fleet_suppressed_ids = set()
    node._fleet_alert_count = 0
    node._causal_links = {}
    node._machine_gcode_context = {}
    node._block_anomaly_history = {}
    node._block_anomaly_history_cap = 1000
    # Provide a logger and clock mock
    node.get_logger = lambda: MagicMock()
    node.get_clock = lambda: MagicMock(now=lambda: MagicMock(to_msg=lambda: MagicMock()))
    return node


# ===================================================================
# Test GCodeContext creation and serialization
# ===================================================================


class TestGCodeContextCreation:
    """Test GCodeContext dataclass construction."""

    def test_create_basic(self):
        ctx = _make_context()
        assert ctx.program_name == 'part_001.nc'
        assert ctx.block_index == 42
        assert ctx.operation_type == 'LINEAR_CUT'
        assert ctx.feed_rate == 500.0
        assert ctx.spindle_rpm == 12000.0

    def test_create_with_all_operation_types(self):
        for op in ['LINEAR_CUT', 'ARC_CUT', 'DRILL', 'RAPID', 'TOOL_CHANGE']:
            ctx = _make_context(operation_type=op)
            assert ctx.operation_type == op

    def test_elapsed_pct_boundaries(self):
        ctx_start = _make_context(elapsed_program_pct=0.0)
        ctx_end = _make_context(elapsed_program_pct=100.0)
        assert ctx_start.elapsed_program_pct == 0.0
        assert ctx_end.elapsed_program_pct == 100.0


class TestGCodeContextSerialization:
    """Test GCodeContext JSON round-trip."""

    def test_to_json(self):
        ctx = _make_context()
        j = ctx.to_json()
        parsed = json.loads(j)
        assert parsed['program_name'] == 'part_001.nc'
        assert parsed['block_index'] == 42
        assert parsed['feed_rate'] == 500.0

    def test_from_json_roundtrip(self):
        ctx = _make_context(block_index=99, spindle_rpm=8000.0)
        j = ctx.to_json()
        restored = GCodeContext.from_json(j)
        assert restored.block_index == 99
        assert restored.spindle_rpm == 8000.0
        assert restored.program_name == ctx.program_name

    def test_from_json_preserves_all_fields(self):
        ctx = _make_context()
        restored = GCodeContext.from_json(ctx.to_json())
        assert restored == ctx


# ===================================================================
# Test context attachment to correlated alerts
# ===================================================================


class TestContextAttachment:
    """Test that G-code context is attached to correlated alerts."""

    def test_enrich_alert_with_context(self):
        node = _make_node()
        ctx = _make_context(block_index=10)
        node.update_gcode_context('cnc1', ctx)

        confidence, gcode_json, blocks_until = node._enrich_alert_with_gcode(
            None, 'cnc1', 0.8,
        )
        assert gcode_json is not None
        parsed = json.loads(gcode_json)
        assert parsed['block_index'] == 10

    def test_enrich_alert_without_context(self):
        node = _make_node()
        confidence, gcode_json, blocks_until = node._enrich_alert_with_gcode(
            None, 'cnc1', 0.8,
        )
        assert gcode_json is None
        assert blocks_until is None
        assert confidence == 0.8

    def test_context_attached_during_correlation(self):
        """When a correlation fires, the gcode context JSON should be set on msg."""
        node = _make_node()
        ctx = _make_context(block_index=50)
        node.update_gcode_context('cnc1', ctx)

        now = time.time()
        alerts = [
            AlertEntry('vibration_anomaly', 'cnc1', 0.9, 'vib', now - 2),
            AlertEntry('force_anomaly', 'cnc1', 0.85, 'force', now - 1),
        ]
        node._alert_buffer['cnc1'] = alerts
        rule = CorrelationRule(
            name='tool_degradation_chain',
            required_types=['vibration_anomaly', 'force_anomaly'],
            time_window_sec=30.0,
            root_cause='Tool wear',
            recommended_actions=['Replace tool'],
        )
        node._rules = [rule]

        node._try_correlate('cnc1', alerts, rule, now)

        # The publisher should have been called
        assert node._correlated_pub.publish.called
        msg = node._correlated_pub.publish.call_args[0][0]
        assert hasattr(msg, 'gcode_context_json')


# ===================================================================
# Test recurring block detection
# ===================================================================


class TestRecurringBlockDetection:
    """Test detection of recurring anomalies on the same G-code block."""

    def test_single_anomaly_no_recurring(self):
        node = _make_node()
        node._record_block_anomaly('cnc1', 10)
        conf, is_recurring = node._correlate_with_gcode_patterns('cnc1', 10, 0.7)
        assert not is_recurring
        assert conf == 0.7

    def test_two_anomalies_no_recurring(self):
        node = _make_node()
        node._record_block_anomaly('cnc1', 10)
        node._record_block_anomaly('cnc1', 10)
        conf, is_recurring = node._correlate_with_gcode_patterns('cnc1', 10, 0.7)
        assert not is_recurring
        assert conf == 0.7

    def test_three_anomalies_triggers_recurring(self):
        node = _make_node()
        for _ in range(3):
            node._record_block_anomaly('cnc1', 10)
        conf, is_recurring = node._correlate_with_gcode_patterns('cnc1', 10, 0.7)
        assert is_recurring
        assert conf == pytest.approx(0.85)

    def test_recurring_different_block_unaffected(self):
        node = _make_node()
        for _ in range(5):
            node._record_block_anomaly('cnc1', 10)
        conf, is_recurring = node._correlate_with_gcode_patterns('cnc1', 20, 0.7)
        assert not is_recurring
        assert conf == 0.7


# ===================================================================
# Test confidence boost for recurring issues
# ===================================================================


class TestConfidenceBoost:
    """Test that recurring block issues properly boost confidence."""

    def test_boost_capped_at_one(self):
        node = _make_node()
        for _ in range(10):
            node._record_block_anomaly('cnc1', 5)
        conf, is_recurring = node._correlate_with_gcode_patterns('cnc1', 5, 0.95)
        assert is_recurring
        assert conf == 1.0

    def test_boost_exact_value(self):
        node = _make_node()
        for _ in range(4):
            node._record_block_anomaly('cnc1', 7)
        conf, _ = node._correlate_with_gcode_patterns('cnc1', 7, 0.60)
        assert conf == pytest.approx(0.75)


# ===================================================================
# Test context tracking across multiple machines
# ===================================================================


class TestMultiMachineContext:
    """Test that G-code context is tracked independently per machine."""

    def test_separate_contexts(self):
        node = _make_node()
        ctx1 = _make_context(block_index=10, program_name='job_a.nc')
        ctx2 = _make_context(block_index=55, program_name='job_b.nc')
        node.update_gcode_context('cnc1', ctx1)
        node.update_gcode_context('cnc2', ctx2)

        assert node.get_gcode_context('cnc1').block_index == 10
        assert node.get_gcode_context('cnc2').block_index == 55
        assert node.get_gcode_context('cnc1').program_name == 'job_a.nc'

    def test_update_replaces_context(self):
        node = _make_node()
        ctx1 = _make_context(block_index=10)
        ctx2 = _make_context(block_index=20)
        node.update_gcode_context('cnc1', ctx1)
        node.update_gcode_context('cnc1', ctx2)
        assert node.get_gcode_context('cnc1').block_index == 20

    def test_block_history_per_machine(self):
        node = _make_node()
        for _ in range(4):
            node._record_block_anomaly('cnc1', 10)
        node._record_block_anomaly('cnc2', 10)

        conf1, recurring1 = node._correlate_with_gcode_patterns('cnc1', 10, 0.5)
        conf2, recurring2 = node._correlate_with_gcode_patterns('cnc2', 10, 0.5)
        assert recurring1 is True
        assert recurring2 is False


# ===================================================================
# Test missing context doesn't break correlation
# ===================================================================


class TestMissingContext:
    """Test that missing G-code context doesn't break anything."""

    def test_no_context_returns_none(self):
        node = _make_node()
        assert node.get_gcode_context('cnc999') is None

    def test_enrich_without_context_passthrough(self):
        node = _make_node()
        conf, gcode_json, blocks_until = node._enrich_alert_with_gcode(None, 'cnc1', 0.75)
        assert conf == 0.75
        assert gcode_json is None
        assert blocks_until is None

    def test_correlation_works_without_context(self):
        """Full correlation should succeed when no G-code context is set."""
        node = _make_node()
        now = time.time()
        alerts = [
            AlertEntry('vibration_anomaly', 'cnc1', 0.9, 'vib', now - 2),
            AlertEntry('force_anomaly', 'cnc1', 0.85, 'force', now - 1),
        ]
        rule = CorrelationRule(
            name='tool_degradation_chain',
            required_types=['vibration_anomaly', 'force_anomaly'],
            time_window_sec=30.0,
            root_cause='Tool wear',
            recommended_actions=['Replace tool'],
        )
        node._try_correlate('cnc1', alerts, rule, now)
        assert node._correlated_pub.publish.called


# ===================================================================
# Test block anomaly history update and cap
# ===================================================================


class TestBlockAnomalyHistory:
    """Test block anomaly history management."""

    def test_history_increments(self):
        node = _make_node()
        node._record_block_anomaly('cnc1', 10)
        node._record_block_anomaly('cnc1', 10)
        assert node._block_anomaly_history['cnc1'][10] == 2

    def test_history_cap_enforced(self):
        node = _make_node()
        node._block_anomaly_history_cap = 5
        for block in range(10):
            node._record_block_anomaly('cnc1', block)
        # After 10 blocks added with cap of 5, only 5 should remain
        assert len(node._block_anomaly_history['cnc1']) <= 5

    def test_cap_removes_lowest_count(self):
        node = _make_node()
        node._block_anomaly_history_cap = 3
        # Block 0 has count 1, block 1 has count 2, block 2 has count 3
        node._record_block_anomaly('cnc1', 0)
        node._record_block_anomaly('cnc1', 1)
        node._record_block_anomaly('cnc1', 1)
        node._record_block_anomaly('cnc1', 2)
        node._record_block_anomaly('cnc1', 2)
        node._record_block_anomaly('cnc1', 2)
        # Now add block 3 — should evict block 0 (lowest count=1)
        node._record_block_anomaly('cnc1', 3)
        hist = node._block_anomaly_history['cnc1']
        assert 0 not in hist
        assert len(hist) <= 3

    def test_blocks_until_predicted_issue(self):
        """Test lookahead finds future blocks with known issues."""
        node = _make_node()
        # Create history: block 50 has recurring issues
        for _ in range(4):
            node._record_block_anomaly('cnc1', 50)

        ctx = _make_context(block_index=45)
        node.update_gcode_context('cnc1', ctx)

        conf, gcode_json, blocks_until = node._enrich_alert_with_gcode(None, 'cnc1', 0.7)
        assert blocks_until == 5  # 50 - 45

    def test_no_lookahead_when_no_future_issues(self):
        """When no future blocks have issues, blocks_until should be None."""
        node = _make_node()
        ctx = _make_context(block_index=45)
        node.update_gcode_context('cnc1', ctx)

        conf, gcode_json, blocks_until = node._enrich_alert_with_gcode(None, 'cnc1', 0.7)
        assert blocks_until is None
