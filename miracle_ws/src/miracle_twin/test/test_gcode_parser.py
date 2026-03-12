"""Tests for the enhanced G-code parser in PredictionRunnerNode."""
import math
import sys
import types
import pytest
from unittest.mock import MagicMock

from miracle_twin.cutting_sim_proxy import GCodeBlock


# Build mock modules with a real base class so PredictionRunnerNode
# can be properly instantiated without ROS2.
class _FakeLifecycleNode:
    """Minimal stand-in for MiracleLifecycleNode."""
    CRITICALITY_MEDIUM = 2

    def __init__(self, *args, **kwargs):
        pass


def _build_mock_modules():
    mocks = {}
    for name in (
        'rclpy', 'rclpy.lifecycle', 'rclpy.action',
        'rclpy.action.server', 'rclpy.callback_groups',
        'miracle_core', 'miracle_core.qos_profiles',
        'miracle_msgs', 'miracle_msgs.msg',
        'miracle_msgs.action', 'miracle_msgs.srv',
    ):
        mocks[name] = MagicMock()

    # miracle_core.lifecycle_node_base must expose a real class
    lifecycle_mod = types.ModuleType('miracle_core.lifecycle_node_base')
    lifecycle_mod.MiracleLifecycleNode = _FakeLifecycleNode
    mocks['miracle_core.lifecycle_node_base'] = lifecycle_mod

    return mocks


_mock_modules = _build_mock_modules()


@pytest.fixture(autouse=True)
def _patch_ros():
    """Patch ROS2 modules so we can unit-test the parser in isolation."""
    saved = {}
    for mod_name, mock_obj in _mock_modules.items():
        saved[mod_name] = sys.modules.get(mod_name)
        sys.modules[mod_name] = mock_obj

    # Force reimport of prediction_runner so it picks up mocks
    sys.modules.pop('miracle_twin.prediction_runner', None)

    yield

    sys.modules.pop('miracle_twin.prediction_runner', None)
    for mod_name, orig in saved.items():
        if orig is None:
            sys.modules.pop(mod_name, None)
        else:
            sys.modules[mod_name] = orig


def _make_parser():
    """Create a PredictionRunnerNode instance with mocked ROS dependencies."""
    from miracle_twin.prediction_runner import PredictionRunnerNode
    return PredictionRunnerNode()


class TestCommentStripping:
    """Verify that comments are properly stripped from G-code lines."""

    def test_inline_comment_removed(self):
        parser = _make_parser()
        blocks = parser._parse_gcode_to_blocks(
            "G1 X10 Y20 (move to corner) F200\nS1000"
        )
        assert len(blocks) == 1
        assert blocks[0].feed_rate_mmpm == 200.0

    def test_semicolon_comment_removed(self):
        parser = _make_parser()
        blocks = parser._parse_gcode_to_blocks(
            "S1000\nG1 X10 F200 ; rapid to position"
        )
        assert len(blocks) == 1
        b = blocks[0]
        # X10 should be parsed; the '; rapid...' part is stripped
        assert b.feed_rate_mmpm == 200.0

    def test_full_line_comment_skipped(self):
        parser = _make_parser()
        blocks = parser._parse_gcode_to_blocks(
            "(this is a full line comment)\nG1 X5 F100\nS1000"
        )
        assert len(blocks) == 1


class TestArcCommands:
    """Verify G2/G3 arc parsing."""

    def test_g2_arc_ij_format(self):
        parser = _make_parser()
        gcode = "S1000\nG1 X0 Y0 F200\nG2 X20 Y0 I10 J0 F200"
        blocks = parser._parse_gcode_to_blocks(gcode)
        # Should have two blocks: the G1 and the G2
        arc_blocks = [b for b in blocks if b.is_arc]
        assert len(arc_blocks) == 1
        arc = arc_blocks[0]
        assert arc.arc_direction == 'CW'
        assert arc.arc_center_i == 10.0
        assert arc.arc_center_j == 0.0
        assert arc.length_mm > 0  # arc length should be positive

    def test_g3_arc_ccw(self):
        parser = _make_parser()
        gcode = "S1000\nG1 X0 Y0 F200\nG3 X20 Y0 I10 J0 F200"
        blocks = parser._parse_gcode_to_blocks(gcode)
        arc_blocks = [b for b in blocks if b.is_arc]
        assert len(arc_blocks) == 1
        assert arc_blocks[0].arc_direction == 'CCW'

    def test_arc_preserves_start_end(self):
        parser = _make_parser()
        gcode = "S1000\nG1 X5 Y3 F200\nG2 X15 Y3 I5 J0 F200"
        blocks = parser._parse_gcode_to_blocks(gcode)
        arc = [b for b in blocks if b.is_arc][0]
        assert arc.start_x == 5.0
        assert arc.start_y == 3.0
        assert arc.end_x == 15.0
        assert arc.end_y == 3.0


class TestCannedCycles:
    """Verify G81/G83/G73 canned cycle parsing."""

    def test_g81_drill_cycle(self):
        parser = _make_parser()
        gcode = "S1000 F50\nG81 X10 Y10 Z-5 R2"
        blocks = parser._parse_gcode_to_blocks(gcode)
        assert len(blocks) == 1
        b = blocks[0]
        # Total depth = |R - Z| = |2 - (-5)| = 7
        assert b.axial_depth_mm == pytest.approx(7.0)
        assert b.length_mm == pytest.approx(7.0)

    def test_g83_peck_drill(self):
        parser = _make_parser()
        gcode = "S1000 F50\nG83 X10 Y10 Z-15 Q3 R2"
        blocks = parser._parse_gcode_to_blocks(gcode)
        # Total depth = |2 - (-15)| = 17, peck = 3 -> ceil(17/3) = 6 blocks
        # 5 full pecks of 3 + 1 peck of 2
        assert len(blocks) == 6
        # First 5 blocks should be 3mm deep
        for b in blocks[:5]:
            assert b.axial_depth_mm == pytest.approx(3.0)
        # Last block is the remainder
        assert blocks[5].axial_depth_mm == pytest.approx(2.0)

    def test_g73_chip_break(self):
        parser = _make_parser()
        gcode = "S1000 F50\nG73 X10 Y10 Z-9 Q3 R0"
        blocks = parser._parse_gcode_to_blocks(gcode)
        # Total depth = 9, peck = 3 -> 3 blocks
        assert len(blocks) == 3
        for b in blocks:
            assert b.axial_depth_mm == pytest.approx(3.0)


class TestModalTracking:
    """Verify that modal G-code state is tracked correctly."""

    def test_g90_absolute_mode(self):
        parser = _make_parser()
        gcode = "S1000 F200\nG90\nG1 X10 Y0\nG1 X20 Y0"
        blocks = parser._parse_gcode_to_blocks(gcode)
        assert len(blocks) == 2
        # In absolute mode, second move is from (10,0) to (20,0) = 10mm
        assert blocks[1].length_mm == pytest.approx(10.0)

    def test_g91_incremental_mode(self):
        parser = _make_parser()
        gcode = "S1000 F200\nG91\nG1 X10 Y0\nG1 X10 Y0"
        blocks = parser._parse_gcode_to_blocks(gcode)
        assert len(blocks) == 2
        # In incremental mode, each move is 10mm
        assert blocks[0].length_mm == pytest.approx(10.0)
        assert blocks[1].length_mm == pytest.approx(10.0)

    def test_feed_persists_modally(self):
        parser = _make_parser()
        gcode = "S1000\nG1 X10 F300\nG1 X20"
        blocks = parser._parse_gcode_to_blocks(gcode)
        assert len(blocks) == 2
        assert blocks[0].feed_rate_mmpm == 300.0
        assert blocks[1].feed_rate_mmpm == 300.0  # modal persistence


class TestLineNumbers:
    """Verify N-word line number stripping."""

    def test_n_prefix_stripped(self):
        parser = _make_parser()
        gcode = "S1000 F200\nN100 G1 X10"
        blocks = parser._parse_gcode_to_blocks(gcode)
        assert len(blocks) == 1

    def test_n_prefix_with_spaces(self):
        parser = _make_parser()
        gcode = "S1000 F200\nN 200 G1 X15 Y5"
        blocks = parser._parse_gcode_to_blocks(gcode)
        assert len(blocks) == 1
        assert blocks[0].length_mm > 0


class TestBlankLinesAndDelimiters:
    """Verify that blank lines and % delimiters are handled gracefully."""

    def test_blank_lines_skipped(self):
        parser = _make_parser()
        gcode = "S1000 F200\n\n\nG1 X10\n\n"
        blocks = parser._parse_gcode_to_blocks(gcode)
        assert len(blocks) == 1

    def test_percent_delimiters_skipped(self):
        parser = _make_parser()
        gcode = "%\nS1000 F200\nG1 X10\n%"
        blocks = parser._parse_gcode_to_blocks(gcode)
        assert len(blocks) == 1


class TestToolChange:
    """Verify M6 tool change handling."""

    def test_m6_updates_tool(self):
        parser = _make_parser()
        # After M6 T2, the parser state should track tool 2.
        # We verify this indirectly: the program still parses correctly.
        gcode = "S1000 F200\nG1 X10\nM6 T2\nG1 X20"
        blocks = parser._parse_gcode_to_blocks(gcode)
        assert len(blocks) == 2


class TestSubprogramWarning:
    """Verify M98 subprogram call logs a warning."""

    def test_m98_does_not_crash(self):
        parser = _make_parser()
        gcode = "S1000 F200\nM98 P1234\nG1 X10"
        blocks = parser._parse_gcode_to_blocks(gcode)
        # M98 should be skipped gracefully; G1 still parses
        assert len(blocks) == 1


class TestDepthOverride:
    """Verify depth-of-cut override scaling."""

    def test_z_scaled_by_depth_override(self):
        parser = _make_parser()
        # Without override
        gcode = "S1000 F200\nG90\nG1 X0 Y0 Z0\nG1 X10 Z-10"
        blocks_normal = parser._parse_gcode_to_blocks(gcode)

        # With 0.5x depth override
        blocks_half = parser._parse_gcode_to_blocks(gcode, depth_override=0.5)

        # The Z movement should be halved
        assert len(blocks_normal) >= 1
        assert len(blocks_half) >= 1
        # Find the block with Z movement
        z_block_normal = [b for b in blocks_normal if b.axial_depth_mm > 1.0]
        z_block_half = [b for b in blocks_half if b.axial_depth_mm > 0.1]
        assert len(z_block_normal) > 0
        assert len(z_block_half) > 0
        # Half depth should produce smaller axial depth
        assert z_block_half[0].axial_depth_mm < z_block_normal[0].axial_depth_mm

    def test_drill_cycle_depth_scaled(self):
        parser = _make_parser()
        gcode = "S1000 F50\nG81 X10 Y10 Z-10 R2"
        blocks_normal = parser._parse_gcode_to_blocks(gcode)
        blocks_half = parser._parse_gcode_to_blocks(gcode, depth_override=0.5)
        # Normal: |2 - (-10)| = 12
        assert blocks_normal[0].axial_depth_mm == pytest.approx(12.0)
        # Half: |2 - (-5)| = 7
        assert blocks_half[0].axial_depth_mm == pytest.approx(7.0)


class TestArcSimulation:
    """Verify CuttingSimProxy.simulate_arc_block integration."""

    def test_arc_block_simulated(self):
        from miracle_twin.cutting_sim_proxy import CuttingSimProxy, ToolState
        proxy = CuttingSimProxy()
        tool = ToolState()

        arc_block = GCodeBlock(
            feed_rate_mmpm=200.0,
            spindle_rpm=8000.0,
            axial_depth_mm=1.5,
            radial_depth_mm=3.175,
            length_mm=31.4,  # ~half circle r=10
            arc_center_i=10.0,
            arc_center_j=0.0,
            arc_direction='CW',
            start_x=0.0,
            start_y=0.0,
            end_x=20.0,
            end_y=0.0,
        )

        pred, vb, ct = proxy.simulate_arc_block(
            arc_block, tool, vb=0.02, cutting_time=0.0
        )
        assert pred.peak_force_n > 0
        assert pred.avg_force_n > 0
        assert pred.power_w > 0
        assert vb >= 0.02  # wear should not decrease

    def test_arc_block_in_simulate_program(self):
        from miracle_twin.cutting_sim_proxy import CuttingSimProxy, ToolState
        proxy = CuttingSimProxy()

        blocks = [
            GCodeBlock(
                feed_rate_mmpm=200.0,
                spindle_rpm=8000.0,
                length_mm=10.0,
            ),
            GCodeBlock(
                feed_rate_mmpm=200.0,
                spindle_rpm=8000.0,
                length_mm=31.4,
                arc_center_i=10.0,
                arc_center_j=0.0,
                arc_direction='CCW',
                start_x=0.0,
                start_y=0.0,
                end_x=20.0,
                end_y=0.0,
            ),
        ]
        result = proxy.simulate_program(blocks)
        assert len(result.block_predictions) == 2
        assert result.block_predictions[1].peak_force_n > 0


class TestGenerateExplanation:
    """Verify the generate_explanation helper."""

    def test_explanation_counts(self):
        parser = _make_parser()
        gcode = "S1000 F200\nG1 X10\nG2 X20 Y0 I5 J0"
        blocks = parser._parse_gcode_to_blocks(gcode)
        explanation = parser.generate_explanation(blocks, depth_override=0.5)
        assert 'linear' in explanation
        assert 'arc' in explanation
        assert '0.50x' in explanation
