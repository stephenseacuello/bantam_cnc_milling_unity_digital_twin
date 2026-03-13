"""Tests for G-code macro/subroutine expansion.

Covers GCodeMacro, MacroLibrary registration, M98/G65 call expansion,
O-word definition parsing, built-in macros, parameter substitution,
nested expansion, and depth protection.
"""

import sys
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Mock all ROS2 / project dependencies so the module can be imported without
# a live ROS2 environment.
# ---------------------------------------------------------------------------
sys.modules.setdefault('rclpy', MagicMock())
sys.modules['rclpy.lifecycle'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.qos'] = MagicMock()
sys.modules['rclpy.duration'] = MagicMock()
sys.modules['rclpy.action'] = MagicMock()
sys.modules['rclpy.action.server'] = MagicMock()
sys.modules['rclpy.callback_groups'] = MagicMock()
sys.modules['rcl_interfaces'] = MagicMock()
sys.modules['rcl_interfaces.msg'] = MagicMock()
sys.modules['builtin_interfaces'] = MagicMock()
sys.modules['builtin_interfaces.msg'] = MagicMock()

# miracle_core submodules (NOT top-level miracle_core)
for _mc_sub in ('miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
                'miracle_core.heartbeat_mixin', 'miracle_core.exceptions'):
    sys.modules.setdefault(_mc_sub, MagicMock())

# miracle_msgs submodules
for _mm_sub in ('miracle_msgs', 'miracle_msgs.msg', 'miracle_msgs.action', 'miracle_msgs.srv'):
    sys.modules.setdefault(_mm_sub, MagicMock())

# miracle_security submodules (NOT top-level miracle_security)
sys.modules.setdefault('miracle_security.gcode_signer', MagicMock())

from miracle_cnc.gcode_executor import (  # noqa: E402
    GCodeMacro,
    MacroExpansionError,
    MacroLibrary,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def lib():
    """Return a fresh MacroLibrary (includes built-in macros)."""
    return MacroLibrary()


@pytest.fixture
def simple_macro():
    """A trivial macro with no parameters."""
    return GCodeMacro(
        macro_id='O1000',
        name='SIMPLE',
        parameters=[],
        body=['G0 X0 Y0', 'G0 Z5'],
        description='Move to origin',
    )


@pytest.fixture
def param_macro():
    """A macro with parameters #1 and #2."""
    return GCodeMacro(
        macro_id='O2000',
        name='PARAM_TEST',
        parameters=['#1', '#2'],
        body=['G1 X#1 Y#2 F500', 'G0 Z5'],
        description='Move to given X/Y',
    )


# ---------------------------------------------------------------------------
# Registration & retrieval
# ---------------------------------------------------------------------------

class TestRegisterAndRetrieve:
    def test_register_and_get(self, lib, simple_macro):
        lib.register_macro(simple_macro)
        result = lib.get_macro('O1000')
        assert result is not None
        assert result.name == 'SIMPLE'

    def test_get_unknown_returns_none(self, lib):
        assert lib.get_macro('O9999') is None

    def test_register_overwrites(self, lib, simple_macro):
        lib.register_macro(simple_macro)
        updated = GCodeMacro(macro_id='O1000', name='UPDATED', body=['G0 Z10'])
        lib.register_macro(updated)
        assert lib.get_macro('O1000').name == 'UPDATED'

    def test_macro_fields(self, simple_macro):
        assert simple_macro.macro_id == 'O1000'
        assert simple_macro.name == 'SIMPLE'
        assert simple_macro.parameters == []
        assert len(simple_macro.body) == 2
        assert simple_macro.description == 'Move to origin'


# ---------------------------------------------------------------------------
# Simple expansion (no parameters)
# ---------------------------------------------------------------------------

class TestSimpleExpansion:
    def test_m98_expands_simple(self, lib, simple_macro):
        lib.register_macro(simple_macro)
        result = lib.expand_macro_call('M98 P1000')
        assert result == ['G0 X0 Y0', 'G0 Z5']

    def test_non_macro_line_unchanged(self, lib):
        result = lib.expand_macro_call('G0 X10 Y20')
        assert result == ['G0 X10 Y20']

    def test_comment_line_unchanged(self, lib):
        result = lib.expand_macro_call('; this is a comment')
        assert result == ['; this is a comment']

    def test_empty_body_macro(self, lib):
        lib.register_macro(GCodeMacro(macro_id='O5000', name='EMPTY', body=[]))
        result = lib.expand_macro_call('M98 P5000')
        assert result == []


# ---------------------------------------------------------------------------
# M98 repeat syntax (L parameter)
# ---------------------------------------------------------------------------

class TestM98Repeat:
    def test_m98_repeat_3(self, lib, simple_macro):
        lib.register_macro(simple_macro)
        result = lib.expand_macro_call('M98 P1000 L3')
        assert len(result) == 6  # 2 lines * 3 repetitions
        assert result == ['G0 X0 Y0', 'G0 Z5'] * 3

    def test_m98_repeat_1_default(self, lib, simple_macro):
        lib.register_macro(simple_macro)
        result = lib.expand_macro_call('M98 P1000')
        assert len(result) == 2

    def test_m98_repeat_0_is_empty(self, lib, simple_macro):
        """L0 should produce no lines (0 repetitions)."""
        lib.register_macro(simple_macro)
        # L0 means no loop — but our regex matches L(\d+) -> int('0') = 0
        result = lib.expand_macro_call('M98 P1000 L0')
        assert result == []

    def test_m98_unknown_macro_passthrough(self, lib):
        result = lib.expand_macro_call('M98 P7777')
        assert result == ['M98 P7777']


# ---------------------------------------------------------------------------
# G65 call with parameter substitution
# ---------------------------------------------------------------------------

class TestG65Expansion:
    def test_g65_param_a_b(self, lib, param_macro):
        lib.register_macro(param_macro)
        result = lib.expand_macro_call('G65 P2000 A10.0 B20.0')
        assert result[0] == 'G1 X10.0 Y20.0 F500'
        assert result[1] == 'G0 Z5'

    def test_g65_partial_params(self, lib, param_macro):
        lib.register_macro(param_macro)
        # Only A provided; #2 stays as literal
        result = lib.expand_macro_call('G65 P2000 A5.5')
        assert '5.5' in result[0]

    def test_g65_unknown_macro_passthrough(self, lib):
        result = lib.expand_macro_call('G65 P8888 A1.0')
        assert result == ['G65 P8888 A1.0']

    def test_g65_decimal_values(self, lib):
        macro = GCodeMacro(
            macro_id='O3000', name='DEC', parameters=['#1', '#2', '#3'],
            body=['G1 X#1 Y#2 Z#3'],
        )
        lib.register_macro(macro)
        result = lib.expand_macro_call('G65 P3000 A1.234 B-5.67 C0.01')
        assert 'X1.234' in result[0]
        assert 'Y-5.67' in result[0]

    def test_g65_multiple_params(self, lib):
        macro = GCodeMacro(
            macro_id='O3100', name='MULTI',
            parameters=['#1', '#2', '#3', '#4', '#5'],
            body=['G1 X#1 Y#2 Z#3 I#4 J#5'],
        )
        lib.register_macro(macro)
        result = lib.expand_macro_call('G65 P3100 A1 B2 C3 I4 J5')
        line = result[0]
        assert 'X1' in line
        assert 'Y2' in line
        assert 'Z3' in line  # C -> #3
        assert 'I4' in line
        assert 'J5' in line


# ---------------------------------------------------------------------------
# Nested macro expansion
# ---------------------------------------------------------------------------

class TestNestedExpansion:
    def test_nested_m98(self, lib):
        inner = GCodeMacro(macro_id='O100', name='INNER', body=['G0 Z10'])
        outer = GCodeMacro(macro_id='O200', name='OUTER', body=['M98 P100', 'G0 X0'])
        lib.register_macro(inner)
        lib.register_macro(outer)
        result = lib.expand_macro_call('M98 P200')
        assert result == ['G0 Z10', 'G0 X0']

    def test_nested_two_levels(self, lib):
        lib.register_macro(GCodeMacro(macro_id='O10', name='L0', body=['G0 Z1']))
        lib.register_macro(GCodeMacro(macro_id='O20', name='L1', body=['M98 P10']))
        lib.register_macro(GCodeMacro(macro_id='O30', name='L2', body=['M98 P20']))
        result = lib.expand_macro_call('M98 P30')
        assert result == ['G0 Z1']

    def test_max_depth_raises(self, lib):
        """Recursive macro that calls itself should raise at depth > 5."""
        lib.register_macro(GCodeMacro(
            macro_id='O999', name='RECURSIVE', body=['M98 P999'],
        ))
        with pytest.raises(MacroExpansionError, match='maximum depth'):
            lib.expand_macro_call('M98 P999')


# ---------------------------------------------------------------------------
# O-word definition parsing
# ---------------------------------------------------------------------------

class TestOWordParsing:
    def test_parse_single_definition(self, lib):
        program = [
            'O1000 (MY_MACRO)',
            'G0 X0 Y0',
            'G0 Z5',
            'M99',
            'G1 X10 F500',
        ]
        remaining = lib.parse_macro_definitions(program)
        assert remaining == ['G1 X10 F500']
        macro = lib.get_macro('O1000')
        assert macro is not None
        assert macro.name == 'MY_MACRO'
        assert macro.body == ['G0 X0 Y0', 'G0 Z5']

    def test_parse_multiple_definitions(self, lib):
        program = [
            'O1000 (FIRST)',
            'G0 X1',
            'M99',
            'O2000 (SECOND)',
            'G0 X2',
            'M99',
            'G1 X100',
        ]
        remaining = lib.parse_macro_definitions(program)
        assert remaining == ['G1 X100']
        assert lib.get_macro('O1000') is not None
        assert lib.get_macro('O2000') is not None

    def test_parse_extracts_parameters(self, lib):
        program = [
            'O1500 (PARAMETRIC)',
            'G1 X#1 Y#2 Z#3',
            'M99',
        ]
        lib.parse_macro_definitions(program)
        macro = lib.get_macro('O1500')
        assert '#1' in macro.parameters
        assert '#2' in macro.parameters
        assert '#3' in macro.parameters

    def test_parse_with_description(self, lib):
        program = [
            'O4000 (NAME) (A useful description)',
            'G0 Z0',
            'M99',
        ]
        lib.parse_macro_definitions(program)
        macro = lib.get_macro('O4000')
        assert macro.name == 'NAME'
        assert 'useful description' in macro.description

    def test_parse_no_definitions(self, lib):
        program = ['G0 X0', 'G1 X10 F500', 'M30']
        remaining = lib.parse_macro_definitions(program)
        assert remaining == program

    def test_parse_empty_program(self, lib):
        remaining = lib.parse_macro_definitions([])
        assert remaining == []


# ---------------------------------------------------------------------------
# Built-in macro availability
# ---------------------------------------------------------------------------

class TestBuiltinMacros:
    def test_probe_z_available(self, lib):
        macro = lib.get_macro('O9000')
        assert macro is not None
        assert macro.name == 'PROBE_Z'
        assert 'G38.2' in ' '.join(macro.body)

    def test_tool_measure_available(self, lib):
        macro = lib.get_macro('O9001')
        assert macro is not None
        assert macro.name == 'TOOL_MEASURE'

    def test_safe_retract_available(self, lib):
        macro = lib.get_macro('O9002')
        assert macro is not None
        assert macro.name == 'SAFE_RETRACT'
        assert 'G28' in ' '.join(macro.body)

    def test_peck_drill_available(self, lib):
        macro = lib.get_macro('O9003')
        assert macro is not None
        assert macro.name == 'PECK_DRILL'

    def test_safe_retract_expansion(self, lib):
        result = lib.expand_macro_call('M98 P9002')
        assert result == ['G28 G91 Z0']

    def test_probe_z_g65_expansion(self, lib):
        result = lib.expand_macro_call('G65 P9000 A-25.0 B50')
        # #1 = A = -25.0, #2 = B = 50
        body_text = ' '.join(result)
        assert '-25.0' in body_text
        assert '50' in body_text


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:
    def test_whitespace_line(self, lib):
        result = lib.expand_macro_call('   ')
        assert result == ['   ']

    def test_m98_case_insensitive(self, lib, simple_macro):
        """Expansion should work regardless of case in M98 line."""
        lib.register_macro(simple_macro)
        # The upper() is done internally so mixed-case input should work.
        result = lib.expand_macro_call('m98 p1000')
        assert result == ['G0 X0 Y0', 'G0 Z5']

    def test_g65_case_insensitive(self, lib, param_macro):
        lib.register_macro(param_macro)
        result = lib.expand_macro_call('g65 p2000 a7 b8')
        assert 'X7' in result[0]
        assert 'Y8' in result[0]

    def test_macro_dataclass_defaults(self):
        m = GCodeMacro(macro_id='O1', name='test')
        assert m.parameters == []
        assert m.body == []
        assert m.description == ''

    def test_repeated_register_does_not_duplicate(self, lib):
        macro = GCodeMacro(macro_id='O1000', name='A', body=['G0 X0'])
        lib.register_macro(macro)
        lib.register_macro(macro)
        assert lib.get_macro('O1000').name == 'A'
