"""Tests for parameter declaration and validation utilities.

Validates range checking, type checking, choice validation, and the
parameter change callback generator. Mocks all ROS2 dependencies.
"""

import sys
from unittest.mock import MagicMock, patch, PropertyMock

import pytest


# --- Provide a real SetParametersResult so the callback can construct it ---

class _SetParametersResult:
    """Stand-in for rcl_interfaces.msg.SetParametersResult."""
    def __init__(self, successful=True, reason=''):
        self.successful = successful
        self.reason = reason


# --- Mock ROS2 dependencies ---
_rcl_msg_mock = MagicMock()
_rcl_msg_mock.SetParametersResult = _SetParametersResult

sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.parameter'] = MagicMock()
sys.modules['rcl_interfaces'] = MagicMock()
sys.modules['rcl_interfaces.msg'] = _rcl_msg_mock

# Import the validation error (no ROS deps)
from miracle_core.exceptions import ParameterValidationError

# We test the pure validation logic by calling the functions with mock nodes.
# Re-import after mocks are in place.
from miracle_core.parameter_validation import (
    declare_and_validate_parameters,
    create_parameter_callback,
)


def _make_mock_node(param_values: dict):
    """Create a mock ROS2 Node that returns given parameter values."""
    node = MagicMock()

    def _declare_param(name, default_value, descriptor):
        pass  # no-op in mock

    node.declare_parameter = _declare_param

    def _get_param(name):
        mock_param = MagicMock()
        mock_param.value = param_values.get(name, None)
        return mock_param

    node.get_parameter = _get_param
    return node


class TestDeclareAndValidateParameters:
    """Tests for declare_and_validate_parameters."""

    def test_missing_default_key_raises(self):
        """Spec without 'default' should raise ParameterValidationError."""
        node = _make_mock_node({})
        with pytest.raises(ParameterValidationError, match="missing required 'default'"):
            declare_and_validate_parameters(node, {
                'bad_param': {'type': int}  # no 'default'
            })

    def test_valid_parameters_returned(self):
        """Valid parameters should be declared and returned as a dict."""
        node = _make_mock_node({'rate_hz': 10.0, 'machine_id': 'cnc1'})
        result = declare_and_validate_parameters(node, {
            'rate_hz': {'default': 10.0, 'type': float, 'range': (1.0, 100.0)},
            'machine_id': {'default': 'cnc1', 'type': str},
        })
        assert result['rate_hz'] == 10.0, "rate_hz should be 10.0"
        assert result['machine_id'] == 'cnc1', "machine_id should be 'cnc1'"

    def test_range_violation_raises(self):
        """Value outside the declared range should raise ParameterValidationError."""
        node = _make_mock_node({'value': 200.0})
        with pytest.raises(ParameterValidationError, match="out of range"):
            declare_and_validate_parameters(node, {
                'value': {'default': 10.0, 'type': float, 'range': (0.0, 100.0)},
            })

    def test_range_boundary_low_accepted(self):
        """Value exactly at the lower range boundary should be accepted."""
        node = _make_mock_node({'value': 0.0})
        result = declare_and_validate_parameters(node, {
            'value': {'default': 50.0, 'type': float, 'range': (0.0, 100.0)},
        })
        assert result['value'] == 0.0

    def test_range_boundary_high_accepted(self):
        """Value exactly at the upper range boundary should be accepted."""
        node = _make_mock_node({'value': 100.0})
        result = declare_and_validate_parameters(node, {
            'value': {'default': 50.0, 'type': float, 'range': (0.0, 100.0)},
        })
        assert result['value'] == 100.0

    def test_choices_violation_raises(self):
        """Value not in the choices list should raise ParameterValidationError."""
        node = _make_mock_node({'policy': 'delete'})
        with pytest.raises(ParameterValidationError, match="not in allowed choices"):
            declare_and_validate_parameters(node, {
                'policy': {
                    'default': 'allow',
                    'type': str,
                    'choices': ['allow', 'deny'],
                },
            })

    def test_int_to_float_coercion(self):
        """An int value for a float parameter should be coerced automatically."""
        node = _make_mock_node({'value': 5})
        result = declare_and_validate_parameters(node, {
            'value': {'default': 5.0, 'type': float},
        })
        assert isinstance(result['value'], float), "int should be coerced to float"
        assert result['value'] == 5.0


class TestParameterCallback:
    """Tests for create_parameter_callback."""

    def _make_mock_param(self, name, value):
        p = MagicMock()
        p.name = name
        p.value = value
        return p

    def test_callback_accepts_valid_value(self):
        """Callback should return successful=True for a valid value."""
        node = MagicMock()
        specs = {'rate': {'default': 10.0, 'range': (1.0, 100.0)}}
        cb = create_parameter_callback(node, specs)
        result = cb([self._make_mock_param('rate', 50.0)])
        assert result.successful is True

    def test_callback_rejects_out_of_range(self):
        """Callback should return successful=False for out-of-range values."""
        node = MagicMock()
        specs = {'rate': {'default': 10.0, 'range': (1.0, 100.0)}}
        cb = create_parameter_callback(node, specs)
        result = cb([self._make_mock_param('rate', 999.0)])
        assert result.successful is False
        assert "out of range" in result.reason

    def test_callback_rejects_invalid_choice(self):
        """Callback should return successful=False for value not in choices."""
        node = MagicMock()
        specs = {'mode': {'default': 'auto', 'choices': ['auto', 'manual']}}
        cb = create_parameter_callback(node, specs)
        result = cb([self._make_mock_param('mode', 'turbo')])
        assert result.successful is False
        assert "not in allowed choices" in result.reason

    def test_callback_ignores_unknown_parameters(self):
        """Callback should pass through parameters not in the spec."""
        node = MagicMock()
        specs = {'known': {'default': 1.0}}
        cb = create_parameter_callback(node, specs)
        result = cb([self._make_mock_param('unknown_param', 'anything')])
        assert result.successful is True
