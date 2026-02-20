"""
Parameter declaration and validation utilities for MIRACLE nodes.

Provides type checking, range validation, and enum validation
for ROS2 node parameters.
"""

from typing import Any, Dict, List, Optional, Tuple, Type, Union

from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult,
)
from rclpy.node import Node
from rclpy.parameter import Parameter

from miracle_core.exceptions import ParameterValidationError


# Mapping from Python types to ROS2 Parameter types
_TYPE_MAP: Dict[Type, Parameter.Type] = {
    bool: Parameter.Type.BOOL,
    int: Parameter.Type.INTEGER,
    float: Parameter.Type.DOUBLE,
    str: Parameter.Type.STRING,
    list: Parameter.Type.STRING_ARRAY,
}


def declare_and_validate_parameters(
    node: Node,
    param_specs: Dict[str, Dict[str, Any]],
) -> Dict[str, Any]:
    """Declare and validate multiple parameters on a node.

    Args:
        node: The ROS2 node to declare parameters on.
        param_specs: Dictionary mapping parameter names to their specs.
            Each spec can contain:
            - 'default': Default value (required)
            - 'type': Python type (bool, int, float, str, list)
            - 'range': Tuple of (min, max) for numeric types
            - 'choices': List of valid values for enum-like parameters
            - 'description': Human-readable description

    Returns:
        Dictionary of parameter name to validated value.

    Raises:
        ParameterValidationError: If any parameter fails validation.
    """
    validated: Dict[str, Any] = {}

    for name, spec in param_specs.items():
        if 'default' not in spec:
            raise ParameterValidationError(
                f"Parameter '{name}' missing required 'default' key"
            )

        default_value = spec['default']
        param_type = spec.get('type', type(default_value))
        description = spec.get('description', '')
        value_range = spec.get('range')
        choices = spec.get('choices')

        # Build descriptor
        descriptor = ParameterDescriptor(description=description)

        if value_range is not None and param_type == float:
            descriptor.floating_point_range = [
                FloatingPointRange(
                    from_value=float(value_range[0]),
                    to_value=float(value_range[1]),
                    step=0.0,
                )
            ]
        elif value_range is not None and param_type == int:
            descriptor.integer_range = [
                IntegerRange(
                    from_value=int(value_range[0]),
                    to_value=int(value_range[1]),
                    step=0,
                )
            ]

        # Declare parameter
        try:
            node.declare_parameter(name, default_value, descriptor)
        except Exception as exc:
            raise ParameterValidationError(
                f"Failed to declare parameter '{name}': {exc}"
            ) from exc

        # Get actual value (may have been overridden by launch file)
        value = node.get_parameter(name).value

        # Validate type
        if not isinstance(value, param_type) and param_type in _TYPE_MAP:
            # Allow int-to-float coercion
            if param_type == float and isinstance(value, int):
                value = float(value)
            else:
                raise ParameterValidationError(
                    f"Parameter '{name}' expected type {param_type.__name__}, "
                    f"got {type(value).__name__}"
                )

        # Validate range
        if value_range is not None and isinstance(value, (int, float)):
            if value < value_range[0] or value > value_range[1]:
                raise ParameterValidationError(
                    f"Parameter '{name}' value {value} out of range "
                    f"[{value_range[0]}, {value_range[1]}]"
                )

        # Validate choices
        if choices is not None and value not in choices:
            raise ParameterValidationError(
                f"Parameter '{name}' value '{value}' not in allowed "
                f"choices: {choices}"
            )

        validated[name] = value

    return validated


def create_parameter_callback(
    node: Node,
    param_specs: Dict[str, Dict[str, Any]],
):
    """Create a parameter change callback that validates new values.

    Args:
        node: The ROS2 node.
        param_specs: Same format as declare_and_validate_parameters.

    Returns:
        Callback function suitable for add_on_set_parameters_callback.
    """

    def _callback(params: List[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name not in param_specs:
                continue

            spec = param_specs[param.name]
            value = param.value

            # Validate range
            value_range = spec.get('range')
            if value_range is not None and isinstance(value, (int, float)):
                if value < value_range[0] or value > value_range[1]:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f"Parameter '{param.name}' value {value} "
                            f"out of range [{value_range[0]}, {value_range[1]}]"
                        ),
                    )

            # Validate choices
            choices = spec.get('choices')
            if choices is not None and value not in choices:
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"Parameter '{param.name}' value '{value}' "
                        f"not in allowed choices: {choices}"
                    ),
                )

        return SetParametersResult(successful=True)

    return _callback
