"""
Serialization utilities for MIRACLE data types.

Provides JSON serialization/deserialization for ROS2 messages
and custom data structures.
"""

from typing import Any, Dict, List, Optional, Type
import json

from builtin_interfaces.msg import Time as TimeMsg


def msg_to_dict(msg: Any) -> Dict[str, Any]:
    """Convert a ROS2 message to a Python dictionary.

    Handles nested messages, arrays, and builtin types.

    Args:
        msg: A ROS2 message instance.

    Returns:
        Dictionary representation of the message.
    """
    result: Dict[str, Any] = {}

    for field_name in msg.get_fields_and_field_types():
        value = getattr(msg, field_name)

        if isinstance(value, TimeMsg):
            result[field_name] = {
                'sec': value.sec,
                'nanosec': value.nanosec,
            }
        elif hasattr(value, 'get_fields_and_field_types'):
            # Nested ROS2 message
            result[field_name] = msg_to_dict(value)
        elif isinstance(value, (list, tuple)):
            result[field_name] = [
                msg_to_dict(item)
                if hasattr(item, 'get_fields_and_field_types')
                else item
                for item in value
            ]
        elif isinstance(value, bytes):
            result[field_name] = list(value)
        else:
            result[field_name] = value

    return result


def msg_to_json(msg: Any, indent: Optional[int] = None) -> str:
    """Convert a ROS2 message to a JSON string.

    Args:
        msg: A ROS2 message instance.
        indent: JSON indentation level.

    Returns:
        JSON string representation.
    """
    return json.dumps(msg_to_dict(msg), indent=indent, default=str)


def dict_to_msg(data: Dict[str, Any], msg_type: Type) -> Any:
    """Populate a ROS2 message from a dictionary.

    Args:
        data: Dictionary with field values.
        msg_type: The ROS2 message class to instantiate.

    Returns:
        Populated message instance.
    """
    msg = msg_type()

    for field_name, value in data.items():
        if hasattr(msg, field_name):
            if isinstance(value, dict) and field_name == 'timestamp':
                time_msg = TimeMsg()
                time_msg.sec = value.get('sec', 0)
                time_msg.nanosec = value.get('nanosec', 0)
                setattr(msg, field_name, time_msg)
            elif isinstance(value, dict):
                field_type = type(getattr(msg, field_name))
                setattr(msg, field_name, dict_to_msg(value, field_type))
            else:
                setattr(msg, field_name, value)

    return msg
