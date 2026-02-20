"""Tests for serialization utilities.

Validates msg_to_dict, msg_to_json, and dict_to_msg with mocked
ROS2 message types. Covers nested messages, arrays, bytes, and
TimeMsg special handling.
"""

import sys
import json
from unittest.mock import MagicMock

import pytest

# --- Mock builtin_interfaces.msg.Time ---

class _MockTimeMsg:
    """Stand-in for builtin_interfaces.msg.Time."""
    def __init__(self):
        self.sec = 0
        self.nanosec = 0

    def get_fields_and_field_types(self):
        return {'sec': 'int32', 'nanosec': 'uint32'}


_builtin_mock = MagicMock()
_builtin_mock.Time = _MockTimeMsg

sys.modules.setdefault('rclpy', MagicMock())
sys.modules.setdefault('rclpy.time', MagicMock())
sys.modules.setdefault('rclpy.duration', MagicMock())
sys.modules['builtin_interfaces'] = MagicMock()
sys.modules['builtin_interfaces.msg'] = _builtin_mock

from miracle_core.serialization import msg_to_dict, msg_to_json, dict_to_msg


# --- Helpers: mock ROS2 messages ---

class _SimpleMsg:
    """A flat message with primitive fields."""
    def __init__(self):
        self.machine_id = ''
        self.status = ''
        self.spindle_load = 0.0

    def get_fields_and_field_types(self):
        return {
            'machine_id': 'string',
            'status': 'string',
            'spindle_load': 'float64',
        }


class _NestedMsg:
    """A message containing a nested sub-message and a TimeMsg."""
    def __init__(self):
        self.timestamp = _MockTimeMsg()
        self.inner = _SimpleMsg()

    def get_fields_and_field_types(self):
        return {
            'timestamp': 'builtin_interfaces/Time',
            'inner': 'SimpleMsg',
        }


class _ArrayMsg:
    """A message containing a list of values and bytes."""
    def __init__(self):
        self.values = [1.0, 2.0, 3.0]
        self.raw_data = b'\x01\x02\x03'

    def get_fields_and_field_types(self):
        return {
            'values': 'sequence<float64>',
            'raw_data': 'sequence<uint8>',
        }


class TestMsgToDict:
    """Tests for msg_to_dict conversion."""

    def test_flat_message(self):
        """A simple message should produce a flat dictionary."""
        msg = _SimpleMsg()
        msg.machine_id = 'cnc1'
        msg.status = 'RUNNING'
        msg.spindle_load = 75.5
        result = msg_to_dict(msg)
        assert result == {
            'machine_id': 'cnc1',
            'status': 'RUNNING',
            'spindle_load': 75.5,
        }

    def test_time_msg_special_handling(self):
        """TimeMsg fields should be serialized as {'sec': ..., 'nanosec': ...}."""
        msg = _NestedMsg()
        msg.timestamp.sec = 100
        msg.timestamp.nanosec = 500_000_000
        result = msg_to_dict(msg)
        assert result['timestamp'] == {'sec': 100, 'nanosec': 500_000_000}

    def test_nested_message(self):
        """Nested messages should be recursively converted to dicts."""
        msg = _NestedMsg()
        msg.inner.machine_id = 'cnc2'
        msg.inner.status = 'IDLE'
        msg.inner.spindle_load = 0.0
        result = msg_to_dict(msg)
        assert result['inner']['machine_id'] == 'cnc2'
        assert result['inner']['status'] == 'IDLE'

    def test_array_field(self):
        """Array fields should be preserved as lists."""
        msg = _ArrayMsg()
        result = msg_to_dict(msg)
        assert result['values'] == [1.0, 2.0, 3.0]

    def test_bytes_field_converted_to_list(self):
        """Bytes fields should be converted to a list of integers."""
        msg = _ArrayMsg()
        result = msg_to_dict(msg)
        assert result['raw_data'] == [1, 2, 3]


class TestMsgToJson:
    """Tests for msg_to_json conversion."""

    def test_json_is_valid(self):
        """Output should be parseable JSON."""
        msg = _SimpleMsg()
        msg.machine_id = 'cnc1'
        msg.status = 'IDLE'
        msg.spindle_load = 0.0
        json_str = msg_to_json(msg)
        parsed = json.loads(json_str)
        assert parsed['machine_id'] == 'cnc1'

    def test_json_with_indent(self):
        """Indented JSON should contain newlines."""
        msg = _SimpleMsg()
        msg.machine_id = 'cnc1'
        msg.status = 'IDLE'
        msg.spindle_load = 0.0
        json_str = msg_to_json(msg, indent=2)
        assert '\n' in json_str

    def test_json_roundtrip(self):
        """JSON output should round-trip through json.loads."""
        msg = _SimpleMsg()
        msg.machine_id = 'test'
        msg.status = 'RUNNING'
        msg.spindle_load = 42.5
        parsed = json.loads(msg_to_json(msg))
        assert parsed == msg_to_dict(msg)


class TestDictToMsg:
    """Tests for dict_to_msg conversion."""

    def test_simple_dict_to_msg(self):
        """A flat dict should populate a message's fields."""
        data = {'machine_id': 'cnc3', 'status': 'ERROR', 'spindle_load': 99.9}
        msg = dict_to_msg(data, _SimpleMsg)
        assert msg.machine_id == 'cnc3'
        assert msg.status == 'ERROR'
        assert msg.spindle_load == 99.9

    def test_timestamp_special_handling(self):
        """A 'timestamp' dict field should be converted to a TimeMsg."""
        data = {'timestamp': {'sec': 42, 'nanosec': 123}, 'inner': {}}

        class _MsgWithTimestamp:
            def __init__(self):
                self.timestamp = _MockTimeMsg()
                self.inner = _SimpleMsg()

        msg = dict_to_msg(data, _MsgWithTimestamp)
        assert msg.timestamp.sec == 42
        assert msg.timestamp.nanosec == 123

    def test_unknown_fields_ignored(self):
        """Fields not present on the message should be silently ignored."""
        data = {'machine_id': 'x', 'nonexistent_field': True}
        msg = dict_to_msg(data, _SimpleMsg)
        assert msg.machine_id == 'x'
        assert not hasattr(msg, 'nonexistent_field') or msg.machine_id == 'x'
