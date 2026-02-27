"""Tests for miracle_unity_bridge package.

Covers:
- Package / class / entry-point importability
- TCP startup parameter handling
- MachineState message serialization
- Topic registration verification
- Connection status tracking
- Keepalive timeout detection
- Reconnection after disconnect (heartbeat recovery)
- Multiple topic subscriptions
"""

import time
from unittest.mock import MagicMock, patch, PropertyMock

import pytest


# ---------------------------------------------------------------------------
# Helpers -- lightweight stubs for rclpy so we never need a real ROS graph
# ---------------------------------------------------------------------------

class _FakeParameterValue:
    """Mimics rclpy ParameterValue for string / int / float."""

    def __init__(self, value):
        self._value = value

    @property
    def string_value(self):
        return str(self._value)

    @property
    def integer_value(self):
        return int(self._value)

    @property
    def double_value(self):
        return float(self._value)


class _FakeParameter:
    """Mimics rclpy Parameter."""

    def __init__(self, value):
        self._pv = _FakeParameterValue(value)

    def get_parameter_value(self):
        return self._pv


_DEFAULT_PARAMS = {
    'tcp_ip': '0.0.0.0',
    'tcp_port': 10000,
    'keepalive_timeout': 10.0,
    'log_level': 'info',
}


@pytest.fixture()
def mock_rclpy():
    """Patch rclpy.node.Node so we can instantiate without a ROS context."""
    with patch('rclpy.node.Node.__init__', return_value=None), \
         patch('rclpy.node.Node.declare_parameter', side_effect=lambda n, v: None), \
         patch('rclpy.node.Node.get_parameter', side_effect=lambda n: _FakeParameter(_DEFAULT_PARAMS.get(n, ''))), \
         patch('rclpy.node.Node.create_publisher', return_value=MagicMock()) as mock_pub, \
         patch('rclpy.node.Node.create_subscription', return_value=MagicMock()) as mock_sub, \
         patch('rclpy.node.Node.create_timer', return_value=MagicMock()) as mock_timer, \
         patch('rclpy.node.Node.get_logger', return_value=MagicMock()):
        yield {
            'create_publisher': mock_pub,
            'create_subscription': mock_sub,
            'create_timer': mock_timer,
        }


@pytest.fixture()
def endpoint(mock_rclpy):
    """Return a fully-constructed UnityEndpointConfig with mocked rclpy."""
    from miracle_unity_bridge.unity_endpoint import UnityEndpointConfig
    node = UnityEndpointConfig()
    node._mock_rclpy = mock_rclpy
    return node


# ---------------------------------------------------------------------------
# 1. Package import
# ---------------------------------------------------------------------------

def test_package_import():
    """Verify the package can be imported."""
    import miracle_unity_bridge
    assert miracle_unity_bridge is not None


# ---------------------------------------------------------------------------
# 2. Class existence
# ---------------------------------------------------------------------------

def test_endpoint_config_class_exists():
    """Verify the UnityEndpointConfig class exists."""
    from miracle_unity_bridge.unity_endpoint import UnityEndpointConfig
    assert UnityEndpointConfig is not None


# ---------------------------------------------------------------------------
# 3. Entry-point callable
# ---------------------------------------------------------------------------

def test_main_function_exists():
    """Verify main entry point exists."""
    from miracle_unity_bridge.unity_endpoint import main
    assert callable(main)


# ---------------------------------------------------------------------------
# 4. TCP startup -- parameters are declared and publisher / timer created
# ---------------------------------------------------------------------------

def test_tcp_startup_creates_publisher_and_timer(endpoint, mock_rclpy):
    """After construction the node should have created a status publisher
    and a health-check timer."""
    mock_rclpy['create_publisher'].assert_called()
    mock_rclpy['create_timer'].assert_called()


# ---------------------------------------------------------------------------
# 5. MachineState message serialization helper
# ---------------------------------------------------------------------------

def test_machine_state_serialization():
    """Verify that MachineState fields can be populated without error.

    This is an import / construction test -- it does NOT require a running
    ROS graph because we only exercise the generated Python dataclass.
    """
    try:
        from miracle_msgs.msg import MachineState
        msg = MachineState()
        msg.machine_id = 'cnc1'
        msg.status = 'IDLE'
        msg.spindle_speed = 1200.0
        msg.feed_rate = 500.0
        msg.axis_positions = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]
        msg.axis_velocities = [0.0] * 6
        assert msg.machine_id == 'cnc1'
        assert msg.spindle_speed == 1200.0
    except ImportError:
        pytest.skip("miracle_msgs not built / available in this environment")


# ---------------------------------------------------------------------------
# 6. Topic registration verification
# ---------------------------------------------------------------------------

def test_registered_topic_types_not_empty():
    """The REGISTERED_TOPIC_TYPES list should contain all expected
    message and service types."""
    from miracle_unity_bridge.unity_endpoint import REGISTERED_TOPIC_TYPES
    assert len(REGISTERED_TOPIC_TYPES) >= 10
    assert 'miracle_msgs/msg/MachineState' in REGISTERED_TOPIC_TYPES
    assert 'miracle_msgs/msg/Heartbeat' in REGISTERED_TOPIC_TYPES
    assert 'miracle_msgs/srv/TriggerEStop' in REGISTERED_TOPIC_TYPES


# ---------------------------------------------------------------------------
# 7. Connection status tracking -- initially disconnected
# ---------------------------------------------------------------------------

def test_initial_connection_status_is_false(endpoint):
    """The node should start with _unity_connected == False."""
    assert endpoint._unity_connected is False


# ---------------------------------------------------------------------------
# 8. Keepalive timeout detection
# ---------------------------------------------------------------------------

def test_keepalive_timeout_marks_disconnected(endpoint):
    """If _last_msg_time is far in the past, _check_health should mark
    the connection as lost and log a warning."""
    # Simulate that Unity was connected and then went silent
    endpoint._unity_connected = True
    endpoint._last_msg_time = time.monotonic() - 20.0  # 20 s ago
    endpoint._keepalive_timeout = 10.0

    endpoint._publish_connection_status = MagicMock()
    endpoint._check_health()

    assert endpoint._unity_connected is False
    endpoint._publish_connection_status.assert_called_once()


# ---------------------------------------------------------------------------
# 9. Reconnection after disconnect (heartbeat recovery)
# ---------------------------------------------------------------------------

def test_heartbeat_reconnects_after_timeout(endpoint):
    """Receiving a heartbeat after a timeout should restore connected state."""
    from std_msgs.msg import String

    # Start disconnected
    endpoint._unity_connected = False
    endpoint._publish_connection_status = MagicMock()

    heartbeat_msg = MagicMock(spec=String)
    heartbeat_msg.data = 'alive'

    endpoint._on_unity_heartbeat(heartbeat_msg)

    assert endpoint._unity_connected is True
    endpoint._publish_connection_status.assert_called_once()


# ---------------------------------------------------------------------------
# 10. Multiple topic subscriptions -- heartbeat sub is created
# ---------------------------------------------------------------------------

def test_heartbeat_subscription_created(endpoint, mock_rclpy):
    """The constructor must subscribe to /miracle/unity/heartbeat."""
    calls = mock_rclpy['create_subscription'].call_args_list
    topic_names = [c[0][1] for c in calls]
    assert '/miracle/unity/heartbeat' in topic_names


# ---------------------------------------------------------------------------
# 11. record_message_received refreshes keepalive
# ---------------------------------------------------------------------------

def test_record_message_received_updates_timestamp(endpoint):
    """Calling record_message_received should refresh _last_msg_time."""
    old_time = endpoint._last_msg_time
    time.sleep(0.01)
    endpoint.record_message_received()
    assert endpoint._last_msg_time > old_time
