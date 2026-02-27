"""Tests for the OPC-UA bridge node.

Extracts the core logic from OPCUABridgeNode -- node mappings,
exponential backoff reconnection, simulated state publishing, and
parameter validation -- and tests it without any ROS2 or opcua
dependency.

Covers:
- Package / class / entry-point importability
- Node mappings populated (10 entries)
- Exponential backoff delay sequence
- Max backoff cap at 60s
- Reconnect resets attempts on success
- Simulated state published when disconnected
- Parameter validation (poll_rate_hz, namespace_index, security_policy)
"""

from unittest.mock import MagicMock

import pytest


# ---------------------------------------------------------------------------
# Extracted helpers (mirror the logic in opc_ua_bridge.py)
# ---------------------------------------------------------------------------

def build_node_mappings(namespace_index: int = 2) -> dict:
    """Build OPC-UA node ID to ROS field mappings.

    Mirrors OPCUABridgeNode._do_configure mapping construction.
    """
    ns = namespace_index
    return {
        f'ns={ns};s=SpindleSpeed': 'spindle_speed',
        f'ns={ns};s=FeedRate': 'feed_rate',
        f'ns={ns};s=SpindleLoad': 'spindle_load',
        f'ns={ns};s=CoolantLevel': 'coolant_level',
        f'ns={ns};s=AxisPositionX': 'axis_x',
        f'ns={ns};s=AxisPositionY': 'axis_y',
        f'ns={ns};s=AxisPositionZ': 'axis_z',
        f'ns={ns};s=MachineStatus': 'status',
        f'ns={ns};s=CurrentProgram': 'current_program',
        f'ns={ns};s=CurrentLine': 'current_line',
    }


def compute_backoff_delay(
    attempt: int,
    base_backoff: float = 1.0,
    max_backoff: float = 60.0,
) -> float:
    """Compute exponential backoff delay for a given attempt number.

    Mirrors OPCUABridgeNode._schedule_reconnect:
        delay = min(base * 2^attempts, max_backoff)
    """
    return min(base_backoff * (2 ** attempt), max_backoff)


class _ReconnectTracker:
    """Simulates the reconnection state of OPCUABridgeNode.

    Mirrors _reconnect_attempts, _schedule_reconnect, and _retry_connect.
    """

    def __init__(
        self,
        base_backoff: float = 1.0,
        max_backoff: float = 60.0,
    ):
        self.reconnect_attempts: int = 0
        self.base_backoff = base_backoff
        self.max_backoff = max_backoff
        self.connected: bool = False
        self.scheduled_delays: list = []

    def schedule_reconnect(self) -> float:
        """Schedule a reconnect -- mirrors _schedule_reconnect."""
        delay = compute_backoff_delay(
            self.reconnect_attempts,
            self.base_backoff,
            self.max_backoff,
        )
        self.reconnect_attempts += 1
        self.scheduled_delays.append(delay)
        return delay

    def retry_connect(self, succeed: bool = True) -> None:
        """Attempt a reconnect -- mirrors _retry_connect."""
        if succeed:
            self.connected = True
            self.reconnect_attempts = 0
        else:
            self.connected = False
            self.schedule_reconnect()


class _SimulatedStatePublisher:
    """Simulates the _publish_simulated_state / _poll_opcua path."""

    def __init__(self, connected: bool = False, machine_id: str = 'cnc1'):
        self.connected = connected
        self.machine_id = machine_id
        self.published_messages: list = []

    def poll(self) -> None:
        """Mirrors _poll_opcua: publishes simulated state when disconnected."""
        if not self.connected:
            self.publish_simulated_state()

    def publish_simulated_state(self) -> None:
        """Mirrors _publish_simulated_state."""
        self.published_messages.append({
            'machine_id': self.machine_id,
            'status': 'IDLE',
            'axis_positions': [0.0] * 6,
            'axis_velocities': [0.0] * 6,
        })


def validate_poll_rate_hz(value: float) -> bool:
    """Validate poll_rate_hz is within the allowed range (0.1 .. 1000.0)."""
    return 0.1 <= value <= 1000.0


def validate_namespace_index(value: int) -> bool:
    """Validate namespace_index is within the allowed range (0 .. 100)."""
    return 0 <= value <= 100


def validate_security_policy(value: str) -> bool:
    """Validate security_policy is one of the allowed choices."""
    return value in ('None', 'Basic128Rsa15', 'Basic256', 'Basic256Sha256')


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def tracker():
    """Return a fresh reconnect tracker."""
    return _ReconnectTracker(base_backoff=1.0, max_backoff=60.0)


@pytest.fixture
def sim_publisher():
    """Return a disconnected simulated state publisher."""
    return _SimulatedStatePublisher(connected=False, machine_id='cnc1')


# ---------------------------------------------------------------------------
# 1. Package import
# ---------------------------------------------------------------------------

def test_package_import():
    """Verify the miracle_bridges package can be imported."""
    import miracle_bridges
    assert miracle_bridges is not None


# ---------------------------------------------------------------------------
# 2. Class existence
# ---------------------------------------------------------------------------

def test_opcua_bridge_class_exists():
    """Verify the OPCUABridgeNode class exists in the module."""
    from miracle_bridges.opc_ua_bridge import OPCUABridgeNode
    assert OPCUABridgeNode is not None


# ---------------------------------------------------------------------------
# 3. Node mappings populated (10 entries)
# ---------------------------------------------------------------------------

def test_node_mappings_has_10_entries():
    """The default node mappings should contain exactly 10 entries."""
    mappings = build_node_mappings(namespace_index=2)
    assert len(mappings) == 10


# ---------------------------------------------------------------------------
# 4. Exponential backoff delay sequence
# ---------------------------------------------------------------------------

def test_exponential_backoff_sequence():
    """Verify the delay doubles: 1 -> 2 -> 4 -> 8 -> 16 -> 32 -> 60 -> 60."""
    expected = [1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 60.0, 60.0]
    actual = [compute_backoff_delay(i) for i in range(8)]
    assert actual == expected


# ---------------------------------------------------------------------------
# 5. Max backoff cap at 60s
# ---------------------------------------------------------------------------

def test_max_backoff_cap():
    """After enough attempts, delay should never exceed max_backoff (60s)."""
    for attempt in range(20):
        delay = compute_backoff_delay(attempt, base_backoff=1.0, max_backoff=60.0)
        assert delay <= 60.0


# ---------------------------------------------------------------------------
# 6. Reconnect resets attempts on success
# ---------------------------------------------------------------------------

def test_reconnect_resets_on_success(tracker):
    """A successful retry_connect should reset reconnect_attempts to 0."""
    # Simulate several failed reconnects
    tracker.schedule_reconnect()
    tracker.schedule_reconnect()
    tracker.schedule_reconnect()
    assert tracker.reconnect_attempts == 3

    # Now succeed
    tracker.retry_connect(succeed=True)
    assert tracker.reconnect_attempts == 0
    assert tracker.connected is True


# ---------------------------------------------------------------------------
# 7. Simulated state published when disconnected
# ---------------------------------------------------------------------------

def test_simulated_state_published_when_disconnected(sim_publisher):
    """When disconnected, poll should publish a simulated IDLE state."""
    sim_publisher.poll()
    assert len(sim_publisher.published_messages) == 1
    msg = sim_publisher.published_messages[0]
    assert msg['status'] == 'IDLE'
    assert msg['machine_id'] == 'cnc1'
    assert msg['axis_positions'] == [0.0] * 6


# ---------------------------------------------------------------------------
# 8. poll_rate_hz parameter range validation
# ---------------------------------------------------------------------------

def test_poll_rate_hz_valid():
    """Default poll_rate_hz of 10.0 should be valid."""
    assert validate_poll_rate_hz(10.0) is True


def test_poll_rate_hz_too_low():
    """poll_rate_hz of 0.01 should be invalid (minimum is 0.1)."""
    assert validate_poll_rate_hz(0.01) is False


def test_poll_rate_hz_too_high():
    """poll_rate_hz of 1001.0 should be invalid (maximum is 1000.0)."""
    assert validate_poll_rate_hz(1001.0) is False


# ---------------------------------------------------------------------------
# 9. namespace_index parameter range validation
# ---------------------------------------------------------------------------

def test_namespace_index_valid():
    """Default namespace_index of 2 should be valid."""
    assert validate_namespace_index(2) is True


def test_namespace_index_too_high():
    """namespace_index of 101 should be invalid (maximum is 100)."""
    assert validate_namespace_index(101) is False


# ---------------------------------------------------------------------------
# 10. security_policy choices validation
# ---------------------------------------------------------------------------

def test_security_policy_valid_none():
    """'None' should be a valid security_policy choice."""
    assert validate_security_policy('None') is True


def test_security_policy_valid_sha256():
    """'Basic256Sha256' should be a valid security_policy choice."""
    assert validate_security_policy('Basic256Sha256') is True


def test_security_policy_invalid():
    """'TLS_PSK' should be an invalid security_policy choice."""
    assert validate_security_policy('TLS_PSK') is False
