"""Tests for QoS profile configurations.

Verifies that each QoS profile has the correct reliability, durability,
history, and depth settings appropriate for its data type.
Mocks rclpy since we do not require a ROS2 runtime.
"""

import importlib
import sys
from unittest.mock import MagicMock
from enum import Enum

import pytest

# --- Mock rclpy QoS types before importing the module under test ---

class _MockReliability(Enum):
    BEST_EFFORT = 1
    RELIABLE = 2

class _MockDurability(Enum):
    VOLATILE = 1
    TRANSIENT_LOCAL = 2

class _MockHistory(Enum):
    KEEP_LAST = 1
    KEEP_ALL = 2

class _MockLiveliness(Enum):
    AUTOMATIC = 1
    MANUAL_BY_TOPIC = 2


class _MockQoSProfile:
    """Lightweight stand-in for rclpy.qos.QoSProfile."""
    def __init__(self, **kwargs):
        self.reliability = kwargs.get('reliability')
        self.durability = kwargs.get('durability')
        self.history = kwargs.get('history')
        self.depth = kwargs.get('depth')
        self.liveliness = kwargs.get('liveliness')
        self.deadline = kwargs.get('deadline')


class _MockDuration:
    def __init__(self, seconds=0, nanoseconds=0):
        self.seconds = seconds
        self.nanoseconds = nanoseconds


# Wire up the mock module hierarchy
_mock_qos = MagicMock()
_mock_qos.QoSProfile = _MockQoSProfile
_mock_qos.QoSReliabilityPolicy = _MockReliability
_mock_qos.QoSHistoryPolicy = _MockHistory
_mock_qos.QoSDurabilityPolicy = _MockDurability
_mock_qos.QoSLivelinessPolicy = _MockLiveliness

_mock_duration = MagicMock()
_mock_duration.Duration = _MockDuration

sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.qos'] = _mock_qos
sys.modules['rclpy.duration'] = _mock_duration

# Force a fresh import so the mocks above are used even in combined test runs
sys.modules.pop('miracle_core.qos_profiles', None)
from miracle_core.qos_profiles import QoSProfiles


class TestSensorDataQoS:
    """Tests for the sensor_data QoS profile."""

    def test_sensor_data_best_effort(self):
        """Sensor data should use best-effort reliability for low latency."""
        profile = QoSProfiles.sensor_data()
        assert profile.reliability == _MockReliability.BEST_EFFORT, (
            "Sensor data QoS must use BEST_EFFORT reliability"
        )

    def test_sensor_data_volatile(self):
        """Sensor data should be volatile since historical values are not needed."""
        profile = QoSProfiles.sensor_data()
        assert profile.durability == _MockDurability.VOLATILE

    def test_sensor_data_depth(self):
        """Sensor data should keep a small queue (5)."""
        profile = QoSProfiles.sensor_data()
        assert profile.depth == 5

    def test_sensor_data_has_deadline(self):
        """Sensor data should specify a deadline for freshness detection."""
        profile = QoSProfiles.sensor_data()
        assert profile.deadline is not None


class TestStateDataQoS:
    """Tests for the state_data QoS profile."""

    def test_state_data_reliable(self):
        """State data must be reliably delivered."""
        profile = QoSProfiles.state_data()
        assert profile.reliability == _MockReliability.RELIABLE

    def test_state_data_transient_local(self):
        """State data should use transient local durability for late joiners."""
        profile = QoSProfiles.state_data()
        assert profile.durability == _MockDurability.TRANSIENT_LOCAL

    def test_state_data_depth_one(self):
        """State data should keep only the last value."""
        profile = QoSProfiles.state_data()
        assert profile.depth == 1


class TestCommandQoS:
    """Tests for the command QoS profile."""

    def test_command_reliable(self):
        """Commands must be reliably delivered."""
        profile = QoSProfiles.command()
        assert profile.reliability == _MockReliability.RELIABLE

    def test_command_volatile(self):
        """Commands should be volatile (not stored for late joiners)."""
        profile = QoSProfiles.command()
        assert profile.durability == _MockDurability.VOLATILE

    def test_command_depth(self):
        """Commands should queue up to 10 messages."""
        profile = QoSProfiles.command()
        assert profile.depth == 10


class TestAlertQoS:
    """Tests for the alert QoS profile."""

    def test_alert_reliable(self):
        """Alerts must never be lost."""
        profile = QoSProfiles.alert()
        assert profile.reliability == _MockReliability.RELIABLE

    def test_alert_keep_all(self):
        """Alerts should keep all history so none are dropped."""
        profile = QoSProfiles.alert()
        assert profile.history == _MockHistory.KEEP_ALL

    def test_alert_transient_local(self):
        """Alerts should persist for late-joining subscribers."""
        profile = QoSProfiles.alert()
        assert profile.durability == _MockDurability.TRANSIENT_LOCAL


class TestHeartbeatQoS:
    """Tests for the heartbeat QoS profile."""

    def test_heartbeat_best_effort(self):
        """Heartbeat can be best-effort; missing one IS the failure signal."""
        profile = QoSProfiles.heartbeat()
        assert profile.reliability == _MockReliability.BEST_EFFORT

    def test_heartbeat_depth_one(self):
        """Heartbeat only needs the latest value."""
        profile = QoSProfiles.heartbeat()
        assert profile.depth == 1


class TestBulkDataQoS:
    """Tests for the bulk_data QoS profile."""

    def test_bulk_reliable(self):
        """Bulk transfers must be reliable."""
        profile = QoSProfiles.bulk_data()
        assert profile.reliability == _MockReliability.RELIABLE

    def test_bulk_depth(self):
        """Bulk transfers should have a larger queue (50)."""
        profile = QoSProfiles.bulk_data()
        assert profile.depth == 50


class TestLoggingQoS:
    """Tests for the logging QoS profile."""

    def test_logging_reliable(self):
        """Audit logs must be reliably delivered."""
        profile = QoSProfiles.logging()
        assert profile.reliability == _MockReliability.RELIABLE

    def test_logging_keep_all(self):
        """Audit logs must keep all history for compliance."""
        profile = QoSProfiles.logging()
        assert profile.history == _MockHistory.KEEP_ALL
