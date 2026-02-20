"""Tests for Modbus bridge register reading and data conversion logic.

Extracts the core algorithmic logic from ModbusBridgeNode -- float32
assembly from 16-bit registers, register address mapping, connection
retry behaviour, and scaling/calibration factors -- and tests it
without any ROS2 or pymodbus dependency.
"""

import struct
import pytest


# ---------------------------------------------------------------------------
# Extracted helpers (mirror the logic in modbus_bridge.py)
# ---------------------------------------------------------------------------

# Default register map from ModbusBridgeNode._do_configure
_DEFAULT_REGISTER_MAP = {
    'spindle_speed': {'address': 0, 'count': 2, 'type': 'float32'},
    'feed_rate':     {'address': 2, 'count': 2, 'type': 'float32'},
    'spindle_load':  {'address': 4, 'count': 1, 'type': 'uint16'},
    'coolant_temp':  {'address': 5, 'count': 1, 'type': 'uint16'},
    'axis_x':        {'address': 10, 'count': 2, 'type': 'float32'},
    'axis_y':        {'address': 12, 'count': 2, 'type': 'float32'},
    'axis_z':        {'address': 14, 'count': 2, 'type': 'float32'},
}


def decode_register_value(registers, reg_type, reg_count):
    """Decode raw Modbus registers into a Python float.

    Mirrors the decoding branch inside ModbusBridgeNode._poll_registers:
      - float32 with count==2: pack two uint16 big-endian, unpack as float.
      - otherwise: cast the first register to float.
    """
    if reg_type == 'float32' and reg_count == 2:
        raw = struct.pack('>HH', *registers)
        return struct.unpack('>f', raw)[0]
    return float(registers[0])


def encode_float32_to_registers(value):
    """Encode a Python float into two big-endian uint16 Modbus registers."""
    raw = struct.pack('>f', value)
    return list(struct.unpack('>HH', raw))


class _ModbusConnectionTracker:
    """Simulates the connection/retry logic of ModbusBridgeNode.

    Tracks connection attempts and whether the bridge considers itself
    connected, mirroring the _connect_modbus / _disconnect_modbus /
    _poll_registers error-handling paths.
    """

    def __init__(self, max_retries=3):
        self.connected = False
        self.attempt_count = 0
        self.max_retries = max_retries
        self.last_error = None

    def connect(self, succeed=True):
        """Attempt a connection (mirrors _connect_modbus)."""
        self.attempt_count += 1
        if succeed:
            self.connected = True
            self.last_error = None
        else:
            self.connected = False
            self.last_error = 'connection refused'

    def disconnect(self):
        """Disconnect (mirrors _disconnect_modbus)."""
        self.connected = False

    def poll(self, should_error=False):
        """Attempt a poll cycle (mirrors _poll_registers error path).

        Returns True if poll succeeded.
        """
        if not self.connected:
            return False
        if should_error:
            self.connected = False
            self.last_error = 'poll error'
            return False
        return True

    def retry_connect(self, outcomes):
        """Try connecting multiple times with given outcomes.

        Args:
            outcomes: list of bools indicating success per attempt.

        Returns:
            True if eventually connected.
        """
        for succeed in outcomes:
            if self.attempt_count >= self.max_retries:
                break
            self.connect(succeed=succeed)
            if self.connected:
                return True
        return self.connected


def apply_scaling(raw_value, scale=1.0, offset=0.0):
    """Apply linear scaling/calibration to a raw sensor value.

    In many Modbus integrations, the raw register value must be scaled:
        calibrated = raw * scale + offset
    """
    return raw_value * scale + offset


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def register_map():
    """Return a copy of the default register map."""
    return dict(_DEFAULT_REGISTER_MAP)


@pytest.fixture
def tracker():
    """Return a fresh connection tracker."""
    return _ModbusConnectionTracker(max_retries=3)


# ---------------------------------------------------------------------------
# Test classes
# ---------------------------------------------------------------------------

class TestFloat32Assembly:
    """Test float32 value assembly from two 16-bit Modbus registers."""

    def test_positive_float(self):
        """A known positive float should survive encode/decode round-trip."""
        value = 1234.5
        regs = encode_float32_to_registers(value)
        decoded = decode_register_value(regs, 'float32', 2)
        assert decoded == pytest.approx(value, rel=1e-5)

    def test_negative_float(self):
        """Negative floats should survive round-trip conversion."""
        value = -567.89
        regs = encode_float32_to_registers(value)
        decoded = decode_register_value(regs, 'float32', 2)
        assert decoded == pytest.approx(value, rel=1e-4)

    def test_zero_float(self):
        """Zero should encode and decode cleanly."""
        regs = encode_float32_to_registers(0.0)
        decoded = decode_register_value(regs, 'float32', 2)
        assert decoded == pytest.approx(0.0)

    def test_very_small_float(self):
        """A very small positive float (sub-normal territory)."""
        value = 1.0e-10
        regs = encode_float32_to_registers(value)
        decoded = decode_register_value(regs, 'float32', 2)
        assert decoded == pytest.approx(value, rel=1e-3)

    def test_large_float(self):
        """A large float should round-trip within float32 precision."""
        value = 99999.0
        regs = encode_float32_to_registers(value)
        decoded = decode_register_value(regs, 'float32', 2)
        assert decoded == pytest.approx(value, rel=1e-5)

    def test_register_byte_order_big_endian(self):
        """Registers are packed big-endian (high word first)."""
        value = 1.0
        raw = struct.pack('>f', value)
        hi, lo = struct.unpack('>HH', raw)
        regs = encode_float32_to_registers(value)
        assert regs == [hi, lo], "Registers should be [high_word, low_word]"

    def test_uint16_single_register(self):
        """A single uint16 register should be returned as a float."""
        decoded = decode_register_value([4567], 'uint16', 1)
        assert decoded == pytest.approx(4567.0)

    def test_uint16_zero(self):
        """A single uint16 register with value 0."""
        decoded = decode_register_value([0], 'uint16', 1)
        assert decoded == pytest.approx(0.0)

    def test_uint16_max(self):
        """A single uint16 register at maximum value (65535)."""
        decoded = decode_register_value([65535], 'uint16', 1)
        assert decoded == pytest.approx(65535.0)


class TestRegisterAddressMapping:
    """Test the register address map structure and lookups."""

    def test_all_expected_registers_present(self, register_map):
        """The default map should contain all seven expected registers."""
        expected_names = {
            'spindle_speed', 'feed_rate', 'spindle_load',
            'coolant_temp', 'axis_x', 'axis_y', 'axis_z',
        }
        assert set(register_map.keys()) == expected_names

    def test_no_address_overlap(self, register_map):
        """Register address ranges must not overlap."""
        ranges = []
        for name, reg in register_map.items():
            start = reg['address']
            end = start + reg['count'] - 1
            ranges.append((start, end, name))
        ranges.sort()
        for i in range(len(ranges) - 1):
            _, end_a, name_a = ranges[i]
            start_b, _, name_b = ranges[i + 1]
            assert end_a < start_b, (
                f"Register '{name_a}' (ends at {end_a}) overlaps with "
                f"'{name_b}' (starts at {start_b})"
            )

    def test_float32_registers_have_count_two(self, register_map):
        """All float32-typed registers should read 2 registers."""
        for name, reg in register_map.items():
            if reg['type'] == 'float32':
                assert reg['count'] == 2, (
                    f"float32 register '{name}' should have count=2"
                )

    def test_uint16_registers_have_count_one(self, register_map):
        """All uint16-typed registers should read 1 register."""
        for name, reg in register_map.items():
            if reg['type'] == 'uint16':
                assert reg['count'] == 1, (
                    f"uint16 register '{name}' should have count=1"
                )

    def test_axis_registers_are_contiguous(self, register_map):
        """The three axis registers should be laid out contiguously."""
        ax = register_map['axis_x']['address']
        ay = register_map['axis_y']['address']
        az = register_map['axis_z']['address']
        assert ay == ax + 2, "axis_y should start 2 registers after axis_x"
        assert az == ay + 2, "axis_z should start 2 registers after axis_y"


class TestConnectionRetryLogic:
    """Test connection and retry behaviour."""

    def test_initial_state_disconnected(self, tracker):
        """Tracker should start disconnected."""
        assert tracker.connected is False
        assert tracker.attempt_count == 0

    def test_successful_connect(self, tracker):
        """A successful connect should set connected=True."""
        tracker.connect(succeed=True)
        assert tracker.connected is True
        assert tracker.attempt_count == 1

    def test_failed_connect(self, tracker):
        """A failed connect should remain disconnected and record error."""
        tracker.connect(succeed=False)
        assert tracker.connected is False
        assert tracker.last_error == 'connection refused'

    def test_disconnect_clears_state(self, tracker):
        """Disconnecting should set connected=False."""
        tracker.connect(succeed=True)
        assert tracker.connected is True
        tracker.disconnect()
        assert tracker.connected is False

    def test_poll_when_disconnected_returns_false(self, tracker):
        """Polling while disconnected should return False without error."""
        result = tracker.poll()
        assert result is False

    def test_poll_success(self, tracker):
        """Polling while connected should succeed."""
        tracker.connect(succeed=True)
        result = tracker.poll()
        assert result is True
        assert tracker.connected is True

    def test_poll_error_disconnects(self, tracker):
        """A poll error should set connected=False (match source behaviour)."""
        tracker.connect(succeed=True)
        result = tracker.poll(should_error=True)
        assert result is False
        assert tracker.connected is False

    def test_retry_succeeds_on_second_attempt(self, tracker):
        """Retry should succeed if the second attempt works."""
        result = tracker.retry_connect([False, True])
        assert result is True
        assert tracker.attempt_count == 2

    def test_retry_all_failures(self, tracker):
        """If all retry attempts fail, should remain disconnected."""
        result = tracker.retry_connect([False, False, False])
        assert result is False
        assert tracker.connected is False

    def test_retry_respects_max_retries(self, tracker):
        """Retry should stop after max_retries attempts."""
        outcomes = [False] * 10  # More outcomes than max_retries
        tracker.retry_connect(outcomes)
        assert tracker.attempt_count == tracker.max_retries


class TestScalingCalibration:
    """Test linear scaling/calibration of raw register values."""

    def test_identity_scaling(self):
        """Scale=1.0 and offset=0.0 should return raw value unchanged."""
        assert apply_scaling(100.0) == pytest.approx(100.0)

    def test_scale_factor(self):
        """A scale factor should multiply the raw value."""
        assert apply_scaling(100.0, scale=0.1) == pytest.approx(10.0)

    def test_offset(self):
        """An offset should be added after scaling."""
        assert apply_scaling(0.0, scale=1.0, offset=273.15) == pytest.approx(273.15)

    def test_scale_and_offset(self):
        """Combined scale and offset: calibrated = raw * scale + offset."""
        # Convert raw ADC count to temperature: temp = count * 0.01 - 40.0
        assert apply_scaling(5000, scale=0.01, offset=-40.0) == pytest.approx(10.0)

    def test_negative_scale(self):
        """Negative scale should invert the value."""
        assert apply_scaling(10.0, scale=-1.0) == pytest.approx(-10.0)

    def test_zero_raw_value(self):
        """Zero raw with non-zero offset should return the offset."""
        assert apply_scaling(0.0, scale=2.5, offset=10.0) == pytest.approx(10.0)
