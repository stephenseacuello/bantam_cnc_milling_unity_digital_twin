"""Tests for Sparkplug B protocol logic.

Extracts and tests the core Sparkplug B concepts used by
SparkplugBridgeNode -- birth certificate generation, metric
encoding/decoding, topic namespace construction, and device
online/offline state transitions -- without any ROS2 or MQTT
dependency.
"""

import json
import time
import pytest


# ---------------------------------------------------------------------------
# Extracted helpers (mirror the logic in sparkplug_bridge.py)
# ---------------------------------------------------------------------------

class _SparkplugTopicBuilder:
    """Builds Sparkplug B v1.0 MQTT topic strings.

    Topic format: spBv1.0/<group_id>/<message_type>/<edge_node_id>[/<device_id>]
    """

    VERSION = 'spBv1.0'

    # Valid Sparkplug message types
    NBIRTH = 'NBIRTH'
    NDEATH = 'NDEATH'
    NDATA = 'NDATA'
    NCMD = 'NCMD'
    DBIRTH = 'DBIRTH'
    DDEATH = 'DDEATH'
    DDATA = 'DDATA'
    DCMD = 'DCMD'

    NODE_MESSAGE_TYPES = {NBIRTH, NDEATH, NDATA, NCMD}
    DEVICE_MESSAGE_TYPES = {DBIRTH, DDEATH, DDATA, DCMD}

    def __init__(self, group_id, edge_node_id):
        self.group_id = group_id
        self.edge_node_id = edge_node_id

    def node_topic(self, message_type):
        """Build a node-level topic (no device suffix)."""
        if message_type not in self.NODE_MESSAGE_TYPES:
            raise ValueError(
                f"'{message_type}' is not a valid node message type"
            )
        return f"{self.VERSION}/{self.group_id}/{message_type}/{self.edge_node_id}"

    def device_topic(self, message_type, device_id):
        """Build a device-level topic."""
        if message_type not in self.DEVICE_MESSAGE_TYPES:
            raise ValueError(
                f"'{message_type}' is not a valid device message type"
            )
        return (
            f"{self.VERSION}/{self.group_id}/{message_type}/"
            f"{self.edge_node_id}/{device_id}"
        )

    def subscribe_pattern(self, message_type):
        """Build a wildcard subscription topic matching all devices.

        Mirrors the pattern used in SparkplugBridgeNode._connect_mqtt:
            spBv1.0/{group_id}/DDATA/{edge_node}/#
        """
        return (
            f"{self.VERSION}/{self.group_id}/{message_type}/"
            f"{self.edge_node_id}/#"
        )


class _SparkplugMetric:
    """Represents a single Sparkplug B metric.

    In production Sparkplug uses protobuf; here we use a dict-based
    representation matching the JSON fallback in the bridge code.
    """

    # Sparkplug datatype codes (subset)
    TYPE_INT32 = 7
    TYPE_INT64 = 8
    TYPE_FLOAT = 9
    TYPE_DOUBLE = 10
    TYPE_STRING = 12
    TYPE_BOOLEAN = 11

    def __init__(self, name, datatype, value, timestamp=None):
        self.name = name
        self.datatype = datatype
        self.value = value
        self.timestamp = timestamp or int(time.time() * 1000)

    def to_dict(self):
        """Encode metric as a dictionary (JSON-serialisable)."""
        return {
            'name': self.name,
            'datatype': self.datatype,
            'value': self.value,
            'timestamp': self.timestamp,
        }

    @classmethod
    def from_dict(cls, d):
        """Decode metric from a dictionary."""
        return cls(
            name=d['name'],
            datatype=d['datatype'],
            value=d['value'],
            timestamp=d.get('timestamp'),
        )


class _SparkplugBirthCertificate:
    """Generates Sparkplug B birth certificate payloads.

    A birth certificate (NBIRTH / DBIRTH) contains all metrics a node or
    device will publish, establishing the metric namespace.
    """

    def __init__(self, node_name, seq=0):
        self.node_name = node_name
        self.seq = seq
        self.metrics = []

    def add_metric(self, name, datatype, initial_value):
        """Add a metric to the birth certificate."""
        self.metrics.append(_SparkplugMetric(name, datatype, initial_value))

    def to_payload(self):
        """Generate the birth certificate payload."""
        payload = {
            'timestamp': int(time.time() * 1000),
            'seq': self.seq,
            'metrics': [m.to_dict() for m in self.metrics],
        }
        return payload

    def metric_names(self):
        """Return the set of metric names in this certificate."""
        return {m.name for m in self.metrics}


class _DeviceStateTracker:
    """Tracks online/offline state transitions for Sparkplug devices.

    Sparkplug lifecycle:
      OFFLINE -> NBIRTH/DBIRTH -> ONLINE -> NDEATH/DDEATH -> OFFLINE
    """

    OFFLINE = 'OFFLINE'
    ONLINE = 'ONLINE'

    def __init__(self):
        self._devices = {}
        self._transitions = []

    def device_born(self, device_id):
        """Handle a birth message (device goes ONLINE)."""
        previous = self._devices.get(device_id, self.OFFLINE)
        self._devices[device_id] = self.ONLINE
        self._transitions.append((device_id, previous, self.ONLINE))

    def device_died(self, device_id):
        """Handle a death message (device goes OFFLINE)."""
        previous = self._devices.get(device_id, self.OFFLINE)
        self._devices[device_id] = self.OFFLINE
        self._transitions.append((device_id, previous, self.OFFLINE))

    def is_online(self, device_id):
        """Check whether a device is currently online."""
        return self._devices.get(device_id) == self.ONLINE

    def online_devices(self):
        """Return the set of currently online devices."""
        return {d for d, s in self._devices.items() if s == self.ONLINE}

    def transition_count(self, device_id=None):
        """Count state transitions, optionally filtered by device."""
        if device_id is None:
            return len(self._transitions)
        return sum(1 for d, _, _ in self._transitions if d == device_id)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def topic_builder():
    """Return a topic builder with default MIRACLE settings."""
    return _SparkplugTopicBuilder(group_id='MIRACLE', edge_node_id='CNCCell1')


@pytest.fixture
def state_tracker():
    """Return a fresh device state tracker."""
    return _DeviceStateTracker()


@pytest.fixture
def cnc_birth():
    """Return a birth certificate populated with typical CNC metrics."""
    cert = _SparkplugBirthCertificate('CNCCell1')
    cert.add_metric('spindle_speed', _SparkplugMetric.TYPE_FLOAT, 0.0)
    cert.add_metric('feed_rate', _SparkplugMetric.TYPE_FLOAT, 0.0)
    cert.add_metric('spindle_load', _SparkplugMetric.TYPE_FLOAT, 0.0)
    cert.add_metric('coolant_temp', _SparkplugMetric.TYPE_FLOAT, 25.0)
    cert.add_metric('axis_x', _SparkplugMetric.TYPE_DOUBLE, 0.0)
    cert.add_metric('axis_y', _SparkplugMetric.TYPE_DOUBLE, 0.0)
    cert.add_metric('axis_z', _SparkplugMetric.TYPE_DOUBLE, 0.0)
    cert.add_metric('machine_status', _SparkplugMetric.TYPE_STRING, 'IDLE')
    cert.add_metric('emergency_stop', _SparkplugMetric.TYPE_BOOLEAN, False)
    return cert


# ---------------------------------------------------------------------------
# Test classes
# ---------------------------------------------------------------------------

class TestBirthCertificateGeneration:
    """Test Sparkplug B birth certificate payloads."""

    def test_payload_contains_all_metrics(self, cnc_birth):
        """Birth payload should contain every added metric."""
        payload = cnc_birth.to_payload()
        names = {m['name'] for m in payload['metrics']}
        assert 'spindle_speed' in names
        assert 'axis_x' in names
        assert 'machine_status' in names
        assert len(payload['metrics']) == 9

    def test_payload_has_timestamp(self, cnc_birth):
        """Birth payload must carry a millisecond timestamp."""
        payload = cnc_birth.to_payload()
        assert 'timestamp' in payload
        assert isinstance(payload['timestamp'], int)
        assert payload['timestamp'] > 0

    def test_payload_has_sequence_number(self, cnc_birth):
        """Birth payload must include a sequence counter."""
        payload = cnc_birth.to_payload()
        assert 'seq' in payload
        assert payload['seq'] == 0

    def test_metric_names_helper(self, cnc_birth):
        """metric_names() should return the full set of names."""
        names = cnc_birth.metric_names()
        assert names == {
            'spindle_speed', 'feed_rate', 'spindle_load', 'coolant_temp',
            'axis_x', 'axis_y', 'axis_z', 'machine_status', 'emergency_stop',
        }

    def test_empty_birth_certificate(self):
        """An empty birth certificate should have zero metrics."""
        cert = _SparkplugBirthCertificate('empty_node')
        payload = cert.to_payload()
        assert len(payload['metrics']) == 0

    def test_payload_is_json_serialisable(self, cnc_birth):
        """The payload dict must be JSON-serialisable (no exotic types)."""
        payload = cnc_birth.to_payload()
        serialised = json.dumps(payload)
        assert isinstance(serialised, str)
        roundtrip = json.loads(serialised)
        assert len(roundtrip['metrics']) == 9


class TestMetricEncodingDecoding:
    """Test Sparkplug metric encode/decode round-trips."""

    @pytest.mark.parametrize("name,dtype,value", [
        ('speed', _SparkplugMetric.TYPE_FLOAT, 12000.0),
        ('count', _SparkplugMetric.TYPE_INT32, 42),
        ('label', _SparkplugMetric.TYPE_STRING, 'RUNNING'),
        ('estop', _SparkplugMetric.TYPE_BOOLEAN, True),
        ('big_val', _SparkplugMetric.TYPE_INT64, 2**40),
        ('precise', _SparkplugMetric.TYPE_DOUBLE, 3.141592653589793),
    ])
    def test_round_trip(self, name, dtype, value):
        """Encoding then decoding a metric should preserve all fields."""
        original = _SparkplugMetric(name, dtype, value)
        d = original.to_dict()
        restored = _SparkplugMetric.from_dict(d)
        assert restored.name == name
        assert restored.datatype == dtype
        assert restored.value == value

    def test_metric_timestamp_present(self):
        """Each metric should carry a timestamp."""
        m = _SparkplugMetric('test', _SparkplugMetric.TYPE_FLOAT, 1.0)
        d = m.to_dict()
        assert 'timestamp' in d
        assert d['timestamp'] > 0

    def test_metric_from_dict_with_missing_timestamp(self):
        """from_dict should handle missing timestamp by generating one."""
        d = {'name': 'x', 'datatype': _SparkplugMetric.TYPE_FLOAT, 'value': 0.0}
        m = _SparkplugMetric.from_dict(d)
        assert m.name == 'x'
        # When timestamp is not in the dict, from_dict passes None to __init__,
        # which defaults to int(time.time() * 1000) -- a valid epoch-ms value.
        assert isinstance(m.timestamp, int)
        assert m.timestamp > 0


class TestTopicNamespaceConstruction:
    """Test Sparkplug B MQTT topic construction."""

    def test_nbirth_topic(self, topic_builder):
        """NBIRTH topic should follow the spBv1.0 format."""
        topic = topic_builder.node_topic('NBIRTH')
        assert topic == 'spBv1.0/MIRACLE/NBIRTH/CNCCell1'

    def test_ndeath_topic(self, topic_builder):
        """NDEATH topic should follow the spBv1.0 format."""
        topic = topic_builder.node_topic('NDEATH')
        assert topic == 'spBv1.0/MIRACLE/NDEATH/CNCCell1'

    def test_ndata_topic(self, topic_builder):
        """NDATA topic for node-level telemetry."""
        topic = topic_builder.node_topic('NDATA')
        assert topic == 'spBv1.0/MIRACLE/NDATA/CNCCell1'

    def test_ddata_topic(self, topic_builder):
        """DDATA topic should include the device ID suffix."""
        topic = topic_builder.device_topic('DDATA', 'spindle1')
        assert topic == 'spBv1.0/MIRACLE/DDATA/CNCCell1/spindle1'

    def test_dbirth_topic(self, topic_builder):
        """DBIRTH topic should include the device ID suffix."""
        topic = topic_builder.device_topic('DBIRTH', 'coolant_pump')
        assert topic == 'spBv1.0/MIRACLE/DBIRTH/CNCCell1/coolant_pump'

    def test_subscribe_pattern_matches_source(self, topic_builder):
        """Subscribe pattern should match what SparkplugBridgeNode uses."""
        pattern = topic_builder.subscribe_pattern('DDATA')
        assert pattern == 'spBv1.0/MIRACLE/DDATA/CNCCell1/#'

    def test_invalid_node_message_type_raises(self, topic_builder):
        """Using a device message type for node_topic should raise."""
        with pytest.raises(ValueError):
            topic_builder.node_topic('DDATA')

    def test_invalid_device_message_type_raises(self, topic_builder):
        """Using a node message type for device_topic should raise."""
        with pytest.raises(ValueError):
            topic_builder.device_topic('NBIRTH', 'dev1')

    def test_custom_group_and_node(self):
        """Topic builder should work with arbitrary group/node names."""
        builder = _SparkplugTopicBuilder('FactoryFloor', 'Cell42')
        topic = builder.device_topic('DDATA', 'robot_arm')
        assert topic == 'spBv1.0/FactoryFloor/DDATA/Cell42/robot_arm'


class TestDeviceStateTransitions:
    """Test device online/offline state transitions."""

    def test_initial_state_is_offline(self, state_tracker):
        """A device not yet seen should be considered offline."""
        assert state_tracker.is_online('cnc1') is False

    def test_birth_brings_device_online(self, state_tracker):
        """A birth message should set the device to ONLINE."""
        state_tracker.device_born('cnc1')
        assert state_tracker.is_online('cnc1') is True

    def test_death_takes_device_offline(self, state_tracker):
        """A death message should set the device to OFFLINE."""
        state_tracker.device_born('cnc1')
        state_tracker.device_died('cnc1')
        assert state_tracker.is_online('cnc1') is False

    def test_multiple_devices_independent(self, state_tracker):
        """Each device tracks state independently."""
        state_tracker.device_born('cnc1')
        state_tracker.device_born('cnc2')
        state_tracker.device_died('cnc1')
        assert state_tracker.is_online('cnc1') is False
        assert state_tracker.is_online('cnc2') is True

    def test_online_devices_set(self, state_tracker):
        """online_devices() should return only currently online devices."""
        state_tracker.device_born('a')
        state_tracker.device_born('b')
        state_tracker.device_born('c')
        state_tracker.device_died('b')
        assert state_tracker.online_devices() == {'a', 'c'}

    def test_rebirth_after_death(self, state_tracker):
        """A device should be able to go OFFLINE -> ONLINE again."""
        state_tracker.device_born('cnc1')
        state_tracker.device_died('cnc1')
        state_tracker.device_born('cnc1')
        assert state_tracker.is_online('cnc1') is True

    def test_transition_count_tracks_all_changes(self, state_tracker):
        """Total transition count should reflect all state changes."""
        state_tracker.device_born('cnc1')
        state_tracker.device_died('cnc1')
        state_tracker.device_born('cnc1')
        assert state_tracker.transition_count() == 3

    def test_transition_count_per_device(self, state_tracker):
        """Per-device transition count should filter correctly."""
        state_tracker.device_born('cnc1')
        state_tracker.device_born('cnc2')
        state_tracker.device_died('cnc1')
        assert state_tracker.transition_count('cnc1') == 2
        assert state_tracker.transition_count('cnc2') == 1

    def test_death_when_already_offline(self, state_tracker):
        """A death for an already-offline device should not raise."""
        state_tracker.device_died('unknown')
        assert state_tracker.is_online('unknown') is False
        assert state_tracker.transition_count('unknown') == 1
