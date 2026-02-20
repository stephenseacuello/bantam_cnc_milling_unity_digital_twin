"""Tests for device discovery and registration logic.

Extracts the core device registry logic from DiscoveryServerNode --
registration with trust scores, duplicate handling, deregistration/timeout,
and fleet status aggregation -- and tests it without any ROS2 dependency.
"""

import time
import pytest
from typing import Any, Dict, List, Optional, Set
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Extracted helpers (mirror the logic in discovery_server.py)
# ---------------------------------------------------------------------------

@dataclass
class _DeviceRecord:
    """A registered device in the discovery registry."""
    device_id: str
    device_type: str
    capabilities: List[str]
    firmware_version: str
    trust_score: float
    registered_at: float = 0.0
    last_seen: float = 0.0


class _DeviceRegistry:
    """Extracted device registration logic from DiscoveryServerNode.

    Mirrors the _handle_register method and device management.
    """

    def __init__(self, max_devices=100, initial_trust=0.5):
        self.max_devices = max_devices
        self.initial_trust = initial_trust
        self._devices: Dict[str, _DeviceRecord] = {}

    def register(self, device_id, device_type, capabilities=None,
                 firmware_version='', current_time=0.0):
        """Register a new device or update an existing one.

        Returns:
            (success: bool, message: str, trust_score: float)
        """
        capabilities = capabilities or []

        if len(self._devices) >= self.max_devices and device_id not in self._devices:
            return False, 'Device registry full', 0.0

        if device_id in self._devices:
            existing = self._devices[device_id]
            existing.last_seen = current_time
            # Update mutable fields
            existing.capabilities = capabilities
            existing.firmware_version = firmware_version
            return True, 'Device already registered (updated)', existing.trust_score

        record = _DeviceRecord(
            device_id=device_id,
            device_type=device_type,
            capabilities=capabilities,
            firmware_version=firmware_version,
            trust_score=self.initial_trust,
            registered_at=current_time,
            last_seen=current_time,
        )
        self._devices[device_id] = record
        return True, f'Device {device_id} registered', self.initial_trust

    def deregister(self, device_id):
        """Remove a device from the registry.

        Returns:
            True if the device was found and removed.
        """
        if device_id in self._devices:
            del self._devices[device_id]
            return True
        return False

    def get_device(self, device_id):
        """Retrieve a device record, or None if not found."""
        return self._devices.get(device_id)

    def update_trust(self, device_id, new_trust):
        """Update a device's trust score.

        Returns:
            True if the device exists and was updated.
        """
        if device_id in self._devices:
            self._devices[device_id].trust_score = max(0.0, min(1.0, new_trust))
            return True
        return False

    def heartbeat(self, device_id, current_time):
        """Record a heartbeat (update last_seen timestamp).

        Returns:
            True if the device exists.
        """
        if device_id in self._devices:
            self._devices[device_id].last_seen = current_time
            return True
        return False

    def expire_stale(self, current_time, timeout_sec):
        """Remove devices that have not been seen within timeout.

        Returns:
            List of expired device IDs.
        """
        expired = []
        for dev_id, record in list(self._devices.items()):
            if current_time - record.last_seen > timeout_sec:
                del self._devices[dev_id]
                expired.append(dev_id)
        return expired

    def device_count(self):
        """Return the number of registered devices."""
        return len(self._devices)

    def all_device_ids(self):
        """Return the set of all registered device IDs."""
        return set(self._devices.keys())

    def devices_by_type(self, device_type):
        """Return device IDs filtered by type."""
        return [
            d.device_id for d in self._devices.values()
            if d.device_type == device_type
        ]


class _FleetStatusAggregator:
    """Aggregates fleet-level statistics from a device registry."""

    def __init__(self, registry: _DeviceRegistry):
        self._registry = registry

    def total_devices(self):
        """Total number of registered devices."""
        return self._registry.device_count()

    def average_trust(self):
        """Average trust score across all devices."""
        devices = list(self._registry._devices.values())
        if not devices:
            return 0.0
        return sum(d.trust_score for d in devices) / len(devices)

    def devices_above_trust(self, threshold):
        """Count devices with trust score above threshold."""
        return sum(
            1 for d in self._registry._devices.values()
            if d.trust_score >= threshold
        )

    def devices_below_trust(self, threshold):
        """Count devices with trust score below threshold."""
        return sum(
            1 for d in self._registry._devices.values()
            if d.trust_score < threshold
        )

    def type_distribution(self):
        """Return a dict mapping device_type -> count."""
        dist = {}
        for d in self._registry._devices.values():
            dist[d.device_type] = dist.get(d.device_type, 0) + 1
        return dist

    def capability_summary(self):
        """Return a set of all unique capabilities across the fleet."""
        caps = set()
        for d in self._registry._devices.values():
            caps.update(d.capabilities)
        return caps


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def registry():
    """Return a fresh device registry."""
    return _DeviceRegistry(max_devices=10, initial_trust=0.5)


@pytest.fixture
def populated_registry():
    """Return a registry pre-populated with several devices."""
    reg = _DeviceRegistry(max_devices=100, initial_trust=0.5)
    reg.register('cnc1', 'CNC', ['milling', '5axis'], '1.0.0', current_time=0.0)
    reg.register('cnc2', 'CNC', ['milling'], '1.0.1', current_time=0.0)
    reg.register('robot1', 'COBOT', ['pick_place'], '2.1.0', current_time=0.0)
    reg.register('sensor1', 'SENSOR', ['temperature', 'vibration'], '0.5.0', current_time=0.0)
    return reg


@pytest.fixture
def aggregator(populated_registry):
    """Return a fleet status aggregator backed by the populated registry."""
    return _FleetStatusAggregator(populated_registry)


# ---------------------------------------------------------------------------
# Test classes
# ---------------------------------------------------------------------------

class TestDeviceRegistration:
    """Test device registration with trust scores."""

    def test_register_new_device(self, registry):
        """Registering a new device should succeed with initial trust."""
        success, msg, trust = registry.register('cnc1', 'CNC')
        assert success is True
        assert 'registered' in msg.lower()
        assert trust == 0.5

    def test_initial_trust_score(self, registry):
        """Registered device should have the configured initial trust."""
        registry.register('cnc1', 'CNC')
        device = registry.get_device('cnc1')
        assert device.trust_score == 0.5

    def test_custom_initial_trust(self):
        """Registry with custom initial trust should apply it."""
        reg = _DeviceRegistry(initial_trust=0.3)
        success, _, trust = reg.register('dev1', 'SENSOR')
        assert trust == pytest.approx(0.3)

    def test_registration_stores_metadata(self, registry):
        """All registration metadata should be stored correctly."""
        registry.register('cnc1', 'CNC', ['milling', '5axis'], 'v2.0', current_time=100.0)
        device = registry.get_device('cnc1')
        assert device.device_type == 'CNC'
        assert device.capabilities == ['milling', '5axis']
        assert device.firmware_version == 'v2.0'
        assert device.registered_at == 100.0

    def test_device_count_increments(self, registry):
        """Device count should reflect registrations."""
        assert registry.device_count() == 0
        registry.register('a', 'T')
        assert registry.device_count() == 1
        registry.register('b', 'T')
        assert registry.device_count() == 2

    def test_get_nonexistent_device(self, registry):
        """Getting a non-existent device should return None."""
        assert registry.get_device('missing') is None


class TestDuplicateRegistration:
    """Test duplicate device registration handling."""

    def test_duplicate_returns_success(self, registry):
        """Re-registering the same device_id should succeed."""
        registry.register('cnc1', 'CNC', current_time=0.0)
        success, msg, _ = registry.register('cnc1', 'CNC', current_time=10.0)
        assert success is True
        assert 'already registered' in msg.lower()

    def test_duplicate_preserves_trust(self, registry):
        """Re-registering should keep the existing trust score."""
        registry.register('cnc1', 'CNC')
        registry.update_trust('cnc1', 0.9)
        _, _, trust = registry.register('cnc1', 'CNC')
        assert trust == pytest.approx(0.9)

    def test_duplicate_updates_last_seen(self, registry):
        """Re-registering should update last_seen."""
        registry.register('cnc1', 'CNC', current_time=0.0)
        registry.register('cnc1', 'CNC', current_time=50.0)
        assert registry.get_device('cnc1').last_seen == 50.0

    def test_duplicate_does_not_increase_count(self, registry):
        """Re-registering should not increase device count."""
        registry.register('cnc1', 'CNC')
        registry.register('cnc1', 'CNC')
        assert registry.device_count() == 1

    def test_duplicate_updates_capabilities(self, registry):
        """Re-registering should update capabilities."""
        registry.register('cnc1', 'CNC', ['milling'])
        registry.register('cnc1', 'CNC', ['milling', '5axis'])
        assert registry.get_device('cnc1').capabilities == ['milling', '5axis']


class TestRegistryCapacityLimit:
    """Test registry capacity enforcement."""

    def test_registry_full_rejects_new(self, registry):
        """When full, new registrations should be rejected."""
        for i in range(10):
            registry.register(f'dev{i}', 'T')
        success, msg, trust = registry.register('dev_extra', 'T')
        assert success is False
        assert 'full' in msg.lower()
        assert trust == 0.0

    def test_registry_full_allows_duplicate(self, registry):
        """When full, re-registering an existing device should succeed."""
        for i in range(10):
            registry.register(f'dev{i}', 'T')
        success, _, _ = registry.register('dev0', 'T')
        assert success is True


class TestDeregistrationAndTimeout:
    """Test device deregistration and timeout expiry."""

    def test_deregister_existing(self, registry):
        """Deregistering an existing device should succeed."""
        registry.register('cnc1', 'CNC')
        result = registry.deregister('cnc1')
        assert result is True
        assert registry.device_count() == 0

    def test_deregister_nonexistent(self, registry):
        """Deregistering a non-existent device should return False."""
        assert registry.deregister('missing') is False

    def test_heartbeat_updates_last_seen(self, registry):
        """Heartbeat should update the last_seen timestamp."""
        registry.register('cnc1', 'CNC', current_time=0.0)
        registry.heartbeat('cnc1', current_time=100.0)
        assert registry.get_device('cnc1').last_seen == 100.0

    def test_heartbeat_nonexistent_returns_false(self, registry):
        """Heartbeat for non-existent device should return False."""
        assert registry.heartbeat('missing', 0.0) is False

    def test_expire_stale_devices(self, registry):
        """Devices not seen within timeout should be expired."""
        registry.register('cnc1', 'CNC', current_time=0.0)
        registry.register('cnc2', 'CNC', current_time=50.0)
        expired = registry.expire_stale(current_time=100.0, timeout_sec=60.0)
        assert 'cnc1' in expired
        assert 'cnc2' not in expired
        assert registry.device_count() == 1

    def test_expire_no_stale_devices(self, registry):
        """No devices should be expired if all are recent."""
        registry.register('cnc1', 'CNC', current_time=90.0)
        expired = registry.expire_stale(current_time=100.0, timeout_sec=60.0)
        assert expired == []
        assert registry.device_count() == 1

    def test_expire_all_devices(self, registry):
        """All devices should be expired if all are old enough."""
        registry.register('a', 'T', current_time=0.0)
        registry.register('b', 'T', current_time=0.0)
        expired = registry.expire_stale(current_time=1000.0, timeout_sec=60.0)
        assert set(expired) == {'a', 'b'}
        assert registry.device_count() == 0


class TestTrustScoreManagement:
    """Test trust score updates and bounds."""

    def test_update_trust(self, registry):
        """Trust score should be updated to the given value."""
        registry.register('cnc1', 'CNC')
        registry.update_trust('cnc1', 0.9)
        assert registry.get_device('cnc1').trust_score == pytest.approx(0.9)

    def test_trust_clamped_to_one(self, registry):
        """Trust score should not exceed 1.0."""
        registry.register('cnc1', 'CNC')
        registry.update_trust('cnc1', 1.5)
        assert registry.get_device('cnc1').trust_score == pytest.approx(1.0)

    def test_trust_clamped_to_zero(self, registry):
        """Trust score should not go below 0.0."""
        registry.register('cnc1', 'CNC')
        registry.update_trust('cnc1', -0.5)
        assert registry.get_device('cnc1').trust_score == pytest.approx(0.0)

    def test_update_trust_nonexistent(self, registry):
        """Updating trust for non-existent device should return False."""
        assert registry.update_trust('missing', 0.5) is False


class TestFleetStatusAggregation:
    """Test fleet-level status aggregation."""

    def test_total_devices(self, aggregator):
        """Total device count should match populated registry."""
        assert aggregator.total_devices() == 4

    def test_average_trust_all_default(self, aggregator):
        """Average trust should be initial_trust when none are updated."""
        assert aggregator.average_trust() == pytest.approx(0.5)

    def test_average_trust_mixed(self, populated_registry):
        """Average trust should reflect individual updates."""
        populated_registry.update_trust('cnc1', 1.0)
        populated_registry.update_trust('cnc2', 0.0)
        # cnc1=1.0, cnc2=0.0, robot1=0.5, sensor1=0.5 => avg=0.5
        agg = _FleetStatusAggregator(populated_registry)
        assert agg.average_trust() == pytest.approx(0.5)

    def test_average_trust_empty_registry(self):
        """Average trust of empty registry should be 0.0."""
        reg = _DeviceRegistry()
        agg = _FleetStatusAggregator(reg)
        assert agg.average_trust() == pytest.approx(0.0)

    def test_devices_above_trust(self, aggregator, populated_registry):
        """Should count devices with trust >= threshold."""
        populated_registry.update_trust('cnc1', 0.9)
        assert aggregator.devices_above_trust(0.8) == 1

    def test_type_distribution(self, aggregator):
        """Type distribution should count each device type."""
        dist = aggregator.type_distribution()
        assert dist['CNC'] == 2
        assert dist['COBOT'] == 1
        assert dist['SENSOR'] == 1

    def test_capability_summary(self, aggregator):
        """Capability summary should include all unique capabilities."""
        caps = aggregator.capability_summary()
        assert 'milling' in caps
        assert '5axis' in caps
        assert 'pick_place' in caps
        assert 'temperature' in caps
        assert 'vibration' in caps

    def test_devices_by_type(self, populated_registry):
        """devices_by_type should filter correctly."""
        cncs = populated_registry.devices_by_type('CNC')
        assert set(cncs) == {'cnc1', 'cnc2'}
        cobots = populated_registry.devices_by_type('COBOT')
        assert cobots == ['robot1']
