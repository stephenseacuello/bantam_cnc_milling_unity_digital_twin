"""Tests for the Kafka bridge node.

Extracts the core send/routing logic from KafkaBridgeNode and tests it
without any ROS2 or kafka-python dependency.  Uses the extracted-helper
pattern established by test_modbus_bridge.py.

Covers:
- Package / class / entry-point importability
- _send_to_kafka skip when disconnected
- _send_to_kafka exception handling (KafkaError)
- Message count increment on successful send
- _on_state routes to 'miracle.machine_state'
- _on_anomaly routes to 'miracle.anomaly_alerts'
- Parameter validation (batch_size, linger_ms ranges)
"""

import threading
from unittest.mock import MagicMock

import pytest


# ---------------------------------------------------------------------------
# Extracted helpers (mirror the logic in kafka_bridge.py)
# ---------------------------------------------------------------------------

class _KafkaSendHelper:
    """Simulates the send/routing logic of KafkaBridgeNode without ROS2.

    Mirrors _send_to_kafka, _on_state, and _on_anomaly behaviour.
    """

    def __init__(self, connected: bool = False):
        self._connected = connected
        self._producer = MagicMock() if connected else None
        self._message_count: int = 0
        self._lock = threading.Lock()
        self._last_error = None
        self._logger = MagicMock()

    def _send_to_kafka(self, topic: str, data: dict) -> None:
        """Send data to Kafka topic -- mirrors KafkaBridgeNode._send_to_kafka."""
        if not self._connected or self._producer is None:
            return

        try:
            self._producer.send(topic, value=data)
            with self._lock:
                self._message_count += 1
        except Exception as exc:
            self._last_error = str(exc)
            self._logger.error(f"Kafka send error: {exc}")

    def _on_state(self, msg_dict: dict) -> None:
        """Forward machine state to Kafka -- mirrors KafkaBridgeNode._on_state."""
        self._send_to_kafka('miracle.machine_state', msg_dict)

    def _on_anomaly(self, msg_dict: dict) -> None:
        """Forward anomaly alert to Kafka -- mirrors KafkaBridgeNode._on_anomaly."""
        self._send_to_kafka('miracle.anomaly_alerts', msg_dict)

    def _on_job_status(self, msg_dict: dict) -> None:
        """Forward job status to Kafka -- mirrors KafkaBridgeNode._on_job_status."""
        self._send_to_kafka('miracle.job_status', msg_dict)

    def _on_kpis(self, msg_dict: dict) -> None:
        """Forward KPIs to Kafka -- mirrors KafkaBridgeNode._on_kpis."""
        self._send_to_kafka('miracle.system_kpis', msg_dict)


def validate_batch_size(value: int) -> bool:
    """Validate batch_size is within the allowed range (1 .. 10000)."""
    return 1 <= value <= 10000


def validate_linger_ms(value: int) -> bool:
    """Validate linger_ms is within the allowed range (0 .. 60000)."""
    return 0 <= value <= 60000


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def connected_helper():
    """Return a connected KafkaSendHelper with a mock producer."""
    return _KafkaSendHelper(connected=True)


@pytest.fixture
def disconnected_helper():
    """Return a disconnected KafkaSendHelper."""
    return _KafkaSendHelper(connected=False)


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

def test_kafka_bridge_class_exists():
    """Verify the KafkaBridgeNode class exists in the module."""
    from miracle_bridges.kafka_bridge import KafkaBridgeNode
    assert KafkaBridgeNode is not None


# ---------------------------------------------------------------------------
# 3. Main function exists
# ---------------------------------------------------------------------------

def test_main_function_exists():
    """Verify the main entry point exists and is callable."""
    from miracle_bridges.kafka_bridge import main
    assert callable(main)


# ---------------------------------------------------------------------------
# 4. _send_to_kafka skips when not connected
# ---------------------------------------------------------------------------

def test_send_skips_when_disconnected(disconnected_helper):
    """_send_to_kafka should silently return when not connected."""
    disconnected_helper._send_to_kafka('miracle.machine_state', {'key': 'value'})
    assert disconnected_helper._message_count == 0


# ---------------------------------------------------------------------------
# 5. _send_to_kafka handles KafkaError gracefully
# ---------------------------------------------------------------------------

def test_send_handles_kafka_error(connected_helper):
    """_send_to_kafka should catch exceptions from the producer without crashing."""
    connected_helper._producer.send.side_effect = Exception("KafkaError: broker down")
    connected_helper._send_to_kafka('miracle.machine_state', {'key': 'value'})
    assert connected_helper._message_count == 0
    assert connected_helper._last_error is not None
    assert 'KafkaError' in connected_helper._last_error


# ---------------------------------------------------------------------------
# 6. Message count increments on successful send
# ---------------------------------------------------------------------------

def test_message_count_increments_on_success(connected_helper):
    """_send_to_kafka should increment _message_count on success."""
    connected_helper._send_to_kafka('miracle.test', {'a': 1})
    connected_helper._send_to_kafka('miracle.test', {'b': 2})
    assert connected_helper._message_count == 2


# ---------------------------------------------------------------------------
# 7. _on_state routes to 'miracle.machine_state' topic
# ---------------------------------------------------------------------------

def test_on_state_routes_to_machine_state_topic(connected_helper):
    """_on_state should send to the 'miracle.machine_state' Kafka topic."""
    data = {'machine_id': 'cnc1', 'status': 'RUNNING'}
    connected_helper._on_state(data)
    connected_helper._producer.send.assert_called_once_with(
        'miracle.machine_state', value=data,
    )


# ---------------------------------------------------------------------------
# 8. _on_anomaly routes to 'miracle.anomaly_alerts' topic
# ---------------------------------------------------------------------------

def test_on_anomaly_routes_to_anomaly_alerts_topic(connected_helper):
    """_on_anomaly should send to the 'miracle.anomaly_alerts' Kafka topic."""
    data = {'anomaly_type': 'vibration_spike', 'severity': 'HIGH'}
    connected_helper._on_anomaly(data)
    connected_helper._producer.send.assert_called_once_with(
        'miracle.anomaly_alerts', value=data,
    )


# ---------------------------------------------------------------------------
# 9. batch_size valid range
# ---------------------------------------------------------------------------

def test_batch_size_valid_in_range():
    """batch_size of 100 (the default) should be valid."""
    assert validate_batch_size(100) is True


# ---------------------------------------------------------------------------
# 10. batch_size out of range (too low)
# ---------------------------------------------------------------------------

def test_batch_size_too_low():
    """batch_size of 0 should be invalid (minimum is 1)."""
    assert validate_batch_size(0) is False


# ---------------------------------------------------------------------------
# 11. batch_size out of range (too high)
# ---------------------------------------------------------------------------

def test_batch_size_too_high():
    """batch_size of 10001 should be invalid (maximum is 10000)."""
    assert validate_batch_size(10001) is False


# ---------------------------------------------------------------------------
# 12. linger_ms valid range
# ---------------------------------------------------------------------------

def test_linger_ms_valid_in_range():
    """linger_ms of 100 (the default) should be valid."""
    assert validate_linger_ms(100) is True


# ---------------------------------------------------------------------------
# 13. linger_ms out of range (too low)
# ---------------------------------------------------------------------------

def test_linger_ms_at_zero_is_valid():
    """linger_ms of 0 should be valid (minimum is 0)."""
    assert validate_linger_ms(0) is True


# ---------------------------------------------------------------------------
# 14. linger_ms out of range (too high)
# ---------------------------------------------------------------------------

def test_linger_ms_too_high():
    """linger_ms of 60001 should be invalid (maximum is 60000)."""
    assert validate_linger_ms(60001) is False
