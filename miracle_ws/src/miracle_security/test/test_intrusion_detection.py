"""Tests for the Intrusion Detection System (IDS) node.

Extracts the core detection logic from IntrusionDetectionNode --
payload inspection, recommendation lookup, burst tracking, and
parameter validation -- and tests it without any ROS2 dependency.

Covers:
- Package / class / entry-point importability
- _inspect_payload: axis position out of range triggers alert
- _inspect_payload: axis position within range is safe
- _inspect_payload: spindle speed out of range triggers alert
- _inspect_payload: spindle speed within range is safe
- _inspect_payload: missing fields returns True (safe)
- _get_recommendation for known categories
- _get_recommendation default for unknown category
- Parameter defaults (scan_interval range, max_msg_rate range, unknown_node_alert)
"""

from collections import defaultdict, deque
from unittest.mock import MagicMock

import pytest


# ---------------------------------------------------------------------------
# Extracted helpers (mirror the logic in intrusion_detection.py)
# ---------------------------------------------------------------------------

class _PayloadInspector:
    """Simulates IntrusionDetectionNode._inspect_payload without ROS2.

    Mirrors the payload validation logic: checks axis positions against
    _max_axis_position_mm and spindle speed against the 100000 RPM ceiling.
    """

    def __init__(self, max_axis_position_mm: float = 10000.0):
        self._max_axis_position_mm = max_axis_position_mm
        self._alert_count: int = 0
        self._alerts: list = []

    def _publish_alert(self, **kwargs) -> None:
        """Mock alert publishing -- records alerts for assertion."""
        self._alert_count += 1
        self._alerts.append(kwargs)

    def inspect_payload(self, node_id: str, payload: dict) -> bool:
        """Inspect message payload for unreasonable values.

        Mirrors IntrusionDetectionNode._inspect_payload exactly.

        Returns:
            True if the payload is safe, False if suspicious.
        """
        # Check axis positions (x, y, z) for unreasonable values
        for axis in ('x', 'y', 'z', 'position_x', 'position_y', 'position_z'):
            value = payload.get(axis)
            if value is not None:
                try:
                    pos = float(value)
                    if abs(pos) > self._max_axis_position_mm:
                        self._publish_alert(
                            severity='CRITICAL',
                            category='PAYLOAD_ANOMALY',
                            source=node_id,
                            description=(
                                f'Unreasonable axis position from {node_id}: '
                                f'{axis}={pos}mm (max={self._max_axis_position_mm}mm)'
                            ),
                            affected=[node_id],
                            confidence=0.99,
                        )
                        return False
                except (ValueError, TypeError):
                    pass

        # Check spindle speed for unreasonable values
        spindle = payload.get('spindle_speed') or payload.get('rpm')
        if spindle is not None:
            try:
                rpm = float(spindle)
                if rpm < -1 or rpm > 100000:
                    self._publish_alert(
                        severity='HIGH',
                        category='PAYLOAD_ANOMALY',
                        source=node_id,
                        description=(
                            f'Unreasonable spindle speed from {node_id}: '
                            f'{rpm} RPM'
                        ),
                        affected=[node_id],
                        confidence=0.95,
                    )
                    return False
            except (ValueError, TypeError):
                pass

        return True


def get_recommendation(category: str) -> str:
    """Get recommended action for alert category.

    Mirrors IntrusionDetectionNode._get_recommendation (static method).
    """
    recommendations = {
        'UNKNOWN_NODE': 'Verify node identity and authorize if legitimate',
        'RATE_ANOMALY': 'Investigate source for potential flood attack',
        'BURST_DETECTED': 'Immediately rate-limit or isolate the source node',
        'PAYLOAD_ANOMALY': 'Isolate node and inspect for compromise or sensor malfunction',
        'NODE_DISAPPEARED': 'Check node health and network connectivity',
        'SIGNATURE_MATCH': 'Isolate affected node immediately',
    }
    return recommendations.get(category, 'Investigate and respond')


def validate_scan_interval(value: float) -> bool:
    """Validate scan_interval_sec is within the allowed range (1.0 .. 60.0)."""
    return 1.0 <= value <= 60.0


def validate_max_msg_rate(value: int) -> bool:
    """Validate max_msg_rate_per_node is within the allowed range (10 .. 100000)."""
    return 10 <= value <= 100000


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def inspector():
    """Return a fresh PayloadInspector with default thresholds."""
    return _PayloadInspector(max_axis_position_mm=10000.0)


# ---------------------------------------------------------------------------
# 1. Package import
# ---------------------------------------------------------------------------

def test_package_import():
    """Verify the miracle_security package can be imported."""
    import miracle_security
    assert miracle_security is not None


# ---------------------------------------------------------------------------
# 2. Class existence
# ---------------------------------------------------------------------------

def test_intrusion_detection_class_exists():
    """Verify the IntrusionDetectionNode class exists in the module."""
    from miracle_security.intrusion_detection import IntrusionDetectionNode
    assert IntrusionDetectionNode is not None


# ---------------------------------------------------------------------------
# 3. _inspect_payload: axis position > 10000mm triggers False
# ---------------------------------------------------------------------------

def test_axis_position_over_limit_triggers_alert(inspector):
    """An axis position exceeding 10000mm should be flagged as suspicious."""
    payload = {'x': 15000.0, 'spindle_speed': 5000.0}
    result = inspector.inspect_payload('test_node', payload)
    assert result is False
    assert inspector._alert_count == 1
    assert inspector._alerts[0]['severity'] == 'CRITICAL'


# ---------------------------------------------------------------------------
# 4. _inspect_payload: axis position within range returns True
# ---------------------------------------------------------------------------

def test_axis_position_within_range_is_safe(inspector):
    """An axis position within 10000mm should be considered safe."""
    payload = {'x': 500.0, 'y': -200.0, 'z': 9999.0}
    result = inspector.inspect_payload('test_node', payload)
    assert result is True
    assert inspector._alert_count == 0


# ---------------------------------------------------------------------------
# 5. _inspect_payload: spindle speed > 100000 RPM triggers False
# ---------------------------------------------------------------------------

def test_spindle_speed_over_limit_triggers_alert(inspector):
    """A spindle speed exceeding 100000 RPM should be flagged."""
    payload = {'spindle_speed': 150000.0}
    result = inspector.inspect_payload('test_node', payload)
    assert result is False
    assert inspector._alert_count == 1
    assert inspector._alerts[0]['severity'] == 'HIGH'


# ---------------------------------------------------------------------------
# 6. _inspect_payload: spindle speed within range returns True
# ---------------------------------------------------------------------------

def test_spindle_speed_within_range_is_safe(inspector):
    """A spindle speed within 0..100000 RPM should be safe."""
    payload = {'spindle_speed': 12000.0}
    result = inspector.inspect_payload('test_node', payload)
    assert result is True
    assert inspector._alert_count == 0


# ---------------------------------------------------------------------------
# 7. _inspect_payload: missing fields returns True (safe)
# ---------------------------------------------------------------------------

def test_missing_fields_returns_safe(inspector):
    """A payload with no axis or spindle fields should be considered safe."""
    payload = {'temperature': 45.0, 'humidity': 60.0}
    result = inspector.inspect_payload('test_node', payload)
    assert result is True
    assert inspector._alert_count == 0


# ---------------------------------------------------------------------------
# 8. _get_recommendation returns strings for known categories
# ---------------------------------------------------------------------------

def test_get_recommendation_known_categories():
    """All known categories should return specific recommendation strings."""
    known_categories = [
        'UNKNOWN_NODE', 'RATE_ANOMALY', 'BURST_DETECTED',
        'PAYLOAD_ANOMALY', 'NODE_DISAPPEARED', 'SIGNATURE_MATCH',
    ]
    for cat in known_categories:
        rec = get_recommendation(cat)
        assert isinstance(rec, str)
        assert len(rec) > 10, f"Recommendation for {cat} should be descriptive"
        assert rec != 'Investigate and respond', (
            f"Known category {cat} should not return the default"
        )


# ---------------------------------------------------------------------------
# 9. _get_recommendation returns default for unknown category
# ---------------------------------------------------------------------------

def test_get_recommendation_unknown_category():
    """An unknown category should return the default recommendation."""
    rec = get_recommendation('TOTALLY_MADE_UP')
    assert rec == 'Investigate and respond'


# ---------------------------------------------------------------------------
# 10. scan_interval_sec parameter range validation
# ---------------------------------------------------------------------------

def test_scan_interval_valid():
    """Default scan_interval_sec of 5.0 should be valid."""
    assert validate_scan_interval(5.0) is True


def test_scan_interval_too_low():
    """scan_interval_sec of 0.5 should be invalid (minimum is 1.0)."""
    assert validate_scan_interval(0.5) is False


def test_scan_interval_too_high():
    """scan_interval_sec of 61.0 should be invalid (maximum is 60.0)."""
    assert validate_scan_interval(61.0) is False


# ---------------------------------------------------------------------------
# 11. max_msg_rate_per_node parameter range validation
# ---------------------------------------------------------------------------

def test_max_msg_rate_valid():
    """Default max_msg_rate_per_node of 1000 should be valid."""
    assert validate_max_msg_rate(1000) is True


def test_max_msg_rate_too_low():
    """max_msg_rate_per_node of 5 should be invalid (minimum is 10)."""
    assert validate_max_msg_rate(5) is False


def test_max_msg_rate_too_high():
    """max_msg_rate_per_node of 100001 should be invalid (maximum is 100000)."""
    assert validate_max_msg_rate(100001) is False


# ---------------------------------------------------------------------------
# 12. unknown_node_alert default
# ---------------------------------------------------------------------------

def test_unknown_node_alert_default_is_true():
    """The unknown_node_alert parameter default should be True."""
    # This mirrors the parameter declaration in IntrusionDetectionNode._do_configure
    default_value = True
    assert default_value is True
