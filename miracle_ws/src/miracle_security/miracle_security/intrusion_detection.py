"""
Intrusion Detection System (IDS) Node.

Monitors ROS2 network traffic patterns for anomalous behavior
indicative of cyber attacks. Uses behavioral analysis and
signature-based detection.
"""

from typing import Any, Dict, List, Optional, Tuple
from collections import defaultdict, deque
import threading
import hashlib
import time

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import SecurityAlert, Heartbeat


class IntrusionDetectionNode(MiracleLifecycleNode):
    """Network intrusion detection for ROS2 systems.

    Parameters:
        scan_interval_sec (float): IDS scan interval.
        max_msg_rate_per_node (int): Max messages/sec before alert.
        unknown_node_alert (bool): Alert on unknown nodes.
        signature_file (str): Path to attack signature database.

    Subscribed Topics:
        /miracle/heartbeats (Heartbeat): Monitor node behavior.

    Published Topics:
        /miracle/security/alerts (SecurityAlert): Security alerts.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'intrusion_detection',
            criticality=self.CRITICALITY_CRITICAL,
            **kwargs,
        )
        self._heartbeat_sub = None
        self._alert_pub = None
        self._scan_timer = None

        self._known_nodes: set = set()
        self._msg_counts: Dict[str, int] = defaultdict(int)
        self._msg_history: Dict[str, deque] = defaultdict(lambda: deque(maxlen=100))
        self._burst_history: Dict[str, deque] = defaultdict(lambda: deque(maxlen=1000))
        self._lock = threading.Lock()
        self._max_rate: int = 1000
        self._alert_unknown: bool = True
        self._alert_count: int = 0
        # Maximum allowed axis position in mm (anything beyond is suspicious)
        self._max_axis_position_mm: float = 10000.0

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure IDS."""
        params = self.declare_and_validate_parameters({
            'scan_interval_sec': {
                'default': 5.0,
                'type': float,
                'range': (1.0, 60.0),
            },
            'max_msg_rate_per_node': {
                'default': 1000,
                'type': int,
                'range': (10, 100000),
            },
            'unknown_node_alert': {
                'default': True,
                'type': bool,
            },
        })

        self._max_rate = params['max_msg_rate_per_node']
        self._alert_unknown = params['unknown_node_alert']

        self._heartbeat_sub = self.create_subscription(
            Heartbeat,
            '/miracle/heartbeats',
            self._on_heartbeat,
            QoSProfiles.heartbeat(),
        )

        self._alert_pub = self.create_publisher(
            SecurityAlert,
            '/miracle/security/alerts',
            QoSProfiles.alert(),
        )

        self.get_logger().info("Intrusion detection configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate IDS scanning."""
        interval = self.get_parameter('scan_interval_sec').value
        self._scan_timer = self.create_timer(
            interval,
            self._run_scan,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("Intrusion detection activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate IDS."""
        if self._scan_timer is not None:
            self._scan_timer.cancel()
            self._scan_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_heartbeat(self, msg: Heartbeat) -> None:
        """Analyze heartbeat for anomalies."""
        node_id = f"{msg.node_namespace}/{msg.node_name}"

        with self._lock:
            # Track message rate using monotonic clock for precision
            now = time.monotonic()
            self._msg_counts[node_id] += 1
            self._msg_history[node_id].append(now)
            self._burst_history[node_id].append(now)

            # Check for unknown nodes
            if node_id not in self._known_nodes:
                if self._alert_unknown and len(self._known_nodes) > 0:
                    self._publish_alert(
                        severity='MEDIUM',
                        category='UNKNOWN_NODE',
                        source=node_id,
                        description=f'Unknown node detected: {node_id}',
                        affected=[node_id],
                        confidence=0.7,
                    )
                self._known_nodes.add(node_id)

    def _run_scan(self) -> None:
        """Periodic security scan."""
        with self._lock:
            interval = self.get_parameter('scan_interval_sec').value

            for node_id, count in self._msg_counts.items():
                rate = count / interval
                if rate > self._max_rate:
                    self._publish_alert(
                        severity='HIGH',
                        category='RATE_ANOMALY',
                        source=node_id,
                        description=(
                            f'Abnormal message rate from {node_id}: '
                            f'{rate:.0f} msg/s'
                        ),
                        affected=[node_id],
                        confidence=0.85,
                    )

            # Burst detection: check if >10x normal rate in 1s window
            now = time.monotonic()
            for node_id, timestamps in self._burst_history.items():
                if len(timestamps) < 2:
                    continue
                # Count messages in the last 1 second
                one_sec_ago = now - 1.0
                burst_count = sum(1 for ts in timestamps if ts >= one_sec_ago)
                # Normal rate per second
                normal_rate = self._max_rate / interval if interval > 0 else 1
                if burst_count > normal_rate * 10:
                    self._publish_alert(
                        severity='CRITICAL',
                        category='BURST_DETECTED',
                        source=node_id,
                        description=(
                            f'Burst detected from {node_id}: '
                            f'{burst_count} msgs in 1s window '
                            f'(>{normal_rate * 10:.0f} threshold)'
                        ),
                        affected=[node_id],
                        confidence=0.95,
                    )

            # Check for disappeared nodes (potential DoS)
            for node_id in list(self._known_nodes):
                if node_id in self._msg_history:
                    history = self._msg_history[node_id]
                    if history and (now - history[-1]) > interval * 10:
                        self._publish_alert(
                            severity='LOW',
                            category='NODE_DISAPPEARED',
                            source=node_id,
                            description=f'Node disappeared: {node_id}',
                            affected=[node_id],
                            confidence=0.5,
                        )

            # Reset counters
            self._msg_counts.clear()

    def _publish_alert(
        self,
        severity: str,
        category: str,
        source: str,
        description: str,
        affected: List[str],
        confidence: float,
    ) -> None:
        """Publish a security alert."""
        self._alert_count += 1
        msg = SecurityAlert()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.alert_id = f"IDS-{self._alert_count:06d}"
        msg.severity = severity
        msg.category = category
        msg.source_node = source
        msg.description = description
        msg.affected_nodes = affected
        msg.recommended_action = self._get_recommendation(category)
        msg.requires_isolation = severity in ('CRITICAL', 'HIGH')
        msg.confidence = confidence

        self._alert_pub.publish(msg)
        self.get_logger().warn(f"Security alert: [{severity}] {description}")

    def _inspect_payload(self, node_id: str, payload: Dict) -> bool:
        """Inspect message payload for unreasonable values.

        Checks axis positions and other numeric fields for values that
        would indicate a spoofed or corrupted message.

        Args:
            node_id: Source node identifier.
            payload: Message payload as a dictionary.

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

    @staticmethod
    def _get_recommendation(category: str) -> str:
        """Get recommended action for alert category."""
        recommendations = {
            'UNKNOWN_NODE': 'Verify node identity and authorize if legitimate',
            'RATE_ANOMALY': 'Investigate source for potential flood attack',
            'BURST_DETECTED': 'Immediately rate-limit or isolate the source node',
            'PAYLOAD_ANOMALY': 'Isolate node and inspect for compromise or sensor malfunction',
            'NODE_DISAPPEARED': 'Check node health and network connectivity',
            'SIGNATURE_MATCH': 'Isolate affected node immediately',
        }
        return recommendations.get(category, 'Investigate and respond')


def main(args=None):
    """Entry point for the IDS node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = IntrusionDetectionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
