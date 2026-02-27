"""
Unity TCP Endpoint Launcher.

Wraps the ros_tcp_endpoint to provide MIRACLE-specific configuration
and topic registration for the Unity Digital Twin.

Includes:
- 5-second health-check timer that tracks ``_last_msg_time``
- Subscription to ``/miracle/unity/heartbeat`` for keepalive
- Warning logs on keepalive timeout
- Published connection status on ``~/connection_status``
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool, String


# Topic types registered by the Unity bridge
REGISTERED_TOPIC_TYPES = [
    'miracle_msgs/msg/MachineState',
    'miracle_msgs/msg/AnomalyAlert',
    'miracle_msgs/msg/ToolWearEstimate',
    'miracle_msgs/msg/TwinSyncStatus',
    'miracle_msgs/msg/SystemKPIs',
    'miracle_msgs/msg/JobStatus',
    'miracle_msgs/msg/FleetHealth',
    'miracle_msgs/msg/Heartbeat',
    'miracle_msgs/msg/SecurityAlert',
    'miracle_msgs/msg/TaskAward',
    'miracle_msgs/srv/TriggerEStop',
    'miracle_msgs/srv/ValidateGCode',
    'miracle_msgs/srv/GetFleetStatus',
]


class UnityEndpointConfig(Node):
    """Configuration node for Unity TCP endpoint.

    Publishes connection status and manages topic registration
    for the Unity Digital Twin visualization.

    Parameters:
        tcp_ip (str): TCP bind address (default ``0.0.0.0``).
        tcp_port (int): TCP port (default ``10000``).
        keepalive_timeout (float): Seconds without a heartbeat before
            the connection is considered unhealthy (default ``10.0``).
        log_level (str): Logging verbosity (default ``info``).

    Published Topics:
        ~/connection_status (std_msgs/Bool): ``True`` while the Unity
            client is considered connected.

    Subscribed Topics:
        /miracle/unity/heartbeat (std_msgs/String): Heartbeat messages
            from the Unity client.
    """

    def __init__(self):
        super().__init__('unity_endpoint_config')

        # -- Parameters --------------------------------------------------
        self.declare_parameter('tcp_ip', '0.0.0.0')
        self.declare_parameter('tcp_port', 10000)
        self.declare_parameter('keepalive_timeout', 10.0)
        self.declare_parameter('log_level', 'info')

        tcp_ip = self.get_parameter('tcp_ip').get_parameter_value().string_value
        tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        self._keepalive_timeout: float = (
            self.get_parameter('keepalive_timeout')
            .get_parameter_value()
            .double_value
        )

        # -- Internal state -----------------------------------------------
        self._last_msg_time: float = time.monotonic()
        self._unity_connected: bool = False

        # -- Connection status publisher -----------------------------------
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._status_pub = self.create_publisher(
            Bool,
            '~/connection_status',
            status_qos,
        )

        # -- Unity heartbeat subscriber ------------------------------------
        self._heartbeat_sub = self.create_subscription(
            String,
            '/miracle/unity/heartbeat',
            self._on_unity_heartbeat,
            10,
        )

        # -- 5-second health check timer -----------------------------------
        self._health_timer = self.create_timer(5.0, self._check_health)

        # -- Startup logging -----------------------------------------------
        self.get_logger().info(
            f'Unity TCP Endpoint configured: {tcp_ip}:{tcp_port}'
        )
        self.get_logger().info('Registered MIRACLE topic types:')
        for topic_type in REGISTERED_TOPIC_TYPES:
            self.get_logger().info(f'  - {topic_type}')

        # Publish initial disconnected status
        self._publish_connection_status()

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_unity_heartbeat(self, msg: String) -> None:
        """Handle an incoming heartbeat from the Unity client."""
        self._last_msg_time = time.monotonic()
        if not self._unity_connected:
            self._unity_connected = True
            self.get_logger().info('Unity client connected (heartbeat received)')
            self._publish_connection_status()

    def _check_health(self) -> None:
        """Periodic health check (runs every 5 s).

        If no message has been received within ``keepalive_timeout``
        seconds the connection is marked as lost and a warning is logged.
        """
        elapsed = time.monotonic() - self._last_msg_time
        if self._unity_connected and elapsed > self._keepalive_timeout:
            self._unity_connected = False
            self.get_logger().warn(
                f'Unity keepalive timeout: no message for {elapsed:.1f}s '
                f'(threshold {self._keepalive_timeout:.1f}s)'
            )
            self._publish_connection_status()

    def _publish_connection_status(self) -> None:
        """Publish the current connection status."""
        msg = Bool()
        msg.data = self._unity_connected
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Public helpers (useful in tests)
    # ------------------------------------------------------------------

    def record_message_received(self) -> None:
        """Record that *any* message was received from Unity.

        Can be called by external code (e.g. the TCP transport layer)
        to keep the keepalive timer alive even when no dedicated
        heartbeat message is used.
        """
        self._last_msg_time = time.monotonic()


def main(args=None):
    """Launch the Unity endpoint configuration node."""
    rclpy.init(args=args)
    node = UnityEndpointConfig()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
