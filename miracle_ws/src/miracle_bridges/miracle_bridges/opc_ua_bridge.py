"""
OPC-UA Bridge Node.

Bridges OPC-UA industrial protocol to ROS2 topics.
Supports reading/writing OPC-UA variables and mapping them
to ROS2 messages for CNC machine integration.
"""

from typing import Any, Dict, List, Optional
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, SensorData


class OPCUABridgeNode(MiracleLifecycleNode):
    """Bridges OPC-UA to ROS2.

    Parameters:
        server_url (str): OPC-UA server endpoint URL.
        machine_id (str): Machine identifier.
        poll_rate_hz (float): OPC-UA polling rate.
        namespace_index (int): OPC-UA namespace index.
        security_policy (str): OPC-UA security policy.
        username (str): OPC-UA username (optional).

    Published Topics:
        ~/state (MachineState): Machine state from OPC-UA.
        ~/sensor_data (SensorData): Sensor data from OPC-UA.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'opc_ua_bridge',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._state_pub = None
        self._sensor_pub = None
        self._poll_timer = None
        self._client = None
        self._connected: bool = False
        self._server_url: str = ''
        self._machine_id: str = ''
        self._node_mappings: Dict[str, str] = {}
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure OPC-UA bridge."""
        params = self.declare_and_validate_parameters({
            'server_url': {
                'default': 'opc.tcp://localhost:4840',
                'type': str,
                'description': 'OPC-UA server endpoint URL',
            },
            'machine_id': {
                'default': 'cnc1',
                'type': str,
            },
            'poll_rate_hz': {
                'default': 10.0,
                'type': float,
                'range': (0.1, 1000.0),
            },
            'namespace_index': {
                'default': 2,
                'type': int,
                'range': (0, 100),
            },
            'security_policy': {
                'default': 'None',
                'type': str,
                'choices': ['None', 'Basic128Rsa15', 'Basic256', 'Basic256Sha256'],
            },
        })

        self._server_url = params['server_url']
        self._machine_id = params['machine_id']

        self._state_pub = self.create_publisher(
            MachineState,
            'state',
            QoSProfiles.state_data(),
        )

        self._sensor_pub = self.create_publisher(
            SensorData,
            'sensor_data',
            QoSProfiles.sensor_data(),
        )

        # Define OPC-UA node ID to ROS field mappings
        ns = params['namespace_index']
        self._node_mappings = {
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

        self.get_logger().info(
            f"OPC-UA bridge configured: {self._server_url}"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Connect to OPC-UA server and start polling."""
        try:
            self._connect_opcua()
        except Exception as exc:
            self.get_logger().error(f"OPC-UA connection failed: {exc}")
            self.get_logger().warn("Running in simulation mode")

        rate = self.get_parameter('poll_rate_hz').value
        self._poll_timer = self.create_timer(
            1.0 / rate,
            self._poll_opcua,
            callback_group=self.realtime_callback_group,
        )

        self.get_logger().info("OPC-UA bridge activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Disconnect from OPC-UA server."""
        if self._poll_timer is not None:
            self._poll_timer.cancel()
            self._poll_timer = None
        self._disconnect_opcua()
        return TransitionCallbackReturn.SUCCESS

    def _connect_opcua(self) -> None:
        """Establish OPC-UA connection."""
        try:
            from opcua import Client as OPCUAClient
            self._client = OPCUAClient(self._server_url)
            self._client.connect()
            self._connected = True
            self.get_logger().info(f"Connected to OPC-UA: {self._server_url}")
        except ImportError:
            self.get_logger().warn(
                "opcua package not installed, using simulated data"
            )
            self._connected = False
        except Exception as exc:
            self.get_logger().error(f"OPC-UA connection error: {exc}")
            self._connected = False

    def _disconnect_opcua(self) -> None:
        """Disconnect from OPC-UA server."""
        if self._client is not None and self._connected:
            try:
                self._client.disconnect()
            except Exception as exc:
                self.get_logger().warn(f"OPC-UA disconnect error: {exc}")
            self._connected = False
            self._client = None

    def _poll_opcua(self) -> None:
        """Poll OPC-UA server for current values."""
        if not self._connected:
            self._publish_simulated_state()
            return

        try:
            values: Dict[str, Any] = {}
            for node_id, field_name in self._node_mappings.items():
                try:
                    node = self._client.get_node(node_id)
                    values[field_name] = node.get_value()
                except Exception as exc:
                    self.get_logger().debug(
                        f"Failed to read {node_id}: {exc}"
                    )

            self._publish_state_from_values(values)

        except Exception as exc:
            self.get_logger().error(f"OPC-UA poll error: {exc}")
            self._connected = False

    def _publish_state_from_values(self, values: Dict[str, Any]) -> None:
        """Build and publish MachineState from OPC-UA values."""
        msg = MachineState()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.status = str(values.get('status', 'UNKNOWN'))
        msg.spindle_speed = float(values.get('spindle_speed', 0.0))
        msg.feed_rate = float(values.get('feed_rate', 0.0))
        msg.spindle_load = float(values.get('spindle_load', 0.0))
        msg.coolant_level = float(values.get('coolant_level', 100.0))
        msg.axis_positions = [
            float(values.get('axis_x', 0.0)),
            float(values.get('axis_y', 0.0)),
            float(values.get('axis_z', 0.0)),
            0.0, 0.0, 0.0,
        ]
        msg.axis_velocities = [0.0] * 6
        msg.current_program = str(values.get('current_program', ''))
        msg.current_line = int(values.get('current_line', 0))

        self._state_pub.publish(msg)

    def _publish_simulated_state(self) -> None:
        """Publish simulated state when OPC-UA is not connected."""
        msg = MachineState()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.status = 'IDLE'
        msg.axis_positions = [0.0] * 6
        msg.axis_velocities = [0.0] * 6

        self._state_pub.publish(msg)


def main(args=None):
    """Entry point for the OPC-UA bridge node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = OPCUABridgeNode()
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
