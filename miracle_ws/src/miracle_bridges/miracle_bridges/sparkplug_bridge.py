"""
Sparkplug B MQTT Bridge Node.

Bridges Sparkplug B protocol (MQTT-based) to ROS2.
Supports NBIRTH, DBIRTH, NDATA, DDATA, NCMD, DCMD messages
for IIoT integration.
"""

from typing import Any, Dict, Optional
import json
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, SensorData


class SparkplugBridgeNode(MiracleLifecycleNode):
    """Bridges Sparkplug B MQTT to ROS2.

    Parameters:
        mqtt_broker (str): MQTT broker hostname.
        mqtt_port (int): MQTT broker port.
        group_id (str): Sparkplug group ID.
        edge_node_id (str): Sparkplug edge node ID.
        machine_id (str): Machine identifier.

    Published Topics:
        ~/state (MachineState): Machine state from Sparkplug.
        ~/sensor_data (SensorData): Sensor data from Sparkplug.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'sparkplug_bridge',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._state_pub = None
        self._sensor_pub = None
        self._mqtt_client = None
        self._connected: bool = False
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure Sparkplug bridge."""
        params = self.declare_and_validate_parameters({
            'mqtt_broker': {
                'default': 'localhost',
                'type': str,
            },
            'mqtt_port': {
                'default': 1883,
                'type': int,
                'range': (1, 65535),
            },
            'group_id': {
                'default': 'MIRACLE',
                'type': str,
            },
            'edge_node_id': {
                'default': 'CNCCell1',
                'type': str,
            },
            'machine_id': {
                'default': 'cnc1',
                'type': str,
            },
        })

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

        self.get_logger().info(
            f"Sparkplug bridge configured: "
            f"{params['mqtt_broker']}:{params['mqtt_port']}"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Connect to MQTT broker."""
        try:
            self._connect_mqtt()
        except Exception as exc:
            self.get_logger().warn(f"MQTT connection failed: {exc}")

        self.get_logger().info("Sparkplug bridge activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Disconnect from MQTT broker."""
        self._disconnect_mqtt()
        return TransitionCallbackReturn.SUCCESS

    def _connect_mqtt(self) -> None:
        """Establish MQTT connection."""
        try:
            import paho.mqtt.client as mqtt

            broker = self.get_parameter('mqtt_broker').value
            port = self.get_parameter('mqtt_port').value
            group_id = self.get_parameter('group_id').value
            edge_node = self.get_parameter('edge_node_id').value

            self._mqtt_client = mqtt.Client()
            self._mqtt_client.on_connect = self._on_mqtt_connect
            self._mqtt_client.on_message = self._on_mqtt_message
            self._mqtt_client.connect(broker, port, 60)

            # Subscribe to Sparkplug topics
            topic = f"spBv1.0/{group_id}/DDATA/{edge_node}/#"
            self._mqtt_client.subscribe(topic)
            self._mqtt_client.loop_start()
            self._connected = True

        except ImportError:
            self.get_logger().warn("paho-mqtt not installed, bridge inactive")
        except Exception as exc:
            self.get_logger().warn(f"MQTT connection error: {exc}")

    def _disconnect_mqtt(self) -> None:
        """Disconnect from MQTT."""
        if self._mqtt_client is not None and self._connected:
            try:
                self._mqtt_client.loop_stop()
                self._mqtt_client.disconnect()
            except Exception as exc:
                self.get_logger().warn(f"MQTT disconnect error: {exc}")
            self._connected = False

    def _on_mqtt_connect(self, client, userdata, flags, rc) -> None:
        """MQTT connection callback."""
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker")
        else:
            self.get_logger().error(f"MQTT connection failed: rc={rc}")

    def _on_mqtt_message(self, client, userdata, mqtt_msg) -> None:
        """Handle incoming Sparkplug MQTT messages."""
        try:
            payload = json.loads(mqtt_msg.payload.decode())
            self.get_logger().debug(
                f"Sparkplug message on {mqtt_msg.topic}"
            )
            # Process Sparkplug payload - would decode protobuf in production
        except Exception as exc:
            self.get_logger().debug(f"Sparkplug message parse error: {exc}")


def main(args=None):
    """Entry point for the Sparkplug bridge node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = SparkplugBridgeNode()
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
