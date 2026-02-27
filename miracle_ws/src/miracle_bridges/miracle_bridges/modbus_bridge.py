"""
Modbus Bridge Node.

Bridges Modbus TCP/RTU to ROS2. Reads/writes Modbus registers
for PLC and sensor integration.

Includes exponential backoff reconnection logic so that transient
connection failures are retried with increasing delays (1 s -> 2 s
-> 4 s ... capped at 60 s).
"""

from typing import Any, Dict, List, Optional
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import SensorData


class ModbusBridgeNode(MiracleLifecycleNode):
    """Bridges Modbus TCP/RTU to ROS2.

    Parameters:
        host (str): Modbus TCP host.
        port (int): Modbus TCP port.
        unit_id (int): Modbus unit/slave ID.
        machine_id (str): Machine identifier.
        poll_rate_hz (float): Register polling rate.
        register_map_json (str): JSON string of register mappings.

    Published Topics:
        ~/sensor_data (SensorData): Sensor data from Modbus registers.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'modbus_bridge',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._sensor_pub = None
        self._poll_timer = None
        self._client = None
        self._connected: bool = False
        self._machine_id: str = ''
        self._register_map: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()

        # Exponential backoff reconnection state
        self._reconnect_attempts: int = 0
        self._max_backoff: float = 60.0
        self._base_backoff: float = 1.0
        self._reconnect_timer = None

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure Modbus bridge."""
        params = self.declare_and_validate_parameters({
            'host': {
                'default': 'localhost',
                'type': str,
            },
            'port': {
                'default': 502,
                'type': int,
                'range': (1, 65535),
            },
            'unit_id': {
                'default': 1,
                'type': int,
                'range': (1, 247),
            },
            'machine_id': {
                'default': 'cnc1',
                'type': str,
            },
            'poll_rate_hz': {
                'default': 10.0,
                'type': float,
                'range': (0.1, 100.0),
            },
        })

        self._machine_id = params['machine_id']

        # Default register map
        self._register_map = {
            'spindle_speed': {'address': 0, 'count': 2, 'type': 'float32'},
            'feed_rate': {'address': 2, 'count': 2, 'type': 'float32'},
            'spindle_load': {'address': 4, 'count': 1, 'type': 'uint16'},
            'coolant_temp': {'address': 5, 'count': 1, 'type': 'uint16'},
            'axis_x': {'address': 10, 'count': 2, 'type': 'float32'},
            'axis_y': {'address': 12, 'count': 2, 'type': 'float32'},
            'axis_z': {'address': 14, 'count': 2, 'type': 'float32'},
        }

        self._sensor_pub = self.create_publisher(
            SensorData,
            'sensor_data',
            QoSProfiles.sensor_data(),
        )

        self.get_logger().info(
            f"Modbus bridge configured: "
            f"{params['host']}:{params['port']}"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Connect and start polling."""
        self._reconnect_attempts = 0
        self._connect_modbus()

        rate = self.get_parameter('poll_rate_hz').value
        self._poll_timer = self.create_timer(
            1.0 / rate,
            self._poll_registers,
            callback_group=self.realtime_callback_group,
        )
        self.get_logger().info("Modbus bridge activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Disconnect and stop polling."""
        if self._poll_timer is not None:
            self._poll_timer.cancel()
            self._poll_timer = None
        if self._reconnect_timer is not None:
            self._reconnect_timer.cancel()
            self._reconnect_timer = None
        self._disconnect_modbus()
        return TransitionCallbackReturn.SUCCESS

    def _connect_modbus(self) -> None:
        """Connect to Modbus device."""
        try:
            from pymodbus.client import ModbusTcpClient

            host = self.get_parameter('host').value
            port = self.get_parameter('port').value

            self._client = ModbusTcpClient(host, port=port)
            self._connected = self._client.connect()
            if self._connected:
                self.get_logger().info(f"Connected to Modbus: {host}:{port}")
            else:
                self.get_logger().warn(f"Modbus connection failed: {host}:{port}")
        except ImportError:
            self.get_logger().warn("pymodbus not installed, bridge inactive")
            self._connected = False
        except Exception as exc:
            self.get_logger().warn(f"Modbus connection error: {exc}")
            self._connected = False

    def _disconnect_modbus(self) -> None:
        """Disconnect from Modbus."""
        if self._client is not None and self._connected:
            try:
                self._client.close()
            except Exception as exc:
                self.get_logger().warn(f"Modbus disconnect error: {exc}")
            self._connected = False

    def _schedule_reconnect(self) -> None:
        """Schedule a reconnection attempt with exponential backoff.

        Delay doubles each attempt: 1 s -> 2 s -> 4 s ... capped at
        ``_max_backoff`` (default 60 s).
        """
        delay = min(
            self._base_backoff * (2 ** self._reconnect_attempts),
            self._max_backoff,
        )
        self._reconnect_attempts += 1
        self.get_logger().warn(
            f"Scheduling Modbus reconnect in {delay:.1f}s "
            f"(attempt {self._reconnect_attempts})"
        )

        # Cancel any previous reconnect timer before creating a new one
        if self._reconnect_timer is not None:
            self._reconnect_timer.cancel()

        self._reconnect_timer = self.create_timer(
            delay,
            self._retry_connect,
        )

    def _retry_connect(self) -> None:
        """Attempt to reconnect to the Modbus device.

        Called by the one-shot reconnect timer. On success the backoff
        counter is reset; on failure a new timer is scheduled with a
        longer delay.
        """
        # Cancel the one-shot timer so it does not fire again
        if self._reconnect_timer is not None:
            self._reconnect_timer.cancel()
            self._reconnect_timer = None

        self.get_logger().info(
            f"Attempting Modbus reconnect (attempt {self._reconnect_attempts})"
        )
        self._connect_modbus()

        if self._connected:
            self._reconnect_attempts = 0
            self.get_logger().info("Modbus reconnection successful")
        else:
            self._schedule_reconnect()

    def _poll_registers(self) -> None:
        """Poll Modbus registers and publish sensor data."""
        if not self._connected:
            return

        try:
            unit = self.get_parameter('unit_id').value
            values: List[float] = []
            labels: List[str] = []

            for name, reg in self._register_map.items():
                result = self._client.read_holding_registers(
                    address=reg['address'],
                    count=reg['count'],
                    slave=unit,
                )
                if result.isError():
                    self.get_logger().debug(
                        f"Modbus read error for {name}"
                    )
                    continue

                if reg['type'] == 'float32' and reg['count'] == 2:
                    import struct
                    raw = struct.pack('>HH', *result.registers)
                    value = struct.unpack('>f', raw)[0]
                else:
                    value = float(result.registers[0])

                values.append(value)
                labels.append(name)

            if values:
                msg = SensorData()
                msg.timestamp = self.get_clock().now().to_msg()
                msg.machine_id = self._machine_id
                msg.sensor_type = 'modbus'
                msg.sensor_id = f'modbus_{unit}'
                msg.values = values
                msg.labels = labels
                msg.sample_rate = self.get_parameter('poll_rate_hz').value
                msg.quality = 1.0

                self._sensor_pub.publish(msg)

        except Exception as exc:
            self.get_logger().error(f"Modbus poll error: {exc}")
            self._connected = False
            self._schedule_reconnect()


def main(args=None):
    """Entry point for the Modbus bridge node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = ModbusBridgeNode()
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
