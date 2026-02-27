"""
CNC Machine State Publisher Node.

Publishes the current state of a CNC machine at a configurable rate.
Reads from machine interface (simulated or real via protocol bridge)
and publishes MachineState messages.
"""

from typing import Any, Dict, Optional
import random
import math

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState
from miracle_msgs.srv import TriggerEStop


class StatePublisherNode(MiracleLifecycleNode):
    """Publishes CNC machine state at a configurable rate.

    Parameters:
        machine_id (str): Unique machine identifier.
        publish_rate_hz (float): State publishing rate in Hz.
        simulation_mode (bool): If true, generate simulated data.

    Published Topics:
        ~/state (MachineState): Current machine state.

    Service Servers:
        ~/trigger_estop (TriggerEStop): Emergency stop handler.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'state_publisher',
            criticality=self.CRITICALITY_CRITICAL,
            **kwargs,
        )
        self._state_pub = None
        self._estop_srv = None
        self._timer = None
        self._machine_id: str = ''
        self._simulation_mode: bool = True
        self._estopped: bool = False

        # Simulated state
        self._sim_status: str = 'IDLE'
        self._sim_spindle: float = 0.0
        self._sim_feed: float = 0.0
        self._sim_positions: list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._sim_line: int = 0
        self._sim_total_lines: int = 0
        self._sim_cycle_start: float = 0.0
        self._sim_tick: int = 0

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure state publisher."""
        params = self.declare_and_validate_parameters({
            'machine_id': {
                'default': 'cnc1',
                'type': str,
                'description': 'Unique machine identifier',
            },
            'publish_rate_hz': {
                'default': 50.0,
                'type': float,
                'range': (1.0, 1000.0),
                'description': 'State publishing rate in Hz',
            },
            'simulation_mode': {
                'default': True,
                'type': bool,
                'description': 'Generate simulated state data',
            },
            'bridge_type': {
                'default': 'modbus',
                'type': str,
                'description': 'Protocol bridge type: modbus, opc_ua, or mtconnect',
            },
            'bridge_host': {
                'default': '127.0.0.1',
                'type': str,
                'description': 'Protocol bridge host address',
            },
            'bridge_port': {
                'default': 502,
                'type': int,
                'range': (1, 65535),
                'description': 'Protocol bridge port',
            },
        })

        self._machine_id = params['machine_id']
        self._simulation_mode = params['simulation_mode']
        self._bridge_type = params['bridge_type']
        self._bridge_host = params['bridge_host']
        self._bridge_port = params['bridge_port']
        self._bridge_client = None
        self._bridge_connected = False
        self._reconnect_attempts = 0

        if not self._simulation_mode:
            self._init_bridge_client()

        self._state_pub = self.create_publisher(
            MachineState,
            'state',
            QoSProfiles.state_data(),
        )

        self._estop_srv = self.create_service(
            TriggerEStop,
            'trigger_estop',
            self._handle_estop,
            callback_group=self.service_callback_group,
        )

        self.get_logger().info(
            f"Configured state publisher for '{self._machine_id}' "
            f"at {params['publish_rate_hz']}Hz "
            f"(sim={self._simulation_mode})"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Start publishing state."""
        rate = self.get_parameter('publish_rate_hz').value
        self._timer = self.create_timer(
            1.0 / rate,
            self._publish_state,
            callback_group=self.realtime_callback_group,
        )
        self.get_logger().info("State publisher activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Stop publishing state."""
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        return TransitionCallbackReturn.SUCCESS

    def _handle_estop(
        self,
        request: TriggerEStop.Request,
        response: TriggerEStop.Response,
    ) -> TriggerEStop.Response:
        """Handle emergency stop requests."""
        self.get_logger().error(
            f"E-STOP triggered by {request.requesting_node}: {request.reason}"
        )
        self._estopped = True
        self._sim_status = 'ESTOP'
        self._sim_spindle = 0.0
        self._sim_feed = 0.0
        response.success = True
        response.message = f"E-stop activated for {self._machine_id}"
        return response

    def _publish_state(self) -> None:
        """Publish current machine state."""
        if self._simulation_mode:
            msg = self._generate_simulated_state()
        else:
            msg = self._read_real_state()

        self._state_pub.publish(msg)

    def _generate_simulated_state(self) -> MachineState:
        """Generate a simulated machine state message."""
        self._sim_tick += 1
        t = self._sim_tick * 0.02  # ~50Hz

        msg = MachineState()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id

        # Simulate state transitions
        if self._sim_tick % 500 == 0:
            states = ['IDLE', 'RUNNING', 'RUNNING', 'RUNNING', 'PAUSED']
            self._sim_status = random.choice(states)

        msg.status = self._sim_status

        if self._estopped:
            msg.spindle_speed = 0.0
            msg.feed_rate = 0.0
            msg.axis_positions = self._sim_positions
            msg.axis_velocities = [0.0] * 6
            msg.spindle_load = 0.0
            msg.coolant_level = max(0.0, 95.0 - t * 0.001)
            msg.current_program = ''
            msg.current_line = 0
            msg.cycle_time_elapsed = 0.0
            msg.cycle_time_remaining = 0.0
            return msg

        if self._sim_status == 'RUNNING':
            self._sim_spindle = 8000.0 + 500.0 * math.sin(t * 0.1)
            self._sim_feed = 2000.0 + 200.0 * math.sin(t * 0.05)
            self._sim_positions = [
                50.0 * math.sin(t * 0.3),
                30.0 * math.cos(t * 0.2),
                -10.0 + 5.0 * math.sin(t * 0.1),
                0.0, 0.0, 0.0,
            ]
            self._sim_line = min(self._sim_line + 1, 1000)
            self._sim_total_lines = 1000
        else:
            self._sim_spindle = 0.0
            self._sim_feed = 0.0
            self._sim_line = 0

        msg.spindle_speed = self._sim_spindle
        msg.feed_rate = self._sim_feed
        msg.axis_positions = self._sim_positions
        msg.axis_velocities = [0.0] * 6
        msg.spindle_load = max(0.0, min(100.0, 35.0 + 15.0 * math.sin(t * 0.2)))
        msg.coolant_level = max(0.0, 95.0 - t * 0.001)
        msg.current_program = 'sim_program.nc' if self._sim_status == 'RUNNING' else ''
        msg.current_line = self._sim_line
        msg.cycle_time_elapsed = t if self._sim_status == 'RUNNING' else 0.0
        msg.cycle_time_remaining = max(0.0, 120.0 - t) if self._sim_status == 'RUNNING' else 0.0

        return msg

    def _init_bridge_client(self) -> None:
        """Initialize the protocol bridge client based on bridge_type."""
        try:
            if self._bridge_type == 'modbus':
                from pymodbus.client import ModbusTcpClient
                self._bridge_client = ModbusTcpClient(
                    self._bridge_host, port=self._bridge_port
                )
                self._bridge_connected = self._bridge_client.connect()
            elif self._bridge_type == 'opc_ua':
                from opcua import Client as OPCUAClient
                url = f"opc.tcp://{self._bridge_host}:{self._bridge_port}"
                self._bridge_client = OPCUAClient(url)
                self._bridge_client.connect()
                self._bridge_connected = True
            elif self._bridge_type == 'mtconnect':
                # MTConnect uses HTTP polling — store URL for _read_real_state
                self._bridge_client = (
                    f"http://{self._bridge_host}:{self._bridge_port}/current"
                )
                self._bridge_connected = True
            else:
                self.get_logger().error(
                    f"Unknown bridge type: {self._bridge_type}"
                )
                return

            if self._bridge_connected:
                self._reconnect_attempts = 0
                self.get_logger().info(
                    f"Connected to {self._bridge_type} bridge at "
                    f"{self._bridge_host}:{self._bridge_port}"
                )
            else:
                self.get_logger().warn(
                    f"Failed to connect to {self._bridge_type} bridge"
                )
        except Exception as exc:
            self._bridge_connected = False
            self.get_logger().error(
                f"Bridge connection error ({self._bridge_type}): {exc}"
            )

    def _try_reconnect(self) -> None:
        """Attempt to reconnect with exponential backoff."""
        max_backoff = 60.0
        backoff = min(1.0 * (2 ** self._reconnect_attempts), max_backoff)
        self._reconnect_attempts += 1
        self.get_logger().info(
            f"Reconnect attempt {self._reconnect_attempts} "
            f"(backoff {backoff:.1f}s)"
        )
        self._init_bridge_client()

    def _read_real_state(self) -> MachineState:
        """Read state from a real CNC machine via the configured protocol bridge."""
        msg = MachineState()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.axis_positions = [0.0] * 6
        msg.axis_velocities = [0.0] * 6

        if not self._bridge_connected:
            # Attempt periodic reconnection
            if self._reconnect_attempts % 10 == 0:
                self._try_reconnect()
            else:
                self._reconnect_attempts += 1
            msg.status = 'ERROR'
            return msg

        try:
            if self._bridge_type == 'modbus':
                msg = self._read_modbus_state(msg)
            elif self._bridge_type == 'opc_ua':
                msg = self._read_opcua_state(msg)
            elif self._bridge_type == 'mtconnect':
                msg = self._read_mtconnect_state(msg)
        except Exception as exc:
            self.get_logger().error(f"Bridge read error: {exc}")
            self._bridge_connected = False
            msg.status = 'ERROR'

        return msg

    def _read_modbus_state(self, msg: MachineState) -> MachineState:
        """Read machine state from Modbus registers."""
        import struct

        # Register map (matches modbus_bridge.py conventions):
        # 0-1: spindle_speed (float32), 2-3: feed_rate (float32),
        # 4-5: spindle_load (float32), 6-7: coolant_temp (float32),
        # 8-9: axis_x, 10-11: axis_y, 12-13: axis_z (float32 each)
        result = self._bridge_client.read_holding_registers(0, 14)
        if result.isError():
            raise RuntimeError(f"Modbus read error: {result}")

        regs = result.registers

        def regs_to_float(hi, lo):
            raw = struct.pack('>HH', hi, lo)
            return struct.unpack('>f', raw)[0]

        msg.spindle_speed = regs_to_float(regs[0], regs[1])
        msg.feed_rate = regs_to_float(regs[2], regs[3])
        msg.spindle_load = regs_to_float(regs[4], regs[5])
        msg.coolant_level = regs_to_float(regs[6], regs[7])
        msg.axis_positions = [
            regs_to_float(regs[8], regs[9]),
            regs_to_float(regs[10], regs[11]),
            regs_to_float(regs[12], regs[13]),
            0.0, 0.0, 0.0,
        ]

        # Derive status from spindle speed
        if self._estopped:
            msg.status = 'ESTOP'
        elif msg.spindle_speed > 100:
            msg.status = 'RUNNING'
        else:
            msg.status = 'IDLE'

        return msg

    def _read_opcua_state(self, msg: MachineState) -> MachineState:
        """Read machine state from OPC-UA nodes."""
        root = self._bridge_client.get_root_node()
        objects = root.get_child(["0:Objects", "2:CNCMachine"])

        msg.spindle_speed = float(
            objects.get_child("2:SpindleSpeed").get_value()
        )
        msg.feed_rate = float(
            objects.get_child("2:FeedRate").get_value()
        )
        msg.spindle_load = float(
            objects.get_child("2:SpindleLoad").get_value()
        )
        msg.coolant_level = float(
            objects.get_child("2:CoolantLevel").get_value()
        )
        msg.axis_positions = [
            float(objects.get_child("2:AxisPositionX").get_value()),
            float(objects.get_child("2:AxisPositionY").get_value()),
            float(objects.get_child("2:AxisPositionZ").get_value()),
            0.0, 0.0, 0.0,
        ]

        status_str = str(objects.get_child("2:MachineStatus").get_value())
        status_map = {
            'ACTIVE': 'RUNNING', 'READY': 'IDLE',
            'STOPPED': 'PAUSED', 'INTERRUPTED': 'ERROR',
        }
        msg.status = status_map.get(status_str.upper(), 'UNKNOWN')
        msg.current_program = str(
            objects.get_child("2:CurrentProgram").get_value()
        )
        msg.current_line = int(
            objects.get_child("2:CurrentLine").get_value()
        )

        return msg

    def _read_mtconnect_state(self, msg: MachineState) -> MachineState:
        """Read machine state from MTConnect HTTP endpoint."""
        import urllib.request
        import xml.etree.ElementTree as ET

        req = urllib.request.Request(self._bridge_client)
        with urllib.request.urlopen(req, timeout=2.0) as resp:
            xml_data = resp.read().decode('utf-8')

        root = ET.fromstring(xml_data)
        ns = {'m': root.tag.split('}')[0].strip('{')} if '}' in root.tag else {}

        def find_value(tag, default='0'):
            elem = root.find(f'.//{tag}', ns) if not ns else root.find(
                f'.//m:{tag}', ns
            )
            return elem.text if elem is not None else default

        msg.spindle_speed = float(find_value('SpindleSpeed', '0'))
        msg.feed_rate = float(find_value('PathFeedrate', '0'))
        msg.spindle_load = float(find_value('Load', '0'))

        exec_status = find_value('Execution', 'READY')
        status_map = {
            'ACTIVE': 'RUNNING', 'READY': 'IDLE',
            'STOPPED': 'PAUSED', 'INTERRUPTED': 'ERROR',
        }
        msg.status = status_map.get(exec_status.upper(), 'UNKNOWN')

        return msg


def main(args=None):
    """Entry point for the state publisher node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = StatePublisherNode()
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
