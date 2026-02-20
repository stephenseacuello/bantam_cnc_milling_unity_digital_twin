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
        })

        self._machine_id = params['machine_id']
        self._simulation_mode = params['simulation_mode']

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

    def _read_real_state(self) -> MachineState:
        """Read state from a real CNC machine interface.

        This is a placeholder - actual implementation would read from
        OPC-UA, MTConnect, or Modbus bridge.
        """
        msg = MachineState()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.status = 'IDLE'
        msg.axis_positions = [0.0] * 6
        msg.axis_velocities = [0.0] * 6
        self.get_logger().warn("Real machine interface not implemented", once=True)
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
