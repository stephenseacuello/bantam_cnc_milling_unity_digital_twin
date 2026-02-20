"""
MTConnect Agent Bridge Node.

Bridges MTConnect protocol to ROS2. Reads MTConnect XML streams
from CNC machines and publishes as ROS2 messages.
"""

from typing import Any, Dict, Optional
import xml.etree.ElementTree as ET

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState


class MTConnectAgentNode(MiracleLifecycleNode):
    """Bridges MTConnect to ROS2.

    Parameters:
        agent_url (str): MTConnect agent HTTP URL.
        machine_id (str): Machine identifier.
        poll_rate_hz (float): Polling rate for MTConnect current endpoint.
        device_name (str): MTConnect device name.

    Published Topics:
        ~/state (MachineState): Machine state from MTConnect.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'mtconnect_agent',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._state_pub = None
        self._poll_timer = None
        self._agent_url: str = ''
        self._machine_id: str = ''
        self._device_name: str = ''
        self._sequence: int = 0

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure MTConnect agent."""
        params = self.declare_and_validate_parameters({
            'agent_url': {
                'default': 'http://localhost:5000',
                'type': str,
            },
            'machine_id': {
                'default': 'cnc1',
                'type': str,
            },
            'poll_rate_hz': {
                'default': 5.0,
                'type': float,
                'range': (0.1, 100.0),
            },
            'device_name': {
                'default': 'CNC1',
                'type': str,
            },
        })

        self._agent_url = params['agent_url']
        self._machine_id = params['machine_id']
        self._device_name = params['device_name']

        self._state_pub = self.create_publisher(
            MachineState,
            'state',
            QoSProfiles.state_data(),
        )

        self.get_logger().info(
            f"MTConnect agent configured: {self._agent_url}"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Start polling MTConnect agent."""
        rate = self.get_parameter('poll_rate_hz').value
        self._poll_timer = self.create_timer(
            1.0 / rate,
            self._poll_mtconnect,
            callback_group=self.realtime_callback_group,
        )
        self.get_logger().info("MTConnect agent activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Stop polling."""
        if self._poll_timer is not None:
            self._poll_timer.cancel()
            self._poll_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _poll_mtconnect(self) -> None:
        """Poll MTConnect current endpoint."""
        try:
            import urllib.request
            url = f"{self._agent_url}/current"
            with urllib.request.urlopen(url, timeout=2) as response:
                xml_data = response.read().decode()
            self._parse_mtconnect_xml(xml_data)
        except Exception as exc:
            self.get_logger().debug(
                f"MTConnect poll failed: {exc}",
                throttle_duration_sec=10.0,
            )

    def _parse_mtconnect_xml(self, xml_data: str) -> None:
        """Parse MTConnect XML response into MachineState."""
        try:
            root = ET.fromstring(xml_data)
            ns = {'mt': 'urn:mtconnect.org:MTConnectStreams:2.0'}

            msg = MachineState()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.machine_id = self._machine_id

            # Extract data items
            for event in root.findall('.//mt:Events/*', ns):
                tag = event.tag.split('}')[-1] if '}' in event.tag else event.tag
                if tag == 'Execution':
                    status_map = {
                        'ACTIVE': 'RUNNING',
                        'READY': 'IDLE',
                        'STOPPED': 'PAUSED',
                        'INTERRUPTED': 'ERROR',
                    }
                    msg.status = status_map.get(event.text, 'UNKNOWN')
                elif tag == 'Program':
                    msg.current_program = event.text or ''

            for sample in root.findall('.//mt:Samples/*', ns):
                tag = sample.tag.split('}')[-1] if '}' in sample.tag else sample.tag
                try:
                    value = float(sample.text) if sample.text else 0.0
                except ValueError:
                    value = 0.0

                if tag == 'SpindleSpeed':
                    msg.spindle_speed = value
                elif tag == 'PathFeedrate':
                    msg.feed_rate = value
                elif tag == 'Load':
                    msg.spindle_load = value

            msg.axis_positions = [0.0] * 6
            msg.axis_velocities = [0.0] * 6

            self._state_pub.publish(msg)

        except ET.ParseError as exc:
            self.get_logger().warn(f"MTConnect XML parse error: {exc}")


def main(args=None):
    """Entry point for the MTConnect agent node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = MTConnectAgentNode()
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
