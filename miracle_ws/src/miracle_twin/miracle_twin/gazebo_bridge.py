"""
Gazebo Simulation Bridge Node.

Bridges ROS2 data to Gazebo simulation for digital twin visualization.
Publishes joint states, receives simulation sensor feedback.
"""

from typing import Any, Dict, Optional

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState


class GazeboBridgeNode(MiracleLifecycleNode):
    """Bridges machine state to Gazebo for visualization.

    Parameters:
        machine_id (str): Machine identifier.
        gazebo_model_name (str): Gazebo model name.
        update_rate_hz (float): Gazebo update rate.

    Subscribed Topics:
        ~/twin_state (MachineState): Twin state to visualize.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'gazebo_bridge',
            criticality=self.CRITICALITY_LOW,
            **kwargs,
        )
        self._state_sub = None
        self._machine_id: str = ''
        self._model_name: str = ''
        self._last_state: Optional[MachineState] = None

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure Gazebo bridge."""
        params = self.declare_and_validate_parameters({
            'machine_id': {'default': 'cnc1', 'type': str},
            'gazebo_model_name': {
                'default': 'cnc_machine_1',
                'type': str,
            },
            'update_rate_hz': {
                'default': 30.0,
                'type': float,
                'range': (1.0, 120.0),
            },
        })

        self._machine_id = params['machine_id']
        self._model_name = params['gazebo_model_name']

        self._state_sub = self.create_subscription(
            MachineState,
            '/miracle/twin/twin_state',
            self._on_twin_state,
            QoSProfiles.state_data(),
        )

        self.get_logger().info(
            f"Gazebo bridge configured: {self._model_name}"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate Gazebo bridge."""
        self.get_logger().info("Gazebo bridge activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate Gazebo bridge."""
        return TransitionCallbackReturn.SUCCESS

    def _on_twin_state(self, msg: MachineState) -> None:
        """Forward twin state to Gazebo."""
        self._last_state = msg
        # In production, this would use Gazebo transport
        # to set joint positions and velocities


def main(args=None):
    """Entry point for the Gazebo bridge node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = GazeboBridgeNode()
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
