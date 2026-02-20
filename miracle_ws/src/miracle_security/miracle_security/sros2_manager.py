"""
SROS2 Security Manager Node.

Manages SROS2 (Secure ROS2) keystore, certificates, and
access control policies for DDS security.
"""

from typing import Any, Dict, Optional
import os

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles


class SROS2ManagerNode(MiracleLifecycleNode):
    """Manages SROS2 security infrastructure.

    Parameters:
        keystore_path (str): Path to SROS2 keystore.
        auto_generate (bool): Auto-generate keys for new nodes.
        policy_directory (str): Directory with security policies.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'sros2_manager',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._keystore_path: str = ''
        self._policy_dir: str = ''

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure SROS2 manager."""
        params = self.declare_and_validate_parameters({
            'keystore_path': {
                'default': '/tmp/miracle_keystore',
                'type': str,
            },
            'auto_generate': {
                'default': False,
                'type': bool,
            },
            'policy_directory': {
                'default': '',
                'type': str,
            },
        })

        self._keystore_path = params['keystore_path']
        self._policy_dir = params['policy_directory']

        os.makedirs(self._keystore_path, exist_ok=True)

        self.get_logger().info(
            f"SROS2 manager configured (keystore={self._keystore_path})"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate SROS2 management."""
        self.get_logger().info("SROS2 manager activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate SROS2 management."""
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    """Entry point for the SROS2 manager node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = SROS2ManagerNode()
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
