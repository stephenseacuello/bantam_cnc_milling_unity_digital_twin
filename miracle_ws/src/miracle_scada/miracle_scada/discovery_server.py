"""
Device Discovery Server Node.

Manages registration and discovery of all devices (CNC machines,
sensors, cobots) in the manufacturing cell. Maintains device registry
and tracks device capabilities.
"""

from typing import Any, Dict
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import DeviceTrustStatus
from miracle_msgs.srv import RegisterDevice


class DiscoveryServerNode(MiracleLifecycleNode):
    """Manages device registration and discovery.

    Parameters:
        max_devices (int): Maximum number of registered devices.
        initial_trust_score (float): Default trust score for new devices.

    Service Servers:
        ~/register_device (RegisterDevice): Register a new device.

    Published Topics:
        ~/device_status (DeviceTrustStatus): Device trust status updates.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'discovery_server',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._register_srv = None
        self._status_pub = None
        self._devices: Dict[str, Dict[str, Any]] = {}
        self._devices_lock = threading.Lock()
        self._max_devices: int = 100
        self._initial_trust: float = 0.5

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure discovery server."""
        params = self.declare_and_validate_parameters({
            'max_devices': {
                'default': 100,
                'type': int,
                'range': (1, 10000),
            },
            'initial_trust_score': {
                'default': 0.5,
                'type': float,
                'range': (0.0, 1.0),
            },
        })

        self._max_devices = params['max_devices']
        self._initial_trust = params['initial_trust_score']

        self._register_srv = self.create_service(
            RegisterDevice,
            'register_device',
            self._handle_register,
            callback_group=self.service_callback_group,
        )

        self._status_pub = self.create_publisher(
            DeviceTrustStatus,
            'device_status',
            QoSProfiles.state_data(),
        )

        self.get_logger().info("Discovery server configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate discovery server."""
        self.get_logger().info("Discovery server activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate discovery server."""
        return TransitionCallbackReturn.SUCCESS

    def _handle_register(
        self,
        request: RegisterDevice.Request,
        response: RegisterDevice.Response,
    ) -> RegisterDevice.Response:
        """Handle device registration requests."""
        with self._devices_lock:
            if len(self._devices) >= self._max_devices:
                response.success = False
                response.message = 'Device registry full'
                response.initial_trust_score = 0.0
                return response

            if request.device_id in self._devices:
                response.success = True
                response.message = 'Device already registered (updated)'
                response.initial_trust_score = self._devices[request.device_id]['trust']
            else:
                self._devices[request.device_id] = {
                    'type': request.device_type,
                    'capabilities': list(request.capabilities),
                    'firmware': request.firmware_version,
                    'trust': self._initial_trust,
                }
                response.success = True
                response.message = f'Device {request.device_id} registered'
                response.initial_trust_score = self._initial_trust

                self.get_logger().info(
                    f"Registered device: {request.device_id} "
                    f"(type={request.device_type})"
                )

            # Publish status
            status = DeviceTrustStatus()
            status.timestamp = self.get_clock().now().to_msg()
            status.device_id = request.device_id
            status.device_type = request.device_type
            status.trust_score = response.initial_trust_score
            status.attestation_status = 'PENDING'
            status.last_attestation = self.get_clock().now().to_msg()
            status.active_policies = []
            status.is_quarantined = False

            self._status_pub.publish(status)

        return response


def main(args=None):
    """Entry point for the discovery server node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = DiscoveryServerNode()
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
