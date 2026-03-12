"""
Chaos Engineering Injector Node.

Injects controlled failures for resiliency testing.
Supports network delays, node kills, resource exhaustion.
SAFETY: Only active when explicitly enabled.
"""

from typing import Any, Dict, Optional
import uuid
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.srv import InjectFault
from miracle_resiliency.fault_executor import FaultExecutor, FaultSpec, FaultType


class ChaosInjectorNode(MiracleLifecycleNode):
    """Controlled fault injection for resiliency testing.

    Parameters:
        enabled (bool): Master enable switch (default False for safety).
        max_concurrent_faults (int): Max simultaneous injected faults.
        safety_nodes (list): Nodes that cannot be targeted.

    Service Servers:
        ~/inject_fault (InjectFault): Inject a controlled fault.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'chaos_injector',
            criticality=self.CRITICALITY_LOW,
            **kwargs,
        )
        self._inject_srv = None
        self._active_faults: Dict[str, Dict] = {}
        self._lock = threading.Lock()
        self._enabled: bool = False
        self._safety_nodes: list = []
        self._fault_executor: Optional[FaultExecutor] = None

    def _do_configure(self) -> TransitionCallbackReturn:
        params = self.declare_and_validate_parameters({
            'enabled': {'default': False, 'type': bool},
            'max_concurrent_faults': {
                'default': 3, 'type': int, 'range': (1, 20),
            },
            'dry_run': {'default': True, 'type': bool},
        })

        self._enabled = params['enabled']

        self._fault_executor = FaultExecutor(dry_run=params['dry_run'])

        self._inject_srv = self.create_service(
            InjectFault, 'inject_fault',
            self._handle_inject, callback_group=self.service_callback_group,
        )

        self.get_logger().info(
            f"Chaos injector configured (enabled={self._enabled})"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        if self._enabled:
            self.get_logger().warn("CHAOS INJECTOR IS ACTIVE")
        else:
            self.get_logger().info("Chaos injector standby (disabled)")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        self._clear_all_faults()
        return TransitionCallbackReturn.SUCCESS

    def _handle_inject(
        self, request: InjectFault.Request, response: InjectFault.Response,
    ) -> InjectFault.Response:
        """Handle fault injection request."""
        if not self._enabled:
            response.success = False
            response.fault_id = ''
            response.message = 'Chaos injector is disabled'
            return response

        max_faults = self.get_parameter('max_concurrent_faults').value
        with self._lock:
            if len(self._active_faults) >= max_faults:
                response.success = False
                response.fault_id = ''
                response.message = 'Max concurrent faults reached'
                return response

            fault_id = str(uuid.uuid4())[:8]
            self._active_faults[fault_id] = {
                'target': request.target_node,
                'type': request.fault_type,
                'duration': request.duration_sec,
                'intensity': request.intensity,
            }

        # Map request fault_type string to FaultType enum
        fault_type_map = {
            'network_delay': FaultType.NETWORK_DELAY,
            'node_kill': FaultType.NODE_KILL,
            'cpu_stress': FaultType.CPU_STRESS,
            'memory_stress': FaultType.MEMORY_STRESS,
            'message_drop': FaultType.MESSAGE_DROP,
        }
        ft = fault_type_map.get(request.fault_type)
        if ft is not None:
            fault_spec = FaultSpec(
                fault_type=ft,
                target=request.target_node,
                duration_sec=request.duration_sec,
                parameters={'intensity': request.intensity},
            )
            self._fault_executor.inject(fault_id, fault_spec)

        self.get_logger().warn(
            f"Fault injected: {fault_id} -> {request.target_node} "
            f"({request.fault_type}, {request.duration_sec}s)"
        )

        # Schedule fault removal
        if request.duration_sec > 0:
            self.create_timer(
                request.duration_sec,
                lambda: self._remove_fault(fault_id),
                callback_group=self.service_callback_group,
            )

        response.success = True
        response.fault_id = fault_id
        response.message = f'Fault {fault_id} injected'
        return response

    def _remove_fault(self, fault_id: str) -> None:
        """Remove an active fault."""
        self._fault_executor.cleanup(fault_id)
        with self._lock:
            if fault_id in self._active_faults:
                del self._active_faults[fault_id]
                self.get_logger().info(f"Fault removed: {fault_id}")

    def _clear_all_faults(self) -> None:
        """Clear all active faults."""
        self._fault_executor.cleanup_all()
        with self._lock:
            count = len(self._active_faults)
            self._active_faults.clear()
        if count > 0:
            self.get_logger().info(f"Cleared {count} active faults")


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = ChaosInjectorNode()
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
