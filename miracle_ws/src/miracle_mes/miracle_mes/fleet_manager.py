"""
Fleet Manager Node.

Manages the fleet of CNC machines. Tracks capabilities, availability,
maintenance schedules, and provides fleet-level status reporting.
"""

from typing import Any, Dict, Optional
from dataclasses import dataclass, field
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, FleetHealth
from miracle_msgs.srv import GetFleetStatus


@dataclass
class MachineInfo:
    """Information about a machine in the fleet."""
    machine_id: str
    capabilities: list = field(default_factory=list)
    status: str = 'UNKNOWN'
    current_job: str = ''
    total_runtime_hours: float = 0.0
    jobs_completed: int = 0
    last_maintenance: float = 0.0


class FleetManagerNode(MiracleLifecycleNode):
    """Fleet management with capability tracking.

    Parameters:
        fleet_size (int): Expected number of machines.
        status_publish_rate_hz (float): Fleet status publish rate.

    Subscribed Topics:
        /miracle/{machine_id}/state (MachineState): Machine states.

    Published Topics:
        ~/fleet_health (FleetHealth): Aggregated fleet health.

    Service Servers:
        ~/get_fleet_status (GetFleetStatus): Query fleet status.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'fleet_manager',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._machines: Dict[str, MachineInfo] = {}
        self._machines_lock = threading.Lock()
        self._state_subs = None
        self._health_pub = None
        self._status_srv = None
        self._publish_timer = None

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure fleet manager."""
        params = self.declare_and_validate_parameters({
            'fleet_size': {
                'default': 3,
                'type': int,
                'range': (1, 100),
            },
            'status_publish_rate_hz': {
                'default': 1.0,
                'type': float,
                'range': (0.1, 10.0),
            },
            'machine_ids': {
                'default': 'cnc1,cnc2,cnc3',
                'type': str,
            },
        })

        machine_ids = self.get_machine_ids(params)

        self._state_subs = self.create_multi_machine_subscriptions(
            MachineState,
            'state',
            self._on_machine_state,
            QoSProfiles.state_data(),
            machine_ids,
        )

        self._health_pub = self.create_publisher(
            FleetHealth,
            'fleet_health',
            QoSProfiles.state_data(),
        )

        self._status_srv = self.create_service(
            GetFleetStatus,
            'get_fleet_status',
            self._handle_fleet_status,
            callback_group=self.service_callback_group,
        )

        self.get_logger().info("Fleet manager configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate fleet monitoring."""
        rate = self.get_parameter('status_publish_rate_hz').value
        self._publish_timer = self.create_timer(
            1.0 / rate,
            self._publish_fleet_health,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("Fleet manager activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate fleet monitoring."""
        if self._publish_timer is not None:
            self._publish_timer.cancel()
            self._publish_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_machine_state(self, msg: MachineState) -> None:
        """Update machine info from state message."""
        with self._machines_lock:
            if msg.machine_id not in self._machines:
                self._machines[msg.machine_id] = MachineInfo(
                    machine_id=msg.machine_id,
                )
                self.get_logger().info(
                    f"New machine discovered: {msg.machine_id}"
                )

            machine = self._machines[msg.machine_id]
            machine.status = msg.status
            machine.current_job = msg.current_program

    def _publish_fleet_health(self) -> None:
        """Publish aggregated fleet health."""
        with self._machines_lock:
            msg = FleetHealth()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.total_nodes = len(self._machines)
            msg.healthy_nodes = sum(
                1 for m in self._machines.values()
                if m.status in ('IDLE', 'RUNNING', 'READY')
            )
            msg.degraded_nodes = sum(
                1 for m in self._machines.values()
                if m.status in ('PAUSED', 'MAINTENANCE')
            )
            msg.failed_nodes = sum(
                1 for m in self._machines.values()
                if m.status in ('ERROR', 'UNKNOWN')
            )
            msg.failed_node_names = [
                m.machine_id for m in self._machines.values()
                if m.status in ('ERROR', 'UNKNOWN')
            ]
            msg.degraded_node_names = [
                m.machine_id for m in self._machines.values()
                if m.status in ('PAUSED', 'MAINTENANCE')
            ]

            total = msg.total_nodes or 1
            msg.health_score = msg.healthy_nodes / total
            msg.critical_healthy = msg.healthy_nodes
            msg.critical_total = msg.total_nodes

        self._health_pub.publish(msg)

    def _handle_fleet_status(
        self,
        request: GetFleetStatus.Request,
        response: GetFleetStatus.Response,
    ) -> GetFleetStatus.Response:
        """Handle fleet status queries."""
        import json

        with self._machines_lock:
            msg = FleetHealth()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.total_nodes = len(self._machines)
            msg.healthy_nodes = sum(
                1 for m in self._machines.values()
                if m.status not in ('ERROR', 'UNKNOWN')
            )
            msg.failed_nodes = msg.total_nodes - msg.healthy_nodes
            msg.health_score = msg.healthy_nodes / max(msg.total_nodes, 1)

            response.fleet_health = msg

            details = []
            for mid, machine in self._machines.items():
                if request.filter_state and machine.status != request.filter_state:
                    continue
                details.append(json.dumps({
                    'machine_id': mid,
                    'status': machine.status,
                    'current_job': machine.current_job,
                    'jobs_completed': machine.jobs_completed,
                }))

            response.node_details_json = details

        return response


def main(args=None):
    """Entry point for the fleet manager node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = FleetManagerNode()
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
