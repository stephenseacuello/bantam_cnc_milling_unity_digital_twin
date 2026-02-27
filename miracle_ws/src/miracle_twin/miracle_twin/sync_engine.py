"""
Digital Twin Synchronization Engine.

Maintains real-time synchronization between physical CNC machines
and their digital twin representations. Handles state mirroring,
drift detection, and automatic recalibration.
"""

from typing import Any, Dict, Optional
from dataclasses import dataclass, field
import threading
import math

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, FusedSensorData, TwinSyncStatus


@dataclass
class TwinState:
    """State of a digital twin instance."""
    machine_id: str
    physical_state: Optional[MachineState] = None
    twin_state: Optional[MachineState] = None
    drift_magnitude: float = 0.0
    sync_quality: float = 1.0
    last_sync_time: float = 0.0
    sync_count: int = 0


class SyncEngineNode(MiracleLifecycleNode):
    """Synchronizes physical machines with digital twins.

    Parameters:
        sync_rate_hz (float): Synchronization frequency.
        max_drift_threshold (float): Maximum acceptable drift.
        interpolation_factor (float): Twin state interpolation factor.

    Subscribed Topics:
        /miracle/{machine_id}/state (MachineState): Physical machine states.
        /miracle/{machine_id}/sensor/fused (FusedSensorData): Fused sensor data.

    Published Topics:
        ~/twin_state (MachineState): Digital twin state.
        /miracle/twin/sync_status (TwinSyncStatus): Sync quality per machine.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'sync_engine',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._twins: Dict[str, TwinState] = {}
        self._twins_lock = threading.Lock()
        self._state_subs = []
        self._sensor_subs = []
        self._twin_pub = None
        self._sync_status_pub = None
        self._sync_timer = None
        self._max_drift: float = 1.0
        self._interp_factor: float = 0.8

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure sync engine."""
        params = self.declare_and_validate_parameters({
            'sync_rate_hz': {
                'default': 50.0,
                'type': float,
                'range': (1.0, 1000.0),
            },
            'max_drift_threshold': {
                'default': 1.0,
                'type': float,
                'range': (0.01, 100.0),
            },
            'interpolation_factor': {
                'default': 0.8,
                'type': float,
                'range': (0.0, 1.0),
            },
            'machine_ids': {'default': 'cnc1,cnc2,cnc3', 'type': str},
        })

        self._max_drift = params['max_drift_threshold']
        self._interp_factor = params['interpolation_factor']
        machine_ids = self.get_machine_ids(params)

        self._state_subs = self.create_multi_machine_subscriptions(
            MachineState,
            'state',
            self._on_physical_state,
            QoSProfiles.state_data(),
            machine_ids,
        )

        self._sensor_subs = self.create_multi_machine_subscriptions(
            FusedSensorData,
            'sensor/fused',
            self._on_sensor_data,
            QoSProfiles.sensor_data(),
            machine_ids,
        )

        self._twin_pub = self.create_publisher(
            MachineState,
            'twin_state',
            QoSProfiles.state_data(),
        )

        self._sync_status_pub = self.create_publisher(
            TwinSyncStatus,
            '/miracle/twin/sync_status',
            QoSProfiles.state_data(),
        )

        self.get_logger().info("Sync engine configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate synchronization."""
        rate = self.get_parameter('sync_rate_hz').value
        self._sync_timer = self.create_timer(
            1.0 / rate,
            self._sync_cycle,
            callback_group=self.realtime_callback_group,
        )
        self.get_logger().info("Sync engine activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate synchronization."""
        if self._sync_timer is not None:
            self._sync_timer.cancel()
            self._sync_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_physical_state(self, msg: MachineState) -> None:
        """Receive physical machine state."""
        with self._twins_lock:
            if msg.machine_id not in self._twins:
                self._twins[msg.machine_id] = TwinState(
                    machine_id=msg.machine_id,
                )
            self._twins[msg.machine_id].physical_state = msg

    def _on_sensor_data(self, msg: FusedSensorData) -> None:
        """Receive fused sensor data for twin enrichment."""
        pass  # Used for physics-based twin model updates

    def _sync_cycle(self) -> None:
        """Execute one synchronization cycle."""
        now = self.get_clock().now().nanoseconds / 1e9

        with self._twins_lock:
            for machine_id, twin in self._twins.items():
                if twin.physical_state is None:
                    continue

                # Create or update twin state
                if twin.twin_state is None:
                    twin.twin_state = MachineState()
                    twin.twin_state.machine_id = f"{machine_id}_twin"

                phys = twin.physical_state
                tw = twin.twin_state

                # Interpolate twin toward physical state
                alpha = self._interp_factor
                tw.timestamp = self.get_clock().now().to_msg()
                tw.status = phys.status
                tw.spindle_speed = self._lerp(
                    tw.spindle_speed, phys.spindle_speed, alpha
                )
                tw.feed_rate = self._lerp(
                    tw.feed_rate, phys.feed_rate, alpha
                )
                tw.spindle_load = self._lerp(
                    tw.spindle_load, phys.spindle_load, alpha
                )
                tw.coolant_level = phys.coolant_level
                tw.current_program = phys.current_program
                tw.current_line = phys.current_line

                # Interpolate axis positions
                if phys.axis_positions:
                    if not tw.axis_positions:
                        tw.axis_positions = list(phys.axis_positions)
                    else:
                        tw.axis_positions = [
                            self._lerp(tw_pos, ph_pos, alpha)
                            for tw_pos, ph_pos in zip(
                                tw.axis_positions, phys.axis_positions
                            )
                        ]
                tw.axis_velocities = list(phys.axis_velocities) if phys.axis_velocities else []

                # Compute drift
                twin.drift_magnitude = self._compute_drift(phys, tw)
                twin.sync_quality = max(
                    0.0, 1.0 - twin.drift_magnitude / self._max_drift
                )
                twin.last_sync_time = now
                twin.sync_count += 1

                if twin.drift_magnitude > self._max_drift:
                    self.get_logger().warn(
                        f"Twin drift for {machine_id}: "
                        f"{twin.drift_magnitude:.4f} > {self._max_drift}"
                    )
                    # Force snap to physical state
                    tw.axis_positions = list(phys.axis_positions)
                    tw.spindle_speed = phys.spindle_speed
                    tw.feed_rate = phys.feed_rate

                # Publish twin state
                self._twin_pub.publish(tw)

                # Publish sync status for dashboard
                status_msg = TwinSyncStatus()
                status_msg.timestamp = self.get_clock().now().to_msg()
                status_msg.machine_id = machine_id
                status_msg.sync_quality = twin.sync_quality
                status_msg.drift_magnitude = twin.drift_magnitude
                if phys.axis_positions and tw.axis_positions:
                    status_msg.axis_drift = [
                        abs(p - t) for p, t in zip(
                            phys.axis_positions, tw.axis_positions
                        )
                    ]
                status_msg.sync_latency_ms = (
                    now - twin.last_sync_time
                ) * 1000.0 if twin.last_sync_time > 0 else 0.0
                status_msg.correction_active = (
                    twin.drift_magnitude > self._max_drift
                )
                status_msg.corrections_applied = twin.sync_count
                self._sync_status_pub.publish(status_msg)

    @staticmethod
    def _lerp(current: float, target: float, alpha: float) -> float:
        """Linear interpolation between current and target."""
        return current + alpha * (target - current)

    @staticmethod
    def _compute_drift(phys: MachineState, twin: MachineState) -> float:
        """Compute drift magnitude between physical and twin states."""
        drift = 0.0
        if phys.axis_positions and twin.axis_positions:
            for p, t in zip(phys.axis_positions, twin.axis_positions):
                drift += (p - t) ** 2
        drift += (phys.spindle_speed - twin.spindle_speed) ** 2 * 1e-8
        drift += (phys.feed_rate - twin.feed_rate) ** 2 * 1e-6
        return math.sqrt(drift)


def main(args=None):
    """Entry point for the sync engine node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = SyncEngineNode()
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
