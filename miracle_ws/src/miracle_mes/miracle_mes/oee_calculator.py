"""
OEE (Overall Equipment Effectiveness) Calculator Node.

Computes OEE and related KPIs in real-time from machine state data.
OEE = Availability x Performance x Quality
"""

from typing import Any, Dict
from dataclasses import dataclass
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, SystemKPIs, JobStatus


@dataclass
class MachineMetrics:
    """Accumulated metrics for OEE calculation."""
    total_time_sec: float = 0.0
    running_time_sec: float = 0.0
    planned_time_sec: float = 0.0
    ideal_cycle_time_sec: float = 60.0
    actual_output: int = 0
    good_output: int = 0
    total_output: int = 0
    downtime_sec: float = 0.0
    last_status: str = 'IDLE'
    last_update: float = 0.0


class OEECalculatorNode(MiracleLifecycleNode):
    """Computes OEE and system KPIs in real-time.

    Parameters:
        calculation_interval_sec (float): KPI calculation interval.
        shift_duration_hours (float): Work shift duration.
        ideal_cycle_time_sec (float): Ideal cycle time per part.

    Subscribed Topics:
        /miracle/+/state (MachineState): Machine states.
        /miracle/+/job_status (JobStatus): Job completions.

    Published Topics:
        /miracle/system_kpis (SystemKPIs): Computed KPIs.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'oee_calculator',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._metrics: Dict[str, MachineMetrics] = {}
        self._metrics_lock = threading.Lock()
        self._state_sub = None
        self._job_sub = None
        self._kpi_pub = None
        self._calc_timer = None
        self._jobs_completed_today: int = 0
        self._jobs_in_progress: int = 0

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure OEE calculator."""
        params = self.declare_and_validate_parameters({
            'calculation_interval_sec': {
                'default': 5.0,
                'type': float,
                'range': (1.0, 60.0),
            },
            'shift_duration_hours': {
                'default': 8.0,
                'type': float,
                'range': (1.0, 24.0),
            },
            'ideal_cycle_time_sec': {
                'default': 60.0,
                'type': float,
                'range': (1.0, 3600.0),
            },
        })

        self._state_sub = self.create_subscription(
            MachineState,
            '/miracle/+/state',
            self._on_machine_state,
            QoSProfiles.state_data(),
        )

        self._job_sub = self.create_subscription(
            JobStatus,
            '/miracle/+/job_status',
            self._on_job_status,
            QoSProfiles.state_data(),
        )

        self._kpi_pub = self.create_publisher(
            SystemKPIs,
            '/miracle/system_kpis',
            QoSProfiles.state_data(),
        )

        self.get_logger().info("OEE calculator configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate OEE calculation."""
        interval = self.get_parameter('calculation_interval_sec').value
        self._calc_timer = self.create_timer(
            interval,
            self._calculate_kpis,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("OEE calculator activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate OEE calculation."""
        if self._calc_timer is not None:
            self._calc_timer.cancel()
            self._calc_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_machine_state(self, msg: MachineState) -> None:
        """Update machine metrics from state."""
        now = self.get_clock().now().nanoseconds / 1e9

        with self._metrics_lock:
            if msg.machine_id not in self._metrics:
                self._metrics[msg.machine_id] = MachineMetrics(
                    last_update=now,
                    ideal_cycle_time_sec=self.get_parameter(
                        'ideal_cycle_time_sec'
                    ).value,
                )

            m = self._metrics[msg.machine_id]
            dt = now - m.last_update if m.last_update > 0 else 0.0
            m.last_update = now

            m.total_time_sec += dt

            if msg.status == 'RUNNING':
                m.running_time_sec += dt
                m.planned_time_sec += dt
            elif msg.status in ('IDLE', 'PAUSED'):
                m.planned_time_sec += dt
            elif msg.status == 'ERROR':
                m.downtime_sec += dt
                m.planned_time_sec += dt

            m.last_status = msg.status

    def _on_job_status(self, msg: JobStatus) -> None:
        """Track job completions."""
        if msg.status == 'COMPLETED':
            self._jobs_completed_today += 1
            with self._metrics_lock:
                if msg.machine_id in self._metrics:
                    self._metrics[msg.machine_id].total_output += 1
                    self._metrics[msg.machine_id].good_output += 1

        if msg.status == 'RUNNING':
            self._jobs_in_progress += 1
        elif msg.status in ('COMPLETED', 'FAILED', 'CANCELLED'):
            self._jobs_in_progress = max(0, self._jobs_in_progress - 1)

    def _calculate_kpis(self) -> None:
        """Calculate and publish system KPIs."""
        with self._metrics_lock:
            if not self._metrics:
                return

            # Aggregate across all machines
            total_availability = 0.0
            total_performance = 0.0
            total_quality = 0.0
            count = 0

            for m in self._metrics.values():
                if m.planned_time_sec <= 0:
                    continue

                count += 1

                # Availability = Running Time / Planned Time
                availability = m.running_time_sec / m.planned_time_sec
                total_availability += min(availability, 1.0)

                # Performance = (Ideal Cycle Time * Output) / Running Time
                if m.running_time_sec > 0 and m.total_output > 0:
                    performance = (
                        m.ideal_cycle_time_sec * m.total_output
                        / m.running_time_sec
                    )
                else:
                    performance = 0.0
                total_performance += min(performance, 1.0)

                # Quality = Good Output / Total Output
                if m.total_output > 0:
                    quality = m.good_output / m.total_output
                else:
                    quality = 1.0
                total_quality += min(quality, 1.0)

            if count == 0:
                return

            avg_avail = total_availability / count
            avg_perf = total_performance / count
            avg_qual = total_quality / count
            oee = avg_avail * avg_perf * avg_qual

        msg = SystemKPIs()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.oee = oee
        msg.availability = avg_avail
        msg.performance = avg_perf
        msg.quality = avg_qual
        msg.cpk = 1.33  # Placeholder
        msg.mtbf = 500.0  # Placeholder hours
        msg.mttr = 30.0  # Placeholder minutes
        msg.energy_efficiency = 0.75
        msg.schedule_adherence = 0.90
        msg.tool_life_utilization = 0.80
        msg.jobs_completed_today = self._jobs_completed_today
        msg.jobs_in_progress = self._jobs_in_progress
        msg.jobs_queued = 0

        self._kpi_pub.publish(msg)


def main(args=None):
    """Entry point for the OEE calculator node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = OEECalculatorNode()
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
