"""
KPI Calculator Node.

Computes Overall Equipment Effectiveness (OEE) and related manufacturing
KPIs across all machines in the MIRACLE cell.  Publishes SystemKPIs at a
configurable interval (default 10 s).

OEE = Availability x Performance x Quality

Availability = (Planned Production Time - Downtime) / Planned Production Time
Performance  = (Ideal Cycle Time x Total Count) / Run Time
Quality      = Good Count / Total Count
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
import math
import threading
import time

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, JobStatus, AnomalyAlert, SystemKPIs


# ---------------------------------------------------------------------------
# OEE Trending Dataclasses
# ---------------------------------------------------------------------------

@dataclass
class OEESnapshot:
    """A point-in-time OEE measurement."""
    timestamp: float
    oee: float
    availability: float
    performance: float
    quality: float
    machine_id: str = 'fleet'  # 'fleet' for aggregate


@dataclass
class OEETrendAnalysis:
    """Trend analysis result for OEE metrics."""
    metric: str           # 'oee', 'availability', 'performance', 'quality'
    trend_direction: str  # 'improving', 'stable', 'degrading'
    slope_per_hour: float  # change rate per hour
    current_value: float
    avg_1h: float         # average over last hour
    avg_8h: float         # average over last 8 hours (shift)
    min_value: float
    max_value: float
    samples: int


# Maximum number of snapshots to retain (24 hours at 30s interval)
_MAX_OEE_HISTORY = 2880


# Machine states derived from MachineState.status strings
_STATE_RUNNING = 'RUNNING'
_STATE_IDLE = 'IDLE'
_STATE_ERROR = 'ERROR'
_STATE_MAINTENANCE = 'MAINTENANCE'

_DOWNTIME_STATES = {_STATE_ERROR, _STATE_MAINTENANCE}

# Anomaly types treated as quality defects
_QUALITY_ANOMALY_TYPES = {
    'surface_quality_anomaly',
    'surface_quality',
    'dimensional',
    'dimensional_deviation',
    'tool_wear_excessive',
}


@dataclass
class MachineMetrics:
    """Per-machine time and count accumulators."""

    # Time tracking (seconds)
    planned_production_time: float = 0.0
    run_time: float = 0.0
    downtime: float = 0.0
    idle_time: float = 0.0
    repair_time: float = 0.0

    # Count tracking
    total_jobs: int = 0
    good_jobs: int = 0
    defect_jobs: int = 0
    jobs_in_progress: int = 0
    jobs_queued: int = 0

    # Cycle time tracking
    ideal_cycle_time: float = 60.0  # configurable default (seconds)
    total_actual_cycle_time: float = 0.0

    # Failure tracking
    failure_count: int = 0
    total_repair_time: float = 0.0

    # State tracking
    last_state: str = _STATE_IDLE
    last_state_change: float = 0.0

    # Tool life tracking (0-1, fraction of life consumed)
    tool_life_used: float = 0.0
    tool_life_total: float = 1.0

    # Schedule tracking
    jobs_on_time: int = 0
    jobs_scheduled: int = 0

    # Track defect job IDs to avoid double-counting
    defect_job_ids: set = field(default_factory=set)
    completed_job_ids: set = field(default_factory=set)


class KPICalculatorNode(MiracleLifecycleNode):
    """Calculates OEE and publishes SystemKPIs for the manufacturing cell.

    Parameters:
        publish_interval_sec (float): How often to publish KPIs.
        ideal_cycle_time_sec (float): Ideal cycle time per job.
        planned_hours_per_day (float): Planned production hours per day.
        machine_ids (str): Comma-separated machine IDs.

    Subscribed Topics:
        /miracle/{machine_id}/state (MachineState): Machine state updates.
        /miracle/{machine_id}/job_status (JobStatus): Job progress updates.
        /miracle/{machine_id}/anomaly (AnomalyAlert): Quality/anomaly alerts.

    Published Topics:
        /miracle/scada/system_kpis (SystemKPIs): Aggregated KPIs.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'kpi_calculator',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._lock = threading.Lock()
        self._machines: Dict[str, MachineMetrics] = {}
        self._state_subs: Optional[List] = None
        self._job_subs: Optional[List] = None
        self._anomaly_subs: Optional[List] = None
        self._kpi_pub = None
        self._publish_timer = None
        self._trend_timer = None
        self._publish_interval: float = 10.0
        self._ideal_cycle_time: float = 60.0
        self._planned_hours: float = 24.0

        # OEE history and trending
        self._oee_history: List[OEESnapshot] = []
        self._machine_oee_history: Dict[str, List[OEESnapshot]] = {}
        self._trend_interval_sec: float = 300.0  # 5 minutes
        self._degradation_threshold: float = -0.02  # slope per hour
        self._improving_threshold: float = 0.01    # slope per hour

        # MTBF/MTTR real tracking
        self._failure_timestamps: List[float] = []
        self._repair_durations: List[float] = []

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure subscriptions and publisher."""
        params = self.declare_and_validate_parameters({
            'publish_interval_sec': {
                'default': 10.0,
                'type': float,
                'range': (1.0, 300.0),
            },
            'ideal_cycle_time_sec': {
                'default': 60.0,
                'type': float,
                'range': (1.0, 86400.0),
            },
            'planned_hours_per_day': {
                'default': 24.0,
                'type': float,
                'range': (0.1, 24.0),
            },
            'machine_ids': {
                'default': 'cnc1,cnc2,cnc3',
                'type': str,
            },
            'trend_interval_sec': {
                'default': 300.0,
                'type': float,
                'range': (10.0, 3600.0),
            },
            'degradation_threshold': {
                'default': -0.02,
                'type': float,
                'range': (-1.0, 0.0),
            },
        })

        machine_ids = self.get_machine_ids(params)
        self._publish_interval = params['publish_interval_sec']
        self._ideal_cycle_time = params['ideal_cycle_time_sec']
        self._planned_hours = params['planned_hours_per_day']
        self._trend_interval_sec = params.get('trend_interval_sec', 300.0)
        self._degradation_threshold = params.get('degradation_threshold', -0.02)

        # Initialise per-machine metrics
        with self._lock:
            for mid in machine_ids:
                self._machines[mid] = MachineMetrics(
                    ideal_cycle_time=self._ideal_cycle_time,
                )

        # Subscriptions
        self._state_subs = self.create_multi_machine_subscriptions(
            MachineState,
            'state',
            self._on_machine_state,
            QoSProfiles.state_data(),
            machine_ids,
        )

        self._job_subs = self.create_multi_machine_subscriptions(
            JobStatus,
            'job_status',
            self._on_job_status,
            QoSProfiles.state_data(),
            machine_ids,
        )

        self._anomaly_subs = self.create_multi_machine_subscriptions(
            AnomalyAlert,
            'anomaly',
            self._on_anomaly,
            QoSProfiles.alert(),
            machine_ids,
        )

        # Publisher
        self._kpi_pub = self.create_publisher(
            SystemKPIs,
            '/miracle/scada/system_kpis',
            QoSProfiles.state_data(),
        )

        self.get_logger().info("KPI calculator configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Start periodic KPI publishing."""
        now = self._now_sec()
        with self._lock:
            for m in self._machines.values():
                m.last_state_change = now

        self._publish_timer = self.create_timer(
            self._publish_interval,
            self._publish_kpis,
            callback_group=self.service_callback_group,
        )
        self._trend_timer = self.create_timer(
            self._trend_interval_sec,
            self._on_trend_timer,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("KPI calculator activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Stop periodic publishing."""
        if self._publish_timer is not None:
            self._publish_timer.cancel()
            self._publish_timer = None
        if self._trend_timer is not None:
            self._trend_timer.cancel()
            self._trend_timer = None
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_machine_state(self, msg: MachineState) -> None:
        """Track time in each machine state."""
        mid = msg.machine_id
        now = self._now_sec()
        status = msg.status.upper() if msg.status else _STATE_IDLE

        with self._lock:
            m = self._machines.get(mid)
            if m is None:
                m = MachineMetrics(ideal_cycle_time=self._ideal_cycle_time)
                self._machines[mid] = m

            elapsed = max(0.0, now - m.last_state_change)
            self._accumulate_state_time(m, m.last_state, elapsed)

            # Detect transitions into/out of ERROR for failure counting
            if status == _STATE_ERROR and m.last_state != _STATE_ERROR:
                m.failure_count += 1
            if m.last_state == _STATE_ERROR and status != _STATE_ERROR:
                m.total_repair_time += elapsed

            m.last_state = status
            m.last_state_change = now

    def _on_job_status(self, msg: JobStatus) -> None:
        """Track job counts and cycle times."""
        mid = msg.machine_id
        status = msg.status.upper() if msg.status else ''

        with self._lock:
            m = self._machines.get(mid)
            if m is None:
                m = MachineMetrics(ideal_cycle_time=self._ideal_cycle_time)
                self._machines[mid] = m

            if status == 'COMPLETED' and msg.job_id not in m.completed_job_ids:
                m.completed_job_ids.add(msg.job_id)
                m.total_jobs += 1
                m.total_actual_cycle_time += msg.elapsed_sec
                # Count as good unless already flagged as defective
                if msg.job_id not in m.defect_job_ids:
                    m.good_jobs += 1
                # Schedule adherence: on-time if no errors
                m.jobs_scheduled += 1
                if not msg.errors:
                    m.jobs_on_time += 1
            elif status == 'IN_PROGRESS':
                m.jobs_in_progress = max(1, m.jobs_in_progress)
            elif status == 'QUEUED':
                m.jobs_queued += 1

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        """Track quality-related anomalies as defects."""
        mid = msg.machine_id
        anomaly = msg.anomaly_type.lower() if msg.anomaly_type else ''

        with self._lock:
            m = self._machines.get(mid)
            if m is None:
                m = MachineMetrics(ideal_cycle_time=self._ideal_cycle_time)
                self._machines[mid] = m

            if anomaly in _QUALITY_ANOMALY_TYPES:
                m.defect_jobs += 1
                # If a job is currently being tracked, mark it as defective
                # We use a synthetic ID based on the defect count
                defect_id = f"defect_{mid}_{m.defect_jobs}"
                m.defect_job_ids.add(defect_id)
                # Decrement good_jobs if we have completed jobs
                if m.good_jobs > 0:
                    m.good_jobs -= 1

    # ------------------------------------------------------------------
    # KPI Calculation
    # ------------------------------------------------------------------

    def _publish_kpis(self) -> None:
        """Compute aggregate KPIs and publish."""
        msg = self._compute_fleet_kpis()
        if self._kpi_pub is not None:
            self._kpi_pub.publish(msg)

        # Record fleet OEE snapshot
        now = self._now_sec()
        snapshot = OEESnapshot(
            timestamp=now,
            oee=msg.oee,
            availability=msg.availability,
            performance=msg.performance,
            quality=msg.quality,
            machine_id='fleet',
        )
        self._append_snapshot(snapshot)

    def _compute_fleet_kpis(self) -> SystemKPIs:
        """Aggregate KPIs across all machines."""
        msg = SystemKPIs()
        now = self._now_sec()
        msg.timestamp.sec = int(now)
        msg.timestamp.nanosec = int((now - int(now)) * 1e9)

        with self._lock:
            machines = list(self._machines.values())

        if not machines:
            return msg

        # Flush pending state time
        with self._lock:
            for m in machines:
                elapsed = max(0.0, now - m.last_state_change)
                self._accumulate_state_time(m, m.last_state, elapsed)
                m.last_state_change = now

        # Aggregate across fleet
        total_planned = sum(m.planned_production_time for m in machines)
        total_downtime = sum(m.downtime for m in machines)
        total_run = sum(m.run_time for m in machines)
        total_jobs = sum(m.total_jobs for m in machines)
        total_good = sum(m.good_jobs for m in machines)
        total_failures = sum(m.failure_count for m in machines)
        total_repair = sum(m.total_repair_time for m in machines)
        total_on_time = sum(m.jobs_on_time for m in machines)
        total_scheduled = sum(m.jobs_scheduled for m in machines)
        jobs_in_progress = sum(m.jobs_in_progress for m in machines)
        jobs_queued = sum(m.jobs_queued for m in machines)

        # OEE components
        availability = compute_availability(total_planned, total_downtime)
        performance = compute_performance(
            self._ideal_cycle_time, total_jobs, total_run,
        )
        quality = compute_quality(total_good, total_jobs)
        oee = compute_oee(availability, performance, quality)

        # Reliability
        mtbf = compute_mtbf(total_run, total_failures)
        mttr = compute_mttr(total_repair, total_failures)

        # Schedule adherence
        schedule_adherence = (
            total_on_time / total_scheduled if total_scheduled > 0 else 1.0
        )

        # Tool life utilization (average across fleet)
        tool_utils = [
            m.tool_life_used / m.tool_life_total
            for m in machines
            if m.tool_life_total > 0
        ]
        tool_life_util = (
            sum(tool_utils) / len(tool_utils) if tool_utils else 0.0
        )

        # Populate message
        msg.oee = oee
        msg.availability = availability
        msg.performance = performance
        msg.quality = quality
        msg.cpk = 0.0  # requires SPC data not yet available
        msg.mtbf = mtbf
        msg.mttr = mttr
        msg.energy_efficiency = 0.0  # requires energy monitoring
        msg.schedule_adherence = schedule_adherence
        msg.tool_life_utilization = tool_life_util
        msg.jobs_completed_today = total_jobs
        msg.jobs_in_progress = jobs_in_progress
        msg.jobs_queued = jobs_queued

        return msg

    # ------------------------------------------------------------------
    # OEE History & Trending
    # ------------------------------------------------------------------

    def _append_snapshot(self, snapshot: OEESnapshot) -> None:
        """Append a snapshot to the appropriate history list, capping size."""
        with self._lock:
            if snapshot.machine_id == 'fleet':
                self._oee_history.append(snapshot)
                if len(self._oee_history) > _MAX_OEE_HISTORY:
                    self._oee_history = self._oee_history[-_MAX_OEE_HISTORY:]
            else:
                hist = self._machine_oee_history.setdefault(
                    snapshot.machine_id, [],
                )
                hist.append(snapshot)
                if len(hist) > _MAX_OEE_HISTORY:
                    self._machine_oee_history[snapshot.machine_id] = \
                        hist[-_MAX_OEE_HISTORY:]

    def _on_trend_timer(self) -> None:
        """Periodic callback to run trend analysis and log degradation."""
        trends = self.get_trend_analysis()
        for metric, analysis in trends.items():
            if analysis.trend_direction == 'degrading':
                self.get_logger().warning(
                    f"OEE DEGRADATION: {metric} trending down at "
                    f"{analysis.slope_per_hour:+.4f}/hr "
                    f"(current={analysis.current_value:.3f}, "
                    f"1h_avg={analysis.avg_1h:.3f})"
                )

    @property
    def oee_history(self) -> List[OEESnapshot]:
        """Return fleet OEE history (up to 24 hours)."""
        with self._lock:
            return list(self._oee_history)

    def get_oee_history(self, hours: float = 24.0) -> List[OEESnapshot]:
        """Return fleet OEE snapshots for the last *hours* hours."""
        cutoff = self._now_sec() - hours * 3600.0
        with self._lock:
            return [s for s in self._oee_history if s.timestamp >= cutoff]

    def get_trend_analysis(self) -> Dict[str, OEETrendAnalysis]:
        """Compute trend analysis for each OEE metric from fleet history."""
        with self._lock:
            snapshots = list(self._oee_history)
        return _analyze_trends(
            snapshots,
            degradation_threshold=self._degradation_threshold,
            improving_threshold=self._improving_threshold,
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _accumulate_state_time(
        m: MachineMetrics, state: str, elapsed: float,
    ) -> None:
        """Add *elapsed* seconds to the appropriate accumulator."""
        m.planned_production_time += elapsed
        if state == _STATE_RUNNING:
            m.run_time += elapsed
        elif state in _DOWNTIME_STATES:
            m.downtime += elapsed
        elif state == _STATE_IDLE:
            m.idle_time += elapsed

    def _now_sec(self) -> float:
        """Return current ROS time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9


# ------------------------------------------------------------------
# Pure functions (testable without ROS2)
# ------------------------------------------------------------------

def compute_availability(
    planned_production_time: float, downtime: float,
) -> float:
    """Availability = (Planned - Downtime) / Planned."""
    if planned_production_time <= 0.0:
        return 1.0
    return max(0.0, min(1.0, (planned_production_time - downtime) / planned_production_time))


def compute_performance(
    ideal_cycle_time: float, total_count: int, run_time: float,
) -> float:
    """Performance = (Ideal Cycle Time x Total Count) / Run Time."""
    if run_time <= 0.0 or total_count <= 0:
        return 1.0
    return max(0.0, min(1.0, (ideal_cycle_time * total_count) / run_time))


def compute_quality(good_count: int, total_count: int) -> float:
    """Quality = Good Count / Total Count."""
    if total_count <= 0:
        return 1.0
    return max(0.0, min(1.0, good_count / total_count))


def compute_oee(
    availability: float, performance: float, quality: float,
) -> float:
    """OEE = Availability x Performance x Quality."""
    return availability * performance * quality


def compute_mtbf(total_run_time: float, failure_count: int) -> float:
    """Mean Time Between Failures = Total Run Time / Failures."""
    if failure_count <= 0:
        return 0.0
    return total_run_time / failure_count


def compute_mttr(total_repair_time: float, failure_count: int) -> float:
    """Mean Time To Repair = Total Repair Time / Failures."""
    if failure_count <= 0:
        return 0.0
    return total_repair_time / failure_count


# ------------------------------------------------------------------
# Trending pure functions (testable without ROS2)
# ------------------------------------------------------------------

def compute_moving_average(
    snapshots: List[OEESnapshot],
    metric: str,
    window_seconds: float,
    reference_time: Optional[float] = None,
) -> float:
    """Compute the moving average of *metric* over the last *window_seconds*.

    Args:
        snapshots: Sorted list of OEESnapshot.
        metric: One of 'oee', 'availability', 'performance', 'quality'.
        window_seconds: Time window in seconds.
        reference_time: End of the window; defaults to the last snapshot ts.

    Returns:
        Average value, or 0.0 if no samples in the window.
    """
    if not snapshots:
        return 0.0
    if reference_time is None:
        reference_time = snapshots[-1].timestamp
    cutoff = reference_time - window_seconds
    values = [
        getattr(s, metric)
        for s in snapshots
        if s.timestamp >= cutoff
    ]
    if not values:
        return 0.0
    return sum(values) / len(values)


def compute_linear_regression_slope(
    snapshots: List[OEESnapshot],
    metric: str,
    window_seconds: float,
    reference_time: Optional[float] = None,
) -> float:
    """Compute a least-squares linear regression slope for *metric*.

    The slope is expressed in *change per hour*.

    Args:
        snapshots: Sorted list of OEESnapshot.
        metric: One of 'oee', 'availability', 'performance', 'quality'.
        window_seconds: How far back to look (seconds).
        reference_time: End of window; defaults to last snapshot ts.

    Returns:
        Slope (change per hour), or 0.0 if fewer than 2 data points.
    """
    if not snapshots:
        return 0.0
    if reference_time is None:
        reference_time = snapshots[-1].timestamp
    cutoff = reference_time - window_seconds

    points = [
        (s.timestamp, getattr(s, metric))
        for s in snapshots
        if s.timestamp >= cutoff
    ]
    n = len(points)
    if n < 2:
        return 0.0

    # Convert timestamps to hours relative to first point
    t0 = points[0][0]
    xs = [(t - t0) / 3600.0 for t, _ in points]
    ys = [v for _, v in points]

    mean_x = sum(xs) / n
    mean_y = sum(ys) / n

    numerator = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    denominator = sum((x - mean_x) ** 2 for x in xs)

    if denominator == 0.0:
        return 0.0
    return numerator / denominator


def _classify_trend(
    slope: float,
    degradation_threshold: float = -0.02,
    improving_threshold: float = 0.01,
) -> str:
    """Classify a slope as 'improving', 'degrading', or 'stable'."""
    if slope <= degradation_threshold:
        return 'degrading'
    if slope >= improving_threshold:
        return 'improving'
    return 'stable'


def _analyze_trends(
    snapshots: List[OEESnapshot],
    degradation_threshold: float = -0.02,
    improving_threshold: float = 0.01,
) -> Dict[str, OEETrendAnalysis]:
    """Analyse trend for each OEE metric.

    Returns a dict keyed by metric name ('oee', 'availability', etc.).
    """
    metrics = ('oee', 'availability', 'performance', 'quality')
    results: Dict[str, OEETrendAnalysis] = {}

    for metric in metrics:
        if not snapshots:
            results[metric] = OEETrendAnalysis(
                metric=metric,
                trend_direction='stable',
                slope_per_hour=0.0,
                current_value=0.0,
                avg_1h=0.0,
                avg_8h=0.0,
                min_value=0.0,
                max_value=0.0,
                samples=0,
            )
            continue

        values = [getattr(s, metric) for s in snapshots]
        slope = compute_linear_regression_slope(
            snapshots, metric, window_seconds=3600.0,
        )
        direction = _classify_trend(
            slope, degradation_threshold, improving_threshold,
        )

        results[metric] = OEETrendAnalysis(
            metric=metric,
            trend_direction=direction,
            slope_per_hour=slope,
            current_value=values[-1],
            avg_1h=compute_moving_average(snapshots, metric, 3600.0),
            avg_8h=compute_moving_average(snapshots, metric, 28800.0),
            min_value=min(values),
            max_value=max(values),
            samples=len(values),
        )

    return results


def compute_real_mtbf(
    failure_timestamps: List[float],
    total_uptime: float,
) -> float:
    """Compute actual MTBF = total_uptime / failure_count."""
    if not failure_timestamps:
        return 0.0
    return total_uptime / len(failure_timestamps)


def compute_real_mttr(repair_durations: List[float]) -> float:
    """Compute actual MTTR = total_repair_time / failure_count."""
    if not repair_durations:
        return 0.0
    return sum(repair_durations) / len(repair_durations)


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main(args=None):
    """Entry point for the KPI calculator node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = KPICalculatorNode()
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
