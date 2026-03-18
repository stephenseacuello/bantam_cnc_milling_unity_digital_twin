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

from typing import Any, Dict, List, Optional, Tuple
from enum import Enum
from dataclasses import dataclass, field, asdict
import json
import math
import threading
import time
import uuid

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


@dataclass
class PredictiveQualityMetrics:
    """Quality metrics enhanced with prediction data."""
    current_quality_pct: float       # existing quality score
    predicted_quality_pct: float     # predicted quality for remaining program
    anomaly_risk_count: int          # number of predicted anomalies
    force_exceedance_blocks: int     # blocks predicted to exceed force limit
    thermal_risk_blocks: int         # blocks with thermal warnings
    surface_quality_risk_blocks: int # blocks with predicted Ra > spec
    predicted_scrap_probability: float  # 0-1 probability of producing scrap
    quality_trend: str               # "improving", "stable", "degrading"
    recommended_action: str          # "none", "adjust_parameters", "tool_change", "abort"


@dataclass
class PredictiveOEE:
    """OEE with forward-looking adjustments."""
    current_oee: float
    predicted_oee: float  # what OEE will be if current trends continue
    availability_risk: float  # risk of unplanned downtime (from tool RUL, anomaly predictions)
    performance_risk: float  # risk of speed loss (from adaptive overrides)
    quality_risk: float  # risk of quality loss (from predicted anomalies)
    time_to_next_intervention_min: float  # predicted time until operator must act


# ---------------------------------------------------------------------------
# Predictive KPI pure functions (testable without ROS2)
# ---------------------------------------------------------------------------

def compute_predictive_quality(anomaly_markers: list,
                               total_blocks: int,
                               current_quality: float) -> PredictiveQualityMetrics:
    """Compute quality prediction from anomaly markers.

    Each marker is a dict with at least a 'type' key:
      - 'force_exceedance' reduces quality by 2% per block
      - 'thermal_warning' reduces quality by 1% per block
      - 'surface_quality_risk' reduces quality by 3% per block

    Scrap probability = 1 - predicted_quality/100 if quality < 90%, else 0.
    Predicted quality is capped at [0, current_quality] (can only degrade).
    """
    force_blocks = sum(1 for m in anomaly_markers if m.get('type') == 'force_exceedance')
    thermal_blocks = sum(1 for m in anomaly_markers if m.get('type') == 'thermal_warning')
    surface_blocks = sum(1 for m in anomaly_markers if m.get('type') == 'surface_quality_risk')

    penalty = (force_blocks * 2.0) + (thermal_blocks * 1.0) + (surface_blocks * 3.0)
    predicted_quality = max(0.0, min(current_quality, current_quality - penalty))

    scrap_prob = max(0.0, 1.0 - predicted_quality / 100.0) if predicted_quality < 90.0 else 0.0

    trend = _classify_quality_trend(current_quality, predicted_quality)
    anomaly_count = force_blocks + thermal_blocks + surface_blocks

    metrics = PredictiveQualityMetrics(
        current_quality_pct=current_quality,
        predicted_quality_pct=predicted_quality,
        anomaly_risk_count=anomaly_count,
        force_exceedance_blocks=force_blocks,
        thermal_risk_blocks=thermal_blocks,
        surface_quality_risk_blocks=surface_blocks,
        predicted_scrap_probability=scrap_prob,
        quality_trend=trend,
        recommended_action='',  # placeholder
    )
    metrics.recommended_action = _recommend_quality_action(metrics)
    return metrics


def compute_predictive_oee(current_oee: float,
                           tool_rul_min: float,
                           anomaly_markers: list,
                           feed_override_pct: float) -> PredictiveOEE:
    """Forward-looking OEE prediction.

    - availability_risk increases as tool RUL approaches 0
      (risk = 1 - min(tool_rul/60, 1.0), i.e. risk=1 when RUL=0)
    - performance_risk increases when feed overrides are active
      (risk = max(0, 1 - feed_override_pct/100))
    - quality_risk from predictive quality anomaly count
    - time_to_intervention = min(tool_rul, time_to_first_critical_anomaly)
    """
    # Availability risk: inversely proportional to tool remaining life
    availability_risk = max(0.0, min(1.0, 1.0 - min(tool_rul_min / 60.0, 1.0)))

    # Performance risk: how much feed is being reduced
    performance_risk = max(0.0, min(1.0, 1.0 - feed_override_pct / 100.0))

    # Quality risk: based on anomaly density
    total_anomalies = len(anomaly_markers)
    quality_risk = max(0.0, min(1.0, total_anomalies * 0.1))

    # Predicted OEE: current OEE adjusted down by risk factors
    risk_factor = (1.0 - availability_risk * 0.3) * (1.0 - performance_risk * 0.3) * (1.0 - quality_risk * 0.4)
    predicted_oee = max(0.0, current_oee * risk_factor)

    # Time to intervention: minimum of tool RUL and first critical anomaly time
    critical_times = [
        m.get('time_from_now_min', float('inf'))
        for m in anomaly_markers
        if m.get('severity') == 'critical'
    ]
    time_to_critical = min(critical_times) if critical_times else float('inf')
    time_to_intervention = min(tool_rul_min, time_to_critical)

    return PredictiveOEE(
        current_oee=current_oee,
        predicted_oee=predicted_oee,
        availability_risk=availability_risk,
        performance_risk=performance_risk,
        quality_risk=quality_risk,
        time_to_next_intervention_min=time_to_intervention,
    )


def _classify_quality_trend(current: float, predicted: float) -> str:
    """Classify quality trend based on current vs predicted."""
    diff = predicted - current
    if diff > 1.0:
        return 'improving'
    elif diff < -1.0:
        return 'degrading'
    return 'stable'


def _recommend_quality_action(metrics: PredictiveQualityMetrics) -> str:
    """Recommend action based on quality risk level."""
    if metrics.predicted_scrap_probability > 0.5:
        return 'abort'
    if metrics.predicted_scrap_probability > 0.2:
        return 'tool_change'
    if metrics.anomaly_risk_count > 0:
        return 'adjust_parameters'
    return 'none'


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
# Shift Handoff Report
# ------------------------------------------------------------------

# Valid event types for ShiftEvent
SHIFT_EVENT_TYPES = {
    'ALARM', 'TOOL_CHANGE', 'PROGRAM_START', 'PROGRAM_END',
    'OVERRIDE', 'ANOMALY', 'MAINTENANCE', 'CALIBRATION',
}

# Valid severity levels for ShiftEvent
SHIFT_SEVERITY_LEVELS = {'INFO', 'WARNING', 'CRITICAL'}


@dataclass
class ShiftEvent:
    """An event that occurred during a shift."""
    timestamp: float
    event_type: str  # ALARM, TOOL_CHANGE, PROGRAM_START, PROGRAM_END, OVERRIDE, ANOMALY, MAINTENANCE, CALIBRATION
    machine_id: str
    severity: str  # INFO, WARNING, CRITICAL
    description: str
    data: Optional[Dict] = field(default_factory=dict)


@dataclass
class ShiftReport:
    """Aggregated report for a single shift, used for handoff."""
    shift_id: str
    shift_start: float
    shift_end: float
    operator_id: str
    machine_ids: List[str]
    oee_summary: Dict[str, Dict[str, float]]  # machine_id -> {availability, performance, quality, overall}
    total_parts_produced: int
    total_scrap_count: int
    tool_changes: int
    alarms: List[ShiftEvent]
    anomalies: List[ShiftEvent]
    maintenance_performed: List[ShiftEvent]
    pending_issues: List[str]
    recommendations: List[str]
    highlight: str


class ShiftReportGenerator:
    """Generates shift handoff reports from recorded events and KPI data."""

    def __init__(self) -> None:
        self._events: List[ShiftEvent] = []

    @property
    def events(self) -> List[ShiftEvent]:
        return list(self._events)

    def record_event(self, event: ShiftEvent) -> None:
        """Add an event to the buffer."""
        self._events.append(event)

    def generate_report(
        self,
        shift_start: float,
        shift_end: float,
        operator_id: str,
        machine_ids: List[str],
        machine_metrics: Optional[Dict[str, MachineMetrics]] = None,
        oee_snapshots: Optional[List[OEESnapshot]] = None,
        machine_oee_snapshots: Optional[Dict[str, List[OEESnapshot]]] = None,
    ) -> ShiftReport:
        """Generate a shift handoff report.

        Args:
            shift_start: Shift start time in seconds.
            shift_end: Shift end time in seconds.
            operator_id: Operator identifier.
            machine_ids: List of machine IDs in this shift.
            machine_metrics: Optional per-machine MachineMetrics for parts/scrap counts.
            oee_snapshots: Optional fleet-level OEE snapshots for the shift window.
            machine_oee_snapshots: Optional per-machine OEE snapshots.

        Returns:
            A populated ShiftReport.
        """
        # Filter events within time window
        shift_events = [
            e for e in self._events
            if shift_start <= e.timestamp <= shift_end
        ]

        # Categorise events
        alarms = [e for e in shift_events if e.event_type == 'ALARM']
        anomalies = [e for e in shift_events if e.event_type == 'ANOMALY']
        maintenance = [e for e in shift_events if e.event_type == 'MAINTENANCE']
        tool_change_events = [e for e in shift_events if e.event_type == 'TOOL_CHANGE']
        tool_changes = len(tool_change_events)

        # OEE summary per machine
        oee_summary: Dict[str, Dict[str, float]] = {}
        if machine_oee_snapshots:
            for mid in machine_ids:
                snaps = [
                    s for s in machine_oee_snapshots.get(mid, [])
                    if shift_start <= s.timestamp <= shift_end
                ]
                if snaps:
                    oee_summary[mid] = {
                        'availability': sum(s.availability for s in snaps) / len(snaps),
                        'performance': sum(s.performance for s in snaps) / len(snaps),
                        'quality': sum(s.quality for s in snaps) / len(snaps),
                        'overall': sum(s.oee for s in snaps) / len(snaps),
                    }
                else:
                    oee_summary[mid] = {
                        'availability': 0.0, 'performance': 0.0,
                        'quality': 0.0, 'overall': 0.0,
                    }
        elif oee_snapshots:
            # Use fleet-level snapshots if per-machine not available
            fleet_snaps = [
                s for s in oee_snapshots
                if shift_start <= s.timestamp <= shift_end
            ]
            if fleet_snaps:
                fleet_avg = {
                    'availability': sum(s.availability for s in fleet_snaps) / len(fleet_snaps),
                    'performance': sum(s.performance for s in fleet_snaps) / len(fleet_snaps),
                    'quality': sum(s.quality for s in fleet_snaps) / len(fleet_snaps),
                    'overall': sum(s.oee for s in fleet_snaps) / len(fleet_snaps),
                }
                for mid in machine_ids:
                    oee_summary[mid] = dict(fleet_avg)
            else:
                for mid in machine_ids:
                    oee_summary[mid] = {
                        'availability': 0.0, 'performance': 0.0,
                        'quality': 0.0, 'overall': 0.0,
                    }
        else:
            for mid in machine_ids:
                oee_summary[mid] = {
                    'availability': 0.0, 'performance': 0.0,
                    'quality': 0.0, 'overall': 0.0,
                }

        # Parts and scrap from machine metrics
        total_parts = 0
        total_scrap = 0
        if machine_metrics:
            for mid in machine_ids:
                m = machine_metrics.get(mid)
                if m:
                    total_parts += m.total_jobs
                    total_scrap += m.defect_jobs

        # Pending issues
        pending_issues = self._identify_pending_issues(
            alarms, anomalies, shift_events, machine_metrics, machine_ids,
        )

        # Recommendations
        recommendations = self._generate_recommendations(
            shift_events, machine_metrics, machine_ids, oee_summary,
        )

        # Highlight
        highlight = self._pick_highlight(shift_events, oee_summary, machine_ids)

        shift_id = str(uuid.uuid4())

        return ShiftReport(
            shift_id=shift_id,
            shift_start=shift_start,
            shift_end=shift_end,
            operator_id=operator_id,
            machine_ids=list(machine_ids),
            oee_summary=oee_summary,
            total_parts_produced=total_parts,
            total_scrap_count=total_scrap,
            tool_changes=tool_changes,
            alarms=alarms,
            anomalies=anomalies,
            maintenance_performed=maintenance,
            pending_issues=pending_issues,
            recommendations=recommendations,
            highlight=highlight,
        )

    def export_report_text(self, report: ShiftReport) -> str:
        """Export a shift report as human-readable markdown."""
        lines = []
        lines.append(f"# Shift Handoff Report")
        lines.append(f"")
        lines.append(f"**Shift ID:** {report.shift_id}")
        lines.append(f"**Operator:** {report.operator_id}")
        lines.append(f"**Period:** {report.shift_start:.0f} - {report.shift_end:.0f}")
        lines.append(f"**Machines:** {', '.join(report.machine_ids)}")
        lines.append(f"")
        lines.append(f"## Production Summary")
        lines.append(f"")
        lines.append(f"- **Parts Produced:** {report.total_parts_produced}")
        lines.append(f"- **Scrap Count:** {report.total_scrap_count}")
        lines.append(f"- **Tool Changes:** {report.tool_changes}")
        lines.append(f"")
        lines.append(f"## OEE Summary")
        lines.append(f"")
        for mid, oee in report.oee_summary.items():
            lines.append(
                f"- **{mid}:** OEE={oee['overall']:.1%} "
                f"(A={oee['availability']:.1%}, "
                f"P={oee['performance']:.1%}, "
                f"Q={oee['quality']:.1%})"
            )
        lines.append(f"")

        if report.alarms:
            lines.append(f"## Alarms ({len(report.alarms)})")
            lines.append(f"")
            for a in report.alarms:
                lines.append(f"- [{a.severity}] {a.machine_id}: {a.description}")
            lines.append(f"")

        if report.anomalies:
            lines.append(f"## Anomalies ({len(report.anomalies)})")
            lines.append(f"")
            for a in report.anomalies:
                lines.append(f"- [{a.severity}] {a.machine_id}: {a.description}")
            lines.append(f"")

        if report.maintenance_performed:
            lines.append(f"## Maintenance Performed ({len(report.maintenance_performed)})")
            lines.append(f"")
            for m in report.maintenance_performed:
                lines.append(f"- {m.machine_id}: {m.description}")
            lines.append(f"")

        if report.pending_issues:
            lines.append(f"## Pending Issues")
            lines.append(f"")
            for issue in report.pending_issues:
                lines.append(f"- {issue}")
            lines.append(f"")

        if report.recommendations:
            lines.append(f"## Recommendations")
            lines.append(f"")
            for rec in report.recommendations:
                lines.append(f"- {rec}")
            lines.append(f"")

        if report.highlight:
            lines.append(f"## Shift Highlight")
            lines.append(f"")
            lines.append(f"{report.highlight}")
            lines.append(f"")

        return '\n'.join(lines)

    def export_report_json(self, report: ShiftReport) -> str:
        """Export a shift report as JSON string for integration."""
        data = {
            'shift_id': report.shift_id,
            'shift_start': report.shift_start,
            'shift_end': report.shift_end,
            'operator_id': report.operator_id,
            'machine_ids': report.machine_ids,
            'oee_summary': report.oee_summary,
            'total_parts_produced': report.total_parts_produced,
            'total_scrap_count': report.total_scrap_count,
            'tool_changes': report.tool_changes,
            'alarms': [self._event_to_dict(e) for e in report.alarms],
            'anomalies': [self._event_to_dict(e) for e in report.anomalies],
            'maintenance_performed': [self._event_to_dict(e) for e in report.maintenance_performed],
            'pending_issues': report.pending_issues,
            'recommendations': report.recommendations,
            'highlight': report.highlight,
        }
        return json.dumps(data, indent=2)

    def get_shift_comparison(
        self, report1: ShiftReport, report2: ShiftReport,
    ) -> Dict[str, Any]:
        """Compare two shift reports and return deltas.

        Returns a dict with delta values for key metrics.  Positive deltas
        indicate improvement from report1 to report2.
        """
        # Aggregate OEE across all machines in each report
        oee1 = self._avg_oee(report1)
        oee2 = self._avg_oee(report2)

        return {
            'oee_delta': oee2 - oee1,
            'availability_delta': self._avg_metric(report2, 'availability') - self._avg_metric(report1, 'availability'),
            'performance_delta': self._avg_metric(report2, 'performance') - self._avg_metric(report1, 'performance'),
            'quality_delta': self._avg_metric(report2, 'quality') - self._avg_metric(report1, 'quality'),
            'parts_delta': report2.total_parts_produced - report1.total_parts_produced,
            'scrap_delta': report2.total_scrap_count - report1.total_scrap_count,
            'alarm_count_delta': len(report2.alarms) - len(report1.alarms),
            'tool_changes_delta': report2.tool_changes - report1.tool_changes,
            'shift1_id': report1.shift_id,
            'shift2_id': report2.shift_id,
        }

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _event_to_dict(event: ShiftEvent) -> Dict:
        return {
            'timestamp': event.timestamp,
            'event_type': event.event_type,
            'machine_id': event.machine_id,
            'severity': event.severity,
            'description': event.description,
            'data': event.data if event.data else {},
        }

    @staticmethod
    def _avg_oee(report: ShiftReport) -> float:
        if not report.oee_summary:
            return 0.0
        vals = [v.get('overall', 0.0) for v in report.oee_summary.values()]
        return sum(vals) / len(vals) if vals else 0.0

    @staticmethod
    def _avg_metric(report: ShiftReport, metric: str) -> float:
        if not report.oee_summary:
            return 0.0
        vals = [v.get(metric, 0.0) for v in report.oee_summary.values()]
        return sum(vals) / len(vals) if vals else 0.0

    @staticmethod
    def _identify_pending_issues(
        alarms: List[ShiftEvent],
        anomalies: List[ShiftEvent],
        shift_events: List[ShiftEvent],
        machine_metrics: Optional[Dict[str, MachineMetrics]],
        machine_ids: List[str],
    ) -> List[str]:
        """Identify issues the next shift needs to know about."""
        issues: List[str] = []

        # Unresolved critical/warning alarms
        critical_alarms = [a for a in alarms if a.severity in ('CRITICAL', 'WARNING')]
        for a in critical_alarms:
            issues.append(f"Unresolved {a.severity.lower()} alarm on {a.machine_id}: {a.description}")

        # Recurring anomalies (same type on same machine appears 2+ times)
        anomaly_counts: Dict[str, int] = {}
        for a in anomalies:
            key = f"{a.machine_id}:{a.event_type}:{a.description}"
            anomaly_counts[key] = anomaly_counts.get(key, 0) + 1
        for key, count in anomaly_counts.items():
            if count >= 2:
                parts = key.split(':', 2)
                issues.append(
                    f"Recurring anomaly on {parts[0]}: {parts[2]} ({count} occurrences)"
                )

        # Low tool RUL from metrics
        if machine_metrics:
            for mid in machine_ids:
                m = machine_metrics.get(mid)
                if m and m.tool_life_total > 0:
                    remaining = 1.0 - (m.tool_life_used / m.tool_life_total)
                    if remaining < 0.2:
                        issues.append(
                            f"Tool on {mid} at {remaining:.0%} remaining life"
                        )

        # Calibration events that may indicate drift
        calibrations = [e for e in shift_events if e.event_type == 'CALIBRATION']
        for c in calibrations:
            drift = (c.data or {}).get('drift')
            if drift is not None and abs(drift) > 0.05:
                issues.append(
                    f"Calibration drift on {c.machine_id}: {drift:.3f}"
                )

        return issues

    @staticmethod
    def _generate_recommendations(
        shift_events: List[ShiftEvent],
        machine_metrics: Optional[Dict[str, MachineMetrics]],
        machine_ids: List[str],
        oee_summary: Dict[str, Dict[str, float]],
    ) -> List[str]:
        """Generate actionable recommendations for the next shift."""
        recs: List[str] = []

        # Tool approaching end-of-life
        if machine_metrics:
            for mid in machine_ids:
                m = machine_metrics.get(mid)
                if m and m.tool_life_total > 0:
                    remaining = 1.0 - (m.tool_life_used / m.tool_life_total)
                    if remaining < 0.3:
                        recs.append(
                            f"Schedule tool change for {mid} "
                            f"({remaining:.0%} remaining)"
                        )

        # Recurring anomaly patterns
        anomaly_types: Dict[str, int] = {}
        for e in shift_events:
            if e.event_type == 'ANOMALY':
                key = f"{e.machine_id}:{e.description}"
                anomaly_types[key] = anomaly_types.get(key, 0) + 1
        for key, count in anomaly_types.items():
            if count >= 3:
                mid, desc = key.split(':', 1)
                recs.append(
                    f"Investigate recurring anomaly on {mid}: "
                    f"{desc} ({count} times this shift)"
                )

        # Low OEE machines
        for mid, oee in oee_summary.items():
            if oee.get('overall', 0.0) < 0.65:
                recs.append(
                    f"Review low OEE on {mid} "
                    f"({oee['overall']:.1%})"
                )

        # High scrap rate
        if machine_metrics:
            for mid in machine_ids:
                m = machine_metrics.get(mid)
                if m and m.total_jobs > 0:
                    scrap_rate = m.defect_jobs / m.total_jobs
                    if scrap_rate > 0.1:
                        recs.append(
                            f"High scrap rate on {mid}: "
                            f"{scrap_rate:.1%} — check tooling and material"
                        )

        return recs

    @staticmethod
    def _pick_highlight(
        shift_events: List[ShiftEvent],
        oee_summary: Dict[str, Dict[str, float]],
        machine_ids: List[str],
    ) -> str:
        """Pick the most notable event or achievement for the shift."""
        # Priority: critical event > best OEE achievement > summary
        critical_events = [
            e for e in shift_events if e.severity == 'CRITICAL'
        ]
        if critical_events:
            e = critical_events[0]
            return (
                f"CRITICAL event on {e.machine_id}: {e.description}"
            )

        # Best OEE achievement
        best_mid = None
        best_oee = -1.0
        for mid, oee in oee_summary.items():
            val = oee.get('overall', 0.0)
            if val > best_oee:
                best_oee = val
                best_mid = mid
        if best_mid and best_oee > 0.85:
            return (
                f"Excellent OEE on {best_mid}: {best_oee:.1%}"
            )

        if not shift_events:
            return "Quiet shift with no notable events."

        return (
            f"Shift completed with {len(shift_events)} events across "
            f"{len(machine_ids)} machine(s)."
        )


# ---------------------------------------------------------------------------
# Process Capability Analysis (Cp / Cpk)
# ---------------------------------------------------------------------------

@dataclass
class MeasurementSample:
    """A single dimensional measurement from a machined part."""
    value: float
    timestamp: float
    feature_id: str       # e.g. "diameter_A", "length_B"
    machine_id: str
    program_id: str


@dataclass
class ProcessCapability:
    """Statistical process capability indices for a feature."""
    feature_id: str
    sample_count: int
    mean: float
    std_dev: float
    usl: float            # upper specification limit
    lsl: float            # lower specification limit
    cp: float             # (USL - LSL) / (6 * sigma)
    cpk: float            # min(Cpu, Cpl)
    cpu: float            # (USL - mean) / (3 * sigma)
    cpl: float            # (mean - LSL) / (3 * sigma)
    pp: float             # process performance (overall std dev)
    ppk: float
    is_capable: bool      # True when Cpk >= 1.33
    trend: str            # STABLE, IMPROVING, DEGRADING
    out_of_spec_pct: float


def _normal_cdf(x: float) -> float:
    """Approximate the standard normal CDF using the error function."""
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2.0)))


class CapabilityProfiler:
    """Collects measurement samples and computes SPC capability indices."""

    _MIN_SAMPLES = 30

    def __init__(self) -> None:
        # feature_id -> list of MeasurementSample (ordered by insertion)
        self._samples: Dict[str, List[MeasurementSample]] = {}

    # -- sample ingestion ---------------------------------------------------

    def add_sample(self, sample: MeasurementSample) -> None:
        """Append a measurement sample."""
        self._samples.setdefault(sample.feature_id, []).append(sample)

    # -- capability computation ---------------------------------------------

    def compute_capability(
        self, feature_id: str, usl: float, lsl: float,
    ) -> ProcessCapability:
        """Compute Cp/Cpk and related indices for *feature_id*.

        Raises ``ValueError`` if fewer than 30 samples are available.
        """
        samples = self._samples.get(feature_id, [])
        n = len(samples)
        if n < self._MIN_SAMPLES:
            raise ValueError(
                f"Need >= {self._MIN_SAMPLES} samples for capability "
                f"analysis, got {n}"
            )

        values = [s.value for s in samples]
        mean = sum(values) / n
        variance = sum((v - mean) ** 2 for v in values) / (n - 1)
        std_dev = math.sqrt(variance) if variance > 0 else 0.0

        if std_dev == 0.0:
            # Perfectly repeatable process — capability is infinite
            cp = float('inf')
            cpk = float('inf')
            cpu = float('inf')
            cpl = float('inf')
            pp = float('inf')
            ppk = float('inf')
        else:
            cp = (usl - lsl) / (6.0 * std_dev)
            cpu = (usl - mean) / (3.0 * std_dev)
            cpl = (mean - lsl) / (3.0 * std_dev)
            cpk = min(cpu, cpl)

            # Pp / Ppk use overall (population) std dev
            overall_var = sum((v - mean) ** 2 for v in values) / n
            overall_std = math.sqrt(overall_var) if overall_var > 0 else 0.0
            pp = (usl - lsl) / (6.0 * overall_std)
            ppu = (usl - mean) / (3.0 * overall_std)
            ppl = (mean - lsl) / (3.0 * overall_std)
            ppk = min(ppu, ppl)

        # Trend: compare first-half mean vs second-half mean
        trend = self._compute_trend(values)

        # Out-of-spec percentage
        out_of_spec_pct = self._out_of_spec_pct(values, usl, lsl)

        return ProcessCapability(
            feature_id=feature_id,
            sample_count=n,
            mean=mean,
            std_dev=std_dev,
            usl=usl,
            lsl=lsl,
            cp=cp,
            cpk=cpk,
            cpu=cpu,
            cpl=cpl,
            pp=pp,
            ppk=ppk,
            is_capable=(cpk >= 1.33),
            trend=trend,
            out_of_spec_pct=out_of_spec_pct,
        )

    # -- control chart ------------------------------------------------------

    def get_control_chart_data(self, feature_id: str) -> Dict[str, Any]:
        """Return X-bar control chart data with Western Electric rules.

        Returns a dict with keys: values, ucl, lcl, center_line,
        out_of_control_points.
        """
        samples = self._samples.get(feature_id, [])
        values = [s.value for s in samples]
        n = len(values)

        if n == 0:
            return {
                'values': [],
                'ucl': 0.0,
                'lcl': 0.0,
                'center_line': 0.0,
                'out_of_control_points': [],
            }

        mean = sum(values) / n
        if n > 1:
            variance = sum((v - mean) ** 2 for v in values) / (n - 1)
            sigma = math.sqrt(variance) if variance > 0 else 0.0
        else:
            sigma = 0.0

        ucl = mean + 3.0 * sigma
        lcl = mean - 3.0 * sigma

        ooc = self._western_electric(values, mean, sigma)

        return {
            'values': values,
            'ucl': ucl,
            'lcl': lcl,
            'center_line': mean,
            'out_of_control_points': ooc,
        }

    # -- machine summaries --------------------------------------------------

    def get_machine_capability_summary(
        self,
        machine_id: str,
    ) -> Dict[str, List[MeasurementSample]]:
        """Return all feature samples produced by *machine_id*.

        The caller should supply USL/LSL per feature to compute capability
        via :meth:`compute_capability`.  This method returns a dict of
        ``feature_id -> [MeasurementSample, ...]`` filtered to *machine_id*.
        """
        result: Dict[str, List[MeasurementSample]] = {}
        for fid, samples in self._samples.items():
            filtered = [s for s in samples if s.machine_id == machine_id]
            if filtered:
                result[fid] = filtered
        return result

    # -- machine comparison -------------------------------------------------

    def compare_machines(
        self,
        machine_id_1: str,
        machine_id_2: str,
        feature_id: str,
        usl: float,
        lsl: float,
    ) -> Dict[str, Any]:
        """Compare Cpk between two machines for a given feature.

        Returns a dict with cpk_1, cpk_2, better_machine, and delta.
        """
        # Build per-machine profilers on the fly
        def _machine_cpk(mid: str) -> Optional[float]:
            samples = [
                s for s in self._samples.get(feature_id, [])
                if s.machine_id == mid
            ]
            if len(samples) < self._MIN_SAMPLES:
                return None
            tmp = CapabilityProfiler()
            for s in samples:
                tmp.add_sample(s)
            cap = tmp.compute_capability(feature_id, usl, lsl)
            return cap.cpk

        cpk1 = _machine_cpk(machine_id_1)
        cpk2 = _machine_cpk(machine_id_2)

        better: Optional[str] = None
        delta = 0.0
        if cpk1 is not None and cpk2 is not None:
            delta = cpk1 - cpk2
            better = machine_id_1 if cpk1 >= cpk2 else machine_id_2

        return {
            'feature_id': feature_id,
            'machine_id_1': machine_id_1,
            'cpk_1': cpk1,
            'machine_id_2': machine_id_2,
            'cpk_2': cpk2,
            'better_machine': better,
            'delta': delta,
        }

    # -- scrap rate prediction ----------------------------------------------

    def predict_scrap_rate(
        self, feature_id: str, usl: float, lsl: float,
    ) -> float:
        """Estimate scrap percentage from a fitted normal distribution.

        Returns fraction (0-1) of parts predicted to fall outside [LSL, USL].
        """
        samples = self._samples.get(feature_id, [])
        n = len(samples)
        if n < 2:
            return 0.0

        values = [s.value for s in samples]
        mean = sum(values) / n
        variance = sum((v - mean) ** 2 for v in values) / (n - 1)
        std_dev = math.sqrt(variance) if variance > 0 else 0.0

        if std_dev == 0.0:
            # All identical — if within spec, 0 scrap; else 100%
            if lsl <= mean <= usl:
                return 0.0
            return 1.0

        p_below_lsl = _normal_cdf((lsl - mean) / std_dev)
        p_above_usl = 1.0 - _normal_cdf((usl - mean) / std_dev)
        return p_below_lsl + p_above_usl

    # -- internal helpers ---------------------------------------------------

    @staticmethod
    def _compute_trend(values: List[float]) -> str:
        """Compare first-half vs second-half std deviation to detect trend."""
        n = len(values)
        if n < 10:
            return 'STABLE'
        mid = n // 2
        first_half = values[:mid]
        second_half = values[mid:]

        def _std(v: List[float]) -> float:
            m = sum(v) / len(v)
            var = sum((x - m) ** 2 for x in v) / len(v)
            return math.sqrt(var)

        std1 = _std(first_half)
        std2 = _std(second_half)

        if std1 == 0.0 and std2 == 0.0:
            return 'STABLE'
        if std1 == 0.0:
            return 'DEGRADING'

        ratio = std2 / std1
        if ratio > 1.25:
            return 'DEGRADING'
        if ratio < 0.75:
            return 'IMPROVING'
        return 'STABLE'

    @staticmethod
    def _out_of_spec_pct(
        values: List[float], usl: float, lsl: float,
    ) -> float:
        """Compute observed out-of-spec percentage."""
        if not values:
            return 0.0
        ooc = sum(1 for v in values if v < lsl or v > usl)
        return ooc / len(values) * 100.0

    @staticmethod
    def _western_electric(
        values: List[float], mean: float, sigma: float,
    ) -> List[int]:
        """Detect out-of-control points using Western Electric rules.

        Rules applied:
          1. Any single point beyond 3-sigma.
          2. Two of three consecutive points beyond 2-sigma (same side).
          3. Four of five consecutive points beyond 1-sigma (same side).
          4. Eight consecutive points on the same side of the centre line.

        Returns a sorted list of unique indices that violate any rule.
        """
        n = len(values)
        if n == 0 or sigma == 0.0:
            return []

        flagged: set = set()

        for i in range(n):
            z = (values[i] - mean) / sigma

            # Rule 1: beyond 3-sigma
            if abs(z) > 3.0:
                flagged.add(i)

        # Rule 2: 2 of 3 beyond 2-sigma on same side
        for i in range(2, n):
            window = [values[i - 2], values[i - 1], values[i]]
            above = sum(1 for v in window if (v - mean) > 2.0 * sigma)
            below = sum(1 for v in window if (mean - v) > 2.0 * sigma)
            if above >= 2:
                flagged.update([i - 2, i - 1, i])
            if below >= 2:
                flagged.update([i - 2, i - 1, i])

        # Rule 3: 4 of 5 beyond 1-sigma on same side
        for i in range(4, n):
            window = [values[j] for j in range(i - 4, i + 1)]
            above = sum(1 for v in window if (v - mean) > 1.0 * sigma)
            below = sum(1 for v in window if (mean - v) > 1.0 * sigma)
            if above >= 4:
                flagged.update(range(i - 4, i + 1))
            if below >= 4:
                flagged.update(range(i - 4, i + 1))

        # Rule 4: 8 consecutive on same side
        for i in range(7, n):
            window = values[i - 7: i + 1]
            all_above = all(v > mean for v in window)
            all_below = all(v < mean for v in window)
            if all_above or all_below:
                flagged.update(range(i - 7, i + 1))

        return sorted(flagged)


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

# ---------------------------------------------------------------------------
# SPC Control Charts
# ---------------------------------------------------------------------------

class ControlChartType(Enum):
    """Types of SPC control charts."""
    XBAR_R = 'xbar_r'
    XBAR_S = 'xbar_s'
    EWMA = 'ewma'
    CUSUM = 'cusum'


@dataclass
class RuleViolation:
    """A Western Electric rule violation on a control chart."""
    rule_number: int
    rule_name: str
    point_index: int
    value: float
    description: str


@dataclass
class ControlChartData:
    """X-bar and R chart data with control limits and violations."""
    subgroup_means: List[float] = field(default_factory=list)
    subgroup_ranges: List[float] = field(default_factory=list)
    x_bar_ucl: float = 0.0
    x_bar_cl: float = 0.0
    x_bar_lcl: float = 0.0
    r_ucl: float = 0.0
    r_cl: float = 0.0
    r_lcl: float = 0.0
    out_of_control_points: List[int] = field(default_factory=list)
    rule_violations: List[RuleViolation] = field(default_factory=list)


@dataclass
class EWMAChartData:
    """EWMA chart data."""
    ewma_values: List[float] = field(default_factory=list)
    ucl_values: List[float] = field(default_factory=list)
    lcl_values: List[float] = field(default_factory=list)
    center_line: float = 0.0
    lambda_param: float = 0.2
    out_of_control_points: List[int] = field(default_factory=list)


# Standard SPC constants for subgroup sizes 2-10
_A2 = {2: 1.880, 3: 1.023, 4: 0.729, 5: 0.577, 6: 0.483, 7: 0.419, 8: 0.373, 9: 0.337, 10: 0.308}
_D3 = {2: 0.000, 3: 0.000, 4: 0.000, 5: 0.000, 6: 0.000, 7: 0.076, 8: 0.136, 9: 0.184, 10: 0.223}
_D4 = {2: 3.267, 3: 2.574, 4: 2.282, 5: 2.114, 6: 2.004, 7: 1.924, 8: 1.864, 9: 1.816, 10: 1.777}
_d2 = {2: 1.128, 3: 1.693, 4: 2.059, 5: 2.326, 6: 2.534, 7: 2.704, 8: 2.847, 9: 2.970, 10: 3.078}


class ControlChartGenerator:
    """Generates SPC control charts from subgroup measurement data.

    Supports X-bar/R charts, EWMA charts, and Western Electric rule detection.
    """

    @staticmethod
    def compute_xbar_r(subgroups: List[List[float]]) -> ControlChartData:
        """Compute X-bar and R chart from subgroup data.

        Each subgroup is a list of measurements. All subgroups should be the same size.
        """
        if not subgroups:
            return ControlChartData()

        n = len(subgroups[0])  # subgroup size
        if n < 2 or n > 10:
            return ControlChartData()

        means = [sum(sg) / len(sg) for sg in subgroups]
        ranges = [max(sg) - min(sg) for sg in subgroups]

        grand_mean = sum(means) / len(means)
        r_bar = sum(ranges) / len(ranges)

        a2 = _A2[n]
        d3 = _D3[n]
        d4 = _D4[n]

        chart = ControlChartData(
            subgroup_means=means,
            subgroup_ranges=ranges,
            x_bar_ucl=grand_mean + a2 * r_bar,
            x_bar_cl=grand_mean,
            x_bar_lcl=grand_mean - a2 * r_bar,
            r_ucl=d4 * r_bar,
            r_cl=r_bar,
            r_lcl=d3 * r_bar,
        )

        # Detect out-of-control points (beyond X-bar limits)
        for i, m in enumerate(means):
            if m > chart.x_bar_ucl or m < chart.x_bar_lcl:
                chart.out_of_control_points.append(i)

        # Check Western Electric rules
        chart.rule_violations = ControlChartGenerator._check_western_electric(
            means, grand_mean, chart.x_bar_ucl, chart.x_bar_lcl)

        return chart

    @staticmethod
    def compute_ewma(data: List[float], lambda_param: float = 0.2,
                     L: float = 3.0, mu: Optional[float] = None,
                     sigma: Optional[float] = None) -> EWMAChartData:
        """Compute EWMA chart.

        UCL/LCL = mu ± L * sigma * sqrt(lambda/(2-lambda) * (1-(1-lambda)^(2i)))
        """
        if not data:
            return EWMAChartData(lambda_param=lambda_param)

        if mu is None:
            mu = sum(data) / len(data)
        if sigma is None:
            # Estimate sigma from moving ranges
            if len(data) > 1:
                mr = [abs(data[i] - data[i - 1]) for i in range(1, len(data))]
                sigma = (sum(mr) / len(mr)) / 1.128  # d2 for n=2
            else:
                sigma = 1.0

        ewma_values: List[float] = []
        ucl_values: List[float] = []
        lcl_values: List[float] = []
        ooc: List[int] = []

        z = mu  # initial EWMA = mu
        lam = lambda_param
        for i, x in enumerate(data):
            z = lam * x + (1 - lam) * z
            ewma_values.append(z)

            # Control limits widen then stabilise
            factor = math.sqrt(lam / (2 - lam) * (1 - (1 - lam) ** (2 * (i + 1))))
            ucl = mu + L * sigma * factor
            lcl = mu - L * sigma * factor
            ucl_values.append(ucl)
            lcl_values.append(lcl)

            if z > ucl or z < lcl:
                ooc.append(i)

        return EWMAChartData(
            ewma_values=ewma_values,
            ucl_values=ucl_values,
            lcl_values=lcl_values,
            center_line=mu,
            lambda_param=lam,
            out_of_control_points=ooc,
        )

    @staticmethod
    def _check_western_electric(values: List[float], cl: float,
                                 ucl: float, lcl: float) -> List[RuleViolation]:
        """Check Western Electric rules 1-4."""
        violations: List[RuleViolation] = []
        sigma = (ucl - cl) / 3.0 if ucl != cl else 1.0

        # Rule 1: Point beyond 3σ (already caught as OOC, but record as violation)
        for i, v in enumerate(values):
            if v > ucl or v < lcl:
                violations.append(RuleViolation(
                    rule_number=1, rule_name='Beyond 3-sigma',
                    point_index=i, value=v,
                    description=f'Point {i} ({v:.4f}) beyond control limits',
                ))

        # Rule 2: 9 consecutive points on same side of CL
        if len(values) >= 9:
            above = [v > cl for v in values]
            for i in range(len(values) - 8):
                run = above[i:i + 9]
                if all(run) or not any(run):
                    side = 'above' if run[0] else 'below'
                    violations.append(RuleViolation(
                        rule_number=2, rule_name='9 consecutive same side',
                        point_index=i, value=values[i],
                        description=f'9 consecutive points {side} CL starting at {i}',
                    ))
                    break  # report first occurrence

        # Rule 3: 6 consecutive increasing or decreasing
        if len(values) >= 6:
            for i in range(len(values) - 5):
                segment = values[i:i + 6]
                increasing = all(segment[j + 1] > segment[j] for j in range(5))
                decreasing = all(segment[j + 1] < segment[j] for j in range(5))
                if increasing or decreasing:
                    direction = 'increasing' if increasing else 'decreasing'
                    violations.append(RuleViolation(
                        rule_number=3, rule_name='6 consecutive trend',
                        point_index=i, value=values[i],
                        description=f'6 consecutive {direction} points starting at {i}',
                    ))
                    break

        # Rule 4: 14 consecutive alternating up/down
        if len(values) >= 14:
            for i in range(len(values) - 13):
                segment = values[i:i + 14]
                alternating = True
                for j in range(1, 13):
                    if j % 2 == 1:
                        if not (segment[j] > segment[j - 1]):
                            alternating = False
                            break
                    else:
                        if not (segment[j] < segment[j - 1]):
                            alternating = False
                            break
                if not alternating:
                    # Check opposite pattern
                    alternating = True
                    for j in range(1, 13):
                        if j % 2 == 1:
                            if not (segment[j] < segment[j - 1]):
                                alternating = False
                                break
                        else:
                            if not (segment[j] > segment[j - 1]):
                                alternating = False
                                break
                if alternating:
                    violations.append(RuleViolation(
                        rule_number=4, rule_name='14 consecutive alternating',
                        point_index=i, value=values[i],
                        description=f'14 consecutive alternating points starting at {i}',
                    ))
                    break

        return violations

    @staticmethod
    def get_constants(n: int) -> Optional[Dict[str, float]]:
        """Get A2, D3, D4, d2 constants for subgroup size n (2-10)."""
        if n < 2 or n > 10:
            return None
        return {'A2': _A2[n], 'D3': _D3[n], 'D4': _D4[n], 'd2': _d2[n]}


# ---------------------------------------------------------------------------
# Downtime Event Classifier
# ---------------------------------------------------------------------------

_DOWNTIME_CATEGORIES = frozenset({
    'planned', 'unplanned', 'changeover', 'maintenance',
    'quality', 'material', 'operator',
})


@dataclass
class DowntimeEvent:
    """A single machine downtime event."""
    event_id: str
    machine_id: str
    start_time: float
    end_time: Optional[float]
    category: str          # one of _DOWNTIME_CATEGORIES
    reason: str
    resolved: bool = False

    def duration_min(self) -> float:
        """Return event duration in minutes.  Returns 0 if still open."""
        if self.end_time is None:
            return 0.0
        return max(0.0, (self.end_time - self.start_time) / 60.0)


@dataclass
class DowntimeSummary:
    """Aggregated downtime statistics over a time range."""
    total_downtime_min: float = 0.0
    planned_min: float = 0.0
    unplanned_min: float = 0.0
    by_category: Dict[str, float] = field(default_factory=dict)
    by_machine: Dict[str, float] = field(default_factory=dict)
    event_count: int = 0
    mttr_min: float = 0.0      # mean time to repair (resolved events)
    top_reasons: List[Tuple[str, int, float]] = field(default_factory=list)


class DowntimeClassifier:
    """Classifies and tracks machine downtime events by category.

    Provides summary statistics, Pareto analysis, and OEE availability
    calculations for recorded downtime events.
    """

    def __init__(self) -> None:
        self._events: Dict[str, DowntimeEvent] = {}   # event_id -> event
        self._lock = threading.Lock()

    # -- Recording ----------------------------------------------------------

    def record_event(self, event: DowntimeEvent) -> None:
        """Record a downtime event.

        Raises ``ValueError`` if the category is not one of the valid
        downtime categories or the event_id is already registered.
        """
        if event.category not in _DOWNTIME_CATEGORIES:
            raise ValueError(
                f"Invalid category '{event.category}'. "
                f"Must be one of {sorted(_DOWNTIME_CATEGORIES)}."
            )
        with self._lock:
            if event.event_id in self._events:
                raise ValueError(
                    f"Event '{event.event_id}' already exists."
                )
            self._events[event.event_id] = event

    def close_event(self, event_id: str, end_time: float) -> None:
        """Mark an event as resolved with the given *end_time*.

        Raises ``KeyError`` if the event_id is not found.
        """
        with self._lock:
            if event_id not in self._events:
                raise KeyError(f"Event '{event_id}' not found.")
            ev = self._events[event_id]
            ev.end_time = end_time
            ev.resolved = True

    # -- Queries ------------------------------------------------------------

    def get_events_by_machine(self, machine_id: str) -> List[DowntimeEvent]:
        """Return all events for a given machine."""
        with self._lock:
            return [e for e in self._events.values()
                    if e.machine_id == machine_id]

    def get_open_events(self) -> List[DowntimeEvent]:
        """Return events that have not been closed/resolved."""
        with self._lock:
            return [e for e in self._events.values() if not e.resolved]

    # -- Analysis -----------------------------------------------------------

    def _events_in_range(
        self, start_time: float, end_time: float,
    ) -> List[DowntimeEvent]:
        """Return resolved events overlapping the given time window."""
        results: List[DowntimeEvent] = []
        for ev in self._events.values():
            if ev.end_time is None:
                continue
            # Event overlaps window if it starts before window-end and
            # ends after window-start.
            if ev.start_time < end_time and ev.end_time > start_time:
                results.append(ev)
        return results

    @staticmethod
    def _clipped_duration_min(
        ev: DowntimeEvent, start_time: float, end_time: float,
    ) -> float:
        """Duration of *ev* clipped to the [start_time, end_time] window, in minutes."""
        if ev.end_time is None:
            return 0.0
        clipped_start = max(ev.start_time, start_time)
        clipped_end = min(ev.end_time, end_time)
        return max(0.0, (clipped_end - clipped_start) / 60.0)

    def get_summary(
        self, start_time: float, end_time: float,
    ) -> DowntimeSummary:
        """Generate a ``DowntimeSummary`` for the given time range.

        Only resolved (closed) events that overlap the window are included.
        Durations are clipped to the window boundaries.
        """
        with self._lock:
            events = self._events_in_range(start_time, end_time)

        total = 0.0
        planned = 0.0
        unplanned = 0.0
        by_cat: Dict[str, float] = {}
        by_machine: Dict[str, float] = {}
        reason_map: Dict[str, List[float]] = {}  # reason -> list of durations

        resolved_durations: List[float] = []

        for ev in events:
            dur = self._clipped_duration_min(ev, start_time, end_time)
            total += dur

            if ev.category == 'planned':
                planned += dur
            else:
                unplanned += dur

            by_cat[ev.category] = by_cat.get(ev.category, 0.0) + dur
            by_machine[ev.machine_id] = by_machine.get(ev.machine_id, 0.0) + dur

            reason_map.setdefault(ev.reason, []).append(dur)

            if ev.resolved:
                resolved_durations.append(ev.duration_min())

        mttr = (sum(resolved_durations) / len(resolved_durations)
                if resolved_durations else 0.0)

        # Top reasons sorted by total duration descending
        top_reasons: List[Tuple[str, int, float]] = sorted(
            [(reason, len(durs), sum(durs))
             for reason, durs in reason_map.items()],
            key=lambda t: t[2],
            reverse=True,
        )

        return DowntimeSummary(
            total_downtime_min=total,
            planned_min=planned,
            unplanned_min=unplanned,
            by_category=by_cat,
            by_machine=by_machine,
            event_count=len(events),
            mttr_min=mttr,
            top_reasons=top_reasons,
        )

    def get_pareto_reasons(
        self, top_n: int = 5,
    ) -> List[Tuple[str, int, float]]:
        """Return the top *top_n* downtime reasons by total duration.

        Each entry is ``(reason, occurrence_count, total_duration_min)``.
        """
        with self._lock:
            reason_map: Dict[str, List[float]] = {}
            for ev in self._events.values():
                dur = ev.duration_min()
                reason_map.setdefault(ev.reason, []).append(dur)

        ranked = sorted(
            [(reason, len(durs), sum(durs))
             for reason, durs in reason_map.items()],
            key=lambda t: t[2],
            reverse=True,
        )
        return ranked[:top_n]

    def calculate_availability(
        self,
        machine_id: str,
        planned_production_min: float,
        start_time: float,
        end_time: float,
    ) -> float:
        """Compute the OEE availability component for *machine_id*.

        availability = (planned_production_min - downtime) / planned_production_min

        Returns a value clamped to [0.0, 1.0].  If *planned_production_min*
        is zero the result is 1.0 (no planned time => full availability).
        """
        if planned_production_min <= 0.0:
            return 1.0

        with self._lock:
            events = self._events_in_range(start_time, end_time)

        downtime = sum(
            self._clipped_duration_min(ev, start_time, end_time)
            for ev in events if ev.machine_id == machine_id
        )

        availability = (planned_production_min - downtime) / planned_production_min
        return max(0.0, min(1.0, availability))


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
