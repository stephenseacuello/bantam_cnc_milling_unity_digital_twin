"""
Job Scheduler Node.

Manages manufacturing job queue with priority scheduling.
Assigns jobs to machines based on capability, availability, and optimization.
Supports priority levels: LOW, NORMAL, HIGH, RUSH.

Alarm escalation integration: subscribes to alarm escalation events and
automatically pauses or reassigns jobs on affected machines.
"""

from typing import Any, Dict, List, Optional, Set, Tuple
from dataclasses import dataclass, field
from enum import IntEnum
import copy
import math
import threading
import time
import uuid
import heapq

from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import JobStatus, MachineState, TaskAnnouncement
from miracle_msgs.action import ExecuteJob
from miracle_msgs.srv import SubmitTask
from miracle_mes.digital_thread import DigitalThreadNode

try:
    from miracle_msgs.msg import AlarmEscalation
except ImportError:  # pragma: no cover
    AlarmEscalation = None  # type: ignore[assignment,misc]


class Priority(IntEnum):
    """Job priority levels (lower value = higher priority)."""
    RUSH = 0
    HIGH = 1
    NORMAL = 2
    LOW = 3


@dataclass(order=True)
class Job:
    """Manufacturing job entry."""
    priority: int
    submit_time: float
    job_id: str = field(compare=False)
    program_name: str = field(compare=False)
    machine_id: str = field(compare=False, default='')
    material: str = field(compare=False, default='steel')
    status: str = field(compare=False, default='QUEUED')
    task_type: str = field(compare=False, default='MILLING')
    required_capabilities: List[str] = field(compare=False, default_factory=list)
    material_batch: str = field(compare=False, default='')
    part_serial: str = field(compare=False, default='')
    tool_id: str = field(compare=False, default='')
    due_date: float = field(compare=False, default=0.0)  # unix timestamp, 0 = no deadline
    fixture_type: str = field(compare=False, default='')
    required_tools: List[str] = field(compare=False, default_factory=list)
    estimated_duration_sec: float = field(compare=False, default=3600.0)
    material_status: str = field(compare=False, default='LOADED')  # LOADED, QUEUED, NOT_AVAILABLE
    compatible_machines: List[str] = field(compare=False, default_factory=list)


@dataclass
class JobPriority:
    """Computed priority score for intelligent queue optimization."""
    base_priority: int  # 1=highest (RUSH), maps from Priority enum
    due_date_urgency: float  # 0-1, exponential as deadline approaches
    setup_affinity: float  # 0-1, how well job matches current setup
    tool_availability: float  # 0-1, based on RUL of required tools
    material_readiness: float  # 0-1, is material loaded?
    machine_suitability: float  # 0-1, best machine for this job
    composite_score: float  # weighted sum (higher = schedule sooner)


class JobQueueOptimizer:
    """Intelligent job queue priority optimizer.

    Computes composite priority scores considering urgency, setup affinity,
    tool availability, material readiness, and machine suitability.  Groups
    jobs with the same fixture/tooling to minimise setup changes.
    """

    def __init__(
        self,
        urgency_weight: float = 0.30,
        setup_weight: float = 0.25,
        tool_weight: float = 0.20,
        material_weight: float = 0.15,
        machine_weight: float = 0.10,
    ) -> None:
        self.urgency_weight = urgency_weight
        self.setup_weight = setup_weight
        self.tool_weight = tool_weight
        self.material_weight = material_weight
        self.machine_weight = machine_weight

    # ------------------------------------------------------------------
    # Priority computation
    # ------------------------------------------------------------------

    def compute_priority(
        self,
        job: 'Job',
        current_setup: Dict[str, Any],
        tool_states: Dict[str, float],
        current_time: float,
    ) -> JobPriority:
        """Compute a composite priority for a single job.

        Args:
            job: The job to evaluate.
            current_setup: Dict with keys like 'fixture_type', 'tooling',
                           'machine_id' describing the current machine setup.
            tool_states: Mapping of tool_id -> remaining useful life (minutes).
            current_time: Current unix timestamp.

        Returns:
            A JobPriority with all factor scores and the composite.
        """
        base_priority = max(1, job.priority + 1)  # Priority enum 0-3 -> 1-4

        # --- Due-date urgency (exponential as deadline approaches) ---
        due_date_urgency = self._compute_due_date_urgency(job, current_time)

        # --- Setup affinity ---
        setup_affinity = self._compute_setup_affinity(job, current_setup)

        # --- Tool availability ---
        tool_availability = self._compute_tool_availability(job, tool_states)

        # --- Material readiness ---
        material_readiness = self._compute_material_readiness(job)

        # --- Machine suitability ---
        machine_suitability = self._compute_machine_suitability(job, current_setup)

        # --- Composite score (higher = schedule sooner) ---
        # Base priority contribution: RUSH(1)->1.0, HIGH(2)->0.75, NORMAL(3)->0.5, LOW(4)->0.25
        base_score = max(0.0, 1.0 - (base_priority - 1) * 0.25)

        composite_score = (
            base_score * 0.5  # base priority always has significant weight
            + due_date_urgency * self.urgency_weight
            + setup_affinity * self.setup_weight
            + tool_availability * self.tool_weight
            + material_readiness * self.material_weight
            + machine_suitability * self.machine_weight
        )

        return JobPriority(
            base_priority=base_priority,
            due_date_urgency=due_date_urgency,
            setup_affinity=setup_affinity,
            tool_availability=tool_availability,
            material_readiness=material_readiness,
            machine_suitability=machine_suitability,
            composite_score=composite_score,
        )

    def _compute_due_date_urgency(self, job: 'Job', current_time: float) -> float:
        """Exponential urgency as deadline approaches. Overdue -> 1.0."""
        if job.due_date <= 0:
            return 0.0  # no deadline
        time_remaining = job.due_date - current_time
        if time_remaining <= 0:
            return 1.0  # overdue
        # Exponential decay: urgency increases as deadline nears
        # At 24h out ~ 0.0, at 1h out ~ 0.63, at 0h -> 1.0
        hours_remaining = time_remaining / 3600.0
        return min(1.0, 1.0 - math.exp(-1.0 / max(hours_remaining, 0.001)))

    def _compute_setup_affinity(
        self, job: 'Job', current_setup: Dict[str, Any],
    ) -> float:
        """1.0 if same fixture/tooling, decreasing with setup changes needed."""
        if not current_setup:
            return 0.5  # neutral when no setup info
        score = 0.0
        checks = 0

        # Fixture match
        current_fixture = current_setup.get('fixture_type', '')
        if current_fixture and job.fixture_type:
            checks += 1
            if job.fixture_type == current_fixture:
                score += 1.0
            else:
                score += 0.0

        # Tooling overlap
        current_tools = set(current_setup.get('tooling', []))
        if current_tools and job.required_tools:
            checks += 1
            required = set(job.required_tools)
            overlap = len(current_tools & required) / max(len(required), 1)
            score += overlap

        if checks == 0:
            return 0.5
        return score / checks

    def _compute_tool_availability(
        self, job: 'Job', tool_states: Dict[str, float],
    ) -> float:
        """Based on RUL of required tools.  Low RUL -> low score."""
        if not job.required_tools:
            return 1.0  # no specific tools needed
        if not tool_states:
            return 0.5  # unknown state

        scores: List[float] = []
        for tool_id in job.required_tools:
            rul = tool_states.get(tool_id)
            if rul is None:
                scores.append(0.5)  # unknown
            elif rul <= 0:
                scores.append(0.0)  # tool dead
            elif rul >= 60:
                scores.append(1.0)  # plenty of life
            else:
                scores.append(rul / 60.0)  # linear scale 0-60 min
        return sum(scores) / len(scores)

    def _compute_material_readiness(self, job: 'Job') -> float:
        """1.0 if loaded, 0.5 if queued, 0.0 if not available."""
        status = getattr(job, 'material_status', 'LOADED')
        mapping = {'LOADED': 1.0, 'QUEUED': 0.5, 'NOT_AVAILABLE': 0.0}
        return mapping.get(status, 0.0)

    def _compute_machine_suitability(
        self, job: 'Job', current_setup: Dict[str, Any],
    ) -> float:
        """1.0 if the current machine is in the job's compatible list."""
        if not job.compatible_machines:
            return 1.0  # no preference
        machine_id = current_setup.get('machine_id', '')
        if not machine_id:
            return 0.5
        return 1.0 if machine_id in job.compatible_machines else 0.3

    # ------------------------------------------------------------------
    # Queue optimization
    # ------------------------------------------------------------------

    def optimize_queue(
        self,
        jobs: List['Job'],
        current_setup: Dict[str, Any],
        tool_states: Dict[str, float],
        current_time: float,
    ) -> List['Job']:
        """Reorder jobs by composite priority with setup batching.

        Args:
            jobs: Unordered list of queued jobs.
            current_setup: Current machine setup context.
            tool_states: tool_id -> RUL mapping.
            current_time: Unix timestamp.

        Returns:
            Reordered list (highest priority first).
        """
        if not jobs:
            return []

        # Compute priorities
        scored: List[Tuple[float, int, 'Job', JobPriority]] = []
        for idx, job in enumerate(jobs):
            jp = self.compute_priority(job, current_setup, tool_states, current_time)
            # Negate composite so highest score sorts first; idx for stability
            scored.append((-jp.composite_score, idx, job, jp))

        scored.sort()

        # --- Setup batching ---
        # Group jobs with the same fixture together, keeping priority order
        # within each group, then interleave groups starting with the one
        # matching the current setup.
        fixture_groups: Dict[str, List[Tuple[float, int, 'Job', JobPriority]]] = {}
        no_fixture: List[Tuple[float, int, 'Job', JobPriority]] = []
        for entry in scored:
            fixture = entry[2].fixture_type
            if fixture:
                fixture_groups.setdefault(fixture, []).append(entry)
            else:
                no_fixture.append(entry)

        result: List['Job'] = []
        current_fixture = current_setup.get('fixture_type', '')

        # Start with current fixture group
        if current_fixture and current_fixture in fixture_groups:
            for entry in fixture_groups.pop(current_fixture):
                result.append(entry[2])

        # Then remaining fixture groups in priority order
        remaining_groups = sorted(
            fixture_groups.values(),
            key=lambda g: g[0][0],  # sort by best score in group
        )
        for group in remaining_groups:
            for entry in group:
                result.append(entry[2])

        # Finally jobs with no fixture preference
        for entry in no_fixture:
            result.append(entry[2])

        return result

    # ------------------------------------------------------------------
    # Completion time estimation
    # ------------------------------------------------------------------

    def estimate_completion_times(
        self,
        ordered_jobs: List['Job'],
        current_time: float,
    ) -> Dict[str, float]:
        """Estimate completion timestamps for an ordered job list.

        Args:
            ordered_jobs: Jobs in execution order.
            current_time: Start time (unix timestamp).

        Returns:
            Mapping of job_id -> estimated completion timestamp.
        """
        estimates: Dict[str, float] = {}
        clock = current_time
        prev_fixture = ''
        for job in ordered_jobs:
            # Setup change penalty: 15 minutes if fixture changes
            if job.fixture_type and prev_fixture and job.fixture_type != prev_fixture:
                clock += 15 * 60  # 15 min setup change
            clock += job.estimated_duration_sec
            estimates[job.job_id] = clock
            if job.fixture_type:
                prev_fixture = job.fixture_type
        return estimates

    # ------------------------------------------------------------------
    # Bottleneck identification
    # ------------------------------------------------------------------

    def identify_bottlenecks(
        self,
        jobs: List['Job'],
        machines: List[str],
    ) -> List[Tuple[str, int, float]]:
        """Identify resource bottlenecks (machines with high contention).

        Args:
            jobs: All queued/active jobs.
            machines: Available machine IDs.

        Returns:
            List of (resource, contention_count, delay_impact_sec) sorted
            by delay impact descending.
        """
        if not jobs or not machines:
            return []

        # Count how many jobs target each machine
        machine_contention: Dict[str, int] = {m: 0 for m in machines}
        machine_load: Dict[str, float] = {m: 0.0 for m in machines}

        for job in jobs:
            targets = job.compatible_machines if job.compatible_machines else machines
            for m in targets:
                if m in machine_contention:
                    machine_contention[m] += 1
                    machine_load[m] += job.estimated_duration_sec

        # Also track tool contention
        tool_demand: Dict[str, int] = {}
        tool_load: Dict[str, float] = {}
        for job in jobs:
            for tool_id in job.required_tools:
                tool_demand[tool_id] = tool_demand.get(tool_id, 0) + 1
                tool_load[tool_id] = tool_load.get(tool_id, 0.0) + job.estimated_duration_sec

        bottlenecks: List[Tuple[str, int, float]] = []

        for m in machines:
            if machine_contention[m] > 1:
                # Delay impact = total load beyond first job
                delay = max(0.0, machine_load[m] - (
                    machine_load[m] / machine_contention[m]
                ))
                bottlenecks.append((m, machine_contention[m], delay))

        for tool_id, count in tool_demand.items():
            if count > 1:
                delay = max(0.0, tool_load[tool_id] - (
                    tool_load[tool_id] / count
                ))
                bottlenecks.append((tool_id, count, delay))

        bottlenecks.sort(key=lambda x: x[2], reverse=True)
        return bottlenecks

    # ------------------------------------------------------------------
    # Parallel job suggestion
    # ------------------------------------------------------------------

    def suggest_parallel_jobs(
        self,
        jobs: List['Job'],
        available_machines: List[str],
    ) -> List[Tuple[str, str]]:
        """Suggest (machine_id, job_id) assignments for parallel execution.

        Assigns the highest-priority unassigned job to each available
        machine, respecting machine compatibility constraints.

        Args:
            jobs: Candidate jobs (should be priority-sorted).
            available_machines: Machines currently idle.

        Returns:
            List of (machine_id, job_id) tuples.
        """
        if not jobs or not available_machines:
            return []

        assignments: List[Tuple[str, str]] = []
        assigned_jobs: Set[str] = set()
        used_machines: Set[str] = set()

        for job in jobs:
            if job.job_id in assigned_jobs:
                continue
            compatible = (
                [m for m in available_machines
                 if m not in used_machines and m in job.compatible_machines]
                if job.compatible_machines
                else [m for m in available_machines if m not in used_machines]
            )
            if compatible:
                machine = compatible[0]
                assignments.append((machine, job.job_id))
                assigned_jobs.add(job.job_id)
                used_machines.add(machine)
            if len(used_machines) >= len(available_machines):
                break

        return assignments


@dataclass
class MaintenanceWindow:
    """A scheduled maintenance window for a machine."""
    window_id: str
    machine_id: str
    maintenance_type: str  # TOOL_CHANGE, CALIBRATION, COOLANT_REFILL, SPINDLE_SERVICE, PREVENTIVE
    scheduled_start: float  # timestamp
    estimated_duration_min: float
    priority: int  # 1=critical, 5=routine
    triggered_by: str  # prediction, schedule, operator
    tool_id: str = ''
    rul_at_scheduling: float = 0.0  # minutes remaining
    status: str = 'PENDING'  # PENDING, IN_PROGRESS, COMPLETED, CANCELLED


class MaintenanceScheduler:
    """Predictive maintenance scheduler integrated with the job scheduler.

    Manages a priority queue of maintenance windows sorted by (priority, scheduled_start).
    Automatically schedules maintenance based on tool RUL predictions and calibration drift.
    """

    # Thresholds
    RUL_THRESHOLD_MIN: float = 30.0  # auto-schedule tool change below this
    CALIBRATION_DRIFT_THRESHOLD_PCT: float = 8.0  # auto-schedule calibration above this
    URGENT_INTERRUPT_THRESHOLD_MIN: float = 10.0  # interrupt current job if below this

    VALID_MAINTENANCE_TYPES = {
        'TOOL_CHANGE', 'CALIBRATION', 'COOLANT_REFILL',
        'SPINDLE_SERVICE', 'PREVENTIVE',
    }

    # Default durations by type (minutes)
    DEFAULT_DURATIONS = {
        'TOOL_CHANGE': 15.0,
        'CALIBRATION': 30.0,
        'COOLANT_REFILL': 10.0,
        'SPINDLE_SERVICE': 60.0,
        'PREVENTIVE': 45.0,
    }

    def __init__(self) -> None:
        self._queue: List[Tuple[int, float, str, MaintenanceWindow]] = []  # heapq: (priority, time, id, window)
        self._windows: Dict[str, MaintenanceWindow] = {}
        self._completed: List[MaintenanceWindow] = []
        self._lock = threading.Lock()
        self._wear_rates: Dict[str, Dict[str, float]] = {}  # machine_id -> {tool_id: wear_rate_per_min}
        self._last_rul: Dict[str, Dict[str, float]] = {}  # machine_id -> {tool_id: rul_minutes}
        self._last_drift: Dict[str, float] = {}  # machine_id -> drift_pct
        self._scheduled_keys: Set[str] = set()  # dedup key: "machine_id:type:tool_id"
        # Reference to parent scheduler node (set after construction)
        self._node: Any = None

    def schedule_maintenance(
        self,
        machine_id: str,
        maintenance_type: str,
        urgency_minutes: float = 60.0,
        **kwargs: Any,
    ) -> MaintenanceWindow:
        """Schedule a maintenance window for a machine.

        Args:
            machine_id: Target machine.
            maintenance_type: One of TOOL_CHANGE, CALIBRATION, COOLANT_REFILL, SPINDLE_SERVICE, PREVENTIVE.
            urgency_minutes: How soon maintenance is needed (minutes from now).
            **kwargs: Optional overrides - priority, triggered_by, tool_id, rul_at_scheduling,
                      estimated_duration_min.

        Returns:
            The created MaintenanceWindow.
        """
        now = time.time()
        priority = kwargs.get('priority', 5 if urgency_minutes > 60 else (1 if urgency_minutes < 10 else 3))
        triggered_by = kwargs.get('triggered_by', 'schedule')
        tool_id = kwargs.get('tool_id', '')
        rul_at_scheduling = kwargs.get('rul_at_scheduling', 0.0)
        duration = kwargs.get(
            'estimated_duration_min',
            self.DEFAULT_DURATIONS.get(maintenance_type, 30.0),
        )

        window = MaintenanceWindow(
            window_id=str(uuid.uuid4())[:12],
            machine_id=machine_id,
            maintenance_type=maintenance_type,
            scheduled_start=now + urgency_minutes * 60,
            estimated_duration_min=duration,
            priority=priority,
            triggered_by=triggered_by,
            tool_id=tool_id,
            rul_at_scheduling=rul_at_scheduling,
        )

        dedup_key = f"{machine_id}:{maintenance_type}:{tool_id}"

        with self._lock:
            # Prevent duplicate scheduling for the same machine/type/tool
            if dedup_key in self._scheduled_keys:
                # Return existing window
                for _, _, _, w in self._queue:
                    if (w.machine_id == machine_id
                            and w.maintenance_type == maintenance_type
                            and w.tool_id == tool_id
                            and w.status == 'PENDING'):
                        return w

            self._scheduled_keys.add(dedup_key)
            self._windows[window.window_id] = window
            heapq.heappush(self._queue, (window.priority, window.scheduled_start, window.window_id, window))

        # Handle urgent maintenance: interrupt current job if needed
        if urgency_minutes < self.URGENT_INTERRUPT_THRESHOLD_MIN and self._node:
            job = self._node._find_active_job_on_machine(machine_id)
            if job and job.status in ('ASSIGNED', 'RUNNING'):
                self._node.pause_job(job.job_id)
                window.status = 'IN_PROGRESS'

        return window

    def check_tool_rul(self, machine_id: str, tool_id: str, rul_minutes: float) -> Optional[MaintenanceWindow]:
        """Check tool remaining useful life and auto-schedule TOOL_CHANGE if needed.

        Args:
            machine_id: The machine running the tool.
            tool_id: The tool identifier.
            rul_minutes: Remaining useful life in minutes.

        Returns:
            A MaintenanceWindow if one was created, else None.
        """
        # Track for wear rate estimation
        if machine_id not in self._last_rul:
            self._last_rul[machine_id] = {}
        prev = self._last_rul[machine_id].get(tool_id)
        self._last_rul[machine_id][tool_id] = rul_minutes

        # Estimate wear rate
        if prev is not None and prev > rul_minutes:
            if machine_id not in self._wear_rates:
                self._wear_rates[machine_id] = {}
            self._wear_rates[machine_id][tool_id] = prev - rul_minutes  # per check interval

        if rul_minutes < self.RUL_THRESHOLD_MIN:
            return self.schedule_maintenance(
                machine_id=machine_id,
                maintenance_type='TOOL_CHANGE',
                urgency_minutes=max(rul_minutes, 1.0),
                priority=1 if rul_minutes < 10 else 2,
                triggered_by='prediction',
                tool_id=tool_id,
                rul_at_scheduling=rul_minutes,
            )
        return None

    def check_calibration_drift(self, machine_id: str, drift_pct: float) -> Optional[MaintenanceWindow]:
        """Check calibration drift and auto-schedule CALIBRATION if needed.

        Args:
            machine_id: The machine to check.
            drift_pct: Current calibration drift percentage.

        Returns:
            A MaintenanceWindow if one was created, else None.
        """
        self._last_drift[machine_id] = drift_pct

        if drift_pct > self.CALIBRATION_DRIFT_THRESHOLD_PCT:
            urgency = max(5.0, 60.0 - (drift_pct - self.CALIBRATION_DRIFT_THRESHOLD_PCT) * 5)
            return self.schedule_maintenance(
                machine_id=machine_id,
                maintenance_type='CALIBRATION',
                urgency_minutes=urgency,
                priority=2 if drift_pct > 15 else 3,
                triggered_by='prediction',
            )
        return None

    def get_maintenance_queue(self, machine_id: Optional[str] = None) -> List[MaintenanceWindow]:
        """Get sorted list of pending maintenance windows.

        Args:
            machine_id: Filter by machine. If None, returns all.

        Returns:
            List of MaintenanceWindow sorted by priority then scheduled_start.
        """
        with self._lock:
            pending = [
                w for _, _, _, w in sorted(self._queue)
                if w.status == 'PENDING'
                and (machine_id is None or w.machine_id == machine_id)
            ]
        return pending

    def complete_maintenance(self, window_id: str) -> bool:
        """Mark a maintenance window as completed.

        Args:
            window_id: The window to complete.

        Returns:
            True if the window was found and completed.
        """
        with self._lock:
            window = self._windows.get(window_id)
            if not window or window.status in ('COMPLETED', 'CANCELLED'):
                return False
            window.status = 'COMPLETED'
            dedup_key = f"{window.machine_id}:{window.maintenance_type}:{window.tool_id}"
            self._scheduled_keys.discard(dedup_key)
            self._completed.append(window)

        # Record in digital thread if available
        if self._node and self._node._digital_thread:
            self._node._digital_thread.record_genealogy_event(
                'MAINTENANCE_COMPLETE',
                machine_id=window.machine_id,
                tool_id=window.tool_id,
                metadata={
                    'window_id': window.window_id,
                    'maintenance_type': window.maintenance_type,
                    'triggered_by': window.triggered_by,
                    'duration_min': window.estimated_duration_min,
                },
            )

        # Resume paused job if the node interrupted one
        if self._node:
            job = self._node._find_active_job_on_machine(window.machine_id)
            if job and job.status == 'PAUSED':
                self._node.resume_job(job.job_id)

        return True

    def cancel_maintenance(self, window_id: str) -> bool:
        """Cancel a pending maintenance window.

        Args:
            window_id: The window to cancel.

        Returns:
            True if the window was found and cancelled.
        """
        with self._lock:
            window = self._windows.get(window_id)
            if not window or window.status != 'PENDING':
                return False
            window.status = 'CANCELLED'
            dedup_key = f"{window.machine_id}:{window.maintenance_type}:{window.tool_id}"
            self._scheduled_keys.discard(dedup_key)
        return True

    def get_maintenance_forecast(self, hours_ahead: float = 24.0) -> List[Dict[str, Any]]:
        """Predict maintenance needs based on current wear rates and drift.

        Args:
            hours_ahead: How far ahead to forecast (hours).

        Returns:
            List of dicts with predicted maintenance needs.
        """
        forecasts: List[Dict[str, Any]] = []
        minutes_ahead = hours_ahead * 60.0

        # Tool wear forecasts
        for machine_id, tools in self._last_rul.items():
            rates = self._wear_rates.get(machine_id, {})
            for tool_id, rul in tools.items():
                if rul <= 0:
                    continue
                rate = rates.get(tool_id, 0)
                if rate > 0:
                    time_to_failure = rul  # already in minutes
                    if time_to_failure < minutes_ahead:
                        forecasts.append({
                            'machine_id': machine_id,
                            'maintenance_type': 'TOOL_CHANGE',
                            'tool_id': tool_id,
                            'predicted_minutes_until_needed': time_to_failure,
                            'confidence': min(0.95, 0.5 + rate * 0.1),
                            'source': 'wear_rate_model',
                        })
                elif rul < minutes_ahead:
                    # No rate data but RUL is within window
                    forecasts.append({
                        'machine_id': machine_id,
                        'maintenance_type': 'TOOL_CHANGE',
                        'tool_id': tool_id,
                        'predicted_minutes_until_needed': rul,
                        'confidence': 0.5,
                        'source': 'rul_estimate',
                    })

        # Calibration drift forecasts
        for machine_id, drift in self._last_drift.items():
            if drift > self.CALIBRATION_DRIFT_THRESHOLD_PCT * 0.7:
                # Extrapolate drift growth
                time_to_threshold = max(
                    0,
                    (self.CALIBRATION_DRIFT_THRESHOLD_PCT - drift) / max(drift * 0.01, 0.001),
                )
                if time_to_threshold < minutes_ahead or drift > self.CALIBRATION_DRIFT_THRESHOLD_PCT:
                    forecasts.append({
                        'machine_id': machine_id,
                        'maintenance_type': 'CALIBRATION',
                        'drift_pct': drift,
                        'predicted_minutes_until_needed': time_to_threshold if drift <= self.CALIBRATION_DRIFT_THRESHOLD_PCT else 0,
                        'confidence': 0.7,
                        'source': 'drift_extrapolation',
                    })

        # Sort by urgency
        forecasts.sort(key=lambda f: f.get('predicted_minutes_until_needed', float('inf')))
        return forecasts


class JobSchedulerNode(MiracleLifecycleNode):
    """Priority-based job scheduler with machine assignment.

    Parameters:
        max_queue_size (int): Maximum jobs in queue.
        scheduling_interval_sec (float): Scheduling cycle interval.
        enable_auction (bool): Use auction-based task allocation.

    Service Servers:
        ~/submit_task (SubmitTask): Submit a new manufacturing task.

    Action Servers:
        ~/execute_job (ExecuteJob): Execute a manufacturing job.

    Published Topics:
        ~/job_status (JobStatus): Job status updates.
        ~/task_announcements (TaskAnnouncement): Auction announcements.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'job_scheduler',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._job_queue: List[Job] = []
        self._active_jobs: Dict[str, Job] = {}
        self._completed_jobs: List[Job] = []
        self._queue_lock = threading.Lock()

        self._submit_srv = None
        self._action_server = None
        self._status_pub = None
        self._announcement_pub = None
        self._schedule_timer = None
        self._state_subs = None

        self._machine_states: Dict[str, MachineState] = {}
        self._max_queue: int = 1000
        self._enable_auction: bool = False
        self._digital_thread: Optional[DigitalThreadNode] = None
        self._blocked_machines: Set[str] = set()
        self._alarm_escalation_sub = None
        self._job_history: List[Dict[str, Any]] = []

        # Predictive maintenance scheduler
        self.maintenance_scheduler = MaintenanceScheduler()
        self.maintenance_scheduler._node = self

        # Intelligent queue optimizer
        self._queue_optimizer = JobQueueOptimizer()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure job scheduler."""
        params = self.declare_and_validate_parameters({
            'max_queue_size': {
                'default': 1000,
                'type': int,
                'range': (10, 100000),
            },
            'scheduling_interval_sec': {
                'default': 1.0,
                'type': float,
                'range': (0.1, 60.0),
            },
            'enable_auction': {
                'default': False,
                'type': bool,
            },
            'machine_ids': {
                'default': 'cnc1,cnc2,cnc3',
                'type': str,
            },
        })

        machine_ids = self.get_machine_ids(params)

        self._max_queue = params['max_queue_size']
        self._enable_auction = params['enable_auction']

        self._submit_srv = self.create_service(
            SubmitTask,
            'submit_task',
            self._handle_submit,
            callback_group=self.service_callback_group,
        )

        self._action_server = ActionServer(
            self,
            ExecuteJob,
            'execute_job',
            execute_callback=self._execute_job,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self._status_pub = self.create_publisher(
            JobStatus,
            'job_status',
            QoSProfiles.state_data(),
        )

        self._announcement_pub = self.create_publisher(
            TaskAnnouncement,
            '/miracle/cognitive/task_announcements',
            QoSProfiles.command(),
        )

        self._state_subs = self.create_multi_machine_subscriptions(
            MachineState,
            'state',
            self._on_machine_state,
            QoSProfiles.state_data(),
            machine_ids,
        )

        # Subscribe to alarm escalation events for automatic job pausing
        if AlarmEscalation is not None:
            self._alarm_escalation_sub = self.create_subscription(
                AlarmEscalation,
                '/miracle/scada/alarm_escalations',
                self._on_alarm_escalation,
                QoSProfiles.alert(),
            )

        self.get_logger().info("Job scheduler configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate scheduling."""
        interval = self.get_parameter('scheduling_interval_sec').value
        self._schedule_timer = self.create_timer(
            interval,
            self._scheduling_cycle,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("Job scheduler activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate scheduling."""
        if self._schedule_timer is not None:
            self._schedule_timer.cancel()
            self._schedule_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_machine_state(self, msg: MachineState) -> None:
        """Track machine states for scheduling decisions."""
        self._machine_states[msg.machine_id] = msg

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Accept job execution goals."""
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Accept cancellation of job execution."""
        return CancelResponse.ACCEPT

    def _handle_submit(
        self,
        request: SubmitTask.Request,
        response: SubmitTask.Response,
    ) -> SubmitTask.Response:
        """Handle task submission."""
        with self._queue_lock:
            if len(self._job_queue) >= self._max_queue:
                response.accepted = False
                response.message = 'Queue is full'
                response.auction_id = ''
                return response

            priority_map = {
                'RUSH': Priority.RUSH,
                'HIGH': Priority.HIGH,
                'NORMAL': Priority.NORMAL,
                'LOW': Priority.LOW,
            }

            job = Job(
                priority=priority_map.get(request.priority, Priority.NORMAL),
                submit_time=self.get_clock().now().nanoseconds / 1e9,
                job_id=request.job_id or str(uuid.uuid4())[:8],
                program_name='',
                material=request.material,
                task_type=request.task_type,
                required_capabilities=list(request.required_capabilities),
            )

            heapq.heappush(self._job_queue, job)

            response.accepted = True
            response.auction_id = job.job_id
            response.message = f'Job {job.job_id} queued (priority={request.priority})'

            self.get_logger().info(
                f"Job submitted: {job.job_id} (priority={request.priority})"
            )

        return response

    def _scheduling_cycle(self) -> None:
        """Run one scheduling cycle."""
        with self._queue_lock:
            if not self._job_queue:
                return

            # Find idle machines, excluding blocked ones
            idle_machines = [
                mid for mid, state in self._machine_states.items()
                if state.status in ('IDLE', 'READY')
                and mid not in self._blocked_machines
            ]

            # Log skipped blocked machines
            blocked_idle = [
                mid for mid, state in self._machine_states.items()
                if state.status in ('IDLE', 'READY')
                and mid in self._blocked_machines
            ]
            for mid in blocked_idle:
                self.get_logger().warn(
                    f"Machine {mid} skipped for scheduling: "
                    f"blocked by alarm escalation"
                )

            if not idle_machines:
                return

            # Assign highest priority job to first available machine
            job = heapq.heappop(self._job_queue)
            machine_id = idle_machines[0]
            job.machine_id = machine_id
            job.status = 'ASSIGNED'
            self._active_jobs[job.job_id] = job

            self.get_logger().info(
                f"Job {job.job_id} assigned to {machine_id}"
            )

            self._publish_job_status(job)

            if self._digital_thread:
                self._digital_thread.record_genealogy_event(
                    DigitalThreadNode.ENTRY_MATERIAL_LOADED,
                    machine_id=job.machine_id,
                    batch_id=job.material_batch or '',
                    serial_number=job.part_serial or '',
                    metadata={'job_id': job.job_id, 'program': job.program_name},
                )
                # Record tool installation for the job
                if job.tool_id:
                    self._digital_thread.record_genealogy_event(
                        DigitalThreadNode.ENTRY_TOOL_INSTALLED,
                        machine_id=job.machine_id,
                        tool_id=job.tool_id,
                        serial_number=job.part_serial or '',
                        batch_id=job.material_batch or '',
                        metadata={'job_id': job.job_id},
                    )
                # Set active job context for anomaly linking
                self._digital_thread.set_active_job(
                    machine_id=job.machine_id,
                    job_id=job.job_id,
                    serial_number=job.part_serial or '',
                    batch_id=job.material_batch or '',
                )

            if self._enable_auction:
                self._announce_task(job)

    def _announce_task(self, job: Job) -> None:
        """Announce task for multi-agent auction."""
        msg = TaskAnnouncement()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.auction_id = job.job_id
        msg.task_type = job.task_type
        msg.job_id = job.job_id
        msg.material = job.material
        msg.complexity = 0.5
        msg.required_capabilities = job.required_capabilities
        msg.estimated_duration = 3600.0
        msg.priority = ['RUSH', 'HIGH', 'NORMAL', 'LOW'][job.priority]

        self._announcement_pub.publish(msg)

    async def _execute_job(
        self, goal_handle: ServerGoalHandle
    ) -> ExecuteJob.Result:
        """Execute a manufacturing job."""
        request = goal_handle.request
        result = ExecuteJob.Result()

        self.get_logger().info(
            f"Executing job '{request.job_id}' on {request.machine_id}"
        )

        job = self._active_jobs.get(request.job_id)

        # Simulate job execution phases
        phases = [
            ('SETUP', 0.1),
            ('LOADING', 0.2),
            ('MACHINING', 0.8),
            ('INSPECTION', 0.9),
            ('COMPLETE', 1.0),
        ]

        import asyncio
        start = self.get_clock().now()

        for phase_name, progress in phases:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Job cancelled'
                # Record cancellation in genealogy
                if self._digital_thread:
                    self._digital_thread.record_genealogy_event(
                        DigitalThreadNode.ENTRY_JOB_CANCELLED,
                        machine_id=request.machine_id,
                        serial_number=job.part_serial if job else '',
                        batch_id=job.material_batch if job else '',
                        tool_id=job.tool_id if job else '',
                        metadata={
                            'job_id': request.job_id,
                            'cancelled_at_phase': phase_name,
                        },
                    )
                    self._digital_thread.clear_active_job(request.machine_id)
                return result

            phase_start = self.get_clock().now().nanoseconds / 1e9

            feedback = ExecuteJob.Feedback()
            feedback.progress = progress
            feedback.current_phase = phase_name
            feedback.current_operation = f'{phase_name} phase'
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            feedback.elapsed_sec = elapsed
            feedback.estimated_remaining_sec = max(0.0, 60.0 * (1.0 - progress))

            state_msg = MachineState()
            state_msg.timestamp = self.get_clock().now().to_msg()
            state_msg.machine_id = request.machine_id
            state_msg.status = 'RUNNING'
            feedback.machine_state = state_msg

            goal_handle.publish_feedback(feedback)
            await asyncio.sleep(0.5)

            phase_end = self.get_clock().now().nanoseconds / 1e9

            # Record OPERATION_COMPLETE for cutting phases
            if self._digital_thread and phase_name in ('MACHINING',):
                self._digital_thread.record_operation_complete(
                    machine_id=request.machine_id,
                    tool_id=job.tool_id if job else '',
                    operation_type=job.task_type if job else 'MILLING',
                    start_time=phase_start,
                    end_time=phase_end,
                    serial_number=job.part_serial if job else '',
                    batch_id=job.material_batch if job else '',
                    metadata={'job_id': request.job_id, 'phase': phase_name},
                )

        elapsed_total = (self.get_clock().now() - start).nanoseconds / 1e9
        goal_handle.succeed()
        result.success = True
        result.message = 'Job completed successfully'
        result.total_time_sec = elapsed_total
        result.oee_achieved = 0.85
        result.quality_metrics = [0.98, 0.95, 0.99]

        if self._digital_thread:
            # Record tool removal on job completion
            if job and job.tool_id:
                self._digital_thread.record_genealogy_event(
                    DigitalThreadNode.ENTRY_TOOL_REMOVED,
                    machine_id=request.machine_id,
                    tool_id=job.tool_id,
                    serial_number=job.part_serial if job else '',
                    batch_id=job.material_batch if job else '',
                    metadata={'job_id': request.job_id},
                )

            self._digital_thread.record_genealogy_event(
                DigitalThreadNode.ENTRY_PART_COMPLETE,
                machine_id=request.machine_id,
                serial_number=job.part_serial if job else '',
                batch_id=job.material_batch if job else '',
                metadata={
                    'job_id': request.job_id,
                    'total_time_sec': elapsed_total,
                    'oee_achieved': result.oee_achieved,
                },
            )

            self._digital_thread.clear_active_job(request.machine_id)

        return result

    def pause_job(self, job_id: str) -> bool:
        """Pause an active job and record in genealogy."""
        job = self._active_jobs.get(job_id)
        if not job:
            return False
        job.status = 'PAUSED'
        self._publish_job_status(job)
        if self._digital_thread:
            self._digital_thread.record_genealogy_event(
                DigitalThreadNode.ENTRY_JOB_PAUSED,
                machine_id=job.machine_id,
                serial_number=job.part_serial or '',
                batch_id=job.material_batch or '',
                tool_id=job.tool_id or '',
                metadata={'job_id': job.job_id},
            )
        return True

    def resume_job(self, job_id: str) -> bool:
        """Resume a paused job and record in genealogy."""
        job = self._active_jobs.get(job_id)
        if not job or job.status != 'PAUSED':
            return False
        job.status = 'RUNNING'
        self._publish_job_status(job)
        if self._digital_thread:
            self._digital_thread.record_genealogy_event(
                DigitalThreadNode.ENTRY_JOB_RESUMED,
                machine_id=job.machine_id,
                serial_number=job.part_serial or '',
                batch_id=job.material_batch or '',
                tool_id=job.tool_id or '',
                metadata={'job_id': job.job_id},
            )
        return True

    def cancel_job(self, job_id: str) -> bool:
        """Cancel an active job and record in genealogy."""
        job = self._active_jobs.get(job_id)
        if not job:
            return False
        job.status = 'CANCELLED'
        self._publish_job_status(job)
        if self._digital_thread:
            self._digital_thread.record_genealogy_event(
                DigitalThreadNode.ENTRY_JOB_CANCELLED,
                machine_id=job.machine_id,
                serial_number=job.part_serial or '',
                batch_id=job.material_batch or '',
                tool_id=job.tool_id or '',
                metadata={'job_id': job.job_id},
            )
            self._digital_thread.clear_active_job(job.machine_id)
        self._active_jobs.pop(job_id, None)
        return True

    def record_job_failure(
        self, job_id: str, error: str, machine_context: Optional[Dict] = None,
    ) -> bool:
        """Record a job failure with error details in genealogy."""
        job = self._active_jobs.get(job_id)
        if not job:
            return False
        job.status = 'FAILED'
        self._publish_job_status(job)
        if self._digital_thread:
            meta: Dict[str, Any] = {
                'job_id': job.job_id,
                'error': error,
            }
            if machine_context:
                meta['machine_context'] = machine_context
            self._digital_thread.record_genealogy_event(
                DigitalThreadNode.ENTRY_JOB_FAILED,
                machine_id=job.machine_id,
                serial_number=job.part_serial or '',
                batch_id=job.material_batch or '',
                tool_id=job.tool_id or '',
                metadata=meta,
            )
            self._digital_thread.clear_active_job(job.machine_id)
        return True

    def record_machine_error(
        self, job_id: str, error: str,
        machine_state_context: Optional[Dict] = None,
    ) -> bool:
        """Record a machine error event with state context in genealogy."""
        job = self._active_jobs.get(job_id)
        if not job:
            return False
        if self._digital_thread:
            meta: Dict[str, Any] = {
                'job_id': job.job_id,
                'error': error,
            }
            if machine_state_context:
                meta['machine_state'] = machine_state_context
            self._digital_thread.record_genealogy_event(
                DigitalThreadNode.ENTRY_MACHINE_ERROR,
                machine_id=job.machine_id,
                serial_number=job.part_serial or '',
                batch_id=job.material_batch or '',
                tool_id=job.tool_id or '',
                metadata=meta,
            )
        return True

    # ------------------------------------------------------------------
    # Queue re-optimization
    # ------------------------------------------------------------------

    def _reoptimize_queue(self) -> None:
        """Re-optimize the job queue using the intelligent optimizer.

        Called when new jobs arrive, tool states change, or conditions
        change that might affect scheduling priorities.
        """
        with self._queue_lock:
            if not self._job_queue:
                return

            # Build current setup context from machine states
            current_setup: Dict[str, Any] = {}
            for mid, state in self._machine_states.items():
                if getattr(state, 'status', '') in ('IDLE', 'READY'):
                    current_setup['machine_id'] = mid
                    current_setup['fixture_type'] = getattr(state, 'fixture_type', '')
                    current_setup['tooling'] = getattr(state, 'current_tools', [])
                    break

            # Build tool state mapping from maintenance scheduler data
            tool_states: Dict[str, float] = {}
            for _machine_id, tools in self.maintenance_scheduler._last_rul.items():
                for tool_id, rul in tools.items():
                    tool_states[tool_id] = rul

            current_time = self.get_clock().now().nanoseconds / 1e9

            # Extract all queued jobs
            all_jobs = list(self._job_queue)
            self._job_queue.clear()

            # Optimize and rebuild queue
            optimized = self._queue_optimizer.optimize_queue(
                all_jobs, current_setup, tool_states, current_time,
            )

            self._job_queue = optimized

    # ------------------------------------------------------------------
    # Predictive maintenance trigger checks
    # ------------------------------------------------------------------

    def _check_maintenance_triggers(self) -> None:
        """Check maintenance triggers based on current machine states and predictions.

        Call this periodically (e.g., from _scheduling_cycle) or when
        prediction data is updated.
        """
        for machine_id, state in self._machine_states.items():
            # Check tool RUL if available on the state message
            tool_id = getattr(state, 'tool_id', '') or ''
            rul = getattr(state, 'tool_rul_minutes', None)
            if tool_id and rul is not None and rul != '':
                try:
                    self.maintenance_scheduler.check_tool_rul(machine_id, tool_id, float(rul))
                except (ValueError, TypeError):
                    pass

            # Check calibration drift if available
            drift = getattr(state, 'calibration_drift_pct', None)
            if drift is not None and drift != '':
                try:
                    self.maintenance_scheduler.check_calibration_drift(machine_id, float(drift))
                except (ValueError, TypeError):
                    pass

    # ------------------------------------------------------------------
    # Alarm escalation handling
    # ------------------------------------------------------------------

    def _on_alarm_escalation(self, msg) -> None:
        """Handle alarm escalation events from the alarm manager.

        Depending on escalation level/action, pause or reassign jobs on
        the affected machine.
        """
        action = msg.escalation_action
        machine_id = msg.machine_id
        reason = msg.reason

        if action == 'CLEARED':
            self._unblock_machine(machine_id)
            return

        if action == 'PRIORITY_BOOST':
            # Level 1: log warning only, no job action
            self.get_logger().warn(
                f"Alarm escalation PRIORITY_BOOST on {machine_id}: {reason}"
            )
            return

        if action == 'FORCED_ACK_REQUIRED':
            # Level 2: pause active job on the machine
            self.get_logger().warn(
                f"Alarm escalation FORCED_ACK_REQUIRED on {machine_id}: "
                f"{reason}"
            )
            job = self._find_active_job_on_machine(machine_id)
            if job and job.status != 'PAUSED':
                self._pause_job_for_alarm(
                    job, f"Paused due to alarm escalation: {reason}"
                )
            return

        if action == 'SUPERVISOR_NOTIFY':
            # Level 3: pause job + block machine
            self.get_logger().error(
                f"Alarm escalation SUPERVISOR_NOTIFY on {machine_id}: "
                f"{reason}"
            )
            job = self._find_active_job_on_machine(machine_id)
            if job and job.status != 'PAUSED':
                self._pause_job_for_alarm(
                    job, f"Paused due to alarm escalation: {reason}"
                )
            self._blocked_machines.add(machine_id)
            return

        if action == 'EMERGENCY_STOP':
            # Emergency: pause + block + attempt reassignment
            job = self._find_active_job_on_machine(machine_id)
            affected_job_id = job.job_id if job else 'N/A'
            self.get_logger().critical(
                f"Emergency stop on {machine_id}, "
                f"job {affected_job_id} paused"
            )
            if job and job.status != 'PAUSED':
                self._pause_job_for_alarm(
                    job, f"Paused due to alarm escalation: {reason}"
                )
            self._blocked_machines.add(machine_id)

            # Attempt to reassign the job to another machine
            if job:
                self.reassign_job(job.job_id, machine_id)
            return

    def _find_active_job_on_machine(self, machine_id: str) -> Optional[Job]:
        """Find the active (non-completed) job assigned to a machine."""
        for job in self._active_jobs.values():
            if job.machine_id == machine_id and job.status in (
                'ASSIGNED', 'RUNNING', 'PAUSED',
            ):
                return job
        return None

    def _pause_job_for_alarm(self, job: Job, reason: str) -> None:
        """Pause a job due to alarm escalation and record history."""
        job.status = 'PAUSED'
        self._publish_job_status(job)
        self._job_history.append({
            'job_id': job.job_id,
            'machine_id': job.machine_id,
            'action': 'PAUSED',
            'reason': reason,
        })
        if self._digital_thread:
            self._digital_thread.record_genealogy_event(
                DigitalThreadNode.ENTRY_JOB_PAUSED,
                machine_id=job.machine_id,
                serial_number=job.part_serial or '',
                batch_id=job.material_batch or '',
                tool_id=job.tool_id or '',
                metadata={
                    'job_id': job.job_id,
                    'reason': reason,
                },
            )

    def _unblock_machine(self, machine_id: str) -> None:
        """Unblock a machine after its alarm is cleared or acknowledged.

        Removes the machine from the blocked set so it becomes available
        for scheduling again.
        """
        if machine_id in self._blocked_machines:
            self._blocked_machines.discard(machine_id)
            self.get_logger().info(
                f"Machine {machine_id} unblocked after alarm cleared"
            )

    def reassign_job(
        self, job_id: str, from_machine: str, to_machine: Optional[str] = None,
    ) -> bool:
        """Reassign a job from one machine to another.

        Args:
            job_id: The job to reassign.
            from_machine: The machine the job is currently on.
            to_machine: Target machine. If None, the best available
                (not blocked, idle/ready) machine is chosen automatically.

        Returns:
            True if the job was successfully reassigned.
        """
        job = self._active_jobs.get(job_id)
        if not job:
            return False

        if to_machine is None:
            # Find best available machine (not blocked, lowest queue)
            candidates = [
                mid for mid, state in self._machine_states.items()
                if mid != from_machine
                and mid not in self._blocked_machines
                and state.status in ('IDLE', 'READY')
            ]
            if not candidates:
                self.get_logger().warn(
                    f"No available machines to reassign job {job_id}"
                )
                return False
            to_machine = candidates[0]

        job.machine_id = to_machine
        job.status = 'ASSIGNED'
        self._publish_job_status(job)

        if self._digital_thread:
            self._digital_thread.record_genealogy_event(
                DigitalThreadNode.ENTRY_JOB_PAUSED,
                machine_id=to_machine,
                serial_number=job.part_serial or '',
                batch_id=job.material_batch or '',
                tool_id=job.tool_id or '',
                metadata={
                    'job_id': job.job_id,
                    'reassigned_from': from_machine,
                    'reassigned_to': to_machine,
                },
            )

        self.get_logger().info(
            f"Job {job_id} reassigned from {from_machine} to {to_machine}"
        )
        return True

    @property
    def blocked_machines(self) -> Set[str]:
        """Return the set of machines currently blocked by alarm escalations."""
        return set(self._blocked_machines)

    def _publish_job_status(self, job: Job) -> None:
        """Publish current job status."""
        msg = JobStatus()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.job_id = job.job_id
        msg.machine_id = job.machine_id
        msg.status = job.status
        msg.program_name = job.program_name

        self._status_pub.publish(msg)


# ---------------------------------------------------------------------------
# Setup Sheet Generator
# ---------------------------------------------------------------------------

_DEFAULT_SAFETY_NOTES = [
    'Verify tool lengths with probe before first cut.',
    'Confirm workholding clamping force before cycle start.',
    'Ensure coolant level is adequate and nozzles are aimed correctly.',
    'Wear safety glasses and hearing protection.',
    'Keep hands clear of moving axes during automatic operation.',
]


@dataclass
class ToolSetup:
    """A single tool entry on a setup sheet."""
    pocket_number: int
    tool_id: str
    description: str = ''
    diameter: float = 0.0
    length_offset: Optional[float] = None
    radius_comp: Optional[float] = None
    min_stickout: float = 0.0
    notes: str = ''


@dataclass
class WorkholdingSpec:
    """Workholding configuration on a setup sheet."""
    fixture_id: str
    fixture_type: str = 'vise'
    clamping_force_n: float = 0.0
    jaw_width_mm: float = 0.0
    parallels: bool = False
    soft_jaws: bool = False
    notes: str = ''


@dataclass
class SetupSheet:
    """Complete machine setup sheet for a job."""
    job_id: str
    machine_id: str
    program_name: str
    tools: List[ToolSetup] = field(default_factory=list)
    workholding: Optional[WorkholdingSpec] = None
    wcs_offsets: Dict[str, Dict[str, float]] = field(default_factory=dict)
    material: str = ''
    safety_notes: List[str] = field(default_factory=lambda: list(_DEFAULT_SAFETY_NOTES))
    estimated_cycle_time: float = 0.0
    revision: int = 1
    created_at: float = field(default_factory=time.time)


class SetupSheetGenerator:
    """Generates and manages machine setup sheets for jobs."""

    def __init__(self) -> None:
        self._sheets: Dict[str, List[SetupSheet]] = {}

    def create_sheet(self, job_id: str, machine_id: str, program_name: str,
                     tools: Optional[List[ToolSetup]] = None,
                     workholding: Optional[WorkholdingSpec] = None,
                     wcs_offsets: Optional[Dict[str, Dict[str, float]]] = None,
                     material: str = '',
                     safety_notes: Optional[List[str]] = None,
                     estimated_cycle_time: float = 0.0) -> SetupSheet:
        revisions = self._sheets.get(job_id, [])
        revision = len(revisions) + 1
        sheet = SetupSheet(
            job_id=job_id, machine_id=machine_id, program_name=program_name,
            tools=tools or [], workholding=workholding,
            wcs_offsets=wcs_offsets or {}, material=material,
            safety_notes=safety_notes if safety_notes is not None else list(_DEFAULT_SAFETY_NOTES),
            estimated_cycle_time=estimated_cycle_time, revision=revision,
        )
        self._sheets.setdefault(job_id, []).append(sheet)
        return sheet

    def get_sheet(self, job_id: str, revision: Optional[int] = None) -> Optional[SetupSheet]:
        revisions = self._sheets.get(job_id, [])
        if not revisions:
            return None
        if revision is not None:
            for s in revisions:
                if s.revision == revision:
                    return s
            return None
        return revisions[-1]

    def get_revision_history(self, job_id: str) -> List[SetupSheet]:
        return list(self._sheets.get(job_id, []))

    def validate(self, sheet: SetupSheet) -> Tuple[bool, List[str]]:
        issues: List[str] = []
        if not sheet.tools:
            issues.append('No tools defined')
        else:
            for t in sheet.tools:
                if t.length_offset is None:
                    issues.append(f'Tool {t.tool_id} (pocket {t.pocket_number}) missing length offset')
        if not sheet.wcs_offsets:
            issues.append('No WCS offsets defined')
        if not sheet.material:
            issues.append('Material not specified')
        if sheet.workholding is None:
            issues.append('Workholding not specified')
        return (len(issues) == 0, issues)

    def compare_revisions(self, job_id: str, rev_a: int, rev_b: int) -> Dict[str, List[str]]:
        sheet_a = self.get_sheet(job_id, rev_a)
        sheet_b = self.get_sheet(job_id, rev_b)
        changes: Dict[str, List[str]] = {'added': [], 'removed': [], 'changed': []}
        if sheet_a is None or sheet_b is None:
            return changes
        tools_a = {t.tool_id for t in sheet_a.tools}
        tools_b = {t.tool_id for t in sheet_b.tools}
        for tid in tools_b - tools_a:
            changes['added'].append(f'Tool {tid}')
        for tid in tools_a - tools_b:
            changes['removed'].append(f'Tool {tid}')
        wcs_a = set(sheet_a.wcs_offsets.keys())
        wcs_b = set(sheet_b.wcs_offsets.keys())
        for w in wcs_b - wcs_a:
            changes['added'].append(f'WCS {w}')
        for w in wcs_a - wcs_b:
            changes['removed'].append(f'WCS {w}')
        for w in wcs_a & wcs_b:
            if sheet_a.wcs_offsets[w] != sheet_b.wcs_offsets[w]:
                changes['changed'].append(f'WCS {w} offsets modified')
        if sheet_a.material != sheet_b.material:
            changes['changed'].append(f'Material: {sheet_a.material} -> {sheet_b.material}')
        if sheet_a.machine_id != sheet_b.machine_id:
            changes['changed'].append(f'Machine: {sheet_a.machine_id} -> {sheet_b.machine_id}')
        return changes

    def generate_checklist(self, sheet: SetupSheet) -> List[str]:
        checklist: List[str] = []
        for t in sheet.tools:
            checklist.append(f'Load tool {t.tool_id} ({t.description}) in pocket {t.pocket_number}')
            if t.length_offset is not None:
                checklist.append(f'  Set length offset H{t.pocket_number} = {t.length_offset:.3f} mm')
            if t.radius_comp is not None:
                checklist.append(f'  Set radius comp D{t.pocket_number} = {t.radius_comp:.3f} mm')
        for wcs_name, offsets in sorted(sheet.wcs_offsets.items()):
            parts = [f'{axis}={val:.3f}' for axis, val in sorted(offsets.items())]
            checklist.append(f'Set {wcs_name}: {", ".join(parts)}')
        if sheet.workholding:
            wh = sheet.workholding
            checklist.append(f'Install {wh.fixture_type} fixture {wh.fixture_id}')
            if wh.clamping_force_n > 0:
                checklist.append(f'  Clamping force: {wh.clamping_force_n:.0f} N')
            if wh.parallels:
                checklist.append('  Place parallels under workpiece')
            if wh.soft_jaws:
                checklist.append('  Install soft jaws')
        if sheet.material:
            checklist.append(f'Verify material is {sheet.material}')
        checklist.append(f'Load program {sheet.program_name}')
        for note in sheet.safety_notes:
            checklist.append(f'[SAFETY] {note}')
        return checklist


# ---------------------------------------------------------------------------
# Cycle Time Estimator
# ---------------------------------------------------------------------------


@dataclass
class MachineCapability:
    """Machine kinematic and process capability parameters."""
    max_feed_mm_min: float = 5000.0
    max_rapid_mm_min: float = 30000.0
    tool_change_sec: float = 5.0
    spindle_accel_sec: float = 2.0
    axis_accel_mm_s2: float = 2000.0


@dataclass
class CycleTimeBreakdown:
    """Breakdown of estimated cycle time by category."""
    cutting_time_min: float
    rapid_time_min: float
    tool_change_time_min: float
    dwell_time_min: float
    spindle_time_min: float
    total_time_min: float
    efficiency_pct: float


class CycleTimeEstimator:
    """Estimates machining cycle times from G-code parameters and machine capabilities.

    Supports block-level estimation from parsed G-code, summary-level quick
    estimates, accuracy comparison against actuals, and time-saving suggestions.
    """

    DEFAULT_CAPABILITY = MachineCapability()

    def estimate_from_blocks(
        self,
        blocks: List[Dict[str, Any]],
        capability: Optional[MachineCapability] = None,
    ) -> CycleTimeBreakdown:
        """Estimate cycle time from a list of G-code blocks.

        Each block is a dict with keys:
            type: 'cut' | 'rapid' | 'dwell' | 'tool_change'
            distance_mm: float  (for cut/rapid)
            feed_mm_min: float  (for cut; 0 or absent uses max_feed)
            dwell_sec: float    (for dwell)

        Args:
            blocks: Parsed G-code block descriptions.
            capability: Machine capability parameters (uses defaults if None).

        Returns:
            CycleTimeBreakdown with per-category times.
        """
        cap = capability or self.DEFAULT_CAPABILITY

        cutting_sec = 0.0
        rapid_sec = 0.0
        tool_change_sec = 0.0
        dwell_sec = 0.0
        spindle_events = 0  # count of tool changes that require spindle re-accel

        for block in blocks:
            btype = block.get('type', '')

            if btype == 'cut':
                distance = block.get('distance_mm', 0.0)
                feed = block.get('feed_mm_min', 0.0)
                if feed <= 0:
                    feed = cap.max_feed_mm_min
                # Clamp feed to machine maximum
                feed = min(feed, cap.max_feed_mm_min)
                if feed > 0 and distance > 0:
                    # Add acceleration/deceleration overhead
                    feed_mm_s = feed / 60.0
                    accel_time = feed_mm_s / cap.axis_accel_mm_s2
                    accel_dist = 0.5 * cap.axis_accel_mm_s2 * accel_time ** 2
                    if 2 * accel_dist >= distance:
                        # Move is too short to reach full speed
                        cutting_sec += 2.0 * math.sqrt(distance / cap.axis_accel_mm_s2)
                    else:
                        cruise_dist = distance - 2.0 * accel_dist
                        cruise_sec = cruise_dist / feed_mm_s
                        cutting_sec += 2.0 * accel_time + cruise_sec

            elif btype == 'rapid':
                distance = block.get('distance_mm', 0.0)
                if distance > 0:
                    rapid_mm_s = cap.max_rapid_mm_min / 60.0
                    accel_time = rapid_mm_s / cap.axis_accel_mm_s2
                    accel_dist = 0.5 * cap.axis_accel_mm_s2 * accel_time ** 2
                    if 2 * accel_dist >= distance:
                        rapid_sec += 2.0 * math.sqrt(distance / cap.axis_accel_mm_s2)
                    else:
                        cruise_dist = distance - 2.0 * accel_dist
                        cruise_sec = cruise_dist / rapid_mm_s
                        rapid_sec += 2.0 * accel_time + cruise_sec

            elif btype == 'dwell':
                dwell_sec += block.get('dwell_sec', 0.0)

            elif btype == 'tool_change':
                tool_change_sec += cap.tool_change_sec
                spindle_events += 1

        spindle_sec = spindle_events * cap.spindle_accel_sec

        cutting_min = cutting_sec / 60.0
        rapid_min = rapid_sec / 60.0
        tool_change_min = tool_change_sec / 60.0
        dwell_min = dwell_sec / 60.0
        spindle_min = spindle_sec / 60.0
        total_min = cutting_min + rapid_min + tool_change_min + dwell_min + spindle_min

        # Efficiency: cutting time as percentage of total time
        efficiency_pct = (cutting_min / total_min * 100.0) if total_min > 0 else 0.0

        return CycleTimeBreakdown(
            cutting_time_min=round(cutting_min, 4),
            rapid_time_min=round(rapid_min, 4),
            tool_change_time_min=round(tool_change_min, 4),
            dwell_time_min=round(dwell_min, 4),
            spindle_time_min=round(spindle_min, 4),
            total_time_min=round(total_min, 4),
            efficiency_pct=round(efficiency_pct, 2),
        )

    def estimate_from_program_stats(
        self,
        total_cut_distance: float,
        total_rapid_distance: float,
        num_tool_changes: int,
        total_dwell_sec: float,
        avg_feed: float,
        capability: Optional[MachineCapability] = None,
    ) -> CycleTimeBreakdown:
        """Quick cycle-time estimate from program summary statistics.

        Uses simple distance/speed calculations without per-block
        acceleration modelling.

        Args:
            total_cut_distance: Total cutting distance in mm.
            total_rapid_distance: Total rapid traverse distance in mm.
            num_tool_changes: Number of tool changes.
            total_dwell_sec: Total dwell time in seconds.
            avg_feed: Average cutting feedrate in mm/min.
            capability: Machine capability parameters.

        Returns:
            CycleTimeBreakdown.
        """
        cap = capability or self.DEFAULT_CAPABILITY

        feed = min(avg_feed, cap.max_feed_mm_min) if avg_feed > 0 else cap.max_feed_mm_min
        cutting_min = (total_cut_distance / feed) if feed > 0 else 0.0
        rapid_min = (total_rapid_distance / cap.max_rapid_mm_min) if cap.max_rapid_mm_min > 0 else 0.0
        tool_change_min = (num_tool_changes * cap.tool_change_sec) / 60.0
        dwell_min = total_dwell_sec / 60.0
        spindle_min = (num_tool_changes * cap.spindle_accel_sec) / 60.0

        total_min = cutting_min + rapid_min + tool_change_min + dwell_min + spindle_min
        efficiency_pct = (cutting_min / total_min * 100.0) if total_min > 0 else 0.0

        return CycleTimeBreakdown(
            cutting_time_min=round(cutting_min, 4),
            rapid_time_min=round(rapid_min, 4),
            tool_change_time_min=round(tool_change_min, 4),
            dwell_time_min=round(dwell_min, 4),
            spindle_time_min=round(spindle_min, 4),
            total_time_min=round(total_min, 4),
            efficiency_pct=round(efficiency_pct, 2),
        )

    def compare_estimated_vs_actual(
        self,
        estimated: CycleTimeBreakdown,
        actual_min: float,
    ) -> Dict[str, Any]:
        """Compare an estimated cycle time against an actual measured time.

        Args:
            estimated: The estimated breakdown.
            actual_min: Actual measured cycle time in minutes.

        Returns:
            Dict with 'accuracy_pct' (100 = perfect), 'deviation_min',
            and 'indication' ('over', 'under', or 'exact').
        """
        if actual_min <= 0:
            return {
                'accuracy_pct': 0.0,
                'deviation_min': estimated.total_time_min,
                'indication': 'over' if estimated.total_time_min > 0 else 'exact',
            }

        deviation = estimated.total_time_min - actual_min
        accuracy_pct = max(0.0, (1.0 - abs(deviation) / actual_min) * 100.0)

        if abs(deviation) < 1e-9:
            indication = 'exact'
        elif deviation > 0:
            indication = 'over'
        else:
            indication = 'under'

        return {
            'accuracy_pct': round(accuracy_pct, 2),
            'deviation_min': round(deviation, 4),
            'indication': indication,
        }

    def suggest_time_savings(
        self, breakdown: CycleTimeBreakdown,
    ) -> List[str]:
        """Suggest actionable time savings based on a cycle time breakdown.

        Args:
            breakdown: A CycleTimeBreakdown to analyse.

        Returns:
            List of suggestion strings.
        """
        suggestions: List[str] = []

        if breakdown.total_time_min <= 0:
            return suggestions

        rapid_pct = (breakdown.rapid_time_min / breakdown.total_time_min) * 100.0
        tc_pct = (breakdown.tool_change_time_min / breakdown.total_time_min) * 100.0
        dwell_pct = (breakdown.dwell_time_min / breakdown.total_time_min) * 100.0
        spindle_pct = (breakdown.spindle_time_min / breakdown.total_time_min) * 100.0

        if rapid_pct > 15.0:
            suggestions.append(
                'Reduce rapid distance by optimising tool-path retract heights and positioning.'
            )

        if tc_pct > 10.0:
            suggestions.append(
                'Minimize tool changes by reordering operations or using combination tools.'
            )

        if dwell_pct > 5.0:
            suggestions.append(
                'Review dwell times; reduce or eliminate unnecessary G4 pauses.'
            )

        if spindle_pct > 5.0:
            suggestions.append(
                'Reduce spindle acceleration overhead by grouping operations at similar RPM.'
            )

        if breakdown.efficiency_pct < 60.0:
            suggestions.append(
                'Overall cutting efficiency is below 60%; review non-cutting time contributors.'
            )

        if breakdown.efficiency_pct >= 90.0:
            suggestions.append(
                'Cutting efficiency is excellent; focus on feed/speed optimisation for further gains.'
            )

        return suggestions


# ---------------------------------------------------------------------------
# Work Order Tracker
# ---------------------------------------------------------------------------


@dataclass
class WorkOrderStage:
    """A single manufacturing stage within a work order."""

    stage_name: str
    status: str = 'pending'  # 'pending' | 'in_progress' | 'completed' | 'skipped'
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    operator: str = ''
    notes: str = ''


@dataclass
class WorkOrder:
    """A manufacturing work order consisting of multiple stages."""

    wo_id: str
    part_number: str
    quantity: int
    stages: List[WorkOrderStage] = field(default_factory=list)
    created_at: float = 0.0
    priority: int = 0
    customer: str = ''
    due_date: float = 0.0


@dataclass
class WorkOrderMetrics:
    """Aggregate metrics across all tracked work orders."""

    total_orders: int = 0
    completed: int = 0
    in_progress: int = 0
    avg_cycle_time_min: float = 0.0
    on_time_delivery_pct: float = 0.0
    stage_bottleneck: str = ''


class WorkOrderTracker:
    """Tracks work orders through manufacturing stages.

    Provides lifecycle management for work orders, including stage
    transitions (start, complete, skip) and aggregate metrics with
    bottleneck analysis.
    """

    def __init__(self) -> None:
        self._orders: Dict[str, WorkOrder] = {}

    # -- helpers -------------------------------------------------------------

    @staticmethod
    def _overall_status(wo: WorkOrder) -> str:
        """Derive the overall status of a work order from its stages."""
        statuses = {s.status for s in wo.stages}
        if all(s in ('completed', 'skipped') for s in statuses):
            return 'completed'
        if 'in_progress' in statuses:
            return 'in_progress'
        # Mix of pending / completed / skipped but nothing in_progress
        if 'completed' in statuses or 'skipped' in statuses:
            # Some work done but not all finished
            if 'pending' in statuses:
                return 'in_progress'
            return 'completed'
        return 'pending'

    # -- public API ----------------------------------------------------------

    def create_work_order(
        self,
        wo_id: str,
        part_number: str,
        quantity: int,
        stage_names: List[str],
        priority: int = 0,
        customer: str = '',
        due_date: float = 0.0,
    ) -> WorkOrder:
        """Create a new work order with all stages initialised to *pending*."""
        stages = [WorkOrderStage(stage_name=name) for name in stage_names]
        wo = WorkOrder(
            wo_id=wo_id,
            part_number=part_number,
            quantity=quantity,
            stages=stages,
            created_at=time.time(),
            priority=priority,
            customer=customer,
            due_date=due_date,
        )
        self._orders[wo_id] = wo
        return wo

    def start_stage(self, wo_id: str, stage_name: str, operator: str) -> None:
        """Mark a stage as *in_progress* and record the operator."""
        wo = self._orders[wo_id]
        for stage in wo.stages:
            if stage.stage_name == stage_name:
                stage.status = 'in_progress'
                stage.start_time = time.time()
                stage.operator = operator
                return
        raise ValueError(f"Stage '{stage_name}' not found in work order '{wo_id}'")

    def complete_stage(self, wo_id: str, stage_name: str, notes: str = '') -> None:
        """Mark a stage as *completed* and auto-start the next pending stage."""
        wo = self._orders[wo_id]
        completed_stage: Optional[WorkOrderStage] = None
        for stage in wo.stages:
            if stage.stage_name == stage_name:
                stage.status = 'completed'
                stage.end_time = time.time()
                stage.notes = notes
                completed_stage = stage
                break
        if completed_stage is None:
            raise ValueError(f"Stage '{stage_name}' not found in work order '{wo_id}'")

        # Auto-start the next pending stage if one exists
        for stage in wo.stages:
            if stage.status == 'pending':
                stage.status = 'in_progress'
                stage.start_time = time.time()
                if completed_stage.operator:
                    stage.operator = completed_stage.operator
                break

    def skip_stage(self, wo_id: str, stage_name: str, reason: str = '') -> None:
        """Skip a stage with an optional reason stored in notes."""
        wo = self._orders[wo_id]
        for stage in wo.stages:
            if stage.stage_name == stage_name:
                stage.status = 'skipped'
                stage.end_time = time.time()
                stage.notes = reason
                return
        raise ValueError(f"Stage '{stage_name}' not found in work order '{wo_id}'")

    def get_work_order(self, wo_id: str) -> WorkOrder:
        """Return a single work order by ID."""
        return self._orders[wo_id]

    def get_all_orders(self) -> List[WorkOrder]:
        """Return all tracked work orders."""
        return list(self._orders.values())

    def get_orders_by_status(self, status: str) -> List[WorkOrder]:
        """Filter work orders by overall status (pending/in_progress/completed)."""
        return [wo for wo in self._orders.values() if self._overall_status(wo) == status]

    def get_metrics(self) -> WorkOrderMetrics:
        """Compute aggregate metrics including bottleneck analysis."""
        all_orders = list(self._orders.values())
        total = len(all_orders)
        if total == 0:
            return WorkOrderMetrics()

        completed_orders = [wo for wo in all_orders if self._overall_status(wo) == 'completed']
        in_progress_orders = [wo for wo in all_orders if self._overall_status(wo) == 'in_progress']

        # Average cycle time (for completed orders, in minutes)
        cycle_times: List[float] = []
        for wo in completed_orders:
            stage_times = [
                s.end_time - s.start_time
                for s in wo.stages
                if s.status == 'completed' and s.start_time is not None and s.end_time is not None
            ]
            if stage_times:
                cycle_times.append(sum(stage_times))

        avg_cycle_min = (sum(cycle_times) / len(cycle_times) / 60.0) if cycle_times else 0.0

        # On-time delivery percentage (completed orders finished by due_date)
        if completed_orders:
            on_time = 0
            for wo in completed_orders:
                if wo.due_date <= 0:
                    on_time += 1  # no due date counts as on-time
                else:
                    last_end = max(
                        (s.end_time for s in wo.stages if s.end_time is not None),
                        default=0.0,
                    )
                    if last_end <= wo.due_date:
                        on_time += 1
            on_time_pct = (on_time / len(completed_orders)) * 100.0
        else:
            on_time_pct = 0.0

        # Bottleneck analysis: stage with the longest average duration
        stage_durations: Dict[str, List[float]] = {}
        for wo in all_orders:
            for s in wo.stages:
                if s.status == 'completed' and s.start_time is not None and s.end_time is not None:
                    dur = s.end_time - s.start_time
                    stage_durations.setdefault(s.stage_name, []).append(dur)

        bottleneck = ''
        if stage_durations:
            bottleneck = max(
                stage_durations,
                key=lambda name: sum(stage_durations[name]) / len(stage_durations[name]),
            )

        return WorkOrderMetrics(
            total_orders=total,
            completed=len(completed_orders),
            in_progress=len(in_progress_orders),
            avg_cycle_time_min=avg_cycle_min,
            on_time_delivery_pct=on_time_pct,
            stage_bottleneck=bottleneck,
        )


# ---------------------------------------------------------------------------
# Operator Skill Matrix
# ---------------------------------------------------------------------------


@dataclass
class OperatorProfile:
    """Profile for a manufacturing operator."""
    operator_id: str
    name: str
    skills: Dict[str, int] = field(default_factory=dict)       # skill_name -> proficiency 1-5
    certifications: List[str] = field(default_factory=list)
    qualified_machines: List[str] = field(default_factory=list)
    shift: str = ''
    hire_date: float = 0.0    # unix timestamp
    total_hours: float = 0.0


@dataclass
class SkillAssessment:
    """Record of a single skill assessment for an operator."""
    operator_id: str
    skill_name: str
    score: int          # 1-5
    assessed_by: str
    timestamp: float
    notes: str = ''


class OperatorSkillMatrix:
    """Tracks operator skills, certifications, and machine qualifications.

    Supports workforce planning by identifying skill gaps, recommending
    training, and ensuring shift coverage across machines.
    """

    def __init__(self) -> None:
        self._operators: Dict[str, OperatorProfile] = {}
        self._assessments: List[SkillAssessment] = []

    # ------------------------------------------------------------------
    # Registration helpers
    # ------------------------------------------------------------------

    def register_operator(self, profile: OperatorProfile) -> None:
        """Register or update an operator profile."""
        self._operators[profile.operator_id] = profile

    def get_operator(self, operator_id: str) -> Optional[OperatorProfile]:
        """Return the profile for *operator_id*, or ``None``."""
        return self._operators.get(operator_id)

    def get_all_operators(self) -> List[OperatorProfile]:
        """Return all registered operator profiles."""
        return list(self._operators.values())

    # ------------------------------------------------------------------
    # Skill assessment
    # ------------------------------------------------------------------

    def record_assessment(self, assessment: SkillAssessment) -> None:
        """Record an assessment and update the operator's skill level."""
        self._assessments.append(assessment)
        profile = self._operators.get(assessment.operator_id)
        if profile is not None:
            profile.skills[assessment.skill_name] = max(
                1, min(5, assessment.score),
            )

    # ------------------------------------------------------------------
    # Query helpers
    # ------------------------------------------------------------------

    def find_qualified_operators(
        self,
        machine_id: str,
        required_skills: Optional[Dict[str, int]] = None,
    ) -> List[OperatorProfile]:
        """Find operators qualified for *machine_id* with *required_skills*.

        Args:
            machine_id: The machine the operator must be qualified on.
            required_skills: Mapping of skill_name -> minimum proficiency.
                             If ``None`` or empty, only machine qualification
                             is checked.

        Returns:
            List of matching :class:`OperatorProfile` instances.
        """
        required_skills = required_skills or {}
        result: List[OperatorProfile] = []
        for op in self._operators.values():
            if machine_id not in op.qualified_machines:
                continue
            qualified = True
            for skill, min_level in required_skills.items():
                if op.skills.get(skill, 0) < min_level:
                    qualified = False
                    break
            if qualified:
                result.append(op)
        return result

    def get_skill_gaps(
        self,
        operator_id: str,
        required_skills: Dict[str, int],
    ) -> Dict[str, Tuple[int, int]]:
        """Identify skills below the required level for an operator.

        Args:
            operator_id: The operator to evaluate.
            required_skills: Mapping of skill_name -> minimum proficiency.

        Returns:
            Dict of skill_name -> (current_level, required_level) for every
            skill that is below the requirement.  A missing skill is treated
            as level 0.
        """
        profile = self._operators.get(operator_id)
        if profile is None:
            return {skill: (0, level) for skill, level in required_skills.items()}
        gaps: Dict[str, Tuple[int, int]] = {}
        for skill, required_level in required_skills.items():
            current = profile.skills.get(skill, 0)
            if current < required_level:
                gaps[skill] = (current, required_level)
        return gaps

    def get_team_coverage(
        self,
        machine_ids: List[str],
        shift: str,
    ) -> Dict[str, List[str]]:
        """Check whether a shift has operators for every machine.

        Args:
            machine_ids: Machines that need coverage.
            shift: The shift identifier to filter by.

        Returns:
            Mapping of machine_id -> list of qualified operator_ids on that
            shift.  An empty list means the machine is uncovered.
        """
        coverage: Dict[str, List[str]] = {m: [] for m in machine_ids}
        for op in self._operators.values():
            if op.shift != shift:
                continue
            for mid in machine_ids:
                if mid in op.qualified_machines:
                    coverage[mid].append(op.operator_id)
        return coverage

    def recommend_training(
        self,
        operator_id: str,
    ) -> List[Tuple[str, int, int]]:
        """Suggest skills to improve based on gaps across all machines.

        Compares the operator's current skills to the skills required by all
        machines they are *not* yet qualified on (or where their proficiency
        is below 3).  Returns a list of (skill_name, current_level,
        recommended_level) tuples sorted by the size of the gap (largest
        first).
        """
        profile = self._operators.get(operator_id)
        if profile is None:
            return []

        # Collect the maximum proficiency seen for every skill across all
        # operators (a proxy for "what the team values").
        team_skills: Dict[str, int] = {}
        for op in self._operators.values():
            for skill, level in op.skills.items():
                if level > team_skills.get(skill, 0):
                    team_skills[skill] = level

        recommendations: List[Tuple[str, int, int]] = []
        for skill, team_max in team_skills.items():
            current = profile.skills.get(skill, 0)
            if current < team_max and current < 3:
                recommendations.append((skill, current, team_max))

        # Sort by gap size descending, then skill name for stability
        recommendations.sort(key=lambda r: (-r[2] + r[1], r[0]))
        return recommendations

    def get_skill_summary(self) -> Dict[str, Dict[str, Any]]:
        """Return team-wide skill distribution.

        Returns:
            Mapping of skill_name -> {
                'avg': float,       # average proficiency
                'min': int,         # minimum proficiency
                'max': int,         # maximum proficiency
                'count': int,       # number of operators with this skill
                'operators': list,  # operator_ids who hold this skill
            }
        """
        skill_data: Dict[str, List[Tuple[str, int]]] = {}
        for op in self._operators.values():
            for skill, level in op.skills.items():
                skill_data.setdefault(skill, []).append((op.operator_id, level))

        summary: Dict[str, Dict[str, Any]] = {}
        for skill, entries in skill_data.items():
            levels = [lvl for _, lvl in entries]
            summary[skill] = {
                'avg': sum(levels) / len(levels),
                'min': min(levels),
                'max': max(levels),
                'count': len(levels),
                'operators': [oid for oid, _ in entries],
            }
        return summary


# ---------------------------------------------------------------------------
# Quality Gate Checker
# ---------------------------------------------------------------------------


@dataclass
class QualityCheck:
    """A single dimensional or parametric check within a quality gate."""
    check_id: str
    parameter: str
    nominal: float
    tolerance_plus: float
    tolerance_minus: float
    unit: str
    is_critical: bool


@dataclass
class QualityGate:
    """A quality checkpoint that parts must pass during manufacturing."""
    gate_id: str
    name: str
    stage: str
    checks: List[QualityCheck] = field(default_factory=list)
    is_mandatory: bool = True
    pass_threshold_pct: float = 100.0


@dataclass
class InspectionResult:
    """Result of a single quality check measurement."""
    check_id: str
    measured_value: float
    deviation: float
    passed: bool
    timestamp: float
    inspector: str


@dataclass
class GateResult:
    """Aggregated result of inspecting a part at a quality gate."""
    gate_id: str
    part_id: str
    results: List[InspectionResult] = field(default_factory=list)
    overall_passed: bool = False
    pass_rate_pct: float = 0.0
    timestamp: float = 0.0


class QualityGateChecker:
    """Manages quality checkpoints (gates) that parts must pass through.

    Each gate contains a set of quality checks with nominal values and
    tolerances.  Parts are inspected against these checks and the results
    are recorded for traceability and process analysis.
    """

    # Define a canonical stage ordering for ``can_proceed`` logic.
    STAGE_ORDER: List[str] = [
        'incoming',
        'roughing',
        'semi_finish',
        'finishing',
        'inspection',
        'shipping',
    ]

    def __init__(self) -> None:
        self._gates: Dict[str, QualityGate] = {}
        # part_id -> list of GateResult (chronological)
        self._part_history: Dict[str, List[GateResult]] = {}

    # -- gate management -----------------------------------------------------

    def add_gate(self, gate: QualityGate) -> None:
        """Register a quality gate."""
        self._gates[gate.gate_id] = gate

    def get_gate(self, gate_id: str) -> Optional[QualityGate]:
        """Return a gate by id, or ``None`` if not found."""
        return self._gates.get(gate_id)

    def get_all_gates(self) -> List[QualityGate]:
        """Return all registered gates."""
        return list(self._gates.values())

    # -- inspection ----------------------------------------------------------

    def inspect(
        self,
        gate_id: str,
        part_id: str,
        measurements: Dict[str, float],
        inspector: str,
    ) -> GateResult:
        """Evaluate *measurements* against the checks defined in *gate_id*.

        Parameters
        ----------
        gate_id:
            The quality gate to inspect against.
        part_id:
            Identifier of the part being inspected.
        measurements:
            Mapping of ``check_id`` -> ``measured_value``.
        inspector:
            Name or id of the person/system performing the inspection.

        Returns
        -------
        GateResult
            Aggregated inspection result with per-check details.

        Raises
        ------
        KeyError
            If *gate_id* is not registered.
        """
        gate = self._gates.get(gate_id)
        if gate is None:
            raise KeyError(f"Unknown gate: {gate_id}")

        now = time.time()
        results: List[InspectionResult] = []

        for check in gate.checks:
            measured = measurements.get(check.check_id)
            if measured is None:
                # Missing measurement counts as a failure.
                results.append(InspectionResult(
                    check_id=check.check_id,
                    measured_value=float('nan'),
                    deviation=float('nan'),
                    passed=False,
                    timestamp=now,
                    inspector=inspector,
                ))
                continue

            deviation = measured - check.nominal
            lower = check.nominal - check.tolerance_minus
            upper = check.nominal + check.tolerance_plus
            passed = lower <= measured <= upper

            results.append(InspectionResult(
                check_id=check.check_id,
                measured_value=measured,
                deviation=deviation,
                passed=passed,
                timestamp=now,
                inspector=inspector,
            ))

        total = len(results)
        passed_count = sum(1 for r in results if r.passed)
        pass_rate = (passed_count / total * 100.0) if total > 0 else 0.0
        overall_passed = pass_rate >= gate.pass_threshold_pct

        # If any critical check failed, the gate fails regardless of threshold.
        critical_ids = {c.check_id for c in gate.checks if c.is_critical}
        for r in results:
            if r.check_id in critical_ids and not r.passed:
                overall_passed = False
                break

        gate_result = GateResult(
            gate_id=gate_id,
            part_id=part_id,
            results=results,
            overall_passed=overall_passed,
            pass_rate_pct=pass_rate,
            timestamp=now,
        )

        self._part_history.setdefault(part_id, []).append(gate_result)
        return gate_result

    # -- history & statistics ------------------------------------------------

    def get_part_history(self, part_id: str) -> List[GateResult]:
        """Return all gate results for *part_id* in chronological order."""
        return list(self._part_history.get(part_id, []))

    def get_gate_statistics(self, gate_id: str) -> Dict[str, Any]:
        """Compute aggregate statistics for a gate across all inspections.

        Returns a dict with keys:
        - ``total_inspections`` (int)
        - ``pass_rate_pct`` (float)
        - ``most_failed_check`` (str or None)
        - ``avg_deviation`` (dict of check_id -> float)
        """
        all_results: List[GateResult] = []
        for history in self._part_history.values():
            for gr in history:
                if gr.gate_id == gate_id:
                    all_results.append(gr)

        total = len(all_results)
        if total == 0:
            return {
                'total_inspections': 0,
                'pass_rate_pct': 0.0,
                'most_failed_check': None,
                'avg_deviation': {},
            }

        passed = sum(1 for gr in all_results if gr.overall_passed)

        # Per-check failure counts and deviation sums.
        fail_counts: Dict[str, int] = {}
        deviation_sums: Dict[str, float] = {}
        deviation_counts: Dict[str, int] = {}

        for gr in all_results:
            for ir in gr.results:
                if not ir.passed:
                    fail_counts[ir.check_id] = fail_counts.get(ir.check_id, 0) + 1
                if not math.isnan(ir.deviation):
                    deviation_sums[ir.check_id] = (
                        deviation_sums.get(ir.check_id, 0.0) + ir.deviation
                    )
                    deviation_counts[ir.check_id] = (
                        deviation_counts.get(ir.check_id, 0) + 1
                    )

        most_failed = max(fail_counts, key=fail_counts.get) if fail_counts else None  # type: ignore[arg-type]

        avg_deviation: Dict[str, float] = {}
        for cid, s in deviation_sums.items():
            avg_deviation[cid] = s / deviation_counts[cid]

        return {
            'total_inspections': total,
            'pass_rate_pct': passed / total * 100.0,
            'most_failed_check': most_failed,
            'avg_deviation': avg_deviation,
        }

    # -- stage gating --------------------------------------------------------

    def can_proceed(self, part_id: str, next_stage: str) -> bool:
        """Check whether *part_id* may advance to *next_stage*.

        A part can proceed only if it has passed **all mandatory gates**
        whose stage precedes *next_stage* in ``STAGE_ORDER``.

        If *next_stage* is not in ``STAGE_ORDER`` (custom stage), this
        method returns ``True`` (permissive by default).
        """
        if next_stage not in self.STAGE_ORDER:
            return True

        next_idx = self.STAGE_ORDER.index(next_stage)
        prior_stages = set(self.STAGE_ORDER[:next_idx])

        # Collect mandatory gates for prior stages.
        required_gate_ids: Set[str] = set()
        for gate in self._gates.values():
            if gate.is_mandatory and gate.stage in prior_stages:
                required_gate_ids.add(gate.gate_id)

        if not required_gate_ids:
            return True

        # Check part history for passed results.
        history = self._part_history.get(part_id, [])
        passed_gates: Set[str] = set()
        for gr in history:
            if gr.overall_passed:
                passed_gates.add(gr.gate_id)

        return required_gate_ids.issubset(passed_gates)


# ---------------------------------------------------------------------------
# Manufacturing Recipe Version Control
# ---------------------------------------------------------------------------


@dataclass
class RecipeParameter:
    """A single tuneable parameter within a manufacturing recipe."""

    name: str
    value: float
    unit: str
    min_value: float
    max_value: float
    is_critical: bool = False


@dataclass
class Recipe:
    """A versioned manufacturing recipe – a complete parameter set for a part."""

    recipe_id: str
    part_number: str
    version: int
    parameters: List[RecipeParameter]
    created_by: str
    created_at: float
    approved: bool = False
    approved_by: str = ''
    notes: str = ''


@dataclass
class RecipeDiff:
    """Result of comparing two recipe versions."""

    recipe_id: str
    version_a: int
    version_b: int
    added: List[str]
    removed: List[str]
    changed: Dict[str, Tuple[float, float]]


class RecipeVersionControl:
    """Manages versioned manufacturing recipes.

    Stores every version of each recipe, supports approval workflows,
    parameter-bound validation, and version-to-version diffing.
    """

    def __init__(self) -> None:
        # recipe_id -> {version -> Recipe}
        self._recipes: Dict[str, Dict[int, Recipe]] = {}

    # -- public API --------------------------------------------------------

    def create_recipe(
        self,
        recipe_id: str,
        part_number: str,
        parameters: List[RecipeParameter],
        created_by: str,
        notes: str = '',
    ) -> Recipe:
        """Create a brand-new recipe at version 1."""
        if recipe_id in self._recipes:
            raise ValueError(
                f"Recipe '{recipe_id}' already exists. "
                "Use update_recipe to create a new version."
            )
        recipe = Recipe(
            recipe_id=recipe_id,
            part_number=part_number,
            version=1,
            parameters=copy.deepcopy(parameters),
            created_by=created_by,
            created_at=time.time(),
            notes=notes,
        )
        self._recipes[recipe_id] = {1: recipe}
        return recipe

    def update_recipe(
        self,
        recipe_id: str,
        parameters: List[RecipeParameter],
        created_by: str,
        notes: str = '',
    ) -> Recipe:
        """Create a new version of an existing recipe."""
        if recipe_id not in self._recipes:
            raise KeyError(f"Recipe '{recipe_id}' does not exist.")
        versions = self._recipes[recipe_id]
        latest_version = max(versions)
        latest_recipe = versions[latest_version]
        new_version = latest_version + 1
        recipe = Recipe(
            recipe_id=recipe_id,
            part_number=latest_recipe.part_number,
            version=new_version,
            parameters=copy.deepcopy(parameters),
            created_by=created_by,
            created_at=time.time(),
            notes=notes,
        )
        versions[new_version] = recipe
        return recipe

    def approve_recipe(
        self,
        recipe_id: str,
        version: int,
        approved_by: str,
    ) -> Recipe:
        """Mark a specific recipe version as approved."""
        recipe = self._get_version(recipe_id, version)
        recipe.approved = True
        recipe.approved_by = approved_by
        return recipe

    def get_recipe(
        self,
        recipe_id: str,
        version: Optional[int] = None,
    ) -> Recipe:
        """Return a specific version, or the latest if *version* is ``None``."""
        if version is not None:
            return self._get_version(recipe_id, version)
        if recipe_id not in self._recipes:
            raise KeyError(f"Recipe '{recipe_id}' does not exist.")
        latest_version = max(self._recipes[recipe_id])
        return self._recipes[recipe_id][latest_version]

    def get_approved_recipe(self, recipe_id: str) -> Optional[Recipe]:
        """Return the latest *approved* version, or ``None``."""
        if recipe_id not in self._recipes:
            raise KeyError(f"Recipe '{recipe_id}' does not exist.")
        approved: List[Recipe] = [
            r for r in self._recipes[recipe_id].values() if r.approved
        ]
        if not approved:
            return None
        return max(approved, key=lambda r: r.version)

    def diff_versions(
        self,
        recipe_id: str,
        v1: int,
        v2: int,
    ) -> RecipeDiff:
        """Compare two versions of a recipe and return the diff."""
        recipe_a = self._get_version(recipe_id, v1)
        recipe_b = self._get_version(recipe_id, v2)

        params_a = {p.name: p.value for p in recipe_a.parameters}
        params_b = {p.name: p.value for p in recipe_b.parameters}

        names_a = set(params_a)
        names_b = set(params_b)

        added = sorted(names_b - names_a)
        removed = sorted(names_a - names_b)
        changed: Dict[str, Tuple[float, float]] = {}
        for name in sorted(names_a & names_b):
            if params_a[name] != params_b[name]:
                changed[name] = (params_a[name], params_b[name])

        return RecipeDiff(
            recipe_id=recipe_id,
            version_a=v1,
            version_b=v2,
            added=added,
            removed=removed,
            changed=changed,
        )

    @staticmethod
    def validate_parameters(parameters: List[RecipeParameter]) -> List[str]:
        """Validate that every parameter value is within its min/max bounds.

        Returns a list of human-readable error strings.  An empty list
        means all parameters are valid.
        """
        errors: List[str] = []
        for p in parameters:
            if p.value < p.min_value:
                errors.append(
                    f"{p.name}: value {p.value} is below minimum {p.min_value}"
                )
            if p.value > p.max_value:
                errors.append(
                    f"{p.name}: value {p.value} is above maximum {p.max_value}"
                )
        return errors

    # -- private helpers ---------------------------------------------------

    def _get_version(self, recipe_id: str, version: int) -> Recipe:
        if recipe_id not in self._recipes:
            raise KeyError(f"Recipe '{recipe_id}' does not exist.")
        versions = self._recipes[recipe_id]
        if version not in versions:
            raise KeyError(
                f"Version {version} not found for recipe '{recipe_id}'."
            )
        return versions[version]


# ---------------------------------------------------------------------------
# Shift Handoff Manager
# ---------------------------------------------------------------------------


@dataclass
class ShiftInfo:
    """Information about a manufacturing shift."""

    shift_id: str
    shift_name: str
    start_time: float
    end_time: float
    supervisor: str
    operators: List[str] = field(default_factory=list)


@dataclass
class HandoffItem:
    """A single item to be communicated during a shift handoff."""

    item_id: str
    category: str  # 'job_in_progress'|'pending_issue'|'safety_note'|'quality_alert'|'maintenance_needed'
    description: str
    priority: int  # 1 (highest) – 5 (lowest)
    machine_id: str
    status: str = 'open'  # 'open'|'acknowledged'|'resolved'


@dataclass
class HandoffReport:
    """Report generated when one shift hands off to another."""

    report_id: str
    from_shift: ShiftInfo
    to_shift: ShiftInfo
    items: List[HandoffItem] = field(default_factory=list)
    created_at: float = 0.0
    acknowledged_by: str = ''
    notes: str = ''


_VALID_CATEGORIES: Set[str] = {
    'job_in_progress',
    'pending_issue',
    'safety_note',
    'quality_alert',
    'maintenance_needed',
}

_VALID_ITEM_STATUSES: Set[str] = {'open', 'acknowledged', 'resolved'}


class ShiftHandoffManager:
    """Manages shift handoffs with status tracking and task transfer.

    Tracks shifts, handoff items (open issues, jobs in progress, safety notes,
    etc.) and generates handoff reports that the incoming shift can acknowledge.
    """

    def __init__(self) -> None:
        self._shifts: Dict[str, ShiftInfo] = {}
        self._items: Dict[str, HandoffItem] = {}
        self._reports: Dict[str, HandoffReport] = {}
        self._report_history: List[HandoffReport] = []
        self._lock = threading.Lock()

    # -- shift management ---------------------------------------------------

    def create_shift(self, shift_info: ShiftInfo) -> ShiftInfo:
        """Register a new shift.  Returns the stored *ShiftInfo*."""
        with self._lock:
            self._shifts[shift_info.shift_id] = shift_info
        return shift_info

    def get_shift(self, shift_id: str) -> Optional[ShiftInfo]:
        """Return the *ShiftInfo* for *shift_id*, or ``None``."""
        return self._shifts.get(shift_id)

    # -- handoff items ------------------------------------------------------

    def add_handoff_item(self, item: HandoffItem) -> HandoffItem:
        """Add a handoff item to the current pool of open items.

        Validates *category*, *status*, and *priority* before storing.
        Returns the stored item.
        """
        if item.category not in _VALID_CATEGORIES:
            raise ValueError(
                f"Invalid category '{item.category}'. "
                f"Must be one of {sorted(_VALID_CATEGORIES)}."
            )
        if item.status not in _VALID_ITEM_STATUSES:
            raise ValueError(
                f"Invalid status '{item.status}'. "
                f"Must be one of {sorted(_VALID_ITEM_STATUSES)}."
            )
        if not (1 <= item.priority <= 5):
            raise ValueError(
                f"Priority must be between 1 and 5, got {item.priority}."
            )
        with self._lock:
            self._items[item.item_id] = item
        return item

    def resolve_item(self, item_id: str) -> HandoffItem:
        """Mark a handoff item as resolved.  Returns the updated item."""
        with self._lock:
            if item_id not in self._items:
                raise KeyError(f"Handoff item '{item_id}' not found.")
            self._items[item_id].status = 'resolved'
            return self._items[item_id]

    def get_open_items(self, machine_id: Optional[str] = None) -> List[HandoffItem]:
        """Return unresolved items, optionally filtered by *machine_id*.

        Items are sorted by priority (1 = highest first).
        """
        with self._lock:
            results = [
                item
                for item in self._items.values()
                if item.status != 'resolved'
                and (machine_id is None or item.machine_id == machine_id)
            ]
        results.sort(key=lambda i: i.priority)
        return results

    # -- reports ------------------------------------------------------------

    def generate_handoff_report(
        self,
        from_shift_id: str,
        to_shift_id: str,
        notes: str = '',
    ) -> HandoffReport:
        """Generate a handoff report containing all open items.

        Raises ``KeyError`` if either shift id is unknown.
        """
        from_shift = self._shifts.get(from_shift_id)
        if from_shift is None:
            raise KeyError(f"Shift '{from_shift_id}' not found.")
        to_shift = self._shifts.get(to_shift_id)
        if to_shift is None:
            raise KeyError(f"Shift '{to_shift_id}' not found.")

        open_items = self.get_open_items()

        report = HandoffReport(
            report_id=str(uuid.uuid4()),
            from_shift=from_shift,
            to_shift=to_shift,
            items=list(open_items),
            created_at=time.time(),
            notes=notes,
        )

        with self._lock:
            self._reports[report.report_id] = report
            self._report_history.append(report)

        return report

    def acknowledge_handoff(self, report_id: str, operator: str) -> HandoffReport:
        """Mark a handoff report as acknowledged by *operator*.

        Also sets all contained items' status to ``'acknowledged'``.
        """
        with self._lock:
            if report_id not in self._reports:
                raise KeyError(f"Handoff report '{report_id}' not found.")
            report = self._reports[report_id]
            report.acknowledged_by = operator
            for item in report.items:
                if item.status == 'open':
                    item.status = 'acknowledged'
                    # Propagate to the master item store as well.
                    if item.item_id in self._items:
                        self._items[item.item_id].status = 'acknowledged'
        return report

    def get_handoff_history(self, last_n: int = 10) -> List[HandoffReport]:
        """Return the *last_n* most recent handoff reports (newest first)."""
        with self._lock:
            history = list(self._report_history)
        return list(reversed(history[-last_n:]))


# ---------------------------------------------------------------------------
# Changeover Optimizer (SMED)
# ---------------------------------------------------------------------------


@dataclass
class ChangeoverStep:
    """A single step in a machine changeover procedure."""

    step_id: str
    description: str
    duration_min: float
    category: str  # 'internal' | 'external' | 'waste'
    can_externalize: bool = False
    dependencies: List[str] = field(default_factory=list)
    tools_needed: List[str] = field(default_factory=list)


@dataclass
class ChangeoverAnalysis:
    """Result of analysing or optimising a changeover sequence."""

    total_time_min: float
    internal_time_min: float
    external_time_min: float
    waste_time_min: float
    optimized_time_min: float
    savings_min: float
    savings_pct: float
    recommendations: List[str] = field(default_factory=list)


class ChangeoverOptimizer:
    """SMED (Single Minute Exchange of Die) changeover optimizer.

    Implements the core SMED methodology:
    1. Separate internal and external setup operations.
    2. Convert internal operations to external where possible.
    3. Streamline remaining internal operations.
    4. Eliminate waste steps entirely.

    Usage::

        opt = ChangeoverOptimizer()
        opt.add_step(ChangeoverStep(...))
        analysis = opt.analyze()      # current-state analysis
        optimized = opt.optimize()    # future-state after SMED
        groups = opt.get_parallel_groups()
        path = opt.get_critical_path()
    """

    def __init__(self) -> None:
        self._steps: Dict[str, ChangeoverStep] = {}

    # -- mutators ------------------------------------------------------------

    def add_step(self, step: ChangeoverStep) -> None:
        """Register a changeover step."""
        self._steps[step.step_id] = step

    # -- analysis ------------------------------------------------------------

    def analyze(self) -> ChangeoverAnalysis:
        """Categorize and analyse all registered steps.

        Returns a :class:`ChangeoverAnalysis` reflecting the *current* state
        (no optimisation applied).
        """
        internal = 0.0
        external = 0.0
        waste = 0.0
        for step in self._steps.values():
            if step.category == 'internal':
                internal += step.duration_min
            elif step.category == 'external':
                external += step.duration_min
            elif step.category == 'waste':
                waste += step.duration_min

        total = internal + external + waste
        # In the unoptimised view the "optimized_time" equals total (no
        # savings yet).
        return ChangeoverAnalysis(
            total_time_min=total,
            internal_time_min=internal,
            external_time_min=external,
            waste_time_min=waste,
            optimized_time_min=total,
            savings_min=0.0,
            savings_pct=0.0,
            recommendations=self._generate_recommendations(),
        )

    # -- optimisation --------------------------------------------------------

    def optimize(self) -> ChangeoverAnalysis:
        """Apply SMED optimisation and return the improved analysis.

        Optimisation rules:
        * Waste steps are eliminated entirely.
        * Internal steps marked *can_externalize* are moved to external.
        * External steps can be performed while the machine is running, so
          only *internal* time counts towards machine downtime.

        The ``optimized_time_min`` therefore equals the remaining internal
        time after externalisation.
        """
        internal = 0.0
        external = 0.0
        waste = 0.0
        optimized_internal = 0.0
        externalized = 0.0

        for step in self._steps.values():
            if step.category == 'waste':
                waste += step.duration_min
            elif step.category == 'external':
                external += step.duration_min
            elif step.category == 'internal':
                internal += step.duration_min
                if step.can_externalize:
                    externalized += step.duration_min
                else:
                    optimized_internal += step.duration_min

        total = internal + external + waste
        # Savings = time removed from machine downtime.  Waste is eliminated,
        # and externalized internal steps move off-line.
        savings = waste + externalized
        savings_pct = (savings / total * 100.0) if total > 0 else 0.0

        return ChangeoverAnalysis(
            total_time_min=total,
            internal_time_min=internal,
            external_time_min=external,
            waste_time_min=waste,
            optimized_time_min=optimized_internal,
            savings_min=savings,
            savings_pct=savings_pct,
            recommendations=self._generate_recommendations(),
        )

    # -- dependency / scheduling helpers ------------------------------------

    def get_parallel_groups(self) -> List[List[str]]:
        """Identify groups of steps that can run in parallel.

        Two steps can run in parallel when neither depends on the other
        (directly or transitively).  The method returns a list of groups
        where each group is a list of step_ids that share no dependency
        conflicts and can therefore be executed concurrently.
        """
        if not self._steps:
            return []

        # Build adjacency (step -> set of direct dependents)
        dependents: Dict[str, Set[str]] = {sid: set() for sid in self._steps}
        for sid, step in self._steps.items():
            for dep in step.dependencies:
                if dep in dependents:
                    dependents[dep].add(sid)

        # Topological-layer grouping (Kahn's algorithm variant).
        in_degree: Dict[str, int] = {sid: 0 for sid in self._steps}
        for step in self._steps.values():
            for dep in step.dependencies:
                if dep in in_degree:
                    in_degree[step.step_id] += 1

        groups: List[List[str]] = []
        remaining = dict(in_degree)

        while remaining:
            layer = [sid for sid, deg in remaining.items() if deg == 0]
            if not layer:
                # Cycle detected – break out with remaining steps
                layer = list(remaining.keys())
            groups.append(sorted(layer))
            for sid in layer:
                del remaining[sid]
                for dep_sid in list(remaining):
                    step = self._steps[dep_sid]
                    if sid in step.dependencies:
                        remaining[dep_sid] = max(remaining[dep_sid] - 1, 0)

        return groups

    def get_critical_path(self) -> List[str]:
        """Return the longest chain of dependent *internal* steps.

        Only internal steps contribute to machine downtime; the critical
        path is the longest sequential chain of such steps (by total
        duration).
        """
        internal_ids = {
            sid for sid, s in self._steps.items() if s.category == 'internal'
        }
        if not internal_ids:
            return []

        # For each internal step compute the longest path ending at that step
        # considering only internal predecessors.
        memo: Dict[str, Tuple[float, List[str]]] = {}

        def _longest(sid: str) -> Tuple[float, List[str]]:
            if sid in memo:
                return memo[sid]
            step = self._steps[sid]
            best_dur = 0.0
            best_path: List[str] = []
            for dep in step.dependencies:
                if dep in internal_ids:
                    dur, path = _longest(dep)
                    if dur > best_dur:
                        best_dur = dur
                        best_path = path
            result = (best_dur + step.duration_min, best_path + [sid])
            memo[sid] = result
            return result

        longest_dur = 0.0
        longest_path: List[str] = []
        for sid in internal_ids:
            dur, path = _longest(sid)
            if dur > longest_dur:
                longest_dur = dur
                longest_path = path

        return longest_path

    # -- estimation ----------------------------------------------------------

    def estimate_savings(self, externalize_steps: List[str]) -> float:
        """Estimate time saved by externalising the given steps.

        Only internal steps that are currently *not* already external
        contribute to savings.  Returns the total duration (in minutes)
        that would be removed from machine downtime.
        """
        saved = 0.0
        for sid in externalize_steps:
            step = self._steps.get(sid)
            if step and step.category == 'internal':
                saved += step.duration_min
        return saved

    # -- checklist -----------------------------------------------------------

    def get_checklist(self) -> List[Dict[str, Any]]:
        """Generate an ordered changeover checklist.

        Internal steps come first (they must be done while the machine is
        stopped), then external steps (can be done before/after), waste
        steps are excluded.  Within each group, dependency order is
        respected.
        """
        ordered = self._topo_sort()

        internal: List[Dict[str, Any]] = []
        external: List[Dict[str, Any]] = []

        seq = 1
        for sid in ordered:
            step = self._steps[sid]
            if step.category == 'waste':
                continue
            entry = {
                'sequence': seq,
                'step_id': step.step_id,
                'description': step.description,
                'duration_min': step.duration_min,
                'category': step.category,
                'tools_needed': step.tools_needed,
            }
            if step.category == 'internal':
                internal.append(entry)
            else:
                external.append(entry)
            seq += 1

        # Re-number: internal first, then external
        result: List[Dict[str, Any]] = []
        seq = 1
        for item in internal + external:
            item['sequence'] = seq
            result.append(item)
            seq += 1
        return result

    # -- private helpers -----------------------------------------------------

    def _topo_sort(self) -> List[str]:
        """Topological sort of all steps respecting dependencies."""
        in_degree: Dict[str, int] = {sid: 0 for sid in self._steps}
        for step in self._steps.values():
            for dep in step.dependencies:
                if dep in in_degree:
                    in_degree[step.step_id] += 1

        queue: List[str] = sorted(
            [sid for sid, d in in_degree.items() if d == 0]
        )
        result: List[str] = []
        while queue:
            sid = queue.pop(0)
            result.append(sid)
            for other_sid, other_step in sorted(self._steps.items()):
                if sid in other_step.dependencies and other_sid in in_degree:
                    in_degree[other_sid] -= 1
                    if in_degree[other_sid] == 0:
                        queue.append(other_sid)
                        queue.sort()

        # Append any remaining (cycle) nodes
        for sid in sorted(self._steps):
            if sid not in result:
                result.append(sid)
        return result

    def _generate_recommendations(self) -> List[str]:
        """Generate SMED improvement recommendations."""
        recs: List[str] = []
        for step in self._steps.values():
            if step.category == 'internal' and step.can_externalize:
                recs.append(
                    f"Externalize '{step.step_id}': {step.description} "
                    f"(saves {step.duration_min:.1f} min)"
                )
            if step.category == 'waste':
                recs.append(
                    f"Eliminate waste '{step.step_id}': {step.description} "
                    f"(saves {step.duration_min:.1f} min)"
                )
        if not recs:
            recs.append("Changeover is already well-optimized.")
        return recs


def main(args=None):
    """Entry point for the job scheduler node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = JobSchedulerNode()
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
