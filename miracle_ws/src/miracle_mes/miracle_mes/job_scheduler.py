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
