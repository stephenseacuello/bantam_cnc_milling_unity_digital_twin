"""
Job Scheduler Node.

Manages manufacturing job queue with priority scheduling.
Assigns jobs to machines based on capability, availability, and optimization.
Supports priority levels: LOW, NORMAL, HIGH, RUSH.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
from enum import IntEnum
import threading
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

            # Find idle machines
            idle_machines = [
                mid for mid, state in self._machine_states.items()
                if state.status in ('IDLE', 'READY')
            ]

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
                return result

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

        elapsed_total = (self.get_clock().now() - start).nanoseconds / 1e9
        goal_handle.succeed()
        result.success = True
        result.message = 'Job completed successfully'
        result.total_time_sec = elapsed_total
        result.oee_achieved = 0.85
        result.quality_metrics = [0.98, 0.95, 0.99]

        return result

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
