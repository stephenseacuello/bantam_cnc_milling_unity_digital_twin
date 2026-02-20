"""
Checkpoint Manager Node.

Creates and manages state checkpoints for recovery.
Supports periodic and event-triggered checkpointing.
"""

from typing import Any, Dict, Optional
import json
import os
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, JobStatus
from miracle_msgs.srv import RestoreCheckpoint


class CheckpointManagerNode(MiracleLifecycleNode):
    """Manages state checkpoints for system recovery.

    Parameters:
        checkpoint_directory (str): Storage directory.
        checkpoint_interval_sec (float): Periodic checkpoint interval.
        max_checkpoints (int): Maximum stored checkpoints.

    Subscribed Topics:
        /miracle/+/state (MachineState): Machine states to checkpoint.
        /miracle/+/job_status (JobStatus): Job states to checkpoint.

    Service Servers:
        ~/restore_checkpoint (RestoreCheckpoint): Restore from checkpoint.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'checkpoint_manager',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._state_sub = None
        self._job_sub = None
        self._restore_srv = None
        self._checkpoint_timer = None
        self._current_state: Dict[str, Any] = {}
        self._checkpoint_dir: str = ''
        self._checkpoint_count: int = 0
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure checkpoint manager."""
        params = self.declare_and_validate_parameters({
            'checkpoint_directory': {
                'default': '/tmp/miracle_checkpoints', 'type': str,
            },
            'checkpoint_interval_sec': {
                'default': 60.0, 'type': float, 'range': (5.0, 3600.0),
            },
            'max_checkpoints': {
                'default': 50, 'type': int, 'range': (5, 1000),
            },
        })

        self._checkpoint_dir = params['checkpoint_directory']
        os.makedirs(self._checkpoint_dir, exist_ok=True)

        self._state_sub = self.create_subscription(
            MachineState, '/miracle/+/state',
            self._on_state, QoSProfiles.state_data(),
        )
        self._job_sub = self.create_subscription(
            JobStatus, '/miracle/+/job_status',
            self._on_job, QoSProfiles.state_data(),
        )
        self._restore_srv = self.create_service(
            RestoreCheckpoint, 'restore_checkpoint',
            self._handle_restore, callback_group=self.service_callback_group,
        )

        self.get_logger().info("Checkpoint manager configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        interval = self.get_parameter('checkpoint_interval_sec').value
        self._checkpoint_timer = self.create_timer(
            interval, self._create_checkpoint, callback_group=self.service_callback_group,
        )
        self.get_logger().info("Checkpoint manager activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        if self._checkpoint_timer is not None:
            self._checkpoint_timer.cancel()
            self._checkpoint_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_state(self, msg: MachineState) -> None:
        with self._lock:
            self._current_state[f'machine_{msg.machine_id}'] = {
                'status': msg.status,
                'spindle_speed': msg.spindle_speed,
                'feed_rate': msg.feed_rate,
                'program': msg.current_program,
                'line': msg.current_line,
            }

    def _on_job(self, msg: JobStatus) -> None:
        with self._lock:
            self._current_state[f'job_{msg.job_id}'] = {
                'status': msg.status,
                'machine': msg.machine_id,
                'progress': msg.progress,
            }

    def _create_checkpoint(self) -> None:
        """Create a state checkpoint."""
        with self._lock:
            if not self._current_state:
                return
            state_copy = dict(self._current_state)

        self._checkpoint_count += 1
        checkpoint_id = f"ckpt_{self._checkpoint_count:06d}"
        filepath = os.path.join(self._checkpoint_dir, f"{checkpoint_id}.json")

        data = {
            'checkpoint_id': checkpoint_id,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'state': state_copy,
        }

        try:
            with open(filepath, 'w') as f:
                json.dump(data, f, default=str)
            self.get_logger().debug(f"Checkpoint created: {checkpoint_id}")

            # Cleanup old checkpoints
            max_ckpt = self.get_parameter('max_checkpoints').value
            self._cleanup_old_checkpoints(max_ckpt)
        except Exception as exc:
            self.get_logger().error(f"Checkpoint error: {exc}")

    def _cleanup_old_checkpoints(self, max_count: int) -> None:
        """Remove old checkpoints beyond limit."""
        try:
            files = sorted([
                f for f in os.listdir(self._checkpoint_dir)
                if f.endswith('.json')
            ])
            while len(files) > max_count:
                os.remove(os.path.join(self._checkpoint_dir, files.pop(0)))
        except Exception as exc:
            self.get_logger().debug(f"Cleanup error: {exc}")

    def _handle_restore(
        self, request: RestoreCheckpoint.Request, response: RestoreCheckpoint.Response,
    ) -> RestoreCheckpoint.Response:
        """Handle checkpoint restore request."""
        checkpoint_id = request.checkpoint_id
        filepath = os.path.join(self._checkpoint_dir, f"{checkpoint_id}.json")

        if not os.path.exists(filepath):
            response.success = False
            response.message = f'Checkpoint {checkpoint_id} not found'
            return response

        try:
            with open(filepath, 'r') as f:
                data = json.load(f)

            with self._lock:
                self._current_state = data.get('state', {})

            response.success = True
            response.message = f'Restored checkpoint {checkpoint_id}'
            self.get_logger().info(f"Restored checkpoint: {checkpoint_id}")
        except Exception as exc:
            response.success = False
            response.message = str(exc)

        return response


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = CheckpointManagerNode()
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
