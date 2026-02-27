"""
Data Historian Node.

Time-series data archival with configurable retention policies.
Stores machine state, sensor data, and events for analysis.
"""

from typing import Any, Dict, List, Optional
from collections import deque
import json
import os
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_core.serialization import msg_to_dict
from miracle_msgs.msg import MachineState, AnomalyAlert, JobStatus


class HistorianNode(MiracleLifecycleNode):
    """Time-series data archival node.

    Parameters:
        storage_path (str): Base directory for data storage.
        buffer_size (int): In-memory buffer size before flush.
        flush_interval_sec (float): Periodic flush interval.
        retention_days (int): Data retention period in days.

    Subscribed Topics:
        /miracle/{machine_id}/state (MachineState): Machine states.
        /miracle/{machine_id}/anomaly (AnomalyAlert): Anomaly events.
        /miracle/{machine_id}/job_status (JobStatus): Job status updates.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'historian',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._state_subs = None
        self._anomaly_subs = None
        self._job_subs = None
        self._flush_timer = None

        self._buffer: deque = deque()
        self._buffer_lock = threading.Lock()
        self._storage_path: str = '/tmp/miracle_history'
        self._buffer_size: int = 1000
        self._flush_count: int = 0

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure historian."""
        params = self.declare_and_validate_parameters({
            'storage_path': {
                'default': '/tmp/miracle_history',
                'type': str,
            },
            'buffer_size': {
                'default': 1000,
                'type': int,
                'range': (100, 100000),
            },
            'flush_interval_sec': {
                'default': 10.0,
                'type': float,
                'range': (1.0, 300.0),
            },
            'retention_days': {
                'default': 90,
                'type': int,
                'range': (1, 3650),
            },
            'machine_ids': {
                'default': 'cnc1,cnc2,cnc3',
                'type': str,
            },
        })

        machine_ids = self.get_machine_ids(params)

        self._storage_path = params['storage_path']
        self._buffer_size = params['buffer_size']

        os.makedirs(self._storage_path, exist_ok=True)

        self._state_subs = self.create_multi_machine_subscriptions(
            MachineState,
            'state',
            self._on_state,
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

        self._job_subs = self.create_multi_machine_subscriptions(
            JobStatus,
            'job_status',
            self._on_job_status,
            QoSProfiles.state_data(),
            machine_ids,
        )

        self.get_logger().info(f"Historian configured (path={self._storage_path})")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate historian."""
        interval = self.get_parameter('flush_interval_sec').value
        self._flush_timer = self.create_timer(
            interval,
            self._flush_buffer,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("Historian activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Flush remaining data and deactivate."""
        if self._flush_timer is not None:
            self._flush_timer.cancel()
            self._flush_timer = None
        self._flush_buffer()
        return TransitionCallbackReturn.SUCCESS

    def _on_state(self, msg: MachineState) -> None:
        """Archive machine state."""
        self._add_to_buffer('machine_state', msg_to_dict(msg))

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        """Archive anomaly alert."""
        self._add_to_buffer('anomaly', msg_to_dict(msg))

    def _on_job_status(self, msg: JobStatus) -> None:
        """Archive job status."""
        self._add_to_buffer('job_status', msg_to_dict(msg))

    def _add_to_buffer(self, data_type: str, data: Dict) -> None:
        """Add data to in-memory buffer."""
        entry = {
            'type': data_type,
            'data': data,
            'archived_at': self.get_clock().now().nanoseconds / 1e9,
        }
        with self._buffer_lock:
            self._buffer.append(entry)
            if len(self._buffer) >= self._buffer_size:
                self._flush_buffer()

    def _flush_buffer(self) -> None:
        """Flush buffer to storage."""
        with self._buffer_lock:
            if not self._buffer:
                return

            entries = list(self._buffer)
            self._buffer.clear()

        self._flush_count += 1
        filename = os.path.join(
            self._storage_path,
            f"history_{self._flush_count:06d}.jsonl",
        )

        try:
            with open(filename, 'w') as f:
                for entry in entries:
                    f.write(json.dumps(entry, default=str) + '\n')

            self.get_logger().debug(
                f"Flushed {len(entries)} entries to {filename}"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to flush historian data: {exc}")


def main(args=None):
    """Entry point for the historian node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = HistorianNode()
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
