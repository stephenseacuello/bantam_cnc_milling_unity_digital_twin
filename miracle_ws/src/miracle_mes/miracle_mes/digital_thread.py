"""
Digital Thread Node.

Maintains an immutable, traceable record of all manufacturing data
from design to production. Implements a blockchain-like chain of
entries with hash links for tamper evidence.
"""

from typing import Any, Dict, List, Optional
import hashlib
import json
import threading
import time
import uuid

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import (
    DigitalThreadEntry, JobStatus, AnomalyAlert, MachineState,
)


class DigitalThreadNode(MiracleLifecycleNode):
    """Maintains immutable manufacturing data chain.

    Parameters:
        storage_path (str): Path for persistent storage.
        max_chain_memory (int): Max entries held in memory.

    Subscribed Topics:
        /miracle/{machine_id}/job_status (JobStatus): Job events.
        /miracle/{machine_id}/anomaly (AnomalyAlert): Anomaly events.
        /miracle/{machine_id}/state (MachineState): State snapshots.

    Published Topics:
        ~/entries (DigitalThreadEntry): New thread entries.
    """

    # Material genealogy entry type constants
    ENTRY_MATERIAL_LOADED = 'MATERIAL_LOADED'
    ENTRY_TOOL_INSTALLED = 'TOOL_INSTALLED'
    ENTRY_OPERATION_COMPLETE = 'OPERATION_COMPLETE'
    ENTRY_PART_COMPLETE = 'PART_COMPLETE'
    ENTRY_TOOL_REMOVED = 'TOOL_REMOVED'

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'digital_thread',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._entry_pub = None
        self._chain: List[DigitalThreadEntry] = []
        self._chain_lock = threading.Lock()
        self._last_hash: str = '0' * 64
        self._job_subs = None
        self._anomaly_subs = None
        self._state_subs = None

        # Material genealogy entries (dict-based, separate from ROS chain)
        self._entries: List[Dict[str, Any]] = []
        self._thread_lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure digital thread."""
        params = self.declare_and_validate_parameters({
            'storage_path': {
                'default': '/tmp/miracle_thread',
                'type': str,
            },
            'max_chain_memory': {
                'default': 10000,
                'type': int,
                'range': (100, 1000000),
            },
            'machine_ids': {
                'default': 'cnc1,cnc2,cnc3',
                'type': str,
            },
        })

        machine_ids = self.get_machine_ids(params)

        self._entry_pub = self.create_publisher(
            DigitalThreadEntry,
            'entries',
            QoSProfiles.logging(),
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

        self._state_subs = self.create_multi_machine_subscriptions(
            MachineState,
            'state',
            self._on_state,
            QoSProfiles.state_data(),
            machine_ids,
        )

        self.get_logger().info("Digital thread configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate digital thread."""
        self.get_logger().info("Digital thread activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate digital thread."""
        return TransitionCallbackReturn.SUCCESS

    def _add_entry(
        self,
        entry_type: str,
        job_id: str,
        source_node: str,
        data: Dict[str, Any],
        tags: List[str],
    ) -> None:
        """Add a new entry to the digital thread."""
        entry = DigitalThreadEntry()
        entry.timestamp = self.get_clock().now().to_msg()
        entry.entry_id = str(uuid.uuid4())[:12]
        entry.job_id = job_id
        entry.entry_type = entry_type
        entry.source_node = source_node
        entry.data_json = json.dumps(data, default=str)
        entry.tags = tags

        with self._chain_lock:
            entry.previous_entry_id = (
                self._chain[-1].entry_id if self._chain else ''
            )

            # Compute hash linking to previous entry
            hash_input = (
                f"{entry.entry_id}"
                f"{self._last_hash}"
                f"{entry.data_json}"
            )
            entry.hash_value = hashlib.sha256(
                hash_input.encode()
            ).hexdigest()

            self._chain.append(entry)
            self._last_hash = entry.hash_value

            # Trim in-memory chain
            max_mem = self.get_parameter('max_chain_memory').value
            if len(self._chain) > max_mem:
                self._chain = self._chain[-max_mem:]

        self._entry_pub.publish(entry)

    def _on_job_status(self, msg: JobStatus) -> None:
        """Record job status change in digital thread."""
        self._add_entry(
            entry_type='JOB_STATUS',
            job_id=msg.job_id,
            source_node=msg.machine_id,
            data={
                'status': msg.status,
                'program': msg.program_name,
                'progress': msg.progress,
                'current_line': msg.current_line,
            },
            tags=['job', msg.status.lower()],
        )

    def _on_state(self, msg: MachineState) -> None:
        """Record state snapshot in digital thread."""
        self._add_entry(
            entry_type='STATE_SNAPSHOT',
            job_id='',
            source_node=msg.machine_id,
            data={
                'status': msg.status,
                'spindle_speed': msg.spindle_speed,
                'feed_rate': msg.feed_rate,
                'axis_positions': list(msg.axis_positions),
                'spindle_load': msg.spindle_load,
                'current_program': msg.current_program,
            },
            tags=['state', msg.status.lower()],
        )

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        """Record anomaly in digital thread."""
        self._add_entry(
            entry_type='ANOMALY',
            job_id='',
            source_node=msg.machine_id,
            data={
                'type': msg.anomaly_type,
                'severity': msg.severity,
                'confidence': msg.confidence,
                'action': msg.recommended_action,
            },
            tags=['anomaly', msg.anomaly_type.lower()],
        )

    # ------------------------------------------------------------------
    # Material genealogy helpers
    # ------------------------------------------------------------------

    def _record_entry(self, entry: Dict[str, Any]) -> None:
        """Append an entry to the genealogy log."""
        with self._thread_lock:
            self._entries.append(entry)

    def record_genealogy_event(
        self, entry_type: str, machine_id: str,
        serial_number: str = '', tool_id: str = '',
        batch_id: str = '', metadata: Optional[Dict] = None,
    ) -> None:
        """Record a material genealogy event in the digital thread."""
        entry = {
            'entry_type': entry_type,
            'machine_id': machine_id,
            'timestamp': time.time(),
            'serial_number': serial_number,
            'tool_id': tool_id,
            'batch_id': batch_id,
        }
        if metadata:
            entry.update(metadata)
        self._record_entry(entry)

    def get_part_history(self, serial_number: str) -> List[Dict[str, Any]]:
        """Get complete manufacturing history for a part by serial number."""
        with self._thread_lock:
            return [
                e for e in self._entries
                if e.get('serial_number') == serial_number or
                   e.get('part_serial') == serial_number
            ]

    def get_tool_history(self, tool_id: str) -> List[Dict[str, Any]]:
        """Get usage history for a specific tool."""
        with self._thread_lock:
            return [
                e for e in self._entries
                if e.get('tool_id') == tool_id
            ]

    def get_batch_traceability(self, batch_id: str) -> List[Dict[str, Any]]:
        """Get all entries related to a material batch."""
        with self._thread_lock:
            return [
                e for e in self._entries
                if e.get('batch_id') == batch_id or
                   e.get('material_batch') == batch_id
            ]

    def verify_chain_integrity(self) -> bool:
        """Verify the integrity of the digital thread chain.

        Returns:
            True if chain integrity is maintained.
        """
        with self._chain_lock:
            prev_hash = '0' * 64
            for entry in self._chain:
                hash_input = (
                    f"{entry.entry_id}"
                    f"{prev_hash}"
                    f"{entry.data_json}"
                )
                expected = hashlib.sha256(hash_input.encode()).hexdigest()
                if entry.hash_value != expected:
                    self.get_logger().error(
                        f"Chain integrity violation at entry {entry.entry_id}"
                    )
                    return False
                prev_hash = entry.hash_value

        return True


def main(args=None):
    """Entry point for the digital thread node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = DigitalThreadNode()
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
