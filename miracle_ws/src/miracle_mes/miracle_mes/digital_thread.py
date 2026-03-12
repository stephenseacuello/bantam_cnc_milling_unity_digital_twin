"""
Digital Thread Node.

Maintains an immutable, traceable record of all manufacturing data
from design to production. Implements a blockchain-like chain of
entries with hash links for tamper evidence.
"""

from typing import Any, Dict, List, Optional, Tuple
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
    ENTRY_OPERATION_FAILED = 'OPERATION_FAILED'
    ENTRY_PART_COMPLETE = 'PART_COMPLETE'
    ENTRY_TOOL_REMOVED = 'TOOL_REMOVED'
    ENTRY_ANOMALY_DETECTED = 'ANOMALY_DETECTED'
    ENTRY_JOB_PAUSED = 'JOB_PAUSED'
    ENTRY_JOB_RESUMED = 'JOB_RESUMED'
    ENTRY_JOB_CANCELLED = 'JOB_CANCELLED'
    ENTRY_JOB_FAILED = 'JOB_FAILED'
    ENTRY_MACHINE_ERROR = 'MACHINE_ERROR'

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
        self._genealogy_last_hash: str = '0' * 64

        # Tool tracking per machine for auto-detection of tool changes
        self._current_tool_per_machine: Dict[str, str] = {}
        self._active_job_per_machine: Dict[str, Dict[str, Any]] = {}

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
        """Record state snapshot and detect tool changes in digital thread."""
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

        # Auto-detect tool changes from MachineState messages
        new_tool_id = getattr(msg, 'tool_id', '')
        if new_tool_id:
            old_tool_id = self._current_tool_per_machine.get(msg.machine_id, '')
            if old_tool_id and old_tool_id != new_tool_id:
                # Tool changed: record REMOVED for old, INSTALLED for new
                job_ctx = self._active_job_per_machine.get(msg.machine_id, {})
                self.record_genealogy_event(
                    self.ENTRY_TOOL_REMOVED,
                    machine_id=msg.machine_id,
                    tool_id=old_tool_id,
                    serial_number=job_ctx.get('serial_number', ''),
                    metadata={
                        'operator': job_ctx.get('operator', ''),
                        'job_id': job_ctx.get('job_id', ''),
                    },
                )
                self.record_genealogy_event(
                    self.ENTRY_TOOL_INSTALLED,
                    machine_id=msg.machine_id,
                    tool_id=new_tool_id,
                    serial_number=job_ctx.get('serial_number', ''),
                    metadata={
                        'operator': job_ctx.get('operator', ''),
                        'job_id': job_ctx.get('job_id', ''),
                    },
                )
            elif not old_tool_id:
                # First tool seen on this machine
                job_ctx = self._active_job_per_machine.get(msg.machine_id, {})
                self.record_genealogy_event(
                    self.ENTRY_TOOL_INSTALLED,
                    machine_id=msg.machine_id,
                    tool_id=new_tool_id,
                    serial_number=job_ctx.get('serial_number', ''),
                    metadata={
                        'operator': job_ctx.get('operator', ''),
                        'job_id': job_ctx.get('job_id', ''),
                    },
                )
            self._current_tool_per_machine[msg.machine_id] = new_tool_id

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        """Record anomaly in digital thread and link to active job genealogy."""
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

        # If there is an active job on this machine, link anomaly to genealogy
        job_ctx = self._active_job_per_machine.get(msg.machine_id, {})
        if job_ctx:
            self.record_genealogy_event(
                self.ENTRY_ANOMALY_DETECTED,
                machine_id=msg.machine_id,
                serial_number=job_ctx.get('serial_number', ''),
                batch_id=job_ctx.get('batch_id', ''),
                tool_id=self._current_tool_per_machine.get(msg.machine_id, ''),
                metadata={
                    'anomaly_type': msg.anomaly_type,
                    'severity': msg.severity,
                    'confidence': msg.confidence,
                    'recommended_action': msg.recommended_action,
                    'job_id': job_ctx.get('job_id', ''),
                },
            )

    # ------------------------------------------------------------------
    # Material genealogy helpers
    # ------------------------------------------------------------------

    def _record_entry(self, entry: Dict[str, Any]) -> None:
        """Append an entry to the genealogy log with hash chain integrity."""
        with self._thread_lock:
            # Compute genealogy hash chain
            hash_input = (
                f"{entry.get('entry_type', '')}"
                f"{entry.get('timestamp', '')}"
                f"{self._genealogy_last_hash}"
                f"{json.dumps(entry, default=str, sort_keys=True)}"
            )
            entry['previous_hash'] = self._genealogy_last_hash
            entry['hash'] = hashlib.sha256(hash_input.encode()).hexdigest()
            self._genealogy_last_hash = entry['hash']
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

    def set_active_job(
        self, machine_id: str, job_id: str,
        serial_number: str = '', batch_id: str = '',
        operator: str = '',
    ) -> None:
        """Register an active job context for a machine.

        This is used by external nodes (e.g. job_scheduler) to provide
        context for automatic genealogy event linking.
        """
        self._active_job_per_machine[machine_id] = {
            'job_id': job_id,
            'serial_number': serial_number,
            'batch_id': batch_id,
            'operator': operator,
        }

    def clear_active_job(self, machine_id: str) -> None:
        """Clear the active job context for a machine."""
        self._active_job_per_machine.pop(machine_id, None)

    def record_operation_complete(
        self, machine_id: str, tool_id: str,
        operation_type: str, start_time: float, end_time: float,
        serial_number: str = '', batch_id: str = '',
        material_removed_volume: float = 0.0,
        surface_finish_ra: float = 0.0,
        metadata: Optional[Dict] = None,
    ) -> None:
        """Convenience method to record OPERATION_COMPLETE with full details."""
        meta = {
            'operation_type': operation_type,
            'start_time': start_time,
            'end_time': end_time,
            'duration_sec': end_time - start_time,
            'material_removed_volume': material_removed_volume,
            'surface_finish_ra': surface_finish_ra,
        }
        if metadata:
            meta.update(metadata)
        self.record_genealogy_event(
            self.ENTRY_OPERATION_COMPLETE,
            machine_id=machine_id,
            tool_id=tool_id,
            serial_number=serial_number,
            batch_id=batch_id,
            metadata=meta,
        )

    def get_full_part_genealogy(
        self, serial_number: str,
    ) -> List[Dict[str, Any]]:
        """Return a complete, sorted timeline for a part.

        Returns all genealogy entries (material loaded, tool installed,
        operations, anomalies, tool removed, part complete) for the
        given serial number, sorted by timestamp.
        """
        with self._thread_lock:
            entries = [
                e for e in self._entries
                if e.get('serial_number') == serial_number or
                   e.get('part_serial') == serial_number
            ]
        return sorted(entries, key=lambda e: e.get('timestamp', 0))

    def get_machine_utilization_history(
        self, machine_id: str, hours: float = 24,
    ) -> List[Tuple[float, float, str, str]]:
        """Return utilization history for a machine.

        Returns a list of (start_time, end_time, event_type, job_id) tuples
        for all genealogy entries on the given machine within the specified
        time window.
        """
        cutoff = time.time() - (hours * 3600)
        with self._thread_lock:
            entries = [
                e for e in self._entries
                if e.get('machine_id') == machine_id
                and e.get('timestamp', 0) >= cutoff
            ]
        entries.sort(key=lambda e: e.get('timestamp', 0))
        result = []
        for entry in entries:
            ts = entry.get('timestamp', 0)
            start = entry.get('start_time', ts)
            end = entry.get('end_time', ts)
            result.append((
                start,
                end,
                entry.get('entry_type', ''),
                entry.get('job_id', ''),
            ))
        return result

    def verify_genealogy_integrity(self) -> bool:
        """Verify the hash chain integrity of genealogy entries.

        Returns:
            True if the genealogy chain is intact.
        """
        with self._thread_lock:
            prev_hash = '0' * 64
            for entry in self._entries:
                if entry.get('previous_hash') != prev_hash:
                    return False
                # Recompute hash — need a copy without hash/previous_hash
                entry_copy = {
                    k: v for k, v in entry.items()
                    if k not in ('hash', 'previous_hash')
                }
                hash_input = (
                    f"{entry.get('entry_type', '')}"
                    f"{entry.get('timestamp', '')}"
                    f"{prev_hash}"
                    f"{json.dumps(entry_copy, default=str, sort_keys=True)}"
                )
                expected = hashlib.sha256(hash_input.encode()).hexdigest()
                if entry.get('hash') != expected:
                    return False
                prev_hash = entry['hash']
        return True

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
