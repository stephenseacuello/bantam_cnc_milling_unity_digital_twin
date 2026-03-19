"""
Digital Thread Node.

Maintains an immutable, traceable record of all manufacturing data
from design to production. Implements a blockchain-like chain of
entries with hash links for tamper evidence.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import hashlib
import json
import threading
import time
import uuid


# ------------------------------------------------------------------
# Energy consumption tracking
# ------------------------------------------------------------------

@dataclass
class EnergyProfile:
    """Energy consumption profile for a machining operation or time window."""

    total_kwh: float = 0.0
    spindle_kwh: float = 0.0
    axis_kwh: float = 0.0
    coolant_kwh: float = 0.0
    auxiliary_kwh: float = 0.0
    idle_kwh: float = 0.0
    peak_power_kw: float = 0.0
    avg_power_kw: float = 0.0
    energy_per_part_kwh: float = 0.0
    energy_per_cm3_wh: float = 0.0
    carbon_footprint_kg: float = 0.0


class EnergyTracker:
    """Tracks and analyses energy consumption across CNC subsystems.

    Collects timestamped power samples and provides methods for
    computing energy profiles, trends, idle waste, and program
    comparisons.
    """

    def __init__(self, grid_emission_factor_kg_per_kwh: float = 0.4) -> None:
        self._grid_emission_factor = grid_emission_factor_kg_per_kwh
        self._power_log: List[Dict[str, float]] = []
        self._log_lock = threading.Lock()

    # -- recording ---------------------------------------------------

    def record_power(
        self,
        timestamp: float,
        spindle_kw: float,
        axis_kw: float,
        coolant_kw: float,
        auxiliary_kw: float,
    ) -> None:
        """Record a timestamped power sample."""
        sample = {
            'timestamp': timestamp,
            'spindle_kw': spindle_kw,
            'axis_kw': axis_kw,
            'coolant_kw': coolant_kw,
            'auxiliary_kw': auxiliary_kw,
            'total_kw': spindle_kw + axis_kw + coolant_kw + auxiliary_kw,
        }
        with self._log_lock:
            self._power_log.append(sample)

    # -- computation -------------------------------------------------

    def compute_energy_profile(
        self,
        start_time: float,
        end_time: float,
        parts_produced: int = 1,
        volume_removed_cm3: float = 0.0,
    ) -> EnergyProfile:
        """Compute an *EnergyProfile* from recorded power samples.

        Uses the trapezoidal rule to integrate power over time for each
        subsystem.  Idle energy is computed as the sum of auxiliary
        power samples that fall below a 0.5 kW total threshold.
        """
        with self._log_lock:
            samples = [
                s for s in self._power_log
                if start_time <= s['timestamp'] <= end_time
            ]

        if len(samples) < 2:
            # Not enough data points for integration
            profile = EnergyProfile()
            if len(samples) == 1:
                profile.peak_power_kw = samples[0]['total_kw']
                profile.avg_power_kw = samples[0]['total_kw']
            return profile

        samples.sort(key=lambda s: s['timestamp'])

        spindle_kwh = 0.0
        axis_kwh = 0.0
        coolant_kwh = 0.0
        auxiliary_kwh = 0.0
        idle_kwh = 0.0
        peak_kw = 0.0

        for i in range(1, len(samples)):
            dt_h = (samples[i]['timestamp'] - samples[i - 1]['timestamp']) / 3600.0
            # Trapezoidal rule per subsystem
            spindle_kwh += 0.5 * (samples[i - 1]['spindle_kw'] + samples[i]['spindle_kw']) * dt_h
            axis_kwh += 0.5 * (samples[i - 1]['axis_kw'] + samples[i]['axis_kw']) * dt_h
            coolant_kwh += 0.5 * (samples[i - 1]['coolant_kw'] + samples[i]['coolant_kw']) * dt_h
            auxiliary_kwh += 0.5 * (samples[i - 1]['auxiliary_kw'] + samples[i]['auxiliary_kw']) * dt_h

            # Idle energy: intervals where both endpoints are below idle threshold
            idle_threshold = 0.5
            if samples[i - 1]['total_kw'] < idle_threshold and samples[i]['total_kw'] < idle_threshold:
                idle_kwh += 0.5 * (samples[i - 1]['total_kw'] + samples[i]['total_kw']) * dt_h

            for s in (samples[i - 1], samples[i]):
                if s['total_kw'] > peak_kw:
                    peak_kw = s['total_kw']

        total_kwh = spindle_kwh + axis_kwh + coolant_kwh + auxiliary_kwh
        duration_h = (samples[-1]['timestamp'] - samples[0]['timestamp']) / 3600.0
        avg_kw = total_kwh / duration_h if duration_h > 0 else 0.0

        parts = max(parts_produced, 1)
        energy_per_part = total_kwh / parts
        energy_per_cm3 = (total_kwh * 1000.0 / volume_removed_cm3) if volume_removed_cm3 > 0 else 0.0
        carbon = total_kwh * self._grid_emission_factor

        return EnergyProfile(
            total_kwh=total_kwh,
            spindle_kwh=spindle_kwh,
            axis_kwh=axis_kwh,
            coolant_kwh=coolant_kwh,
            auxiliary_kwh=auxiliary_kwh,
            idle_kwh=idle_kwh,
            peak_power_kw=peak_kw,
            avg_power_kw=avg_kw,
            energy_per_part_kwh=energy_per_part,
            energy_per_cm3_wh=energy_per_cm3,
            carbon_footprint_kg=carbon,
        )

    # -- breakdown ---------------------------------------------------

    def get_power_breakdown(
        self, start_time: float, end_time: float,
    ) -> Dict[str, float]:
        """Return subsystem energy as a percentage of total."""
        profile = self.compute_energy_profile(start_time, end_time)
        total = profile.total_kwh
        if total <= 0:
            return {
                'spindle_pct': 0.0,
                'axis_pct': 0.0,
                'coolant_pct': 0.0,
                'auxiliary_pct': 0.0,
            }
        return {
            'spindle_pct': profile.spindle_kwh / total * 100.0,
            'axis_pct': profile.axis_kwh / total * 100.0,
            'coolant_pct': profile.coolant_kwh / total * 100.0,
            'auxiliary_pct': profile.auxiliary_kwh / total * 100.0,
        }

    # -- trend -------------------------------------------------------

    def get_energy_trend(
        self, hours_back: float = 24, slot_hours: float = 1,
    ) -> List[Tuple[float, float]]:
        """Return energy per time slot over the last *hours_back* hours.

        Returns a list of ``(slot_start_timestamp, total_kwh)`` tuples.
        """
        now = time.time()
        start = now - hours_back * 3600.0
        slots: List[Tuple[float, float]] = []
        cursor = start
        while cursor < now:
            slot_end = min(cursor + slot_hours * 3600.0, now)
            profile = self.compute_energy_profile(cursor, slot_end)
            slots.append((cursor, profile.total_kwh))
            cursor = slot_end
        return slots

    # -- estimation --------------------------------------------------

    def estimate_job_energy(
        self,
        duration_min: float,
        avg_spindle_load_pct: float,
        spindle_power_kw: float = 5.5,
    ) -> float:
        """Estimate total energy (kWh) for a job based on spindle load."""
        duration_h = duration_min / 60.0
        spindle_kwh = spindle_power_kw * (avg_spindle_load_pct / 100.0) * duration_h
        # Assume axis ~15 %, coolant ~10 %, auxiliary ~5 % of spindle power
        axis_kwh = spindle_power_kw * 0.15 * duration_h
        coolant_kwh = spindle_power_kw * 0.10 * duration_h
        aux_kwh = spindle_power_kw * 0.05 * duration_h
        return spindle_kwh + axis_kwh + coolant_kwh + aux_kwh

    # -- idle waste --------------------------------------------------

    def get_idle_energy_waste(
        self,
        start_time: float,
        end_time: float,
        idle_threshold_kw: float = 0.5,
    ) -> Tuple[float, float, float]:
        """Compute idle energy waste.

        Returns ``(idle_kwh, idle_pct, idle_cost_estimate)`` where
        cost is estimated at $0.12/kWh.
        """
        with self._log_lock:
            samples = [
                s for s in self._power_log
                if start_time <= s['timestamp'] <= end_time
            ]
        if len(samples) < 2:
            return (0.0, 0.0, 0.0)

        samples.sort(key=lambda s: s['timestamp'])

        total_kwh = 0.0
        idle_kwh = 0.0
        for i in range(1, len(samples)):
            dt_h = (samples[i]['timestamp'] - samples[i - 1]['timestamp']) / 3600.0
            seg_kwh = 0.5 * (samples[i - 1]['total_kw'] + samples[i]['total_kw']) * dt_h
            total_kwh += seg_kwh
            if samples[i - 1]['total_kw'] < idle_threshold_kw and samples[i]['total_kw'] < idle_threshold_kw:
                idle_kwh += seg_kwh

        idle_pct = (idle_kwh / total_kwh * 100.0) if total_kwh > 0 else 0.0
        cost_per_kwh = 0.12
        return (idle_kwh, idle_pct, idle_kwh * cost_per_kwh)

    # -- program comparison ------------------------------------------

    @staticmethod
    def compare_programs(
        program_a_profile: EnergyProfile,
        program_b_profile: EnergyProfile,
    ) -> Dict[str, float]:
        """Compare two program energy profiles.

        Returns a dict with relative differences (positive means B uses
        more energy than A).
        """
        def _pct_diff(a: float, b: float) -> float:
            if a == 0:
                return 0.0 if b == 0 else 100.0
            return (b - a) / a * 100.0

        return {
            'total_kwh_diff_pct': _pct_diff(program_a_profile.total_kwh, program_b_profile.total_kwh),
            'spindle_kwh_diff_pct': _pct_diff(program_a_profile.spindle_kwh, program_b_profile.spindle_kwh),
            'peak_power_diff_pct': _pct_diff(program_a_profile.peak_power_kw, program_b_profile.peak_power_kw),
            'energy_per_part_diff_pct': _pct_diff(program_a_profile.energy_per_part_kwh, program_b_profile.energy_per_part_kwh),
            'carbon_diff_pct': _pct_diff(program_a_profile.carbon_footprint_kg, program_b_profile.carbon_footprint_kg),
        }

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

    # Prediction and calibration entry types
    PREDICTION_RECORDED = 'PREDICTION_RECORDED'
    PREDICTION_COMPARED = 'PREDICTION_COMPARED'
    CALIBRATION_APPLIED = 'CALIBRATION_APPLIED'
    CALIBRATION_REVERTED = 'CALIBRATION_REVERTED'

    # Energy tracking entry types
    ENERGY_RECORDED = 'energy_recorded'
    ENERGY_OPTIMIZATION = 'energy_optimization'

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

    # ------------------------------------------------------------------
    # Prediction accuracy tracking
    # ------------------------------------------------------------------

    def record_prediction(
        self,
        machine_id: str,
        program_name: str,
        block_index: int,
        predicted_force: float,
        predicted_temp: float,
        predicted_wear: float,
        predicted_rul_min: float,
        anomaly_markers: Optional[List[str]] = None,
    ) -> None:
        """Record a prediction for later comparison with actuals."""
        entry = {
            'entry_type': self.PREDICTION_RECORDED,
            'machine_id': machine_id,
            'program_name': program_name,
            'block_index': block_index,
            'predicted_force': predicted_force,
            'predicted_temp': predicted_temp,
            'predicted_wear': predicted_wear,
            'predicted_rul_min': predicted_rul_min,
            'anomaly_markers': anomaly_markers or [],
            'timestamp': time.time(),
        }
        self._record_entry(entry)

    def record_prediction_comparison(
        self,
        machine_id: str,
        program_name: str,
        block_index: int,
        predicted_force: float,
        actual_force: float,
        predicted_temp: float,
        actual_temp: float,
        force_error_pct: float,
        temp_error_pct: float,
    ) -> None:
        """Record comparison of prediction vs actual for traceability."""
        entry = {
            'entry_type': self.PREDICTION_COMPARED,
            'machine_id': machine_id,
            'program_name': program_name,
            'block_index': block_index,
            'predicted_force': predicted_force,
            'actual_force': actual_force,
            'predicted_temp': predicted_temp,
            'actual_temp': actual_temp,
            'force_error_pct': force_error_pct,
            'temp_error_pct': temp_error_pct,
            'timestamp': time.time(),
        }
        self._record_entry(entry)

    def record_calibration_event(
        self,
        machine_id: str,
        tool_id: str,
        calibration_type: str,
        adjustments: dict,
        reason: str,
        blocks_analyzed: int,
    ) -> None:
        """Log a calibration event in the digital thread."""
        entry = {
            'entry_type': self.CALIBRATION_APPLIED,
            'machine_id': machine_id,
            'tool_id': tool_id,
            'calibration_type': calibration_type,
            'adjustments': adjustments,
            'reason': reason,
            'blocks_analyzed': blocks_analyzed,
            'timestamp': time.time(),
        }
        self._record_entry(entry)

    def get_prediction_accuracy_history(
        self,
        machine_id: Optional[str] = None,
        program_name: Optional[str] = None,
        last_n: int = 100,
    ) -> List[Dict[str, Any]]:
        """Query prediction comparison history for trend analysis."""
        with self._thread_lock:
            results = [
                e for e in self._entries
                if e.get('entry_type') == self.PREDICTION_COMPARED
            ]
            if machine_id is not None:
                results = [e for e in results if e.get('machine_id') == machine_id]
            if program_name is not None:
                results = [e for e in results if e.get('program_name') == program_name]
            results.sort(key=lambda e: e.get('timestamp', 0))
            return results[-last_n:]

    def get_calibration_history(
        self,
        machine_id: Optional[str] = None,
        tool_id: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """Query calibration events for audit trail."""
        with self._thread_lock:
            results = [
                e for e in self._entries
                if e.get('entry_type') == self.CALIBRATION_APPLIED
            ]
            if machine_id is not None:
                results = [e for e in results if e.get('machine_id') == machine_id]
            if tool_id is not None:
                results = [e for e in results if e.get('tool_id') == tool_id]
            results.sort(key=lambda e: e.get('timestamp', 0))
            return results

    def compute_model_accuracy_summary(self, machine_id: str) -> dict:
        """Compute overall model accuracy metrics from comparison history.

        Returns:
            {
                'total_comparisons': int,
                'mean_force_error_pct': float,
                'mean_temp_error_pct': float,
                'force_r_squared': float,
                'calibrations_applied': int,
                'accuracy_trend': str,  # "improving", "stable", "degrading"
            }
        """
        comparisons = self.get_prediction_accuracy_history(machine_id=machine_id, last_n=10000)
        calibrations = self.get_calibration_history(machine_id=machine_id)

        if not comparisons:
            return {
                'total_comparisons': 0,
                'mean_force_error_pct': 0.0,
                'mean_temp_error_pct': 0.0,
                'force_r_squared': 0.0,
                'calibrations_applied': len(calibrations),
                'accuracy_trend': 'stable',
            }

        force_errors = [abs(c['force_error_pct']) for c in comparisons]
        temp_errors = [abs(c['temp_error_pct']) for c in comparisons]
        mean_force_err = sum(force_errors) / len(force_errors)
        mean_temp_err = sum(temp_errors) / len(temp_errors)

        # Compute R-squared for force predictions
        predicted = [c['predicted_force'] for c in comparisons]
        actual = [c['actual_force'] for c in comparisons]
        force_r_sq = self._compute_r_squared(predicted, actual)

        # Determine accuracy trend from recent vs older comparisons
        accuracy_trend = self._classify_accuracy_trend(force_errors)

        return {
            'total_comparisons': len(comparisons),
            'mean_force_error_pct': round(mean_force_err, 4),
            'mean_temp_error_pct': round(mean_temp_err, 4),
            'force_r_squared': round(force_r_sq, 4),
            'calibrations_applied': len(calibrations),
            'accuracy_trend': accuracy_trend,
        }

    @staticmethod
    def _compute_r_squared(predicted: List[float], actual: List[float]) -> float:
        """Compute R-squared (coefficient of determination)."""
        n = len(actual)
        if n < 2:
            return 0.0
        mean_actual = sum(actual) / n
        ss_tot = sum((a - mean_actual) ** 2 for a in actual)
        if ss_tot == 0:
            return 1.0 if all(p == a for p, a in zip(predicted, actual)) else 0.0
        ss_res = sum((a - p) ** 2 for a, p in zip(actual, predicted))
        return max(0.0, 1.0 - ss_res / ss_tot)

    @staticmethod
    def _classify_accuracy_trend(errors: List[float]) -> str:
        """Classify accuracy trend as improving, stable, or degrading."""
        if len(errors) < 6:
            return 'stable'
        mid = len(errors) // 2
        first_half_mean = sum(errors[:mid]) / mid
        second_half_mean = sum(errors[mid:]) / (len(errors) - mid)
        if first_half_mean == 0:
            return 'stable' if second_half_mean == 0 else 'degrading'
        ratio = second_half_mean / first_half_mean
        if ratio < 0.85:
            return 'improving'
        elif ratio > 1.15:
            return 'degrading'
        return 'stable'

    # ------------------------------------------------------------------
    # Energy tracking
    # ------------------------------------------------------------------

    def record_energy_profile(
        self,
        machine_id: str,
        program_id: str,
        profile: 'EnergyProfile',
    ) -> None:
        """Record an energy profile entry in the digital thread."""
        from dataclasses import asdict
        entry = {
            'entry_type': self.ENERGY_RECORDED,
            'machine_id': machine_id,
            'program_id': program_id,
            'timestamp': time.time(),
        }
        entry.update(asdict(profile))
        self._record_entry(entry)

    def get_energy_history(
        self,
        machine_id: Optional[str] = None,
        last_n: int = 20,
    ) -> List[Dict[str, Any]]:
        """Return recent energy profile entries from the digital thread."""
        with self._thread_lock:
            results = [
                e for e in self._entries
                if e.get('entry_type') == self.ENERGY_RECORDED
            ]
            if machine_id is not None:
                results = [e for e in results if e.get('machine_id') == machine_id]
            results.sort(key=lambda e: e.get('timestamp', 0))
            return results[-last_n:]

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


# ------------------------------------------------------------------
# Batch traceability
# ------------------------------------------------------------------

@dataclass
class MaterialBatch:
    """A material batch received from a supplier."""

    batch_id: str
    material_type: str
    supplier: str
    lot_number: str
    received_date: float
    quantity: float
    unit: str
    certifications: List[str] = field(default_factory=list)
    properties: Dict[str, Any] = field(default_factory=dict)


@dataclass
class BatchUsageRecord:
    """A record of material batch consumption during manufacturing."""

    batch_id: str
    job_id: str
    machine_id: str
    quantity_used: float
    timestamp: float
    operation: str


class BatchTraceabilityManager:
    """Tracks material batches through the manufacturing process.

    Provides forward traceability (batch -> jobs) and backward
    traceability (job -> batches), remaining quantity tracking,
    certification verification, and shelf-life expiry detection.
    """

    def __init__(self) -> None:
        self._batches: Dict[str, MaterialBatch] = {}
        self._usage_records: List[BatchUsageRecord] = []
        self._lock = threading.Lock()

    # -- registration ------------------------------------------------

    def register_batch(self, batch: MaterialBatch) -> None:
        """Register a new material batch."""
        with self._lock:
            self._batches[batch.batch_id] = batch

    # -- usage recording ---------------------------------------------

    def record_usage(
        self,
        batch_id: str,
        job_id: str,
        machine_id: str,
        quantity: float,
        operation: str,
    ) -> None:
        """Record consumption of material from a batch."""
        record = BatchUsageRecord(
            batch_id=batch_id,
            job_id=job_id,
            machine_id=machine_id,
            quantity_used=quantity,
            timestamp=time.time(),
            operation=operation,
        )
        with self._lock:
            self._usage_records.append(record)

    # -- queries -----------------------------------------------------

    def get_batch_history(self, batch_id: str) -> List[BatchUsageRecord]:
        """Return all usage records for a given batch."""
        with self._lock:
            return [r for r in self._usage_records if r.batch_id == batch_id]

    def get_job_materials(self, job_id: str) -> List[BatchUsageRecord]:
        """Return all batch usage records associated with a job."""
        with self._lock:
            return [r for r in self._usage_records if r.job_id == job_id]

    def get_remaining_quantity(self, batch_id: str) -> float:
        """Compute the remaining quantity for a batch.

        Returns the original quantity minus the sum of all recorded
        usage.  Returns ``0.0`` if the batch is unknown.
        """
        with self._lock:
            batch = self._batches.get(batch_id)
            if batch is None:
                return 0.0
            used = sum(
                r.quantity_used
                for r in self._usage_records
                if r.batch_id == batch_id
            )
            return batch.quantity - used

    # -- traceability ------------------------------------------------

    def trace_forward(self, batch_id: str) -> List[str]:
        """Forward traceability: find all jobs that used a batch.

        Returns a deduplicated list of job IDs.
        """
        with self._lock:
            seen: Dict[str, None] = {}
            for r in self._usage_records:
                if r.batch_id == batch_id:
                    seen[r.job_id] = None
            return list(seen.keys())

    def trace_backward(self, job_id: str) -> List[str]:
        """Backward traceability: find all batches that went into a job.

        Returns a deduplicated list of batch IDs.
        """
        with self._lock:
            seen: Dict[str, None] = {}
            for r in self._usage_records:
                if r.job_id == job_id:
                    seen[r.batch_id] = None
            return list(seen.keys())

    # -- certification -----------------------------------------------

    def check_certification(self, batch_id: str, required_cert: str) -> bool:
        """Verify that a batch holds a required certification.

        Returns ``False`` if the batch is unknown or lacks the
        certification.
        """
        with self._lock:
            batch = self._batches.get(batch_id)
            if batch is None:
                return False
            return required_cert in batch.certifications

    # -- shelf-life expiry -------------------------------------------

    def get_expiring_batches(self, days_ahead: float) -> List[MaterialBatch]:
        """Find batches whose shelf life expires within *days_ahead* days.

        Only considers batches that have a ``shelf_life_days`` key in
        their *properties* dict.  A batch expires when
        ``received_date + shelf_life_days * 86400`` is within
        *days_ahead* days from now.
        """
        now = time.time()
        horizon = now + days_ahead * 86400.0
        with self._lock:
            expiring: List[MaterialBatch] = []
            for batch in self._batches.values():
                shelf_life = batch.properties.get('shelf_life_days')
                if shelf_life is None:
                    continue
                expiry_ts = batch.received_date + float(shelf_life) * 86400.0
                if expiry_ts <= horizon:
                    expiring.append(batch)
            return expiring


# ------------------------------------------------------------------
# Production order scheduling
# ------------------------------------------------------------------

@dataclass
class ProductionOrder:
    """A production order to be scheduled across available machines."""

    order_id: str
    part_number: str
    quantity: int
    due_date: float
    priority: int  # 1 (highest) to 5 (lowest)
    estimated_time_per_part_min: float
    required_operations: List[str] = field(default_factory=list)
    material_type: str = ''
    customer: str = ''
    status: str = 'pending'  # pending | scheduled | in_progress | completed | late


@dataclass
class ScheduleEntry:
    """An assignment of a production order (or portion) to a machine."""

    order_id: str
    machine_id: str
    start_time: float
    end_time: float
    quantity: int


@dataclass
class ScheduleResult:
    """The output of a scheduling run."""

    entries: List[ScheduleEntry] = field(default_factory=list)
    unscheduled_orders: List[str] = field(default_factory=list)
    total_makespan_min: float = 0.0
    machine_utilization: Dict[str, float] = field(default_factory=dict)
    on_time_pct: float = 0.0


class ProductionOrderScheduler:
    """Schedules production orders across available machines.

    Uses an earliest-due-date-first strategy with priority as a
    tiebreaker (lower priority value == higher urgency).  Maintains
    an internal order store, schedule entries, and provides queries
    for late orders, per-machine schedules, and production summaries.
    """

    def __init__(self) -> None:
        self._orders: Dict[str, ProductionOrder] = {}
        self._schedule_entries: List[ScheduleEntry] = []
        self._lock = threading.Lock()

    # -- order management --------------------------------------------

    def add_order(self, order: ProductionOrder) -> None:
        """Add a production order to the scheduler."""
        with self._lock:
            self._orders[order.order_id] = order

    def remove_order(self, order_id: str) -> bool:
        """Remove a production order by ID.  Returns True if found."""
        with self._lock:
            if order_id in self._orders:
                del self._orders[order_id]
                # Also remove associated schedule entries
                self._schedule_entries = [
                    e for e in self._schedule_entries
                    if e.order_id != order_id
                ]
                return True
            return False

    def get_order(self, order_id: str) -> Optional[ProductionOrder]:
        """Retrieve a production order by ID, or None if not found."""
        with self._lock:
            return self._orders.get(order_id)

    def update_order_status(self, order_id: str, status: str) -> bool:
        """Update the status of a production order.

        Valid statuses: pending, scheduled, in_progress, completed, late.
        Returns True if the order was found and updated.
        """
        valid_statuses = {'pending', 'scheduled', 'in_progress', 'completed', 'late'}
        if status not in valid_statuses:
            return False
        with self._lock:
            order = self._orders.get(order_id)
            if order is None:
                return False
            order.status = status
            return True

    # -- scheduling --------------------------------------------------

    def schedule(
        self,
        machine_ids: List[str],
        current_time: float,
    ) -> ScheduleResult:
        """Schedule all pending orders using earliest-due-date-first.

        Orders are sorted by due date ascending, with priority (lower
        is more urgent) as the tiebreaker.  Each order is assigned to
        the machine with the earliest available end time.

        Args:
            machine_ids: Available machine identifiers.
            current_time: The reference "now" timestamp (epoch seconds).

        Returns:
            A *ScheduleResult* with entries, unscheduled orders,
            makespan, machine utilisation, and on-time percentage.
        """
        with self._lock:
            pending = [
                o for o in self._orders.values()
                if o.status == 'pending'
            ]

        if not pending or not machine_ids:
            unscheduled = [o.order_id for o in pending] if not machine_ids else []
            return ScheduleResult(
                entries=[],
                unscheduled_orders=unscheduled,
                total_makespan_min=0.0,
                machine_utilization={mid: 0.0 for mid in machine_ids},
                on_time_pct=100.0 if not pending else 0.0,
            )

        # Sort: earliest due date first, then lower priority value first
        pending.sort(key=lambda o: (o.due_date, o.priority))

        # Track machine availability (next free timestamp)
        machine_avail: Dict[str, float] = {mid: current_time for mid in machine_ids}
        entries: List[ScheduleEntry] = []
        unscheduled: List[str] = []

        for order in pending:
            # Pick the machine with the earliest availability
            best_machine = min(machine_avail, key=machine_avail.get)  # type: ignore[arg-type]
            start = machine_avail[best_machine]
            total_time_min = order.estimated_time_per_part_min * order.quantity
            end = start + total_time_min * 60.0  # convert minutes to seconds

            entry = ScheduleEntry(
                order_id=order.order_id,
                machine_id=best_machine,
                start_time=start,
                end_time=end,
                quantity=order.quantity,
            )
            entries.append(entry)
            machine_avail[best_machine] = end

            # Update order status
            with self._lock:
                order.status = 'scheduled'

        # Store schedule entries
        with self._lock:
            self._schedule_entries = list(entries)

        # Compute makespan (in minutes)
        if entries:
            latest_end = max(e.end_time for e in entries)
            total_makespan_min = (latest_end - current_time) / 60.0
        else:
            total_makespan_min = 0.0

        # Machine utilisation (% of makespan that each machine is busy)
        machine_utilization: Dict[str, float] = {}
        for mid in machine_ids:
            busy_sec = sum(
                e.end_time - e.start_time
                for e in entries
                if e.machine_id == mid
            )
            if total_makespan_min > 0:
                machine_utilization[mid] = (
                    busy_sec / (total_makespan_min * 60.0) * 100.0
                )
            else:
                machine_utilization[mid] = 0.0

        # On-time percentage
        on_time_count = sum(
            1 for e in entries
            if e.end_time <= self._orders[e.order_id].due_date
        )
        on_time_pct = (on_time_count / len(entries) * 100.0) if entries else 100.0

        return ScheduleResult(
            entries=entries,
            unscheduled_orders=unscheduled,
            total_makespan_min=total_makespan_min,
            machine_utilization=machine_utilization,
            on_time_pct=on_time_pct,
        )

    # -- queries -----------------------------------------------------

    def get_late_orders(self, current_time: float) -> List[ProductionOrder]:
        """Return orders whose due date has passed and are not completed."""
        with self._lock:
            return [
                o for o in self._orders.values()
                if o.due_date < current_time and o.status not in ('completed',)
            ]

    def get_schedule_for_machine(self, machine_id: str) -> List[ScheduleEntry]:
        """Return all schedule entries assigned to a specific machine."""
        with self._lock:
            return [
                e for e in self._schedule_entries
                if e.machine_id == machine_id
            ]

    def get_production_summary(self) -> Dict[str, Any]:
        """Return a summary of all production orders.

        Returns:
            {
                'total_orders': int,
                'by_status': {status: count, ...},
                'avg_lead_time_min': float,
            }
        """
        with self._lock:
            orders = list(self._orders.values())

        total = len(orders)
        by_status: Dict[str, int] = {}
        for order in orders:
            by_status[order.status] = by_status.get(order.status, 0) + 1

        # Average lead time: estimated total production time per order
        if orders:
            lead_times = [
                o.estimated_time_per_part_min * o.quantity for o in orders
            ]
            avg_lead_time = sum(lead_times) / len(lead_times)
        else:
            avg_lead_time = 0.0

        return {
            'total_orders': total,
            'by_status': by_status,
            'avg_lead_time_min': avg_lead_time,
        }


# ------------------------------------------------------------------
# Inventory tracking
# ------------------------------------------------------------------

@dataclass
class InventoryItem:
    """A single inventory item with stock levels and reorder parameters."""

    item_id: str
    name: str
    category: str  # 'raw_material' | 'wip' | 'finished_good' | 'consumable' | 'tooling'
    quantity: float
    unit: str
    location: str
    reorder_point: float
    reorder_quantity: float
    unit_cost: float
    last_updated: float = 0.0


@dataclass
class InventoryTransaction:
    """A single inventory transaction record."""

    transaction_id: str
    item_id: str
    transaction_type: str  # 'receive' | 'issue' | 'adjust' | 'scrap' | 'return'
    quantity: float
    timestamp: float
    reference: str
    operator: str


class InventoryTracker:
    """Tracks raw material and finished goods inventory levels.

    Provides methods for recording transactions, computing reorder alerts,
    calculating inventory value, and forecasting stockouts based on
    historical consumption rates.
    """

    def __init__(self) -> None:
        self._items: Dict[str, InventoryItem] = {}
        self._transactions: List[InventoryTransaction] = []
        self._lock = threading.Lock()

    # -- Item CRUD -------------------------------------------------------

    def add_item(self, item: InventoryItem) -> None:
        """Register a new inventory item."""
        with self._lock:
            self._items[item.item_id] = item

    def get_item(self, item_id: str) -> Optional[InventoryItem]:
        """Return an inventory item by its id, or ``None``."""
        with self._lock:
            return self._items.get(item_id)

    def get_all_items(self) -> List[InventoryItem]:
        """Return every registered inventory item."""
        with self._lock:
            return list(self._items.values())

    # -- Transactions ----------------------------------------------------

    def record_transaction(self, transaction: InventoryTransaction) -> None:
        """Record an inventory transaction and update the item quantity.

        Transaction types that *increase* stock: ``receive``, ``return``.
        Transaction types that *decrease* stock: ``issue``, ``scrap``.
        ``adjust`` sets the quantity to exactly ``transaction.quantity``.
        """
        with self._lock:
            item = self._items.get(transaction.item_id)
            if item is None:
                raise KeyError(
                    f"Unknown item_id: {transaction.item_id}"
                )

            if transaction.transaction_type in ('receive', 'return'):
                item.quantity += transaction.quantity
            elif transaction.transaction_type in ('issue', 'scrap'):
                item.quantity -= transaction.quantity
            elif transaction.transaction_type == 'adjust':
                item.quantity = transaction.quantity
            else:
                raise ValueError(
                    f"Unknown transaction_type: {transaction.transaction_type}"
                )

            item.last_updated = transaction.timestamp
            self._transactions.append(transaction)

    # -- Queries ---------------------------------------------------------

    def get_reorder_alerts(self) -> List[InventoryItem]:
        """Return items whose quantity is at or below their reorder point."""
        with self._lock:
            return [
                item for item in self._items.values()
                if item.quantity <= item.reorder_point
            ]

    def get_inventory_value(self) -> Dict[str, float]:
        """Return total inventory value grouped by category."""
        with self._lock:
            value_by_category: Dict[str, float] = {}
            for item in self._items.values():
                value_by_category[item.category] = (
                    value_by_category.get(item.category, 0.0)
                    + item.quantity * item.unit_cost
                )
            return value_by_category

    def get_transaction_history(
        self, item_id: str
    ) -> List[InventoryTransaction]:
        """Return all transactions for *item_id*, ordered by timestamp."""
        with self._lock:
            return sorted(
                [t for t in self._transactions if t.item_id == item_id],
                key=lambda t: t.timestamp,
            )

    # -- Analytics -------------------------------------------------------

    def get_consumption_rate(self, item_id: str, days: float) -> float:
        """Return the average daily consumption over the last *days* days.

        Only ``issue`` and ``scrap`` transactions are counted as
        consumption.  Returns ``0.0`` when there are no qualifying
        transactions in the window.
        """
        now = time.time()
        window_start = now - days * 86400.0
        with self._lock:
            total_consumed = sum(
                t.quantity
                for t in self._transactions
                if t.item_id == item_id
                and t.transaction_type in ('issue', 'scrap')
                and t.timestamp >= window_start
            )
        if days <= 0:
            return 0.0
        return total_consumed / days

    def forecast_stockout(self, item_id: str, lookback_days: float = 30.0) -> Optional[float]:
        """Predict the number of days until *item_id* reaches zero stock.

        Uses :meth:`get_consumption_rate` over *lookback_days* to
        extrapolate.  Returns ``None`` when there is no consumption
        (infinite runway) or the item does not exist.
        """
        item = self.get_item(item_id)
        if item is None:
            return None
        rate = self.get_consumption_rate(item_id, lookback_days)
        if rate <= 0:
            return None
        return item.quantity / rate


# ------------------------------------------------------------------
# SPC Run Rules Engine (Nelson Rules)
# ------------------------------------------------------------------

@dataclass
class RunRuleViolation:
    """A single violation detected by the SPC Run Rules Engine."""

    rule_number: int
    rule_name: str
    start_index: int
    end_index: int
    severity: str  # 'warning' | 'action'
    description: str
    values: List[float] = field(default_factory=list)


@dataclass
class SPCAnalysis:
    """Result of an SPC analysis over a set of measurement data."""

    mean: float
    std: float
    ucl: float
    lcl: float
    violations: List[RunRuleViolation] = field(default_factory=list)
    in_control: bool = True
    total_points: int = 0


class SPCRunRulesEngine:
    """Evaluates Statistical Process Control run rules (Nelson rules).

    The eight Nelson rules are standard tests applied to control-chart
    data to detect non-random patterns that indicate special-cause
    variation.  Each rule returns a list of :class:`RunRuleViolation`
    instances.
    """

    _RULE_DESCRIPTIONS: Dict[int, str] = {
        1: 'One point beyond 3 sigma from the centre line',
        2: 'Nine consecutive points on the same side of the centre line',
        3: 'Six consecutive points steadily increasing or decreasing',
        4: 'Fourteen consecutive points alternating up and down',
        5: 'Two out of three consecutive points beyond 2 sigma (same side)',
        6: 'Four out of five consecutive points beyond 1 sigma (same side)',
        7: 'Fifteen consecutive points within 1 sigma of the centre line (stratification)',
        8: 'Eight consecutive points beyond 1 sigma on both sides (mixture)',
    }

    # ----- public API --------------------------------------------------

    def analyze(
        self,
        data: List[float],
        target_mean: Optional[float] = None,
        target_std: Optional[float] = None,
    ) -> SPCAnalysis:
        """Run all eight Nelson rules and return an :class:`SPCAnalysis`.

        Parameters
        ----------
        data:
            Sequence of measured values (at least 1 element).
        target_mean:
            Known process mean.  If *None* the sample mean is used.
        target_std:
            Known process standard deviation.  If *None* the sample std
            is used (ddof=1 when len > 1).
        """
        n = len(data)
        if n == 0:
            return SPCAnalysis(
                mean=0.0, std=0.0, ucl=0.0, lcl=0.0,
                violations=[], in_control=True, total_points=0,
            )

        mean = target_mean if target_mean is not None else sum(data) / n
        if target_std is not None:
            std = target_std
        elif n > 1:
            variance = sum((x - mean) ** 2 for x in data) / (n - 1)
            std = variance ** 0.5
        else:
            std = 0.0

        ucl = mean + 3.0 * std
        lcl = mean - 3.0 * std

        violations: List[RunRuleViolation] = []
        for rule_num in range(1, 9):
            violations.extend(self.check_rule(rule_num, data, mean, std))

        in_control = len(violations) == 0

        return SPCAnalysis(
            mean=mean,
            std=std,
            ucl=ucl,
            lcl=lcl,
            violations=violations,
            in_control=in_control,
            total_points=n,
        )

    def check_rule(
        self,
        rule_number: int,
        data: List[float],
        mean: float,
        std: float,
    ) -> List[RunRuleViolation]:
        """Evaluate a single Nelson rule and return any violations."""
        handler = {
            1: self._rule_1,
            2: self._rule_2,
            3: self._rule_3,
            4: self._rule_4,
            5: self._rule_5,
            6: self._rule_6,
            7: self._rule_7,
            8: self._rule_8,
        }.get(rule_number)
        if handler is None:
            raise ValueError(f'Unknown rule number: {rule_number}')
        return handler(data, mean, std)

    def get_rule_description(self, rule_number: int) -> str:
        """Return a human-readable description for *rule_number*."""
        desc = self._RULE_DESCRIPTIONS.get(rule_number)
        if desc is None:
            raise ValueError(f'Unknown rule number: {rule_number}')
        return desc

    # ----- individual rules -------------------------------------------

    def _rule_1(self, data: List[float], mean: float, std: float) -> List[RunRuleViolation]:
        """Rule 1: Any single point beyond 3 sigma."""
        violations: List[RunRuleViolation] = []
        if std == 0:
            return violations
        for i, x in enumerate(data):
            if abs(x - mean) > 3.0 * std:
                violations.append(RunRuleViolation(
                    rule_number=1,
                    rule_name='Beyond 3 sigma',
                    start_index=i,
                    end_index=i,
                    severity='action',
                    description=self._RULE_DESCRIPTIONS[1],
                    values=[x],
                ))
        return violations

    def _rule_2(self, data: List[float], mean: float, std: float) -> List[RunRuleViolation]:
        """Rule 2: Nine consecutive points on the same side of the mean."""
        violations: List[RunRuleViolation] = []
        n = len(data)
        run_length = 9
        if n < run_length:
            return violations
        for i in range(n - run_length + 1):
            window = data[i:i + run_length]
            above = all(x > mean for x in window)
            below = all(x < mean for x in window)
            if above or below:
                violations.append(RunRuleViolation(
                    rule_number=2,
                    rule_name='9 points same side',
                    start_index=i,
                    end_index=i + run_length - 1,
                    severity='action',
                    description=self._RULE_DESCRIPTIONS[2],
                    values=list(window),
                ))
        return violations

    def _rule_3(self, data: List[float], mean: float, std: float) -> List[RunRuleViolation]:
        """Rule 3: Six consecutive points steadily increasing or decreasing."""
        violations: List[RunRuleViolation] = []
        n = len(data)
        run_length = 6
        if n < run_length:
            return violations
        for i in range(n - run_length + 1):
            window = data[i:i + run_length]
            increasing = all(window[j + 1] > window[j] for j in range(run_length - 1))
            decreasing = all(window[j + 1] < window[j] for j in range(run_length - 1))
            if increasing or decreasing:
                violations.append(RunRuleViolation(
                    rule_number=3,
                    rule_name='6 points trending',
                    start_index=i,
                    end_index=i + run_length - 1,
                    severity='warning',
                    description=self._RULE_DESCRIPTIONS[3],
                    values=list(window),
                ))
        return violations

    def _rule_4(self, data: List[float], mean: float, std: float) -> List[RunRuleViolation]:
        """Rule 4: Fourteen consecutive points alternating up and down."""
        violations: List[RunRuleViolation] = []
        n = len(data)
        run_length = 14
        if n < run_length:
            return violations
        for i in range(n - run_length + 1):
            window = data[i:i + run_length]
            alternating = True
            for j in range(run_length - 2):
                d1 = window[j + 1] - window[j]
                d2 = window[j + 2] - window[j + 1]
                if d1 == 0 or d2 == 0 or (d1 > 0) == (d2 > 0):
                    alternating = False
                    break
            if alternating:
                violations.append(RunRuleViolation(
                    rule_number=4,
                    rule_name='14 points alternating',
                    start_index=i,
                    end_index=i + run_length - 1,
                    severity='warning',
                    description=self._RULE_DESCRIPTIONS[4],
                    values=list(window),
                ))
        return violations

    def _rule_5(self, data: List[float], mean: float, std: float) -> List[RunRuleViolation]:
        """Rule 5: Two of three consecutive points beyond 2 sigma (same side)."""
        violations: List[RunRuleViolation] = []
        n = len(data)
        if n < 3 or std == 0:
            return violations
        two_sigma = 2.0 * std
        for i in range(n - 2):
            window = data[i:i + 3]
            above_count = sum(1 for x in window if x - mean > two_sigma)
            below_count = sum(1 for x in window if mean - x > two_sigma)
            if above_count >= 2 or below_count >= 2:
                violations.append(RunRuleViolation(
                    rule_number=5,
                    rule_name='2 of 3 beyond 2 sigma',
                    start_index=i,
                    end_index=i + 2,
                    severity='warning',
                    description=self._RULE_DESCRIPTIONS[5],
                    values=list(window),
                ))
        return violations

    def _rule_6(self, data: List[float], mean: float, std: float) -> List[RunRuleViolation]:
        """Rule 6: Four of five consecutive points beyond 1 sigma (same side)."""
        violations: List[RunRuleViolation] = []
        n = len(data)
        if n < 5 or std == 0:
            return violations
        one_sigma = 1.0 * std
        for i in range(n - 4):
            window = data[i:i + 5]
            above_count = sum(1 for x in window if x - mean > one_sigma)
            below_count = sum(1 for x in window if mean - x > one_sigma)
            if above_count >= 4 or below_count >= 4:
                violations.append(RunRuleViolation(
                    rule_number=6,
                    rule_name='4 of 5 beyond 1 sigma',
                    start_index=i,
                    end_index=i + 4,
                    severity='warning',
                    description=self._RULE_DESCRIPTIONS[6],
                    values=list(window),
                ))
        return violations

    def _rule_7(self, data: List[float], mean: float, std: float) -> List[RunRuleViolation]:
        """Rule 7: Fifteen consecutive points within 1 sigma (stratification)."""
        violations: List[RunRuleViolation] = []
        n = len(data)
        run_length = 15
        if n < run_length or std == 0:
            return violations
        one_sigma = 1.0 * std
        for i in range(n - run_length + 1):
            window = data[i:i + run_length]
            if all(abs(x - mean) <= one_sigma for x in window):
                violations.append(RunRuleViolation(
                    rule_number=7,
                    rule_name='15 points within 1 sigma',
                    start_index=i,
                    end_index=i + run_length - 1,
                    severity='warning',
                    description=self._RULE_DESCRIPTIONS[7],
                    values=list(window),
                ))
        return violations

    def _rule_8(self, data: List[float], mean: float, std: float) -> List[RunRuleViolation]:
        """Rule 8: Eight consecutive points beyond 1 sigma on both sides (mixture)."""
        violations: List[RunRuleViolation] = []
        n = len(data)
        run_length = 8
        if n < run_length or std == 0:
            return violations
        one_sigma = 1.0 * std
        for i in range(n - run_length + 1):
            window = data[i:i + run_length]
            all_beyond = all(abs(x - mean) > one_sigma for x in window)
            if not all_beyond:
                continue
            has_above = any(x > mean for x in window)
            has_below = any(x < mean for x in window)
            if has_above and has_below:
                violations.append(RunRuleViolation(
                    rule_number=8,
                    rule_name='8 points beyond 1 sigma (mixture)',
                    start_index=i,
                    end_index=i + run_length - 1,
                    severity='action',
                    description=self._RULE_DESCRIPTIONS[8],
                    values=list(window),
                ))
        return violations


# ------------------------------------------------------------------
# Scrap / Reject Tracking
# ------------------------------------------------------------------


@dataclass
class ScrapEvent:
    """A single scrap or reject event."""

    event_id: str
    part_id: str
    job_id: str
    machine_id: str
    reason_code: str
    reason_description: str
    timestamp: float
    quantity: int
    material_cost: float
    labor_cost: float
    operation: str
    is_reworkable: bool


@dataclass
class ScrapSummary:
    """Aggregated scrap summary over a time window."""

    total_scrap_count: int
    total_cost: float
    scrap_rate_pct: float
    top_reasons: List[Tuple[str, int, float]]
    by_machine: Dict[str, int]
    by_operation: Dict[str, int]
    rework_pct: float


class ScrapTracker:
    """Tracks scrap / reject parts with reason codes and cost analysis.

    Provides Pareto analysis, trend reporting, cost-of-poor-quality
    calculation, and rework candidate identification.
    """

    def __init__(self) -> None:
        self._events: List[ScrapEvent] = []
        self._lock = threading.Lock()

    # -- recording ---------------------------------------------------

    def record_scrap(self, event: ScrapEvent) -> None:
        """Record a scrap event."""
        with self._lock:
            self._events.append(event)

    # -- querying ----------------------------------------------------

    def get_summary(
        self,
        start_time: float,
        end_time: float,
        total_parts_produced: int,
    ) -> ScrapSummary:
        """Generate a :class:`ScrapSummary` for the given time window."""
        with self._lock:
            window = [
                e for e in self._events
                if start_time <= e.timestamp <= end_time
            ]

        total_scrap = sum(e.quantity for e in window)
        total_cost = sum(e.material_cost + e.labor_cost for e in window)

        if total_parts_produced > 0:
            scrap_rate = (total_scrap / total_parts_produced) * 100.0
        else:
            scrap_rate = 0.0

        # Top reasons by count
        reason_counts: Dict[str, int] = {}
        reason_costs: Dict[str, float] = {}
        for e in window:
            reason_counts[e.reason_code] = reason_counts.get(e.reason_code, 0) + e.quantity
            reason_costs[e.reason_code] = reason_costs.get(e.reason_code, 0.0) + e.material_cost + e.labor_cost
        top_reasons = sorted(
            [(rc, reason_counts[rc], reason_costs[rc]) for rc in reason_counts],
            key=lambda t: t[1],
            reverse=True,
        )

        by_machine: Dict[str, int] = {}
        for e in window:
            by_machine[e.machine_id] = by_machine.get(e.machine_id, 0) + e.quantity

        by_operation: Dict[str, int] = {}
        for e in window:
            by_operation[e.operation] = by_operation.get(e.operation, 0) + e.quantity

        rework_count = sum(e.quantity for e in window if e.is_reworkable)
        rework_pct = (rework_count / total_scrap * 100.0) if total_scrap > 0 else 0.0

        return ScrapSummary(
            total_scrap_count=total_scrap,
            total_cost=total_cost,
            scrap_rate_pct=scrap_rate,
            top_reasons=top_reasons,
            by_machine=by_machine,
            by_operation=by_operation,
            rework_pct=rework_pct,
        )

    def get_scrap_by_reason(self, reason_code: str) -> List[ScrapEvent]:
        """Return all scrap events matching *reason_code*."""
        with self._lock:
            return [e for e in self._events if e.reason_code == reason_code]

    def get_pareto_analysis(self, top_n: int = 5) -> List[Tuple[str, float, float]]:
        """Pareto analysis of scrap reasons by cost.

        Returns a list of ``(reason_code, cost, cumulative_pct)`` tuples
        sorted by descending cost, limited to *top_n* entries.
        """
        with self._lock:
            events = list(self._events)

        reason_costs: Dict[str, float] = {}
        for e in events:
            reason_costs[e.reason_code] = reason_costs.get(e.reason_code, 0.0) + e.material_cost + e.labor_cost

        sorted_reasons = sorted(reason_costs.items(), key=lambda t: t[1], reverse=True)
        total_cost = sum(c for _, c in sorted_reasons) if sorted_reasons else 1.0

        result: List[Tuple[str, float, float]] = []
        cumulative = 0.0
        for reason, cost in sorted_reasons[:top_n]:
            cumulative += cost
            result.append((reason, cost, (cumulative / total_cost) * 100.0))
        return result

    def get_scrap_trend(
        self,
        periods: int,
        period_duration_sec: float,
    ) -> List[Tuple[float, int]]:
        """Return scrap count per period.

        *periods* buckets are created, each spanning *period_duration_sec*
        seconds, ending at the latest event timestamp.  Returns a list
        of ``(period_start, scrap_count)`` tuples.
        """
        with self._lock:
            events = list(self._events)

        if not events:
            return []

        latest = max(e.timestamp for e in events)
        overall_start = latest - periods * period_duration_sec

        buckets: List[Tuple[float, int]] = []
        for i in range(periods):
            bucket_start = overall_start + i * period_duration_sec
            bucket_end = bucket_start + period_duration_sec
            is_last = (i == periods - 1)
            count = sum(
                e.quantity for e in events
                if bucket_start <= e.timestamp < bucket_end
                or (is_last and e.timestamp == bucket_end)
            )
            buckets.append((bucket_start, count))
        return buckets

    def get_cost_of_poor_quality(
        self,
        start_time: float,
        end_time: float,
    ) -> Dict[str, float]:
        """Total cost of poor quality (COPQ) in the given window.

        Returns a dict with ``scrap_cost``, ``rework_cost``, and
        ``total_copq`` keys.
        """
        with self._lock:
            window = [
                e for e in self._events
                if start_time <= e.timestamp <= end_time
            ]

        scrap_cost = sum(
            e.material_cost + e.labor_cost for e in window if not e.is_reworkable
        )
        rework_cost = sum(
            e.labor_cost for e in window if e.is_reworkable
        )
        return {
            'scrap_cost': scrap_cost,
            'rework_cost': rework_cost,
            'total_copq': scrap_cost + rework_cost,
        }

    def get_rework_candidates(self) -> List[ScrapEvent]:
        """Return all reworkable scrap events."""
        with self._lock:
            return [e for e in self._events if e.is_reworkable]


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
