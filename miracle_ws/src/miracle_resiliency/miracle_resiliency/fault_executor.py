"""
Fault Executor — Real fault injection for chaos engineering.

Implements actual fault injection mechanisms:
  - Network delay via tc/netem
  - Node kill via ROS2 lifecycle transitions
  - CPU/memory stress via stress-ng
  - Message drop via topic relay with configurable drop rate

Safety: excluded nodes list, max 60s duration, auto-cleanup.
"""

import asyncio
import os
import signal
import subprocess
import time
import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Set


class FaultType(Enum):
    """Types of injectable faults."""
    NETWORK_DELAY = 'network_delay'
    NODE_KILL = 'node_kill'
    CPU_STRESS = 'cpu_stress'
    MEMORY_STRESS = 'memory_stress'
    MESSAGE_DROP = 'message_drop'


@dataclass
class FaultSpec:
    """Specification for a fault to inject."""
    fault_type: FaultType
    target: str  # node name, interface, or topic
    duration_sec: float = 30.0
    parameters: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ActiveFault:
    """An actively injected fault."""
    fault_id: str
    spec: FaultSpec
    start_time: float
    cleanup_cmd: Optional[List[str]] = None
    process: Optional[subprocess.Popen] = None
    cleaned_up: bool = False


class FaultExecutor:
    """Executes real fault injection with safety guardrails.

    Args:
        excluded_nodes: Node names that must never be targeted.
        max_duration_sec: Maximum fault duration (hard limit).
        dry_run: Log actions without executing (for testing).
    """

    DEFAULT_EXCLUDED = frozenset({
        'safety_monitor',
        'e_stop_controller',
        'alarm_manager',
        'audit_logger',
        'intrusion_detector',
    })

    MAX_DURATION = 60.0

    def __init__(
        self,
        excluded_nodes: Optional[Set[str]] = None,
        max_duration_sec: float = 60.0,
        dry_run: bool = False,
    ) -> None:
        self._excluded = excluded_nodes or set(self.DEFAULT_EXCLUDED)
        self._max_duration = min(max_duration_sec, self.MAX_DURATION)
        self._dry_run = dry_run
        self._active_faults: Dict[str, ActiveFault] = {}
        self._lock = threading.Lock()
        self._cleanup_timer: Optional[threading.Timer] = None

    def inject(self, fault_id: str, spec: FaultSpec) -> bool:
        """Inject a fault. Returns True if successfully injected.

        Args:
            fault_id: Unique identifier for this fault instance.
            spec: Fault specification.

        Returns:
            True if fault was injected, False if rejected.
        """
        # Safety checks
        if spec.target in self._excluded:
            return False

        if spec.duration_sec > self._max_duration:
            spec.duration_sec = self._max_duration

        if spec.duration_sec <= 0:
            return False

        with self._lock:
            if fault_id in self._active_faults:
                return False  # Already active

        active = ActiveFault(
            fault_id=fault_id,
            spec=spec,
            start_time=time.time(),
        )

        success = False
        if spec.fault_type == FaultType.NETWORK_DELAY:
            success = self._inject_network_delay(active)
        elif spec.fault_type == FaultType.NODE_KILL:
            success = self._inject_node_kill(active)
        elif spec.fault_type == FaultType.CPU_STRESS:
            success = self._inject_cpu_stress(active)
        elif spec.fault_type == FaultType.MEMORY_STRESS:
            success = self._inject_memory_stress(active)
        elif spec.fault_type == FaultType.MESSAGE_DROP:
            success = self._inject_message_drop(active)

        if success:
            with self._lock:
                self._active_faults[fault_id] = active
            # Schedule auto-cleanup
            timer = threading.Timer(spec.duration_sec, self.cleanup, args=[fault_id])
            timer.daemon = True
            timer.start()

        return success

    def cleanup(self, fault_id: str) -> bool:
        """Clean up an injected fault.

        Returns:
            True if fault was cleaned up successfully.
        """
        with self._lock:
            active = self._active_faults.get(fault_id)
            if active is None or active.cleaned_up:
                return False

        success = True

        # Kill subprocess if running
        if active.process and active.process.poll() is None:
            try:
                active.process.terminate()
                active.process.wait(timeout=5)
            except (subprocess.TimeoutExpired, OSError):
                try:
                    active.process.kill()
                except OSError:
                    pass

        # Run cleanup command
        if active.cleanup_cmd and not self._dry_run:
            try:
                subprocess.run(
                    active.cleanup_cmd,
                    capture_output=True, timeout=10,
                )
            except (subprocess.TimeoutExpired, OSError):
                success = False

        active.cleaned_up = True

        with self._lock:
            self._active_faults.pop(fault_id, None)

        return success

    def cleanup_all(self) -> int:
        """Clean up all active faults. Returns count of cleaned faults."""
        with self._lock:
            fault_ids = list(self._active_faults.keys())

        count = 0
        for fid in fault_ids:
            if self.cleanup(fid):
                count += 1
        return count

    def _inject_network_delay(self, active: ActiveFault) -> bool:
        """Inject network delay via tc/netem."""
        delay_ms = active.spec.parameters.get('delay_ms', 100)
        interface = active.spec.target or 'eth0'

        cmd = [
            'tc', 'qdisc', 'add', 'dev', interface,
            'root', 'netem', 'delay', f'{delay_ms}ms',
        ]
        cleanup_cmd = [
            'tc', 'qdisc', 'del', 'dev', interface, 'root',
        ]

        active.cleanup_cmd = cleanup_cmd

        if self._dry_run:
            return True

        try:
            result = subprocess.run(cmd, capture_output=True, timeout=10)
            return result.returncode == 0
        except (subprocess.TimeoutExpired, OSError):
            return False

    def _inject_node_kill(self, active: ActiveFault) -> bool:
        """Kill a ROS2 node via lifecycle shutdown transition."""
        node_name = active.spec.target

        if self._dry_run:
            return True

        try:
            # Try lifecycle shutdown first
            result = subprocess.run(
                ['ros2', 'lifecycle', 'set', node_name, 'shutdown'],
                capture_output=True, timeout=10,
            )
            return result.returncode == 0
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return False

    def _inject_cpu_stress(self, active: ActiveFault) -> bool:
        """Inject CPU stress via stress-ng."""
        cpu_pct = active.spec.parameters.get('cpu_percent', 80)
        workers = active.spec.parameters.get('workers', 2)
        duration = int(active.spec.duration_sec)

        cmd = [
            'stress-ng', '--cpu', str(workers),
            '--cpu-load', str(cpu_pct),
            '--timeout', f'{duration}s',
        ]

        if self._dry_run:
            return True

        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            active.process = proc
            return True
        except (OSError, FileNotFoundError):
            return False

    def _inject_memory_stress(self, active: ActiveFault) -> bool:
        """Inject memory pressure via stress-ng."""
        mem_pct = active.spec.parameters.get('memory_percent', 50)
        duration = int(active.spec.duration_sec)

        cmd = [
            'stress-ng', '--vm', '1',
            '--vm-bytes', f'{mem_pct}%',
            '--timeout', f'{duration}s',
        ]

        if self._dry_run:
            return True

        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            active.process = proc
            return True
        except (OSError, FileNotFoundError):
            return False

    def _inject_message_drop(self, active: ActiveFault) -> bool:
        """Simulate message drops by recording drop intent.

        Note: Actual message dropping requires a relay node. This records
        the drop configuration for the chaos_injector relay to apply.
        """
        drop_rate = active.spec.parameters.get('drop_rate', 0.5)
        topic = active.spec.target

        # Store drop config for relay node to read
        active.spec.parameters['_effective_drop_rate'] = max(0.0, min(1.0, drop_rate))
        active.spec.parameters['_topic'] = topic
        return True

    @property
    def active_fault_count(self) -> int:
        with self._lock:
            return len(self._active_faults)

    @property
    def active_faults(self) -> Dict[str, FaultSpec]:
        with self._lock:
            return {fid: af.spec for fid, af in self._active_faults.items()}
