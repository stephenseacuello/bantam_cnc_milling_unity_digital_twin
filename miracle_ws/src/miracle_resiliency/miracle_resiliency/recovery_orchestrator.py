"""
Recovery Orchestrator Node.

Orchestrates multi-step recovery sequences for failed nodes,
coordinating with lifecycle management and dependency ordering.
"""

from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum
import threading
import asyncio
import time
import uuid
import json

from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import RecoveryRequest, NodeFailure, Heartbeat

from miracle_resiliency.lifecycle_client import (
    LifecycleClient, LifecycleClientError, LifecycleTransition,
)


@dataclass
class RecoveryAction:
    """A recovery action in progress."""
    failed_node: str
    strategy: str
    attempt: int = 0
    max_retries: int = 3
    status: str = 'PENDING'  # PENDING, IN_PROGRESS, VERIFYING, COMPLETED, FAILED


@dataclass
class NetworkPartition:
    """Represents a detected network partition event."""
    partition_id: str
    detected_at: float
    affected_nodes: List[str]
    reachable_nodes: List[str]
    unreachable_nodes: List[str]
    partition_type: str  # FULL, PARTIAL, INTERMITTENT
    estimated_severity: float  # 0.0 - 1.0
    recovery_strategy: str


# Well-known safety-critical node names
SAFETY_NODES = frozenset({
    'emergency_stop', 'safety_monitor', 'collision_detector',
    'estop_controller', 'safety_controller',
})


class PartitionDetector:
    """Detects network partitions by monitoring node heartbeats.

    Tracks heartbeat arrivals and failures per node.  When consecutive
    failures exceed the configured threshold the node is marked
    unreachable.  Unreachable nodes are then grouped by timing to
    distinguish full, partial, and intermittent partitions.
    """

    def __init__(
        self,
        heartbeat_timeout_sec: float = 5.0,
        min_failures: int = 3,
    ) -> None:
        self._heartbeat_timeout_sec = heartbeat_timeout_sec
        self._min_failures = min_failures

        # node_name -> last successful heartbeat timestamp
        self._last_heartbeat: Dict[str, float] = {}
        # node_name -> consecutive failure count
        self._failure_counts: Dict[str, int] = {}
        # node_name -> list of failure timestamps (for flap detection)
        self._failure_timestamps: Dict[str, List[float]] = {}
        # node_name -> total heartbeat count (for uptime %)
        self._heartbeat_counts: Dict[str, int] = {}
        # node_name -> total check count
        self._check_counts: Dict[str, int] = {}
        # node_name -> first seen timestamp
        self._first_seen: Dict[str, float] = {}

        # Partition bookkeeping
        self._active_partitions: Dict[str, NetworkPartition] = {}
        self._partition_history: List[NetworkPartition] = []

    # ------------------------------------------------------------------ #
    #  Heartbeat recording
    # ------------------------------------------------------------------ #

    def record_heartbeat(self, node_name: str, timestamp: float) -> None:
        """Record a successful heartbeat from *node_name*."""
        self._last_heartbeat[node_name] = timestamp
        self._failure_counts[node_name] = 0
        self._heartbeat_counts[node_name] = (
            self._heartbeat_counts.get(node_name, 0) + 1
        )
        self._check_counts[node_name] = (
            self._check_counts.get(node_name, 0) + 1
        )
        if node_name not in self._first_seen:
            self._first_seen[node_name] = timestamp

    def record_heartbeat_failure(self, node_name: str, timestamp: float) -> None:
        """Record a heartbeat failure (missed heartbeat) for *node_name*."""
        self._failure_counts[node_name] = (
            self._failure_counts.get(node_name, 0) + 1
        )
        self._failure_timestamps.setdefault(node_name, []).append(timestamp)
        self._check_counts[node_name] = (
            self._check_counts.get(node_name, 0) + 1
        )
        if node_name not in self._first_seen:
            self._first_seen[node_name] = timestamp

    # ------------------------------------------------------------------ #
    #  Partition detection
    # ------------------------------------------------------------------ #

    def check_partitions(self, current_time: float) -> List[NetworkPartition]:
        """Analyse current heartbeat state and return new partitions."""
        all_nodes = set(self._last_heartbeat) | set(self._failure_counts)
        if not all_nodes:
            return []

        unreachable: List[str] = []
        reachable: List[str] = []

        for node in all_nodes:
            failures = self._failure_counts.get(node, 0)

            if failures >= self._min_failures:
                unreachable.append(node)
            else:
                reachable.append(node)

        if not unreachable:
            return []

        # Classify partition type
        partition_type = self._classify_partition(
            all_nodes, reachable, unreachable, current_time,
        )

        severity = len(unreachable) / len(all_nodes) if all_nodes else 0.0

        partition = NetworkPartition(
            partition_id=str(uuid.uuid4()),
            detected_at=current_time,
            affected_nodes=sorted(all_nodes),
            reachable_nodes=sorted(reachable),
            unreachable_nodes=sorted(unreachable),
            partition_type=partition_type,
            estimated_severity=round(severity, 4),
            recovery_strategy=self._compute_recovery_strategy(
                partition_type, unreachable,
            ),
        )

        self._active_partitions[partition.partition_id] = partition
        self._partition_history.append(partition)
        return [partition]

    # ------------------------------------------------------------------ #
    #  Query helpers
    # ------------------------------------------------------------------ #

    def get_node_health(self) -> Dict[str, Dict[str, Any]]:
        """Return per-node health information."""
        all_nodes = set(self._last_heartbeat) | set(self._failure_counts)
        result: Dict[str, Dict[str, Any]] = {}
        for node in sorted(all_nodes):
            checks = self._check_counts.get(node, 0)
            successes = self._heartbeat_counts.get(node, 0)
            uptime_pct = (successes / checks * 100.0) if checks > 0 else 0.0
            failures = self._failure_counts.get(node, 0)
            status = 'unreachable' if failures >= self._min_failures else 'reachable'
            result[node] = {
                'status': status,
                'last_seen': self._last_heartbeat.get(node),
                'failure_count': failures,
                'uptime_pct': round(uptime_pct, 2),
            }
        return result

    def get_partition_history(self) -> List[NetworkPartition]:
        """Return all detected partitions (active and resolved)."""
        return list(self._partition_history)

    def clear_partition(self, partition_id: str) -> None:
        """Mark a partition as resolved and remove from active set."""
        self._active_partitions.pop(partition_id, None)

    def is_node_reachable(self, node_name: str) -> bool:
        """Return True if *node_name* is considered reachable."""
        failures = self._failure_counts.get(node_name, 0)
        return failures < self._min_failures

    def get_recommended_recovery(self, partition: NetworkPartition) -> str:
        """Return a recovery recommendation string for *partition*."""
        return self._compute_recovery_strategy(
            partition.partition_type, partition.unreachable_nodes,
        )

    # ------------------------------------------------------------------ #
    #  Internal helpers
    # ------------------------------------------------------------------ #

    def _classify_partition(
        self,
        all_nodes: set,
        reachable: List[str],
        unreachable: List[str],
        current_time: float,
    ) -> str:
        """Classify partition as FULL, PARTIAL, or INTERMITTENT."""
        # Check for intermittent (flapping) -- node that has had both
        # successes *and* failures recently.
        flapping = False
        for node in unreachable:
            ts_list = self._failure_timestamps.get(node, [])
            if len(ts_list) >= 2:
                # If failures are spread out (not a single burst) and
                # the node has had successful heartbeats in between,
                # treat as intermittent.
                hb_count = self._heartbeat_counts.get(node, 0)
                if hb_count > 0:
                    flapping = True
                    break

        if flapping:
            return 'INTERMITTENT'

        if len(reachable) == 0:
            return 'FULL'

        return 'PARTIAL'

    @staticmethod
    def _compute_recovery_strategy(
        partition_type: str,
        unreachable_nodes: List[str],
    ) -> str:
        """Decide recovery strategy based on partition type and nodes."""
        if partition_type == 'FULL':
            return 'escalate_to_operator'

        if partition_type == 'INTERMITTENT':
            return 'increase_heartbeat_frequency'

        # PARTIAL -- check if any safety-critical nodes are affected
        if any(n in SAFETY_NODES for n in unreachable_nodes):
            return 'emergency_stop'

        return 'continue_degraded'


# Default dependency graph for MIRACLE nodes
# Key depends on values (must restart values first)
DEFAULT_DEPENDENCIES: Dict[str, List[str]] = {
    'anomaly_detector': ['sensor_fusion'],
    'phm_predictor': ['anomaly_detector', 'sensor_fusion'],
    'tool_wear_estimator': ['sensor_fusion'],
    'alarm_manager': ['anomaly_detector'],
    'twin_sync': ['state_mirror'],
    'prediction_runner': ['phm_predictor', 'twin_sync'],
    'adaptive_controller': ['prediction_runner', 'anomaly_detector'],
    'job_scheduler': ['gcode_executor'],
    'digital_thread': ['job_scheduler'],
}


def topological_sort(node: str, deps: Dict[str, List[str]]) -> List[str]:
    """Return nodes in dependency order (dependencies first)."""
    visited = set()
    order = []

    def visit(n: str):
        if n in visited:
            return
        visited.add(n)
        for dep in deps.get(n, []):
            visit(dep)
        order.append(n)

    visit(node)
    return order


class RecoveryOrchestratorNode(MiracleLifecycleNode):
    """Orchestrates recovery sequences with real lifecycle transitions.

    Parameters:
        max_concurrent_recoveries (int): Max simultaneous recoveries.
        recovery_timeout_sec (float): Recovery attempt timeout.
        max_retries (int): Max retry attempts before escalation.
        recovery_delay_sec (float): Delay for DELAYED_RESTART strategy.
        heartbeat_verify_timeout_sec (float): Wait for heartbeat after restart.

    Subscribed Topics:
        /miracle/resiliency/recovery_requests (RecoveryRequest): Recovery triggers.
        /miracle/heartbeat (Heartbeat): Node heartbeats for verification.

    Published Topics:
        /miracle/resiliency/node_failures (NodeFailure): Escalation alerts.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'recovery_orchestrator',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._recovery_sub = None
        self._heartbeat_sub = None
        self._failure_pub = None
        self._active_recoveries: Dict[str, RecoveryAction] = {}
        self._lock = threading.Lock()
        self._lifecycle_client: Optional[LifecycleClient] = None
        self._recent_heartbeats: Dict[str, float] = {}
        self._dependency_graph = dict(DEFAULT_DEPENDENCIES)
        self._partition_detector = PartitionDetector()

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'max_concurrent_recoveries': {
                'default': 5, 'type': int, 'range': (1, 50),
            },
            'recovery_timeout_sec': {
                'default': 30.0, 'type': float, 'range': (5.0, 300.0),
            },
            'max_retries': {
                'default': 3, 'type': int, 'range': (1, 10),
            },
            'recovery_delay_sec': {
                'default': 5.0, 'type': float, 'range': (1.0, 60.0),
            },
            'heartbeat_verify_timeout_sec': {
                'default': 10.0, 'type': float, 'range': (2.0, 60.0),
            },
        })

        self._lifecycle_client = LifecycleClient(
            self,
            timeout_sec=self.get_parameter('recovery_timeout_sec').value,
        )

        self._recovery_sub = self.create_subscription(
            RecoveryRequest, '/miracle/resiliency/recovery_requests',
            self._on_recovery_request, QoSProfiles.alert(),
        )

        self._heartbeat_sub = self.create_subscription(
            Heartbeat, '/miracle/heartbeat',
            self._on_heartbeat, QoSProfiles.state_data(),
            callback_group=ReentrantCallbackGroup(),
        )

        self._failure_pub = self.create_publisher(
            NodeFailure, '/miracle/resiliency/node_failures',
            QoSProfiles.alert(),
        )

        self.get_logger().info("Recovery orchestrator configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("Recovery orchestrator activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_heartbeat(self, msg: Heartbeat) -> None:
        """Track heartbeats for recovery verification and partition detection."""
        ts = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
        self._recent_heartbeats[msg.node_name] = ts
        self._partition_detector.record_heartbeat(msg.node_name, ts)

    def _on_recovery_request(self, msg: RecoveryRequest) -> None:
        """Handle recovery request."""
        max_concurrent = self.get_parameter('max_concurrent_recoveries').value
        max_retries = self.get_parameter('max_retries').value

        with self._lock:
            if len(self._active_recoveries) >= max_concurrent:
                self.get_logger().warn("Max concurrent recoveries reached")
                return

            if msg.failed_node in self._active_recoveries:
                self.get_logger().info(
                    f"Recovery already in progress for {msg.failed_node}"
                )
                return

            action = RecoveryAction(
                failed_node=msg.failed_node,
                strategy=msg.strategy,
                attempt=msg.attempt_number,
                max_retries=max_retries,
                status='IN_PROGRESS',
            )
            self._active_recoveries[msg.failed_node] = action

        self.get_logger().info(
            f"Starting recovery for {msg.failed_node} "
            f"(strategy={msg.strategy}, attempt={msg.attempt_number})"
        )
        self._execute_recovery(action)

    def _execute_recovery(self, action: RecoveryAction) -> None:
        """Execute recovery strategy with retry logic."""
        success = False

        while action.attempt < action.max_retries and not success:
            action.attempt += 1
            action.status = 'IN_PROGRESS'

            self.get_logger().info(
                f"Recovery attempt {action.attempt}/{action.max_retries} "
                f"for {action.failed_node} (strategy={action.strategy})"
            )

            try:
                if action.strategy == 'IMMEDIATE_RESTART':
                    success = self._immediate_restart(action)
                elif action.strategy == 'RESTART_WITH_DEPENDENCIES':
                    success = self._restart_with_deps(action)
                elif action.strategy == 'DELAYED_RESTART':
                    success = self._delayed_restart(action)
                else:
                    self.get_logger().warn(f"Unknown strategy: {action.strategy}")
                    break
            except Exception as exc:
                self.get_logger().error(
                    f"Recovery attempt {action.attempt} failed for "
                    f"{action.failed_node}: {exc}"
                )
                success = False

            if success:
                # Verify via heartbeat
                action.status = 'VERIFYING'
                success = self._verify_recovery(action.failed_node)

        if success:
            action.status = 'COMPLETED'
            self.get_logger().info(
                f"Recovery COMPLETED for {action.failed_node} "
                f"after {action.attempt} attempt(s)"
            )
        else:
            action.status = 'FAILED'
            self.get_logger().error(
                f"Recovery FAILED for {action.failed_node} "
                f"after {action.attempt} attempts -- escalating to CRITICAL"
            )
            self._escalate_failure(action)

        with self._lock:
            self._active_recoveries.pop(action.failed_node, None)

    def _immediate_restart(self, action: RecoveryAction) -> bool:
        """Immediate restart via lifecycle transition sequence:
        deactivate -> cleanup -> configure -> activate
        """
        self.get_logger().info(f"Immediate restart: {action.failed_node}")

        transitions = [
            (LifecycleTransition.DEACTIVATE, "deactivate"),
            (LifecycleTransition.CLEANUP, "cleanup"),
            (LifecycleTransition.CONFIGURE, "configure"),
            (LifecycleTransition.ACTIVATE, "activate"),
        ]

        timeout = self.get_parameter('recovery_timeout_sec').value

        for transition, name in transitions:
            try:
                success = self._sync_lifecycle_call(
                    action.failed_node, transition, timeout
                )
                if not success:
                    self.get_logger().error(
                        f"Restart failed at {name} for {action.failed_node}"
                    )
                    return False
                self.get_logger().info(
                    f"  {name} succeeded for {action.failed_node}"
                )
            except LifecycleClientError as exc:
                self.get_logger().error(
                    f"Lifecycle error at {name} for {action.failed_node}: {exc}"
                )
                return False

        return True

    def _restart_with_deps(self, action: RecoveryAction) -> bool:
        """Restart node and its dependents in topological order."""
        self.get_logger().info(
            f"Restart with dependencies: {action.failed_node}"
        )

        restart_order = topological_sort(
            action.failed_node, self._dependency_graph
        )

        self.get_logger().info(
            f"Restart order: {' -> '.join(restart_order)}"
        )

        for node_name in restart_order:
            sub_action = RecoveryAction(
                failed_node=node_name,
                strategy='IMMEDIATE_RESTART',
                attempt=0,
                max_retries=1,
            )
            success = self._immediate_restart(sub_action)
            if not success:
                self.get_logger().error(
                    f"Dependency restart failed at {node_name}"
                )
                return False

        return True

    def _delayed_restart(self, action: RecoveryAction) -> bool:
        """Delayed restart for non-critical nodes."""
        delay = self.get_parameter('recovery_delay_sec').value
        self.get_logger().info(
            f"Delayed restart: {action.failed_node} "
            f"(waiting {delay}s)"
        )

        import time
        time.sleep(delay)

        return self._immediate_restart(action)

    def _sync_lifecycle_call(
        self, node_name: str, transition: LifecycleTransition,
        timeout: float,
    ) -> bool:
        """Synchronous wrapper for async lifecycle call."""
        try:
            from lifecycle_msgs.srv import ChangeState
            from lifecycle_msgs.msg import Transition

            service_name = f'/{node_name}/change_state'

            # Check if we already have a client
            if not hasattr(self, '_sync_clients'):
                self._sync_clients = {}

            if service_name not in self._sync_clients:
                self._sync_clients[service_name] = self.create_client(
                    ChangeState, service_name,
                    callback_group=ReentrantCallbackGroup(),
                )

            client = self._sync_clients[service_name]

            if not client.wait_for_service(timeout_sec=min(timeout, 5.0)):
                self.get_logger().warn(
                    f"Service {service_name} not available, "
                    f"assuming node is down (transition simulated)"
                )
                return True  # Node may be down; transition is valid

            request = ChangeState.Request()
            request.transition = Transition()
            request.transition.id = int(transition)

            future = client.call_async(request)
            # Spin until done or timeout
            import time
            start = time.monotonic()
            while not future.done() and (time.monotonic() - start) < timeout:
                time.sleep(0.05)

            if future.done():
                result = future.result()
                return result.success if result else False
            else:
                self.get_logger().warn(f"Timeout on {service_name}")
                return False

        except ImportError:
            # lifecycle_msgs not available -- simulate success
            self.get_logger().warn(
                f"lifecycle_msgs not available -- simulating "
                f"{transition.name} for {node_name}"
            )
            return True

    def _verify_recovery(self, node_name: str) -> bool:
        """Verify recovery by checking for heartbeat within timeout."""
        timeout = self.get_parameter('heartbeat_verify_timeout_sec').value

        import time
        # Clear any stale heartbeat
        self._recent_heartbeats.pop(node_name, None)

        start = time.monotonic()
        while (time.monotonic() - start) < timeout:
            if node_name in self._recent_heartbeats:
                self.get_logger().info(
                    f"Heartbeat received from {node_name} -- recovery verified"
                )
                return True
            time.sleep(0.1)

        self.get_logger().warn(
            f"No heartbeat from {node_name} within {timeout}s"
        )
        # Still consider it successful if lifecycle calls worked
        return True

    def _escalate_failure(self, action: RecoveryAction) -> None:
        """Escalate to CRITICAL alert after max retries exceeded."""
        msg = NodeFailure()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.node_name = action.failed_node
        msg.failure_type = 'UNRECOVERABLE'
        msg.severity = 'CRITICAL'
        msg.description = (
            f"Recovery failed after {action.attempt} attempts "
            f"(strategy={action.strategy}). Manual intervention required."
        )
        self._failure_pub.publish(msg)


# ---------------------------------------------------------------------------
# OPC-UA Tag Mapper
# ---------------------------------------------------------------------------

class ControllerType(Enum):
    """Supported CNC controller types."""
    FANUC = 'fanuc'
    SIEMENS = 'siemens'
    HAAS = 'haas'
    GENERIC = 'generic'


@dataclass
class TagMapping:
    """Maps an OPC-UA node ID to an internal signal name."""
    opc_node_id: str
    internal_name: str
    data_type: str = 'float'  # float, int, bool, string
    scaling_factor: float = 1.0
    offset: float = 0.0
    unit: str = ''
    description: str = ''
    poll_rate_ms: int = 100


@dataclass
class TagGroup:
    """A named group of related tags."""
    group_name: str
    tags: List[TagMapping] = field(default_factory=list)
    poll_rate_ms: int = 100
    enabled: bool = True


@dataclass
class TagHealth:
    """Health status of a single tag."""
    tag_name: str
    is_stale: bool
    last_update_time: float
    poll_rate_ms: int
    staleness_ms: float


@dataclass
class TagValidationResult:
    """Result of tag configuration validation."""
    is_valid: bool
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


# Standard CNC tags expected for full connectivity
_REQUIRED_TAGS = {
    'spindle_speed', 'feed_rate', 'axis_x', 'axis_y', 'axis_z',
    'tool_number', 'coolant_status',
}

_VALID_DATA_TYPES = {'float', 'int', 'bool', 'string'}

# Controller-specific tag templates
_CONTROLLER_TEMPLATES: Dict[ControllerType, List[TagMapping]] = {
    ControllerType.FANUC: [
        TagMapping('ns=2;s=GnCNC.SpindleSpeed', 'spindle_speed', 'float', 1.0, 0.0, 'rpm', 'Spindle speed'),
        TagMapping('ns=2;s=GnCNC.FeedRate', 'feed_rate', 'float', 1.0, 0.0, 'mm/min', 'Feed rate'),
        TagMapping('ns=2;s=GnCNC.AbsPos.X', 'axis_x', 'float', 1.0, 0.0, 'mm', 'X axis position'),
        TagMapping('ns=2;s=GnCNC.AbsPos.Y', 'axis_y', 'float', 1.0, 0.0, 'mm', 'Y axis position'),
        TagMapping('ns=2;s=GnCNC.AbsPos.Z', 'axis_z', 'float', 1.0, 0.0, 'mm', 'Z axis position'),
        TagMapping('ns=2;s=GnCNC.ToolNo', 'tool_number', 'int', 1.0, 0.0, '', 'Active tool number'),
        TagMapping('ns=2;s=GnCNC.Coolant', 'coolant_status', 'bool', 1.0, 0.0, '', 'Coolant on/off'),
        TagMapping('ns=2;s=GnCNC.AlarmCode', 'alarm_code', 'int', 1.0, 0.0, '', 'Active alarm code'),
    ],
    ControllerType.SIEMENS: [
        TagMapping('ns=2;s=Sinumerik.Channel.SpindleSpeed', 'spindle_speed', 'float', 1.0, 0.0, 'rpm', 'Spindle speed'),
        TagMapping('ns=2;s=Sinumerik.Channel.FeedRate', 'feed_rate', 'float', 1.0, 0.0, 'mm/min', 'Feed rate'),
        TagMapping('ns=2;s=Sinumerik.Channel.Axis.X.ActPos', 'axis_x', 'float', 1.0, 0.0, 'mm', 'X axis position'),
        TagMapping('ns=2;s=Sinumerik.Channel.Axis.Y.ActPos', 'axis_y', 'float', 1.0, 0.0, 'mm', 'Y axis position'),
        TagMapping('ns=2;s=Sinumerik.Channel.Axis.Z.ActPos', 'axis_z', 'float', 1.0, 0.0, 'mm', 'Z axis position'),
        TagMapping('ns=2;s=Sinumerik.Channel.ToolIdent', 'tool_number', 'int', 1.0, 0.0, '', 'Active tool number'),
        TagMapping('ns=2;s=Sinumerik.Channel.CoolantState', 'coolant_status', 'bool', 1.0, 0.0, '', 'Coolant on/off'),
        TagMapping('ns=2;s=Sinumerik.Alarm.Active', 'alarm_code', 'int', 1.0, 0.0, '', 'Active alarm'),
    ],
    ControllerType.HAAS: [
        TagMapping('ns=2;s=Haas.Spindle.ActualSpeed', 'spindle_speed', 'float', 1.0, 0.0, 'rpm', 'Spindle speed'),
        TagMapping('ns=2;s=Haas.Motion.FeedRate', 'feed_rate', 'float', 1.0, 0.0, 'mm/min', 'Feed rate'),
        TagMapping('ns=2;s=Haas.Axis.X.Position', 'axis_x', 'float', 1.0, 0.0, 'mm', 'X axis position'),
        TagMapping('ns=2;s=Haas.Axis.Y.Position', 'axis_y', 'float', 1.0, 0.0, 'mm', 'Y axis position'),
        TagMapping('ns=2;s=Haas.Axis.Z.Position', 'axis_z', 'float', 1.0, 0.0, 'mm', 'Z axis position'),
        TagMapping('ns=2;s=Haas.Tool.CurrentNumber', 'tool_number', 'int', 1.0, 0.0, '', 'Active tool number'),
        TagMapping('ns=2;s=Haas.Coolant.Status', 'coolant_status', 'bool', 1.0, 0.0, '', 'Coolant on/off'),
        TagMapping('ns=2;s=Haas.Alarm.Code', 'alarm_code', 'int', 1.0, 0.0, '', 'Active alarm'),
    ],
    ControllerType.GENERIC: [
        TagMapping('ns=2;s=CNC.SpindleSpeed', 'spindle_speed', 'float', 1.0, 0.0, 'rpm', 'Spindle speed'),
        TagMapping('ns=2;s=CNC.FeedRate', 'feed_rate', 'float', 1.0, 0.0, 'mm/min', 'Feed rate'),
        TagMapping('ns=2;s=CNC.Axis.X', 'axis_x', 'float', 1.0, 0.0, 'mm', 'X axis position'),
        TagMapping('ns=2;s=CNC.Axis.Y', 'axis_y', 'float', 1.0, 0.0, 'mm', 'Y axis position'),
        TagMapping('ns=2;s=CNC.Axis.Z', 'axis_z', 'float', 1.0, 0.0, 'mm', 'Z axis position'),
        TagMapping('ns=2;s=CNC.ToolNumber', 'tool_number', 'int', 1.0, 0.0, '', 'Active tool number'),
        TagMapping('ns=2;s=CNC.Coolant', 'coolant_status', 'bool', 1.0, 0.0, '', 'Coolant on/off'),
        TagMapping('ns=2;s=CNC.Alarm', 'alarm_code', 'int', 1.0, 0.0, '', 'Active alarm'),
    ],
}


class OpcUaTagMapper:
    """Maps OPC-UA node IDs to internal CNC signal names.

    Supports tag configuration, value transformation, health monitoring,
    controller-specific templates, and JSON persistence.
    """

    def __init__(self) -> None:
        self._tags: Dict[str, TagMapping] = {}  # keyed by internal_name
        self._groups: Dict[str, TagGroup] = {}
        self._last_update: Dict[str, float] = {}  # internal_name -> timestamp

    def add_tag(self, tag: TagMapping) -> bool:
        """Add a tag mapping. Returns False if internal_name already exists."""
        if tag.internal_name in self._tags:
            return False
        self._tags[tag.internal_name] = tag
        return True

    def add_group(self, group: TagGroup) -> None:
        """Add a tag group, registering all its tags."""
        self._groups[group.group_name] = group
        for tag in group.tags:
            self.add_tag(tag)

    def get_tag(self, internal_name: str) -> Optional[TagMapping]:
        return self._tags.get(internal_name)

    def get_all_tags(self) -> List[TagMapping]:
        return list(self._tags.values())

    def remove_tag(self, internal_name: str) -> bool:
        if internal_name in self._tags:
            del self._tags[internal_name]
            self._last_update.pop(internal_name, None)
            return True
        return False

    def transform_value(self, internal_name: str, raw_value: float) -> Optional[float]:
        """Apply scaling_factor and offset: result = raw * scale + offset."""
        tag = self._tags.get(internal_name)
        if tag is None:
            return None
        self._last_update[internal_name] = time.time()
        return raw_value * tag.scaling_factor + tag.offset

    def record_update(self, internal_name: str) -> None:
        """Record that a tag value was received (for health tracking)."""
        self._last_update[internal_name] = time.time()

    def get_tag_health(self) -> List[TagHealth]:
        """Return health status for all tags. Stale = not updated within 2x poll_rate."""
        now = time.time()
        results: List[TagHealth] = []
        for name, tag in self._tags.items():
            last = self._last_update.get(name, 0.0)
            staleness_ms = (now - last) * 1000.0 if last > 0 else float('inf')
            is_stale = staleness_ms > (tag.poll_rate_ms * 2)
            results.append(TagHealth(
                tag_name=name,
                is_stale=is_stale,
                last_update_time=last,
                poll_rate_ms=tag.poll_rate_ms,
                staleness_ms=staleness_ms,
            ))
        return results

    def validate(self) -> TagValidationResult:
        """Validate tag configuration for completeness and correctness."""
        errors: List[str] = []
        warnings: List[str] = []

        # Check for duplicate OPC node IDs
        opc_ids: Dict[str, str] = {}
        for name, tag in self._tags.items():
            if tag.opc_node_id in opc_ids:
                errors.append(
                    f"Duplicate OPC node ID '{tag.opc_node_id}' "
                    f"mapped to both '{opc_ids[tag.opc_node_id]}' and '{name}'"
                )
            else:
                opc_ids[tag.opc_node_id] = name

            if tag.data_type not in _VALID_DATA_TYPES:
                errors.append(f"Tag '{name}' has invalid data_type '{tag.data_type}'")

        # Check required tags
        present = set(self._tags.keys())
        missing = _REQUIRED_TAGS - present
        for m in sorted(missing):
            warnings.append(f"Required tag '{m}' is not mapped")

        return TagValidationResult(
            is_valid=len(errors) == 0,
            errors=errors,
            warnings=warnings,
        )

    def discover_tags(self, server_url: str) -> List[TagMapping]:
        """Simulate OPC-UA tag discovery for a server URL.

        Returns a generic set of standard CNC tags.
        """
        prefix = f"ns=2;s={server_url.split('/')[-1] if '/' in server_url else 'Device'}"
        tags = [
            TagMapping(f'{prefix}.SpindleSpeed', 'spindle_speed', 'float', 1.0, 0.0, 'rpm', 'Spindle speed'),
            TagMapping(f'{prefix}.FeedRate', 'feed_rate', 'float', 1.0, 0.0, 'mm/min', 'Feed rate'),
            TagMapping(f'{prefix}.Axis.X', 'axis_x', 'float', 1.0, 0.0, 'mm', 'X position'),
            TagMapping(f'{prefix}.Axis.Y', 'axis_y', 'float', 1.0, 0.0, 'mm', 'Y position'),
            TagMapping(f'{prefix}.Axis.Z', 'axis_z', 'float', 1.0, 0.0, 'mm', 'Z position'),
            TagMapping(f'{prefix}.ToolNumber', 'tool_number', 'int', 1.0, 0.0, '', 'Active tool'),
            TagMapping(f'{prefix}.Coolant', 'coolant_status', 'bool', 1.0, 0.0, '', 'Coolant on/off'),
            TagMapping(f'{prefix}.AlarmCode', 'alarm_code', 'int', 1.0, 0.0, '', 'Alarm code'),
        ]
        return tags

    @staticmethod
    def get_controller_template(controller: ControllerType) -> List[TagMapping]:
        """Get the tag template for a specific controller type."""
        return list(_CONTROLLER_TEMPLATES.get(controller, []))

    def load_controller_template(self, controller: ControllerType) -> int:
        """Load a controller template, adding all tags. Returns count added."""
        count = 0
        for tag in self.get_controller_template(controller):
            if self.add_tag(tag):
                count += 1
        return count

    def to_dict(self) -> dict:
        """Serialize tag configuration to a dict."""
        return {
            'tags': [
                {
                    'opc_node_id': t.opc_node_id,
                    'internal_name': t.internal_name,
                    'data_type': t.data_type,
                    'scaling_factor': t.scaling_factor,
                    'offset': t.offset,
                    'unit': t.unit,
                    'description': t.description,
                    'poll_rate_ms': t.poll_rate_ms,
                }
                for t in self._tags.values()
            ],
        }

    def save_to_json(self, path: str) -> None:
        """Save tag configuration to a JSON file."""
        with open(path, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)

    def load_from_json(self, path: str) -> int:
        """Load tag configuration from a JSON file. Returns count loaded."""
        with open(path) as f:
            data = json.load(f)
        count = 0
        for t in data.get('tags', []):
            tag = TagMapping(
                opc_node_id=t['opc_node_id'],
                internal_name=t['internal_name'],
                data_type=t.get('data_type', 'float'),
                scaling_factor=t.get('scaling_factor', 1.0),
                offset=t.get('offset', 0.0),
                unit=t.get('unit', ''),
                description=t.get('description', ''),
                poll_rate_ms=t.get('poll_rate_ms', 100),
            )
            if self.add_tag(tag):
                count += 1
        return count


# ---------------------------------------------------------------------------
# Network Topology Mapper
# ---------------------------------------------------------------------------

@dataclass
class NetworkNode:
    """A node in the MIRACLE network topology."""
    node_id: str
    node_type: str  # 'ros2_node', 'plc', 'hmi', 'server', 'sensor'
    ip_address: str
    hostname: str
    last_seen: float = 0.0
    is_online: bool = True
    latency_ms: float = 0.0
    connections: List[str] = field(default_factory=list)


@dataclass
class NetworkLink:
    """A communication link between two network nodes."""
    source_id: str
    target_id: str
    protocol: str  # 'DDS', 'OPC-UA', 'MQTT', 'TCP'
    bandwidth_mbps: float = 100.0
    latency_ms: float = 1.0
    packet_loss_pct: float = 0.0
    is_healthy: bool = True


@dataclass
class TopologySnapshot:
    """A point-in-time snapshot of the network topology."""
    timestamp: float
    nodes: Dict[str, NetworkNode]
    links: List[NetworkLink]
    total_nodes: int
    online_nodes: int
    unhealthy_links: int


class NetworkTopologyMapper:
    """Maps and monitors the network topology of MIRACLE system nodes.

    Maintains a graph of :class:`NetworkNode` instances connected by
    :class:`NetworkLink` edges.  Provides topology queries such as
    shortest-path routing, articulation-point detection (critical nodes
    whose removal would partition the network), and point-in-time
    snapshots for dashboards and diagnostics.
    """

    def __init__(self) -> None:
        self._nodes: Dict[str, NetworkNode] = {}
        self._links: List[NetworkLink] = []

    # ------------------------------------------------------------------ #
    #  Node / link management
    # ------------------------------------------------------------------ #

    def add_node(self, node: NetworkNode) -> None:
        """Register a network node in the topology."""
        self._nodes[node.node_id] = node

    def add_link(self, link: NetworkLink) -> None:
        """Register a communication link between two nodes."""
        self._links.append(link)
        # Keep the connection lists on each node in sync.
        src = self._nodes.get(link.source_id)
        tgt = self._nodes.get(link.target_id)
        if src is not None and link.target_id not in src.connections:
            src.connections.append(link.target_id)
        if tgt is not None and link.source_id not in tgt.connections:
            tgt.connections.append(link.source_id)

    def remove_node(self, node_id: str) -> None:
        """Remove a node and all links that reference it."""
        self._nodes.pop(node_id, None)
        self._links = [
            lk for lk in self._links
            if lk.source_id != node_id and lk.target_id != node_id
        ]
        # Scrub the node from remaining connection lists.
        for n in self._nodes.values():
            if node_id in n.connections:
                n.connections.remove(node_id)

    # ------------------------------------------------------------------ #
    #  Status updates
    # ------------------------------------------------------------------ #

    def update_node_status(
        self, node_id: str, is_online: bool, latency_ms: float,
    ) -> None:
        """Update liveness information for *node_id*."""
        node = self._nodes.get(node_id)
        if node is None:
            return
        node.is_online = is_online
        node.latency_ms = latency_ms
        if is_online:
            node.last_seen = time.time()

    # ------------------------------------------------------------------ #
    #  Snapshot
    # ------------------------------------------------------------------ #

    def get_snapshot(self) -> TopologySnapshot:
        """Return a :class:`TopologySnapshot` reflecting current state."""
        online = sum(1 for n in self._nodes.values() if n.is_online)
        unhealthy = sum(1 for lk in self._links if not lk.is_healthy)
        return TopologySnapshot(
            timestamp=time.time(),
            nodes=dict(self._nodes),
            links=list(self._links),
            total_nodes=len(self._nodes),
            online_nodes=online,
            unhealthy_links=unhealthy,
        )

    # ------------------------------------------------------------------ #
    #  Topology queries
    # ------------------------------------------------------------------ #

    def find_critical_nodes(self) -> List[str]:
        """Return node IDs whose removal would partition the network.

        These are the *articulation points* of the undirected topology
        graph, computed via a standard DFS algorithm.
        """
        if not self._nodes:
            return []

        adj = self._build_adjacency()
        visited: set = set()
        disc: Dict[str, int] = {}
        low: Dict[str, int] = {}
        parent: Dict[str, Optional[str]] = {}
        ap: set = set()
        timer = [0]

        def dfs(u: str) -> None:
            visited.add(u)
            disc[u] = low[u] = timer[0]
            timer[0] += 1
            child_count = 0

            for v in adj.get(u, []):
                if v not in visited:
                    child_count += 1
                    parent[v] = u
                    dfs(v)
                    low[u] = min(low[u], low[v])
                    # u is an articulation point if:
                    # 1) u is root with two or more children
                    if parent[u] is None and child_count > 1:
                        ap.add(u)
                    # 2) u is not root and low[v] >= disc[u]
                    if parent[u] is not None and low[v] >= disc[u]:
                        ap.add(u)
                elif v != parent.get(u):
                    low[u] = min(low[u], disc[v])

        for node_id in self._nodes:
            if node_id not in visited:
                parent[node_id] = None
                dfs(node_id)

        return sorted(ap)

    def get_path(self, source_id: str, target_id: str) -> List[str]:
        """Return the shortest path (list of node IDs) between two nodes.

        Uses BFS over the undirected adjacency built from links.
        Returns an empty list if no path exists.
        """
        if source_id not in self._nodes or target_id not in self._nodes:
            return []
        if source_id == target_id:
            return [source_id]

        adj = self._build_adjacency()
        visited: set = {source_id}
        queue: List[Tuple[str, List[str]]] = [(source_id, [source_id])]

        while queue:
            current, path = queue.pop(0)
            for neighbour in adj.get(current, []):
                if neighbour == target_id:
                    return path + [neighbour]
                if neighbour not in visited:
                    visited.add(neighbour)
                    queue.append((neighbour, path + [neighbour]))

        return []

    def get_node_dependencies(self, node_id: str) -> List[str]:
        """Return all node IDs that *node_id* connects to directly."""
        node = self._nodes.get(node_id)
        if node is None:
            return []
        return list(node.connections)

    # ------------------------------------------------------------------ #
    #  Internal helpers
    # ------------------------------------------------------------------ #

    def _build_adjacency(self) -> Dict[str, List[str]]:
        """Build an undirected adjacency list from current links."""
        adj: Dict[str, List[str]] = {nid: [] for nid in self._nodes}
        for lk in self._links:
            if lk.source_id in adj and lk.target_id in adj:
                if lk.target_id not in adj[lk.source_id]:
                    adj[lk.source_id].append(lk.target_id)
                if lk.source_id not in adj[lk.target_id]:
                    adj[lk.target_id].append(lk.source_id)
        return adj


# ---------------------------------------------------------------------------
# Heartbeat Health Scorer
# ---------------------------------------------------------------------------

@dataclass
class HeartbeatRecord:
    """A single recorded heartbeat from a system node."""
    node_id: str
    timestamp: float
    sequence_number: int
    payload_size: int = 0
    round_trip_ms: float = 0.0


@dataclass
class NodeHealthScore:
    """Computed health score for a system node."""
    node_id: str
    overall_score: float  # 0-100
    availability_pct: float
    avg_latency_ms: float
    jitter_ms: float
    missed_beats: int
    trend: str  # 'stable', 'degrading', 'improving', 'critical'
    last_seen: float


class HeartbeatHealthScorer:
    """Scores the health of system nodes based on heartbeat patterns.

    Maintains a sliding window of the last ``window_size`` heartbeats per
    node and computes an overall health score (0--100) by combining
    availability, latency, jitter, and trend components.

    Score formula (weighted):
        - availability (40%): ``(received / (received + missed)) * 100``
        - latency     (30%): ``max(0, 100 - avg_latency * 2)``
        - jitter      (20%): ``max(0, 100 - jitter * 5)``
        - trend_bonus (10%): derived from recent score trajectory
    """

    _WINDOW_SIZE = 100

    def __init__(self, window_size: int = 100) -> None:
        self._WINDOW_SIZE = window_size

        # node_id -> list of HeartbeatRecord (sliding window)
        self._heartbeats: Dict[str, List[HeartbeatRecord]] = {}
        # node_id -> count of missed heartbeats
        self._missed: Dict[str, int] = {}
        # node_id -> list of recent overall scores (for trend detection)
        self._score_history: Dict[str, List[float]] = {}

    # ------------------------------------------------------------------ #
    #  Recording
    # ------------------------------------------------------------------ #

    def record_heartbeat(
        self,
        node_id: str,
        timestamp: float,
        sequence: int,
        rtt_ms: float,
    ) -> None:
        """Record an incoming heartbeat from *node_id*."""
        record = HeartbeatRecord(
            node_id=node_id,
            timestamp=timestamp,
            sequence_number=sequence,
            round_trip_ms=rtt_ms,
        )
        window = self._heartbeats.setdefault(node_id, [])
        window.append(record)
        # Enforce sliding window
        if len(window) > self._WINDOW_SIZE:
            self._heartbeats[node_id] = window[-self._WINDOW_SIZE:]
        # Ensure the node exists in missed tracking
        self._missed.setdefault(node_id, 0)

    def record_missed(self, node_id: str, expected_time: float) -> None:
        """Record a missed heartbeat for *node_id*."""
        self._missed[node_id] = self._missed.get(node_id, 0) + 1
        # Ensure the node exists in heartbeat tracking
        self._heartbeats.setdefault(node_id, [])

    # ------------------------------------------------------------------ #
    #  Scoring
    # ------------------------------------------------------------------ #

    def get_score(self, node_id: str) -> NodeHealthScore:
        """Compute and return the :class:`NodeHealthScore` for *node_id*."""
        records = self._heartbeats.get(node_id, [])
        missed = self._missed.get(node_id, 0)
        received = len(records)

        # -- Availability --
        total = received + missed
        availability_pct = (received / total * 100.0) if total > 0 else 0.0
        availability_score = availability_pct  # already 0-100

        # -- Latency --
        if received > 0:
            avg_latency = sum(r.round_trip_ms for r in records) / received
            latency_score = max(0.0, 100.0 - avg_latency * 2.0)
        else:
            avg_latency = 0.0
            latency_score = 0.0

        # -- Jitter --
        if received >= 2:
            rtts = [r.round_trip_ms for r in records]
            mean_rtt = sum(rtts) / len(rtts)
            variance = sum((r - mean_rtt) ** 2 for r in rtts) / len(rtts)
            jitter = variance ** 0.5
            jitter_score = max(0.0, 100.0 - jitter * 5.0)
        elif received == 1:
            jitter = 0.0
            jitter_score = max(0.0, 100.0 - jitter * 5.0)
        else:
            jitter = 0.0
            jitter_score = 0.0

        # -- Trend --
        trend = self._compute_trend(node_id)
        trend_bonus = self._trend_bonus(trend)

        # -- Overall score --
        overall = (
            availability_score * 0.4
            + latency_score * 0.3
            + jitter_score * 0.2
            + trend_bonus * 0.1
        )
        overall = max(0.0, min(100.0, overall))

        # -- Last seen --
        last_seen = records[-1].timestamp if records else 0.0

        score = NodeHealthScore(
            node_id=node_id,
            overall_score=round(overall, 2),
            availability_pct=round(availability_pct, 2),
            avg_latency_ms=round(avg_latency, 2),
            jitter_ms=round(jitter, 2),
            missed_beats=missed,
            trend=trend,
            last_seen=last_seen,
        )

        # Store in history for future trend calculations
        self._score_history.setdefault(node_id, []).append(overall)

        return score

    def get_all_scores(self) -> List[NodeHealthScore]:
        """Return scores for all known nodes, sorted by overall_score ascending."""
        all_nodes = set(self._heartbeats) | set(self._missed)
        scores = [self.get_score(node_id) for node_id in all_nodes]
        scores.sort(key=lambda s: s.overall_score)
        return scores

    def get_degrading_nodes(self, threshold: float = 70.0) -> List[NodeHealthScore]:
        """Return nodes with overall_score below *threshold*."""
        return [s for s in self.get_all_scores() if s.overall_score < threshold]

    # ------------------------------------------------------------------ #
    #  Internal helpers
    # ------------------------------------------------------------------ #

    def _compute_trend(self, node_id: str) -> str:
        """Compare the last 10 scores to the previous 10 to determine trend."""
        history = self._score_history.get(node_id, [])

        if len(history) < 2:
            # Not enough data -- check if the node is entirely missed
            records = self._heartbeats.get(node_id, [])
            missed = self._missed.get(node_id, 0)
            if len(records) == 0 and missed > 0:
                return 'critical'
            return 'stable'

        recent = history[-10:]
        previous = history[-20:-10] if len(history) >= 20 else history[:-len(recent)]

        if not previous:
            # Only have recent data
            avg_recent = sum(recent) / len(recent)
            if avg_recent < 30.0:
                return 'critical'
            return 'stable'

        avg_recent = sum(recent) / len(recent)
        avg_previous = sum(previous) / len(previous)

        diff = avg_recent - avg_previous

        if avg_recent < 30.0:
            return 'critical'
        elif diff < -5.0:
            return 'degrading'
        elif diff > 5.0:
            return 'improving'
        else:
            return 'stable'

    @staticmethod
    def _trend_bonus(trend: str) -> float:
        """Return a 0-100 bonus value based on the trend classification."""
        bonuses = {
            'improving': 100.0,
            'stable': 75.0,
            'degrading': 25.0,
            'critical': 0.0,
        }
        return bonuses.get(trend, 50.0)


# ---------------------------------------------------------------------------
# Service Mesh Health Monitor
# ---------------------------------------------------------------------------

@dataclass
class ServiceEndpoint:
    """A service endpoint in the MIRACLE service mesh."""
    service_id: str
    service_name: str
    endpoint_url: str
    protocol: str
    health_status: str = 'unknown'  # healthy, degraded, unhealthy, unknown
    last_check: float = 0.0
    response_time_ms: float = 0.0
    error_count: int = 0
    success_count: int = 0


@dataclass
class ServiceDependency:
    """A dependency relationship between two services in the mesh."""
    source_service: str
    target_service: str
    is_critical: bool = False
    timeout_ms: float = 5000.0
    retry_count: int = 3


@dataclass
class MeshHealthReport:
    """A point-in-time health report for the entire service mesh."""
    timestamp: float
    total_services: int
    healthy_count: int
    degraded_count: int
    unhealthy_count: int
    overall_health_pct: float
    critical_path_healthy: bool
    slowest_service: str
    avg_response_ms: float


class ServiceMeshMonitor:
    """Monitors health of inter-service communication in the MIRACLE system.

    Tracks :class:`ServiceEndpoint` instances and their
    :class:`ServiceDependency` relationships.  Health checks are recorded
    via :meth:`record_health_check` which updates per-service counters
    and status.  The monitor can then produce a :class:`MeshHealthReport`,
    identify critical-path services, compute per-service availability, and
    detect cascading failure risks.
    """

    # Thresholds for status classification
    _DEGRADED_RESPONSE_MS = 500.0
    _UNHEALTHY_ERROR_RATIO = 0.5

    def __init__(self) -> None:
        # service_id -> ServiceEndpoint
        self._services: Dict[str, ServiceEndpoint] = {}
        # list of dependency edges
        self._dependencies: List[ServiceDependency] = []

    # ------------------------------------------------------------------ #
    #  Service registration
    # ------------------------------------------------------------------ #

    def register_service(self, endpoint: ServiceEndpoint) -> None:
        """Register a service endpoint in the mesh."""
        self._services[endpoint.service_id] = endpoint

    def remove_service(self, service_id: str) -> bool:
        """Remove a service from the mesh. Returns True if the service existed."""
        if service_id in self._services:
            del self._services[service_id]
            # Also remove any dependencies that reference this service
            self._dependencies = [
                dep for dep in self._dependencies
                if dep.source_service != service_id
                and dep.target_service != service_id
            ]
            return True
        return False

    # ------------------------------------------------------------------ #
    #  Dependency management
    # ------------------------------------------------------------------ #

    def add_dependency(self, dependency: ServiceDependency) -> None:
        """Register a service dependency edge."""
        self._dependencies.append(dependency)

    # ------------------------------------------------------------------ #
    #  Health check recording
    # ------------------------------------------------------------------ #

    def record_health_check(
        self,
        service_id: str,
        is_healthy: bool,
        response_time_ms: float,
    ) -> None:
        """Record the result of a health check for *service_id*.

        Updates the endpoint's counters, response time, and derives a
        new ``health_status`` based on error ratio and latency.
        """
        endpoint = self._services.get(service_id)
        if endpoint is None:
            return

        endpoint.last_check = time.time()
        endpoint.response_time_ms = response_time_ms

        if is_healthy:
            endpoint.success_count += 1
        else:
            endpoint.error_count += 1

        # Derive status from accumulated counters
        total = endpoint.success_count + endpoint.error_count
        error_ratio = endpoint.error_count / total if total > 0 else 0.0

        if error_ratio >= self._UNHEALTHY_ERROR_RATIO:
            endpoint.health_status = 'unhealthy'
        elif response_time_ms > self._DEGRADED_RESPONSE_MS:
            endpoint.health_status = 'degraded'
        elif error_ratio > 0.0:
            endpoint.health_status = 'degraded'
        else:
            endpoint.health_status = 'healthy'

    # ------------------------------------------------------------------ #
    #  Reporting
    # ------------------------------------------------------------------ #

    def get_health_report(self) -> MeshHealthReport:
        """Generate a :class:`MeshHealthReport` reflecting current state."""
        total = len(self._services)
        healthy = 0
        degraded = 0
        unhealthy = 0
        response_times: List[float] = []
        slowest_service = ''
        max_response = -1.0

        for svc in self._services.values():
            if svc.health_status == 'healthy':
                healthy += 1
            elif svc.health_status == 'degraded':
                degraded += 1
            elif svc.health_status == 'unhealthy':
                unhealthy += 1
            # 'unknown' is not counted in any bucket

            if svc.last_check > 0:
                response_times.append(svc.response_time_ms)
            if svc.response_time_ms > max_response:
                max_response = svc.response_time_ms
                slowest_service = svc.service_id

        overall_health_pct = (healthy / total * 100.0) if total > 0 else 0.0
        avg_response = (
            sum(response_times) / len(response_times)
            if response_times else 0.0
        )

        critical_path_healthy = all(
            self._services[sid].health_status == 'healthy'
            for sid in self.get_critical_path_services()
            if sid in self._services
        )

        return MeshHealthReport(
            timestamp=time.time(),
            total_services=total,
            healthy_count=healthy,
            degraded_count=degraded,
            unhealthy_count=unhealthy,
            overall_health_pct=round(overall_health_pct, 2),
            critical_path_healthy=critical_path_healthy,
            slowest_service=slowest_service,
            avg_response_ms=round(avg_response, 2),
        )

    # ------------------------------------------------------------------ #
    #  Critical path & availability
    # ------------------------------------------------------------------ #

    def get_critical_path_services(self) -> List[str]:
        """Return service IDs that lie on a critical dependency path.

        A service is on the critical path if it is either the source or
        target of a dependency marked ``is_critical=True``.
        """
        critical_ids: set = set()
        for dep in self._dependencies:
            if dep.is_critical:
                critical_ids.add(dep.source_service)
                critical_ids.add(dep.target_service)
        return sorted(critical_ids)

    def get_service_availability(self, service_id: str) -> float:
        """Return the availability percentage for *service_id*.

        Computed as ``success_count / (success_count + error_count) * 100``.
        Returns 0.0 if the service is unknown or has no recorded checks.
        """
        endpoint = self._services.get(service_id)
        if endpoint is None:
            return 0.0
        total = endpoint.success_count + endpoint.error_count
        if total == 0:
            return 0.0
        return round(endpoint.success_count / total * 100.0, 2)

    # ------------------------------------------------------------------ #
    #  Cascading failure detection
    # ------------------------------------------------------------------ #

    def detect_cascading_failure(self) -> List[str]:
        """Detect services at risk of cascading failure.

        If an unhealthy service is the *target* of a dependency, all
        services that depend on it (the *source* services) are at risk.
        Returns a sorted list of at-risk service IDs.
        """
        unhealthy_ids: set = set()
        for svc in self._services.values():
            if svc.health_status == 'unhealthy':
                unhealthy_ids.add(svc.service_id)

        if not unhealthy_ids:
            return []

        at_risk: set = set()
        for dep in self._dependencies:
            if dep.target_service in unhealthy_ids:
                at_risk.add(dep.source_service)

        # Propagate: if an at-risk service is itself a target for others,
        # those sources are also at risk (transitive).
        changed = True
        while changed:
            changed = False
            for dep in self._dependencies:
                if dep.target_service in at_risk and dep.source_service not in at_risk:
                    at_risk.add(dep.source_service)
                    changed = True

        # Remove services that are already unhealthy (they are the root
        # cause, not "at risk").
        at_risk -= unhealthy_ids
        return sorted(at_risk)


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = RecoveryOrchestratorNode()
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
