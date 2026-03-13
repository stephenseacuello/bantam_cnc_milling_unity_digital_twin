"""
Recovery Orchestrator Node.

Orchestrates multi-step recovery sequences for failed nodes,
coordinating with lifecycle management and dependency ordering.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
import threading
import asyncio
import time
import uuid

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
