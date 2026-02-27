"""
Behavior Tree Execution Engine.

Executes modular, reactive behavior trees for autonomous
job execution, anomaly response, and recovery sequences.
Uses a lightweight built-in BT engine (no external dependency required).
"""

from typing import Any, Dict, List, Optional, Callable
from enum import Enum
from dataclasses import dataclass, field
import threading

from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import BehaviorTreeStatus, MachineState, AnomalyAlert


class BTStatus(Enum):
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'
    RUNNING = 'RUNNING'


@dataclass
class BTNode:
    """Base behavior tree node."""
    name: str
    status: BTStatus = BTStatus.RUNNING
    children: List['BTNode'] = field(default_factory=list)
    node_type: str = 'action'

    def tick(self, blackboard: Dict) -> BTStatus:
        return BTStatus.SUCCESS


class SequenceNode(BTNode):
    """Executes children in order until one fails."""
    def __init__(self, name: str, children: List[BTNode]):
        super().__init__(name=name, children=children, node_type='sequence')
        self._current_idx = 0

    def tick(self, blackboard: Dict) -> BTStatus:
        while self._current_idx < len(self.children):
            status = self.children[self._current_idx].tick(blackboard)
            if status == BTStatus.RUNNING:
                return BTStatus.RUNNING
            elif status == BTStatus.FAILURE:
                self._current_idx = 0
                return BTStatus.FAILURE
            self._current_idx += 1
        self._current_idx = 0
        return BTStatus.SUCCESS


class SelectorNode(BTNode):
    """Tries children until one succeeds."""
    def __init__(self, name: str, children: List[BTNode]):
        super().__init__(name=name, children=children, node_type='selector')

    def tick(self, blackboard: Dict) -> BTStatus:
        for child in self.children:
            status = child.tick(blackboard)
            if status == BTStatus.SUCCESS:
                return BTStatus.SUCCESS
            elif status == BTStatus.RUNNING:
                return BTStatus.RUNNING
        return BTStatus.FAILURE


class ConditionNode(BTNode):
    """Checks a condition on the blackboard."""
    def __init__(self, name: str, check_fn: Callable):
        super().__init__(name=name, node_type='condition')
        self._check = check_fn

    def tick(self, blackboard: Dict) -> BTStatus:
        return BTStatus.SUCCESS if self._check(blackboard) else BTStatus.FAILURE


class ActionNode(BTNode):
    """Executes an action."""
    def __init__(self, name: str, action_fn: Callable):
        super().__init__(name=name, node_type='action')
        self._action = action_fn

    def tick(self, blackboard: Dict) -> BTStatus:
        return self._action(blackboard)


class BehaviorTreeExecutorNode(MiracleLifecycleNode):
    """Executes behavior trees for autonomous manufacturing.

    Parameters:
        tick_rate_hz (float): BT tick rate.
        tree_library_path (str): Path to BT XML files.

    Subscribed Topics:
        /miracle/{machine_id}/state (MachineState): Machine states for blackboard.
        /miracle/{machine_id}/anomaly (AnomalyAlert): Anomalies for blackboard.

    Published Topics:
        ~/tree_status (BehaviorTreeStatus): Tree execution status.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('behavior_tree_executor', criticality=self.CRITICALITY_HIGH, **kwargs)
        self._active_trees: Dict[str, tuple] = {}  # id -> (root, blackboard)
        self._tick_timer = None
        self._status_pub = None
        self._state_subs = []
        self._anomaly_subs = []
        self._blackboard: Dict[str, Any] = {}
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        params = self.declare_and_validate_parameters({
            'tick_rate_hz': {'default': 10.0, 'type': float, 'range': (1.0, 100.0)},
            'tree_library_path': {'default': '', 'type': str},
            'machine_ids': {'default': 'cnc1,cnc2,cnc3', 'type': str},
        })
        machine_ids = self.get_machine_ids(params)

        self._status_pub = self.create_publisher(
            BehaviorTreeStatus, 'tree_status', QoSProfiles.state_data(),
        )
        self._state_subs = self.create_multi_machine_subscriptions(
            MachineState, 'state',
            self._on_state, QoSProfiles.state_data(), machine_ids,
        )
        self._anomaly_subs = self.create_multi_machine_subscriptions(
            AnomalyAlert, 'anomaly',
            self._on_anomaly, QoSProfiles.alert(), machine_ids,
        )

        self._blackboard = {'machine_states': {}, 'anomalies': [], 'jobs': {}}
        self.get_logger().info("Behavior tree executor configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        rate = self.get_parameter('tick_rate_hz').value
        self._tick_timer = self.create_timer(
            1.0 / rate, self._tick_trees, callback_group=self.realtime_callback_group,
        )
        self.get_logger().info(f"BT executor activated at {rate}Hz")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        if self._tick_timer:
            self._tick_timer.cancel()
            self._tick_timer = None
        self._active_trees.clear()
        return TransitionCallbackReturn.SUCCESS

    def _on_state(self, msg: MachineState) -> None:
        self._blackboard['machine_states'][msg.machine_id] = msg

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        anomalies = self._blackboard.get('anomalies', [])
        anomalies.append(msg)
        if len(anomalies) > 100:
            self._blackboard['anomalies'] = anomalies[-50:]

    def start_tree(self, tree_id: str, root: BTNode) -> None:
        """Start executing a behavior tree."""
        with self._lock:
            self._active_trees[tree_id] = (root, self._blackboard)
        self.get_logger().info(f"Started BT: {tree_id}")

    def stop_tree(self, tree_id: str) -> None:
        """Stop a behavior tree."""
        with self._lock:
            self._active_trees.pop(tree_id, None)
        self.get_logger().info(f"Stopped BT: {tree_id}")

    def _tick_trees(self) -> None:
        """Tick all active trees."""
        with self._lock:
            completed = []
            for tree_id, (root, bb) in self._active_trees.items():
                try:
                    status = root.tick(bb)

                    msg = BehaviorTreeStatus()
                    msg.timestamp = self.get_clock().now().to_msg()
                    msg.tree_id = tree_id
                    msg.root_status = status.value
                    msg.tip_name = root.name
                    self._status_pub.publish(msg)

                    if status != BTStatus.RUNNING:
                        completed.append(tree_id)
                        self.get_logger().info(f"BT {tree_id}: {status.value}")
                except Exception as exc:
                    self.get_logger().error(f"BT tick error {tree_id}: {exc}")
                    completed.append(tree_id)

            for tid in completed:
                del self._active_trees[tid]

    def build_autonomous_job_tree(self, job_id: str, machine_id: str) -> BTNode:
        """Build a behavior tree for autonomous job execution."""
        return SelectorNode("AutonomousJob", [
            SequenceNode("NormalOps", [
                ConditionNode("MachineReady", lambda bb: (
                    machine_id in bb.get('machine_states', {}) and
                    bb['machine_states'][machine_id].status in ('IDLE', 'READY')
                )),
                ActionNode("Execute", lambda bb: BTStatus.SUCCESS),
            ]),
            SequenceNode("SafetyFallback", [
                ActionNode("AlertOperator", lambda bb: BTStatus.SUCCESS),
            ]),
        ])


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = BehaviorTreeExecutorNode()
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
