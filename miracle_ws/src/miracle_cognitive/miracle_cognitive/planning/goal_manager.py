"""
Goal Manager Node.

Manages manufacturing goals and priorities. Decomposes high-level
objectives into actionable sub-goals for planners.
"""

from typing import Any, Dict, List
from dataclasses import dataclass, field
from enum import IntEnum
import threading

from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import SystemKPIs


class GoalPriority(IntEnum):
    SAFETY = 0
    QUALITY = 1
    PRODUCTIVITY = 2
    EFFICIENCY = 3
    COST = 4


@dataclass
class Goal:
    """A manufacturing goal."""
    goal_id: str
    description: str
    priority: GoalPriority
    target_kpi: str = ''
    target_value: float = 0.0
    current_value: float = 0.0
    status: str = 'ACTIVE'
    sub_goals: List[str] = field(default_factory=list)


class GoalManagerNode(MiracleLifecycleNode):
    """Manages manufacturing goals and priorities.

    Parameters:
        max_goals (int): Maximum active goals.
        evaluation_interval_sec (float): Goal evaluation interval.

    Subscribed Topics:
        /miracle/system_kpis (SystemKPIs): KPIs for goal evaluation.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('goal_manager', criticality=self.CRITICALITY_HIGH, **kwargs)
        self._goals: Dict[str, Goal] = {}
        self._lock = threading.Lock()
        self._kpi_sub = None
        self._eval_timer = None
        self._latest_kpis = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'max_goals': {'default': 50, 'type': int, 'range': (1, 500)},
            'evaluation_interval_sec': {'default': 30.0, 'type': float, 'range': (5.0, 300.0)},
        })

        self._kpi_sub = self.create_subscription(
            SystemKPIs, '/miracle/system_kpis',
            self._on_kpis, QoSProfiles.state_data(),
        )

        # Initialize default goals
        self._goals['oee_target'] = Goal(
            goal_id='oee_target', description='Maintain OEE above 85%',
            priority=GoalPriority.PRODUCTIVITY, target_kpi='oee', target_value=0.85,
        )
        self._goals['quality_target'] = Goal(
            goal_id='quality_target', description='Maintain quality above 99%',
            priority=GoalPriority.QUALITY, target_kpi='quality', target_value=0.99,
        )
        self._goals['safety_zero'] = Goal(
            goal_id='safety_zero', description='Zero safety incidents',
            priority=GoalPriority.SAFETY, target_kpi='safety', target_value=0.0,
        )

        self.get_logger().info("Goal manager configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        interval = self.get_parameter('evaluation_interval_sec').value
        self._eval_timer = self.create_timer(
            interval, self._evaluate_goals, callback_group=self.service_callback_group,
        )
        self.get_logger().info("Goal manager activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        if self._eval_timer:
            self._eval_timer.cancel()
            self._eval_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_kpis(self, msg: SystemKPIs) -> None:
        self._latest_kpis = msg

    def _evaluate_goals(self) -> None:
        """Evaluate goal progress against KPIs."""
        if self._latest_kpis is None:
            return

        kpi_map = {
            'oee': self._latest_kpis.oee,
            'quality': self._latest_kpis.quality,
            'availability': self._latest_kpis.availability,
            'performance': self._latest_kpis.performance,
        }

        with self._lock:
            for goal in self._goals.values():
                if goal.target_kpi in kpi_map:
                    goal.current_value = kpi_map[goal.target_kpi]
                    if goal.current_value >= goal.target_value:
                        goal.status = 'MET'
                    else:
                        goal.status = 'ACTIVE'
                        gap = goal.target_value - goal.current_value
                        if gap > 0.1:
                            self.get_logger().info(
                                f"Goal gap: {goal.description} "
                                f"(current={goal.current_value:.2f}, "
                                f"target={goal.target_value:.2f})"
                            )

    def get_priority_goals(self) -> List[Goal]:
        """Get goals sorted by priority."""
        with self._lock:
            active = [g for g in self._goals.values() if g.status == 'ACTIVE']
            return sorted(active, key=lambda g: g.priority)


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = GoalManagerNode()
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
