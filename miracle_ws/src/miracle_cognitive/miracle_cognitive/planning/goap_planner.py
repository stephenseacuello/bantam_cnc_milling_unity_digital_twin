"""
Goal-Oriented Action Planning (GOAP) Node.

Plans sequences of actions to achieve goals using A* search
over a state space defined by preconditions and effects.
"""

from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple
from dataclasses import dataclass
import heapq

from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.srv import GOAPPlan


@dataclass
class GOAPAction:
    """A GOAP action with preconditions and effects."""
    name: str
    cost: float
    preconditions: FrozenSet[str]
    effects_add: FrozenSet[str]
    effects_remove: FrozenSet[str] = frozenset()


class GOAPPlannerNode(MiracleLifecycleNode):
    """GOAP planner using A* search.

    Parameters:
        max_search_nodes (int): Maximum A* search nodes.

    Service Servers:
        ~/goap_plan (GOAPPlan): Generate a GOAP plan.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('goap_planner', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._actions: List[GOAPAction] = []
        self._plan_srv = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'max_search_nodes': {'default': 10000, 'type': int, 'range': (100, 1000000)},
        })

        self._plan_srv = self.create_service(
            GOAPPlan, 'goap_plan',
            self._handle_plan, callback_group=self.service_callback_group,
        )

        self._load_actions()
        self.get_logger().info("GOAP planner configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("GOAP planner activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _load_actions(self) -> None:
        """Load available GOAP actions."""
        self._actions = [
            GOAPAction('setup_machine', 5.0,
                       frozenset(['machine_available']),
                       frozenset(['machine_ready'])),
            GOAPAction('load_material', 3.0,
                       frozenset(['material_available', 'machine_ready']),
                       frozenset(['material_loaded'])),
            GOAPAction('execute_machining', 60.0,
                       frozenset(['material_loaded', 'machine_ready']),
                       frozenset(['part_machined']),
                       frozenset(['material_loaded'])),
            GOAPAction('inspect_part', 5.0,
                       frozenset(['part_machined']),
                       frozenset(['part_inspected'])),
            GOAPAction('package_part', 2.0,
                       frozenset(['part_inspected']),
                       frozenset(['part_complete'])),
            GOAPAction('change_tool', 3.0,
                       frozenset(['machine_available']),
                       frozenset(['tool_changed'])),
            GOAPAction('calibrate', 10.0,
                       frozenset(['machine_available']),
                       frozenset(['machine_calibrated'])),
        ]

    def plan(
        self, current_state: FrozenSet[str], goal_state: FrozenSet[str],
    ) -> Optional[List[str]]:
        """Find a plan using A* search."""
        max_nodes = self.get_parameter('max_search_nodes').value

        # Priority queue: (cost + heuristic, cost, state, actions)
        start = (0.0, 0.0, current_state, [])
        queue = [start]
        visited: Set[FrozenSet[str]] = set()
        nodes_explored = 0

        while queue and nodes_explored < max_nodes:
            _, cost, state, actions = heapq.heappop(queue)
            nodes_explored += 1

            if goal_state.issubset(state):
                return actions

            if state in visited:
                continue
            visited.add(state)

            for action in self._actions:
                if action.preconditions.issubset(state):
                    new_state = (state - action.effects_remove) | action.effects_add
                    if new_state not in visited:
                        new_cost = cost + action.cost
                        h = len(goal_state - new_state)
                        heapq.heappush(queue, (
                            new_cost + h,
                            new_cost,
                            new_state,
                            actions + [action.name],
                        ))

        return None

    def _handle_plan(
        self, request: GOAPPlan.Request, response: GOAPPlan.Response,
    ) -> GOAPPlan.Response:
        """Handle GOAP planning request."""
        current = frozenset(request.current_state)
        goal = frozenset(request.goal_state)

        plan = self.plan(current, goal)
        if plan is not None:
            response.success = True
            response.action_sequence = plan
            response.total_cost = sum(
                a.cost for a in self._actions if a.name in plan
            )
            response.plan_explanation = f'Found plan with {len(plan)} actions'
        else:
            response.success = False
            response.action_sequence = []
            response.total_cost = 0.0
            response.plan_explanation = 'No plan found'

        return response


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = GOAPPlannerNode()
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
