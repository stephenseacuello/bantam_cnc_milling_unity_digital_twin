"""
Hierarchical Task Network (HTN) Planner Node.

Decomposes high-level manufacturing tasks into primitive actions
using hierarchical methods.
"""

from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field

from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.srv import HTNPlan


@dataclass
class Method:
    """An HTN decomposition method."""
    name: str
    task: str
    preconditions: List[str]
    subtasks: List[str]


@dataclass
class PrimitiveAction:
    """A primitive executable action."""
    name: str
    preconditions: List[str]
    effects: List[str]
    duration: float = 1.0


class HTNPlannerNode(MiracleLifecycleNode):
    """HTN planner for manufacturing task decomposition.

    Parameters:
        max_plan_depth (int): Maximum decomposition depth.
        max_planning_time_sec (float): Planning time limit.

    Service Servers:
        ~/htn_plan (HTNPlan): Generate an HTN plan.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('htn_planner', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._methods: List[Method] = []
        self._primitives: Dict[str, PrimitiveAction] = {}
        self._plan_srv = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'max_plan_depth': {'default': 20, 'type': int, 'range': (1, 100)},
            'max_planning_time_sec': {'default': 5.0, 'type': float, 'range': (0.1, 60.0)},
        })

        self._plan_srv = self.create_service(
            HTNPlan, 'htn_plan',
            self._handle_plan, callback_group=self.service_callback_group,
        )

        self._load_manufacturing_domain()
        self.get_logger().info("HTN planner configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("HTN planner activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _load_manufacturing_domain(self) -> None:
        """Load manufacturing HTN methods and primitives."""
        self._methods = [
            Method('machine_part', 'produce_part',
                   ['material_available', 'machine_idle'],
                   ['setup_machine', 'load_material', 'execute_program', 'inspect_part', 'unload_part']),
            Method('setup_full', 'setup_machine',
                   ['machine_idle'],
                   ['load_tool', 'set_workholding', 'zero_axes', 'load_program']),
        ]
        self._primitives = {
            'load_tool': PrimitiveAction('load_tool', ['tool_available'], ['tool_loaded'], 2.0),
            'set_workholding': PrimitiveAction('set_workholding', [], ['workholding_set'], 3.0),
            'zero_axes': PrimitiveAction('zero_axes', ['machine_powered'], ['axes_zeroed'], 1.0),
            'load_program': PrimitiveAction('load_program', [], ['program_loaded'], 0.5),
            'load_material': PrimitiveAction('load_material', ['material_available'], ['material_loaded'], 2.0),
            'execute_program': PrimitiveAction('execute_program', ['program_loaded', 'tool_loaded'], ['part_machined'], 60.0),
            'inspect_part': PrimitiveAction('inspect_part', ['part_machined'], ['part_inspected'], 5.0),
            'unload_part': PrimitiveAction('unload_part', ['part_inspected'], ['part_complete'], 1.0),
        }

    def plan(self, task: str, state: List[str], depth: int = 0) -> Optional[List[str]]:
        """Generate a plan for a task."""
        max_depth = self.get_parameter('max_plan_depth').value
        if depth > max_depth:
            return None

        # Check if task is a primitive
        if task in self._primitives:
            prim = self._primitives[task]
            if all(p in state for p in prim.preconditions):
                return [task]
            return None

        # Try methods
        for method in self._methods:
            if method.task != task:
                continue
            if not all(p in state for p in method.preconditions):
                continue

            plan: List[str] = []
            current_state = list(state)
            success = True
            for subtask in method.subtasks:
                sub_plan = self.plan(subtask, current_state, depth + 1)
                if sub_plan is None:
                    success = False
                    break
                plan.extend(sub_plan)
                # Apply effects of primitives
                for action_name in sub_plan:
                    if action_name in self._primitives:
                        current_state.extend(self._primitives[action_name].effects)
            if success:
                return plan

        return None

    def _handle_plan(
        self, request: HTNPlan.Request, response: HTNPlan.Response,
    ) -> HTNPlan.Response:
        """Handle HTN planning request."""
        import json
        state = list(request.task_parameters) if request.task_parameters else [
            'machine_idle', 'material_available', 'tool_available', 'machine_powered',
        ]

        plan = self.plan(request.task_name, state)
        if plan is not None:
            response.success = True
            response.plan_steps = plan
            response.estimated_duration = sum(
                self._primitives[a].duration for a in plan if a in self._primitives
            )
            response.plan_tree_json = json.dumps({'task': request.task_name, 'steps': plan})
        else:
            response.success = False
            response.plan_steps = []
            response.estimated_duration = 0.0
            response.plan_tree_json = '{}'

        return response


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = HTNPlannerNode()
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
