"""Tests for the A* search GOAP (Goal-Oriented Action Planning) planner.

Tests plan generation with simple state/goal combinations, unreachable
goals, cost optimality, and action precondition checking.
"""

import heapq
from dataclasses import dataclass
from typing import FrozenSet, List, Optional, Set

import pytest


@dataclass
class GOAPAction:
    """A GOAP action with preconditions and effects."""
    name: str
    cost: float
    preconditions: FrozenSet[str]
    effects_add: FrozenSet[str]
    effects_remove: FrozenSet[str] = frozenset()


class _GOAPPlanner:
    """Extracted A* GOAP planning logic from GOAPPlannerNode.

    Performs A* search over a state space defined by preconditions and
    effects, without any ROS2 dependencies.
    """

    def __init__(self, actions: List[GOAPAction], max_nodes: int = 10000):
        self.actions = actions
        self.max_nodes = max_nodes

    def plan(self, current_state: FrozenSet[str], goal_state: FrozenSet[str]) -> Optional[List[str]]:
        """Find a plan using A* search."""
        start = (0.0, 0.0, current_state, [])
        queue = [start]
        visited: Set[FrozenSet[str]] = set()
        nodes_explored = 0

        while queue and nodes_explored < self.max_nodes:
            _, cost, state, actions = heapq.heappop(queue)
            nodes_explored += 1

            if goal_state.issubset(state):
                return actions

            if state in visited:
                continue
            visited.add(state)

            for action in self.actions:
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


@pytest.fixture
def manufacturing_planner():
    """Return a planner loaded with the MIRACLE manufacturing actions."""
    actions = [
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
    return _GOAPPlanner(actions)


class TestBasicPlanning:
    """Tests for basic plan generation."""

    def test_simple_one_step_plan(self, manufacturing_planner):
        """A goal requiring one action should produce a single-step plan."""
        current = frozenset(['machine_available'])
        goal = frozenset(['machine_ready'])
        plan = manufacturing_planner.plan(current, goal)
        assert plan is not None, "Should find a plan for machine setup"
        assert plan == ['setup_machine']

    def test_multi_step_plan(self, manufacturing_planner):
        """A complex goal should produce a multi-step plan."""
        current = frozenset(['machine_available', 'material_available'])
        goal = frozenset(['part_complete'])
        plan = manufacturing_planner.plan(current, goal)
        assert plan is not None, "Should find a plan for complete part production"
        assert len(plan) >= 4, f"Expected at least 4 steps, got {len(plan)}: {plan}"
        # Verify ordering constraints
        assert plan.index('setup_machine') < plan.index('load_material')
        assert plan.index('load_material') < plan.index('execute_machining')
        assert plan.index('execute_machining') < plan.index('inspect_part')

    def test_goal_already_satisfied(self, manufacturing_planner):
        """If the goal is already met, the plan should be empty."""
        current = frozenset(['machine_available', 'machine_ready'])
        goal = frozenset(['machine_ready'])
        plan = manufacturing_planner.plan(current, goal)
        assert plan is not None
        assert plan == [], f"Plan should be empty when goal is met, got {plan}"


class TestUnreachableGoals:
    """Tests for goals that cannot be achieved."""

    def test_no_plan_for_impossible_goal(self, manufacturing_planner):
        """A goal requiring missing preconditions should return None."""
        current = frozenset()  # No initial state at all
        goal = frozenset(['part_complete'])
        plan = manufacturing_planner.plan(current, goal)
        assert plan is None, "Should return None for unreachable goal"

    def test_no_plan_for_unknown_goal(self, manufacturing_planner):
        """A goal state not achievable by any action should return None."""
        current = frozenset(['machine_available'])
        goal = frozenset(['teleportation_ready'])  # No action produces this
        plan = manufacturing_planner.plan(current, goal)
        assert plan is None, "Should return None for unknown goal state"

    def test_max_nodes_limit(self):
        """Planner should give up after exceeding max_nodes."""
        actions = [
            GOAPAction('loop', 1.0, frozenset(['a']), frozenset(['b']), frozenset(['a'])),
            GOAPAction('loop2', 1.0, frozenset(['b']), frozenset(['a']), frozenset(['b'])),
        ]
        planner = _GOAPPlanner(actions, max_nodes=10)
        plan = planner.plan(frozenset(['a']), frozenset(['goal_impossible']))
        assert plan is None


class TestActionPreconditions:
    """Tests for precondition checking in plan generation."""

    def test_actions_respect_preconditions(self, manufacturing_planner):
        """Actions should only appear when their preconditions are met."""
        current = frozenset(['machine_available', 'material_available'])
        goal = frozenset(['part_machined'])
        plan = manufacturing_planner.plan(current, goal)
        assert plan is not None
        # load_material requires machine_ready, so setup_machine must come first
        assert 'setup_machine' in plan
        assert plan.index('setup_machine') < plan.index('load_material')

    def test_effects_remove_consumed(self, manufacturing_planner):
        """Actions with effects_remove should consume state facts."""
        current = frozenset(['machine_available', 'material_available'])
        goal = frozenset(['part_machined'])
        plan = manufacturing_planner.plan(current, goal)
        # execute_machining removes material_loaded
        assert 'execute_machining' in plan


class TestPlanCost:
    """Tests for plan cost calculation."""

    def test_cheaper_plan_preferred(self):
        """The planner should prefer lower-cost plans."""
        actions = [
            GOAPAction('expensive', 100.0, frozenset(['start']), frozenset(['goal'])),
            GOAPAction('cheap', 1.0, frozenset(['start']), frozenset(['goal'])),
        ]
        planner = _GOAPPlanner(actions)
        plan = planner.plan(frozenset(['start']), frozenset(['goal']))
        assert plan == ['cheap'], "A* should pick the cheaper action"

    def test_multi_step_cheapest_path(self):
        """The planner should find the cheapest multi-step path."""
        actions = [
            GOAPAction('a_to_b_cheap', 1.0, frozenset(['a']), frozenset(['b'])),
            GOAPAction('a_to_b_expensive', 100.0, frozenset(['a']), frozenset(['b'])),
            GOAPAction('b_to_c', 1.0, frozenset(['b']), frozenset(['c'])),
        ]
        planner = _GOAPPlanner(actions)
        plan = planner.plan(frozenset(['a']), frozenset(['c']))
        assert plan == ['a_to_b_cheap', 'b_to_c']
