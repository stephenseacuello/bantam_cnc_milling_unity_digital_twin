"""Tests for HTN (Hierarchical Task Network) decomposition.

Tests primitive action execution, method decomposition, recursive
planning, precondition checking, and depth limits.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import pytest


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


class _HTNPlanner:
    """Extracted HTN planning logic from HTNPlannerNode.

    Decomposes high-level tasks into primitive actions using method
    decomposition, without any ROS2 dependencies.
    """

    def __init__(self, methods: List[Method], primitives: Dict[str, PrimitiveAction],
                 max_depth: int = 20):
        self.methods = methods
        self.primitives = primitives
        self.max_depth = max_depth

    def plan(self, task: str, state: List[str], depth: int = 0) -> Optional[List[str]]:
        """Generate a plan for a task."""
        if depth > self.max_depth:
            return None

        # Check if task is a primitive
        if task in self.primitives:
            prim = self.primitives[task]
            if all(p in state for p in prim.preconditions):
                return [task]
            return None

        # Try methods
        for method in self.methods:
            if method.task != task:
                continue
            if not all(p in state for p in method.preconditions):
                continue

            plan_result: List[str] = []
            current_state = list(state)
            success = True
            for subtask in method.subtasks:
                sub_plan = self.plan(subtask, current_state, depth + 1)
                if sub_plan is None:
                    success = False
                    break
                plan_result.extend(sub_plan)
                for action_name in sub_plan:
                    if action_name in self.primitives:
                        current_state.extend(self.primitives[action_name].effects)
            if success:
                return plan_result

        return None


@pytest.fixture
def manufacturing_planner():
    """Return an HTN planner loaded with manufacturing domain."""
    methods = [
        Method('machine_part', 'produce_part',
               ['material_available', 'machine_idle'],
               ['setup_machine', 'load_material', 'execute_program',
                'inspect_part', 'unload_part']),
        Method('setup_full', 'setup_machine',
               ['machine_idle'],
               ['load_tool', 'set_workholding', 'zero_axes', 'load_program']),
    ]
    primitives = {
        'load_tool': PrimitiveAction('load_tool', ['tool_available'], ['tool_loaded'], 2.0),
        'set_workholding': PrimitiveAction('set_workholding', [], ['workholding_set'], 3.0),
        'zero_axes': PrimitiveAction('zero_axes', ['machine_powered'], ['axes_zeroed'], 1.0),
        'load_program': PrimitiveAction('load_program', [], ['program_loaded'], 0.5),
        'load_material': PrimitiveAction('load_material', ['material_available'], ['material_loaded'], 2.0),
        'execute_program': PrimitiveAction('execute_program', ['program_loaded', 'tool_loaded'], ['part_machined'], 60.0),
        'inspect_part': PrimitiveAction('inspect_part', ['part_machined'], ['part_inspected'], 5.0),
        'unload_part': PrimitiveAction('unload_part', ['part_inspected'], ['part_complete'], 1.0),
    }
    return _HTNPlanner(methods, primitives)


class TestPrimitiveActions:
    """Tests for primitive action resolution."""

    def test_primitive_with_met_preconditions(self, manufacturing_planner):
        """A primitive whose preconditions are met should return a single-step plan."""
        state = ['tool_available']
        plan = manufacturing_planner.plan('load_tool', state)
        assert plan == ['load_tool']

    def test_primitive_with_unmet_preconditions(self, manufacturing_planner):
        """A primitive whose preconditions are NOT met should return None."""
        state = []  # tool_available missing
        plan = manufacturing_planner.plan('load_tool', state)
        assert plan is None

    def test_primitive_with_no_preconditions(self, manufacturing_planner):
        """A primitive with empty preconditions should always succeed."""
        state = []
        plan = manufacturing_planner.plan('set_workholding', state)
        assert plan == ['set_workholding']

    def test_unknown_task_returns_none(self, manufacturing_planner):
        """A task that is neither primitive nor decomposable should return None."""
        plan = manufacturing_planner.plan('fly_to_moon', ['everything'])
        assert plan is None


class TestMethodDecomposition:
    """Tests for hierarchical method decomposition."""

    def test_setup_machine_decomposition(self, manufacturing_planner):
        """setup_machine should decompose into its 4 primitive subtasks."""
        state = ['machine_idle', 'tool_available', 'machine_powered']
        plan = manufacturing_planner.plan('setup_machine', state)
        assert plan is not None, "setup_machine should be decomposable"
        assert plan == ['load_tool', 'set_workholding', 'zero_axes', 'load_program']

    def test_produce_part_full_decomposition(self, manufacturing_planner):
        """produce_part should recursively decompose into all primitive steps."""
        state = [
            'material_available', 'machine_idle',
            'tool_available', 'machine_powered',
        ]
        plan = manufacturing_planner.plan('produce_part', state)
        assert plan is not None, "produce_part should produce a full plan"
        # Should include setup_machine decomposition + remaining primitives
        expected_start = ['load_tool', 'set_workholding', 'zero_axes', 'load_program']
        assert plan[:4] == expected_start
        assert 'load_material' in plan
        assert 'execute_program' in plan
        assert 'inspect_part' in plan
        assert 'unload_part' in plan

    def test_decomposition_with_missing_preconditions(self, manufacturing_planner):
        """Method with unmet preconditions should return None."""
        state = ['material_available']  # missing machine_idle
        plan = manufacturing_planner.plan('produce_part', state)
        assert plan is None


class TestStateProgression:
    """Tests for state changes through plan execution."""

    def test_effects_propagate_through_subtasks(self, manufacturing_planner):
        """Effects of earlier subtasks should be available for later ones."""
        state = [
            'material_available', 'machine_idle',
            'tool_available', 'machine_powered',
        ]
        plan = manufacturing_planner.plan('produce_part', state)
        assert plan is not None
        # execute_program requires program_loaded and tool_loaded,
        # which are effects of load_program and load_tool
        assert 'execute_program' in plan


class TestDepthLimit:
    """Tests for recursion depth limits."""

    def test_depth_limit_prevents_infinite_recursion(self):
        """Exceeding max_depth should return None, not stack overflow."""
        # Create a circular decomposition
        methods = [
            Method('m1', 'task_a', [], ['task_b']),
            Method('m2', 'task_b', [], ['task_a']),
        ]
        planner = _HTNPlanner(methods, {}, max_depth=5)
        plan = planner.plan('task_a', [])
        assert plan is None, "Circular decomposition should be caught by depth limit"

    def test_shallow_depth_limit(self):
        """A very shallow depth limit should prevent deep decomposition."""
        methods = [
            Method('m1', 'deep_task', [], ['level1']),
            Method('m2', 'level1', [], ['level2']),
            Method('m3', 'level2', [], ['level3']),
        ]
        primitives = {
            'level3': PrimitiveAction('level3', [], ['done'], 1.0),
        }
        planner = _HTNPlanner(methods, primitives, max_depth=2)
        plan = planner.plan('deep_task', [])
        assert plan is None, "Should fail due to shallow depth limit"
