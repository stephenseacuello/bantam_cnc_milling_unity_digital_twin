"""Tests for Behavior Tree node types.

Tests the four core BT node types: Sequence, Selector, Condition, and
Action. Validates their tick behavior, status propagation, and
composition patterns.
"""

from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, List, Callable

import pytest


class BTStatus(Enum):
    """Behavior tree tick status."""
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


# --- Helper fixtures ---

def _success_action(bb):
    return BTStatus.SUCCESS

def _failure_action(bb):
    return BTStatus.FAILURE

def _running_action(bb):
    return BTStatus.RUNNING


class TestSequenceNode:
    """Tests for the Sequence composite node."""

    def test_all_success_returns_success(self):
        """Sequence with all succeeding children should return SUCCESS."""
        seq = SequenceNode("seq", [
            ActionNode("a1", _success_action),
            ActionNode("a2", _success_action),
            ActionNode("a3", _success_action),
        ])
        assert seq.tick({}) == BTStatus.SUCCESS

    def test_first_failure_returns_failure(self):
        """Sequence should return FAILURE as soon as one child fails."""
        seq = SequenceNode("seq", [
            ActionNode("a1", _success_action),
            ActionNode("a2", _failure_action),
            ActionNode("a3", _success_action),  # should not be reached
        ])
        assert seq.tick({}) == BTStatus.FAILURE

    def test_running_child_returns_running(self):
        """Sequence should return RUNNING when a child is still running."""
        seq = SequenceNode("seq", [
            ActionNode("a1", _success_action),
            ActionNode("a2", _running_action),
        ])
        assert seq.tick({}) == BTStatus.RUNNING

    def test_empty_sequence_returns_success(self):
        """An empty sequence should return SUCCESS immediately."""
        seq = SequenceNode("empty_seq", [])
        assert seq.tick({}) == BTStatus.SUCCESS

    def test_sequence_resets_on_failure(self):
        """After failure, the sequence index should reset so it restarts."""
        call_count = [0]

        def counting_action(bb):
            call_count[0] += 1
            return BTStatus.SUCCESS

        seq = SequenceNode("seq", [
            ActionNode("a1", counting_action),
            ActionNode("a2", _failure_action),
        ])

        seq.tick({})  # a1 succeeds, a2 fails
        seq.tick({})  # should restart from a1
        assert call_count[0] == 2, "a1 should execute twice (reset after failure)"


class TestSelectorNode:
    """Tests for the Selector (fallback) composite node."""

    def test_first_success_returns_success(self):
        """Selector should return SUCCESS as soon as one child succeeds."""
        sel = SelectorNode("sel", [
            ActionNode("a1", _failure_action),
            ActionNode("a2", _success_action),
            ActionNode("a3", _failure_action),  # should not be reached
        ])
        assert sel.tick({}) == BTStatus.SUCCESS

    def test_all_failure_returns_failure(self):
        """Selector with all failing children should return FAILURE."""
        sel = SelectorNode("sel", [
            ActionNode("a1", _failure_action),
            ActionNode("a2", _failure_action),
        ])
        assert sel.tick({}) == BTStatus.FAILURE

    def test_running_child_returns_running(self):
        """Selector should return RUNNING when a child is still running."""
        sel = SelectorNode("sel", [
            ActionNode("a1", _failure_action),
            ActionNode("a2", _running_action),
        ])
        assert sel.tick({}) == BTStatus.RUNNING

    def test_empty_selector_returns_failure(self):
        """An empty selector should return FAILURE."""
        sel = SelectorNode("empty_sel", [])
        assert sel.tick({}) == BTStatus.FAILURE

    def test_first_child_success_short_circuits(self):
        """If the first child succeeds, remaining children should not execute."""
        reached = [False]

        def tracking_action(bb):
            reached[0] = True
            return BTStatus.SUCCESS

        sel = SelectorNode("sel", [
            ActionNode("a1", _success_action),
            ActionNode("a2", tracking_action),
        ])
        sel.tick({})
        assert reached[0] is False, "Second child should not be reached"


class TestConditionNode:
    """Tests for the Condition node."""

    def test_true_condition_returns_success(self):
        """A truthy condition check should return SUCCESS."""
        cond = ConditionNode("ready", lambda bb: bb.get('ready', False))
        assert cond.tick({'ready': True}) == BTStatus.SUCCESS

    def test_false_condition_returns_failure(self):
        """A falsy condition check should return FAILURE."""
        cond = ConditionNode("ready", lambda bb: bb.get('ready', False))
        assert cond.tick({'ready': False}) == BTStatus.FAILURE

    def test_missing_key_returns_failure(self):
        """A missing blackboard key should return FAILURE (default is False)."""
        cond = ConditionNode("ready", lambda bb: bb.get('ready', False))
        assert cond.tick({}) == BTStatus.FAILURE

    def test_condition_reads_blackboard(self):
        """Conditions should read from the blackboard dictionary."""
        cond = ConditionNode("temp_check", lambda bb: bb.get('temp', 0) < 100)
        assert cond.tick({'temp': 50}) == BTStatus.SUCCESS
        assert cond.tick({'temp': 150}) == BTStatus.FAILURE


class TestActionNode:
    """Tests for the Action node."""

    def test_action_returns_success(self):
        """An action that completes should return SUCCESS."""
        action = ActionNode("do_thing", _success_action)
        assert action.tick({}) == BTStatus.SUCCESS

    def test_action_returns_failure(self):
        """An action that fails should return FAILURE."""
        action = ActionNode("fail_thing", _failure_action)
        assert action.tick({}) == BTStatus.FAILURE

    def test_action_returns_running(self):
        """An in-progress action should return RUNNING."""
        action = ActionNode("long_thing", _running_action)
        assert action.tick({}) == BTStatus.RUNNING

    def test_action_modifies_blackboard(self):
        """Actions should be able to modify the blackboard."""
        def set_value(bb):
            bb['result'] = 42
            return BTStatus.SUCCESS

        bb = {}
        action = ActionNode("set_val", set_value)
        action.tick(bb)
        assert bb['result'] == 42


class TestComposition:
    """Tests for composing BT nodes into complex trees."""

    def test_selector_with_sequence_fallback(self):
        """A selector containing sequences should implement fallback behavior."""
        tree = SelectorNode("root", [
            SequenceNode("primary", [
                ConditionNode("check", lambda bb: bb.get('primary_ok', False)),
                ActionNode("primary_action", _success_action),
            ]),
            SequenceNode("fallback", [
                ActionNode("fallback_action", _success_action),
            ]),
        ])

        # Primary path fails (condition false), fallback succeeds
        result = tree.tick({'primary_ok': False})
        assert result == BTStatus.SUCCESS

    def test_nested_sequence_in_selector(self):
        """Nested composition: primary path succeeds when condition is true."""
        tree = SelectorNode("root", [
            SequenceNode("primary", [
                ConditionNode("check", lambda bb: bb.get('primary_ok', False)),
                ActionNode("primary_action", _success_action),
            ]),
            SequenceNode("fallback", [
                ActionNode("fallback_action", _success_action),
            ]),
        ])

        result = tree.tick({'primary_ok': True})
        assert result == BTStatus.SUCCESS

    def test_deep_nesting(self):
        """Deeply nested trees should propagate status correctly."""
        leaf = ActionNode("leaf", _success_action)
        inner = SequenceNode("inner", [leaf])
        middle = SelectorNode("middle", [inner])
        outer = SequenceNode("outer", [middle])
        assert outer.tick({}) == BTStatus.SUCCESS
