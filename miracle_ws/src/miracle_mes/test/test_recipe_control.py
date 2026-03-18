"""Tests for RecipeVersionControl, RecipeParameter, Recipe, and RecipeDiff.

Validates manufacturing recipe version control including:
- Creating a new recipe (version 1)
- Updating a recipe (version increments)
- Approving a recipe version
- Retrieving specific / latest / latest-approved versions
- Diffing two recipe versions
- Parameter validation (min/max bounds)
- Error handling for missing recipes and duplicate creation
"""

import os
import sys
import time
from types import ModuleType
from typing import Any, Dict, List, Optional, Set
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Ensure the miracle_mes source directory is on sys.path
# ---------------------------------------------------------------------------
_MES_SRC = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir)
)
if _MES_SRC not in sys.path:
    sys.path.insert(0, _MES_SRC)


# ---------------------------------------------------------------------------
# Stub modules (ROS2 / miracle_core / miracle_msgs)
# ---------------------------------------------------------------------------

# Pre-import miracle_mes so it is the real package, not a stub module.
import miracle_mes  # noqa: E402, F401


def _ensure_module(dotted_name: str) -> ModuleType:
    parts = dotted_name.split('.')
    for i in range(1, len(parts) + 1):
        partial = '.'.join(parts[:i])
        if partial not in sys.modules:
            sys.modules[partial] = ModuleType(partial)
    return sys.modules[dotted_name]


# rclpy stubs
_rclpy = _ensure_module('rclpy')
_rclpy_lifecycle = _ensure_module('rclpy.lifecycle')
_rclpy_lifecycle.TransitionCallbackReturn = MagicMock()
_rclpy_lifecycle.TransitionCallbackReturn.SUCCESS = 'SUCCESS'
_rclpy_action = _ensure_module('rclpy.action')
_rclpy_action.ActionServer = MagicMock()
_rclpy_action.CancelResponse = MagicMock()
_rclpy_action.GoalResponse = MagicMock()
_rclpy_action_server = _ensure_module('rclpy.action.server')
_rclpy_action_server.ServerGoalHandle = MagicMock()
_rclpy_cbg = _ensure_module('rclpy.callback_groups')
_rclpy_cbg.ReentrantCallbackGroup = MagicMock()

# miracle_core stubs -- force-set attributes on existing modules
_mc = _ensure_module('miracle_core')
_mc_lifecycle = _ensure_module('miracle_core.lifecycle_node_base')
_mc_qos = _ensure_module('miracle_core.qos_profiles')


class _StubLifecycleNode:
    CRITICALITY_HIGH = 'HIGH'

    def __init__(self, *a, **kw):
        self.service_callback_group = MagicMock()
        self._logger = MagicMock()

    def get_logger(self):
        return self._logger

    def declare_and_validate_parameters(self, spec):
        return {k: v['default'] for k, v in spec.items()}

    def get_machine_ids(self, params):
        return params.get('machine_ids', 'cnc1,cnc2,cnc3').split(',')

    def create_subscription(self, *a, **kw):
        return MagicMock()

    def create_multi_machine_subscriptions(self, *a, **kw):
        return MagicMock()

    def create_publisher(self, *a, **kw):
        return MagicMock()

    def create_service(self, *a, **kw):
        return MagicMock()

    def create_timer(self, *a, **kw):
        return MagicMock()

    def get_parameter(self, name):
        m = MagicMock()
        m.value = 1.0
        return m

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.nanoseconds = int(time.time() * 1e9)
        clock.now.return_value.to_msg.return_value = MagicMock()
        return clock


_mc_lifecycle.MiracleLifecycleNode = _StubLifecycleNode

_mc_qos.QoSProfiles = MagicMock()
_mc_qos.QoSProfiles.alert.return_value = MagicMock()
_mc_qos.QoSProfiles.state_data.return_value = MagicMock()
_mc_qos.QoSProfiles.command.return_value = MagicMock()

# miracle_msgs stubs
_msgs = _ensure_module('miracle_msgs')
_msgs_msg = _ensure_module('miracle_msgs.msg')
_msgs_action = _ensure_module('miracle_msgs.action')
_msgs_srv = _ensure_module('miracle_msgs.srv')


class _StubMsg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattr__(self, name):
        return ''


_msgs_msg.JobStatus = _StubMsg
_msgs_msg.MachineState = _StubMsg
_msgs_msg.TaskAnnouncement = _StubMsg
_msgs_msg.AlarmEscalation = _StubMsg

_exec_job = MagicMock()
_exec_job.Feedback = _StubMsg
_exec_job.Result = _StubMsg
_msgs_action.ExecuteJob = _exec_job

_submit_task = MagicMock()
_submit_task.Request = _StubMsg
_submit_task.Response = _StubMsg
_msgs_srv.SubmitTask = _submit_task


class _StubDigitalThread:
    ENTRY_MATERIAL_LOADED = 'MATERIAL_LOADED'
    ENTRY_PART_COMPLETE = 'PART_COMPLETE'
    ENTRY_TOOL_INSTALLED = 'TOOL_INSTALLED'
    ENTRY_TOOL_REMOVED = 'TOOL_REMOVED'
    ENTRY_JOB_PAUSED = 'JOB_PAUSED'
    ENTRY_JOB_RESUMED = 'JOB_RESUMED'
    ENTRY_JOB_CANCELLED = 'JOB_CANCELLED'
    ENTRY_JOB_FAILED = 'JOB_FAILED'
    ENTRY_MACHINE_ERROR = 'MACHINE_ERROR'

    def __init__(self):
        self._entries = []

    def record_genealogy_event(self, *a, **kw):
        pass

    def set_active_job(self, **kw):
        pass

    def clear_active_job(self, *a):
        pass

    def record_operation_complete(self, *a, **kw):
        pass


_dt_mod = _ensure_module('miracle_mes.digital_thread')
_dt_mod.DigitalThreadNode = _StubDigitalThread

# ---------------------------------------------------------------------------
# Now import the production code
# ---------------------------------------------------------------------------
from miracle_mes.job_scheduler import (  # noqa: E402
    RecipeDiff,
    RecipeParameter,
    Recipe,
    RecipeVersionControl,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _param(name: str, value: float, unit: str = 'mm',
           min_value: float = 0.0, max_value: float = 100.0,
           is_critical: bool = False) -> RecipeParameter:
    return RecipeParameter(
        name=name,
        value=value,
        unit=unit,
        min_value=min_value,
        max_value=max_value,
        is_critical=is_critical,
    )


def _default_params() -> List[RecipeParameter]:
    return [
        _param('spindle_speed', 12000.0, 'rpm', 1000.0, 24000.0, True),
        _param('feed_rate', 500.0, 'mm/min', 50.0, 2000.0, True),
        _param('depth_of_cut', 2.0, 'mm', 0.1, 10.0),
    ]


# ===================================================================
# Tests: Create Recipe
# ===================================================================

class TestCreateRecipe:
    def setup_method(self):
        self.rvc = RecipeVersionControl()

    def test_create_recipe_returns_version_one(self):
        recipe = self.rvc.create_recipe(
            'R001', 'PART-100', _default_params(), 'engineer_a', 'initial',
        )
        assert recipe.version == 1
        assert recipe.recipe_id == 'R001'
        assert recipe.part_number == 'PART-100'
        assert recipe.created_by == 'engineer_a'
        assert recipe.notes == 'initial'
        assert recipe.approved is False
        assert len(recipe.parameters) == 3

    def test_create_duplicate_recipe_raises(self):
        self.rvc.create_recipe('R001', 'PART-100', _default_params(), 'eng')
        with pytest.raises(ValueError, match='already exists'):
            self.rvc.create_recipe('R001', 'PART-200', [], 'eng')


# ===================================================================
# Tests: Update Recipe
# ===================================================================

class TestUpdateRecipe:
    def setup_method(self):
        self.rvc = RecipeVersionControl()
        self.rvc.create_recipe('R001', 'PART-100', _default_params(), 'eng')

    def test_update_increments_version(self):
        new_params = [_param('spindle_speed', 15000.0, 'rpm', 1000.0, 24000.0)]
        updated = self.rvc.update_recipe('R001', new_params, 'eng_b', 'tuned')
        assert updated.version == 2
        assert updated.created_by == 'eng_b'
        assert updated.notes == 'tuned'
        assert len(updated.parameters) == 1

    def test_update_nonexistent_recipe_raises(self):
        with pytest.raises(KeyError, match='does not exist'):
            self.rvc.update_recipe('NOPE', [], 'eng')

    def test_multiple_updates_increment_correctly(self):
        for i in range(5):
            self.rvc.update_recipe('R001', _default_params(), f'eng_{i}')
        latest = self.rvc.get_recipe('R001')
        assert latest.version == 6  # 1 (create) + 5 updates


# ===================================================================
# Tests: Approve Recipe
# ===================================================================

class TestApproveRecipe:
    def setup_method(self):
        self.rvc = RecipeVersionControl()
        self.rvc.create_recipe('R001', 'PART-100', _default_params(), 'eng')

    def test_approve_marks_recipe(self):
        recipe = self.rvc.approve_recipe('R001', 1, 'qa_lead')
        assert recipe.approved is True
        assert recipe.approved_by == 'qa_lead'

    def test_approve_nonexistent_version_raises(self):
        with pytest.raises(KeyError, match='Version 99 not found'):
            self.rvc.approve_recipe('R001', 99, 'qa_lead')


# ===================================================================
# Tests: Get Recipe
# ===================================================================

class TestGetRecipe:
    def setup_method(self):
        self.rvc = RecipeVersionControl()
        self.rvc.create_recipe('R001', 'PART-100', _default_params(), 'eng')
        self.rvc.update_recipe('R001', _default_params(), 'eng')

    def test_get_specific_version(self):
        recipe = self.rvc.get_recipe('R001', version=1)
        assert recipe.version == 1

    def test_get_latest_version_when_none(self):
        recipe = self.rvc.get_recipe('R001')
        assert recipe.version == 2

    def test_get_nonexistent_recipe_raises(self):
        with pytest.raises(KeyError, match='does not exist'):
            self.rvc.get_recipe('NOPE')


# ===================================================================
# Tests: Get Approved Recipe
# ===================================================================

class TestGetApprovedRecipe:
    def setup_method(self):
        self.rvc = RecipeVersionControl()
        self.rvc.create_recipe('R001', 'PART-100', _default_params(), 'eng')
        self.rvc.update_recipe('R001', _default_params(), 'eng')
        self.rvc.update_recipe('R001', _default_params(), 'eng')

    def test_returns_none_when_nothing_approved(self):
        result = self.rvc.get_approved_recipe('R001')
        assert result is None

    def test_returns_latest_approved(self):
        self.rvc.approve_recipe('R001', 1, 'qa')
        self.rvc.approve_recipe('R001', 2, 'qa')
        result = self.rvc.get_approved_recipe('R001')
        assert result is not None
        assert result.version == 2

    def test_skips_unapproved_later_version(self):
        self.rvc.approve_recipe('R001', 1, 'qa')
        # version 2 and 3 are NOT approved
        result = self.rvc.get_approved_recipe('R001')
        assert result is not None
        assert result.version == 1

    def test_nonexistent_recipe_raises(self):
        with pytest.raises(KeyError, match='does not exist'):
            self.rvc.get_approved_recipe('NOPE')


# ===================================================================
# Tests: Diff Versions
# ===================================================================

class TestDiffVersions:
    def setup_method(self):
        self.rvc = RecipeVersionControl()
        params_v1 = [
            _param('spindle_speed', 12000.0, 'rpm', 1000.0, 24000.0),
            _param('feed_rate', 500.0, 'mm/min', 50.0, 2000.0),
            _param('depth_of_cut', 2.0, 'mm', 0.1, 10.0),
        ]
        self.rvc.create_recipe('R001', 'PART-100', params_v1, 'eng')
        params_v2 = [
            _param('spindle_speed', 15000.0, 'rpm', 1000.0, 24000.0),  # changed
            _param('feed_rate', 500.0, 'mm/min', 50.0, 2000.0),        # same
            _param('coolant_pressure', 5.0, 'bar', 1.0, 10.0),         # added
            # depth_of_cut removed
        ]
        self.rvc.update_recipe('R001', params_v2, 'eng')

    def test_diff_added(self):
        diff = self.rvc.diff_versions('R001', 1, 2)
        assert 'coolant_pressure' in diff.added

    def test_diff_removed(self):
        diff = self.rvc.diff_versions('R001', 1, 2)
        assert 'depth_of_cut' in diff.removed

    def test_diff_changed(self):
        diff = self.rvc.diff_versions('R001', 1, 2)
        assert 'spindle_speed' in diff.changed
        assert diff.changed['spindle_speed'] == (12000.0, 15000.0)

    def test_diff_unchanged_not_in_changed(self):
        diff = self.rvc.diff_versions('R001', 1, 2)
        assert 'feed_rate' not in diff.changed

    def test_diff_metadata(self):
        diff = self.rvc.diff_versions('R001', 1, 2)
        assert diff.recipe_id == 'R001'
        assert diff.version_a == 1
        assert diff.version_b == 2


# ===================================================================
# Tests: Validate Parameters
# ===================================================================

class TestValidateParameters:
    def test_all_valid(self):
        params = _default_params()
        errors = RecipeVersionControl.validate_parameters(params)
        assert errors == []

    def test_below_minimum(self):
        params = [_param('spindle_speed', 500.0, 'rpm', 1000.0, 24000.0)]
        errors = RecipeVersionControl.validate_parameters(params)
        assert len(errors) == 1
        assert 'below minimum' in errors[0]

    def test_above_maximum(self):
        params = [_param('spindle_speed', 30000.0, 'rpm', 1000.0, 24000.0)]
        errors = RecipeVersionControl.validate_parameters(params)
        assert len(errors) == 1
        assert 'above maximum' in errors[0]

    def test_multiple_violations(self):
        params = [
            _param('spindle_speed', 500.0, 'rpm', 1000.0, 24000.0),
            _param('feed_rate', 3000.0, 'mm/min', 50.0, 2000.0),
        ]
        errors = RecipeVersionControl.validate_parameters(params)
        assert len(errors) == 2

    def test_boundary_values_are_valid(self):
        params = [
            _param('spindle_speed', 1000.0, 'rpm', 1000.0, 24000.0),
            _param('feed_rate', 2000.0, 'mm/min', 50.0, 2000.0),
        ]
        errors = RecipeVersionControl.validate_parameters(params)
        assert errors == []
