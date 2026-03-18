"""Tests for OperatorSkillMatrix, OperatorProfile, and SkillAssessment.

Validates operator skill tracking including:
- Operator registration and retrieval
- Skill assessment recording and level updates
- Finding qualified operators for machines
- Skill gap identification
- Team shift coverage analysis
- Training recommendations
- Team-wide skill summary / distribution
- Edge cases (unknown operators, empty data, etc.)
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
    OperatorProfile,
    OperatorSkillMatrix,
    SkillAssessment,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_operator(
    operator_id: str = 'op1',
    name: str = 'Alice',
    skills: Optional[Dict[str, int]] = None,
    certifications: Optional[List[str]] = None,
    qualified_machines: Optional[List[str]] = None,
    shift: str = 'day',
    hire_date: float = 0.0,
    total_hours: float = 1000.0,
) -> OperatorProfile:
    return OperatorProfile(
        operator_id=operator_id,
        name=name,
        skills=skills if skills is not None else {},
        certifications=certifications or [],
        qualified_machines=qualified_machines or [],
        shift=shift,
        hire_date=hire_date,
        total_hours=total_hours,
    )


def _populated_matrix() -> OperatorSkillMatrix:
    """Return a matrix with three operators pre-registered."""
    matrix = OperatorSkillMatrix()
    matrix.register_operator(_make_operator(
        'op1', 'Alice',
        skills={'milling': 5, 'turning': 3, 'grinding': 2},
        certifications=['ISO9001', 'CNC_LEVEL3'],
        qualified_machines=['cnc1', 'cnc2'],
        shift='day',
    ))
    matrix.register_operator(_make_operator(
        'op2', 'Bob',
        skills={'milling': 3, 'turning': 5, 'inspection': 4},
        certifications=['ISO9001'],
        qualified_machines=['cnc2', 'cnc3'],
        shift='day',
    ))
    matrix.register_operator(_make_operator(
        'op3', 'Carol',
        skills={'milling': 4, 'grinding': 5, 'inspection': 3},
        certifications=['ISO9001', 'CNC_LEVEL3'],
        qualified_machines=['cnc1', 'cnc3'],
        shift='night',
    ))
    return matrix


# ===================================================================
# Tests: Operator Registration and Retrieval
# ===================================================================

class TestOperatorRegistration:
    def test_register_and_get(self):
        matrix = OperatorSkillMatrix()
        op = _make_operator('op1', 'Alice')
        matrix.register_operator(op)
        result = matrix.get_operator('op1')
        assert result is not None
        assert result.name == 'Alice'

    def test_get_unknown_returns_none(self):
        matrix = OperatorSkillMatrix()
        assert matrix.get_operator('nonexistent') is None

    def test_get_all_operators(self):
        matrix = _populated_matrix()
        ops = matrix.get_all_operators()
        assert len(ops) == 3
        ids = {op.operator_id for op in ops}
        assert ids == {'op1', 'op2', 'op3'}

    def test_register_overwrites_existing(self):
        matrix = OperatorSkillMatrix()
        matrix.register_operator(_make_operator('op1', 'Alice'))
        matrix.register_operator(_make_operator('op1', 'Alice Updated', shift='night'))
        op = matrix.get_operator('op1')
        assert op.name == 'Alice Updated'
        assert op.shift == 'night'
        assert len(matrix.get_all_operators()) == 1


# ===================================================================
# Tests: Skill Assessment
# ===================================================================

class TestSkillAssessment:
    def test_record_assessment_updates_skill(self):
        matrix = OperatorSkillMatrix()
        matrix.register_operator(_make_operator('op1', 'Alice', skills={'milling': 2}))
        assessment = SkillAssessment(
            operator_id='op1',
            skill_name='milling',
            score=4,
            assessed_by='supervisor',
            timestamp=time.time(),
            notes='Improved technique',
        )
        matrix.record_assessment(assessment)
        op = matrix.get_operator('op1')
        assert op.skills['milling'] == 4

    def test_assessment_adds_new_skill(self):
        matrix = OperatorSkillMatrix()
        matrix.register_operator(_make_operator('op1', 'Alice', skills={}))
        matrix.record_assessment(SkillAssessment(
            operator_id='op1', skill_name='welding', score=3,
            assessed_by='trainer', timestamp=time.time(),
        ))
        assert matrix.get_operator('op1').skills['welding'] == 3

    def test_assessment_clamps_score(self):
        matrix = OperatorSkillMatrix()
        matrix.register_operator(_make_operator('op1', 'Alice'))
        # Score above 5 should be clamped to 5
        matrix.record_assessment(SkillAssessment(
            operator_id='op1', skill_name='milling', score=7,
            assessed_by='test', timestamp=time.time(),
        ))
        assert matrix.get_operator('op1').skills['milling'] == 5
        # Score below 1 should be clamped to 1
        matrix.record_assessment(SkillAssessment(
            operator_id='op1', skill_name='milling', score=-1,
            assessed_by='test', timestamp=time.time(),
        ))
        assert matrix.get_operator('op1').skills['milling'] == 1

    def test_assessment_for_unknown_operator_does_not_crash(self):
        matrix = OperatorSkillMatrix()
        # Should not raise, just record the assessment
        matrix.record_assessment(SkillAssessment(
            operator_id='ghost', skill_name='milling', score=3,
            assessed_by='test', timestamp=time.time(),
        ))


# ===================================================================
# Tests: Find Qualified Operators
# ===================================================================

class TestFindQualifiedOperators:
    def test_find_by_machine_only(self):
        matrix = _populated_matrix()
        result = matrix.find_qualified_operators('cnc1')
        ids = {op.operator_id for op in result}
        assert ids == {'op1', 'op3'}

    def test_find_by_machine_and_skills(self):
        matrix = _populated_matrix()
        result = matrix.find_qualified_operators('cnc2', {'milling': 4})
        ids = {op.operator_id for op in result}
        # op1 has milling=5 and is qualified on cnc2
        # op2 has milling=3 (below 4) on cnc2
        assert ids == {'op1'}

    def test_no_qualified_operators(self):
        matrix = _populated_matrix()
        result = matrix.find_qualified_operators('cnc99')
        assert result == []

    def test_empty_required_skills(self):
        matrix = _populated_matrix()
        result = matrix.find_qualified_operators('cnc3', {})
        ids = {op.operator_id for op in result}
        assert ids == {'op2', 'op3'}


# ===================================================================
# Tests: Skill Gaps
# ===================================================================

class TestSkillGaps:
    def test_identifies_gaps(self):
        matrix = _populated_matrix()
        gaps = matrix.get_skill_gaps('op1', {'milling': 5, 'turning': 4, 'inspection': 3})
        # milling=5 meets 5, turning=3 < 4, inspection missing (0 < 3)
        assert 'milling' not in gaps
        assert gaps['turning'] == (3, 4)
        assert gaps['inspection'] == (0, 3)

    def test_no_gaps(self):
        matrix = _populated_matrix()
        gaps = matrix.get_skill_gaps('op1', {'milling': 3})
        assert gaps == {}

    def test_unknown_operator_all_gaps(self):
        matrix = _populated_matrix()
        gaps = matrix.get_skill_gaps('ghost', {'milling': 3, 'turning': 2})
        assert gaps == {'milling': (0, 3), 'turning': (0, 2)}


# ===================================================================
# Tests: Team Coverage
# ===================================================================

class TestTeamCoverage:
    def test_day_shift_coverage(self):
        matrix = _populated_matrix()
        coverage = matrix.get_team_coverage(['cnc1', 'cnc2', 'cnc3'], 'day')
        assert 'op1' in coverage['cnc1']
        assert 'op1' in coverage['cnc2']
        assert 'op2' in coverage['cnc2']
        assert 'op2' in coverage['cnc3']
        # op3 is night shift, should not appear
        for machine_ops in coverage.values():
            assert 'op3' not in machine_ops

    def test_night_shift_coverage(self):
        matrix = _populated_matrix()
        coverage = matrix.get_team_coverage(['cnc1', 'cnc2', 'cnc3'], 'night')
        assert 'op3' in coverage['cnc1']
        assert coverage['cnc2'] == []  # no night operator on cnc2
        assert 'op3' in coverage['cnc3']

    def test_uncovered_machine(self):
        matrix = _populated_matrix()
        coverage = matrix.get_team_coverage(['cnc99'], 'day')
        assert coverage['cnc99'] == []


# ===================================================================
# Tests: Training Recommendations
# ===================================================================

class TestRecommendTraining:
    def test_recommends_low_skills(self):
        matrix = _populated_matrix()
        recs = matrix.recommend_training('op1')
        skill_names = [r[0] for r in recs]
        # op1 has grinding=2 (below 3) and team max is 5
        # op1 has no inspection (0, below 3) and team max is 4
        assert 'grinding' in skill_names
        assert 'inspection' in skill_names

    def test_no_recommendations_for_expert(self):
        matrix = OperatorSkillMatrix()
        matrix.register_operator(_make_operator(
            'expert', 'Expert',
            skills={'milling': 5, 'turning': 5, 'grinding': 5},
        ))
        recs = matrix.recommend_training('expert')
        assert recs == []

    def test_unknown_operator_empty(self):
        matrix = _populated_matrix()
        recs = matrix.recommend_training('ghost')
        assert recs == []

    def test_recommendations_sorted_by_gap(self):
        matrix = _populated_matrix()
        recs = matrix.recommend_training('op1')
        # Verify descending gap ordering
        for i in range(len(recs) - 1):
            gap_a = recs[i][2] - recs[i][1]
            gap_b = recs[i + 1][2] - recs[i + 1][1]
            assert gap_a >= gap_b


# ===================================================================
# Tests: Skill Summary
# ===================================================================

class TestSkillSummary:
    def test_summary_structure(self):
        matrix = _populated_matrix()
        summary = matrix.get_skill_summary()
        assert 'milling' in summary
        assert 'turning' in summary
        milling = summary['milling']
        assert 'avg' in milling
        assert 'min' in milling
        assert 'max' in milling
        assert 'count' in milling
        assert 'operators' in milling

    def test_summary_values(self):
        matrix = _populated_matrix()
        summary = matrix.get_skill_summary()
        milling = summary['milling']
        # op1=5, op2=3, op3=4 -> avg=4.0, min=3, max=5, count=3
        assert milling['avg'] == 4.0
        assert milling['min'] == 3
        assert milling['max'] == 5
        assert milling['count'] == 3
        assert set(milling['operators']) == {'op1', 'op2', 'op3'}

    def test_empty_matrix_summary(self):
        matrix = OperatorSkillMatrix()
        summary = matrix.get_skill_summary()
        assert summary == {}
