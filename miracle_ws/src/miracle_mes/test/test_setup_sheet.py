"""Tests for SetupSheetGenerator.

Tests sheet creation, validation, revision comparison,
checklist generation, WCS support, and default safety notes.
"""

import os
import sys
import time
from types import ModuleType
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Module stubs (mirrors test_queue_optimizer.py pattern)
# ---------------------------------------------------------------------------

_MES_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
if _MES_SRC not in sys.path:
    sys.path.insert(0, _MES_SRC)

import miracle_mes  # noqa: E402, F401


def _ensure_module(dotted_name: str) -> ModuleType:
    parts = dotted_name.split('.')
    for i in range(1, len(parts) + 1):
        partial = '.'.join(parts[:i])
        if partial not in sys.modules:
            sys.modules[partial] = ModuleType(partial)
    return sys.modules[dotted_name]


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


class _StubMsg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattr__(self, name):
        return ''


_msgs = _ensure_module('miracle_msgs')
_msgs_msg = _ensure_module('miracle_msgs.msg')
_msgs_action = _ensure_module('miracle_msgs.action')
_msgs_srv = _ensure_module('miracle_msgs.srv')
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

_std_msgs = _ensure_module('std_msgs')
_std_msgs_msg = _ensure_module('std_msgs.msg')
_std_msgs_msg.String = _StubMsg

# miracle_msgs.msg needs specific message types used by digital_thread
_msgs_msg.DigitalThreadEntry = _StubMsg
_msgs_msg.EnergyProfile = _StubMsg
_msgs_msg.PredictionRecord = _StubMsg


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

from miracle_mes.job_scheduler import (  # noqa: E402
    SetupSheetGenerator,
    SetupSheet,
    ToolSetup,
    WorkholdingSpec,
    _DEFAULT_SAFETY_NOTES,
)


@pytest.fixture
def gen():
    return SetupSheetGenerator()


def _make_tools():
    return [
        ToolSetup(1, 'T01', '10mm End Mill', 10.0, length_offset=50.0, radius_comp=5.0),
        ToolSetup(2, 'T02', '6mm Drill', 6.0, length_offset=45.0),
    ]


def _make_wcs():
    return {
        'G54': {'X': 100.0, 'Y': 50.0, 'Z': 0.0},
        'G55': {'X': 200.0, 'Y': 50.0, 'Z': 0.0},
    }


def _make_workholding():
    return WorkholdingSpec(
        fixture_id='V-001', fixture_type='vise',
        clamping_force_n=5000.0, jaw_width_mm=150.0,
        parallels=True, soft_jaws=False,
    )


# ---- Creation ----

def test_create_sheet_with_all_fields(gen):
    sheet = gen.create_sheet(
        job_id='J-100', machine_id='MC-01', program_name='O1234',
        tools=_make_tools(), workholding=_make_workholding(),
        wcs_offsets=_make_wcs(), material='6061-T6',
        estimated_cycle_time=12.5,
    )
    assert sheet.job_id == 'J-100'
    assert sheet.revision == 1
    assert len(sheet.tools) == 2
    assert sheet.material == '6061-T6'
    assert sheet.workholding.fixture_id == 'V-001'
    assert 'G54' in sheet.wcs_offsets
    assert 'G55' in sheet.wcs_offsets


# ---- Validation ----

def test_validation_complete_sheet(gen):
    sheet = gen.create_sheet(
        'J-100', 'MC-01', 'O1234',
        tools=_make_tools(), workholding=_make_workholding(),
        wcs_offsets=_make_wcs(), material='6061-T6',
    )
    is_valid, issues = gen.validate(sheet)
    assert is_valid is True
    assert len(issues) == 0


def test_validation_missing_offset(gen):
    tools = [ToolSetup(1, 'T01', '10mm End Mill', 10.0)]  # no length_offset
    sheet = gen.create_sheet('J-100', 'MC-01', 'O1234', tools=tools)
    is_valid, issues = gen.validate(sheet)
    assert is_valid is False
    assert any('length offset' in i for i in issues)


def test_validation_incomplete_sheet(gen):
    """Sheet with no tools, no WCS, no material, no workholding."""
    sheet = gen.create_sheet('J-100', 'MC-01', 'O1234')
    is_valid, issues = gen.validate(sheet)
    assert is_valid is False
    assert len(issues) >= 4


# ---- Revision comparison ----

def test_revision_comparison(gen):
    gen.create_sheet(
        'J-100', 'MC-01', 'O1234',
        tools=[ToolSetup(1, 'T01', 'End Mill', 10.0, length_offset=50.0)],
        wcs_offsets={'G54': {'X': 100.0, 'Y': 50.0, 'Z': 0.0}},
        material='6061-T6',
    )
    gen.create_sheet(
        'J-100', 'MC-01', 'O1234',
        tools=[
            ToolSetup(1, 'T01', 'End Mill', 10.0, length_offset=50.0),
            ToolSetup(2, 'T03', 'New Drill', 8.0, length_offset=40.0),
        ],
        wcs_offsets={'G54': {'X': 105.0, 'Y': 50.0, 'Z': 0.0}},  # changed
        material='7075-T6',  # changed
    )
    changes = gen.compare_revisions('J-100', 1, 2)
    assert 'Tool T03' in changes['added']
    assert any('G54' in c for c in changes['changed'])
    assert any('Material' in c for c in changes['changed'])


# ---- Checklist ----

def test_checklist_generation(gen):
    sheet = gen.create_sheet(
        'J-100', 'MC-01', 'O1234',
        tools=_make_tools(), workholding=_make_workholding(),
        wcs_offsets=_make_wcs(), material='6061-T6',
    )
    checklist = gen.generate_checklist(sheet)
    assert any('Load tool T01' in item for item in checklist)
    assert any('length offset' in item.lower() for item in checklist)
    assert any('G54' in item for item in checklist)
    assert any('Install' in item for item in checklist)
    assert any('parallels' in item.lower() for item in checklist)
    assert any('6061-T6' in item for item in checklist)
    assert any('O1234' in item for item in checklist)
    assert any('[SAFETY]' in item for item in checklist)


# ---- Multiple WCS ----

def test_multiple_wcs_support(gen):
    wcs = {
        'G54': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
        'G55': {'X': 100.0, 'Y': 0.0, 'Z': 0.0},
        'G56': {'X': 200.0, 'Y': 0.0, 'Z': 0.0},
        'G57': {'X': 300.0, 'Y': 0.0, 'Z': 0.0},
        'G58': {'X': 400.0, 'Y': 0.0, 'Z': 0.0},
        'G59': {'X': 500.0, 'Y': 0.0, 'Z': 0.0},
    }
    sheet = gen.create_sheet('J-200', 'MC-02', 'O5678', wcs_offsets=wcs)
    assert len(sheet.wcs_offsets) == 6


# ---- Default safety notes ----

def test_default_safety_notes(gen):
    sheet = gen.create_sheet('J-100', 'MC-01', 'O1234')
    assert len(sheet.safety_notes) == len(_DEFAULT_SAFETY_NOTES)
    assert 'safety glasses' in sheet.safety_notes[3].lower()


def test_custom_safety_notes(gen):
    custom = ['Custom note 1', 'Custom note 2']
    sheet = gen.create_sheet('J-100', 'MC-01', 'O1234', safety_notes=custom)
    assert sheet.safety_notes == custom


# ---- Revision history ----

def test_revision_history(gen):
    gen.create_sheet('J-100', 'MC-01', 'O1234')
    gen.create_sheet('J-100', 'MC-01', 'O1234-v2')
    gen.create_sheet('J-100', 'MC-01', 'O1234-v3')
    history = gen.get_revision_history('J-100')
    assert len(history) == 3
    assert history[0].revision == 1
    assert history[2].revision == 3
    # get_sheet returns latest by default
    latest = gen.get_sheet('J-100')
    assert latest.revision == 3
