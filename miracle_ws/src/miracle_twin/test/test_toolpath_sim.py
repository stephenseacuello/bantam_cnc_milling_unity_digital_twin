"""Tests for ToolPathSimulator — G-code tool path execution and tracking."""

import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import math
import pytest
from miracle_twin.cutting_sim_proxy import (
    ToolPathSimulator,
    ToolPosition,
    SimulationState,
    SimulationSummary,
)


@pytest.fixture
def sim():
    return ToolPathSimulator()


# ---- Basic motion ----

def test_rapid_move_updates_position(sim):
    """G0 rapid move should update the current position."""
    sim.execute_block('G0 X10 Y20 Z-5')
    state = sim.get_state()
    assert state.current_position == (10.0, 20.0, -5.0)


def test_linear_move_updates_position(sim):
    """G1 linear move should update the current position."""
    sim.execute_block('G1 X50 Y0 Z0 F500')
    state = sim.get_state()
    assert state.current_position == (50.0, 0.0, 0.0)


def test_rapid_distance_tracked(sim):
    """G0 moves accumulate rapid distance, not cutting distance."""
    sim.execute_block('G0 X100')
    state = sim.get_state()
    assert state.total_distance_mm == pytest.approx(100.0, abs=0.01)
    history = sim.get_position_history()
    # First record is initial (0,0,0), second is after move
    assert len(history) == 2
    assert history[1].is_rapid is True


def test_cutting_distance_tracked(sim):
    """G1 moves accumulate cutting distance."""
    sim.execute_block('G1 X30 F600')
    summary = sim.execute_program(['G1 X30 F600'])
    assert summary.cutting_distance_mm == pytest.approx(30.0, abs=0.01)
    assert summary.rapid_distance_mm == pytest.approx(0.0, abs=0.01)


# ---- Feed rate and time ----

def test_feed_rate_affects_time(sim):
    """Moving at 600 mm/min for 60 mm should take 6 seconds."""
    sim.execute_block('G1 X60 F600')
    state = sim.get_state()
    expected_sec = (60.0 / 600.0) * 60.0  # 6.0 seconds
    assert state.elapsed_time_sec == pytest.approx(expected_sec, abs=0.01)


def test_max_feed_tracked(sim):
    """The highest F value used should be recorded."""
    summary = sim.execute_program([
        'G1 X10 F200',
        'G1 X20 F800',
        'G1 X30 F400',
    ])
    assert summary.max_feed_used == pytest.approx(800.0)


# ---- Spindle and coolant M-codes ----

def test_spindle_on_off(sim):
    """M3 turns spindle on, M5 turns it off."""
    sim.execute_block('S12000 M3')
    state = sim.get_state()
    assert state.spindle_rpm == 12000.0

    sim.execute_block('M5')
    state = sim.get_state()
    assert state.spindle_rpm == 0.0


def test_coolant_on_off(sim):
    """M8 turns coolant on, M9 turns it off."""
    sim.execute_block('M8')
    assert sim.get_state().coolant_on is True
    sim.execute_block('M9')
    assert sim.get_state().coolant_on is False


# ---- Tool change ----

def test_tool_change(sim):
    """T and M6 should change the active tool and record it."""
    sim.execute_block('T1 M6')
    state = sim.get_state()
    assert state.active_tool == 'T1'

    sim.execute_block('T3 M6')
    state = sim.get_state()
    assert state.active_tool == 'T3'


def test_tool_change_count_in_summary(sim):
    """Tool changes should be counted in the summary."""
    summary = sim.execute_program([
        'T1 M6',
        'G1 X10 F500',
        'T2 M6',
        'G1 X20 F500',
    ])
    assert summary.tool_changes == 2
    assert 'T1' in summary.tools_used
    assert 'T2' in summary.tools_used


# ---- Material removal ----

def test_material_removal_only_when_spindle_on(sim):
    """MRR should only accumulate when the spindle is running."""
    # Without spindle
    sim.execute_block('G1 X10 F500')
    removed_no_spindle = sim.get_state().material_removed_mm3

    sim.reset()
    # With spindle
    sim.execute_block('S8000 M3')
    sim.execute_block('G1 X10 F500')
    removed_with_spindle = sim.get_state().material_removed_mm3

    assert removed_no_spindle == 0.0
    assert removed_with_spindle > 0.0


# ---- Full program execution ----

def test_execute_program_returns_summary(sim):
    """execute_program should return a well-formed SimulationSummary."""
    program = [
        'G0 X0 Y0 Z5',
        'S10000 M3',
        'M8',
        'T1 M6',
        'G0 X0 Y0 Z1',
        'G1 Z-1 F200',
        'G1 X50 F500',
        'G1 Y30 F500',
        'G0 Z5',
        'M5',
        'M9',
    ]
    summary = sim.execute_program(program)

    assert summary.total_time_sec > 0
    assert summary.cutting_time_sec > 0
    assert summary.rapid_time_sec > 0
    assert summary.total_distance_mm > 0
    assert summary.cutting_distance_mm > 0
    assert summary.rapid_distance_mm > 0
    assert summary.tool_changes == 1
    assert 'T1' in summary.tools_used


# ---- Position history ----

def test_position_history_length(sim):
    """Each move should add one record; initial position adds another."""
    sim.execute_block('G0 X10')
    sim.execute_block('G1 X20 F300')
    sim.execute_block('G1 Y5 F300')
    history = sim.get_position_history()
    # 1 initial + 3 moves = 4
    assert len(history) == 4


# ---- Reset ----

def test_reset_clears_state(sim):
    """After reset, all accumulators return to zero / initial values."""
    sim.execute_block('G1 X100 F500')
    sim.execute_block('S8000 M3')
    sim.reset()
    state = sim.get_state()
    assert state.current_position == (0.0, 0.0, 0.0)
    assert state.elapsed_time_sec == 0.0
    assert state.total_distance_mm == 0.0
    assert state.material_removed_mm3 == 0.0
    assert state.spindle_rpm == 0.0
    assert state.coolant_on is False
    assert len(sim.get_position_history()) == 1  # only initial record


# ---- Arc codes treated as linear ----

def test_g2_g3_treated_as_linear(sim):
    """G2/G3 arc codes should move the tool (simplified as linear)."""
    sim.execute_block('S8000 M3')
    sim.execute_block('G2 X10 Y10 F400')
    state = sim.get_state()
    assert state.current_position == (10.0, 10.0, 0.0)
    expected_dist = math.sqrt(10**2 + 10**2)
    assert state.total_distance_mm == pytest.approx(expected_dist, abs=0.01)


# ---- Comments and blank lines ----

def test_comments_and_blanks_ignored(sim):
    """Comments and blank lines should not affect state."""
    sim.execute_block('; this is a comment')
    sim.execute_block('( another comment )')
    sim.execute_block('')
    sim.execute_block('%')
    state = sim.get_state()
    assert state.current_position == (0.0, 0.0, 0.0)
    # Only initial position in history
    assert len(sim.get_position_history()) == 1
