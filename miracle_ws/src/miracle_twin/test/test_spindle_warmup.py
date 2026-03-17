"""Tests for SpindleWarmupManager warmup profile generation,
thermal stability evaluation, and warmup recommendations.
"""

import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import pytest
from miracle_twin.cutting_sim_proxy import (
    SpindleWarmupManager,
    WarmupProfile,
    WarmupStage,
    WarmupStatus,
)


@pytest.fixture
def manager():
    return SpindleWarmupManager()


# ---- Cold start profile ----

def test_cold_start_profile_has_five_stages(manager):
    """Cold-start profile should contain exactly 5 stages ramping up."""
    profile = manager.generate_profile(target_rpm=8000, machine_state='cold')
    assert len(profile.stages) == 5
    assert profile.target_rpm == 8000
    # First stage should be slow (500 rpm)
    assert profile.stages[0].rpm == 500
    # Last stage should be at target
    assert profile.stages[-1].rpm == 8000


def test_cold_start_profile_stage_rpms_ascending(manager):
    """RPMs across cold-start stages must be monotonically increasing."""
    profile = manager.generate_profile(target_rpm=10000, machine_state='cold')
    rpms = [s.rpm for s in profile.stages]
    assert rpms == sorted(rpms)
    assert rpms[0] == 500
    assert rpms[-1] == 10000


# ---- Warm restart profile ----

def test_warm_restart_profile_is_shorter(manager):
    """Warm restart should have fewer stages and shorter total time than cold."""
    cold = manager.generate_profile(target_rpm=8000, machine_state='cold')
    warm = manager.generate_profile(target_rpm=8000, machine_state='warm')
    assert len(warm.stages) == 2
    assert warm.total_duration_min < cold.total_duration_min


# ---- High-speed profile ----

def test_high_speed_profile_six_stages(manager):
    """High-speed profile (>= 20000 rpm) should have 6 stages up to 24000."""
    profile = manager.generate_profile(target_rpm=24000, machine_state='cold')
    assert len(profile.stages) == 6
    assert profile.target_rpm == 24000.0
    assert profile.stages[-1].rpm == 24000


def test_high_speed_profile_overrides_warm_state(manager):
    """Even if machine_state is warm, target >= 20000 uses high-speed profile."""
    profile = manager.generate_profile(target_rpm=22000, machine_state='warm')
    assert len(profile.stages) == 6
    assert profile.target_rpm == 24000.0


# ---- Stability evaluation (converged temps) ----

def test_stability_converged_temps(manager):
    """When all temps are within tolerance, stability should be 100%."""
    bearing = [45.0, 45.5, 45.2]
    spindle = [45.1, 45.3]
    stability = manager.evaluate_stability(bearing, spindle)
    assert stability == 100.0


# ---- Stability evaluation (divergent temps) ----

def test_stability_divergent_temps(manager):
    """Large temperature spread should give low stability percentage."""
    bearing = [30.0, 32.0]
    spindle = [50.0, 55.0]
    stability = manager.evaluate_stability(bearing, spindle)
    assert 0.0 <= stability < 50.0


def test_stability_empty_temps(manager):
    """Empty temp lists should return 0% stability."""
    stability = manager.evaluate_stability([], [])
    assert stability == 0.0


# ---- Warmup recommendation (cold machine) ----

def test_recommend_warmup_cold_machine(manager):
    """Machine idle > 4 h should recommend warmup."""
    recommended, reason = manager.recommend_warmup(target_rpm=8000, hours_since_last_run=6.0)
    assert recommended is True
    assert 'cold-start' in reason.lower() or 'thermal shock' in reason.lower()


# ---- Warmup recommendation (warm machine) ----

def test_recommend_warmup_warm_machine_no_warmup(manager):
    """Machine idle < 2 h with moderate RPM should not need warmup."""
    recommended, reason = manager.recommend_warmup(target_rpm=5000, hours_since_last_run=0.5)
    assert recommended is False
    assert 'no warmup' in reason.lower()


def test_recommend_warmup_warm_restart_range(manager):
    """Machine idle 2-4 h should recommend warm-restart warmup."""
    recommended, reason = manager.recommend_warmup(target_rpm=8000, hours_since_last_run=3.0)
    assert recommended is True
    assert 'warm' in reason.lower()


# ---- Custom target RPM profile ----

def test_custom_target_rpm_profile(manager):
    """Custom target RPM should appear in the final stage of the cold profile."""
    custom_rpm = 6500.0
    profile = manager.generate_profile(target_rpm=custom_rpm, machine_state='cold')
    assert profile.target_rpm == custom_rpm
    assert profile.stages[-1].rpm == custom_rpm
    # Total duration should be the sum of all stage durations
    expected_total = sum(s.duration_min for s in profile.stages)
    assert abs(profile.total_duration_min - expected_total) < 0.01


def test_custom_rpm_hot_machine(manager):
    """Hot machine should produce a single-stage verification profile."""
    profile = manager.generate_profile(target_rpm=8000, machine_state='hot')
    assert len(profile.stages) == 1
    assert profile.stages[0].rpm == 8000
    assert profile.total_duration_min <= 3.0


# ---- Warmup status tracking ----

def test_warmup_status_in_progress(manager):
    """Status mid-warmup should report correct stage and not complete."""
    profile = manager.generate_profile(target_rpm=8000, machine_state='cold')
    status = manager.get_status(profile, elapsed_min=4.0)
    assert isinstance(status, WarmupStatus)
    assert not status.is_complete
    assert status.current_stage_idx >= 1  # past first stage (3 min)
    assert 0.0 <= status.thermal_stability_pct <= 100.0


def test_warmup_status_complete(manager):
    """Status after total duration should be marked complete."""
    profile = manager.generate_profile(target_rpm=8000, machine_state='cold')
    status = manager.get_status(profile, elapsed_min=profile.total_duration_min + 1.0)
    assert status.is_complete
    assert status.thermal_stability_pct == 100.0


def test_high_speed_recommend_warmup_regardless(manager):
    """High-speed target should always recommend warmup, even if recently used."""
    recommended, reason = manager.recommend_warmup(target_rpm=22000, hours_since_last_run=0.5)
    assert recommended is True
    assert 'high-speed' in reason.lower()
