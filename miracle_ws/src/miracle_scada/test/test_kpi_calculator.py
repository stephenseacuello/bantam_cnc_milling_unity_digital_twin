"""Tests for OEE / KPI calculation logic.

Extracts the pure calculation functions and the MachineMetrics accumulator
from kpi_calculator.py and tests them without any ROS2 dependency.
"""

import pytest
from dataclasses import dataclass, field
from typing import Set


# ---------------------------------------------------------------------------
# Pure calculation functions (mirrored from kpi_calculator.py)
# ---------------------------------------------------------------------------

def compute_availability(planned_production_time: float, downtime: float) -> float:
    if planned_production_time <= 0.0:
        return 1.0
    return max(0.0, min(1.0, (planned_production_time - downtime) / planned_production_time))


def compute_performance(ideal_cycle_time: float, total_count: int, run_time: float) -> float:
    if run_time <= 0.0 or total_count <= 0:
        return 1.0
    return max(0.0, min(1.0, (ideal_cycle_time * total_count) / run_time))


def compute_quality(good_count: int, total_count: int) -> float:
    if total_count <= 0:
        return 1.0
    return max(0.0, min(1.0, good_count / total_count))


def compute_oee(availability: float, performance: float, quality: float) -> float:
    return availability * performance * quality


def compute_mtbf(total_run_time: float, failure_count: int) -> float:
    if failure_count <= 0:
        return 0.0
    return total_run_time / failure_count


def compute_mttr(total_repair_time: float, failure_count: int) -> float:
    if failure_count <= 0:
        return 0.0
    return total_repair_time / failure_count


# ---------------------------------------------------------------------------
# Extracted accumulator (mirrors MachineMetrics + state tracking logic)
# ---------------------------------------------------------------------------

_STATE_RUNNING = 'RUNNING'
_STATE_IDLE = 'IDLE'
_STATE_ERROR = 'ERROR'
_STATE_MAINTENANCE = 'MAINTENANCE'
_DOWNTIME_STATES = {_STATE_ERROR, _STATE_MAINTENANCE}

_QUALITY_ANOMALY_TYPES = {
    'surface_quality_anomaly',
    'surface_quality',
    'dimensional',
    'dimensional_deviation',
    'tool_wear_excessive',
}


@dataclass
class _MachineMetrics:
    """Mirrors kpi_calculator.MachineMetrics for testing."""
    planned_production_time: float = 0.0
    run_time: float = 0.0
    downtime: float = 0.0
    idle_time: float = 0.0
    repair_time: float = 0.0

    total_jobs: int = 0
    good_jobs: int = 0
    defect_jobs: int = 0
    jobs_in_progress: int = 0
    jobs_queued: int = 0

    ideal_cycle_time: float = 60.0
    total_actual_cycle_time: float = 0.0

    failure_count: int = 0
    total_repair_time: float = 0.0

    last_state: str = _STATE_IDLE
    last_state_change: float = 0.0

    tool_life_used: float = 0.0
    tool_life_total: float = 1.0

    jobs_on_time: int = 0
    jobs_scheduled: int = 0

    defect_job_ids: set = field(default_factory=set)
    completed_job_ids: set = field(default_factory=set)


def _accumulate_state_time(m: _MachineMetrics, state: str, elapsed: float) -> None:
    """Mirrors KPICalculatorNode._accumulate_state_time."""
    m.planned_production_time += elapsed
    if state == _STATE_RUNNING:
        m.run_time += elapsed
    elif state in _DOWNTIME_STATES:
        m.downtime += elapsed
    elif state == _STATE_IDLE:
        m.idle_time += elapsed


def _transition_state(m: _MachineMetrics, new_state: str, now: float) -> None:
    """Simulate a state transition (mirrors _on_machine_state logic)."""
    elapsed = max(0.0, now - m.last_state_change)
    _accumulate_state_time(m, m.last_state, elapsed)

    if new_state == _STATE_ERROR and m.last_state != _STATE_ERROR:
        m.failure_count += 1
    if m.last_state == _STATE_ERROR and new_state != _STATE_ERROR:
        m.total_repair_time += elapsed

    m.last_state = new_state
    m.last_state_change = now


def _complete_job(m: _MachineMetrics, job_id: str, elapsed_sec: float, has_errors: bool = False) -> None:
    """Simulate a job completion (mirrors _on_job_status logic)."""
    if job_id not in m.completed_job_ids:
        m.completed_job_ids.add(job_id)
        m.total_jobs += 1
        m.total_actual_cycle_time += elapsed_sec
        if job_id not in m.defect_job_ids:
            m.good_jobs += 1
        m.jobs_scheduled += 1
        if not has_errors:
            m.jobs_on_time += 1


def _record_quality_anomaly(m: _MachineMetrics, anomaly_type: str) -> None:
    """Simulate a quality anomaly (mirrors _on_anomaly logic)."""
    if anomaly_type.lower() in _QUALITY_ANOMALY_TYPES:
        m.defect_jobs += 1
        defect_id = f"defect_{m.defect_jobs}"
        m.defect_job_ids.add(defect_id)
        if m.good_jobs > 0:
            m.good_jobs -= 1


def _aggregate_fleet(machines: list, ideal_cycle_time: float = 60.0):
    """Aggregate KPIs across a list of _MachineMetrics (mirrors _compute_fleet_kpis)."""
    total_planned = sum(m.planned_production_time for m in machines)
    total_downtime = sum(m.downtime for m in machines)
    total_run = sum(m.run_time for m in machines)
    total_jobs = sum(m.total_jobs for m in machines)
    total_good = sum(m.good_jobs for m in machines)
    total_failures = sum(m.failure_count for m in machines)
    total_repair = sum(m.total_repair_time for m in machines)
    total_on_time = sum(m.jobs_on_time for m in machines)
    total_scheduled = sum(m.jobs_scheduled for m in machines)

    availability = compute_availability(total_planned, total_downtime)
    performance = compute_performance(ideal_cycle_time, total_jobs, total_run)
    quality = compute_quality(total_good, total_jobs)
    oee = compute_oee(availability, performance, quality)
    mtbf = compute_mtbf(total_run, total_failures)
    mttr = compute_mttr(total_repair, total_failures)
    schedule_adherence = total_on_time / total_scheduled if total_scheduled > 0 else 1.0

    return {
        'availability': availability,
        'performance': performance,
        'quality': quality,
        'oee': oee,
        'mtbf': mtbf,
        'mttr': mttr,
        'schedule_adherence': schedule_adherence,
        'total_jobs': total_jobs,
        'total_good': total_good,
    }


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def machine():
    """Return a fresh machine metrics instance."""
    return _MachineMetrics()


# ---------------------------------------------------------------------------
# Test classes
# ---------------------------------------------------------------------------

class TestOEECalculation:
    """Test OEE formula and its components."""

    def test_oee_standard_values(self):
        """Availability=0.9, Performance=0.85, Quality=0.95 -> OEE~0.727."""
        oee = compute_oee(0.9, 0.85, 0.95)
        assert abs(oee - 0.72675) < 0.001

    def test_perfect_oee(self):
        """All components at 1.0 -> OEE=1.0."""
        assert compute_oee(1.0, 1.0, 1.0) == 1.0

    def test_zero_availability(self):
        """Zero availability -> OEE=0."""
        assert compute_oee(0.0, 0.85, 0.95) == 0.0

    def test_zero_performance(self):
        """Zero performance -> OEE=0."""
        assert compute_oee(0.9, 0.0, 0.95) == 0.0

    def test_zero_quality(self):
        """Zero quality -> OEE=0."""
        assert compute_oee(0.9, 0.85, 0.0) == 0.0

    def test_availability_calculation(self):
        """Planned=1000, Downtime=100 -> Availability=0.9."""
        assert abs(compute_availability(1000.0, 100.0) - 0.9) < 1e-9

    def test_availability_no_downtime(self):
        """No downtime -> Availability=1.0."""
        assert compute_availability(1000.0, 0.0) == 1.0

    def test_availability_all_downtime(self):
        """All downtime -> Availability=0.0."""
        assert compute_availability(1000.0, 1000.0) == 0.0

    def test_performance_calculation(self):
        """Ideal=60s, 10 jobs, run_time=706s -> Performance~0.85."""
        perf = compute_performance(60.0, 10, 706.0)
        assert abs(perf - 0.8499) < 0.01

    def test_performance_perfect(self):
        """Jobs completed exactly at ideal cycle time."""
        perf = compute_performance(60.0, 10, 600.0)
        assert abs(perf - 1.0) < 1e-9

    def test_quality_calculation(self):
        """19 good out of 20 total -> Quality=0.95."""
        assert abs(compute_quality(19, 20) - 0.95) < 1e-9

    def test_quality_all_good(self):
        """All jobs good -> Quality=1.0."""
        assert compute_quality(10, 10) == 1.0

    def test_quality_all_defective(self):
        """No good jobs -> Quality=0.0."""
        assert compute_quality(0, 10) == 0.0


class TestDowntimeTracking:
    """Test that ERROR state increases downtime."""

    def test_error_state_increases_downtime(self, machine):
        """Transitioning to ERROR and staying there increases downtime."""
        _transition_state(machine, _STATE_RUNNING, now=0.0)
        _transition_state(machine, _STATE_ERROR, now=100.0)
        _transition_state(machine, _STATE_RUNNING, now=150.0)

        assert machine.downtime == 50.0
        assert machine.run_time == 100.0

    def test_maintenance_state_increases_downtime(self, machine):
        """MAINTENANCE should also count as downtime."""
        _transition_state(machine, _STATE_RUNNING, now=0.0)
        _transition_state(machine, _STATE_MAINTENANCE, now=100.0)
        _transition_state(machine, _STATE_RUNNING, now=200.0)

        assert machine.downtime == 100.0

    def test_idle_does_not_count_as_downtime(self, machine):
        """IDLE state should not add to downtime."""
        _transition_state(machine, _STATE_RUNNING, now=0.0)
        _transition_state(machine, _STATE_IDLE, now=100.0)
        _transition_state(machine, _STATE_RUNNING, now=200.0)

        assert machine.downtime == 0.0
        assert machine.idle_time == 100.0

    def test_error_increments_failure_count(self, machine):
        """Each transition into ERROR increments failure_count."""
        _transition_state(machine, _STATE_RUNNING, now=0.0)
        _transition_state(machine, _STATE_ERROR, now=100.0)
        _transition_state(machine, _STATE_RUNNING, now=150.0)
        _transition_state(machine, _STATE_ERROR, now=200.0)
        _transition_state(machine, _STATE_RUNNING, now=220.0)

        assert machine.failure_count == 2

    def test_staying_in_error_no_double_count(self, machine):
        """Repeated ERROR state (without leaving) should not double-count."""
        _transition_state(machine, _STATE_ERROR, now=0.0)
        _transition_state(machine, _STATE_ERROR, now=50.0)

        assert machine.failure_count == 1

    def test_repair_time_tracked(self, machine):
        """Time spent in ERROR should be tracked as repair time."""
        _transition_state(machine, _STATE_RUNNING, now=0.0)
        _transition_state(machine, _STATE_ERROR, now=100.0)
        _transition_state(machine, _STATE_RUNNING, now=130.0)

        assert machine.total_repair_time == 30.0


class TestJobTracking:
    """Test that completed jobs update performance metrics."""

    def test_completed_job_increments_count(self, machine):
        """A completed job should increase total_jobs and good_jobs."""
        _complete_job(machine, 'job1', elapsed_sec=65.0)
        assert machine.total_jobs == 1
        assert machine.good_jobs == 1

    def test_duplicate_job_not_counted(self, machine):
        """The same job_id completed twice should only count once."""
        _complete_job(machine, 'job1', elapsed_sec=65.0)
        _complete_job(machine, 'job1', elapsed_sec=65.0)
        assert machine.total_jobs == 1

    def test_multiple_jobs(self, machine):
        """Multiple unique jobs should all be counted."""
        for i in range(5):
            _complete_job(machine, f'job{i}', elapsed_sec=60.0)
        assert machine.total_jobs == 5
        assert machine.good_jobs == 5

    def test_job_with_errors_not_on_time(self, machine):
        """Jobs with errors should not count towards on-time delivery."""
        _complete_job(machine, 'job1', elapsed_sec=65.0, has_errors=True)
        assert machine.jobs_on_time == 0
        assert machine.jobs_scheduled == 1

    def test_cycle_time_accumulation(self, machine):
        """Elapsed time from jobs should accumulate."""
        _complete_job(machine, 'job1', elapsed_sec=55.0)
        _complete_job(machine, 'job2', elapsed_sec=70.0)
        assert machine.total_actual_cycle_time == 125.0


class TestQualityTracking:
    """Test that anomaly alerts reduce quality."""

    def test_surface_quality_anomaly_reduces_good_count(self, machine):
        """A surface quality anomaly should reduce good_jobs."""
        _complete_job(machine, 'job1', elapsed_sec=60.0)
        assert machine.good_jobs == 1
        _record_quality_anomaly(machine, 'surface_quality_anomaly')
        assert machine.good_jobs == 0
        assert machine.defect_jobs == 1

    def test_dimensional_anomaly_reduces_good_count(self, machine):
        """A dimensional anomaly should reduce good_jobs."""
        _complete_job(machine, 'job1', elapsed_sec=60.0)
        _record_quality_anomaly(machine, 'dimensional')
        assert machine.good_jobs == 0

    def test_non_quality_anomaly_ignored(self, machine):
        """Anomaly types not in the quality set should be ignored."""
        _complete_job(machine, 'job1', elapsed_sec=60.0)
        _record_quality_anomaly(machine, 'vibration_anomaly')
        assert machine.good_jobs == 1
        assert machine.defect_jobs == 0

    def test_multiple_anomalies(self, machine):
        """Multiple anomalies should reduce good count cumulatively."""
        for i in range(5):
            _complete_job(machine, f'job{i}', elapsed_sec=60.0)
        assert machine.good_jobs == 5

        _record_quality_anomaly(machine, 'surface_quality')
        _record_quality_anomaly(machine, 'dimensional_deviation')
        assert machine.good_jobs == 3
        assert machine.defect_jobs == 2

    def test_good_count_does_not_go_negative(self, machine):
        """Good count should not drop below zero."""
        _record_quality_anomaly(machine, 'surface_quality_anomaly')
        assert machine.good_jobs == 0  # was already 0


class TestMTBFMTTR:
    """Test MTBF and MTTR calculations."""

    def test_mtbf_basic(self):
        """MTBF = 1000 run time / 2 failures = 500."""
        assert compute_mtbf(1000.0, 2) == 500.0

    def test_mtbf_single_failure(self):
        """MTBF with one failure."""
        assert compute_mtbf(3600.0, 1) == 3600.0

    def test_mtbf_no_failures(self):
        """MTBF with zero failures should return 0 (undefined)."""
        assert compute_mtbf(1000.0, 0) == 0.0

    def test_mttr_basic(self):
        """MTTR = 200 repair time / 2 failures = 100."""
        assert compute_mttr(200.0, 2) == 100.0

    def test_mttr_no_failures(self):
        """MTTR with zero failures should return 0 (undefined)."""
        assert compute_mttr(100.0, 0) == 0.0

    def test_mtbf_mttr_from_state_transitions(self, machine):
        """MTBF and MTTR computed from actual state transitions."""
        # Run 100s, error 30s, run 200s, error 20s, run 100s
        _transition_state(machine, _STATE_RUNNING, now=0.0)
        _transition_state(machine, _STATE_ERROR, now=100.0)
        _transition_state(machine, _STATE_RUNNING, now=130.0)
        _transition_state(machine, _STATE_ERROR, now=330.0)
        _transition_state(machine, _STATE_RUNNING, now=350.0)
        # Flush the last running segment
        _transition_state(machine, _STATE_IDLE, now=450.0)

        # run_time = 100 + 200 + 100 = 400
        # failures = 2
        # repair_time = 30 + 20 = 50
        assert machine.run_time == 400.0
        assert machine.failure_count == 2
        assert machine.total_repair_time == 50.0
        assert compute_mtbf(machine.run_time, machine.failure_count) == 200.0
        assert compute_mttr(machine.total_repair_time, machine.failure_count) == 25.0


class TestFleetAggregation:
    """Test KPI aggregation across multiple machines."""

    def test_two_machine_aggregation(self):
        """Aggregate OEE across two machines."""
        m1 = _MachineMetrics()
        m2 = _MachineMetrics()

        # Machine 1: runs well
        _transition_state(m1, _STATE_RUNNING, now=0.0)
        _transition_state(m1, _STATE_IDLE, now=1000.0)
        for i in range(10):
            _complete_job(m1, f'm1_job{i}', elapsed_sec=60.0)

        # Machine 2: has some downtime
        _transition_state(m2, _STATE_RUNNING, now=0.0)
        _transition_state(m2, _STATE_ERROR, now=500.0)
        _transition_state(m2, _STATE_RUNNING, now=600.0)
        _transition_state(m2, _STATE_IDLE, now=1000.0)
        for i in range(8):
            _complete_job(m2, f'm2_job{i}', elapsed_sec=65.0)

        result = _aggregate_fleet([m1, m2], ideal_cycle_time=60.0)

        # total_planned = 1000 + 1000 = 2000
        # total_downtime = 0 + 100 = 100
        # availability = 1900/2000 = 0.95
        assert abs(result['availability'] - 0.95) < 0.01

        # total_run = 1000 + 900 = 1900
        # total_jobs = 18
        # performance = (60*18)/1900 = 1080/1900 ~ 0.568
        assert result['performance'] > 0.0
        assert result['performance'] <= 1.0

        # total_good = 18 (no anomalies)
        assert result['quality'] == 1.0

        # OEE should be product of components
        expected_oee = result['availability'] * result['performance'] * result['quality']
        assert abs(result['oee'] - expected_oee) < 1e-9

    def test_fleet_with_anomalies(self):
        """Fleet with quality defects should reduce quality."""
        m1 = _MachineMetrics()
        m2 = _MachineMetrics()

        _transition_state(m1, _STATE_RUNNING, now=0.0)
        _transition_state(m1, _STATE_IDLE, now=500.0)
        for i in range(10):
            _complete_job(m1, f'm1_job{i}', elapsed_sec=50.0)
        _record_quality_anomaly(m1, 'surface_quality_anomaly')

        _transition_state(m2, _STATE_RUNNING, now=0.0)
        _transition_state(m2, _STATE_IDLE, now=500.0)
        for i in range(10):
            _complete_job(m2, f'm2_job{i}', elapsed_sec=50.0)

        result = _aggregate_fleet([m1, m2], ideal_cycle_time=50.0)

        # total_good = 9 + 10 = 19, total_jobs = 20
        assert result['total_good'] == 19
        assert result['total_jobs'] == 20
        assert abs(result['quality'] - 0.95) < 1e-9

    def test_empty_fleet(self):
        """No machines should return defaults (1.0 for ratios)."""
        result = _aggregate_fleet([])
        assert result['availability'] == 1.0
        assert result['performance'] == 1.0
        assert result['quality'] == 1.0
        assert result['oee'] == 1.0

    def test_fleet_schedule_adherence(self):
        """Schedule adherence should aggregate across fleet."""
        m1 = _MachineMetrics()
        m2 = _MachineMetrics()

        # m1: 3 on time out of 4
        for i in range(3):
            _complete_job(m1, f'm1_j{i}', elapsed_sec=50.0, has_errors=False)
        _complete_job(m1, 'm1_j3', elapsed_sec=50.0, has_errors=True)

        # m2: 2 on time out of 2
        for i in range(2):
            _complete_job(m2, f'm2_j{i}', elapsed_sec=50.0, has_errors=False)

        result = _aggregate_fleet([m1, m2])

        # 5 on time / 6 scheduled
        assert abs(result['schedule_adherence'] - 5.0 / 6.0) < 1e-9


class TestEdgeCases:
    """Test edge cases and boundary conditions."""

    def test_zero_production_time(self):
        """Zero planned production time should return availability=1.0."""
        assert compute_availability(0.0, 0.0) == 1.0

    def test_zero_run_time_performance(self):
        """Zero run time should return performance=1.0."""
        assert compute_performance(60.0, 0, 0.0) == 1.0

    def test_zero_total_count_quality(self):
        """Zero total count should return quality=1.0."""
        assert compute_quality(0, 0) == 1.0

    def test_negative_downtime_clamped(self):
        """Negative effective downtime should be clamped at 0."""
        # This shouldn't happen in practice, but the formula handles it
        avail = compute_availability(100.0, -10.0)
        assert avail == 1.0  # clamped to max 1.0

    def test_performance_capped_at_one(self):
        """Performance should never exceed 1.0 even if faster than ideal."""
        perf = compute_performance(60.0, 20, 600.0)
        # ideal * count / run = 60*20/600 = 2.0, but capped at 1.0
        assert perf == 1.0

    def test_availability_clamped_at_zero(self):
        """Downtime exceeding planned time should give availability=0."""
        assert compute_availability(100.0, 200.0) == 0.0

    def test_no_state_transitions(self, machine):
        """Machine with no transitions should have zero accumulators."""
        assert machine.planned_production_time == 0.0
        assert machine.run_time == 0.0
        assert machine.downtime == 0.0
        assert machine.failure_count == 0

    def test_single_machine_fleet(self):
        """Single machine fleet should equal single machine KPIs."""
        m = _MachineMetrics()
        _transition_state(m, _STATE_RUNNING, now=0.0)
        _transition_state(m, _STATE_IDLE, now=1000.0)
        for i in range(10):
            _complete_job(m, f'job{i}', elapsed_sec=60.0)

        result = _aggregate_fleet([m], ideal_cycle_time=60.0)

        solo_avail = compute_availability(m.planned_production_time, m.downtime)
        solo_perf = compute_performance(60.0, m.total_jobs, m.run_time)
        solo_qual = compute_quality(m.good_jobs, m.total_jobs)

        assert abs(result['availability'] - solo_avail) < 1e-9
        assert abs(result['performance'] - solo_perf) < 1e-9
        assert abs(result['quality'] - solo_qual) < 1e-9

    def test_mtbf_zero_run_time_with_failures(self):
        """MTBF with zero run time but failures should be 0."""
        assert compute_mtbf(0.0, 3) == 0.0

    def test_schedule_adherence_no_jobs(self):
        """No scheduled jobs should default to 1.0 adherence."""
        result = _aggregate_fleet([_MachineMetrics()])
        assert result['schedule_adherence'] == 1.0
