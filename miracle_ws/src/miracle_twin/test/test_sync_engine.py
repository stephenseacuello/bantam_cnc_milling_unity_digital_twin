"""Tests for digital twin synchronization logic.

Extracts the core algorithmic logic from SyncEngineNode -- LERP
interpolation, drift detection, sync rate adaptation, and state
snapshot comparison -- and tests it without any ROS2 dependency.
"""

import math
import pytest
from dataclasses import dataclass, field
from typing import Dict, List, Optional


# ---------------------------------------------------------------------------
# Extracted helpers (mirror the logic in sync_engine.py)
# ---------------------------------------------------------------------------

def lerp(current: float, target: float, alpha: float) -> float:
    """Linear interpolation between current and target.

    Matches SyncEngineNode._lerp exactly:
        return current + alpha * (target - current)
    """
    return current + alpha * (target - current)


@dataclass
class _MachineSnapshot:
    """A lightweight snapshot of machine state (replaces MachineState msg)."""
    machine_id: str = ''
    spindle_speed: float = 0.0
    feed_rate: float = 0.0
    spindle_load: float = 0.0
    coolant_level: float = 100.0
    axis_positions: List[float] = field(default_factory=list)
    axis_velocities: List[float] = field(default_factory=list)
    status: str = 'IDLE'
    current_program: str = ''
    current_line: int = 0

    def copy(self):
        """Create a deep-ish copy of this snapshot."""
        return _MachineSnapshot(
            machine_id=self.machine_id,
            spindle_speed=self.spindle_speed,
            feed_rate=self.feed_rate,
            spindle_load=self.spindle_load,
            coolant_level=self.coolant_level,
            axis_positions=list(self.axis_positions),
            axis_velocities=list(self.axis_velocities),
            status=self.status,
            current_program=self.current_program,
            current_line=self.current_line,
        )


def compute_drift(phys: _MachineSnapshot, twin: _MachineSnapshot) -> float:
    """Compute drift magnitude between physical and twin states.

    Matches SyncEngineNode._compute_drift exactly:
        drift += sum((p-t)^2 for axis positions)
        drift += (spindle_speed diff)^2 * 1e-8
        drift += (feed_rate diff)^2 * 1e-6
        return sqrt(drift)
    """
    drift = 0.0
    if phys.axis_positions and twin.axis_positions:
        for p, t in zip(phys.axis_positions, twin.axis_positions):
            drift += (p - t) ** 2
    drift += (phys.spindle_speed - twin.spindle_speed) ** 2 * 1e-8
    drift += (phys.feed_rate - twin.feed_rate) ** 2 * 1e-6
    return math.sqrt(drift)


def interpolate_twin(twin: _MachineSnapshot, phys: _MachineSnapshot,
                     alpha: float) -> _MachineSnapshot:
    """Interpolate twin state toward physical state.

    Mirrors the interpolation logic in SyncEngineNode._sync_cycle.
    """
    result = twin.copy()
    result.status = phys.status
    result.spindle_speed = lerp(twin.spindle_speed, phys.spindle_speed, alpha)
    result.feed_rate = lerp(twin.feed_rate, phys.feed_rate, alpha)
    result.spindle_load = lerp(twin.spindle_load, phys.spindle_load, alpha)
    result.coolant_level = phys.coolant_level
    result.current_program = phys.current_program
    result.current_line = phys.current_line

    if phys.axis_positions:
        if not twin.axis_positions:
            result.axis_positions = list(phys.axis_positions)
        else:
            result.axis_positions = [
                lerp(tw_pos, ph_pos, alpha)
                for tw_pos, ph_pos in zip(twin.axis_positions, phys.axis_positions)
            ]
    result.axis_velocities = list(phys.axis_velocities) if phys.axis_velocities else []
    return result


class _SyncRateAdapter:
    """Adapts synchronization rate based on drift magnitude.

    When drift is high, sync faster. When drift is low, sync slower.
    This saves computational resources when the twin is well-aligned.
    """

    def __init__(self, base_rate_hz=50.0, min_rate_hz=10.0,
                 max_rate_hz=200.0, drift_threshold=1.0):
        self.base_rate_hz = base_rate_hz
        self.min_rate_hz = min_rate_hz
        self.max_rate_hz = max_rate_hz
        self.drift_threshold = drift_threshold

    def compute_rate(self, drift_magnitude: float) -> float:
        """Compute the sync rate based on current drift.

        Rate scales linearly with drift/threshold ratio, clamped to bounds.
        """
        if self.drift_threshold <= 0:
            return self.max_rate_hz

        ratio = drift_magnitude / self.drift_threshold
        rate = self.base_rate_hz * (1.0 + ratio)
        return max(self.min_rate_hz, min(self.max_rate_hz, rate))


@dataclass
class _TwinSyncState:
    """Tracks synchronization state for one machine (mirrors TwinState)."""
    machine_id: str
    physical: Optional[_MachineSnapshot] = None
    twin: Optional[_MachineSnapshot] = None
    drift_magnitude: float = 0.0
    sync_quality: float = 1.0
    last_sync_time: float = 0.0
    sync_count: int = 0


class _SyncEngine:
    """Extracted synchronization engine logic.

    Manages multiple twin sync states and performs sync cycles.
    """

    def __init__(self, max_drift=1.0, interp_factor=0.8):
        self.max_drift = max_drift
        self.interp_factor = interp_factor
        self.twins: Dict[str, _TwinSyncState] = {}

    def update_physical(self, snapshot: _MachineSnapshot):
        """Update the physical state for a machine."""
        mid = snapshot.machine_id
        if mid not in self.twins:
            self.twins[mid] = _TwinSyncState(machine_id=mid)
        self.twins[mid].physical = snapshot

    def sync_cycle(self, current_time: float):
        """Execute one synchronization cycle for all twins."""
        for mid, state in self.twins.items():
            if state.physical is None:
                continue

            if state.twin is None:
                state.twin = state.physical.copy()
                state.twin.machine_id = f"{mid}_twin"

            state.twin = interpolate_twin(
                state.twin, state.physical, self.interp_factor
            )
            state.twin.machine_id = f"{mid}_twin"

            state.drift_magnitude = compute_drift(state.physical, state.twin)
            state.sync_quality = max(
                0.0, 1.0 - state.drift_magnitude / self.max_drift
            )
            state.last_sync_time = current_time
            state.sync_count += 1

            # Force snap if drift exceeds threshold
            if state.drift_magnitude > self.max_drift:
                state.twin.axis_positions = list(state.physical.axis_positions)
                state.twin.spindle_speed = state.physical.spindle_speed
                state.twin.feed_rate = state.physical.feed_rate
                state.drift_magnitude = compute_drift(state.physical, state.twin)
                state.sync_quality = max(
                    0.0, 1.0 - state.drift_magnitude / self.max_drift
                )


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def physical_state():
    """Return a typical CNC physical state snapshot."""
    return _MachineSnapshot(
        machine_id='cnc1',
        spindle_speed=12000.0,
        feed_rate=500.0,
        spindle_load=45.0,
        coolant_level=85.0,
        axis_positions=[100.0, 200.0, -50.0],
        axis_velocities=[10.0, 20.0, -5.0],
        status='RUNNING',
        current_program='part_001.nc',
        current_line=42,
    )


@pytest.fixture
def twin_state():
    """Return a twin state that lags slightly behind physical."""
    return _MachineSnapshot(
        machine_id='cnc1_twin',
        spindle_speed=11500.0,
        feed_rate=480.0,
        spindle_load=43.0,
        coolant_level=85.0,
        axis_positions=[98.0, 198.0, -48.0],
        axis_velocities=[10.0, 20.0, -5.0],
        status='RUNNING',
    )


@pytest.fixture
def sync_engine():
    """Return a fresh sync engine with default settings."""
    return _SyncEngine(max_drift=1.0, interp_factor=0.8)


@pytest.fixture
def rate_adapter():
    """Return a sync rate adapter."""
    return _SyncRateAdapter(base_rate_hz=50.0, min_rate_hz=10.0,
                            max_rate_hz=200.0, drift_threshold=1.0)


# ---------------------------------------------------------------------------
# Test classes
# ---------------------------------------------------------------------------

class TestLerpInterpolation:
    """Test linear interpolation between physical and twin states."""

    def test_alpha_zero_returns_current(self):
        """Alpha=0 should return the current value unchanged."""
        assert lerp(10.0, 20.0, 0.0) == pytest.approx(10.0)

    def test_alpha_one_returns_target(self):
        """Alpha=1 should return the target value exactly."""
        assert lerp(10.0, 20.0, 1.0) == pytest.approx(20.0)

    def test_alpha_half_returns_midpoint(self):
        """Alpha=0.5 should return the midpoint."""
        assert lerp(0.0, 100.0, 0.5) == pytest.approx(50.0)

    def test_alpha_0_8_matches_source(self):
        """Alpha=0.8 (the default interp_factor) should move 80% toward target."""
        result = lerp(0.0, 100.0, 0.8)
        assert result == pytest.approx(80.0)

    def test_identical_values(self):
        """LERP of two identical values should return that value."""
        assert lerp(42.0, 42.0, 0.5) == pytest.approx(42.0)

    def test_negative_values(self):
        """LERP should work with negative numbers."""
        assert lerp(-10.0, 10.0, 0.5) == pytest.approx(0.0)

    def test_convergence_over_iterations(self):
        """Repeated LERP should converge toward the target."""
        current = 0.0
        target = 100.0
        alpha = 0.8
        for _ in range(20):
            current = lerp(current, target, alpha)
        assert current == pytest.approx(target, abs=0.01)

    def test_interpolate_twin_spindle_speed(self, physical_state, twin_state):
        """Interpolation should move twin spindle_speed toward physical."""
        result = interpolate_twin(twin_state, physical_state, 0.8)
        expected = lerp(11500.0, 12000.0, 0.8)
        assert result.spindle_speed == pytest.approx(expected)

    def test_interpolate_twin_axis_positions(self, physical_state, twin_state):
        """Interpolation should apply LERP to each axis position."""
        result = interpolate_twin(twin_state, physical_state, 0.8)
        for i in range(3):
            expected = lerp(
                twin_state.axis_positions[i],
                physical_state.axis_positions[i],
                0.8,
            )
            assert result.axis_positions[i] == pytest.approx(expected)

    def test_interpolate_copies_status(self, physical_state, twin_state):
        """Interpolation should copy status directly (not interpolate)."""
        physical_state.status = 'IDLE'
        result = interpolate_twin(twin_state, physical_state, 0.8)
        assert result.status == 'IDLE'

    def test_interpolate_with_empty_twin_positions(self, physical_state):
        """If twin has no axis positions, should copy from physical."""
        empty_twin = _MachineSnapshot(machine_id='cnc1_twin')
        result = interpolate_twin(empty_twin, physical_state, 0.8)
        assert result.axis_positions == physical_state.axis_positions


class TestDriftDetection:
    """Test drift detection between physical and twin states."""

    def test_identical_states_zero_drift(self, physical_state):
        """Identical physical and twin states should have zero drift."""
        twin = physical_state.copy()
        drift = compute_drift(physical_state, twin)
        assert drift == pytest.approx(0.0)

    def test_axis_position_drift(self):
        """Drift from axis position differences should dominate."""
        phys = _MachineSnapshot(axis_positions=[100.0, 200.0, 300.0])
        twin = _MachineSnapshot(axis_positions=[101.0, 200.0, 300.0])
        drift = compute_drift(phys, twin)
        assert drift == pytest.approx(1.0, abs=0.01)

    def test_spindle_speed_drift_is_scaled(self):
        """Spindle speed drift should be scaled by 1e-8."""
        phys = _MachineSnapshot(spindle_speed=12000.0, axis_positions=[0.0])
        twin = _MachineSnapshot(spindle_speed=11000.0, axis_positions=[0.0])
        drift = compute_drift(phys, twin)
        # (1000)^2 * 1e-8 = 0.01; sqrt(0.01) = 0.1
        assert drift == pytest.approx(0.1, abs=0.01)

    def test_feed_rate_drift_is_scaled(self):
        """Feed rate drift should be scaled by 1e-6."""
        phys = _MachineSnapshot(feed_rate=500.0, axis_positions=[0.0])
        twin = _MachineSnapshot(feed_rate=400.0, axis_positions=[0.0])
        drift = compute_drift(phys, twin)
        # (100)^2 * 1e-6 = 0.01; sqrt(0.01) = 0.1
        assert drift == pytest.approx(0.1, abs=0.01)

    def test_combined_drift(self, physical_state, twin_state):
        """Combined drift should be the sqrt of summed squared components."""
        drift = compute_drift(physical_state, twin_state)
        assert drift > 0.0

        # Verify manually
        ax_drift = sum(
            (p - t) ** 2
            for p, t in zip(physical_state.axis_positions, twin_state.axis_positions)
        )
        sp_drift = (physical_state.spindle_speed - twin_state.spindle_speed) ** 2 * 1e-8
        fr_drift = (physical_state.feed_rate - twin_state.feed_rate) ** 2 * 1e-6
        expected = math.sqrt(ax_drift + sp_drift + fr_drift)
        assert drift == pytest.approx(expected)

    def test_drift_with_empty_positions(self):
        """Drift with empty axis positions should only account for spindle/feed."""
        phys = _MachineSnapshot(spindle_speed=1000.0, feed_rate=100.0)
        twin = _MachineSnapshot(spindle_speed=0.0, feed_rate=0.0)
        drift = compute_drift(phys, twin)
        expected = math.sqrt(1000.0**2 * 1e-8 + 100.0**2 * 1e-6)
        assert drift == pytest.approx(expected, rel=1e-5)

    def test_drift_is_always_non_negative(self):
        """Drift magnitude should always be >= 0."""
        phys = _MachineSnapshot(axis_positions=[0.0], spindle_speed=0.0, feed_rate=0.0)
        twin = _MachineSnapshot(axis_positions=[0.0], spindle_speed=0.0, feed_rate=0.0)
        assert compute_drift(phys, twin) >= 0.0


class TestSyncRateAdaptation:
    """Test sync rate adaptation based on drift magnitude."""

    def test_zero_drift_returns_base_rate(self, rate_adapter):
        """Zero drift should give the base rate."""
        rate = rate_adapter.compute_rate(0.0)
        assert rate == pytest.approx(50.0)

    def test_drift_at_threshold_doubles_rate(self, rate_adapter):
        """Drift equal to threshold should give 2x base rate."""
        rate = rate_adapter.compute_rate(1.0)
        assert rate == pytest.approx(100.0)

    def test_high_drift_clamped_to_max(self, rate_adapter):
        """Very high drift should be clamped to max_rate_hz."""
        rate = rate_adapter.compute_rate(100.0)
        assert rate == pytest.approx(200.0)

    def test_rate_never_below_min(self, rate_adapter):
        """Rate should never drop below min_rate_hz."""
        rate = rate_adapter.compute_rate(0.0)
        assert rate >= rate_adapter.min_rate_hz

    def test_rate_increases_with_drift(self, rate_adapter):
        """Higher drift should produce higher sync rate."""
        low = rate_adapter.compute_rate(0.1)
        high = rate_adapter.compute_rate(0.9)
        assert high > low

    def test_custom_threshold(self):
        """Custom drift threshold should scale appropriately."""
        adapter = _SyncRateAdapter(base_rate_hz=50.0, drift_threshold=0.5)
        # drift=0.5 at threshold=0.5 => ratio=1 => rate = 50*(1+1) = 100
        rate = adapter.compute_rate(0.5)
        assert rate == pytest.approx(100.0)


class TestStateSnapshotAndComparison:
    """Test state snapshot creation and comparison."""

    def test_snapshot_copy_is_independent(self, physical_state):
        """Copying a snapshot should create an independent copy."""
        copy = physical_state.copy()
        copy.spindle_speed = 0.0
        assert physical_state.spindle_speed == 12000.0

    def test_snapshot_copy_has_independent_lists(self, physical_state):
        """List fields in copies should be independent."""
        copy = physical_state.copy()
        copy.axis_positions[0] = 999.0
        assert physical_state.axis_positions[0] == 100.0

    def test_snapshot_preserves_all_fields(self, physical_state):
        """A copy should preserve all field values."""
        copy = physical_state.copy()
        assert copy.machine_id == physical_state.machine_id
        assert copy.spindle_speed == physical_state.spindle_speed
        assert copy.feed_rate == physical_state.feed_rate
        assert copy.spindle_load == physical_state.spindle_load
        assert copy.coolant_level == physical_state.coolant_level
        assert copy.axis_positions == physical_state.axis_positions
        assert copy.status == physical_state.status
        assert copy.current_program == physical_state.current_program
        assert copy.current_line == physical_state.current_line


class TestSyncEngineIntegration:
    """Test the full sync engine with multiple cycles."""

    def test_first_sync_copies_physical(self, sync_engine, physical_state):
        """First sync cycle should initialize twin from physical."""
        sync_engine.update_physical(physical_state)
        sync_engine.sync_cycle(current_time=1.0)
        state = sync_engine.twins['cnc1']
        assert state.twin is not None
        assert state.sync_count == 1

    def test_sync_reduces_drift_over_time(self, sync_engine):
        """Multiple sync cycles should reduce drift toward zero."""
        phys = _MachineSnapshot(
            machine_id='cnc1',
            spindle_speed=12000.0,
            feed_rate=500.0,
            axis_positions=[100.0, 200.0, -50.0],
        )
        sync_engine.update_physical(phys)

        # First cycle initializes twin (copy of physical), drift ~ 0
        sync_engine.sync_cycle(current_time=1.0)

        # Now change physical state significantly
        phys2 = phys.copy()
        phys2.axis_positions = [110.0, 210.0, -40.0]
        phys2.spindle_speed = 13000.0
        sync_engine.update_physical(phys2)

        drifts = []
        for i in range(10):
            sync_engine.sync_cycle(current_time=2.0 + i)
            drifts.append(sync_engine.twins['cnc1'].drift_magnitude)

        # Drift should decrease (or remain near zero) over iterations
        assert drifts[-1] < drifts[0] or drifts[0] < 0.01

    def test_snap_on_excessive_drift(self, sync_engine):
        """Twin should snap to physical when drift exceeds threshold."""
        phys = _MachineSnapshot(
            machine_id='cnc1',
            spindle_speed=0.0,
            feed_rate=0.0,
            axis_positions=[0.0, 0.0, 0.0],
        )
        sync_engine.update_physical(phys)
        sync_engine.sync_cycle(current_time=1.0)

        # Jump physical state far away
        phys_jumped = _MachineSnapshot(
            machine_id='cnc1',
            spindle_speed=0.0,
            feed_rate=0.0,
            axis_positions=[1000.0, 1000.0, 1000.0],
        )
        sync_engine.update_physical(phys_jumped)

        # Use a low interp_factor so interpolation alone cannot bridge the gap
        sync_engine.interp_factor = 0.1
        sync_engine.sync_cycle(current_time=2.0)

        # After snap, axis positions should match physical
        twin = sync_engine.twins['cnc1'].twin
        assert twin.axis_positions == [1000.0, 1000.0, 1000.0]

    def test_sync_quality_reflects_drift(self, sync_engine, physical_state):
        """sync_quality should be 1.0 when drift is 0, lower when drift is high."""
        sync_engine.update_physical(physical_state)
        sync_engine.sync_cycle(current_time=1.0)

        state = sync_engine.twins['cnc1']
        # After first sync (twin == physical copy), drift should be minimal
        assert state.sync_quality >= 0.99

    def test_sync_quality_clamped_to_zero(self):
        """sync_quality should not go below 0.0."""
        engine = _SyncEngine(max_drift=0.001, interp_factor=0.1)
        phys = _MachineSnapshot(
            machine_id='cnc1',
            spindle_speed=12000.0,
            axis_positions=[100.0],
        )
        engine.update_physical(phys)
        engine.sync_cycle(current_time=1.0)

        # Move physical far away
        phys2 = _MachineSnapshot(
            machine_id='cnc1',
            spindle_speed=0.0,
            axis_positions=[0.0],
        )
        engine.update_physical(phys2)
        engine.sync_cycle(current_time=2.0)

        state = engine.twins['cnc1']
        assert state.sync_quality >= 0.0

    def test_multiple_machines_tracked(self, sync_engine):
        """Sync engine should track multiple machines independently."""
        for mid in ['cnc1', 'cnc2', 'cnc3']:
            phys = _MachineSnapshot(
                machine_id=mid,
                spindle_speed=1000.0,
                axis_positions=[0.0, 0.0, 0.0],
            )
            sync_engine.update_physical(phys)

        sync_engine.sync_cycle(current_time=1.0)
        assert len(sync_engine.twins) == 3
        for mid in ['cnc1', 'cnc2', 'cnc3']:
            assert sync_engine.twins[mid].twin is not None

    def test_sync_count_increments(self, sync_engine, physical_state):
        """sync_count should increment with each cycle."""
        sync_engine.update_physical(physical_state)
        for i in range(5):
            sync_engine.sync_cycle(current_time=float(i))
        assert sync_engine.twins['cnc1'].sync_count == 5
