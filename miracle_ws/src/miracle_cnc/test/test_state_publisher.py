"""Tests for CNC State Publisher simulation, E-stop, and bridge configuration logic.

Extracts the core simulation state machine, E-stop handling, and bridge
parameter configuration from StatePublisherNode and tests them without
any ROS2 dependency.
"""

import math
import pytest
from dataclasses import dataclass, field
from typing import Dict, List, Optional


# ---------------------------------------------------------------------------
# Extracted helpers (mirror the core logic in state_publisher.py)
# ---------------------------------------------------------------------------

@dataclass
class _SimulatedState:
    """Simulated CNC machine state (mirrors StatePublisherNode's sim fields)."""
    machine_id: str = 'cnc1'
    status: str = 'IDLE'
    spindle_speed: float = 0.0
    feed_rate: float = 0.0
    axis_positions: List[float] = field(default_factory=lambda: [0.0] * 6)
    axis_velocities: List[float] = field(default_factory=lambda: [0.0] * 6)
    spindle_load: float = 0.0
    coolant_level: float = 95.0
    current_program: str = ''
    current_line: int = 0
    cycle_time_elapsed: float = 0.0
    cycle_time_remaining: float = 0.0


class _CNCSimulator:
    """Extracted CNC simulation logic from StatePublisherNode.

    Mirrors _generate_simulated_state with tick-based state transitions
    and E-stop handling.
    """

    def __init__(self, machine_id: str = 'cnc1'):
        self.machine_id = machine_id
        self.estopped = False
        self._status = 'IDLE'
        self._spindle = 0.0
        self._feed = 0.0
        self._positions = [0.0] * 6
        self._line = 0
        self._total_lines = 0
        self._tick = 0

    def trigger_estop(self, reason: str = '') -> dict:
        """Handle E-stop (mirrors _handle_estop)."""
        self.estopped = True
        self._status = 'ESTOP'
        self._spindle = 0.0
        self._feed = 0.0
        return {
            'success': True,
            'message': f'E-stop activated for {self.machine_id}',
        }

    def clear_estop(self) -> None:
        """Clear E-stop state."""
        self.estopped = False
        self._status = 'IDLE'

    def set_status(self, status: str) -> None:
        """Force a status (for testing state transitions)."""
        if not self.estopped:
            self._status = status

    def generate_state(self) -> _SimulatedState:
        """Generate one tick of simulated state (mirrors _generate_simulated_state)."""
        self._tick += 1
        t = self._tick * 0.02  # ~50Hz

        state = _SimulatedState()
        state.machine_id = self.machine_id
        state.status = self._status

        if self.estopped:
            state.spindle_speed = 0.0
            state.feed_rate = 0.0
            state.axis_positions = list(self._positions)
            state.axis_velocities = [0.0] * 6
            state.spindle_load = 0.0
            state.coolant_level = max(0.0, 95.0 - t * 0.001)
            state.current_program = ''
            state.current_line = 0
            state.cycle_time_elapsed = 0.0
            state.cycle_time_remaining = 0.0
            return state

        if self._status == 'RUNNING':
            self._spindle = 8000.0 + 500.0 * math.sin(t * 0.1)
            self._feed = 2000.0 + 200.0 * math.sin(t * 0.05)
            self._positions = [
                50.0 * math.sin(t * 0.3),
                30.0 * math.cos(t * 0.2),
                -10.0 + 5.0 * math.sin(t * 0.1),
                0.0, 0.0, 0.0,
            ]
            self._line = min(self._line + 1, 1000)
            self._total_lines = 1000
        else:
            self._spindle = 0.0
            self._feed = 0.0
            self._line = 0

        state.spindle_speed = self._spindle
        state.feed_rate = self._feed
        state.axis_positions = list(self._positions)
        state.axis_velocities = [0.0] * 6
        state.spindle_load = max(0.0, min(100.0, 35.0 + 15.0 * math.sin(t * 0.2)))
        state.coolant_level = max(0.0, 95.0 - t * 0.001)
        state.current_program = 'sim_program.nc' if self._status == 'RUNNING' else ''
        state.current_line = self._line
        state.cycle_time_elapsed = t if self._status == 'RUNNING' else 0.0
        state.cycle_time_remaining = (
            max(0.0, 120.0 - t) if self._status == 'RUNNING' else 0.0
        )

        return state


class _BridgeConfig:
    """Extracted bridge parameter configuration logic.

    Validates bridge_type, host, and port settings as done in
    StatePublisherNode._do_configure.
    """

    VALID_BRIDGE_TYPES = ('modbus', 'opc_ua', 'mtconnect')
    PORT_RANGE = (1, 65535)

    def __init__(self, bridge_type: str = 'modbus',
                 bridge_host: str = '127.0.0.1',
                 bridge_port: int = 502):
        self.bridge_type = bridge_type
        self.bridge_host = bridge_host
        self.bridge_port = bridge_port

    def validate(self) -> List[str]:
        """Validate bridge configuration.

        Returns:
            List of validation error strings (empty if valid).
        """
        errors = []
        if self.bridge_type not in self.VALID_BRIDGE_TYPES:
            errors.append(
                f"Unknown bridge type: {self.bridge_type}. "
                f"Valid types: {self.VALID_BRIDGE_TYPES}"
            )
        if not self.bridge_host:
            errors.append("Bridge host cannot be empty")
        if not (self.PORT_RANGE[0] <= self.bridge_port <= self.PORT_RANGE[1]):
            errors.append(
                f"Bridge port {self.bridge_port} out of range "
                f"{self.PORT_RANGE}"
            )
        return errors

    def get_connection_url(self) -> str:
        """Get the connection URL for the configured bridge type."""
        if self.bridge_type == 'mtconnect':
            return f"http://{self.bridge_host}:{self.bridge_port}/current"
        elif self.bridge_type == 'opc_ua':
            return f"opc.tcp://{self.bridge_host}:{self.bridge_port}"
        else:
            return f"{self.bridge_host}:{self.bridge_port}"


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def simulator():
    """Return a fresh CNC simulator."""
    return _CNCSimulator(machine_id='cnc1')


@pytest.fixture
def bridge_config():
    """Return a default bridge configuration."""
    return _BridgeConfig()


# ---------------------------------------------------------------------------
# Test classes
# ---------------------------------------------------------------------------

class TestSimulationMode:
    """Test simulated state generation."""

    def test_initial_state_is_idle(self, simulator):
        """Simulator should start in IDLE status."""
        state = simulator.generate_state()
        assert state.status == 'IDLE'

    def test_idle_spindle_is_zero(self, simulator):
        """In IDLE, spindle speed and feed should be zero."""
        state = simulator.generate_state()
        assert state.spindle_speed == 0.0
        assert state.feed_rate == 0.0

    def test_running_has_nonzero_spindle(self, simulator):
        """In RUNNING status, spindle speed should be non-zero."""
        simulator.set_status('RUNNING')
        state = simulator.generate_state()
        assert state.spindle_speed > 0.0
        assert state.feed_rate > 0.0

    def test_running_spindle_range(self, simulator):
        """Spindle speed should oscillate around 8000 RPM."""
        simulator.set_status('RUNNING')
        speeds = []
        for _ in range(100):
            state = simulator.generate_state()
            speeds.append(state.spindle_speed)
        assert min(speeds) >= 7000.0
        assert max(speeds) <= 9000.0

    def test_running_feed_rate_range(self, simulator):
        """Feed rate should oscillate around 2000 mm/min."""
        simulator.set_status('RUNNING')
        feeds = []
        for _ in range(100):
            state = simulator.generate_state()
            feeds.append(state.feed_rate)
        assert min(feeds) >= 1700.0
        assert max(feeds) <= 2300.0

    def test_running_increments_line_counter(self, simulator):
        """Line counter should increment during RUNNING."""
        simulator.set_status('RUNNING')
        for _ in range(10):
            state = simulator.generate_state()
        assert state.current_line > 0
        assert state.current_program == 'sim_program.nc'

    def test_line_counter_capped_at_1000(self, simulator):
        """Line counter should cap at 1000."""
        simulator.set_status('RUNNING')
        for _ in range(1500):
            state = simulator.generate_state()
        assert state.current_line == 1000

    def test_idle_resets_line_counter(self, simulator):
        """Switching to IDLE should reset line counter."""
        simulator.set_status('RUNNING')
        for _ in range(10):
            simulator.generate_state()
        simulator.set_status('IDLE')
        state = simulator.generate_state()
        assert state.current_line == 0
        assert state.current_program == ''

    def test_spindle_load_bounded(self, simulator):
        """Spindle load should stay in [0, 100] range."""
        simulator.set_status('RUNNING')
        for _ in range(200):
            state = simulator.generate_state()
            assert 0.0 <= state.spindle_load <= 100.0

    def test_coolant_level_decreases(self, simulator):
        """Coolant level should slowly decrease over time."""
        simulator.set_status('RUNNING')
        first = simulator.generate_state().coolant_level
        for _ in range(500):
            simulator.generate_state()
        last = simulator.generate_state().coolant_level
        assert last <= first

    def test_axis_positions_vary_during_running(self, simulator):
        """Axis positions should vary during RUNNING."""
        simulator.set_status('RUNNING')
        positions = []
        for _ in range(50):
            state = simulator.generate_state()
            positions.append(tuple(state.axis_positions[:3]))
        # Positions should not all be the same
        unique = set(positions)
        assert len(unique) > 1

    def test_machine_id_preserved(self, simulator):
        """Machine ID should be preserved in generated state."""
        state = simulator.generate_state()
        assert state.machine_id == 'cnc1'

    def test_custom_machine_id(self):
        """Custom machine ID should be used."""
        sim = _CNCSimulator(machine_id='cnc_test_42')
        state = sim.generate_state()
        assert state.machine_id == 'cnc_test_42'


class TestEStopHandling:
    """Test E-stop trigger and state behavior."""

    def test_estop_sets_status(self, simulator):
        """E-stop should set status to ESTOP."""
        simulator.set_status('RUNNING')
        simulator.generate_state()
        result = simulator.trigger_estop('test reason')
        assert result['success'] is True
        state = simulator.generate_state()
        assert state.status == 'ESTOP'

    def test_estop_zeroes_spindle(self, simulator):
        """E-stop should zero spindle speed and feed rate."""
        simulator.set_status('RUNNING')
        simulator.generate_state()
        simulator.trigger_estop('emergency')
        state = simulator.generate_state()
        assert state.spindle_speed == 0.0
        assert state.feed_rate == 0.0

    def test_estop_zeroes_spindle_load(self, simulator):
        """E-stop should zero spindle load."""
        simulator.set_status('RUNNING')
        simulator.generate_state()
        simulator.trigger_estop('emergency')
        state = simulator.generate_state()
        assert state.spindle_load == 0.0

    def test_estop_preserves_positions(self, simulator):
        """E-stop should freeze axis positions."""
        simulator.set_status('RUNNING')
        for _ in range(10):
            simulator.generate_state()
        simulator.trigger_estop('test')
        state1 = simulator.generate_state()
        state2 = simulator.generate_state()
        assert state1.axis_positions == state2.axis_positions

    def test_estop_clears_program(self, simulator):
        """E-stop should clear current program info."""
        simulator.set_status('RUNNING')
        for _ in range(5):
            simulator.generate_state()
        simulator.trigger_estop('test')
        state = simulator.generate_state()
        assert state.current_program == ''
        assert state.current_line == 0

    def test_estop_prevents_status_change(self, simulator):
        """While E-stopped, set_status should not change status."""
        simulator.trigger_estop('test')
        simulator.set_status('RUNNING')
        state = simulator.generate_state()
        assert state.status == 'ESTOP'

    def test_estop_clear_restores_idle(self, simulator):
        """Clearing E-stop should restore IDLE status."""
        simulator.trigger_estop('test')
        simulator.clear_estop()
        state = simulator.generate_state()
        assert state.status == 'IDLE'

    def test_estop_clear_allows_running(self, simulator):
        """After clearing E-stop, RUNNING should work again."""
        simulator.trigger_estop('test')
        simulator.clear_estop()
        simulator.set_status('RUNNING')
        state = simulator.generate_state()
        assert state.status == 'RUNNING'
        assert state.spindle_speed > 0.0

    def test_estop_response_message(self, simulator):
        """E-stop response should contain machine ID."""
        result = simulator.trigger_estop('safety check')
        assert 'cnc1' in result['message']

    def test_multiple_estops(self, simulator):
        """Multiple E-stop triggers should be idempotent."""
        simulator.trigger_estop('first')
        simulator.trigger_estop('second')
        assert simulator.estopped is True
        state = simulator.generate_state()
        assert state.status == 'ESTOP'


class TestBridgeParameterConfig:
    """Test bridge configuration validation."""

    def test_default_config_valid(self, bridge_config):
        """Default configuration should be valid."""
        errors = bridge_config.validate()
        assert len(errors) == 0

    def test_modbus_config(self):
        """Modbus configuration should be valid."""
        cfg = _BridgeConfig(bridge_type='modbus', bridge_host='192.168.1.10', bridge_port=502)
        assert len(cfg.validate()) == 0

    def test_opcua_config(self):
        """OPC-UA configuration should be valid."""
        cfg = _BridgeConfig(bridge_type='opc_ua', bridge_host='10.0.0.1', bridge_port=4840)
        assert len(cfg.validate()) == 0

    def test_mtconnect_config(self):
        """MTConnect configuration should be valid."""
        cfg = _BridgeConfig(bridge_type='mtconnect', bridge_host='localhost', bridge_port=5000)
        assert len(cfg.validate()) == 0

    def test_invalid_bridge_type(self):
        """Unknown bridge type should fail validation."""
        cfg = _BridgeConfig(bridge_type='grpc')
        errors = cfg.validate()
        assert any('Unknown bridge type' in e for e in errors)

    def test_empty_host_fails(self):
        """Empty host should fail validation."""
        cfg = _BridgeConfig(bridge_host='')
        errors = cfg.validate()
        assert any('host' in e.lower() for e in errors)

    def test_port_too_low(self):
        """Port below 1 should fail validation."""
        cfg = _BridgeConfig(bridge_port=0)
        errors = cfg.validate()
        assert any('port' in e.lower() for e in errors)

    def test_port_too_high(self):
        """Port above 65535 should fail validation."""
        cfg = _BridgeConfig(bridge_port=70000)
        errors = cfg.validate()
        assert any('port' in e.lower() for e in errors)

    def test_mtconnect_url(self):
        """MTConnect connection URL should be HTTP."""
        cfg = _BridgeConfig(bridge_type='mtconnect', bridge_host='10.0.0.5', bridge_port=5000)
        url = cfg.get_connection_url()
        assert url == 'http://10.0.0.5:5000/current'

    def test_opcua_url(self):
        """OPC-UA connection URL should use opc.tcp scheme."""
        cfg = _BridgeConfig(bridge_type='opc_ua', bridge_host='10.0.0.1', bridge_port=4840)
        url = cfg.get_connection_url()
        assert url == 'opc.tcp://10.0.0.1:4840'

    def test_modbus_url(self):
        """Modbus URL should be host:port."""
        cfg = _BridgeConfig(bridge_type='modbus', bridge_host='192.168.1.1', bridge_port=502)
        url = cfg.get_connection_url()
        assert url == '192.168.1.1:502'


class TestStateTransitions:
    """Test state transition sequences."""

    def test_idle_to_running_transition(self, simulator):
        """Transitioning from IDLE to RUNNING should start producing data."""
        state = simulator.generate_state()
        assert state.spindle_speed == 0.0
        simulator.set_status('RUNNING')
        state = simulator.generate_state()
        assert state.spindle_speed > 0.0

    def test_running_to_paused_transition(self, simulator):
        """Transitioning to PAUSED should keep spindle at zero."""
        simulator.set_status('RUNNING')
        simulator.generate_state()
        simulator.set_status('PAUSED')
        state = simulator.generate_state()
        assert state.spindle_speed == 0.0
        assert state.status == 'PAUSED'

    def test_full_lifecycle(self, simulator):
        """IDLE -> RUNNING -> PAUSED -> RUNNING -> IDLE lifecycle."""
        assert simulator.generate_state().status == 'IDLE'
        simulator.set_status('RUNNING')
        state = simulator.generate_state()
        assert state.status == 'RUNNING'
        assert state.spindle_speed > 0
        simulator.set_status('PAUSED')
        assert simulator.generate_state().status == 'PAUSED'
        simulator.set_status('RUNNING')
        state = simulator.generate_state()
        assert state.status == 'RUNNING'
        assert state.spindle_speed > 0
        simulator.set_status('IDLE')
        state = simulator.generate_state()
        assert state.status == 'IDLE'
        assert state.spindle_speed == 0.0
