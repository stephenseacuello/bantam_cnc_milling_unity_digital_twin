"""
Prediction Runner Node.

Runs what-if scenarios on the digital twin to predict outcomes
of process parameter changes before applying to physical machine.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
import math
import re
import threading
import logging

try:
    from miracle_twin.prediction_validator import PredictionValidatorNode
except ImportError:
    PredictionValidatorNode = None  # type: ignore[assignment,misc]

from miracle_twin.block_telemetry import BlockTelemetryTracker

from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, PHMPrediction
from miracle_msgs.action import RunPrediction
from miracle_msgs.srv import RunPrediction as RunPredictionSrv
from miracle_twin.cutting_sim_proxy import CuttingSimProxy, GCodeBlock, ToolState
from miracle_twin.tool_library import ToolLibrary


@dataclass
class AnomalyMarker:
    """A predictive anomaly marker flagging a potentially problematic G-code block."""
    block_index: int
    gcode_line: str
    marker_type: str       # e.g. "FORCE_CRITICAL", "THERMAL_WARNING"
    severity: float        # 0.0 - 1.0
    predicted_value: float
    threshold: float
    recommendation: str    # e.g. "Reduce feed rate by 20%"


@dataclass
class GCodeParserState:
    """Tracks modal G-code state across parsed lines."""
    absolute_mode: bool = True    # G90 (True) vs G91 (False)
    units_mm: bool = True         # G21 (True) vs G20 inches (False)
    feed_mode_per_min: bool = True  # G94 (True) vs G95 per-rev (False)
    current_x: float = 0.0
    current_y: float = 0.0
    current_z: float = 0.0
    current_feed: float = 100.0
    current_spindle: float = 1000.0
    current_tool: int = 1
    active_plane: str = 'XY'     # G17=XY, G18=ZX, G19=YZ


@dataclass
class PredictionScenario:
    """A prediction scenario configuration."""
    scenario_id: str
    machine_id: str
    parameter_changes: Dict[str, float]
    prediction_horizon_hours: float = 24.0


class PredictionRunnerNode(MiracleLifecycleNode):
    """Runs predictive scenarios on the digital twin.

    Parameters:
        max_concurrent_predictions (int): Max simultaneous predictions.
        default_horizon_hours (float): Default prediction horizon.

    Action Servers:
        ~/run_prediction (RunPrediction): Run a prediction scenario.

    Service Servers:
        /miracle/twin/run_prediction (RunPredictionSrv): Synchronous prediction for dashboard.

    Published Topics:
        ~/predictions (PHMPrediction): Prediction results.
    """

    # Regex patterns for G-code parsing
    _GCODE_LINE_RE = re.compile(
        r'^\s*([Gg]\s*\d+\.?\d*)\s*(.*)', re.IGNORECASE
    )
    _PARAM_RE = re.compile(r'([A-Za-z])\s*([+-]?\d+\.?\d*)')
    _LINE_NUMBER_RE = re.compile(r'^\s*[Nn]\s*\d+\s*')
    _INLINE_COMMENT_RE = re.compile(r'\([^)]*\)')
    _SEMICOLON_COMMENT_RE = re.compile(r';.*$')

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'prediction_runner',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._action_server: Optional[ActionServer] = None
        self._prediction_srv = None
        self._prediction_pub = None
        self._machine_state_sub = None
        self._active_predictions: int = 0
        self._lock = threading.Lock()
        self._latest_machine_state: Optional[MachineState] = None
        self._tool_library = ToolLibrary()
        self._validator: Optional[Any] = None
        self._block_telemetry = BlockTelemetryTracker()
        self._telemetry_block_counter: int = 0

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure prediction runner."""
        params = self.declare_and_validate_parameters({
            'max_concurrent_predictions': {
                'default': 5,
                'type': int,
                'range': (1, 50),
            },
            'default_horizon_hours': {
                'default': 24.0,
                'type': float,
                'range': (0.1, 720.0),
            },
        })

        self._action_server = ActionServer(
            self,
            RunPrediction,
            'run_prediction',
            execute_callback=self._execute_prediction,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self._prediction_pub = self.create_publisher(
            PHMPrediction,
            'predictions',
            QoSProfiles.state_data(),
        )

        self._prediction_srv = self.create_service(
            RunPredictionSrv,
            '/miracle/twin/run_prediction',
            self._handle_run_prediction,
            callback_group=ReentrantCallbackGroup(),
        )

        self._machine_state_sub = self.create_subscription(
            MachineState,
            '/miracle/machine_state',
            self._on_machine_state,
            QoSProfiles.state_data(),
        )

        self.get_logger().info("Prediction runner configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate prediction runner."""
        self.get_logger().info("Prediction runner activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate prediction runner."""
        return TransitionCallbackReturn.SUCCESS

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Accept or reject prediction goals."""
        max_pred = self.get_parameter('max_concurrent_predictions').value
        with self._lock:
            if self._active_predictions >= max_pred:
                return GoalResponse.REJECT
            self._active_predictions += 1
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Accept cancellation."""
        return CancelResponse.ACCEPT

    def _on_machine_state(self, msg: MachineState) -> None:
        """Cache latest machine state for use in predictions."""
        self._latest_machine_state = msg

    def _record_block_telemetry(
        self,
        block_index: int,
        gcode_line: str,
        predicted: dict,
        actual: dict,
    ) -> None:
        """Record a single block's predicted-vs-actual comparison.

        Automatically checks for drift every 50 blocks.
        """
        self._block_telemetry.record(block_index, gcode_line, predicted, actual)
        self._telemetry_block_counter += 1
        if self._telemetry_block_counter % 50 == 0:
            self._check_drift()

    def _check_drift(self) -> None:
        """Log a warning if the model is drifting."""
        report = self._block_telemetry.get_drift_report(window=50)
        if report.is_drifting:
            self.get_logger().warning(
                "Model drift detected: force_trend=%.2f%%/100blk, "
                "temp_trend=%.2f%%/100blk (blocks_since_cal=%d)",
                report.force_drift_trend,
                report.temp_drift_trend,
                report.blocks_since_calibration,
            )
            if report.recalibration_suggested:
                self.get_logger().warning(
                    "Recalibration suggested: drift persisting for >50 "
                    "consecutive checks"
                )

    def set_validator(self, validator: Any) -> None:
        """Attach a PredictionValidatorNode for prediction-vs-actual tracking.

        Args:
            validator: A PredictionValidatorNode instance (or any object
                with a ``register_prediction(machine_id, predictions)`` method).
        """
        self._validator = validator

    @staticmethod
    def _strip_comments(line: str) -> str:
        """Remove inline (...) and end-of-line ; comments from a G-code line."""
        # Remove inline parenthesised comments
        line = PredictionRunnerNode._INLINE_COMMENT_RE.sub('', line)
        # Remove semicolon end-of-line comments
        line = PredictionRunnerNode._SEMICOLON_COMMENT_RE.sub('', line)
        return line.strip()

    @staticmethod
    def _strip_line_number(line: str) -> str:
        """Remove N#### line number prefix."""
        return PredictionRunnerNode._LINE_NUMBER_RE.sub('', line)

    @staticmethod
    def _parse_params(text: str) -> Dict[str, float]:
        """Extract letter-value pairs from a G-code parameter string."""
        params: Dict[str, float] = {}
        for m in PredictionRunnerNode._PARAM_RE.finditer(text):
            params[m.group(1).upper()] = float(m.group(2))
        return params

    @staticmethod
    def _arc_length(
        sx: float, sy: float, ex: float, ey: float,
        ci: float, cj: float, direction: str,
    ) -> float:
        """Compute arc length from start/end points and center offsets."""
        cx = sx + ci
        cy = sy + cj
        radius = math.sqrt((sx - cx) ** 2 + (sy - cy) ** 2)
        if radius <= 0.0:
            return 0.0
        start_angle = math.atan2(sy - cy, sx - cx)
        end_angle = math.atan2(ey - cy, ex - cx)
        if direction == 'CW':
            sweep = start_angle - end_angle
            if sweep <= 0:
                sweep += 2.0 * math.pi
        else:
            sweep = end_angle - start_angle
            if sweep <= 0:
                sweep += 2.0 * math.pi
        return radius * sweep

    def _parse_gcode_to_blocks(
        self,
        gcode_text: str,
        depth_override: Optional[float] = None,
    ) -> List[GCodeBlock]:
        """Parse raw G-code text into a list of GCodeBlock objects.

        Handles G0/G1 linear moves, G2/G3 arcs, G81/G83/G73 canned cycles,
        modal state tracking, comments, line numbers, and tool changes.

        Args:
            gcode_text: Raw G-code program content as a string.
            depth_override: Optional depth-of-cut scaling factor. When
                provided, all Z movements are scaled proportionally
                (e.g. 0.5 = half depth, 2.0 = double depth).

        Returns:
            List of GCodeBlock objects extracted from cutting moves.
            Returns empty list if no valid cutting moves are found.
        """
        blocks: List[GCodeBlock] = []
        state = GCodeParserState()
        logger = logging.getLogger('prediction_runner.gcode_parser')

        # Track the original Z reference for depth scaling
        z_reference: Optional[float] = None

        for raw_line in gcode_text.splitlines():
            # --- Pre-processing ---
            line = self._strip_comments(raw_line)
            line = self._strip_line_number(line)

            # Skip blank lines and program delimiters
            if not line or line == '%':
                continue

            # --- Extract all G/M codes and parameters from line ---
            # A single line can contain multiple G/M words (e.g. "G90 G21")
            all_params = self._parse_params(line)

            # Update modal feed / spindle from any line
            if 'F' in all_params:
                state.current_feed = all_params['F']
            if 'S' in all_params:
                state.current_spindle = all_params['S']

            # Find all G-codes on this line
            g_codes = re.findall(r'[Gg]\s*(\d+\.?\d*)', line)
            g_codes = [g.replace(' ', '') for g in g_codes]

            # Find all M-codes on this line
            m_codes = re.findall(r'[Mm]\s*(\d+)', line)

            # --- Handle M-codes ---
            for m in m_codes:
                m_num = int(m)
                if m_num == 6:
                    # Tool change — update tool number from T word
                    if 'T' in all_params:
                        state.current_tool = int(all_params['T'])
                    logger.debug(
                        "Tool change M6: tool T%d", state.current_tool
                    )
                elif m_num == 98:
                    # Subprogram call — not expanded
                    p_num = all_params.get('P', 0)
                    logger.warning(
                        "Subprogram call M98 P%d not expanded; "
                        "subroutine content will be skipped",
                        int(p_num),
                    )

            # --- Process G-codes ---
            for g_raw in g_codes:
                g_num_str = g_raw.lstrip('0') or '0'
                # Handle decimal G-codes like G28.1
                try:
                    g_val = float(g_raw)
                except ValueError:
                    continue
                g_int = int(g_val)

                # --- Modal state changes ---
                if g_int == 90:
                    state.absolute_mode = True
                    continue
                elif g_int == 91:
                    state.absolute_mode = False
                    continue
                elif g_int == 20:
                    state.units_mm = False
                    continue
                elif g_int == 21:
                    state.units_mm = True
                    continue
                elif g_int == 94:
                    state.feed_mode_per_min = True
                    continue
                elif g_int == 95:
                    state.feed_mode_per_min = False
                    continue
                elif g_int == 17:
                    state.active_plane = 'XY'
                    continue
                elif g_int == 18:
                    state.active_plane = 'ZX'
                    continue
                elif g_int == 19:
                    state.active_plane = 'YZ'
                    continue

                # --- Convert inch coordinates if needed ---
                inch_scale = 25.4 if not state.units_mm else 1.0

                # --- Rapid move G0 ---
                if g_int == 0:
                    if 'X' in all_params:
                        val = all_params['X'] * inch_scale
                        state.current_x = val if state.absolute_mode else state.current_x + val
                    if 'Y' in all_params:
                        val = all_params['Y'] * inch_scale
                        state.current_y = val if state.absolute_mode else state.current_y + val
                    if 'Z' in all_params:
                        val = all_params['Z'] * inch_scale
                        state.current_z = val if state.absolute_mode else state.current_z + val
                    continue

                # --- Linear cutting move G1 ---
                if g_int == 1:
                    prev_x, prev_y, prev_z = state.current_x, state.current_y, state.current_z

                    if 'X' in all_params:
                        val = all_params['X'] * inch_scale
                        state.current_x = val if state.absolute_mode else state.current_x + val
                    if 'Y' in all_params:
                        val = all_params['Y'] * inch_scale
                        state.current_y = val if state.absolute_mode else state.current_y + val
                    if 'Z' in all_params:
                        val = all_params['Z'] * inch_scale
                        if depth_override is not None:
                            if state.absolute_mode:
                                val = val * depth_override
                            else:
                                val = val * depth_override
                        state.current_z = val if state.absolute_mode else state.current_z + val

                    dx = state.current_x - prev_x
                    dy = state.current_y - prev_y
                    dz = state.current_z - prev_z
                    length = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

                    # Axial depth from Z movement
                    axial_depth = 1.5  # default
                    if dz < 0:
                        axial_depth = abs(dz)
                    elif prev_z < 0 and abs(dz) < 1e-9:
                        # Lateral move at depth
                        axial_depth = abs(prev_z) if abs(prev_z) > 0.01 else 1.5

                    if state.current_feed <= 0 or length <= 0.0:
                        continue

                    blocks.append(GCodeBlock(
                        feed_rate_mmpm=state.current_feed,
                        spindle_rpm=state.current_spindle,
                        axial_depth_mm=axial_depth,
                        radial_depth_mm=3.175,
                        length_mm=length,
                    ))
                    continue

                # --- Arc moves G2 (CW) / G3 (CCW) ---
                if g_int in (2, 3):
                    direction = 'CW' if g_int == 2 else 'CCW'
                    prev_x, prev_y = state.current_x, state.current_y

                    end_x = all_params.get('X', state.current_x)
                    end_y = all_params.get('Y', state.current_y)
                    if state.absolute_mode:
                        end_x *= inch_scale
                        end_y *= inch_scale
                    else:
                        end_x = state.current_x + end_x * inch_scale
                        end_y = state.current_y + end_y * inch_scale

                    ci = all_params.get('I', 0.0) * inch_scale
                    cj = all_params.get('J', 0.0) * inch_scale
                    ck = all_params.get('K', 0.0) * inch_scale

                    # R-format arc: compute I, J from radius
                    if 'R' in all_params and 'I' not in all_params:
                        r = all_params['R'] * inch_scale
                        mid_x = (prev_x + end_x) / 2.0
                        mid_y = (prev_y + end_y) / 2.0
                        chord_dx = end_x - prev_x
                        chord_dy = end_y - prev_y
                        chord_len = math.sqrt(chord_dx ** 2 + chord_dy ** 2)
                        if chord_len > 0 and abs(r) >= chord_len / 2.0:
                            h = math.sqrt(max(0, r ** 2 - (chord_len / 2.0) ** 2))
                            # Direction of perpendicular offset
                            sign = -1.0 if (r > 0) == (direction == 'CW') else 1.0
                            cx = mid_x + sign * h * (-chord_dy / chord_len)
                            cy = mid_y + sign * h * (chord_dx / chord_len)
                            ci = cx - prev_x
                            cj = cy - prev_y

                    arc_len = self._arc_length(
                        prev_x, prev_y, end_x, end_y, ci, cj, direction
                    )

                    # Handle Z change during arc
                    if 'Z' in all_params:
                        val = all_params['Z'] * inch_scale
                        if depth_override is not None:
                            val = val * depth_override
                        state.current_z = val if state.absolute_mode else state.current_z + val

                    state.current_x = end_x
                    state.current_y = end_y

                    if state.current_feed <= 0 or arc_len <= 0.0:
                        continue

                    blocks.append(GCodeBlock(
                        feed_rate_mmpm=state.current_feed,
                        spindle_rpm=state.current_spindle,
                        axial_depth_mm=1.5,
                        radial_depth_mm=3.175,
                        length_mm=arc_len,
                        arc_center_i=ci,
                        arc_center_j=cj,
                        arc_center_k=ck,
                        arc_direction=direction,
                        start_x=prev_x,
                        start_y=prev_y,
                        end_x=end_x,
                        end_y=end_y,
                    ))
                    continue

                # --- Canned drilling cycles ---
                if g_int in (81, 83, 73):
                    drill_x = all_params.get('X', state.current_x)
                    drill_y = all_params.get('Y', state.current_y)
                    z_depth = all_params.get('Z', state.current_z) * inch_scale
                    r_plane = all_params.get('R', 0.0) * inch_scale
                    q_peck = all_params.get('Q', 0.0) * inch_scale

                    if depth_override is not None:
                        z_depth = z_depth * depth_override

                    total_depth = abs(r_plane - z_depth)
                    if total_depth <= 0:
                        continue

                    if g_int == 81:
                        # Simple drill: single plunge
                        blocks.append(GCodeBlock(
                            feed_rate_mmpm=state.current_feed,
                            spindle_rpm=state.current_spindle,
                            axial_depth_mm=total_depth,
                            radial_depth_mm=3.175,
                            length_mm=total_depth,
                        ))
                    elif g_int in (83, 73):
                        # Peck drill / chip break: multiple plunge passes
                        if q_peck <= 0:
                            q_peck = total_depth  # degenerate: single pass
                        remaining = total_depth
                        while remaining > 0:
                            peck = min(q_peck, remaining)
                            blocks.append(GCodeBlock(
                                feed_rate_mmpm=state.current_feed,
                                spindle_rpm=state.current_spindle,
                                axial_depth_mm=peck,
                                radial_depth_mm=3.175,
                                length_mm=peck,
                            ))
                            remaining -= peck

                    # Update position
                    state.current_z = z_depth
                    if state.absolute_mode:
                        if 'X' in all_params:
                            state.current_x = drill_x * inch_scale
                        if 'Y' in all_params:
                            state.current_y = drill_y * inch_scale
                    continue

            # If no G-code was found on this line but there are coordinate
            # words, treat as modal continuation of last motion (G1)
            if not g_codes and ('X' in all_params or 'Y' in all_params or 'Z' in all_params):
                prev_x, prev_y, prev_z = state.current_x, state.current_y, state.current_z
                if 'X' in all_params:
                    val = all_params['X'] * (25.4 if not state.units_mm else 1.0)
                    state.current_x = val if state.absolute_mode else state.current_x + val
                if 'Y' in all_params:
                    val = all_params['Y'] * (25.4 if not state.units_mm else 1.0)
                    state.current_y = val if state.absolute_mode else state.current_y + val
                if 'Z' in all_params:
                    val = all_params['Z'] * (25.4 if not state.units_mm else 1.0)
                    if depth_override is not None:
                        val = val * depth_override
                    state.current_z = val if state.absolute_mode else state.current_z + val

                dx = state.current_x - prev_x
                dy = state.current_y - prev_y
                dz = state.current_z - prev_z
                length = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
                axial_depth = abs(dz) if dz < 0 else 1.5
                if state.current_feed > 0 and length > 0.0:
                    blocks.append(GCodeBlock(
                        feed_rate_mmpm=state.current_feed,
                        spindle_rpm=state.current_spindle,
                        axial_depth_mm=axial_depth,
                        radial_depth_mm=3.175,
                        length_mm=length,
                    ))

        return blocks

    def generate_explanation(
        self,
        blocks: List[GCodeBlock],
        parameter_overrides: Optional[Dict[str, float]] = None,
        depth_override: Optional[float] = None,
    ) -> str:
        """Generate a human-readable explanation of a parsed program.

        Args:
            blocks: Parsed G-code blocks.
            parameter_overrides: Optional feed/speed overrides.
            depth_override: Optional depth scaling factor.

        Returns:
            Multi-line summary string.
        """
        overrides = parameter_overrides or {}
        n_linear = sum(1 for b in blocks if not b.is_arc)
        n_arc = sum(1 for b in blocks if b.is_arc)
        total_length = sum(b.length_mm for b in blocks)

        lines = [
            f"Program: {len(blocks)} cutting blocks "
            f"({n_linear} linear, {n_arc} arc)",
            f"Total toolpath length: {total_length:.1f} mm",
        ]
        if overrides:
            lines.append(f"Parameter overrides: {overrides}")
        if depth_override is not None:
            lines.append(f"Depth-of-cut override: {depth_override:.2f}x")
        return '\n'.join(lines)

    def _get_tool_state_from_machine(self) -> Optional[ToolState]:
        """Build a ToolState from cached machine state if available."""
        state = self._latest_machine_state
        if state is None:
            return None
        # Use spindle load as a rough proxy for wear progression
        # spindle_load typically 0-100%; map to flank wear estimate
        load_fraction = max(0.0, min(1.0, state.spindle_load / 100.0))
        estimated_vb = 0.02 + load_fraction * 0.08  # 0.02 to 0.10 mm range
        return ToolState(flank_wear_vb=estimated_vb)

    def _build_blocks_for_machine(self, machine_id: str) -> List[GCodeBlock]:
        """Build simulation blocks from current machine state.

        Uses live spindle speed and feed rate from the MachineState
        subscription to create a representative block sequence.
        Falls back to hardcoded sample blocks if no state is available.

        Args:
            machine_id: Target machine identifier.

        Returns:
            List of GCodeBlock objects for simulation.
        """
        state = self._latest_machine_state
        if (
            state is not None
            and state.machine_id == machine_id
            and state.spindle_speed > 0
            and state.feed_rate > 0
        ):
            self.get_logger().debug(
                f"Using live machine state: RPM={state.spindle_speed}, "
                f"Feed={state.feed_rate}"
            )
            return [
                GCodeBlock(
                    feed_rate_mmpm=state.feed_rate,
                    spindle_rpm=state.spindle_speed,
                    axial_depth_mm=1.5,
                    radial_depth_mm=3.175,
                    length_mm=50.0,
                )
                for _ in range(10)
            ]

        # Fallback: hardcoded sample blocks
        self.get_logger().debug(
            "No live machine state available; using default sample blocks"
        )
        return [
            GCodeBlock(
                feed_rate_mmpm=500.0, spindle_rpm=8000.0,
                axial_depth_mm=1.5, radial_depth_mm=3.175, length_mm=50.0,
            )
            for _ in range(10)
        ]

    async def _execute_prediction(
        self, goal_handle: ServerGoalHandle
    ) -> RunPrediction.Result:
        """Execute a prediction scenario."""
        import asyncio

        request = goal_handle.request
        result = RunPrediction.Result()

        self.get_logger().info(
            f"Running prediction for {request.machine_id}: "
            f"{request.prediction_type}"
        )

        try:
            # Simulate prediction computation
            phases = ['LOADING_MODEL', 'RUNNING_SIMULATION', 'ANALYZING']
            for i, phase in enumerate(phases):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    return result

                feedback = RunPrediction.Feedback()
                feedback.progress = (i + 1) / len(phases)
                feedback.current_phase = phase
                goal_handle.publish_feedback(feedback)
                await asyncio.sleep(0.5)

            # Build simulation blocks from request or machine state
            blocks: List[GCodeBlock] = []

            # Check if request has program_content (from ExecuteProgram)
            program_content = getattr(request, 'program_content', None)
            if program_content:
                blocks = self._parse_gcode_to_blocks(program_content)
                self.get_logger().info(
                    f"Parsed {len(blocks)} cutting blocks from program content"
                )

            # Fall back to live machine state
            if not blocks:
                blocks = self._build_blocks_for_machine(request.machine_id)

            # Extract tool_id from request goals if present
            tool_id = getattr(request, 'tool_id', None) or None

            # Run real simulation via CuttingSimProxy
            proxy = CuttingSimProxy()
            tool_state = self._get_tool_state_from_machine()
            sim_result = proxy.simulate_program(
                blocks, tool_state=tool_state, tool_id=tool_id,
            )

            # Build tool info string for reporting
            tool_info = ''
            if proxy.active_tool is not None:
                td = proxy.active_tool
                tool_info = (
                    f', "tool": {{"id": "{td.tool_id}", '
                    f'"name": "{td.name}", '
                    f'"material": "{td.tool_material}", '
                    f'"diameter_mm": {td.diameter_mm}}}'
                )

            # Identify anomaly risks in simulation results
            anomaly_markers = self._identify_anomaly_risks(blocks, sim_result)
            if anomaly_markers:
                self.get_logger().warning(
                    f"Identified {len(anomaly_markers)} anomaly markers "
                    f"in prediction for {request.machine_id}"
                )

            prediction = PHMPrediction()
            prediction.timestamp = self.get_clock().now().to_msg()
            prediction.machine_id = request.machine_id
            prediction.component = 'spindle'
            prediction.prediction_type = request.prediction_type
            prediction.remaining_useful_life_hours = sim_result.remaining_useful_life_hours
            prediction.confidence = sim_result.confidence
            prediction.health_index = sim_result.health_index
            prediction.recommended_action = sim_result.recommended_action
            prediction.trend_data = sim_result.trend_data

            self._prediction_pub.publish(prediction)

            # Register predictions with the validator for accuracy tracking
            if self._validator is not None:
                peak_force = max(
                    (bp.peak_force_n for bp in sim_result.block_predictions),
                    default=0.0,
                )
                max_temp = max(
                    (bp.temperature_rise_c for bp in sim_result.block_predictions),
                    default=0.0,
                )
                max_power = max(
                    (bp.power_w for bp in sim_result.block_predictions),
                    default=0.0,
                )
                self._validator.register_prediction(
                    request.machine_id,
                    {
                        'force': peak_force,
                        'temperature': max_temp,
                        'power': max_power,
                    },
                )

            # Build anomaly markers JSON for detailed report
            markers_json = ''
            if anomaly_markers:
                import json
                markers_list = [
                    {
                        'block_index': m.block_index,
                        'marker_type': m.marker_type,
                        'severity': round(m.severity, 3),
                        'predicted_value': round(m.predicted_value, 3),
                        'threshold': round(m.threshold, 3),
                        'recommendation': m.recommendation,
                    }
                    for m in anomaly_markers
                ]
                markers_json = f', "anomaly_markers": {json.dumps(markers_list)}'

            goal_handle.succeed()
            result.success = True
            result.prediction = prediction
            result.detailed_report_json = (
                f'{{"status": "completed"{tool_info}{markers_json}}}'
            )

        except Exception as exc:
            self.get_logger().error(f"Prediction error: {exc}")
            goal_handle.abort()
            result.success = False

        finally:
            with self._lock:
                self._active_predictions = max(
                    0, self._active_predictions - 1
                )

        return result


    @staticmethod
    def _generate_recommendation(marker_type: str, predicted_value: float, threshold: float) -> str:
        """Generate actionable recommendation text for an anomaly marker.

        Args:
            marker_type: The anomaly type identifier.
            predicted_value: The predicted metric value.
            threshold: The threshold that was exceeded.

        Returns:
            Human-readable recommendation string.
        """
        recommendations = {
            'FORCE_WARNING': 'Reduce feed rate by 15% or decrease depth of cut',
            'FORCE_CRITICAL': 'Reduce feed rate by 30% and decrease depth of cut immediately',
            'THERMAL_WARNING': 'Increase coolant flow or reduce cutting speed by 10%',
            'THERMAL_CRITICAL': 'Stop and verify coolant system; reduce speed by 25%',
            'WEAR_ACCELERATED': 'Schedule tool replacement; reduce cutting speed to extend life',
            'CHATTER_RISK': 'Adjust spindle speed to avoid stability lobe boundary',
            'TOOL_END_OF_LIFE': 'Replace tool before next operation',
            'SURFACE_QUALITY_WARNING': 'Reduce feed per tooth or verify tool nose radius',
        }
        base = recommendations.get(marker_type, 'Review cutting parameters')
        if threshold > 0 and predicted_value > 0:
            overshoot_pct = ((predicted_value - threshold) / threshold) * 100.0
            if overshoot_pct > 0:
                return f'{base} (exceeded by {overshoot_pct:.0f}%)'
        return base

    @staticmethod
    def _identify_anomaly_risks(
        blocks: List['GCodeBlock'],
        sim_result: 'SimulationResult',
        force_threshold: float = 180.0,
        temp_warning: float = 15.0,
        temp_critical: float = 25.0,
        wear_rate_threshold: float = 0.005,
        chatter_threshold: float = 0.8,
        rul_threshold_min: float = 30.0,
        surface_ra_threshold: float = 3.2,
    ) -> List['AnomalyMarker']:
        """Scan simulation results and flag blocks with anomaly risks.

        Args:
            blocks: The G-code blocks that were simulated.
            sim_result: The SimulationResult from CuttingSimProxy.
            force_threshold: Force threshold in N (default 180N).
            temp_warning: Temperature rise warning threshold in C.
            temp_critical: Temperature rise critical threshold in C.
            wear_rate_threshold: Wear rate threshold in mm/block.
            chatter_threshold: Chatter risk score threshold (0-1) for HIGH.
            rul_threshold_min: RUL threshold in minutes for TOOL_END_OF_LIFE.
            surface_ra_threshold: Surface roughness threshold in um.

        Returns:
            List of AnomalyMarker instances for flagged blocks.
        """
        markers: List[AnomalyMarker] = []
        prev_wear = 0.0
        rul_hours = sim_result.remaining_useful_life_hours
        rul_min = rul_hours * 60.0

        for i, bp in enumerate(sim_result.block_predictions):
            gcode_line = f'block_{i}'
            if i < len(blocks):
                blk = blocks[i]
                gcode_line = (
                    f'G1 F{blk.feed_rate_mmpm:.0f} S{blk.spindle_rpm:.0f} '
                    f'(block {i})'
                )

            # --- Force checks ---
            force_ratio = bp.peak_force_n / force_threshold if force_threshold > 0 else 0.0
            if force_ratio >= 1.0:
                severity = min(1.0, force_ratio - 0.5)  # 1.0 ratio -> 0.5 severity, 1.5 -> 1.0
                markers.append(AnomalyMarker(
                    block_index=i,
                    gcode_line=gcode_line,
                    marker_type='FORCE_CRITICAL',
                    severity=severity,
                    predicted_value=bp.peak_force_n,
                    threshold=force_threshold,
                    recommendation=PredictionRunnerNode._generate_recommendation(
                        'FORCE_CRITICAL', bp.peak_force_n, force_threshold,
                    ),
                ))
            elif force_ratio >= 0.8:
                severity = (force_ratio - 0.8) / 0.2  # 0.8 -> 0.0, 1.0 -> 1.0
                markers.append(AnomalyMarker(
                    block_index=i,
                    gcode_line=gcode_line,
                    marker_type='FORCE_WARNING',
                    severity=severity,
                    predicted_value=bp.peak_force_n,
                    threshold=force_threshold * 0.8,
                    recommendation=PredictionRunnerNode._generate_recommendation(
                        'FORCE_WARNING', bp.peak_force_n, force_threshold * 0.8,
                    ),
                ))

            # --- Temperature checks ---
            if bp.temperature_rise_c > temp_critical:
                severity = min(1.0, bp.temperature_rise_c / (temp_critical * 2.0))
                markers.append(AnomalyMarker(
                    block_index=i,
                    gcode_line=gcode_line,
                    marker_type='THERMAL_CRITICAL',
                    severity=severity,
                    predicted_value=bp.temperature_rise_c,
                    threshold=temp_critical,
                    recommendation=PredictionRunnerNode._generate_recommendation(
                        'THERMAL_CRITICAL', bp.temperature_rise_c, temp_critical,
                    ),
                ))
            elif bp.temperature_rise_c > temp_warning:
                severity = (bp.temperature_rise_c - temp_warning) / (temp_critical - temp_warning)
                severity = min(1.0, max(0.0, severity))
                markers.append(AnomalyMarker(
                    block_index=i,
                    gcode_line=gcode_line,
                    marker_type='THERMAL_WARNING',
                    severity=severity,
                    predicted_value=bp.temperature_rise_c,
                    threshold=temp_warning,
                    recommendation=PredictionRunnerNode._generate_recommendation(
                        'THERMAL_WARNING', bp.temperature_rise_c, temp_warning,
                    ),
                ))

            # --- Wear rate check ---
            wear_rate = bp.wear_after_block_mm - prev_wear
            prev_wear = bp.wear_after_block_mm
            if wear_rate > wear_rate_threshold:
                severity = min(1.0, wear_rate / (wear_rate_threshold * 3.0))
                markers.append(AnomalyMarker(
                    block_index=i,
                    gcode_line=gcode_line,
                    marker_type='WEAR_ACCELERATED',
                    severity=severity,
                    predicted_value=wear_rate,
                    threshold=wear_rate_threshold,
                    recommendation=PredictionRunnerNode._generate_recommendation(
                        'WEAR_ACCELERATED', wear_rate, wear_rate_threshold,
                    ),
                ))

            # --- Chatter risk check (uses block RPM) ---
            if i < len(blocks):
                from miracle_twin.cutting_sim_proxy import CuttingSimProxy
                chatter_score = CuttingSimProxy()._chatter_risk_score(blocks[i].spindle_rpm)
                if chatter_score >= chatter_threshold:
                    markers.append(AnomalyMarker(
                        block_index=i,
                        gcode_line=gcode_line,
                        marker_type='CHATTER_RISK',
                        severity=chatter_score,
                        predicted_value=chatter_score,
                        threshold=chatter_threshold,
                        recommendation=PredictionRunnerNode._generate_recommendation(
                            'CHATTER_RISK', chatter_score, chatter_threshold,
                        ),
                    ))

            # --- Surface roughness check ---
            if bp.surface_ra_um > surface_ra_threshold:
                severity = min(1.0, bp.surface_ra_um / (surface_ra_threshold * 2.0))
                markers.append(AnomalyMarker(
                    block_index=i,
                    gcode_line=gcode_line,
                    marker_type='SURFACE_QUALITY_WARNING',
                    severity=severity,
                    predicted_value=bp.surface_ra_um,
                    threshold=surface_ra_threshold,
                    recommendation=PredictionRunnerNode._generate_recommendation(
                        'SURFACE_QUALITY_WARNING', bp.surface_ra_um, surface_ra_threshold,
                    ),
                ))

        # --- Tool end of life (global check, applied to last block) ---
        if rul_min < rul_threshold_min and len(sim_result.block_predictions) > 0:
            last_idx = len(sim_result.block_predictions) - 1
            gcode_line = f'block_{last_idx}'
            if last_idx < len(blocks):
                blk = blocks[last_idx]
                gcode_line = (
                    f'G1 F{blk.feed_rate_mmpm:.0f} S{blk.spindle_rpm:.0f} '
                    f'(block {last_idx})'
                )
            severity = min(1.0, 1.0 - (rul_min / rul_threshold_min))
            markers.append(AnomalyMarker(
                block_index=last_idx,
                gcode_line=gcode_line,
                marker_type='TOOL_END_OF_LIFE',
                severity=severity,
                predicted_value=rul_min,
                threshold=rul_threshold_min,
                recommendation=PredictionRunnerNode._generate_recommendation(
                    'TOOL_END_OF_LIFE', rul_min, rul_threshold_min,
                ),
            ))

        return markers

    def _handle_run_prediction(
        self,
        request: RunPredictionSrv.Request,
        response: RunPredictionSrv.Response,
    ) -> RunPredictionSrv.Response:
        """Handle synchronous prediction service calls (dashboard)."""
        self.get_logger().info(
            f"Service prediction for {request.machine_id}: "
            f"{request.scenario_type}"
        )

        # Build simulation blocks from request or machine state
        blocks: List[GCodeBlock] = []

        # Check if request has program_content
        program_content = getattr(request, 'program_content', None)
        if program_content:
            blocks = self._parse_gcode_to_blocks(program_content)
            self.get_logger().info(
                f"Parsed {len(blocks)} cutting blocks from program content"
            )

        # Fall back to live machine state
        if not blocks:
            blocks = self._build_blocks_for_machine(request.machine_id)

        # Extract tool_id from request if present
        tool_id = getattr(request, 'tool_id', None) or None

        # Run real simulation via CuttingSimProxy
        proxy = CuttingSimProxy()
        tool_state = self._get_tool_state_from_machine()
        sim_result = proxy.simulate_program(
            blocks, tool_state=tool_state, tool_id=tool_id,
        )

        # Build tool info for reporting
        tool_summary = ''
        tool_json_fragment = ''
        if proxy.active_tool is not None:
            td = proxy.active_tool
            tool_summary = f', Tool={td.name}'
            tool_json_fragment = (
                f', "tool_id": "{td.tool_id}", '
                f'"tool_name": "{td.name}"'
            )

        prediction = PHMPrediction()
        prediction.timestamp = self.get_clock().now().to_msg()
        prediction.machine_id = request.machine_id
        prediction.component = 'spindle'
        prediction.prediction_type = request.scenario_type
        prediction.remaining_useful_life_hours = sim_result.remaining_useful_life_hours
        prediction.confidence = sim_result.confidence
        prediction.health_index = sim_result.health_index
        prediction.recommended_action = sim_result.recommended_action
        prediction.trend_data = sim_result.trend_data

        self._prediction_pub.publish(prediction)

        # Register predictions with the validator for accuracy tracking
        if self._validator is not None:
            peak_force = max(
                (bp.peak_force_n for bp in sim_result.block_predictions),
                default=0.0,
            )
            max_temp = max(
                (bp.temperature_rise_c for bp in sim_result.block_predictions),
                default=0.0,
            )
            max_power = max(
                (bp.power_w for bp in sim_result.block_predictions),
                default=0.0,
            )
            self._validator.register_prediction(
                request.machine_id,
                {
                    'force': peak_force,
                    'temperature': max_temp,
                    'power': max_power,
                },
            )

        response.success = True
        response.summary = (
            f"Prediction for {request.machine_id}: "
            f"RUL={prediction.remaining_useful_life_hours:.0f}h, "
            f"Health={prediction.health_index:.0%}, "
            f"Action: {prediction.recommended_action}"
            f"{tool_summary}"
        )
        response.confidence = prediction.confidence
        response.detailed_report_json = (
            f'{{"machine_id": "{request.machine_id}", '
            f'"rul_hours": {prediction.remaining_useful_life_hours}, '
            f'"health_index": {prediction.health_index}, '
            f'"confidence": {prediction.confidence}'
            f'{tool_json_fragment}}}'
        )
        return response


def main(args=None):
    """Entry point for the prediction runner node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = PredictionRunnerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
