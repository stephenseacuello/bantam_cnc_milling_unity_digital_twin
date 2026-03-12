"""
Prediction Runner Node.

Runs what-if scenarios on the digital twin to predict outcomes
of process parameter changes before applying to physical machine.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass
import re
import threading

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

    # Regex for parsing G-code lines with G0/G1 motion commands
    _GCODE_LINE_RE = re.compile(
        r'^\s*[Nn]?\d*\s*([Gg]\s*[01])\s*(.*)', re.IGNORECASE
    )
    _PARAM_RE = re.compile(r'([A-Za-z])\s*([+-]?\d+\.?\d*)')

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

    def _parse_gcode_to_blocks(self, gcode_text: str) -> List[GCodeBlock]:
        """Parse raw G-code text into a list of GCodeBlock objects.

        Extracts G0/G1 motion commands and their F (feed), S (spindle),
        X/Y/Z coordinate parameters. Depth of cut is inferred from
        negative Z movements.

        Args:
            gcode_text: Raw G-code program content as a string.

        Returns:
            List of GCodeBlock objects extracted from cutting moves.
            Returns empty list if no valid cutting moves are found.
        """
        blocks: List[GCodeBlock] = []

        # Track modal state across lines (G-code is modal)
        current_feed = 0.0
        current_spindle = 0.0
        prev_z: Optional[float] = None

        for line in gcode_text.splitlines():
            line = line.strip()
            if not line or line.startswith('(') or line.startswith('%'):
                continue

            # Check for standalone S command (spindle speed)
            s_match = re.search(r'[Ss]\s*(\d+\.?\d*)', line)
            if s_match:
                current_spindle = float(s_match.group(1))

            # Check for standalone F command (feed rate)
            f_match = re.search(r'[Ff]\s*(\d+\.?\d*)', line)
            if f_match:
                current_feed = float(f_match.group(1))

            # Match G0/G1 motion commands
            motion_match = self._GCODE_LINE_RE.match(line)
            if not motion_match:
                continue

            g_cmd = motion_match.group(1).replace(' ', '').upper()
            params_str = motion_match.group(2)

            # Parse parameters from the rest of the line
            params: Dict[str, float] = {}
            for p_match in self._PARAM_RE.finditer(params_str):
                params[p_match.group(1).upper()] = float(p_match.group(2))

            # Update modal feed/spindle from this line
            if 'F' in params:
                current_feed = params['F']
            if 'S' in params:
                current_spindle = params['S']

            # G0 is rapid — skip for cutting simulation
            if g_cmd == 'G0':
                if 'Z' in params:
                    prev_z = params['Z']
                continue

            # G1 is a cutting move
            x = params.get('X')
            y = params.get('Y')
            z = params.get('Z')

            # Compute segment length from coordinate deltas
            length = 0.0
            if x is not None or y is not None:
                # Approximate segment length from XY; assume small moves
                dx = abs(x) if x is not None else 0.0
                dy = abs(y) if y is not None else 0.0
                length = (dx ** 2 + dy ** 2) ** 0.5
            if length <= 0.0 and z is not None:
                length = abs(z - (prev_z if prev_z is not None else 0.0))

            # Infer axial depth from Z movement (negative Z = deeper cut)
            axial_depth = 1.5  # default
            if z is not None and prev_z is not None:
                dz = prev_z - z  # positive when cutting deeper
                if dz > 0:
                    axial_depth = dz

            if z is not None:
                prev_z = z

            # Skip zero-length or zero-feed moves
            if current_feed <= 0 or length <= 0.0:
                continue

            blocks.append(GCodeBlock(
                feed_rate_mmpm=current_feed,
                spindle_rpm=current_spindle,
                axial_depth_mm=axial_depth,
                radial_depth_mm=3.175,  # default half-tool-diameter
                length_mm=length,
            ))

        return blocks

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

            # Run real simulation via CuttingSimProxy
            proxy = CuttingSimProxy()
            tool_state = self._get_tool_state_from_machine()
            sim_result = proxy.simulate_program(
                blocks, tool_state=tool_state
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

            goal_handle.succeed()
            result.success = True
            result.prediction = prediction
            result.detailed_report_json = '{"status": "completed"}'

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

        # Run real simulation via CuttingSimProxy
        proxy = CuttingSimProxy()
        tool_state = self._get_tool_state_from_machine()
        sim_result = proxy.simulate_program(
            blocks, tool_state=tool_state
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

        response.success = True
        response.summary = (
            f"Prediction for {request.machine_id}: "
            f"RUL={prediction.remaining_useful_life_hours:.0f}h, "
            f"Health={prediction.health_index:.0%}, "
            f"Action: {prediction.recommended_action}"
        )
        response.confidence = prediction.confidence
        response.detailed_report_json = (
            f'{{"machine_id": "{request.machine_id}", '
            f'"rul_hours": {prediction.remaining_useful_life_hours}, '
            f'"health_index": {prediction.health_index}, '
            f'"confidence": {prediction.confidence}}}'
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
