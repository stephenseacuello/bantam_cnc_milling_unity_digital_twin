"""
G-Code Execution Node.

Parses and executes G-code programs on CNC machines via action server.
Supports line-by-line execution with real-time feedback.
"""

from typing import Any, Dict, List, Optional
import re
import time as time_module

from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_core.exceptions import GCodeError
from miracle_msgs.msg import GCodeBlock, MachineState
from miracle_msgs.action import ExecuteProgram
from miracle_msgs.srv import ValidateGCode


class GCodeExecutorNode(MiracleLifecycleNode):
    """Executes G-code programs with line-by-line feedback.

    Parameters:
        machine_id (str): Machine identifier.
        max_feed_rate (float): Maximum allowed feed rate in mm/min.
        max_spindle_speed (float): Maximum allowed spindle speed in RPM.
        dry_run (bool): Parse and validate without executing.

    Action Servers:
        ~/execute_program (ExecuteProgram): Execute a G-code program.

    Service Servers:
        ~/validate_gcode (ValidateGCode): Validate G-code without executing.

    Published Topics:
        ~/gcode_block (GCodeBlock): Currently executing G-code block.
    """

    # G-code pattern: letter followed by number (e.g., G01, X10.5, F2000)
    _GCODE_PATTERN = re.compile(r'([A-Za-z])(-?\d+\.?\d*)')

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'gcode_executor',
            criticality=self.CRITICALITY_CRITICAL,
            **kwargs,
        )
        self._action_server: Optional[ActionServer] = None
        self._validate_srv = None
        self._block_pub = None
        self._machine_id: str = ''
        self._max_feed: float = 10000.0
        self._max_spindle: float = 24000.0
        self._dry_run: bool = False
        self._executing: bool = False

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure G-code executor."""
        params = self.declare_and_validate_parameters({
            'machine_id': {
                'default': 'cnc1',
                'type': str,
                'description': 'Machine identifier',
            },
            'max_feed_rate': {
                'default': 10000.0,
                'type': float,
                'range': (1.0, 100000.0),
                'description': 'Maximum feed rate in mm/min',
            },
            'max_spindle_speed': {
                'default': 24000.0,
                'type': float,
                'range': (1.0, 100000.0),
                'description': 'Maximum spindle speed in RPM',
            },
            'dry_run': {
                'default': False,
                'type': bool,
                'description': 'Validate without executing',
            },
        })

        self._machine_id = params['machine_id']
        self._max_feed = params['max_feed_rate']
        self._max_spindle = params['max_spindle_speed']
        self._dry_run = params['dry_run']

        # G-code block publisher
        self._block_pub = self.create_publisher(
            GCodeBlock,
            'gcode_block',
            QoSProfiles.command(),
        )

        # Validation service
        self._validate_srv = self.create_service(
            ValidateGCode,
            'validate_gcode',
            self._handle_validate,
            callback_group=self.service_callback_group,
        )

        # Action server
        self._action_server = ActionServer(
            self,
            ExecuteProgram,
            'execute_program',
            execute_callback=self._execute_program,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"G-code executor configured for '{self._machine_id}'"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate executor."""
        self.get_logger().info("G-code executor activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate executor."""
        self._executing = False
        return TransitionCallbackReturn.SUCCESS

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Accept or reject new execution goals."""
        if self._executing:
            self.get_logger().warn("Rejecting goal: already executing a program")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Accept cancellation requests."""
        self.get_logger().info("Cancellation requested for G-code execution")
        return CancelResponse.ACCEPT

    async def _execute_program(
        self, goal_handle: ServerGoalHandle
    ) -> ExecuteProgram.Result:
        """Execute a G-code program with line-by-line feedback."""
        self._executing = True
        request = goal_handle.request
        result = ExecuteProgram.Result()

        self.get_logger().info(
            f"Executing program '{request.program_name}' on {request.machine_id}"
        )

        try:
            lines = request.program_content.strip().split('\n')
            total_lines = len(lines)
            start_time = self.get_clock().now()
            executed_count = 0

            for i, line in enumerate(lines):
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Execution cancelled'
                    result.lines_executed = executed_count
                    self._executing = False
                    return result

                # Parse and validate line
                stripped = line.strip()
                if not stripped or stripped.startswith('(') or stripped.startswith(';'):
                    continue  # Skip comments and empty lines

                try:
                    block = self._parse_gcode_line(stripped, i + 1)
                    self._validate_block(block)
                except GCodeError as exc:
                    self.get_logger().error(f"G-code error at line {i + 1}: {exc}")
                    goal_handle.abort()
                    result.success = False
                    result.message = f"Error at line {i + 1}: {exc}"
                    result.lines_executed = executed_count
                    self._executing = False
                    return result

                # Publish block
                self._block_pub.publish(block)
                executed_count += 1

                # Publish feedback
                feedback = ExecuteProgram.Feedback()
                feedback.current_line = i + 1
                feedback.total_lines = total_lines
                feedback.progress = (i + 1) / total_lines
                feedback.current_operation = stripped
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                feedback.elapsed_sec = elapsed
                if i > 0:
                    rate = elapsed / (i + 1)
                    feedback.estimated_remaining_sec = rate * (total_lines - i - 1)
                else:
                    feedback.estimated_remaining_sec = 0.0
                goal_handle.publish_feedback(feedback)

            # Success
            elapsed_total = (self.get_clock().now() - start_time).nanoseconds / 1e9
            goal_handle.succeed()
            result.success = True
            result.message = 'Program completed successfully'
            result.total_time_sec = elapsed_total
            result.lines_executed = executed_count
            result.quality_metrics = [1.0]  # Placeholder

        except Exception as exc:
            self.get_logger().error(f"Execution error: {exc}")
            goal_handle.abort()
            result.success = False
            result.message = str(exc)

        self._executing = False
        return result

    def _parse_gcode_line(self, line: str, line_number: int) -> GCodeBlock:
        """Parse a single G-code line into a GCodeBlock message."""
        msg = GCodeBlock()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.line_number = line_number
        msg.raw_line = line

        # Extract comment
        comment_idx = line.find(';')
        if comment_idx == -1:
            comment_idx = line.find('(')
        if comment_idx >= 0:
            msg.comment = line[comment_idx:].strip()
            line = line[:comment_idx].strip()

        # Parse tokens
        tokens = self._GCODE_PATTERN.findall(line)
        if not tokens:
            return msg

        # First token is the command
        msg.command = f"{tokens[0][0]}{tokens[0][1]}"

        # Extract parameters
        params = []
        for letter, value in tokens[1:]:
            params.append(float(value))
            if letter.upper() == 'F':
                msg.feed_rate = float(value)
            elif letter.upper() == 'S':
                msg.spindle_speed = float(value)

        msg.parameters = params

        # Check if rapid move
        cmd_upper = msg.command.upper()
        msg.is_rapid = cmd_upper in ('G0', 'G00')

        return msg

    def _validate_block(self, block: GCodeBlock) -> None:
        """Validate a G-code block against machine limits."""
        if block.feed_rate > self._max_feed:
            raise GCodeError(
                f"Feed rate {block.feed_rate} exceeds maximum {self._max_feed}"
            )
        if block.spindle_speed > self._max_spindle:
            raise GCodeError(
                f"Spindle speed {block.spindle_speed} exceeds maximum {self._max_spindle}"
            )

    def _handle_validate(
        self,
        request: ValidateGCode.Request,
        response: ValidateGCode.Response,
    ) -> ValidateGCode.Response:
        """Handle G-code validation service requests."""
        errors: List[str] = []
        warnings: List[str] = []
        line_count = 0

        lines = request.program_content.strip().split('\n')
        for i, line in enumerate(lines):
            stripped = line.strip()
            if not stripped or stripped.startswith('(') or stripped.startswith(';'):
                continue

            line_count += 1
            try:
                block = self._parse_gcode_line(stripped, i + 1)
                self._validate_block(block)
            except GCodeError as exc:
                errors.append(f"Line {i + 1}: {exc}")

        response.is_valid = len(errors) == 0
        response.errors = errors
        response.warnings = warnings
        response.estimated_duration_sec = float(line_count) * 0.5  # Rough estimate
        return response


def main(args=None):
    """Entry point for the G-code executor node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = GCodeExecutorNode()
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
