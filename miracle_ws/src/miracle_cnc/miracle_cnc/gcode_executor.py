"""
G-Code Execution Node.

Parses and executes G-code programs on CNC machines via action server.
Supports line-by-line execution with real-time feedback.
Supports G-code macro/subroutine expansion (O-word, M98, G65).
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
import re
import time as time_module


# ---------------------------------------------------------------------------
# G-code Macro / Subroutine Expansion
# ---------------------------------------------------------------------------

@dataclass
class GCodeMacro:
    """A reusable G-code macro (subroutine).

    Attributes:
        macro_id: Identifier such as "O1000" or a symbolic name.
        name: Human-readable name for the macro.
        parameters: Formal parameter names (e.g. ["#1", "#2", "#3"]).
        body: The G-code lines that make up the macro body.
        description: Optional description of what the macro does.
    """

    macro_id: str
    name: str
    parameters: List[str] = field(default_factory=list)
    body: List[str] = field(default_factory=list)
    description: str = ''


class MacroExpansionError(Exception):
    """Raised when macro expansion fails (e.g. max depth exceeded)."""


# Mapping from G65 letter arguments to numbered parameters.
_G65_PARAM_MAP = {
    'A': '#1', 'B': '#2', 'C': '#3', 'D': '#7', 'E': '#8',
    'F': '#9', 'H': '#11', 'I': '#4', 'J': '#5', 'K': '#6',
    'M': '#13', 'Q': '#17', 'R': '#18', 'S': '#19', 'T': '#20',
    'U': '#21', 'V': '#22', 'W': '#23', 'X': '#24', 'Y': '#25',
    'Z': '#26',
}

_MAX_EXPANSION_DEPTH = 5


class MacroLibrary:
    """Registry of G-code macros with expansion capabilities."""

    def __init__(self) -> None:
        self._macros: Dict[str, GCodeMacro] = {}
        self._register_builtins()

    # -- public API ----------------------------------------------------------

    def register_macro(self, macro: GCodeMacro) -> None:
        """Add or update a macro in the library."""
        self._macros[macro.macro_id] = macro

    def get_macro(self, macro_id: str) -> Optional[GCodeMacro]:
        """Return the macro for *macro_id*, or ``None``."""
        return self._macros.get(macro_id)

    def expand_macro_call(self, line: str, *, _depth: int = 0) -> List[str]:
        """Expand a single G-code line that may contain a macro call.

        Recognised call syntaxes:
        * ``M98 P<id> [L<count>]``  – call O<id>, repeat *count* times.
        * ``G65 P<id> [A<v>] [B<v>] …`` – call with lettered arguments.

        Returns a list of expanded G-code lines.  If the line is not a macro
        call it is returned unchanged as a single-element list.

        Raises:
            MacroExpansionError: if nesting exceeds ``_MAX_EXPANSION_DEPTH``.
        """
        if _depth > _MAX_EXPANSION_DEPTH:
            raise MacroExpansionError(
                f"Macro expansion exceeded maximum depth of {_MAX_EXPANSION_DEPTH}"
            )

        stripped = line.strip()
        upper = stripped.upper()

        # --- M98 call syntax: M98 P<id> [L<count>] -------------------------
        m98_match = re.match(
            r'M98\s+P(\d+)(?:\s+L(\d+))?', upper,
        )
        if m98_match:
            macro_id = f"O{m98_match.group(1)}"
            repeat = int(m98_match.group(2)) if m98_match.group(2) else 1
            macro = self.get_macro(macro_id)
            if macro is None:
                return [line]  # unknown macro – pass through
            expanded: List[str] = []
            for _ in range(repeat):
                for body_line in macro.body:
                    expanded.extend(
                        self.expand_macro_call(body_line, _depth=_depth + 1)
                    )
            return expanded

        # --- G65 call syntax: G65 P<id> [A<v>] [B<v>] … --------------------
        g65_match = re.match(r'G65\s+P(\d+)(.*)', upper)
        if g65_match:
            macro_id = f"O{g65_match.group(1)}"
            args_str = g65_match.group(2)
            macro = self.get_macro(macro_id)
            if macro is None:
                return [line]

            # Parse lettered arguments from the *original* line (preserve case
            # of values but match letters case-insensitively).
            param_values: Dict[str, str] = {}
            for letter_match in re.finditer(r'([A-Za-z])(-?\d+\.?\d*)', args_str):
                letter = letter_match.group(1).upper()
                value = letter_match.group(2)
                param_var = _G65_PARAM_MAP.get(letter)
                if param_var:
                    param_values[param_var] = value

            expanded = []
            for body_line in macro.body:
                substituted = body_line
                for param, value in param_values.items():
                    substituted = substituted.replace(f"[{param}]", value)
                    substituted = substituted.replace(param, value)
                expanded.extend(
                    self.expand_macro_call(substituted, _depth=_depth + 1)
                )
            return expanded

        # Not a macro call – return as-is.
        return [line]

    def parse_macro_definitions(self, program_lines: List[str]) -> List[str]:
        """Extract O-word macro definitions from a program.

        Scans for blocks of the form::

            O<id> (name)       ; or just O<id>
            …body…
            M99

        Each such block is registered as a macro.  The remaining (non-macro)
        lines are returned.
        """
        remaining: List[str] = []
        i = 0
        while i < len(program_lines):
            line = program_lines[i].strip()
            # Match O-word start: O1000 or O1000 (name) or O1000 (name) (desc)
            oword_match = re.match(
                r'^O(\d+)\s*(?:\(([^)]*)\))?\s*(?:\(([^)]*)\))?\s*$',
                line,
                re.IGNORECASE,
            )
            if oword_match:
                macro_id = f"O{oword_match.group(1)}"
                name = (oword_match.group(2) or macro_id).strip()
                description = (oword_match.group(3) or '').strip()
                body: List[str] = []
                i += 1
                # Collect body lines until M99
                while i < len(program_lines):
                    bline = program_lines[i].strip()
                    if bline.upper() == 'M99':
                        i += 1
                        break
                    body.append(bline)
                    i += 1
                # Determine parameters used in body
                params_used = sorted(
                    set(re.findall(r'#\d+', ' '.join(body)))
                )
                macro = GCodeMacro(
                    macro_id=macro_id,
                    name=name,
                    parameters=params_used,
                    body=body,
                    description=description,
                )
                self.register_macro(macro)
            else:
                remaining.append(program_lines[i])
                i += 1
        return remaining

    # -- built-in macros -----------------------------------------------------

    def _register_builtins(self) -> None:
        """Register a set of commonly-used built-in macros."""

        self.register_macro(GCodeMacro(
            macro_id='O9000',
            name='PROBE_Z',
            parameters=['#1', '#2'],
            body=[
                'G91',
                'G38.2 Z[#1] F[#2]',
                'G0 Z2.0',
                'G90',
            ],
            description='G38.2 probe cycle with retract',
        ))

        self.register_macro(GCodeMacro(
            macro_id='O9001',
            name='TOOL_MEASURE',
            parameters=['#1'],
            body=[
                'G91',
                'G38.2 Z-50.0 F100',
                'G43 H[#1]',
                'G0 Z5.0',
                'G90',
            ],
            description='Tool length measurement sequence',
        ))

        self.register_macro(GCodeMacro(
            macro_id='O9002',
            name='SAFE_RETRACT',
            parameters=[],
            body=[
                'G28 G91 Z0',
            ],
            description='G28 G91 Z0 safe retract',
        ))

        self.register_macro(GCodeMacro(
            macro_id='O9003',
            name='PECK_DRILL',
            parameters=['#1', '#2', '#3'],
            body=[
                'G91',
                'G1 Z[#1] F[#3]',
                'G0 Z[#2]',
                'G90',
            ],
            description='Parametric peck drill cycle',
        ))

from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_core.exceptions import GCodeError
from miracle_msgs.msg import GCodeBlock, MachineState, FeedOverride
from miracle_msgs.action import ExecuteProgram
from miracle_msgs.srv import ValidateGCode
from miracle_security.gcode_signer import GCodeSigner, SignatureResult


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
        self._feed_override_pct: float = 100.0
        self._spindle_override_pct: float = 100.0
        self._override_sub = None
        self._macro_library = MacroLibrary()

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
            'require_signed_gcode': {
                'default': False,
                'type': bool,
                'description': 'Refuse to execute unsigned G-code programs',
            },
        })

        self._machine_id = params['machine_id']
        self._max_feed = params['max_feed_rate']
        self._max_spindle = params['max_spindle_speed']
        self._dry_run = params['dry_run']
        self._require_signed = params['require_signed_gcode']
        self._gcode_signer = GCodeSigner()
        self._public_key_pem = self._load_public_key()

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

        # Feed override subscription
        self._override_sub = self.create_subscription(
            FeedOverride,
            f'/miracle/{self._machine_id}/feed_override',
            self._on_feed_override,
            QoSProfiles.command(),
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

    def _load_public_key(self) -> bytes:
        """Load the signing public key from the security directory."""
        import os
        key_paths = [
            os.path.join(os.path.dirname(__file__), '..', '..', 'security', 'gcode_signing.pub'),
            '/miracle_ws/security/gcode_signing.pub',
            os.environ.get('MIRACLE_GCODE_PUBLIC_KEY', ''),
        ]
        for path in key_paths:
            if path and os.path.isfile(path):
                with open(path, 'rb') as f:
                    return f.read()
        return b''

    def _on_feed_override(self, msg: FeedOverride) -> None:
        """Apply adaptive feed override."""
        self._feed_override_pct = max(10.0, min(100.0, msg.feed_override_pct))
        self._spindle_override_pct = max(80.0, min(100.0, msg.spindle_override_pct))
        self.get_logger().info(
            f"Feed override applied: feed={self._feed_override_pct:.0f}%, "
            f"spindle={self._spindle_override_pct:.0f}% ({msg.reason})"
        )

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
            raw_lines = request.program_content.strip().split('\n')

            # Signature verification (runs before macro expansion on raw text)
            if self._public_key_pem:
                sig_result = self._gcode_signer.verify_text(
                    request.program_content, self._public_key_pem
                )
                if sig_result == SignatureResult.INVALID:
                    self.get_logger().error("G-code signature is INVALID — aborting")
                    goal_handle.abort()
                    result.success = False
                    result.message = 'G-code signature verification failed'
                    self._executing = False
                    return result
                elif sig_result == SignatureResult.MISSING and self._require_signed:
                    self.get_logger().error("Unsigned G-code rejected (require_signed_gcode=true)")
                    goal_handle.abort()
                    result.success = False
                    result.message = 'Unsigned G-code not allowed'
                    self._executing = False
                    return result
                elif sig_result == SignatureResult.MISSING:
                    self.get_logger().warn("G-code is unsigned")
                else:
                    self.get_logger().info("G-code signature verified")

            # Macro expansion: extract definitions, then expand calls
            after_defs = self._macro_library.parse_macro_definitions(raw_lines)
            lines: List[str] = []
            for raw_line in after_defs:
                lines.extend(self._macro_library.expand_macro_call(raw_line))

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
        effective_feed = block.feed_rate * (self._feed_override_pct / 100.0)
        if effective_feed > self._max_feed:
            raise GCodeError(
                f"Feed rate {effective_feed:.1f} (override {self._feed_override_pct:.0f}%) exceeds maximum {self._max_feed}"
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

        # Check signature
        if self._public_key_pem:
            sig_result = self._gcode_signer.verify_text(
                request.program_content, self._public_key_pem
            )
            if sig_result == SignatureResult.INVALID:
                response.is_valid = False
                response.errors = ['G-code signature is invalid']
                return response
            elif sig_result == SignatureResult.MISSING:
                warnings.append('G-code is not signed')

        raw_lines = request.program_content.strip().split('\n')
        after_defs = self._macro_library.parse_macro_definitions(raw_lines)
        lines: List[str] = []
        for raw_line in after_defs:
            lines.extend(self._macro_library.expand_macro_call(raw_line))
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
