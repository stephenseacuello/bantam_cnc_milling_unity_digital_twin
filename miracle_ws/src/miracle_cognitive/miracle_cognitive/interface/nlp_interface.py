"""NLP Interface - natural language command processing for manufacturing."""

import re
from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.srv import NLPCommand


@dataclass
class IntentPattern:
    """Compiled regex pattern for an intent with optional parameter extraction."""
    intent: str
    pattern: re.Pattern
    param_names: List[str] = field(default_factory=list)


class NLPInterfaceNode(MiracleLifecycleNode):
    """Natural language interface for manufacturing commands.

    Parameters:
        confidence_threshold (float): Min confidence to execute command.
    """
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('nlp_interface', criticality=self.CRITICALITY_LOW, **kwargs)
        self._nlp_srv = None

        # Compiled regex patterns for intent recognition with parameter capture
        self._intent_patterns: List[IntentPattern] = [
            IntentPattern(
                intent='start',
                pattern=re.compile(
                    r'\b(?:start|begin|run|execute|launch)\b'
                    r'(?:\s+(?:machine|cnc|job|program)\s*)?'
                    r'(?:\s+(?P<target>\S+))?',
                    re.IGNORECASE,
                ),
                param_names=['target'],
            ),
            IntentPattern(
                intent='stop',
                pattern=re.compile(
                    r'\b(?:stop|halt|pause|abort|shutdown|kill)\b'
                    r'(?:\s+(?:machine|cnc|job|program)\s*)?'
                    r'(?:\s+(?P<target>\S+))?',
                    re.IGNORECASE,
                ),
                param_names=['target'],
            ),
            IntentPattern(
                intent='status',
                pattern=re.compile(
                    r'\b(?:status|state|check|report|how\s+is|what\s+is)\b'
                    r'(?:\s+(?:of|for)\s+)?'
                    r'(?:(?:machine|cnc|job)\s*)?'
                    r'(?:\s*(?P<target>\S+))?',
                    re.IGNORECASE,
                ),
                param_names=['target'],
            ),
            IntentPattern(
                intent='set_speed',
                pattern=re.compile(
                    r'\b(?:set|change|adjust)\b\s+'
                    r'(?:spindle\s+)?(?:speed|rpm)\s+'
                    r'(?:to\s+)?(?P<value>\d+(?:\.\d+)?)',
                    re.IGNORECASE,
                ),
                param_names=['value'],
            ),
            IntentPattern(
                intent='set_feed',
                pattern=re.compile(
                    r'\b(?:set|change|adjust)\b\s+'
                    r'(?:feed\s*(?:rate)?)\s+'
                    r'(?:to\s+)?(?P<value>\d+(?:\.\d+)?)',
                    re.IGNORECASE,
                ),
                param_names=['value'],
            ),
        ]

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'confidence_threshold': {'default': 0.7, 'type': float, 'range': (0.1, 1.0)},
        })
        self._nlp_srv = self.create_service(
            NLPCommand, 'nlp_command',
            self._handle_command, callback_group=self.service_callback_group,
        )
        self.get_logger().info("NLP interface configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _handle_command(self, request, response):
        text = request.natural_language_input
        best_action = 'unknown'
        best_confidence = 0.0
        extracted_params: List[str] = []

        for ip in self._intent_patterns:
            match = ip.pattern.search(text)
            if match:
                # Full match gives high confidence; partial gives moderate
                matched_text = match.group(0)
                confidence = min(1.0, len(matched_text) / max(len(text), 1) + 0.5)
                if confidence > best_confidence:
                    best_confidence = confidence
                    best_action = ip.intent
                    # Extract named parameters from capture groups
                    extracted_params = []
                    for pname in ip.param_names:
                        val = match.group(pname)
                        if val:
                            extracted_params.append(f"{pname}={val}")

        threshold = self.get_parameter('confidence_threshold').value
        response.understood = best_confidence >= threshold
        response.interpreted_action = best_action
        response.confidence = min(best_confidence, 1.0)
        response.clarification_question = '' if response.understood else 'Could you rephrase that?'
        response.parameters = extracted_params
        return response


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = NLPInterfaceNode()
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
