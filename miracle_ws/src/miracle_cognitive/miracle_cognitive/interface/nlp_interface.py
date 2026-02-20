"""NLP Interface - natural language command processing for manufacturing."""

from typing import Any
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.srv import NLPCommand


class NLPInterfaceNode(MiracleLifecycleNode):
    """Natural language interface for manufacturing commands.

    Parameters:
        confidence_threshold (float): Min confidence to execute command.
    """
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('nlp_interface', criticality=self.CRITICALITY_LOW, **kwargs)
        self._nlp_srv = None
        self._command_patterns = {
            'start': ['start', 'begin', 'run', 'execute'],
            'stop': ['stop', 'halt', 'pause', 'abort'],
            'status': ['status', 'how', 'what', 'check'],
            'optimize': ['optimize', 'improve', 'tune', 'adjust'],
        }

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
        text = request.natural_language_input.lower()
        best_action = 'unknown'
        best_confidence = 0.0
        for action, keywords in self._command_patterns.items():
            score = sum(1 for k in keywords if k in text) / len(keywords)
            if score > best_confidence:
                best_confidence = score
                best_action = action
        threshold = self.get_parameter('confidence_threshold').value
        response.understood = best_confidence >= threshold
        response.interpreted_action = best_action
        response.confidence = best_confidence
        response.clarification_question = '' if response.understood else 'Could you rephrase that?'
        response.parameters = []
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
