"""Human Escalation - manages escalation of decisions requiring human approval."""

from typing import Any, Dict
from dataclasses import dataclass
import threading
import uuid
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode


@dataclass
class EscalationRequest:
    request_id: str
    reason: str
    urgency: str
    context: str
    status: str = 'PENDING'
    response: str = ''


class HumanEscalationNode(MiracleLifecycleNode):
    """Manages human-in-the-loop escalation."""
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('human_escalation', criticality=self.CRITICALITY_HIGH, **kwargs)
        self._pending: Dict[str, EscalationRequest] = {}
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'escalation_timeout_sec': {'default': 300.0, 'type': float, 'range': (30.0, 3600.0)},
            'auto_approve_low_risk': {'default': False, 'type': bool},
        })
        self.get_logger().info("Human escalation configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def escalate(self, reason: str, urgency: str, context: str) -> str:
        req_id = str(uuid.uuid4())[:8]
        req = EscalationRequest(
            request_id=req_id, reason=reason, urgency=urgency, context=context,
        )
        with self._lock:
            self._pending[req_id] = req
        self.get_logger().warn(f"ESCALATION [{urgency}]: {reason}")
        return req_id

    def respond(self, request_id: str, approved: bool, response: str = '') -> bool:
        with self._lock:
            if request_id in self._pending:
                req = self._pending[request_id]
                req.status = 'APPROVED' if approved else 'REJECTED'
                req.response = response
                return True
        return False


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = HumanEscalationNode()
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
