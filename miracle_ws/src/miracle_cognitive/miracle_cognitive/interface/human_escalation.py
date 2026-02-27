"""Human Escalation - manages escalation of decisions requiring human approval."""

from typing import Any, Dict
from dataclasses import dataclass, field
import time
import threading
import uuid
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import SecurityAlert


@dataclass
class EscalationRequest:
    request_id: str
    reason: str
    urgency: str
    context: str
    status: str = 'PENDING'
    response: str = ''
    created_at: float = 0.0


class HumanEscalationNode(MiracleLifecycleNode):
    """Manages human-in-the-loop escalation with timeout enforcement."""
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('human_escalation', criticality=self.CRITICALITY_HIGH, **kwargs)
        self._pending: Dict[str, EscalationRequest] = {}
        self._timeout_timers: Dict[str, threading.Timer] = {}
        self._lock = threading.Lock()
        self._notification_pub = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'escalation_timeout_sec': {'default': 300.0, 'type': float, 'range': (30.0, 3600.0)},
            'auto_approve_low_risk': {'default': False, 'type': bool},
        })

        self._notification_pub = self.create_publisher(
            SecurityAlert,
            '/miracle/cognitive/escalation_notifications',
            QoSProfiles.alert(),
        )

        self.get_logger().info("Human escalation configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        # Cancel all pending timeout timers
        with self._lock:
            for timer in self._timeout_timers.values():
                timer.cancel()
            self._timeout_timers.clear()
        return TransitionCallbackReturn.SUCCESS

    def escalate(self, reason: str, urgency: str, context: str) -> str:
        """Create an escalation request with timeout enforcement.

        Args:
            reason: Why escalation is needed.
            urgency: Urgency level (LOW, MEDIUM, HIGH, CRITICAL).
            context: Additional context for the human operator.

        Returns:
            The unique request ID.
        """
        timeout_sec = self.get_parameter('escalation_timeout_sec').value
        auto_approve = self.get_parameter('auto_approve_low_risk').value

        req_id = str(uuid.uuid4())[:8]
        req = EscalationRequest(
            request_id=req_id, reason=reason, urgency=urgency,
            context=context, created_at=time.monotonic(),
        )

        # Auto-approve low-risk escalations if configured
        if auto_approve and urgency == 'LOW':
            req.status = 'AUTO_APPROVED'
            req.response = 'Automatically approved (low risk)'
            self.get_logger().info(
                f"ESCALATION [{urgency}] auto-approved: {reason}"
            )
            self._publish_notification(req_id, urgency, reason, 'AUTO_APPROVED')
            return req_id

        with self._lock:
            self._pending[req_id] = req

            # Start timeout timer for this escalation
            timer = threading.Timer(
                timeout_sec, self._on_timeout, args=[req_id]
            )
            timer.daemon = True
            timer.start()
            self._timeout_timers[req_id] = timer

        self.get_logger().warn(
            f"ESCALATION [{urgency}]: {reason} (timeout={timeout_sec}s)"
        )
        self._publish_notification(req_id, urgency, reason, 'PENDING')
        return req_id

    def respond(self, request_id: str, approved: bool, response: str = '') -> bool:
        """Respond to an escalation request.

        Args:
            request_id: The escalation request ID.
            approved: Whether the request is approved.
            response: Optional response message.

        Returns:
            True if the response was recorded.
        """
        with self._lock:
            if request_id not in self._pending:
                return False

            req = self._pending[request_id]
            if req.status != 'PENDING':
                return False  # Already resolved (timed out or responded)

            req.status = 'APPROVED' if approved else 'REJECTED'
            req.response = response

            # Cancel the timeout timer
            timer = self._timeout_timers.pop(request_id, None)
            if timer is not None:
                timer.cancel()

        status = 'APPROVED' if approved else 'REJECTED'
        self.get_logger().info(
            f"Escalation {request_id} {status}: {response}"
        )
        self._publish_notification(
            request_id, req.urgency, req.reason, status
        )
        return True

    def _on_timeout(self, request_id: str) -> None:
        """Handle escalation timeout — mark as timed out and notify."""
        with self._lock:
            if request_id not in self._pending:
                return

            req = self._pending[request_id]
            if req.status != 'PENDING':
                return  # Already resolved

            elapsed = time.monotonic() - req.created_at
            req.status = 'TIMED_OUT'
            req.response = f'No response within {elapsed:.0f}s'
            self._timeout_timers.pop(request_id, None)

        self.get_logger().error(
            f"ESCALATION TIMEOUT [{req.urgency}]: {req.reason} "
            f"(request {request_id}, {elapsed:.0f}s elapsed)"
        )
        self._publish_notification(
            request_id, req.urgency, req.reason, 'TIMED_OUT'
        )

    def _publish_notification(self, request_id: str, urgency: str,
                              reason: str, status: str) -> None:
        """Publish an escalation notification to the notification topic."""
        if self._notification_pub is None:
            return

        try:
            msg = SecurityAlert()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.alert_id = f"ESC-{request_id}"
            msg.severity = urgency
            msg.category = f'ESCALATION_{status}'
            msg.source_node = 'human_escalation'
            msg.description = (
                f"Escalation {request_id} [{urgency}]: {reason} — {status}"
            )
            msg.recommended_action = (
                'Awaiting human response' if status == 'PENDING'
                else f'Escalation {status.lower()}'
            )
            msg.requires_isolation = (status == 'TIMED_OUT' and urgency in ('HIGH', 'CRITICAL'))
            msg.confidence = 1.0

            self._notification_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to publish notification: {exc}")

    def get_pending_count(self) -> int:
        """Return the number of pending escalation requests."""
        with self._lock:
            return sum(1 for r in self._pending.values() if r.status == 'PENDING')


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
