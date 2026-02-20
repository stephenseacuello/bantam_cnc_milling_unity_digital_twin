"""
Threat Response Orchestrator Node.

Coordinates automated responses to security threats.
Implements graduated response based on threat severity.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass
import threading

from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionClient

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from rclpy.callback_groups import ReentrantCallbackGroup

from miracle_msgs.msg import SecurityAlert
from miracle_msgs.action import IsolateNode
from miracle_msgs.srv import IsolateNode as IsolateNodeSrv


@dataclass
class ThreatRecord:
    """Record of an active threat."""
    alert_id: str
    severity: str
    source: str
    response_taken: str = 'NONE'
    isolated: bool = False


class ThreatResponseNode(MiracleLifecycleNode):
    """Automated threat response orchestrator.

    Parameters:
        auto_isolate_critical (bool): Auto-isolate on critical alerts.
        response_delay_sec (float): Delay before automated response.
        max_active_threats (int): Maximum tracked threats.

    Subscribed Topics:
        /miracle/security/alerts (SecurityAlert): Incoming security alerts.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'threat_response',
            criticality=self.CRITICALITY_CRITICAL,
            **kwargs,
        )
        self._alert_sub = None
        self._isolate_client = None
        self._isolate_srv = None
        self._active_threats: Dict[str, ThreatRecord] = {}
        self._isolated_nodes: set = set()
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure threat response."""
        self.declare_and_validate_parameters({
            'auto_isolate_critical': {
                'default': True,
                'type': bool,
            },
            'response_delay_sec': {
                'default': 5.0,
                'type': float,
                'range': (0.0, 60.0),
            },
            'max_active_threats': {
                'default': 100,
                'type': int,
                'range': (10, 10000),
            },
        })

        self._alert_sub = self.create_subscription(
            SecurityAlert,
            '/miracle/security/alerts',
            self._on_alert,
            QoSProfiles.alert(),
        )

        self._isolate_client = ActionClient(
            self,
            IsolateNode,
            '/miracle/security/isolate_node',
        )

        self._isolate_srv = self.create_service(
            IsolateNodeSrv,
            '/miracle/security/isolate_node',
            self._handle_isolate_node,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info("Threat response configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate threat response."""
        self.get_logger().info("Threat response activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate threat response."""
        return TransitionCallbackReturn.SUCCESS

    def _on_alert(self, msg: SecurityAlert) -> None:
        """Handle incoming security alert."""
        threat = ThreatRecord(
            alert_id=msg.alert_id,
            severity=msg.severity,
            source=msg.source_node,
        )

        with self._lock:
            max_threats = self.get_parameter('max_active_threats').value
            if len(self._active_threats) >= max_threats:
                return
            self._active_threats[msg.alert_id] = threat

        # Graduated response
        if msg.severity == 'CRITICAL':
            self._respond_critical(msg, threat)
        elif msg.severity == 'HIGH':
            self._respond_high(msg, threat)
        elif msg.severity == 'MEDIUM':
            self._respond_medium(msg, threat)
        else:
            self._respond_low(msg, threat)

    def _respond_critical(self, alert: SecurityAlert, threat: ThreatRecord) -> None:
        """Critical severity response: immediate isolation."""
        auto_isolate = self.get_parameter('auto_isolate_critical').value
        if auto_isolate and alert.requires_isolation:
            self.get_logger().error(
                f"CRITICAL threat: isolating {alert.source_node}"
            )
            self._request_isolation(alert.source_node, alert.description)
            threat.response_taken = 'ISOLATED'
            threat.isolated = True

    def _respond_high(self, alert: SecurityAlert, threat: ThreatRecord) -> None:
        """High severity response: rate limit and monitor."""
        self.get_logger().warn(
            f"HIGH threat from {alert.source_node}: monitoring"
        )
        threat.response_taken = 'MONITORING'

    def _respond_medium(self, alert: SecurityAlert, threat: ThreatRecord) -> None:
        """Medium severity response: log and alert."""
        self.get_logger().info(
            f"MEDIUM threat from {alert.source_node}: logged"
        )
        threat.response_taken = 'LOGGED'

    def _respond_low(self, alert: SecurityAlert, threat: ThreatRecord) -> None:
        """Low severity response: log only."""
        threat.response_taken = 'LOGGED'

    def _handle_isolate_node(
        self,
        request: IsolateNodeSrv.Request,
        response: IsolateNodeSrv.Response,
    ) -> IsolateNodeSrv.Response:
        """Handle synchronous node isolation requests (dashboard)."""
        self.get_logger().warn(
            f"Isolate request for {request.node_id}: {request.reason}"
        )
        with self._lock:
            self._isolated_nodes.add(request.node_id)
        response.success = True
        response.message = f"Node {request.node_id} isolated"
        return response

    def _request_isolation(self, node_name: str, reason: str) -> None:
        """Request node isolation via action."""
        if not self._isolate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Isolation action server not available")
            return

        goal = IsolateNode.Goal()
        goal.target_node = node_name
        goal.reason = reason
        goal.isolation_level = 'FULL'

        self._isolate_client.send_goal_async(goal)


def main(args=None):
    """Entry point for the threat response node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = ThreatResponseNode()
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
