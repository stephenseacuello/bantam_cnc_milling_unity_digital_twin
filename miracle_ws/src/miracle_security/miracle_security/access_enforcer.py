"""
Access Control Enforcer Node.

Enforces role-based access control (RBAC) for ROS2 topics
and services. Validates permissions before allowing operations.
"""

from typing import Any, Dict, List, Set
from dataclasses import dataclass, field
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import SecurityAlert


@dataclass
class AccessPolicy:
    """Access control policy."""
    role: str
    allowed_topics: Set[str] = field(default_factory=set)
    allowed_services: Set[str] = field(default_factory=set)
    denied_topics: Set[str] = field(default_factory=set)


class AccessEnforcerNode(MiracleLifecycleNode):
    """Role-based access control for ROS2 resources.

    Parameters:
        default_policy (str): Default access policy (allow/deny).
        audit_all_access (bool): Log all access checks.

    Published Topics:
        /miracle/security/alerts (SecurityAlert): Access violation alerts.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'access_enforcer',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._policies: Dict[str, AccessPolicy] = {}
        self._alert_pub = None
        self._lock = threading.Lock()
        self._violation_count: int = 0

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure access enforcer."""
        self.declare_and_validate_parameters({
            'default_policy': {
                'default': 'deny',
                'type': str,
                'choices': ['allow', 'deny'],
            },
            'audit_all_access': {
                'default': False,
                'type': bool,
            },
        })

        self._alert_pub = self.create_publisher(
            SecurityAlert,
            '/miracle/security/alerts',
            QoSProfiles.alert(),
        )

        # Initialize default policies
        self._policies['operator'] = AccessPolicy(
            role='operator',
            allowed_topics={'/miracle/+/state', '/miracle/system_kpis'},
            allowed_services={'/miracle/mes/submit_task'},
        )
        self._policies['engineer'] = AccessPolicy(
            role='engineer',
            allowed_topics={'*'},
            allowed_services={'*'},
        )
        self._policies['viewer'] = AccessPolicy(
            role='viewer',
            allowed_topics={'/miracle/+/state', '/miracle/system_kpis'},
        )

        self.get_logger().info("Access enforcer configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate access enforcement."""
        self.get_logger().info("Access enforcer activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate access enforcement."""
        return TransitionCallbackReturn.SUCCESS

    def check_access(
        self, role: str, resource: str, resource_type: str = 'topic'
    ) -> bool:
        """Check if a role has access to a resource.

        Args:
            role: User/node role.
            resource: Topic or service name.
            resource_type: 'topic' or 'service'.

        Returns:
            True if access is permitted.
        """
        with self._lock:
            policy = self._policies.get(role)
            if policy is None:
                default = self.get_parameter('default_policy').value
                if default == 'deny':
                    self._report_violation(role, resource)
                    return False
                return True

            if resource_type == 'topic':
                allowed = policy.allowed_topics
                denied = policy.denied_topics
            else:
                allowed = policy.allowed_services
                denied = set()

            if resource in denied:
                self._report_violation(role, resource)
                return False
            if '*' in allowed or resource in allowed:
                return True

            default = self.get_parameter('default_policy').value
            if default == 'deny':
                self._report_violation(role, resource)
                return False
            return True

    def _report_violation(self, role: str, resource: str) -> None:
        """Report an access violation."""
        self._violation_count += 1
        msg = SecurityAlert()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.alert_id = f"ACL-{self._violation_count:06d}"
        msg.severity = 'MEDIUM'
        msg.category = 'ACCESS_VIOLATION'
        msg.source_node = role
        msg.description = f"Access denied: role '{role}' to '{resource}'"
        msg.recommended_action = 'Review access policies'
        msg.requires_isolation = False
        msg.confidence = 1.0

        self._alert_pub.publish(msg)
        self.get_logger().warn(f"Access violation: {role} -> {resource}")


def main(args=None):
    """Entry point for the access enforcer node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = AccessEnforcerNode()
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
