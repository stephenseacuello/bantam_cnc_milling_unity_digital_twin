"""
Security Audit Logger Node.

Maintains tamper-evident audit log of all security events,
access attempts, and configuration changes for compliance.
"""

from typing import Any, Dict, List
import json
import hashlib
import os
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import SecurityAlert


class AuditLoggerNode(MiracleLifecycleNode):
    """Tamper-evident security audit logging.

    Parameters:
        log_directory (str): Directory for audit log files.
        max_entries_per_file (int): Entries per log file.
        enable_hash_chain (bool): Enable hash chain for tamper evidence.

    Subscribed Topics:
        /miracle/security/alerts (SecurityAlert): Security events to log.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'audit_logger',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._alert_sub = None
        self._log_dir: str = ''
        self._current_file = None
        self._entry_count: int = 0
        self._file_count: int = 0
        self._last_hash: str = '0' * 64
        self._max_per_file: int = 10000
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure audit logger."""
        params = self.declare_and_validate_parameters({
            'log_directory': {
                'default': '/tmp/miracle_audit',
                'type': str,
            },
            'max_entries_per_file': {
                'default': 10000,
                'type': int,
                'range': (100, 1000000),
            },
            'enable_hash_chain': {
                'default': True,
                'type': bool,
            },
        })

        self._log_dir = params['log_directory']
        self._max_per_file = params['max_entries_per_file']
        os.makedirs(self._log_dir, exist_ok=True)

        self._alert_sub = self.create_subscription(
            SecurityAlert,
            '/miracle/security/alerts',
            self._on_alert,
            QoSProfiles.alert(),
        )

        self.get_logger().info(f"Audit logger configured (dir={self._log_dir})")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate audit logging."""
        self._rotate_file()
        self.get_logger().info("Audit logger activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Close audit log."""
        with self._lock:
            if self._current_file is not None:
                self._current_file.close()
                self._current_file = None
        return TransitionCallbackReturn.SUCCESS

    def _on_alert(self, msg: SecurityAlert) -> None:
        """Log security alert."""
        entry = {
            'timestamp': msg.timestamp.sec + msg.timestamp.nanosec * 1e-9,
            'alert_id': msg.alert_id,
            'severity': msg.severity,
            'category': msg.category,
            'source': msg.source_node,
            'description': msg.description,
            'affected': list(msg.affected_nodes),
            'action': msg.recommended_action,
        }

        enable_chain = self.get_parameter('enable_hash_chain').value
        if enable_chain:
            chain_input = f"{self._last_hash}:{json.dumps(entry, sort_keys=True)}"
            entry['chain_hash'] = hashlib.sha256(chain_input.encode()).hexdigest()
            self._last_hash = entry['chain_hash']

        self._write_entry(entry)

    def _write_entry(self, entry: Dict) -> None:
        """Write entry to audit log."""
        with self._lock:
            if self._current_file is None:
                self._rotate_file()

            try:
                self._current_file.write(json.dumps(entry) + '\n')
                self._current_file.flush()
                self._entry_count += 1

                if self._entry_count >= self._max_per_file:
                    self._rotate_file()
            except Exception as exc:
                self.get_logger().error(f"Audit write error: {exc}")

    def _rotate_file(self) -> None:
        """Rotate to a new audit log file."""
        if self._current_file is not None:
            self._current_file.close()

        self._file_count += 1
        self._entry_count = 0
        filepath = os.path.join(
            self._log_dir,
            f"audit_{self._file_count:06d}.jsonl",
        )
        self._current_file = open(filepath, 'w')
        self.get_logger().debug(f"Audit log rotated: {filepath}")


def main(args=None):
    """Entry point for the audit logger node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = AuditLoggerNode()
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
