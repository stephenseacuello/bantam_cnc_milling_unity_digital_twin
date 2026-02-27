"""
Heartbeat publishing mixin for MIRACLE nodes.

Provides automatic heartbeat publishing with system resource monitoring.
"""

from typing import Optional
import os
import threading

from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from miracle_core.qos_profiles import QoSProfiles


class HeartbeatMixin:
    """Mixin class that adds heartbeat publishing to a ROS2 node.

    Must be used with a class that inherits from rclpy.node.Node.
    Call _setup_heartbeat() during configuration and
    _start_heartbeat() / _stop_heartbeat() during activate/deactivate.
    """

    CRITICALITY_CRITICAL = 'CRITICAL'
    CRITICALITY_HIGH = 'HIGH'
    CRITICALITY_MEDIUM = 'MEDIUM'
    CRITICALITY_LOW = 'LOW'

    # Pre-initialize so _set_lifecycle_state() is safe before _setup_heartbeat()
    _hb_lock = None
    _hb_lifecycle_state = 'unconfigured'

    def _setup_heartbeat(
        self,
        criticality: str = 'MEDIUM',
        dependencies: Optional[list] = None,
        interval_sec: float = 0.5,
    ) -> None:
        """Set up heartbeat publishing infrastructure.

        Args:
            criticality: Node criticality level.
            dependencies: List of node names this node depends on.
            interval_sec: Heartbeat publish interval in seconds.
        """
        self._hb_criticality = criticality
        self._hb_dependencies = dependencies or []
        self._hb_interval = interval_sec
        self._hb_timer: Optional[Timer] = None
        self._hb_lifecycle_state = 'unconfigured'
        self._hb_lock = threading.Lock()

        # Lazy import to avoid circular dependency at module level
        from miracle_msgs.msg import Heartbeat
        self._Heartbeat = Heartbeat

        self._hb_callback_group = MutuallyExclusiveCallbackGroup()

        # Publisher on the global heartbeat topic
        self._hb_publisher = self.create_publisher(
            self._Heartbeat,
            '/miracle/heartbeats',
            QoSProfiles.heartbeat(),
        )

    def _start_heartbeat(self) -> None:
        """Start publishing heartbeats."""
        if self._hb_timer is not None:
            return

        self._hb_timer = self.create_timer(
            self._hb_interval,
            self._publish_heartbeat,
            callback_group=self._hb_callback_group,
        )

    def _stop_heartbeat(self) -> None:
        """Stop publishing heartbeats."""
        if self._hb_timer is not None:
            self._hb_timer.cancel()
            self._hb_timer = None

    def _set_lifecycle_state(self, state: str) -> None:
        """Update the lifecycle state reported in heartbeats."""
        if self._hb_lock is not None:
            with self._hb_lock:
                self._hb_lifecycle_state = state
        else:
            self._hb_lifecycle_state = state

    def _publish_heartbeat(self) -> None:
        """Publish a heartbeat message with current resource usage."""
        msg = self._Heartbeat()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name()
        msg.node_namespace = self.get_namespace()
        msg.criticality = self._hb_criticality

        with self._hb_lock:
            msg.lifecycle_state = self._hb_lifecycle_state

        msg.dependencies = self._hb_dependencies
        msg.cpu_usage = self._get_cpu_usage()
        msg.memory_usage = self._get_memory_usage()

        self._hb_publisher.publish(msg)

    @staticmethod
    def _get_cpu_usage() -> float:
        """Get current process CPU usage as a fraction 0-1."""
        try:
            load = os.getloadavg()[0]
            cpu_count = os.cpu_count() or 1
            return min(load / cpu_count, 1.0)
        except (OSError, AttributeError):
            return 0.0

    @staticmethod
    def _get_memory_usage() -> float:
        """Get current process memory usage as a fraction 0-1."""
        try:
            import resource
            rusage = resource.getrusage(resource.RUSAGE_SELF)
            # maxrss is in kilobytes on Linux
            max_rss_kb = rusage.ru_maxrss
            # Rough estimate against 8GB system
            total_kb = 8 * 1024 * 1024
            return min(max_rss_kb / total_kb, 1.0)
        except Exception:
            return 0.0
