"""
Network Traffic Manager Node.

Monitors and manages DDS/ROS2 network traffic. Implements QoS-aware
topic routing, bandwidth management, and network health monitoring.
"""

from typing import Any, Dict, List
from collections import defaultdict
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import Heartbeat


class TrafficManagerNode(MiracleLifecycleNode):
    """Monitors and manages ROS2 network traffic.

    Parameters:
        bandwidth_limit_mbps (float): Maximum bandwidth in Mbps.
        monitor_interval_sec (float): Traffic check interval.
        alert_threshold (float): Utilization threshold for alerts.

    Subscribed Topics:
        /miracle/heartbeats (Heartbeat): Monitor node presence.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'traffic_manager',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._heartbeat_sub = None
        self._monitor_timer = None
        self._topic_stats: Dict[str, Dict[str, Any]] = defaultdict(
            lambda: {'msg_count': 0, 'bytes': 0}
        )
        self._stats_lock = threading.Lock()
        self._bandwidth_limit: float = 100.0
        self._alert_threshold: float = 0.8

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure traffic manager."""
        params = self.declare_and_validate_parameters({
            'bandwidth_limit_mbps': {
                'default': 100.0,
                'type': float,
                'range': (1.0, 10000.0),
            },
            'monitor_interval_sec': {
                'default': 5.0,
                'type': float,
                'range': (1.0, 60.0),
            },
            'alert_threshold': {
                'default': 0.8,
                'type': float,
                'range': (0.1, 1.0),
            },
        })

        self._bandwidth_limit = params['bandwidth_limit_mbps']
        self._alert_threshold = params['alert_threshold']

        self._heartbeat_sub = self.create_subscription(
            Heartbeat,
            '/miracle/heartbeats',
            self._on_heartbeat,
            QoSProfiles.heartbeat(),
        )

        self.get_logger().info("Traffic manager configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate traffic monitoring."""
        interval = self.get_parameter('monitor_interval_sec').value
        self._monitor_timer = self.create_timer(
            interval,
            self._monitor_traffic,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("Traffic monitoring activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate traffic monitoring."""
        if self._monitor_timer is not None:
            self._monitor_timer.cancel()
            self._monitor_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_heartbeat(self, msg: Heartbeat) -> None:
        """Track heartbeat messages for node monitoring."""
        with self._stats_lock:
            key = f"{msg.node_namespace}/{msg.node_name}"
            self._topic_stats[key]['msg_count'] += 1

    def _monitor_traffic(self) -> None:
        """Periodic traffic monitoring."""
        with self._stats_lock:
            total_msgs = sum(
                s['msg_count'] for s in self._topic_stats.values()
            )
            active_nodes = len(self._topic_stats)

            if total_msgs > 0:
                self.get_logger().debug(
                    f"Traffic: {active_nodes} active nodes, "
                    f"{total_msgs} total messages"
                )

            # Reset counters
            for stats in self._topic_stats.values():
                stats['msg_count'] = 0


def main(args=None):
    """Entry point for the traffic manager node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = TrafficManagerNode()
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
