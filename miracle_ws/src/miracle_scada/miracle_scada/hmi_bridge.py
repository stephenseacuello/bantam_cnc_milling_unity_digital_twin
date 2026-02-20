"""
HMI (Human-Machine Interface) Bridge Node.

Bridges ROS2 topics to WebSocket/REST for the web dashboard.
Provides real-time data streaming and command injection.
"""

from typing import Any, Dict, Optional
import json
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_core.serialization import msg_to_dict
from miracle_msgs.msg import MachineState, SystemKPIs, FleetHealth, AnomalyAlert


class HMIBridgeNode(MiracleLifecycleNode):
    """Bridges ROS2 topics to web dashboard via WebSocket.

    Parameters:
        websocket_port (int): WebSocket server port.
        update_rate_hz (float): Dashboard update rate.
        enable_commands (bool): Allow commands from dashboard.

    Subscribed Topics:
        /miracle/+/state (MachineState): Machine states.
        /miracle/system_kpis (SystemKPIs): System KPIs.
        /miracle/resiliency/fleet_health (FleetHealth): Fleet health.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'hmi_bridge',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._state_sub = None
        self._kpi_sub = None
        self._health_sub = None
        self._anomaly_sub = None
        self._update_timer = None

        self._latest_states: Dict[str, Dict] = {}
        self._latest_kpis: Optional[Dict] = None
        self._latest_health: Optional[Dict] = None
        self._recent_anomalies: list = []
        self._data_lock = threading.Lock()

        self._ws_port: int = 9090
        self._enable_commands: bool = False

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure HMI bridge."""
        params = self.declare_and_validate_parameters({
            'websocket_port': {
                'default': 9090,
                'type': int,
                'range': (1024, 65535),
            },
            'update_rate_hz': {
                'default': 10.0,
                'type': float,
                'range': (1.0, 60.0),
            },
            'enable_commands': {
                'default': False,
                'type': bool,
            },
        })

        self._ws_port = params['websocket_port']
        self._enable_commands = params['enable_commands']

        self._state_sub = self.create_subscription(
            MachineState,
            '/miracle/+/state',
            self._on_state,
            QoSProfiles.state_data(),
        )

        self._kpi_sub = self.create_subscription(
            SystemKPIs,
            '/miracle/system_kpis',
            self._on_kpis,
            QoSProfiles.state_data(),
        )

        self._health_sub = self.create_subscription(
            FleetHealth,
            '/miracle/resiliency/fleet_health',
            self._on_health,
            QoSProfiles.state_data(),
        )

        self._anomaly_sub = self.create_subscription(
            AnomalyAlert,
            '/miracle/+/anomaly',
            self._on_anomaly,
            QoSProfiles.alert(),
        )

        self.get_logger().info(f"HMI bridge configured (port={self._ws_port})")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate HMI bridge."""
        rate = self.get_parameter('update_rate_hz').value
        self._update_timer = self.create_timer(
            1.0 / rate,
            self._broadcast_update,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("HMI bridge activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate HMI bridge."""
        if self._update_timer is not None:
            self._update_timer.cancel()
            self._update_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_state(self, msg: MachineState) -> None:
        """Cache latest machine state."""
        with self._data_lock:
            self._latest_states[msg.machine_id] = msg_to_dict(msg)

    def _on_kpis(self, msg: SystemKPIs) -> None:
        """Cache latest KPIs."""
        with self._data_lock:
            self._latest_kpis = msg_to_dict(msg)

    def _on_health(self, msg: FleetHealth) -> None:
        """Cache latest fleet health."""
        with self._data_lock:
            self._latest_health = msg_to_dict(msg)

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        """Track recent anomalies."""
        with self._data_lock:
            self._recent_anomalies.append(msg_to_dict(msg))
            if len(self._recent_anomalies) > 50:
                self._recent_anomalies = self._recent_anomalies[-50:]

    def _broadcast_update(self) -> None:
        """Broadcast latest data to connected clients."""
        with self._data_lock:
            dashboard_data = {
                'machines': dict(self._latest_states),
                'kpis': self._latest_kpis,
                'fleet_health': self._latest_health,
                'recent_anomalies': list(self._recent_anomalies),
            }

        # In production, this would send via WebSocket
        # For now, just track that data is available
        self.get_logger().debug(
            f"Dashboard data ready: {len(self._latest_states)} machines",
            throttle_duration_sec=10.0,
        )

    def get_dashboard_data(self) -> Dict[str, Any]:
        """Get current dashboard data snapshot.

        Returns:
            Dictionary with all dashboard data.
        """
        with self._data_lock:
            return {
                'machines': dict(self._latest_states),
                'kpis': self._latest_kpis,
                'fleet_health': self._latest_health,
                'recent_anomalies': list(self._recent_anomalies),
            }


def main(args=None):
    """Entry point for the HMI bridge node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = HMIBridgeNode()
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
