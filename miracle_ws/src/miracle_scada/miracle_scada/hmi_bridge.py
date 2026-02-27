"""
HMI (Human-Machine Interface) Bridge Node.

Bridges ROS2 topics to WebSocket/REST for the web dashboard.
Provides real-time data streaming and command injection.
"""

from typing import Any, Dict, Optional, Set
import asyncio
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
        machine_ids (str): Comma-separated machine IDs (e.g. 'cnc1,cnc2,cnc3').

    Subscribed Topics:
        /miracle/{machine_id}/state (MachineState): Per-machine states.
        /miracle/{machine_id}/anomaly (AnomalyAlert): Per-machine anomalies.
        /miracle/system_kpis (SystemKPIs): System KPIs.
        /miracle/resiliency/fleet_health (FleetHealth): Fleet health.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'hmi_bridge',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._state_subs = []
        self._kpi_sub = None
        self._health_sub = None
        self._anomaly_subs = []
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
            'machine_ids': {
                'default': 'cnc1,cnc2,cnc3',
                'type': str,
            },
        })

        self._ws_port = params['websocket_port']
        self._enable_commands = params['enable_commands']

        # Subscribe to each machine's state and anomaly topics individually
        # (ROS2 DDS does not support MQTT-style '+' wildcards)
        machine_ids = [
            m.strip() for m in params['machine_ids'].split(',') if m.strip()
        ]
        self._state_subs = []
        self._anomaly_subs = []
        for machine_id in machine_ids:
            self._state_subs.append(self.create_subscription(
                MachineState,
                f'/miracle/{machine_id}/state',
                self._on_state,
                QoSProfiles.state_data(),
            ))
            self._anomaly_subs.append(self.create_subscription(
                AnomalyAlert,
                f'/miracle/{machine_id}/anomaly',
                self._on_anomaly,
                QoSProfiles.alert(),
            ))

        self.get_logger().info(
            f"Subscribed to {len(machine_ids)} machines: {machine_ids}"
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

        self.get_logger().info(f"HMI bridge configured (port={self._ws_port})")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate HMI bridge — start WebSocket server and update timer."""
        self._connected_clients: Set = set()
        self._ws_loop = asyncio.new_event_loop()
        self._ws_thread = threading.Thread(
            target=self._run_ws_server, daemon=True
        )
        self._ws_thread.start()

        rate = self.get_parameter('update_rate_hz').value
        self._update_timer = self.create_timer(
            1.0 / rate,
            self._broadcast_update,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info(
            f"HMI bridge activated (WebSocket on port {self._ws_port})"
        )
        return TransitionCallbackReturn.SUCCESS

    def _run_ws_server(self) -> None:
        """Run the async WebSocket server in a dedicated thread."""
        try:
            import websockets
        except ImportError:
            self.get_logger().error(
                "websockets package not installed — "
                "run: pip install websockets"
            )
            return

        asyncio.set_event_loop(self._ws_loop)

        async def start_server():
            async with websockets.serve(
                self._ws_handler, '0.0.0.0', self._ws_port
            ):
                await asyncio.Future()  # Run forever

        try:
            self._ws_loop.run_until_complete(start_server())
        except Exception as exc:
            self.get_logger().error(f"WebSocket server error: {exc}")

    async def _ws_handler(self, websocket) -> None:
        """Handle a single WebSocket client connection."""
        self._connected_clients.add(websocket)
        client_addr = websocket.remote_address
        self.get_logger().info(
            f"WebSocket client connected: {client_addr} "
            f"(total: {len(self._connected_clients)})"
        )
        try:
            async for message in websocket:
                if self._enable_commands:
                    await self._handle_ws_command(message, websocket)
        except Exception:
            pass
        finally:
            self._connected_clients.discard(websocket)
            self.get_logger().info(
                f"WebSocket client disconnected: {client_addr} "
                f"(total: {len(self._connected_clients)})"
            )

    async def _handle_ws_command(self, message: str, websocket) -> None:
        """Handle incoming command from a WebSocket client."""
        try:
            cmd = json.loads(message)
            self.get_logger().info(f"Dashboard command: {cmd.get('action')}")
            await websocket.send(json.dumps({
                'type': 'ack', 'action': cmd.get('action')
            }))
        except json.JSONDecodeError:
            await websocket.send(json.dumps({
                'type': 'error', 'message': 'Invalid JSON'
            }))

    async def _broadcast_to_clients(self, payload: str) -> None:
        """Send payload to all connected WebSocket clients."""
        if not self._connected_clients:
            return
        dead = set()
        for ws in self._connected_clients.copy():
            try:
                await ws.send(payload)
            except Exception:
                dead.add(ws)
        self._connected_clients -= dead

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate HMI bridge — stop WebSocket server and timer."""
        if self._update_timer is not None:
            self._update_timer.cancel()
            self._update_timer = None

        # Shut down the WebSocket event loop
        if hasattr(self, '_ws_loop') and self._ws_loop.is_running():
            self._ws_loop.call_soon_threadsafe(self._ws_loop.stop)
        if hasattr(self, '_ws_thread') and self._ws_thread.is_alive():
            self._ws_thread.join(timeout=2.0)

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
        """Broadcast latest data to all connected WebSocket clients."""
        with self._data_lock:
            dashboard_data = {
                'type': 'update',
                'machines': dict(self._latest_states),
                'kpis': self._latest_kpis,
                'fleet_health': self._latest_health,
                'recent_anomalies': list(self._recent_anomalies),
            }

        if not hasattr(self, '_connected_clients') or not self._connected_clients:
            return

        try:
            payload = json.dumps(dashboard_data, default=str)
            asyncio.run_coroutine_threadsafe(
                self._broadcast_to_clients(payload),
                self._ws_loop,
            )
        except RuntimeError as exc:
            # Event loop may be shutting down during node deactivation
            self.get_logger().debug(
                f"Broadcast skipped (event loop shutdown): {exc}",
                throttle_duration_sec=30.0,
            )
        except Exception as exc:
            self.get_logger().debug(
                f"Broadcast error: {exc}", throttle_duration_sec=10.0
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
