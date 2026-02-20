"""
Local Machine Watchdog Node.

Monitors a single CNC machine for safety violations,
communication timeouts, and abnormal behavior.
Triggers emergency stops when safety thresholds are exceeded.
"""

from typing import Any, Optional

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, AnomalyAlert
from miracle_msgs.srv import TriggerEStop


class LocalWatchdogNode(MiracleLifecycleNode):
    """Monitors machine safety with configurable thresholds.

    Parameters:
        machine_id (str): Machine identifier.
        max_spindle_load (float): Max spindle load percentage before alert.
        max_vibration_g (float): Max vibration in g before alert.
        min_coolant_level (float): Minimum coolant level percentage.
        state_timeout_sec (float): Timeout for state messages.
        check_rate_hz (float): Watchdog check frequency.

    Subscribed Topics:
        ~/state (MachineState): Machine state to monitor.

    Published Topics:
        ~/anomaly (AnomalyAlert): Safety violation alerts.

    Service Clients:
        ~/trigger_estop (TriggerEStop): Emergency stop service.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'local_watchdog',
            criticality=self.CRITICALITY_CRITICAL,
            **kwargs,
        )
        self._state_sub = None
        self._anomaly_pub = None
        self._estop_client = None
        self._check_timer = None
        self._last_state: Optional[MachineState] = None
        self._last_state_time = None
        self._machine_id: str = ''
        self._max_spindle_load: float = 95.0
        self._min_coolant: float = 10.0
        self._state_timeout: float = 2.0

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure watchdog."""
        params = self.declare_and_validate_parameters({
            'machine_id': {'default': 'cnc1', 'type': str},
            'max_spindle_load': {
                'default': 95.0,
                'type': float,
                'range': (50.0, 100.0),
            },
            'min_coolant_level': {
                'default': 10.0,
                'type': float,
                'range': (0.0, 50.0),
            },
            'state_timeout_sec': {
                'default': 2.0,
                'type': float,
                'range': (0.1, 30.0),
            },
            'check_rate_hz': {
                'default': 10.0,
                'type': float,
                'range': (1.0, 100.0),
            },
        })

        self._machine_id = params['machine_id']
        self._max_spindle_load = params['max_spindle_load']
        self._min_coolant = params['min_coolant_level']
        self._state_timeout = params['state_timeout_sec']

        self._anomaly_pub = self.create_publisher(
            AnomalyAlert,
            'anomaly',
            QoSProfiles.alert(),
        )

        self._state_sub = self.create_subscription(
            MachineState,
            'state',
            self._on_state,
            QoSProfiles.state_data(),
        )

        self._estop_client = self.create_client(
            TriggerEStop,
            'trigger_estop',
            callback_group=self.service_callback_group,
        )

        self.get_logger().info(f"Watchdog configured for {self._machine_id}")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate watchdog checks."""
        rate = self.get_parameter('check_rate_hz').value
        self._check_timer = self.create_timer(
            1.0 / rate,
            self._check_safety,
            callback_group=self.realtime_callback_group,
        )
        self.get_logger().info("Watchdog activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate watchdog."""
        if self._check_timer is not None:
            self._check_timer.cancel()
            self._check_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_state(self, msg: MachineState) -> None:
        """Receive machine state updates."""
        self._last_state = msg
        self._last_state_time = self.get_clock().now()

    def _check_safety(self) -> None:
        """Periodic safety check."""
        now = self.get_clock().now()

        # Check for state timeout
        if self._last_state_time is not None:
            elapsed = (now - self._last_state_time).nanoseconds / 1e9
            if elapsed > self._state_timeout:
                self._publish_alert(
                    'COMMUNICATION',
                    confidence=1.0,
                    severity=0.8,
                    factors=[f'No state update for {elapsed:.1f}s'],
                    action='Check machine communication',
                    immediate_stop=elapsed > self._state_timeout * 3,
                )
                return

        if self._last_state is None:
            return

        state = self._last_state

        # Check spindle load
        if state.spindle_load > self._max_spindle_load:
            self._publish_alert(
                'OVERLOAD',
                confidence=0.95,
                severity=state.spindle_load / 100.0,
                factors=[f'Spindle load {state.spindle_load:.1f}%'],
                action='Reduce feed rate or check tooling',
                immediate_stop=state.spindle_load > 98.0,
            )

        # Check coolant level
        if state.coolant_level < self._min_coolant:
            self._publish_alert(
                'COOLANT',
                confidence=1.0,
                severity=1.0 - state.coolant_level / 100.0,
                factors=[f'Coolant at {state.coolant_level:.1f}%'],
                action='Refill coolant',
                immediate_stop=state.coolant_level < 2.0,
            )

        # Check for error state
        if state.status == 'ERROR':
            self._publish_alert(
                'UNKNOWN',
                confidence=1.0,
                severity=0.9,
                factors=['Machine reports error state'],
                action='Investigate machine error',
                immediate_stop=True,
            )

    def _publish_alert(
        self,
        anomaly_type: str,
        confidence: float,
        severity: float,
        factors: list,
        action: str,
        immediate_stop: bool,
    ) -> None:
        """Publish an anomaly alert."""
        msg = AnomalyAlert()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.anomaly_type = anomaly_type
        msg.confidence = confidence
        msg.severity = severity
        msg.contributing_factors = factors
        msg.feature_contributions = []
        msg.recommended_action = action
        msg.requires_immediate_stop = immediate_stop

        self._anomaly_pub.publish(msg)

        if immediate_stop:
            self.get_logger().error(
                f"EMERGENCY: {anomaly_type} - triggering E-stop"
            )
            self._trigger_estop(action)

    def _trigger_estop(self, reason: str) -> None:
        """Trigger emergency stop via service call."""
        if not self._estop_client.service_is_ready():
            self.get_logger().error("E-stop service not available!")
            return

        request = TriggerEStop.Request()
        request.machine_id = self._machine_id
        request.reason = reason
        request.requesting_node = self.get_fully_qualified_name()

        future = self._estop_client.call_async(request)
        future.add_done_callback(self._estop_callback)

    def _estop_callback(self, future) -> None:
        """Handle E-stop service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("E-stop triggered successfully")
            else:
                self.get_logger().error(f"E-stop failed: {response.message}")
        except Exception as exc:
            self.get_logger().error(f"E-stop call error: {exc}")


def main(args=None):
    """Entry point for the local watchdog node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = LocalWatchdogNode()
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
