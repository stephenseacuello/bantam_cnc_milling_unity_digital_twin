"""
Tool Wear Estimator Node.

Estimates cutting tool wear using sensor fusion data.
Predicts remaining tool life and recommends tool changes.
"""

from typing import Any, Optional
from collections import deque
import numpy as np
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import FusedSensorData, ToolWearEstimate, MachineState


class ToolWearEstimatorNode(MiracleLifecycleNode):
    """Estimates tool wear from sensor data.

    Parameters:
        machine_id (str): Machine identifier.
        tool_id (str): Current tool identifier.
        max_tool_life_minutes (float): Maximum expected tool life.
        estimation_interval_sec (float): Estimation update interval.
        wear_rate_factor (float): Wear rate scaling factor.

    Subscribed Topics:
        ~/sensor/fused (FusedSensorData): Fused sensor data.
        ~/state (MachineState): Machine state (spindle load).

    Published Topics:
        ~/tool_wear (ToolWearEstimate): Tool wear estimates.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'tool_wear_estimator',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._sensor_sub = None
        self._state_sub = None
        self._wear_pub = None
        self._estimation_timer = None

        self._machine_id: str = ''
        self._tool_id: str = ''
        self._max_life: float = 120.0
        self._wear_factor: float = 1.0

        # Wear tracking
        self._cumulative_wear: float = 0.0
        self._cutting_time_sec: float = 0.0
        self._spindle_load_history: deque = deque(maxlen=500)
        self._is_cutting: bool = False
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure tool wear estimator."""
        params = self.declare_and_validate_parameters({
            'machine_id': {'default': 'cnc1', 'type': str},
            'tool_id': {'default': 'T001', 'type': str},
            'max_tool_life_minutes': {
                'default': 120.0,
                'type': float,
                'range': (1.0, 10000.0),
            },
            'estimation_interval_sec': {
                'default': 10.0,
                'type': float,
                'range': (1.0, 300.0),
            },
            'wear_rate_factor': {
                'default': 1.0,
                'type': float,
                'range': (0.1, 10.0),
            },
        })

        self._machine_id = params['machine_id']
        self._tool_id = params['tool_id']
        self._max_life = params['max_tool_life_minutes']
        self._wear_factor = params['wear_rate_factor']

        self._state_sub = self.create_subscription(
            MachineState,
            f'/miracle/{self._machine_id}/state',
            self._on_state,
            QoSProfiles.state_data(),
        )

        self._sensor_sub = self.create_subscription(
            FusedSensorData,
            f'/miracle/{self._machine_id}/sensor/fused',
            self._on_sensor_data,
            QoSProfiles.sensor_data(),
        )

        self._wear_pub = self.create_publisher(
            ToolWearEstimate,
            f'/miracle/{self._machine_id}/tool_wear',
            QoSProfiles.state_data(),
        )

        self.get_logger().info(
            f"Tool wear estimator configured: {self._tool_id}"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate tool wear estimation."""
        interval = self.get_parameter('estimation_interval_sec').value
        self._estimation_timer = self.create_timer(
            interval,
            self._estimate_wear,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("Tool wear estimation activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate estimation."""
        if self._estimation_timer is not None:
            self._estimation_timer.cancel()
            self._estimation_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_state(self, msg: MachineState) -> None:
        """Track machine state for wear computation."""
        with self._lock:
            self._is_cutting = msg.status == 'RUNNING' and msg.spindle_speed > 0
            if self._is_cutting:
                self._spindle_load_history.append(msg.spindle_load)

    def _on_sensor_data(self, msg: FusedSensorData) -> None:
        """Update wear model with sensor data."""
        with self._lock:
            if self._is_cutting:
                # Increment cutting time (approx 20ms per sample at 50Hz)
                self._cutting_time_sec += 0.02

                # Wear rate proportional to sensor energy
                if msg.current_features:
                    mean_current = msg.current_features[0] if msg.current_features else 0.0
                    wear_increment = (
                        self._wear_factor
                        * (1.0 + mean_current / 100.0)
                        * 0.02 / 60.0  # Convert to minutes
                    )
                    self._cumulative_wear += wear_increment

    def _estimate_wear(self) -> None:
        """Compute and publish wear estimate."""
        with self._lock:
            cutting_min = self._cutting_time_sec / 60.0
            wear_pct = min(100.0, (self._cumulative_wear / self._max_life) * 100.0)
            remaining = max(0.0, self._max_life - self._cumulative_wear)

            avg_load = 0.0
            if self._spindle_load_history:
                avg_load = np.mean(list(self._spindle_load_history))

        # Confidence based on data availability
        confidence = min(0.95, cutting_min / 10.0)

        # Classify wear type
        if wear_pct > 70:
            wear_type = 'SEVERE'
        elif wear_pct > 40:
            wear_type = 'MODERATE'
        else:
            wear_type = 'NORMAL'

        msg = ToolWearEstimate()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.tool_id = self._tool_id
        msg.wear_percentage = wear_pct
        msg.remaining_life_minutes = remaining
        msg.confidence = confidence
        msg.wear_type = wear_type
        msg.flank_wear_mm = wear_pct * 0.003  # Approximate
        msg.crater_wear_mm = wear_pct * 0.001

        if remaining < 5.0:
            msg.recommended_action = 'REPLACE IMMEDIATELY'
        elif remaining < 15.0:
            msg.recommended_action = 'Schedule tool change'
        else:
            msg.recommended_action = 'Continue monitoring'

        self._wear_pub.publish(msg)


def main(args=None):
    """Entry point for the tool wear estimator node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = ToolWearEstimatorNode()
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
