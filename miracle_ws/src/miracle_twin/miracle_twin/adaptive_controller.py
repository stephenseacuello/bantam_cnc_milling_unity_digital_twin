"""
Adaptive Feedrate Controller.

Monitors cutting forces, chatter risk, tool wear, and thermal state to
dynamically adjust feed rate and spindle speed overrides. Publishes
FeedOverride messages for closed-loop control.
"""

from typing import Any, Optional
from dataclasses import dataclass

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import (
    AnomalyAlert,
    MachineState,
    FeedOverride,
)


@dataclass
class AdaptiveState:
    """Current state used for adaptive decisions."""
    force_ratio: float = 0.0
    chatter_risk: str = 'LOW'
    wear_ratio: float = 0.0
    thermal_ratio: float = 0.0
    current_feed_override: float = 100.0
    current_spindle_override: float = 100.0


class AdaptiveControllerNode(MiracleLifecycleNode):
    """Dynamically adjusts feed rate and spindle speed based on process state.

    Decision logic:
    - Force > 80% of limit: reduce feed proportionally
    - Chatter risk HIGH: shift spindle RPM -5%
    - Wear > 90%: reduce feed by 50%
    - Thermal > 85%: reduce feed proportionally
    Safety: never increases beyond 100% of programmed values.

    Parameters:
        machine_id (str): Machine identifier.
        force_threshold (float): Force ratio threshold (0-1).
        wear_threshold (float): Wear ratio threshold (0-1).
        thermal_threshold (float): Thermal ratio threshold (0-1).
        min_feed_override (float): Minimum allowed feed override %.
        update_interval_sec (float): How often to publish overrides.

    Published Topics:
        /miracle/{machine_id}/feed_override (FeedOverride)

    Subscribed Topics:
        /miracle/{machine_id}/state (MachineState)
        /miracle/{machine_id}/anomaly (AnomalyAlert)
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'adaptive_controller',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._machine_id: str = ''
        self._state = AdaptiveState()
        self._override_pub = None
        self._update_timer = None
        self._force_threshold: float = 0.80
        self._wear_threshold: float = 0.90
        self._thermal_threshold: float = 0.85
        self._min_feed_override: float = 20.0
        self._dismissed_until: float = 0.0

    def _do_configure(self) -> TransitionCallbackReturn:
        params = self.declare_and_validate_parameters({
            'machine_id': {'default': 'cnc1', 'type': str, 'description': 'Machine identifier'},
            'force_threshold': {'default': 0.80, 'type': float, 'range': (0.1, 1.0)},
            'wear_threshold': {'default': 0.90, 'type': float, 'range': (0.1, 1.0)},
            'thermal_threshold': {'default': 0.85, 'type': float, 'range': (0.1, 1.0)},
            'min_feed_override': {'default': 20.0, 'type': float, 'range': (5.0, 100.0)},
            'update_interval_sec': {'default': 1.0, 'type': float, 'range': (0.1, 10.0)},
        })

        self._machine_id = params['machine_id']
        self._force_threshold = params['force_threshold']
        self._wear_threshold = params['wear_threshold']
        self._thermal_threshold = params['thermal_threshold']
        self._min_feed_override = params['min_feed_override']

        self._override_pub = self.create_publisher(
            FeedOverride,
            f'/miracle/{self._machine_id}/feed_override',
            QoSProfiles.command(),
        )

        self.create_subscription(
            MachineState,
            f'/miracle/{self._machine_id}/state',
            self._on_machine_state,
            QoSProfiles.sensor_data(),
        )

        self.create_multi_machine_subscriptions(
            AnomalyAlert, 'anomaly', self._on_anomaly,
            QoSProfiles.alert(), [self._machine_id],
        )

        self.get_logger().info(f"Adaptive controller configured for '{self._machine_id}'")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        interval = self.get_parameter('update_interval_sec').value
        self._update_timer = self.create_timer(
            interval, self._compute_and_publish,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("Adaptive controller activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        if self._update_timer is not None:
            self._update_timer.cancel()
            self._update_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_machine_state(self, msg: MachineState) -> None:
        self._state.force_ratio = msg.spindle_load / 100.0
        if hasattr(msg, 'spindle_temp'):
            self._state.thermal_ratio = msg.spindle_temp / 120.0

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        atype = msg.anomaly_type.lower()
        if 'chatter' in atype or 'vibration' in atype:
            if msg.severity > 0.7:
                self._state.chatter_risk = 'HIGH'
            elif msg.severity > 0.4:
                self._state.chatter_risk = 'MEDIUM'
            else:
                self._state.chatter_risk = 'LOW'
        if 'wear' in atype:
            self._state.wear_ratio = msg.severity

    def _compute_and_publish(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec < self._dismissed_until:
            return

        feed_pct = 100.0
        spindle_pct = 100.0
        reasons = []

        if self._state.force_ratio > self._force_threshold:
            excess = self._state.force_ratio - self._force_threshold
            reduction = excess / (1.0 - self._force_threshold)
            feed_pct = max(self._min_feed_override, 100.0 * (1.0 - reduction * 0.5))
            reasons.append(f"force={self._state.force_ratio:.0%}")

        if self._state.chatter_risk == 'HIGH':
            spindle_pct = 95.0
            feed_pct = min(feed_pct, 80.0)
            reasons.append("chatter=HIGH")
        elif self._state.chatter_risk == 'MEDIUM':
            spindle_pct = 97.0
            reasons.append("chatter=MEDIUM")

        if self._state.wear_ratio > self._wear_threshold:
            feed_pct = min(feed_pct, 50.0)
            reasons.append(f"wear={self._state.wear_ratio:.0%}")

        if self._state.thermal_ratio > self._thermal_threshold:
            excess = self._state.thermal_ratio - self._thermal_threshold
            thermal_reduction = excess / (1.0 - self._thermal_threshold)
            thermal_feed = 100.0 * (1.0 - thermal_reduction * 0.4)
            feed_pct = min(feed_pct, max(self._min_feed_override, thermal_feed))
            reasons.append(f"thermal={self._state.thermal_ratio:.0%}")

        feed_pct = min(100.0, max(self._min_feed_override, feed_pct))
        spindle_pct = min(100.0, max(80.0, spindle_pct))

        if feed_pct < 99.9 or spindle_pct < 99.9:
            msg = FeedOverride()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.feed_override_pct = feed_pct
            msg.spindle_override_pct = spindle_pct
            msg.reason = '; '.join(reasons) if reasons else 'nominal'
            msg.confidence = min(1.0, max(
                self._state.force_ratio, self._state.wear_ratio, self._state.thermal_ratio,
            ))
            msg.revert_after_sec = 10.0
            self._override_pub.publish(msg)
            self.get_logger().info(
                f"Feed override: {feed_pct:.0f}% / spindle: {spindle_pct:.0f}% "
                f"({', '.join(reasons)})"
            )

    def dismiss_override(self, duration_sec: float = 60.0) -> None:
        self._dismissed_until = self.get_clock().now().nanoseconds / 1e9 + duration_sec

    @property
    def current_state(self) -> AdaptiveState:
        return self._state


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = AdaptiveControllerNode()
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
