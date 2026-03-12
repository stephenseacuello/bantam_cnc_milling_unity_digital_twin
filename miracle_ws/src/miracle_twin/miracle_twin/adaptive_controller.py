"""
Adaptive Feedrate Controller.

Monitors cutting forces, chatter risk, tool wear, and thermal state to
dynamically adjust feed rate and spindle speed overrides. Publishes
FeedOverride messages for closed-loop control.
"""

import math
import time as _time_mod
from typing import Any, List, Optional, Tuple
from dataclasses import dataclass, field

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import (
    AnomalyAlert,
    MachineState,
    FeedOverride,
    StabilityRecommendation,
)

try:
    from miracle_msgs.msg import InferredAction
except ImportError:
    InferredAction = None


class ControllerState:
    """State machine states for the adaptive controller."""
    NORMAL = 'NORMAL'                    # No overrides active
    FORCE_LIMITED = 'FORCE_LIMITED'      # Feed reduced due to force
    CHATTER_LIMITED = 'CHATTER_LIMITED'  # RPM adjusted for chatter
    WEAR_LIMITED = 'WEAR_LIMITED'        # Feed reduced for tool protection
    THERMAL_LIMITED = 'THERMAL_LIMITED'  # Feed reduced for thermal
    EMERGENCY = 'EMERGENCY'              # Emergency stop (0% feed)


# Hysteresis thresholds — activation and deactivation levels with deadband
# Force hysteresis: activate at 80%, deactivate at 65% (15% band)
FORCE_ACTIVATE_PCT = 0.80
FORCE_DEACTIVATE_PCT = 0.65

# Thermal hysteresis: activate at 85%, deactivate at 70%
THERMAL_ACTIVATE_PCT = 0.85
THERMAL_DEACTIVATE_PCT = 0.70

# Wear hysteresis: activate at 90%, deactivate at 75%
WEAR_ACTIVATE_PCT = 0.90
WEAR_DEACTIVATE_PCT = 0.75


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
        self._max_force_capacity: float = 1000.0  # Newtons, configurable
        self._consecutive_medium_count: int = 0
        self._medium_persistence_threshold: int = 3
        self._last_stability_recommendation: Optional['StabilityRecommendation'] = None
        self._last_inferred_actions: dict = {}  # action_type -> timestamp (seconds)

        # Hysteresis state machine
        self._controller_state: str = ControllerState.NORMAL
        self._state_entry_time: float = 0.0
        self._state_history: List[Tuple[float, str, str]] = []  # (timestamp, from, to)
        self._min_state_duration_sec: float = 5.0  # debounce
        self._override_ramp_rate: float = 0.05  # max 5% change per cycle
        self._current_feed_override: float = 100.0
        self._target_feed_override: float = 100.0
        self._current_spindle_override: float = 100.0
        self._target_spindle_override: float = 100.0

    def _do_configure(self) -> TransitionCallbackReturn:
        params = self.declare_and_validate_parameters({
            'machine_id': {'default': 'cnc1', 'type': str, 'description': 'Machine identifier'},
            'force_threshold': {'default': 0.80, 'type': float, 'range': (0.1, 1.0)},
            'wear_threshold': {'default': 0.90, 'type': float, 'range': (0.1, 1.0)},
            'thermal_threshold': {'default': 0.85, 'type': float, 'range': (0.1, 1.0)},
            'min_feed_override': {'default': 20.0, 'type': float, 'range': (5.0, 100.0)},
            'update_interval_sec': {'default': 1.0, 'type': float, 'range': (0.1, 10.0)},
            'max_force_capacity': {'default': 1000.0, 'type': float, 'range': (100.0, 50000.0)},
        })

        self._machine_id = params['machine_id']
        self._force_threshold = params['force_threshold']
        self._wear_threshold = params['wear_threshold']
        self._thermal_threshold = params['thermal_threshold']
        self._min_feed_override = params['min_feed_override']
        self._max_force_capacity = params['max_force_capacity']

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

        self.create_subscription(
            StabilityRecommendation,
            f'/miracle/{self._machine_id}/stability_recommendation',
            self._on_stability_recommendation,
            QoSProfiles.sensor_data(),
        )

        if InferredAction is not None:
            self.create_subscription(
                InferredAction,
                '/miracle/cognitive/inferred_actions',
                self._on_inferred_action,
                QoSProfiles.sensor_data(),
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
        # Compute resultant cutting force from Fx, Fy, Fz components
        fx = msg.cutting_force_x
        fy = msg.cutting_force_y
        fz = msg.cutting_force_z
        resultant_force = math.sqrt(fx * fx + fy * fy + fz * fz)

        if resultant_force > 0.0:
            # Use actual cutting forces when available
            self._state.force_ratio = resultant_force / self._max_force_capacity
        else:
            # Fallback to spindle_load as proxy when force sensors read zero
            self._state.force_ratio = msg.spindle_load / 100.0

        self._state.thermal_ratio = msg.spindle_temp / 120.0

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        atype = msg.anomaly_type.lower()
        if 'chatter' in atype or 'vibration' in atype:
            effective_severity = msg.severity

            # Boost severity if frequency-related contributing factors are present
            if hasattr(msg, 'contributing_factors') and msg.contributing_factors:
                frequency_keywords = ('frequency', 'freq', 'resonan', 'harmonic', 'fft')
                has_frequency_factor = any(
                    any(kw in factor.lower() for kw in frequency_keywords)
                    for factor in msg.contributing_factors
                )
                if has_frequency_factor:
                    # Frequency-related factors increase chatter confidence
                    effective_severity = min(1.0, effective_severity * 1.2)

            if effective_severity > 0.7:
                self._state.chatter_risk = 'HIGH'
            elif effective_severity > 0.4:
                self._state.chatter_risk = 'MEDIUM'
            else:
                self._state.chatter_risk = 'LOW'
        if 'wear' in atype:
            self._state.wear_ratio = msg.severity

    def _on_stability_recommendation(self, msg: 'StabilityRecommendation') -> None:
        """Handle stability recommendation from the digital twin's StabilityLobePredictor.

        HIGH risk: if recommended RPM differs by >5%, publish spindle override immediately.
        MEDIUM risk: only trigger override after 3 consecutive MEDIUM messages.
        LOW risk: reset consecutive counter.
        """
        self._last_stability_recommendation = msg
        risk = msg.risk_level.upper()

        if risk == 'HIGH':
            self._consecutive_medium_count = 0
            rpm_diff_pct = abs(msg.recommended_rpm - msg.current_rpm) / max(msg.current_rpm, 1.0)
            if rpm_diff_pct > 0.05:
                # Compute spindle override as percentage of current RPM
                spindle_pct = min(100.0, max(80.0, (msg.recommended_rpm / max(msg.current_rpm, 1.0)) * 100.0))
                override_msg = FeedOverride()
                override_msg.timestamp = self.get_clock().now().to_msg()
                override_msg.feed_override_pct = 80.0
                override_msg.spindle_override_pct = spindle_pct
                override_msg.reason = (
                    f"stability_recommendation=HIGH; {msg.recommendation}"
                )
                override_msg.confidence = 1.0 - msg.stability_margin
                override_msg.revert_after_sec = 15.0
                self._override_pub.publish(override_msg)
                self.get_logger().warn(
                    f"Stability HIGH: spindle override {spindle_pct:.0f}% "
                    f"(current={msg.current_rpm:.0f}, recommended={msg.recommended_rpm:.0f})"
                )
            else:
                self.get_logger().warn(
                    f"Stability HIGH but RPM diff <5% — no spindle adjustment. "
                    f"{msg.recommendation}"
                )
        elif risk == 'MEDIUM':
            self._consecutive_medium_count += 1
            self.get_logger().warning(
                f"Stability MEDIUM ({self._consecutive_medium_count}/{self._medium_persistence_threshold}): "
                f"{msg.recommendation}"
            )
            if self._consecutive_medium_count >= self._medium_persistence_threshold:
                rpm_diff_pct = abs(msg.recommended_rpm - msg.current_rpm) / max(msg.current_rpm, 1.0)
                if rpm_diff_pct > 0.05:
                    spindle_pct = min(100.0, max(80.0, (msg.recommended_rpm / max(msg.current_rpm, 1.0)) * 100.0))
                else:
                    spindle_pct = 97.0
                override_msg = FeedOverride()
                override_msg.timestamp = self.get_clock().now().to_msg()
                override_msg.feed_override_pct = 90.0
                override_msg.spindle_override_pct = spindle_pct
                override_msg.reason = (
                    f"stability_recommendation=MEDIUM (persistent {self._consecutive_medium_count}x); "
                    f"{msg.recommendation}"
                )
                override_msg.confidence = 1.0 - msg.stability_margin
                override_msg.revert_after_sec = 10.0
                self._override_pub.publish(override_msg)
                self.get_logger().warn(
                    f"Stability MEDIUM persistent: spindle override {spindle_pct:.0f}%"
                )
                self._consecutive_medium_count = 0
        else:
            # LOW risk — reset counter
            self._consecutive_medium_count = 0

    def _on_inferred_action(self, msg) -> None:
        """Handle an inferred action from the cognitive reasoning engine."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        action = msg.action_type
        confidence = msg.confidence

        # Deduplication: skip if same action_type was applied within 30 seconds
        if action in self._last_inferred_actions and (now_sec - self._last_inferred_actions[action]) < 30.0:
            self.get_logger().debug(
                f"Inferred action '{action}' deduplicated (last applied {now_sec - self._last_inferred_actions[action]:.1f}s ago)"
            )
            return

        override_msg = FeedOverride()
        override_msg.timestamp = self.get_clock().now().to_msg()
        override_msg.reason = f'inferred_action={action}; rule={msg.inference_rule}; {msg.reasoning}'
        override_msg.confidence = confidence
        override_msg.revert_after_sec = 15.0

        if action == 'REDUCE_FEED' and confidence > 0.7:
            override_msg.feed_override_pct = 70.0
            override_msg.spindle_override_pct = 100.0
            self._override_pub.publish(override_msg)
            self._last_inferred_actions[action] = now_sec
            self.get_logger().info(f"Inferred action REDUCE_FEED: feed override 70% (conf={confidence:.2f})")

        elif action == 'REDUCE_SPEED' and confidence > 0.7:
            override_msg.feed_override_pct = 100.0
            override_msg.spindle_override_pct = 85.0
            self._override_pub.publish(override_msg)
            self._last_inferred_actions[action] = now_sec
            self.get_logger().info(f"Inferred action REDUCE_SPEED: spindle override 85% (conf={confidence:.2f})")

        elif action == 'TOOL_CHANGE' and confidence > 0.8:
            override_msg.feed_override_pct = 50.0
            override_msg.spindle_override_pct = 100.0
            self._override_pub.publish(override_msg)
            self._last_inferred_actions[action] = now_sec
            self.get_logger().info(f"Inferred action TOOL_CHANGE: protective slowdown 50% (conf={confidence:.2f})")

        elif action == 'COOLANT_INCREASE':
            self.get_logger().warning(
                f"Inferred action COOLANT_INCREASE: coolant system adjustment needed "
                f"(conf={confidence:.2f}, rule={msg.inference_rule})"
            )
            self._last_inferred_actions[action] = now_sec

        elif action == 'PAUSE' and confidence > 0.9:
            override_msg.feed_override_pct = 0.0
            override_msg.spindle_override_pct = 100.0
            self._override_pub.publish(override_msg)
            self._last_inferred_actions[action] = now_sec
            self.get_logger().warning(f"Inferred action PAUSE: emergency stop 0% (conf={confidence:.2f})")

    def _determine_target_state(self) -> Tuple[str, float, float, List[str]]:
        """Determine the target state based on current sensor readings and hysteresis.

        Uses activation thresholds to enter limited states and deactivation
        thresholds (with hysteresis deadband) to leave them.  Priority ordering:
        EMERGENCY > WEAR_LIMITED > THERMAL_LIMITED > CHATTER_LIMITED > FORCE_LIMITED > NORMAL

        Returns:
            (target_state, target_feed_pct, target_spindle_pct, reasons)
        """
        feed_pct = 100.0
        spindle_pct = 100.0
        reasons: List[str] = []
        candidate_state = ControllerState.NORMAL

        # --- Force ---
        in_force_state = self._controller_state == ControllerState.FORCE_LIMITED
        force_active = (
            self._state.force_ratio > FORCE_ACTIVATE_PCT
            or (in_force_state and self._state.force_ratio > FORCE_DEACTIVATE_PCT)
        )
        if force_active:
            excess = self._state.force_ratio - FORCE_ACTIVATE_PCT
            if excess > 0:
                reduction = excess / (1.0 - FORCE_ACTIVATE_PCT)
                feed_pct = max(self._min_feed_override, 100.0 * (1.0 - reduction * 0.5))
            else:
                # In hysteresis band — hold previous target or light reduction
                feed_pct = max(self._min_feed_override, 90.0)
            reasons.append(f"force={self._state.force_ratio:.0%}")
            candidate_state = ControllerState.FORCE_LIMITED

        # --- Chatter ---
        if self._state.chatter_risk == 'HIGH':
            spindle_pct = 95.0
            feed_pct = min(feed_pct, 80.0)
            reasons.append("chatter=HIGH")
            if candidate_state == ControllerState.NORMAL or candidate_state == ControllerState.FORCE_LIMITED:
                candidate_state = ControllerState.CHATTER_LIMITED
        elif self._state.chatter_risk == 'MEDIUM':
            spindle_pct = 97.0
            reasons.append("chatter=MEDIUM")

        # --- Thermal ---
        in_thermal_state = self._controller_state == ControllerState.THERMAL_LIMITED
        thermal_active = (
            self._state.thermal_ratio > THERMAL_ACTIVATE_PCT
            or (in_thermal_state and self._state.thermal_ratio > THERMAL_DEACTIVATE_PCT)
        )
        if thermal_active:
            excess = self._state.thermal_ratio - THERMAL_ACTIVATE_PCT
            if excess > 0:
                thermal_reduction = excess / (1.0 - THERMAL_ACTIVATE_PCT)
                thermal_feed = 100.0 * (1.0 - thermal_reduction * 0.4)
            else:
                thermal_feed = 90.0
            feed_pct = min(feed_pct, max(self._min_feed_override, thermal_feed))
            reasons.append(f"thermal={self._state.thermal_ratio:.0%}")
            candidate_state = ControllerState.THERMAL_LIMITED

        # --- Wear (higher priority than thermal) ---
        in_wear_state = self._controller_state == ControllerState.WEAR_LIMITED
        wear_active = (
            self._state.wear_ratio > WEAR_ACTIVATE_PCT
            or (in_wear_state and self._state.wear_ratio > WEAR_DEACTIVATE_PCT)
        )
        if wear_active:
            feed_pct = min(feed_pct, 50.0)
            reasons.append(f"wear={self._state.wear_ratio:.0%}")
            candidate_state = ControllerState.WEAR_LIMITED

        # --- Emergency: feed at 0% if force >= 100% capacity ---
        if self._state.force_ratio >= 1.0:
            feed_pct = 0.0
            reasons.append("EMERGENCY: force at capacity")
            candidate_state = ControllerState.EMERGENCY

        feed_pct = min(100.0, max(self._min_feed_override if candidate_state != ControllerState.EMERGENCY else 0.0, feed_pct))
        spindle_pct = min(100.0, max(80.0, spindle_pct))

        return candidate_state, feed_pct, spindle_pct, reasons

    def _transition_state(self, new_state: str, reason: str) -> None:
        """Record a state transition with logging and history tracking."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        old_state = self._controller_state
        self._state_history.append((now_sec, old_state, new_state))
        self._controller_state = new_state
        self._state_entry_time = now_sec
        self.get_logger().info(
            f"State transition: {old_state} -> {new_state} ({reason})"
        )

    def _compute_and_publish(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec < self._dismissed_until:
            return

        target_state, target_feed, target_spindle, reasons = self._determine_target_state()

        # Check minimum state duration (debounce) before allowing transition
        time_in_state = now_sec - self._state_entry_time
        if target_state != self._controller_state:
            # Emergency always transitions immediately
            if target_state == ControllerState.EMERGENCY:
                self._transition_state(target_state, '; '.join(reasons))
            elif time_in_state >= self._min_state_duration_sec:
                self._transition_state(target_state, '; '.join(reasons))
            # else: stay in current state (debounce), but still use current targets

        # Set targets
        self._target_feed_override = target_feed
        self._target_spindle_override = target_spindle

        # Ramp current overrides toward targets smoothly
        ramp_amount = self._override_ramp_rate * 100.0  # convert fraction to percentage points
        feed_delta = self._target_feed_override - self._current_feed_override
        self._current_feed_override += max(-ramp_amount, min(ramp_amount, feed_delta))

        spindle_delta = self._target_spindle_override - self._current_spindle_override
        self._current_spindle_override += max(-ramp_amount, min(ramp_amount, spindle_delta))

        # Clamp
        self._current_feed_override = min(100.0, max(0.0, self._current_feed_override))
        self._current_spindle_override = min(100.0, max(80.0, self._current_spindle_override))

        if self._current_feed_override < 99.9 or self._current_spindle_override < 99.9:
            msg = FeedOverride()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.feed_override_pct = self._current_feed_override
            msg.spindle_override_pct = self._current_spindle_override
            msg.reason = '; '.join(reasons) if reasons else 'nominal'
            msg.confidence = min(1.0, max(
                self._state.force_ratio, self._state.wear_ratio, self._state.thermal_ratio,
            ))
            msg.revert_after_sec = 10.0
            self._override_pub.publish(msg)
            self.get_logger().info(
                f"Feed override: {self._current_feed_override:.1f}% / "
                f"spindle: {self._current_spindle_override:.1f}% "
                f"[state={self._controller_state}] ({', '.join(reasons)})"
            )

    def dismiss_override(self, duration_sec: float = 60.0) -> None:
        self._dismissed_until = self.get_clock().now().nanoseconds / 1e9 + duration_sec

    @property
    def current_state(self) -> AdaptiveState:
        return self._state

    @property
    def state(self) -> str:
        """Current controller state machine state."""
        return self._controller_state

    @property
    def state_history(self) -> List[Tuple[float, str, str]]:
        """History of state transitions: list of (timestamp, from_state, to_state)."""
        return list(self._state_history)


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
