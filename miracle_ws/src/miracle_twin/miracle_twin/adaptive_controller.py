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


@dataclass
class PredictiveOverride:
    """A feed/speed override based on predicted anomaly markers."""
    trigger_block_index: int
    marker_type: str  # "FORCE_CRITICAL", "CHATTER_RISK", etc.
    predicted_severity: float
    feed_override_pct: float  # what feed % to apply
    speed_override_pct: float  # what speed % to apply (100 = no change)
    reason: str
    expires_at_block: int  # override active until this block
    confidence: float


@dataclass
class PreemptiveAction:
    """A preemptive override computed from a predicted anomaly marker."""
    marker_type: str         # From AnomalyMarker
    blocks_ahead: int        # How many blocks until the predicted anomaly
    recommended_feed_pct: float   # Suggested feed override percentage
    recommended_speed_pct: float  # Suggested spindle override percentage
    reason: str
    confidence: float
    applied: bool = False


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

        # Preemptive control state
        self._preemptive_enabled: bool = True
        self._preemptive_horizon: int = 10  # React to anomalies predicted within next N blocks
        self._preemptive_actions: List['PreemptiveAction'] = []  # History of PreemptiveAction
        self._current_preemptive: Optional['PreemptiveAction'] = None  # Active preemptive override
        self._preemptive_min_confidence: float = 0.5
        self._predictive_overrides: List['PredictiveOverride'] = []

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

    # ------------------------------------------------------------------
    # Preemptive / predictive control
    # ------------------------------------------------------------------

    def handle_anomaly_markers(self, markers: list, current_block: int) -> None:
        """Process prediction-based anomaly markers for proactive adjustment.

        Called when new lookahead/prediction results are available.
        Only acts on markers within the preemptive_horizon.
        """
        if not self._preemptive_enabled:
            return

        best_action: Optional[PreemptiveAction] = None

        for marker in markers:
            blocks_ahead = marker.block_index - current_block
            if blocks_ahead < 0 or blocks_ahead > self._preemptive_horizon:
                continue

            confidence = getattr(marker, 'severity', 0.0)
            if confidence < self._preemptive_min_confidence:
                # Still compute, but scale down
                pass

            action = self._compute_preemptive_override(marker, blocks_ahead)
            if action is None:
                continue

            # Pick the most severe (lowest feed or speed)
            if best_action is None:
                best_action = action
            else:
                if action.recommended_feed_pct < best_action.recommended_feed_pct:
                    best_action = action
                elif (action.recommended_feed_pct == best_action.recommended_feed_pct
                      and action.recommended_speed_pct < best_action.recommended_speed_pct):
                    best_action = action

        if best_action is not None:
            best_action.applied = True
            self._current_preemptive = best_action
            self._preemptive_actions.append(best_action)
            self._merge_preemptive_with_reactive()
        # If no actionable markers, leave current preemptive as-is (it auto-clears)

    def _compute_preemptive_override(self, marker, blocks_ahead: int) -> Optional[PreemptiveAction]:
        """Determine feed/speed override based on predicted anomaly type and severity.

        Rules:
        - FORCE_WARNING: reduce feed to 85%
        - FORCE_CRITICAL: reduce feed to 70%
        - THERMAL_WARNING: reduce speed to 90%
        - THERMAL_CRITICAL: reduce speed to 75%, feed to 85%
        - CHATTER_RISK: compute stable RPM (if stability data available) else reduce 10%
        - WEAR_ACCELERATED: reduce feed to 80%
        - TOOL_END_OF_LIFE: reduce feed to 60%, flag for tool change
        - SURFACE_QUALITY_WARNING: reduce feed to 75%

        Confidence scales the override (high confidence = full override, low = softer).
        """
        mtype = marker.marker_type
        confidence = getattr(marker, 'severity', 0.0)

        # Base overrides (before confidence scaling)
        base_feed = 100.0
        base_speed = 100.0
        reason = ''

        if mtype == 'FORCE_WARNING':
            base_feed = 85.0
            reason = 'Predicted force warning ahead'
        elif mtype == 'FORCE_CRITICAL':
            base_feed = 70.0
            reason = 'Predicted force critical ahead'
        elif mtype == 'THERMAL_WARNING':
            base_speed = 90.0
            reason = 'Predicted thermal warning ahead'
        elif mtype == 'THERMAL_CRITICAL':
            base_speed = 75.0
            base_feed = 85.0
            reason = 'Predicted thermal critical ahead'
        elif mtype == 'CHATTER_RISK':
            # Use stability recommendation if available
            if (self._last_stability_recommendation is not None
                    and hasattr(self._last_stability_recommendation, 'recommended_rpm')
                    and self._last_stability_recommendation.recommended_rpm > 0):
                rec = self._last_stability_recommendation
                ratio = rec.recommended_rpm / max(rec.current_rpm, 1.0)
                base_speed = min(100.0, max(80.0, ratio * 100.0))
            else:
                base_speed = 90.0
            reason = 'Predicted chatter risk ahead'
        elif mtype == 'WEAR_ACCELERATED':
            base_feed = 80.0
            reason = 'Predicted accelerated wear ahead'
        elif mtype == 'TOOL_END_OF_LIFE':
            base_feed = 60.0
            reason = 'Predicted tool end-of-life ahead — schedule tool change'
        elif mtype == 'SURFACE_QUALITY_WARNING':
            base_feed = 75.0
            reason = 'Predicted surface quality issue ahead'
        else:
            return None

        # Apply confidence scaling: blend between 100% (no change) and base value
        # At confidence=1.0 use full override; at confidence=0.0 use no override
        scaled_feed = 100.0 - (100.0 - base_feed) * confidence
        scaled_speed = 100.0 - (100.0 - base_speed) * confidence

        return PreemptiveAction(
            marker_type=mtype,
            blocks_ahead=blocks_ahead,
            recommended_feed_pct=scaled_feed,
            recommended_speed_pct=scaled_speed,
            reason=reason,
            confidence=confidence,
        )

    def _merge_preemptive_with_reactive(self) -> None:
        """When both preemptive and reactive overrides exist, take the more conservative one."""
        if self._current_preemptive is None:
            return

        # Use the lower (more conservative) of preemptive and current targets
        self._target_feed_override = min(
            self._target_feed_override,
            self._current_preemptive.recommended_feed_pct,
        )
        self._target_spindle_override = min(
            self._target_spindle_override,
            self._current_preemptive.recommended_speed_pct,
        )

    def clear_preemptive(self) -> None:
        """Clear preemptive override when the predicted anomaly block has passed."""
        self._current_preemptive = None

    # ------------------------------------------------------------------
    # Prediction-based anomaly marker scheduling
    # ------------------------------------------------------------------

    # Marker-type rules: (feed_override_pct, speed_override_pct, pre_blocks, post_blocks)
    _MARKER_RULES: dict = {
        'FORCE_CRITICAL':         (70.0,  100.0, 3, 0),
        'FORCE_WARNING':          (85.0,  100.0, 2, 0),
        'CHATTER_RISK':           (100.0,  90.0, 2, 3),
        'THERMAL_CRITICAL':       (75.0,   90.0, 2, 1),
        'THERMAL_WARNING':        (90.0,  100.0, 1, 0),
        'WEAR_ACCELERATED':       (80.0,  100.0, 1, 1),
        'TOOL_END_OF_LIFE':       (60.0,  100.0, 3, 1),
        'SURFACE_QUALITY_WARNING': (85.0, 100.0, 1, 0),
    }

    def _process_anomaly_markers(self, markers: list) -> list:
        """Convert anomaly markers into predictive overrides.

        Rules:
        - FORCE_CRITICAL: reduce feed to 70%, for 3 blocks before and during
        - FORCE_WARNING: reduce feed to 85%, for 2 blocks before and during
        - CHATTER_RISK: shift RPM to stable pocket (via speed_override), 5 blocks range
        - THERMAL_CRITICAL: reduce feed to 75%, also reduce speed to 90%
        - THERMAL_WARNING: reduce feed to 90%
        - WEAR_ACCELERATED: reduce feed to 80%
        - TOOL_END_OF_LIFE: reduce feed to 60%, flag for tool change
        - SURFACE_QUALITY_WARNING: reduce feed to 85%
        """
        overrides: list = []
        for marker in markers:
            mtype = getattr(marker, 'marker_type', None)
            if mtype is None or mtype not in self._MARKER_RULES:
                continue

            block_idx = getattr(marker, 'block_index', 0)
            severity = getattr(marker, 'severity', 0.5)
            feed_pct, speed_pct, pre_blocks, post_blocks = self._MARKER_RULES[mtype]

            # High-severity markers (>0.8) get 1 extra preemptive block
            extra = 1 if severity > 0.8 else 0
            start_block = block_idx - (pre_blocks + extra)
            end_block = block_idx + post_blocks

            reason = f'Predicted {mtype} at block {block_idx}'
            if mtype == 'TOOL_END_OF_LIFE':
                reason += ' — schedule tool change'

            override = PredictiveOverride(
                trigger_block_index=block_idx,
                marker_type=mtype,
                predicted_severity=severity,
                feed_override_pct=feed_pct,
                speed_override_pct=speed_pct,
                reason=reason,
                expires_at_block=end_block,
                confidence=severity,
            )
            # Attach computed start for merging (not in dataclass, use attribute)
            override._start_block = start_block  # type: ignore[attr-defined]
            overrides.append(override)

        return overrides

    def _merge_overlapping_overrides(self, overrides: list) -> list:
        """When multiple overrides cover the same block range, take the most conservative.

        For each block in the union of all ranges, the effective feed% is the minimum
        of all overrides active at that block, and likewise for speed%.
        We merge contiguous blocks with the same effective values into single overrides.
        """
        if not overrides:
            return []

        # Collect all (start, end, feed, speed, override) tuples
        intervals = []
        for ov in overrides:
            start = getattr(ov, '_start_block', ov.trigger_block_index)
            end = ov.expires_at_block
            intervals.append((start, end, ov))

        # Find the global block range
        all_starts = [s for s, e, o in intervals]
        all_ends = [e for s, e, o in intervals]
        global_start = min(all_starts)
        global_end = max(all_ends)

        merged: list = []
        current_start = None
        current_feed = None
        current_speed = None
        current_severity = None
        current_reasons = []
        current_marker_type = None

        for blk in range(global_start, global_end + 1):
            # Find all overrides active at this block
            active_feed = 100.0
            active_speed = 100.0
            active_severity = 0.0
            active_reasons = []
            active_type = None
            for start, end, ov in intervals:
                if start <= blk <= end:
                    active_feed = min(active_feed, ov.feed_override_pct)
                    active_speed = min(active_speed, ov.speed_override_pct)
                    active_severity = max(active_severity, ov.predicted_severity)
                    if ov.reason not in active_reasons:
                        active_reasons.append(ov.reason)
                    active_type = ov.marker_type

            if active_feed >= 100.0 and active_speed >= 100.0:
                # No override at this block — flush current run
                if current_start is not None:
                    merged.append(PredictiveOverride(
                        trigger_block_index=current_start,
                        marker_type=current_marker_type or 'MERGED',
                        predicted_severity=current_severity or 0.0,
                        feed_override_pct=current_feed or 100.0,
                        speed_override_pct=current_speed or 100.0,
                        reason='; '.join(current_reasons),
                        expires_at_block=blk - 1,
                        confidence=current_severity or 0.0,
                    ))
                    current_start = None
                continue

            if (current_start is not None
                    and active_feed == current_feed
                    and active_speed == current_speed):
                # Extend current run
                current_severity = max(current_severity or 0.0, active_severity)
                for r in active_reasons:
                    if r not in current_reasons:
                        current_reasons.append(r)
                continue

            # Different override values — flush old run, start new
            if current_start is not None:
                merged.append(PredictiveOverride(
                    trigger_block_index=current_start,
                    marker_type=current_marker_type or 'MERGED',
                    predicted_severity=current_severity or 0.0,
                    feed_override_pct=current_feed or 100.0,
                    speed_override_pct=current_speed or 100.0,
                    reason='; '.join(current_reasons),
                    expires_at_block=blk - 1,
                    confidence=current_severity or 0.0,
                ))

            current_start = blk
            current_feed = active_feed
            current_speed = active_speed
            current_severity = active_severity
            current_reasons = list(active_reasons)
            current_marker_type = active_type

        # Flush final run
        if current_start is not None:
            merged.append(PredictiveOverride(
                trigger_block_index=current_start,
                marker_type=current_marker_type or 'MERGED',
                predicted_severity=current_severity or 0.0,
                feed_override_pct=current_feed or 100.0,
                speed_override_pct=current_speed or 100.0,
                reason='; '.join(current_reasons),
                expires_at_block=global_end,
                confidence=current_severity or 0.0,
            ))

        return merged

    def get_override_for_block(self, block_index: int) -> Optional[PredictiveOverride]:
        """Get the active override for a specific block index, if any."""
        for ov in self._predictive_overrides:
            start = getattr(ov, '_start_block', ov.trigger_block_index)
            if start <= block_index <= ov.expires_at_block:
                return ov
        return None

    def update_predictions(self, markers: list, current_block: int) -> None:
        """Called when new prediction results arrive. Updates the override schedule.

        Clears existing predictive overrides and recomputes from fresh markers.
        Only considers markers at or ahead of current_block.
        """
        # Filter to relevant markers (at or ahead of current block)
        relevant = [
            m for m in markers
            if getattr(m, 'block_index', 0) >= current_block
        ]

        raw_overrides = self._process_anomaly_markers(relevant)
        self._predictive_overrides = self._merge_overlapping_overrides(raw_overrides)

        # Publish immediate feed override if current block is within an override
        active = self.get_override_for_block(current_block)
        if active is not None:
            self._apply_predictive_override(active)

    def _apply_predictive_override(self, override: PredictiveOverride) -> None:
        """Publish a FeedOverride message from a predictive override."""
        self._target_feed_override = min(
            self._target_feed_override, override.feed_override_pct
        )
        self._target_spindle_override = min(
            self._target_spindle_override, override.speed_override_pct
        )

        try:
            msg = FeedOverride()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.feed_override_pct = override.feed_override_pct
            msg.spindle_override_pct = override.speed_override_pct
            msg.reason = f'predictive: {override.reason}'
            msg.confidence = override.confidence
            msg.revert_after_sec = 10.0
            self._override_pub.publish(msg)
            self.get_logger().info(
                f"Predictive override: feed={override.feed_override_pct:.0f}% "
                f"speed={override.speed_override_pct:.0f}% ({override.reason})"
            )
        except Exception:
            pass  # In test environments, publisher may be mocked

    def get_preemptive_status(self) -> dict:
        """Return current preemptive state for dashboard display."""
        active = self._current_preemptive
        return {
            'enabled': self._preemptive_enabled,
            'horizon': self._preemptive_horizon,
            'active': active is not None,
            'current_action': {
                'marker_type': active.marker_type,
                'blocks_ahead': active.blocks_ahead,
                'recommended_feed_pct': active.recommended_feed_pct,
                'recommended_speed_pct': active.recommended_speed_pct,
                'reason': active.reason,
                'confidence': active.confidence,
            } if active else None,
            'history_count': len(self._preemptive_actions),
            'min_confidence': self._preemptive_min_confidence,
        }

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
