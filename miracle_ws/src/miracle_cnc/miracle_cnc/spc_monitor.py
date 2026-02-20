"""
Statistical Process Control (SPC) Monitor Node.

Implements real-time SPC with Western Electric rules for
detecting process shifts and out-of-control conditions.
"""

from typing import Any, Dict, List, Optional
from collections import deque
import math

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, AnomalyAlert


class SPCMonitorNode(MiracleLifecycleNode):
    """SPC monitoring with configurable control limits.

    Parameters:
        machine_id (str): Machine identifier.
        window_size (int): Sample window for statistics.
        sigma_limit (float): Number of sigma for control limits.
        warmup_samples (int): Samples needed before monitoring.

    Subscribed Topics:
        ~/state (MachineState): Machine state with process variables.

    Published Topics:
        ~/spc_alert (AnomalyAlert): SPC violation alerts.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'spc_monitor',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._state_sub = None
        self._alert_pub = None
        self._machine_id: str = ''
        self._window_size: int = 100
        self._sigma_limit: float = 3.0
        self._warmup: int = 30

        # SPC data buffers for each monitored variable
        self._buffers: Dict[str, deque] = {}
        self._means: Dict[str, float] = {}
        self._stds: Dict[str, float] = {}
        self._sample_count: int = 0

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure SPC monitor."""
        params = self.declare_and_validate_parameters({
            'machine_id': {'default': 'cnc1', 'type': str},
            'window_size': {
                'default': 100,
                'type': int,
                'range': (20, 10000),
            },
            'sigma_limit': {
                'default': 3.0,
                'type': float,
                'range': (1.0, 6.0),
            },
            'warmup_samples': {
                'default': 30,
                'type': int,
                'range': (10, 1000),
            },
        })

        self._machine_id = params['machine_id']
        self._window_size = params['window_size']
        self._sigma_limit = params['sigma_limit']
        self._warmup = params['warmup_samples']

        # Initialize buffers for monitored variables
        for var_name in ['spindle_load', 'feed_rate', 'spindle_speed']:
            self._buffers[var_name] = deque(maxlen=self._window_size)

        self._alert_pub = self.create_publisher(
            AnomalyAlert,
            'spc_alert',
            QoSProfiles.alert(),
        )

        self._state_sub = self.create_subscription(
            MachineState,
            'state',
            self._on_state,
            QoSProfiles.state_data(),
        )

        self.get_logger().info(f"SPC monitor configured for {self._machine_id}")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate SPC monitoring."""
        self.get_logger().info("SPC monitoring activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate SPC monitoring."""
        for buf in self._buffers.values():
            buf.clear()
        self._sample_count = 0
        return TransitionCallbackReturn.SUCCESS

    def _on_state(self, msg: MachineState) -> None:
        """Process state message for SPC analysis."""
        if msg.status != 'RUNNING':
            return

        self._sample_count += 1

        # Collect variables
        variables = {
            'spindle_load': msg.spindle_load,
            'feed_rate': msg.feed_rate,
            'spindle_speed': msg.spindle_speed,
        }

        for var_name, value in variables.items():
            self._buffers[var_name].append(value)

            if self._sample_count < self._warmup:
                continue

            # Update statistics
            buf = list(self._buffers[var_name])
            mean = sum(buf) / len(buf)
            variance = sum((x - mean) ** 2 for x in buf) / len(buf)
            std = math.sqrt(variance) if variance > 0 else 1e-10

            self._means[var_name] = mean
            self._stds[var_name] = std

            # Check Western Electric rules
            violations = self._check_we_rules(var_name, value, mean, std)
            if violations:
                self._publish_spc_alert(var_name, value, mean, std, violations)

    def _check_we_rules(
        self,
        var_name: str,
        value: float,
        mean: float,
        std: float,
    ) -> List[str]:
        """Check Western Electric rules for out-of-control conditions."""
        violations: List[str] = []
        sigma = self._sigma_limit

        # Rule 1: Single point beyond control limits
        if abs(value - mean) > sigma * std:
            violations.append(
                f"Rule 1: {var_name}={value:.2f} beyond {sigma}-sigma limit"
            )

        # Rule 2: 2 of 3 consecutive points beyond 2-sigma
        buf = list(self._buffers[var_name])
        if len(buf) >= 3:
            last3 = buf[-3:]
            beyond_2sigma = sum(1 for v in last3 if abs(v - mean) > 2 * std)
            if beyond_2sigma >= 2:
                violations.append(
                    f"Rule 2: 2 of 3 points beyond 2-sigma for {var_name}"
                )

        # Rule 3: 4 of 5 consecutive points beyond 1-sigma
        if len(buf) >= 5:
            last5 = buf[-5:]
            beyond_1sigma = sum(1 for v in last5 if abs(v - mean) > std)
            if beyond_1sigma >= 4:
                violations.append(
                    f"Rule 3: 4 of 5 points beyond 1-sigma for {var_name}"
                )

        # Rule 4: 8 consecutive points on same side of center
        if len(buf) >= 8:
            last8 = buf[-8:]
            above = all(v > mean for v in last8)
            below = all(v < mean for v in last8)
            if above or below:
                side = 'above' if above else 'below'
                violations.append(
                    f"Rule 4: 8 consecutive points {side} center for {var_name}"
                )

        return violations

    def _publish_spc_alert(
        self,
        var_name: str,
        value: float,
        mean: float,
        std: float,
        violations: List[str],
    ) -> None:
        """Publish SPC violation alert."""
        msg = AnomalyAlert()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.anomaly_type = 'SPC_VIOLATION'
        msg.confidence = 0.85
        msg.severity = min(1.0, abs(value - mean) / (3 * std + 1e-10))
        msg.contributing_factors = violations
        msg.feature_contributions = [value, mean, std]
        msg.recommended_action = f"Investigate {var_name} drift"
        msg.requires_immediate_stop = False

        self._alert_pub.publish(msg)
        self.get_logger().warn(f"SPC violation: {'; '.join(violations)}")


def main(args=None):
    """Entry point for the SPC monitor node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = SPCMonitorNode()
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
