"""
Chatter Detection Node.

Detects machining chatter (self-excited vibration) using
frequency domain analysis of audio and vibration data.
"""

from typing import Any, Optional
from collections import deque
import numpy as np

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import FusedSensorData, AnomalyAlert, MachineState


class ChatterDetectorNode(MiracleLifecycleNode):
    """Detects machining chatter from frequency analysis.

    Parameters:
        machine_id (str): Machine identifier.
        chatter_freq_min_hz (float): Min expected chatter frequency.
        chatter_freq_max_hz (float): Max expected chatter frequency.
        energy_ratio_threshold (float): Chatter band energy ratio threshold.
        detection_rate_hz (float): Detection check rate.

    Subscribed Topics:
        ~/sensor/fused (FusedSensorData): Fused sensor with audio features.
        ~/state (MachineState): Machine state for spindle speed.

    Published Topics:
        ~/chatter_alert (AnomalyAlert): Chatter detection alerts.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'chatter_detector',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._sensor_sub = None
        self._state_sub = None
        self._alert_pub = None
        self._machine_id: str = ''
        self._audio_buffer: deque = deque(maxlen=100)
        self._current_spindle_speed: float = 0.0
        self._chatter_freq_min: float = 500.0
        self._chatter_freq_max: float = 5000.0
        self._energy_threshold: float = 2.0

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure chatter detector."""
        params = self.declare_and_validate_parameters({
            'machine_id': {'default': 'cnc1', 'type': str},
            'chatter_freq_min_hz': {
                'default': 500.0,
                'type': float,
                'range': (50.0, 20000.0),
            },
            'chatter_freq_max_hz': {
                'default': 5000.0,
                'type': float,
                'range': (100.0, 20000.0),
            },
            'energy_ratio_threshold': {
                'default': 2.0,
                'type': float,
                'range': (1.0, 100.0),
            },
        })

        self._machine_id = params['machine_id']
        self._chatter_freq_min = params['chatter_freq_min_hz']
        self._chatter_freq_max = params['chatter_freq_max_hz']
        self._energy_threshold = params['energy_ratio_threshold']

        self._sensor_sub = self.create_subscription(
            FusedSensorData,
            f'/miracle/{self._machine_id}/sensor/fused',
            self._on_sensor_data,
            QoSProfiles.sensor_data(),
        )

        self._state_sub = self.create_subscription(
            MachineState,
            f'/miracle/{self._machine_id}/state',
            self._on_state,
            QoSProfiles.state_data(),
        )

        self._alert_pub = self.create_publisher(
            AnomalyAlert,
            f'/miracle/{self._machine_id}/chatter_alert',
            QoSProfiles.alert(),
        )

        self.get_logger().info(f"Chatter detector configured for {self._machine_id}")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate chatter detection."""
        self.get_logger().info("Chatter detector activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate chatter detection."""
        self._audio_buffer.clear()
        return TransitionCallbackReturn.SUCCESS

    def _on_state(self, msg: MachineState) -> None:
        """Track spindle speed for chatter frequency estimation."""
        self._current_spindle_speed = msg.spindle_speed

    def _on_sensor_data(self, msg: FusedSensorData) -> None:
        """Analyze audio features for chatter."""
        if not msg.audio_features or len(msg.audio_features) < 3:
            return

        # Audio features: [total_energy, spectral_centroid, band1..bandN]
        total_energy = msg.audio_features[0]
        spectral_centroid = msg.audio_features[1]
        band_energies = msg.audio_features[2:]

        self._audio_buffer.append({
            'total_energy': total_energy,
            'centroid': spectral_centroid,
            'bands': list(band_energies),
        })

        if len(self._audio_buffer) < 10:
            return

        # Compute baseline energy
        recent = list(self._audio_buffer)
        baseline_energy = np.mean([s['total_energy'] for s in recent[:-5]])
        current_energy = np.mean([s['total_energy'] for s in recent[-5:]])

        if baseline_energy <= 0:
            return

        energy_ratio = current_energy / (baseline_energy + 1e-10)

        # Check if energy ratio exceeds threshold (chatter indication)
        if energy_ratio > self._energy_threshold:
            severity = min(1.0, (energy_ratio - self._energy_threshold) / 5.0)

            msg_alert = AnomalyAlert()
            msg_alert.timestamp = self.get_clock().now().to_msg()
            msg_alert.machine_id = self._machine_id
            msg_alert.anomaly_type = 'CHATTER'
            msg_alert.confidence = min(0.95, 0.5 + severity * 0.4)
            msg_alert.severity = severity
            msg_alert.contributing_factors = [
                f'Energy ratio: {energy_ratio:.2f}',
                f'Spectral centroid: {spectral_centroid:.1f}',
                f'Spindle speed: {self._current_spindle_speed:.0f} RPM',
            ]
            msg_alert.feature_contributions = [energy_ratio, spectral_centroid]
            msg_alert.recommended_action = (
                'Reduce feed rate or change spindle speed to avoid chatter'
            )
            msg_alert.requires_immediate_stop = severity > 0.8

            self._alert_pub.publish(msg_alert)
            self.get_logger().warn(
                f"Chatter detected: energy_ratio={energy_ratio:.2f}, "
                f"severity={severity:.2f}"
            )


def main(args=None):
    """Entry point for the chatter detector node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = ChatterDetectorNode()
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
