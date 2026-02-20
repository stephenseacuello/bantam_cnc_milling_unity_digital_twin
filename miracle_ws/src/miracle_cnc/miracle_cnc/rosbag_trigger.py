"""
Rosbag Trigger Node.

Monitors for anomaly events and triggers rosbag2 recording
to capture data surrounding anomaly events for post-analysis.
"""

from typing import Any, Optional
import subprocess
import os

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import AnomalyAlert


class RosbagTriggerNode(MiracleLifecycleNode):
    """Triggers rosbag recording on anomaly detection.

    Parameters:
        machine_id (str): Machine identifier.
        output_directory (str): Directory for bag files.
        pre_trigger_sec (float): Seconds of pre-trigger buffering.
        post_trigger_sec (float): Seconds to record after trigger.
        min_severity (float): Minimum severity to trigger recording.

    Subscribed Topics:
        ~/anomaly (AnomalyAlert): Anomaly alerts that trigger recording.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'rosbag_trigger',
            criticality=self.CRITICALITY_LOW,
            **kwargs,
        )
        self._anomaly_sub = None
        self._machine_id: str = ''
        self._output_dir: str = '/tmp/miracle_bags'
        self._post_trigger_sec: float = 30.0
        self._min_severity: float = 0.5
        self._recording_process: Optional[subprocess.Popen] = None

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure rosbag trigger."""
        params = self.declare_and_validate_parameters({
            'machine_id': {'default': 'cnc1', 'type': str},
            'output_directory': {
                'default': '/tmp/miracle_bags',
                'type': str,
            },
            'post_trigger_sec': {
                'default': 30.0,
                'type': float,
                'range': (5.0, 300.0),
            },
            'min_severity': {
                'default': 0.5,
                'type': float,
                'range': (0.0, 1.0),
            },
        })

        self._machine_id = params['machine_id']
        self._output_dir = params['output_directory']
        self._post_trigger_sec = params['post_trigger_sec']
        self._min_severity = params['min_severity']

        os.makedirs(self._output_dir, exist_ok=True)

        self._anomaly_sub = self.create_subscription(
            AnomalyAlert,
            'anomaly',
            self._on_anomaly,
            QoSProfiles.alert(),
        )

        self.get_logger().info(
            f"Rosbag trigger configured for {self._machine_id}"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate rosbag trigger."""
        self.get_logger().info("Rosbag trigger activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate and stop any recording."""
        self._stop_recording()
        return TransitionCallbackReturn.SUCCESS

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        """Handle anomaly alerts."""
        if msg.severity < self._min_severity:
            return

        if self._recording_process is not None:
            self.get_logger().info("Already recording, skipping trigger")
            return

        self.get_logger().info(
            f"Triggering rosbag recording: {msg.anomaly_type} "
            f"(severity={msg.severity:.2f})"
        )
        self._start_recording(msg)

    def _start_recording(self, alert: AnomalyAlert) -> None:
        """Start rosbag recording."""
        bag_name = (
            f"{self._machine_id}_{alert.anomaly_type}_"
            f"{self.get_clock().now().nanoseconds}"
        )
        bag_path = os.path.join(self._output_dir, bag_name)

        # Topics to record
        topics = [
            f'/miracle/{self._machine_id}/state',
            f'/miracle/{self._machine_id}/sensor/fused',
            f'/miracle/{self._machine_id}/anomaly',
            f'/miracle/{self._machine_id}/gcode_block',
        ]

        cmd = [
            'ros2', 'bag', 'record',
            '-o', bag_path,
            '--max-bag-duration', str(int(self._post_trigger_sec)),
        ] + topics

        try:
            self._recording_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.get_logger().info(f"Recording started: {bag_path}")

            # Schedule stop
            self.create_timer(
                self._post_trigger_sec,
                self._stop_recording_callback,
                callback_group=self.service_callback_group,
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to start recording: {exc}")
            self._recording_process = None

    def _stop_recording_callback(self) -> None:
        """Timer callback to stop recording."""
        self._stop_recording()

    def _stop_recording(self) -> None:
        """Stop any active rosbag recording."""
        if self._recording_process is not None:
            self._recording_process.terminate()
            self._recording_process.wait(timeout=5)
            self._recording_process = None
            self.get_logger().info("Recording stopped")


def main(args=None):
    """Entry point for the rosbag trigger node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = RosbagTriggerNode()
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
