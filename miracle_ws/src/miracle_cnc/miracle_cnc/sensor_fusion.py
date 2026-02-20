"""
Multi-Modal Sensor Fusion Node.

Synchronizes and fuses data from IMU, current sensors, audio, and vision
into a unified feature vector for downstream ML processing.
"""

from typing import Any, List
import numpy as np

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import FusedSensorData
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Float32MultiArray


class SensorFusionNode(MiracleLifecycleNode):
    """Fuses multi-modal sensor data with temporal alignment.

    Parameters:
        machine_id (str): Machine identifier.
        sync_slop_sec (float): Time synchronization tolerance in seconds.
        imu_buffer_size (int): Number of IMU samples for statistics.
        current_buffer_size (int): Number of current samples for statistics.
        audio_buffer_size (int): Number of audio samples for statistics.

    Subscribed Topics:
        ~/sensor/imu (Imu): IMU data.
        ~/sensor/current (Float32MultiArray): Motor currents.
        ~/sensor/audio_fft (Float32MultiArray): Audio FFT data.
        ~/sensor/camera (Image): Camera image.

    Published Topics:
        ~/sensor/fused (FusedSensorData): Fused sensor features.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'sensor_fusion',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._fusion_pub = None
        self._imu_sub = None
        self._current_sub = None
        self._audio_sub = None
        self._vision_sub = None
        self._sync = None

        self._machine_id: str = ''
        self._sync_slop: float = 0.01

        self._imu_buffer: List[np.ndarray] = []
        self._current_buffer: List[np.ndarray] = []
        self._audio_buffer: List[np.ndarray] = []
        self._imu_buffer_size: int = 100
        self._current_buffer_size: int = 50
        self._audio_buffer_size: int = 10

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure sensor fusion."""
        params = self.declare_and_validate_parameters({
            'machine_id': {'default': 'cnc1', 'type': str},
            'sync_slop_sec': {
                'default': 0.01,
                'type': float,
                'range': (0.001, 0.1),
            },
            'imu_buffer_size': {
                'default': 100,
                'type': int,
                'range': (10, 10000),
            },
            'current_buffer_size': {
                'default': 50,
                'type': int,
                'range': (10, 5000),
            },
            'audio_buffer_size': {
                'default': 10,
                'type': int,
                'range': (1, 1000),
            },
        })

        self._machine_id = params['machine_id']
        self._sync_slop = params['sync_slop_sec']
        self._imu_buffer_size = params['imu_buffer_size']
        self._current_buffer_size = params['current_buffer_size']
        self._audio_buffer_size = params['audio_buffer_size']

        self._fusion_pub = self.create_publisher(
            FusedSensorData,
            'sensor/fused',
            QoSProfiles.sensor_data(),
        )

        self.get_logger().info(
            f"Configured sensor fusion for {self._machine_id} "
            f"with {self._sync_slop}s sync tolerance"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate synchronized subscriptions."""
        try:
            from message_filters import Subscriber, ApproximateTimeSynchronizer

            self._imu_sub = Subscriber(
                self, Imu, 'sensor/imu',
                qos_profile=QoSProfiles.sensor_data(),
            )
            self._current_sub = Subscriber(
                self, Float32MultiArray, 'sensor/current',
                qos_profile=QoSProfiles.sensor_data(),
            )
            self._audio_sub = Subscriber(
                self, Float32MultiArray, 'sensor/audio_fft',
                qos_profile=QoSProfiles.sensor_data(),
            )
            self._vision_sub = Subscriber(
                self, Image, 'sensor/camera',
                qos_profile=QoSProfiles.sensor_data(),
            )

            self._sync = ApproximateTimeSynchronizer(
                [self._imu_sub, self._current_sub,
                 self._audio_sub, self._vision_sub],
                queue_size=10,
                slop=self._sync_slop,
            )
            self._sync.registerCallback(self._synchronized_callback)

        except ImportError:
            self.get_logger().warn(
                "message_filters not available, using individual subscriptions"
            )
            self._imu_sub = self.create_subscription(
                Imu, 'sensor/imu',
                self._on_imu, QoSProfiles.sensor_data(),
            )
            self._current_sub = self.create_subscription(
                Float32MultiArray, 'sensor/current',
                self._on_current, QoSProfiles.sensor_data(),
            )
            self._audio_sub = self.create_subscription(
                Float32MultiArray, 'sensor/audio_fft',
                self._on_audio, QoSProfiles.sensor_data(),
            )

        self.get_logger().info("Sensor fusion activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate subscriptions."""
        self._imu_sub = None
        self._current_sub = None
        self._audio_sub = None
        self._vision_sub = None
        self._sync = None
        self._imu_buffer.clear()
        self._current_buffer.clear()
        self._audio_buffer.clear()
        return TransitionCallbackReturn.SUCCESS

    def _synchronized_callback(
        self,
        imu_msg: Imu,
        current_msg: Float32MultiArray,
        audio_msg: Float32MultiArray,
        vision_msg: Image,
    ) -> None:
        """Process synchronized sensor data."""
        try:
            imu_features = self._extract_imu_features(imu_msg)
            current_features = self._extract_current_features(current_msg)
            audio_features = self._extract_audio_features(audio_msg)
            vision_features = self._extract_vision_features(vision_msg)

            feature_vector = np.concatenate([
                imu_features, current_features,
                audio_features, vision_features,
            ])

            fused_msg = FusedSensorData()
            fused_msg.timestamp = self.get_clock().now().to_msg()
            fused_msg.machine_id = self._machine_id
            fused_msg.imu_features = imu_features.tolist()
            fused_msg.current_features = current_features.tolist()
            fused_msg.audio_features = audio_features.tolist()
            fused_msg.vision_features = vision_features.tolist()
            fused_msg.feature_vector = feature_vector.tolist()
            fused_msg.sensor_health = self._compute_sensor_health()
            fused_msg.synchronization_quality = 0.95

            self._fusion_pub.publish(fused_msg)

        except Exception as exc:
            self.get_logger().error(f"Sensor fusion error: {exc}")

    def _on_imu(self, msg: Imu) -> None:
        """Handle individual IMU messages (fallback mode)."""
        features = self._extract_imu_features(msg)
        self._publish_partial_fusion(imu_features=features)

    def _on_current(self, msg: Float32MultiArray) -> None:
        """Handle individual current messages (fallback mode)."""
        features = self._extract_current_features(msg)
        self._publish_partial_fusion(current_features=features)

    def _on_audio(self, msg: Float32MultiArray) -> None:
        """Handle individual audio messages (fallback mode)."""
        features = self._extract_audio_features(msg)
        self._publish_partial_fusion(audio_features=features)

    def _publish_partial_fusion(self, **kwargs: Any) -> None:
        """Publish a partial fusion message when not all sensors are synced."""
        fused_msg = FusedSensorData()
        fused_msg.timestamp = self.get_clock().now().to_msg()
        fused_msg.machine_id = self._machine_id

        imu_f = kwargs.get('imu_features', np.zeros(20))
        cur_f = kwargs.get('current_features', np.zeros(9))
        aud_f = kwargs.get('audio_features', np.zeros(10))
        vis_f = kwargs.get('vision_features', np.zeros(10))

        fused_msg.imu_features = imu_f.tolist()
        fused_msg.current_features = cur_f.tolist()
        fused_msg.audio_features = aud_f.tolist()
        fused_msg.vision_features = vis_f.tolist()
        fused_msg.feature_vector = np.concatenate([
            imu_f, cur_f, aud_f, vis_f,
        ]).tolist()
        fused_msg.sensor_health = self._compute_sensor_health()
        fused_msg.synchronization_quality = 0.5

        self._fusion_pub.publish(fused_msg)

    def _extract_imu_features(self, msg: Imu) -> np.ndarray:
        """Extract features from IMU data."""
        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])

        acc_magnitude = np.linalg.norm(acc)
        gyro_magnitude = np.linalg.norm(gyro)

        self._imu_buffer.append(np.concatenate([acc, gyro]))
        if len(self._imu_buffer) > self._imu_buffer_size:
            self._imu_buffer.pop(0)

        if len(self._imu_buffer) >= 10:
            buffer_array = np.array(self._imu_buffer)
            rms = np.sqrt(np.mean(buffer_array ** 2, axis=0))
            variance = np.var(buffer_array, axis=0)
        else:
            rms = np.zeros(6)
            variance = np.zeros(6)

        return np.concatenate([
            acc, gyro,
            [acc_magnitude, gyro_magnitude],
            rms, variance,
        ])

    def _extract_current_features(self, msg: Float32MultiArray) -> np.ndarray:
        """Extract features from motor current data."""
        currents = np.array(msg.data) if msg.data else np.zeros(6)
        mean_current = np.mean(currents)
        max_current = np.max(currents) if len(currents) > 0 else 0.0
        current_variance = np.var(currents)

        padded = np.zeros(6)
        padded[:min(len(currents), 6)] = currents[:6]

        return np.array([mean_current, max_current, current_variance, *padded])

    def _extract_audio_features(self, msg: Float32MultiArray) -> np.ndarray:
        """Extract features from audio FFT data."""
        fft_data = np.array(msg.data) if msg.data else np.array([])

        if len(fft_data) == 0:
            return np.zeros(10)

        n_bands = min(8, max(1, len(fft_data) // 10))
        band_size = len(fft_data) // n_bands
        band_energies = []

        for i in range(n_bands):
            start = i * band_size
            end = start + band_size
            band_energies.append(np.sum(fft_data[start:end] ** 2))

        # Pad to 8 bands
        while len(band_energies) < 8:
            band_energies.append(0.0)

        total_energy = np.sum(fft_data ** 2)
        spectral_centroid = (
            np.sum(np.arange(len(fft_data)) * fft_data ** 2)
            / (total_energy + 1e-10)
        )

        return np.array([total_energy, spectral_centroid, *band_energies])

    def _extract_vision_features(self, msg: Image) -> np.ndarray:
        """Extract features from vision data."""
        try:
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3,
                )
            elif msg.encoding == 'mono8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width,
                )
            else:
                return np.zeros(10)

            mean_intensity = np.mean(img)
            std_intensity = np.std(img)

            if len(img.shape) == 3:
                gray = np.mean(img, axis=2)
            else:
                gray = img.astype(float)

            edge_x = np.abs(np.diff(gray, axis=1)).mean()
            edge_y = np.abs(np.diff(gray, axis=0)).mean()

            return np.array([
                mean_intensity, std_intensity,
                edge_x, edge_y,
                float(np.min(img)), float(np.max(img)),
                0.0, 0.0, 0.0, 0.0,
            ])

        except Exception as exc:
            self.get_logger().debug(f"Vision feature extraction error: {exc}")
            return np.zeros(10)

    def _compute_sensor_health(self) -> int:
        """Compute sensor health bitfield."""
        health = 0
        if len(self._imu_buffer) > 0:
            health |= 0x01
        if len(self._current_buffer) > 0:
            health |= 0x02
        if len(self._audio_buffer) > 0:
            health |= 0x04
        health |= 0x08  # Vision assumed healthy from callback
        return health


def main(args=None):
    """Entry point for the sensor fusion node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = SensorFusionNode()
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
