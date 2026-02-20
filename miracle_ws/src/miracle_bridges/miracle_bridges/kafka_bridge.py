"""
Apache Kafka Bridge Node.

Bridges ROS2 topics to Apache Kafka for enterprise integration,
data lake ingestion, and event streaming.
"""

from typing import Any, Dict, List, Optional
import json
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_core.serialization import msg_to_dict
from miracle_msgs.msg import MachineState, AnomalyAlert, JobStatus, SystemKPIs


class KafkaBridgeNode(MiracleLifecycleNode):
    """Bridges ROS2 topics to Apache Kafka.

    Parameters:
        bootstrap_servers (str): Kafka bootstrap servers.
        machine_id (str): Machine identifier for topic prefix.
        batch_size (int): Messages per Kafka batch.
        linger_ms (int): Kafka producer linger time in ms.

    Subscribed Topics:
        /miracle/+/state (MachineState): Machine states.
        /miracle/+/anomaly (AnomalyAlert): Anomaly alerts.
        /miracle/+/job_status (JobStatus): Job status updates.
        /miracle/system_kpis (SystemKPIs): System KPIs.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'kafka_bridge',
            criticality=self.CRITICALITY_LOW,
            **kwargs,
        )
        self._producer = None
        self._connected: bool = False
        self._state_sub = None
        self._anomaly_sub = None
        self._job_sub = None
        self._kpi_sub = None
        self._message_count: int = 0
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure Kafka bridge."""
        params = self.declare_and_validate_parameters({
            'bootstrap_servers': {
                'default': 'localhost:9092',
                'type': str,
            },
            'machine_id': {
                'default': 'cnc1',
                'type': str,
            },
            'batch_size': {
                'default': 100,
                'type': int,
                'range': (1, 10000),
            },
            'linger_ms': {
                'default': 100,
                'type': int,
                'range': (0, 60000),
            },
        })

        self._state_sub = self.create_subscription(
            MachineState,
            '/miracle/+/state',
            self._on_state,
            QoSProfiles.state_data(),
        )

        self._anomaly_sub = self.create_subscription(
            AnomalyAlert,
            '/miracle/+/anomaly',
            self._on_anomaly,
            QoSProfiles.alert(),
        )

        self._job_sub = self.create_subscription(
            JobStatus,
            '/miracle/+/job_status',
            self._on_job_status,
            QoSProfiles.state_data(),
        )

        self._kpi_sub = self.create_subscription(
            SystemKPIs,
            '/miracle/system_kpis',
            self._on_kpis,
            QoSProfiles.state_data(),
        )

        self.get_logger().info(
            f"Kafka bridge configured: {params['bootstrap_servers']}"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Connect to Kafka."""
        self._connect_kafka()
        self.get_logger().info("Kafka bridge activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Disconnect from Kafka."""
        self._disconnect_kafka()
        return TransitionCallbackReturn.SUCCESS

    def _connect_kafka(self) -> None:
        """Establish Kafka producer connection."""
        try:
            from kafka import KafkaProducer

            servers = self.get_parameter('bootstrap_servers').value
            batch = self.get_parameter('batch_size').value
            linger = self.get_parameter('linger_ms').value

            self._producer = KafkaProducer(
                bootstrap_servers=servers,
                value_serializer=lambda v: json.dumps(v, default=str).encode(),
                batch_size=batch,
                linger_ms=linger,
            )
            self._connected = True
            self.get_logger().info(f"Connected to Kafka: {servers}")
        except ImportError:
            self.get_logger().warn("kafka-python not installed, bridge inactive")
            self._connected = False
        except Exception as exc:
            self.get_logger().warn(f"Kafka connection failed: {exc}")
            self._connected = False

    def _disconnect_kafka(self) -> None:
        """Disconnect from Kafka."""
        if self._producer is not None:
            try:
                self._producer.flush()
                self._producer.close()
            except Exception as exc:
                self.get_logger().warn(f"Kafka disconnect error: {exc}")
            self._connected = False
            self._producer = None

    def _send_to_kafka(self, topic: str, data: Dict) -> None:
        """Send data to Kafka topic."""
        if not self._connected or self._producer is None:
            return

        try:
            self._producer.send(topic, value=data)
            with self._lock:
                self._message_count += 1
        except Exception as exc:
            self.get_logger().error(f"Kafka send error: {exc}")

    def _on_state(self, msg: MachineState) -> None:
        """Forward machine state to Kafka."""
        self._send_to_kafka('miracle.machine_state', msg_to_dict(msg))

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        """Forward anomaly alerts to Kafka."""
        self._send_to_kafka('miracle.anomaly_alerts', msg_to_dict(msg))

    def _on_job_status(self, msg: JobStatus) -> None:
        """Forward job status to Kafka."""
        self._send_to_kafka('miracle.job_status', msg_to_dict(msg))

    def _on_kpis(self, msg: SystemKPIs) -> None:
        """Forward system KPIs to Kafka."""
        self._send_to_kafka('miracle.system_kpis', msg_to_dict(msg))


def main(args=None):
    """Entry point for the Kafka bridge node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = KafkaBridgeNode()
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
