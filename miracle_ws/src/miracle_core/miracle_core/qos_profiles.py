"""
Standardized QoS Profiles for the MIRACLE system.

Provides appropriate QoS configurations for different types of data:
- Sensor data: Best-effort, volatile (latest value matters most)
- State data: Reliable, transient local (new subscribers get last state)
- Command data: Reliable, keep last (commands must be delivered)
- Alert data: Reliable, keep all (no alert should be lost)
- Heartbeat: Best-effort, volatile (periodic, loss is acceptable)
"""

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
    QoSLivelinessPolicy,
)
from rclpy.duration import Duration


class QoSProfiles:
    """Standardized QoS profiles for the MIRACLE system."""

    @staticmethod
    def sensor_data() -> QoSProfile:
        """QoS for high-frequency sensor data streams.

        Best-effort delivery with small queue. Prioritizes low latency
        over guaranteed delivery since sensor data is periodic.
        """
        return QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            deadline=Duration(seconds=0, nanoseconds=100_000_000),  # 100ms
        )

    @staticmethod
    def state_data() -> QoSProfile:
        """QoS for machine/system state data.

        Reliable delivery with transient local durability so late
        subscribers receive the last known state.
        """
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

    @staticmethod
    def command() -> QoSProfile:
        """QoS for command messages.

        Reliable delivery to ensure commands are received.
        """
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

    @staticmethod
    def alert() -> QoSProfile:
        """QoS for alert and alarm messages.

        Reliable delivery with history to prevent alert loss.
        """
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_ALL,
        )

    @staticmethod
    def heartbeat() -> QoSProfile:
        """QoS for heartbeat messages.

        Best-effort with small queue. Missing a heartbeat is how
        failure is detected, so reliability is not needed.
        """
        return QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

    @staticmethod
    def bulk_data() -> QoSProfile:
        """QoS for bulk data transfers (models, large datasets).

        Reliable delivery with larger queue depth.
        """
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )

    @staticmethod
    def logging() -> QoSProfile:
        """QoS for logging and audit trail data.

        Reliable, keep all history for audit compliance.
        """
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_ALL,
        )
