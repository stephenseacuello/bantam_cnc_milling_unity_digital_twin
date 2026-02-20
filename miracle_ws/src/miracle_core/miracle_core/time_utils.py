"""
Time utilities for the MIRACLE system.

Provides helpers for ROS2 time conversions and duration calculations.
"""

from typing import Optional

from rclpy.time import Time
from rclpy.duration import Duration
from builtin_interfaces.msg import Time as TimeMsg


def time_to_float(time_msg: TimeMsg) -> float:
    """Convert a builtin_interfaces/Time message to float seconds."""
    return float(time_msg.sec) + float(time_msg.nanosec) * 1e-9


def float_to_time_msg(seconds: float) -> TimeMsg:
    """Convert float seconds to a builtin_interfaces/Time message."""
    msg = TimeMsg()
    msg.sec = int(seconds)
    msg.nanosec = int((seconds - msg.sec) * 1e9)
    return msg


def duration_between(start: Time, end: Time) -> float:
    """Compute duration in seconds between two ROS2 Time objects."""
    diff = end - start
    return diff.nanoseconds / 1e9


def is_expired(
    timestamp: Time,
    current_time: Time,
    timeout_sec: float,
) -> bool:
    """Check if a timestamp has expired given a timeout.

    Args:
        timestamp: The timestamp to check.
        current_time: The current time.
        timeout_sec: Timeout in seconds.

    Returns:
        True if the timestamp is older than timeout_sec.
    """
    elapsed = duration_between(timestamp, current_time)
    return elapsed > timeout_sec


def ros_time_now_msg(node) -> TimeMsg:
    """Get the current ROS time as a builtin_interfaces/Time message."""
    return node.get_clock().now().to_msg()
