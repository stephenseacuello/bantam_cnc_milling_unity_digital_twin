"""
Logging configuration for MIRACLE nodes.

Provides structured logging utilities and log formatting.
"""

from typing import Any, Dict, Optional
import json
import time


class StructuredLogger:
    """Wraps a ROS2 logger with structured logging capabilities."""

    def __init__(self, ros_logger: Any, node_name: str) -> None:
        self._logger = ros_logger
        self._node_name = node_name

    def _format_structured(
        self,
        message: str,
        extra: Optional[Dict[str, Any]] = None,
    ) -> str:
        """Format a message with structured data."""
        entry: Dict[str, Any] = {
            'node': self._node_name,
            'ts': time.time(),
            'msg': message,
        }
        if extra:
            entry['data'] = extra
        return json.dumps(entry)

    def info(
        self, message: str, extra: Optional[Dict[str, Any]] = None
    ) -> None:
        """Log an info message."""
        self._logger.info(self._format_structured(message, extra))

    def warn(
        self, message: str, extra: Optional[Dict[str, Any]] = None
    ) -> None:
        """Log a warning message."""
        self._logger.warn(self._format_structured(message, extra))

    def error(
        self, message: str, extra: Optional[Dict[str, Any]] = None
    ) -> None:
        """Log an error message."""
        self._logger.error(self._format_structured(message, extra))

    def debug(
        self, message: str, extra: Optional[Dict[str, Any]] = None
    ) -> None:
        """Log a debug message."""
        self._logger.debug(self._format_structured(message, extra))
