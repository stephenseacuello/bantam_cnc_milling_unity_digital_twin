"""
Base lifecycle node for all MIRACLE nodes.

Provides:
- ROS2 Lifecycle state machine integration
- Automatic heartbeat publishing
- Parameter validation
- Standardized callback groups
- Graceful shutdown handling
"""

from typing import Any, Dict, List, Optional
import signal

from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)

from miracle_core.heartbeat_mixin import HeartbeatMixin
from miracle_core.parameter_validation import (
    declare_and_validate_parameters,
    create_parameter_callback,
)
from miracle_core.exceptions import ConfigurationError


class MiracleLifecycleNode(LifecycleNode, HeartbeatMixin):
    """Base class for all MIRACLE lifecycle-managed nodes.

    Subclasses should override:
    - _do_configure(): Called during on_configure
    - _do_activate(): Called during on_activate
    - _do_deactivate(): Called during on_deactivate
    - _do_cleanup(): Called during on_cleanup
    - _do_shutdown(): Called during on_shutdown
    - _do_error(): Called during on_error

    Each method should return TransitionCallbackReturn.SUCCESS or FAILURE.
    """

    def __init__(
        self,
        node_name: str,
        criticality: str = 'MEDIUM',
        dependencies: Optional[List[str]] = None,
        heartbeat_interval: float = 0.5,
        **kwargs: Any,
    ) -> None:
        super().__init__(node_name, **kwargs)

        self._node_criticality = criticality
        self._node_dependencies = dependencies or []
        self._heartbeat_interval = heartbeat_interval

        # Standard callback groups
        self.realtime_callback_group = MutuallyExclusiveCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)

        self.get_logger().info(
            f"Node '{node_name}' created (criticality={criticality})"
        )

    def _signal_handler(self, signum: int, frame: Any) -> None:
        """Handle shutdown signals gracefully."""
        self.get_logger().info(
            f"Received signal {signum}, initiating graceful shutdown"
        )
        self._stop_heartbeat()

    # --- Lifecycle Callbacks ---

    def on_configure(self, state) -> TransitionCallbackReturn:
        """Handle configure transition."""
        self.get_logger().info("Configuring...")
        self._set_lifecycle_state('configuring')

        try:
            self._setup_heartbeat(
                criticality=self._node_criticality,
                dependencies=self._node_dependencies,
                interval_sec=self._heartbeat_interval,
            )
            result = self._do_configure()
        except Exception as exc:
            self.get_logger().error(f"Configuration failed: {exc}")
            return TransitionCallbackReturn.FAILURE

        if result == TransitionCallbackReturn.SUCCESS:
            self._set_lifecycle_state('inactive')
        return result

    def on_activate(self, state) -> TransitionCallbackReturn:
        """Handle activate transition."""
        self.get_logger().info("Activating...")
        self._set_lifecycle_state('activating')

        try:
            result = self._do_activate()
        except Exception as exc:
            self.get_logger().error(f"Activation failed: {exc}")
            return TransitionCallbackReturn.FAILURE

        if result == TransitionCallbackReturn.SUCCESS:
            self._start_heartbeat()
            self._set_lifecycle_state('active')

        return result

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        """Handle deactivate transition."""
        self.get_logger().info("Deactivating...")
        self._set_lifecycle_state('deactivating')
        self._stop_heartbeat()

        try:
            result = self._do_deactivate()
        except Exception as exc:
            self.get_logger().error(f"Deactivation failed: {exc}")
            return TransitionCallbackReturn.FAILURE

        if result == TransitionCallbackReturn.SUCCESS:
            self._set_lifecycle_state('inactive')

        return result

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        """Handle cleanup transition."""
        self.get_logger().info("Cleaning up...")
        self._stop_heartbeat()

        try:
            result = self._do_cleanup()
        except Exception as exc:
            self.get_logger().error(f"Cleanup failed: {exc}")
            return TransitionCallbackReturn.FAILURE

        if result == TransitionCallbackReturn.SUCCESS:
            self._set_lifecycle_state('unconfigured')

        return result

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        """Handle shutdown transition."""
        self.get_logger().info("Shutting down...")
        self._stop_heartbeat()

        try:
            result = self._do_shutdown()
        except Exception as exc:
            self.get_logger().error(f"Shutdown error: {exc}")
            return TransitionCallbackReturn.FAILURE

        self._set_lifecycle_state('finalized')
        return result

    def on_error(self, state) -> TransitionCallbackReturn:
        """Handle error transition."""
        self.get_logger().error("Error state entered")
        self._stop_heartbeat()

        try:
            result = self._do_error()
        except Exception as exc:
            self.get_logger().error(f"Error handler failed: {exc}")
            return TransitionCallbackReturn.FAILURE

        return result

    # --- Override Points ---

    def _do_configure(self) -> TransitionCallbackReturn:
        """Override to implement configuration logic."""
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Override to implement activation logic."""
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Override to implement deactivation logic."""
        return TransitionCallbackReturn.SUCCESS

    def _do_cleanup(self) -> TransitionCallbackReturn:
        """Override to implement cleanup logic."""
        return TransitionCallbackReturn.SUCCESS

    def _do_shutdown(self) -> TransitionCallbackReturn:
        """Override to implement shutdown logic."""
        return TransitionCallbackReturn.SUCCESS

    def _do_error(self) -> TransitionCallbackReturn:
        """Override to implement error handling logic."""
        return TransitionCallbackReturn.SUCCESS

    # --- Utility Methods ---

    def declare_and_validate_parameters(
        self, param_specs: Dict[str, Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Declare and validate parameters using the standard utility.

        Args:
            param_specs: Parameter specifications dict.

        Returns:
            Dict of validated parameter values.
        """
        validated = declare_and_validate_parameters(self, param_specs)
        callback = create_parameter_callback(self, param_specs)
        self.add_on_set_parameters_callback(callback)
        return validated

    def create_multi_machine_subscriptions(
        self,
        msg_type: Any,
        topic_suffix: str,
        callback: Any,
        qos: Any,
        machine_ids: Optional[List[str]] = None,
    ) -> List:
        """Create per-machine subscriptions for a topic pattern.

        Replaces MQTT-style '/miracle/+/suffix' wildcards with concrete
        per-machine subscriptions, since ROS2 DDS does not support wildcards.

        Args:
            msg_type: Message type class.
            topic_suffix: Topic suffix after machine ID (e.g. 'state', 'anomaly').
            callback: Subscription callback.
            qos: QoS profile.
            machine_ids: List of machine IDs. Defaults to ['cnc1', 'cnc2', 'cnc3'].

        Returns:
            List of subscription objects.
        """
        if machine_ids is None:
            machine_ids = getattr(self, '_miracle_machine_ids', None)
            if machine_ids is None:
                machine_ids = ['cnc1', 'cnc2', 'cnc3']

        subs = []
        for mid in machine_ids:
            subs.append(self.create_subscription(
                msg_type,
                f'/miracle/{mid}/{topic_suffix}',
                callback,
                qos,
            ))
        return subs

    def get_machine_ids(self, params: Dict[str, Any]) -> List[str]:
        """Parse machine_ids from a parameter dict and cache the result.

        Args:
            params: Parameter dict containing 'machine_ids' string.

        Returns:
            List of machine ID strings.
        """
        raw = params.get('machine_ids', 'cnc1,cnc2,cnc3')
        ids = [m.strip() for m in raw.split(',') if m.strip()]
        self._miracle_machine_ids = ids
        return ids
