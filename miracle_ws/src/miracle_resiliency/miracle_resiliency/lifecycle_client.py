"""
Lifecycle Client -- Async wrapper for ROS2 lifecycle service calls.

Provides typed transition methods for managing ROS2 lifecycle nodes:
configure -> activate -> deactivate -> cleanup -> shutdown
"""

import asyncio
from typing import Optional
from enum import IntEnum


class LifecycleTransition(IntEnum):
    """ROS2 lifecycle transition IDs (from lifecycle_msgs)."""
    CONFIGURE = 1
    CLEANUP = 2
    ACTIVATE = 3
    DEACTIVATE = 4
    SHUTDOWN = 5
    DESTROY = 6


class LifecycleClientError(Exception):
    """Error during lifecycle transition."""
    pass


class LifecycleClient:
    """Async wrapper around ROS2 lifecycle service calls.

    Calls /{node_name}/change_state service to transition nodes
    through their lifecycle states.
    """

    def __init__(self, ros_node, timeout_sec: float = 10.0):
        """Initialize lifecycle client.

        Args:
            ros_node: The ROS2 node to create service clients on.
            timeout_sec: Default timeout per transition.
        """
        self._node = ros_node
        self._timeout = timeout_sec
        self._clients = {}

    def _get_or_create_client(self, node_name: str):
        """Get or create a service client for the target node."""
        if node_name not in self._clients:
            try:
                from lifecycle_msgs.srv import ChangeState
                service_name = f'/{node_name}/change_state'
                client = self._node.create_client(ChangeState, service_name)
                self._clients[node_name] = client
            except ImportError:
                # Fallback when lifecycle_msgs is not available (testing)
                self._clients[node_name] = None
        return self._clients[node_name]

    async def change_state(
        self, node_name: str, transition: LifecycleTransition,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        """Send a lifecycle transition to the target node.

        Args:
            node_name: Name of the target lifecycle node.
            transition: The transition to request.
            timeout_sec: Override default timeout.

        Returns:
            True if transition succeeded, False otherwise.

        Raises:
            LifecycleClientError: On timeout or service unavailable.
        """
        timeout = timeout_sec or self._timeout
        client = self._get_or_create_client(node_name)

        if client is None:
            self._node.get_logger().warn(
                f"Lifecycle client unavailable for {node_name} "
                f"(lifecycle_msgs not installed). Simulating transition."
            )
            return True

        # Wait for service to be available
        if not client.wait_for_service(timeout_sec=timeout):
            raise LifecycleClientError(
                f"Service /{node_name}/change_state not available after {timeout}s"
            )

        try:
            from lifecycle_msgs.srv import ChangeState
            from lifecycle_msgs.msg import Transition

            request = ChangeState.Request()
            request.transition = Transition()
            request.transition.id = int(transition)

            future = client.call_async(request)
            # Use asyncio wait with timeout
            result = await asyncio.wait_for(
                asyncio.ensure_future(self._wait_for_future(future)),
                timeout=timeout,
            )

            if result.success:
                self._node.get_logger().info(
                    f"Lifecycle transition {transition.name} succeeded for {node_name}"
                )
                return True
            else:
                self._node.get_logger().warn(
                    f"Lifecycle transition {transition.name} failed for {node_name}"
                )
                return False

        except asyncio.TimeoutError:
            raise LifecycleClientError(
                f"Timeout waiting for {transition.name} on {node_name} ({timeout}s)"
            )
        except Exception as exc:
            raise LifecycleClientError(
                f"Error during {transition.name} on {node_name}: {exc}"
            )

    async def _wait_for_future(self, future):
        """Wait for a ROS2 future to complete."""
        while not future.done():
            await asyncio.sleep(0.01)
        return future.result()

    async def configure(self, node_name: str, timeout_sec: Optional[float] = None) -> bool:
        return await self.change_state(node_name, LifecycleTransition.CONFIGURE, timeout_sec)

    async def activate(self, node_name: str, timeout_sec: Optional[float] = None) -> bool:
        return await self.change_state(node_name, LifecycleTransition.ACTIVATE, timeout_sec)

    async def deactivate(self, node_name: str, timeout_sec: Optional[float] = None) -> bool:
        return await self.change_state(node_name, LifecycleTransition.DEACTIVATE, timeout_sec)

    async def cleanup(self, node_name: str, timeout_sec: Optional[float] = None) -> bool:
        return await self.change_state(node_name, LifecycleTransition.CLEANUP, timeout_sec)

    async def shutdown(self, node_name: str, timeout_sec: Optional[float] = None) -> bool:
        return await self.change_state(node_name, LifecycleTransition.SHUTDOWN, timeout_sec)

    async def full_restart(self, node_name: str, timeout_sec: Optional[float] = None) -> bool:
        """Execute full restart sequence: deactivate -> cleanup -> configure -> activate."""
        steps = [
            (LifecycleTransition.DEACTIVATE, "deactivate"),
            (LifecycleTransition.CLEANUP, "cleanup"),
            (LifecycleTransition.CONFIGURE, "configure"),
            (LifecycleTransition.ACTIVATE, "activate"),
        ]
        for transition, name in steps:
            try:
                success = await self.change_state(node_name, transition, timeout_sec)
                if not success:
                    self._node.get_logger().error(
                        f"Restart failed at {name} for {node_name}"
                    )
                    return False
            except LifecycleClientError as exc:
                self._node.get_logger().error(f"Restart error at {name}: {exc}")
                return False
        return True
