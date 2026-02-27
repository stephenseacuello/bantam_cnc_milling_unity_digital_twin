"""
Lifecycle Auto-Start Node.

Discovers all lifecycle nodes and transitions them through
configure -> activate after a startup delay. Uses a single DDS
participant for fast service discovery instead of spawning
separate CLI processes.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition


class LifecycleAutoStartNode(Node):
    """Auto-configures and activates all lifecycle nodes."""

    def __init__(self):
        super().__init__('lifecycle_autostart')

        self.declare_parameter('startup_delay', 10.0)
        self.declare_parameter('per_node_timeout', 10.0)
        self.declare_parameter('max_retries', 2)

        delay = self.get_parameter('startup_delay').value
        self.get_logger().info(
            f"Will auto-activate lifecycle nodes in {delay}s..."
        )

        # Run in background thread to not block the executor
        self._thread = threading.Thread(
            target=self._activate_all, args=(delay,), daemon=True
        )
        self._thread.start()

    def _activate_all(self, delay: float):
        """Wait for discovery, then activate all lifecycle nodes."""
        time.sleep(delay)

        timeout = self.get_parameter('per_node_timeout').value
        max_retries = self.get_parameter('max_retries').value

        # Discover lifecycle nodes by looking for change_state services
        service_names = self.get_service_names_and_types()
        lifecycle_nodes = set()
        for srv_name, srv_types in service_names:
            if srv_name.endswith('/change_state'):
                for t in srv_types:
                    if 'lifecycle_msgs' in t:
                        node_name = srv_name.rsplit('/change_state', 1)[0]
                        lifecycle_nodes.add(node_name)
                        break

        # Remove ourselves
        lifecycle_nodes.discard(self.get_fully_qualified_name())

        self.get_logger().info(
            f"Discovered {len(lifecycle_nodes)} lifecycle nodes"
        )

        ok = 0
        fail = 0
        for node_name in sorted(lifecycle_nodes):
            success = self._transition_node(node_name, timeout, max_retries)
            if success:
                ok += 1
            else:
                fail += 1

        self.get_logger().info(
            f"Auto-start complete: {ok} activated, {fail} failed"
        )

    def _transition_node(
        self, node_name: str, timeout: float, max_retries: int
    ) -> bool:
        """Configure and activate a single lifecycle node."""
        transitions = [
            (Transition.TRANSITION_CONFIGURE, 'configure'),
            (Transition.TRANSITION_ACTIVATE, 'activate'),
        ]

        for transition_id, label in transitions:
            srv_name = f'{node_name}/change_state'
            client = self.create_client(ChangeState, srv_name)

            try:
                if not client.wait_for_service(timeout_sec=timeout):
                    self.get_logger().warn(
                        f"  SKIP {node_name}: service not available"
                    )
                    return False

                req = ChangeState.Request()
                req.transition.id = transition_id

                for attempt in range(max_retries + 1):
                    future = client.call_async(req)
                    # Spin in this thread until done
                    start = time.time()
                    while not future.done():
                        time.sleep(0.05)
                        if time.time() - start > timeout:
                            break

                    if future.done() and future.result() is not None:
                        if future.result().success:
                            break
                        # Already in target state is OK
                        break
                    elif attempt < max_retries:
                        self.get_logger().debug(
                            f"  Retry {label} for {node_name}"
                        )
                        time.sleep(0.5)
                    else:
                        self.get_logger().warn(
                            f"  FAIL {node_name}: {label} timed out"
                        )
                        return False
            finally:
                self.destroy_client(client)

        return True


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = LifecycleAutoStartNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
