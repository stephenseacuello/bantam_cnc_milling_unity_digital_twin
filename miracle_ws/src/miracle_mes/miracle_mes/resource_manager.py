"""
Resource Manager Node.

Manages shared manufacturing resources: tools, fixtures, materials,
and coolant. Tracks inventory and prevents resource conflicts.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, ToolWearEstimate


@dataclass
class ToolResource:
    """A cutting tool resource."""
    tool_id: str
    tool_type: str
    remaining_life_min: float = 100.0
    assigned_to: str = ''
    location: str = 'magazine'


@dataclass
class MaterialResource:
    """Raw material resource."""
    material_id: str
    material_type: str
    quantity: float = 0.0
    unit: str = 'units'


class ResourceManagerNode(MiracleLifecycleNode):
    """Manages shared manufacturing resources.

    Parameters:
        tool_inventory_size (int): Number of tools in inventory.
        low_tool_life_threshold (float): Minutes threshold for tool warning.
        check_interval_sec (float): Resource check interval.

    Subscribed Topics:
        /miracle/{machine_id}/tool_wear (ToolWearEstimate): Tool wear updates.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'resource_manager',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._tools: Dict[str, ToolResource] = {}
        self._materials: Dict[str, MaterialResource] = {}
        self._resources_lock = threading.Lock()
        self._tool_wear_subs = None
        self._check_timer = None

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure resource manager."""
        params = self.declare_and_validate_parameters({
            'tool_inventory_size': {
                'default': 50,
                'type': int,
                'range': (1, 1000),
            },
            'low_tool_life_threshold': {
                'default': 10.0,
                'type': float,
                'range': (1.0, 60.0),
            },
            'check_interval_sec': {
                'default': 30.0,
                'type': float,
                'range': (1.0, 300.0),
            },
            'machine_ids': {
                'default': 'cnc1,cnc2,cnc3',
                'type': str,
            },
        })

        machine_ids = self.get_machine_ids(params)

        self._tool_wear_subs = self.create_multi_machine_subscriptions(
            ToolWearEstimate,
            'tool_wear',
            self._on_tool_wear,
            QoSProfiles.state_data(),
            machine_ids,
        )

        # Initialize default tool inventory
        for i in range(params['tool_inventory_size']):
            tool_id = f'T{i+1:03d}'
            self._tools[tool_id] = ToolResource(
                tool_id=tool_id,
                tool_type='endmill',
                remaining_life_min=100.0,
            )

        self.get_logger().info(
            f"Resource manager configured with {len(self._tools)} tools"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate resource monitoring."""
        interval = self.get_parameter('check_interval_sec').value
        self._check_timer = self.create_timer(
            interval,
            self._check_resources,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("Resource manager activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate resource monitoring."""
        if self._check_timer is not None:
            self._check_timer.cancel()
            self._check_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _on_tool_wear(self, msg: ToolWearEstimate) -> None:
        """Update tool wear information."""
        with self._resources_lock:
            if msg.tool_id in self._tools:
                tool = self._tools[msg.tool_id]
                tool.remaining_life_min = msg.remaining_life_minutes

                threshold = self.get_parameter('low_tool_life_threshold').value
                if tool.remaining_life_min < threshold:
                    self.get_logger().warn(
                        f"Tool {msg.tool_id} low life: "
                        f"{tool.remaining_life_min:.1f} min remaining"
                    )

    def _check_resources(self) -> None:
        """Periodic resource check."""
        with self._resources_lock:
            threshold = self.get_parameter('low_tool_life_threshold').value
            low_life_tools = [
                t for t in self._tools.values()
                if t.remaining_life_min < threshold
            ]
            if low_life_tools:
                self.get_logger().info(
                    f"{len(low_life_tools)} tools below life threshold"
                )

    def allocate_tool(self, tool_type: str, machine_id: str) -> Optional[str]:
        """Allocate a tool to a machine.

        Args:
            tool_type: Requested tool type.
            machine_id: Machine requesting the tool.

        Returns:
            Tool ID if allocated, None if unavailable.
        """
        with self._resources_lock:
            for tool in self._tools.values():
                if (tool.tool_type == tool_type
                        and tool.assigned_to == ''
                        and tool.remaining_life_min > 5.0):
                    tool.assigned_to = machine_id
                    self.get_logger().info(
                        f"Tool {tool.tool_id} allocated to {machine_id}"
                    )
                    return tool.tool_id
        return None

    def release_tool(self, tool_id: str) -> None:
        """Release a tool back to inventory."""
        with self._resources_lock:
            if tool_id in self._tools:
                self._tools[tool_id].assigned_to = ''


def main(args=None):
    """Entry point for the resource manager node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = ResourceManagerNode()
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
