"""
Scenario Manager Node.

Manages what-if scenarios for the digital twin.
Creates, runs, and compares multiple simulation scenarios
to support decision-making.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
import uuid
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState


@dataclass
class Scenario:
    """A what-if scenario definition."""
    scenario_id: str
    name: str
    description: str
    parameter_overrides: Dict[str, float] = field(default_factory=dict)
    status: str = 'CREATED'
    result_kpis: Dict[str, float] = field(default_factory=dict)


class ScenarioManagerNode(MiracleLifecycleNode):
    """Manages digital twin simulation scenarios.

    Parameters:
        max_scenarios (int): Maximum stored scenarios.
        auto_compare (bool): Automatically compare completed scenarios.

    Published Topics:
        ~/scenario_results (std_msgs/String): Scenario comparison results.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'scenario_manager',
            criticality=self.CRITICALITY_LOW,
            **kwargs,
        )
        self._scenarios: Dict[str, Scenario] = {}
        self._scenarios_lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure scenario manager."""
        self.declare_and_validate_parameters({
            'max_scenarios': {
                'default': 100,
                'type': int,
                'range': (1, 10000),
            },
            'auto_compare': {
                'default': True,
                'type': bool,
            },
        })

        self.get_logger().info("Scenario manager configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate scenario manager."""
        self.get_logger().info("Scenario manager activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate scenario manager."""
        return TransitionCallbackReturn.SUCCESS

    def create_scenario(
        self,
        name: str,
        description: str,
        parameter_overrides: Dict[str, float],
    ) -> str:
        """Create a new scenario.

        Args:
            name: Scenario name.
            description: Scenario description.
            parameter_overrides: Parameter changes to simulate.

        Returns:
            Scenario ID.
        """
        scenario_id = str(uuid.uuid4())[:8]
        scenario = Scenario(
            scenario_id=scenario_id,
            name=name,
            description=description,
            parameter_overrides=parameter_overrides,
        )

        with self._scenarios_lock:
            max_s = self.get_parameter('max_scenarios').value
            if len(self._scenarios) >= max_s:
                # Remove oldest
                oldest = min(self._scenarios.keys())
                del self._scenarios[oldest]

            self._scenarios[scenario_id] = scenario

        self.get_logger().info(f"Created scenario: {name} ({scenario_id})")
        return scenario_id

    def compare_scenarios(self, scenario_ids: List[str]) -> Dict[str, Any]:
        """Compare results of multiple scenarios.

        Args:
            scenario_ids: List of scenario IDs to compare.

        Returns:
            Comparison results dictionary.
        """
        with self._scenarios_lock:
            results: Dict[str, Any] = {}
            for sid in scenario_ids:
                if sid in self._scenarios:
                    s = self._scenarios[sid]
                    results[sid] = {
                        'name': s.name,
                        'status': s.status,
                        'kpis': dict(s.result_kpis),
                    }
        return results


def main(args=None):
    """Entry point for the scenario manager node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = ScenarioManagerNode()
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
