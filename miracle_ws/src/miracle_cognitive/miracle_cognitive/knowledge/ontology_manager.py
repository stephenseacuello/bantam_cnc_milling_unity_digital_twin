"""
Manufacturing Ontology Manager.

Manages OWL ontology for the manufacturing domain.
Provides semantic reasoning and class hierarchy queries.
"""

from typing import Any, Dict, List
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles


class OntologyManagerNode(MiracleLifecycleNode):
    """Manages manufacturing domain ontology.

    Parameters:
        ontology_file (str): Path to OWL ontology file.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('ontology_manager', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._class_hierarchy: Dict[str, List[str]] = {}

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'ontology_file': {'default': '', 'type': str},
        })
        self._build_default_hierarchy()
        self.get_logger().info("Ontology manager configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("Ontology manager activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _build_default_hierarchy(self) -> None:
        """Build default manufacturing class hierarchy."""
        self._class_hierarchy = {
            'Thing': ['PhysicalAsset', 'Process', 'Parameter', 'Quality'],
            'PhysicalAsset': ['ManufacturingEquipment', 'Tool', 'Material', 'Sensor'],
            'ManufacturingEquipment': ['CNCMachine', 'Cobot', 'ConveyorBelt'],
            'CNCMachine': ['MillingMachine', 'Lathe', 'MultiAxis'],
            'Tool': ['CuttingTool', 'MeasuringTool'],
            'CuttingTool': ['EndMill', 'DrillBit', 'TurningInsert'],
            'Process': ['MachiningProcess', 'InspectionProcess', 'SetupProcess'],
            'MachiningProcess': ['Milling', 'Turning', 'Drilling', 'Grinding'],
            'Parameter': ['FeedRate', 'SpindleSpeed', 'DepthOfCut', 'CoolantFlow'],
            'Quality': ['SurfaceRoughness', 'DimensionalAccuracy', 'Flatness'],
        }

    def get_subclasses(self, class_name: str) -> List[str]:
        """Get direct subclasses of a class."""
        return self._class_hierarchy.get(class_name, [])

    def get_all_subclasses(self, class_name: str) -> List[str]:
        """Get all subclasses recursively."""
        result = []
        direct = self.get_subclasses(class_name)
        for sub in direct:
            result.append(sub)
            result.extend(self.get_all_subclasses(sub))
        return result

    def is_subclass_of(self, child: str, parent: str) -> bool:
        """Check if child is a subclass of parent."""
        return child in self.get_all_subclasses(parent)


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = OntologyManagerNode()
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
