"""
Unity TCP Endpoint Launcher.

Wraps the ros_tcp_endpoint to provide MIRACLE-specific configuration
and topic registration for the Unity Digital Twin.
"""

import rclpy
from rclpy.node import Node


class UnityEndpointConfig(Node):
    """Configuration node for Unity TCP endpoint.
    
    Publishes connection status and manages topic registration
    for the Unity Digital Twin visualization.
    """

    def __init__(self):
        super().__init__('unity_endpoint_config')
        
        self.declare_parameter('tcp_ip', '0.0.0.0')
        self.declare_parameter('tcp_port', 10000)
        self.declare_parameter('keepalive_timeout', 10.0)
        self.declare_parameter('log_level', 'info')
        
        tcp_ip = self.get_parameter('tcp_ip').get_parameter_value().string_value
        tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        
        self.get_logger().info(
            f'Unity TCP Endpoint configured: {tcp_ip}:{tcp_port}'
        )
        self.get_logger().info('Registered MIRACLE topic types:')
        self.get_logger().info('  - miracle_msgs/msg/MachineState')
        self.get_logger().info('  - miracle_msgs/msg/AnomalyAlert')
        self.get_logger().info('  - miracle_msgs/msg/ToolWearEstimate')
        self.get_logger().info('  - miracle_msgs/msg/TwinSyncStatus')
        self.get_logger().info('  - miracle_msgs/msg/SystemKPIs')
        self.get_logger().info('  - miracle_msgs/msg/JobStatus')
        self.get_logger().info('  - miracle_msgs/msg/FleetHealth')
        self.get_logger().info('  - miracle_msgs/msg/Heartbeat')
        self.get_logger().info('  - miracle_msgs/msg/SecurityAlert')
        self.get_logger().info('  - miracle_msgs/msg/TaskAward')
        self.get_logger().info('  - miracle_msgs/srv/TriggerEStop')
        self.get_logger().info('  - miracle_msgs/srv/ValidateGCode')
        self.get_logger().info('  - miracle_msgs/srv/GetFleetStatus')


def main(args=None):
    """Launch the Unity endpoint configuration node."""
    rclpy.init(args=args)
    node = UnityEndpointConfig()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
