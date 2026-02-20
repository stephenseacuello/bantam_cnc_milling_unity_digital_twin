"""
Unity Bridge Launch File.

Launches ros_tcp_endpoint for Unity ROS-TCP-Connector communication
plus the MIRACLE-specific endpoint configuration node.

The ros_tcp_endpoint bridges ALL DDS topics to Unity over TCP port 10000.
Unity-side uses ROS-TCP-Connector package to subscribe/publish.

Arguments:
    tcp_ip   - IP address to bind (default: 0.0.0.0)
    tcp_port - TCP port for Unity connection (default: 10000)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the Unity bridge launch description."""
    pkg_share = get_package_share_directory('miracle_unity_bridge')
    params_file = os.path.join(pkg_share, 'config', 'unity_params.yaml')

    tcp_ip = LaunchConfiguration('tcp_ip')
    tcp_port = LaunchConfiguration('tcp_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'tcp_ip',
            default_value='0.0.0.0',
            description='IP address for Unity TCP endpoint',
        ),
        DeclareLaunchArgument(
            'tcp_port',
            default_value='10000',
            description='TCP port for Unity connection',
        ),

        LogInfo(msg='--- MIRACLE Unity Bridge ---'),
        LogInfo(msg=['  TCP Endpoint: ', tcp_ip, ':', tcp_port]),

        # ros_tcp_endpoint node (from Unity-Technologies/ROS-TCP-Endpoint)
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='unity_tcp_endpoint',
            namespace='miracle/unity',
            parameters=[{
                'ROS_IP': tcp_ip,
                'ROS_TCP_PORT': tcp_port,
            }],
            output='screen',
        ),

        # MIRACLE endpoint configuration node
        Node(
            package='miracle_unity_bridge',
            executable='unity_endpoint',
            name='unity_endpoint_config',
            namespace='miracle/unity',
            parameters=[params_file],
            output='screen',
        ),
    ])
