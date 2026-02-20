"""Launch file for the MIRACLE CNC factory Gazebo simulation.

Starts Gazebo with the cnc_factory.sdf world, spawns a configurable
number of CNC machine models at evenly spaced positions, and bridges
clock, joint-state, and sensor data between Gazebo and ROS 2 via
ros_gz_bridge.
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_machines_and_bridges(context, *args, **kwargs) -> List:
    """OpaqueFunction callback that spawns CNC machines and configures bridges.

    Evaluated at launch-time so that the ``machine_count`` argument can be
    resolved to a concrete integer before generating the spawn / bridge
    nodes.
    """
    machine_count = int(LaunchConfiguration('machine_count').perform(context))
    pkg_share = get_package_share_directory('miracle_gazebo')
    model_path = os.path.join(pkg_share, 'models', 'cnc_machine', 'model.sdf')

    actions: List = []

    # X-spacing between machines (metres)
    spacing = 3.0

    for i in range(machine_count):
        machine_name = f'cnc_machine_{i + 1}'
        x_pos = i * spacing

        # ---- Spawn the CNC machine model into Gazebo ----
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{machine_name}',
            output='screen',
            arguments=[
                '-file', model_path,
                '-name', machine_name,
                '-x', str(x_pos),
                '-y', '0',
                '-z', '0',
            ],
        )
        actions.append(spawn_node)

        # ---- ros_gz_bridge for joint states of this machine ----
        joint_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{machine_name}_joint_bridge',
            output='screen',
            arguments=[
                f'/world/cnc_factory/model/{machine_name}/joint_state'
                f'@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
        )
        actions.append(joint_bridge)

    # ---- ros_gz_bridge for sensor data (camera & IMU) ----
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        output='screen',
        arguments=[
            '/sensor_array/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/sensor_array/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
    )
    actions.append(sensor_bridge)

    return actions


def generate_launch_description() -> LaunchDescription:
    """Build and return the full launch description."""

    pkg_share = get_package_share_directory('miracle_gazebo')

    # ---- Declare launch arguments ----
    machine_count_arg = DeclareLaunchArgument(
        'machine_count',
        default_value='3',
        description='Number of CNC machines to spawn in the factory world',
    )

    gz_verbose_arg = DeclareLaunchArgument(
        'gz_verbose',
        default_value='1',
        description='Gazebo verbose logging level (0-4)',
    )

    # ---- World file path ----
    world_file = os.path.join(pkg_share, 'worlds', 'cnc_factory.sdf')

    # ---- Set GZ_SIM_RESOURCE_PATH so Gazebo can find our models ----
    models_dir = os.path.join(pkg_share, 'models')
    gz_resource_env = {
        'GZ_SIM_RESOURCE_PATH': models_dir,
    }

    # ---- Start Gazebo ----
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v {LaunchConfiguration("gz_verbose").describe()} {world_file}',
        }.items(),
    )

    # ---- Clock bridge (Gazebo -> ROS 2) ----
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    # ---- OpaqueFunction for dynamic machine spawning & bridges ----
    spawn_machines = OpaqueFunction(function=_spawn_machines_and_bridges)

    return LaunchDescription([
        machine_count_arg,
        gz_verbose_arg,
        gz_sim,
        clock_bridge,
        spawn_machines,
    ])
