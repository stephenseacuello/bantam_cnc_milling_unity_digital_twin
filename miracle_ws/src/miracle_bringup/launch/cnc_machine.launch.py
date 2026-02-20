"""
MIRACLE CNC Machine Launch - Per-machine node launcher.

Launches the full set of nodes required for a single CNC machine under
the /miracle/{machine_id}/ namespace.

Nodes launched:
    - state_publisher   : Publishes machine state at high frequency
    - gcode_executor    : Interprets and executes G-code programs
    - sensor_fusion     : Fuses multi-sensor data streams
    - local_watchdog    : Per-machine health monitoring
    - spc_monitor       : Statistical process control
    - rosbag_trigger    : Event-driven recording trigger

Arguments:
    machine_id      - Machine identifier, e.g. "cnc1" (required)
    params_file     - Path to parameter YAML file
    simulation_mode - Whether to use simulated hardware (default: true)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, PushRosNamespace


def launch_setup(context, *args, **kwargs):
    """Build per-machine launch actions."""
    machine_id = LaunchConfiguration('machine_id').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    simulation_mode = LaunchConfiguration('simulation_mode').perform(context)

    common_params = [
        params_file,
        {
            'machine_id': machine_id,
            'simulation_mode': simulation_mode.lower() == 'true',
        },
    ]

    machine_nodes = GroupAction(
        actions=[
            PushRosNamespace(machine_id),

            LifecycleNode(
                package='miracle_cnc',
                executable='state_publisher',
                name='state_publisher',
                namespace='',
                parameters=common_params + [{'publish_rate_hz': 50.0}],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            ),

            LifecycleNode(
                package='miracle_cnc',
                executable='gcode_executor',
                name='gcode_executor',
                namespace='',
                parameters=common_params,
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            ),

            LifecycleNode(
                package='miracle_cnc',
                executable='sensor_fusion',
                name='sensor_fusion',
                namespace='',
                parameters=common_params + [{'publish_rate_hz': 100.0}],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            ),

            LifecycleNode(
                package='miracle_cnc',
                executable='local_watchdog',
                name='local_watchdog',
                namespace='',
                parameters=common_params + [{'heartbeat_interval_sec': 1.0}],
                output='screen',
                respawn=True,
                respawn_delay=1.0,
            ),

            LifecycleNode(
                package='miracle_cnc',
                executable='spc_monitor',
                name='spc_monitor',
                namespace='',
                parameters=common_params + [{'window_size': 30}],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            ),

            LifecycleNode(
                package='miracle_cnc',
                executable='rosbag_trigger',
                name='rosbag_trigger',
                namespace='',
                parameters=common_params + [{
                    'pre_trigger_sec': 10.0,
                    'post_trigger_sec': 30.0,
                }],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )

    return [machine_nodes]


def generate_launch_description():
    """Generate the per-machine launch description."""
    bringup_share = get_package_share_directory('miracle_bringup')
    default_params = os.path.join(bringup_share, 'config', 'miracle_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'machine_id',
            description='Unique identifier for this CNC machine (e.g. cnc1)',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the parameter YAML file',
        ),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='true',
            description='Run machine nodes in simulation mode',
        ),

        OpaqueFunction(function=launch_setup),
    ])
