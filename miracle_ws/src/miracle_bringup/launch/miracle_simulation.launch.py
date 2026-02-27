"""
MIRACLE Simulation Launch - Simulation-only deployment.

Launches CNC machines (parameterized count) in simulation mode, plus the
digital twin and AI layers. Sets use_sim_time=true globally.

Arguments:
    machine_count - Number of CNC machines to launch (default: 3)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node as LaunchNode, PushRosNamespace, SetParameter


def launch_setup(context, *args, **kwargs):
    """Build the simulation launch dynamically."""
    machine_count = int(LaunchConfiguration('machine_count').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    bringup_share = get_package_share_directory('miracle_bringup')
    params_file = os.path.join(bringup_share, 'config', 'miracle_params.yaml')

    actions = []

    actions.append(LogInfo(msg='--- MIRACLE Simulation Mode ---'))
    actions.append(LogInfo(msg=f'    Machines: {machine_count}'))
    actions.append(LogInfo(msg=f'    use_sim_time: {use_sim_time}'))

    # Global sim time
    actions.append(SetParameter(name='use_sim_time', value=use_sim_time))

    # ---------------------------------------------------------------
    # CNC Machine nodes (L1)
    # ---------------------------------------------------------------
    for i in range(1, machine_count + 1):
        machine_id = f'cnc{i}'
        machine_params = os.path.join(bringup_share, 'config', f'{machine_id}.yaml')

        actions.append(
            GroupAction(
                actions=[
                    PushRosNamespace('miracle'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(bringup_share, 'launch',
                                         'cnc_machine.launch.py')
                        ),
                        launch_arguments={
                            'machine_id': machine_id,
                            'params_file': machine_params
                                if os.path.exists(machine_params)
                                else params_file,
                            'simulation_mode': 'true',
                        }.items(),
                    ),
                ]
            )
        )

    # ---------------------------------------------------------------
    # SCADA Layer (L2)
    # ---------------------------------------------------------------
    actions.append(
        GroupAction(
            actions=[
                PushRosNamespace('miracle/scada'),
                LifecycleNode(
                    package='miracle_scada',
                    executable='discovery_server',
                    name='discovery_server',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_scada',
                    executable='traffic_manager',
                    name='traffic_manager',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_scada',
                    executable='alarm_manager',
                    name='alarm_manager',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_scada',
                    executable='historian',
                    name='historian',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_scada',
                    executable='hmi_bridge',
                    name='hmi_bridge',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
            ]
        )
    )

    # ---------------------------------------------------------------
    # MES Layer (L3)
    # ---------------------------------------------------------------
    actions.append(
        GroupAction(
            actions=[
                PushRosNamespace('miracle/mes'),
                LifecycleNode(
                    package='miracle_mes',
                    executable='job_scheduler',
                    name='job_scheduler',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_mes',
                    executable='fleet_manager',
                    name='fleet_manager',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_mes',
                    executable='digital_thread',
                    name='digital_thread',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_mes',
                    executable='resource_manager',
                    name='resource_manager',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_mes',
                    executable='oee_calculator',
                    name='oee_calculator',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
            ]
        )
    )

    # ---------------------------------------------------------------
    # Digital Twin (L3)
    # ---------------------------------------------------------------
    actions.append(
        GroupAction(
            actions=[
                PushRosNamespace('miracle/twin'),
                LifecycleNode(
                    package='miracle_twin',
                    executable='sync_engine',
                    name='sync_engine',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_twin',
                    executable='gazebo_bridge',
                    name='gazebo_bridge',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_twin',
                    executable='prediction_runner',
                    name='prediction_runner',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_twin',
                    executable='scenario_manager',
                    name='scenario_manager',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
            ]
        )
    )

    # ---------------------------------------------------------------
    # AI / ML Layer (L3)
    # ---------------------------------------------------------------
    actions.append(
        GroupAction(
            actions=[
                PushRosNamespace('miracle/ai'),
                LifecycleNode(
                    package='miracle_ai',
                    executable='anomaly_detector',
                    name='anomaly_detector',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_ai',
                    executable='phm_predictor',
                    name='phm_predictor',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_ai',
                    executable='tool_wear_estimator',
                    name='tool_wear_estimator',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_ai',
                    executable='chatter_detector',
                    name='chatter_detector',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                LifecycleNode(
                    package='miracle_ai',
                    executable='model_manager',
                    name='model_manager',
                    namespace='',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    output='screen',
                ),
            ]
        )
    )

    # ---------------------------------------------------------------
    # Security & Resiliency Layer (L4)
    # ---------------------------------------------------------------
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch',
                             'security_layer.launch.py')
            ),
            launch_arguments={'params_file': params_file}.items(),
        )
    )

    # ---------------------------------------------------------------
    # Cognitive Layer (L5)
    # ---------------------------------------------------------------
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch',
                             'cognitive_layer.launch.py')
            ),
            launch_arguments={'params_file': params_file}.items(),
        )
    )

    # ---------------------------------------------------------------
    # Unity Digital Twin Bridge
    # ---------------------------------------------------------------
    unity_bridge_share = get_package_share_directory('miracle_unity_bridge')
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(unity_bridge_share, 'launch',
                             'unity_bridge.launch.py')
            ),
        )
    )

    # ---------------------------------------------------------------
    # Auto-start: configure + activate all lifecycle nodes
    # ---------------------------------------------------------------
    actions.append(
        LaunchNode(
            package='miracle_bringup',
            executable='lifecycle_autostart',
            name='lifecycle_autostart',
            namespace='miracle',
            parameters=[{
                'startup_delay': 15.0,
                'per_node_timeout': 10.0,
                'max_retries': 2,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        )
    )

    return actions


def generate_launch_description():
    """Generate the simulation launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'machine_count',
            default_value='3',
            description='Number of CNC machines to simulate',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulated clock (requires /clock publisher)',
        ),

        OpaqueFunction(function=launch_setup),
    ])
