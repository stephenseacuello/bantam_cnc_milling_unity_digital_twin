"""
MIRACLE Full System Launch - Brings up all layers L1 through L5.

Layers:
    L1 - CNC Machine Control (miracle_cnc)
    L2 - SCADA / Supervisory (miracle_scada, miracle_bridges)
    L3 - MES / Orchestration (miracle_mes, miracle_twin, miracle_ai)
    L4 - Security & Resiliency (miracle_security, miracle_resiliency)
    L5 - Cognitive Autonomy (miracle_cognitive)

Arguments:
    machine_count      - Number of CNC machines to launch (default: 3)
    simulation_mode    - Use simulated hardware (default: true)
    security_enabled   - Enable L4 security/resiliency layer (default: true)
    cognitive_enabled  - Enable L5 cognitive layer (default: true)
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
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import PushRosNamespace


def launch_setup(context, *args, **kwargs):
    """Build the launch description dynamically based on arguments."""
    machine_count = int(LaunchConfiguration('machine_count').perform(context))
    simulation_mode = LaunchConfiguration('simulation_mode').perform(context)
    security_enabled = LaunchConfiguration('security_enabled').perform(context)
    cognitive_enabled = LaunchConfiguration('cognitive_enabled').perform(context)

    bringup_share = get_package_share_directory('miracle_bringup')
    params_file = os.path.join(bringup_share, 'config', 'miracle_params.yaml')

    actions = []

    actions.append(LogInfo(msg='========================================'))
    actions.append(LogInfo(msg='  MIRACLE Full System Bringup'))
    actions.append(LogInfo(msg=f'  Machines: {machine_count}'))
    actions.append(LogInfo(msg=f'  Simulation: {simulation_mode}'))
    actions.append(LogInfo(msg=f'  Security: {security_enabled}'))
    actions.append(LogInfo(msg=f'  Cognitive: {cognitive_enabled}'))
    actions.append(LogInfo(msg='========================================'))

    # ---------------------------------------------------------------
    # L1 - CNC Machine Control Layer
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
                            'simulation_mode': simulation_mode,
                        }.items(),
                    ),
                ]
            )
        )

    # ---------------------------------------------------------------
    # L2 - SCADA / Supervisory Layer
    # ---------------------------------------------------------------
    if simulation_mode.lower() == 'true':
        actions.append(
            GroupAction(
                actions=[
                    PushRosNamespace('miracle'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(bringup_share, 'launch',
                                         'miracle_simulation.launch.py')
                        ),
                        launch_arguments={
                            'machine_count': str(machine_count),
                        }.items(),
                    ),
                ]
            )
        )
    else:
        actions.append(
            GroupAction(
                actions=[
                    PushRosNamespace('miracle'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(bringup_share, 'launch',
                                         'miracle_physical.launch.py')
                        ),
                        launch_arguments={
                            'machine_count': str(machine_count),
                        }.items(),
                    ),
                ]
            )
        )

    # ---------------------------------------------------------------
    # L4 - Security & Resiliency Layer
    # ---------------------------------------------------------------
    if security_enabled.lower() == 'true':
        actions.append(
            GroupAction(
                actions=[
                    PushRosNamespace('miracle'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(bringup_share, 'launch',
                                         'security_layer.launch.py')
                        ),
                        launch_arguments={
                            'params_file': params_file,
                        }.items(),
                    ),
                ]
            )
        )

    # ---------------------------------------------------------------
    # L5 - Cognitive Autonomy Layer
    # ---------------------------------------------------------------
    if cognitive_enabled.lower() == 'true':
        actions.append(
            GroupAction(
                actions=[
                    PushRosNamespace('miracle'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(bringup_share, 'launch',
                                         'cognitive_layer.launch.py')
                        ),
                        launch_arguments={
                            'params_file': params_file,
                        }.items(),
                    ),
                ]
            )
        )

    return actions


def generate_launch_description():
    """Generate the top-level launch description."""
    return LaunchDescription([
        # -----------------------------------------------------------
        # Declare launch arguments
        # -----------------------------------------------------------
        DeclareLaunchArgument(
            'machine_count',
            default_value='3',
            description='Number of CNC machines to bring up',
        ),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='true',
            description='Run in simulation mode (true) or physical mode (false)',
        ),
        DeclareLaunchArgument(
            'security_enabled',
            default_value='true',
            description='Enable the L4 security and resiliency layer',
        ),
        DeclareLaunchArgument(
            'cognitive_enabled',
            default_value='true',
            description='Enable the L5 cognitive autonomy layer',
        ),

        # -----------------------------------------------------------
        # Build the system using OpaqueFunction so we can iterate
        # over machine_count at launch time.
        # -----------------------------------------------------------
        OpaqueFunction(function=launch_setup),
    ])
