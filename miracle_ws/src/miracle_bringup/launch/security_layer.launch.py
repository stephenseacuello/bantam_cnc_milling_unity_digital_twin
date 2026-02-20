"""
MIRACLE Security & Resiliency Layer Launch (L4).

Launches all L4 nodes across two namespaces:
    /miracle/security/    - Security nodes (intrusion detection, attestation, etc.)
    /miracle/resiliency/  - Fault tolerance nodes (supervision, failover, etc.)

Security nodes (miracle_security):
    intrusion_detection, attestation_verifier, threat_response,
    access_enforcer, audit_logger, sros2_manager

Resiliency nodes (miracle_resiliency):
    supervisor_root, heartbeat_aggregator, failover_coordinator,
    checkpoint_manager, recovery_orchestrator, chaos_injector

Arguments:
    params_file - Path to the parameter YAML file
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, PushRosNamespace


def generate_launch_description():
    """Generate the security and resiliency layer launch description."""
    bringup_share = get_package_share_directory('miracle_bringup')
    default_params = os.path.join(bringup_share, 'config', 'miracle_params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the parameter YAML file',
        ),

        # -----------------------------------------------------------
        # Security nodes under /miracle/security/
        # -----------------------------------------------------------
        GroupAction(
            actions=[
                PushRosNamespace('miracle/security'),

                LifecycleNode(
                    package='miracle_security',
                    executable='intrusion_detection',
                    name='intrusion_detection',
                    namespace='',
                    parameters=[params_file],
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0,
                ),
                LifecycleNode(
                    package='miracle_security',
                    executable='attestation_verifier',
                    name='attestation_verifier',
                    namespace='',
                    parameters=[params_file],
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0,
                ),
                LifecycleNode(
                    package='miracle_security',
                    executable='threat_response',
                    name='threat_response',
                    namespace='',
                    parameters=[params_file],
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0,
                ),
                LifecycleNode(
                    package='miracle_security',
                    executable='access_enforcer',
                    name='access_enforcer',
                    namespace='',
                    parameters=[params_file],
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0,
                ),
                LifecycleNode(
                    package='miracle_security',
                    executable='audit_logger',
                    name='audit_logger',
                    namespace='',
                    parameters=[params_file],
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0,
                ),
                LifecycleNode(
                    package='miracle_security',
                    executable='sros2_manager',
                    name='sros2_manager',
                    namespace='',
                    parameters=[params_file],
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0,
                ),
            ]
        ),

        # -----------------------------------------------------------
        # Resiliency nodes under /miracle/resiliency/
        # -----------------------------------------------------------
        GroupAction(
            actions=[
                PushRosNamespace('miracle/resiliency'),

                LifecycleNode(
                    package='miracle_resiliency',
                    executable='supervisor_root',
                    name='supervisor_root',
                    namespace='',
                    parameters=[params_file],
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                ),
                LifecycleNode(
                    package='miracle_resiliency',
                    executable='heartbeat_aggregator',
                    name='heartbeat_aggregator',
                    namespace='',
                    parameters=[params_file],
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                ),
                LifecycleNode(
                    package='miracle_resiliency',
                    executable='failover_coordinator',
                    name='failover_coordinator',
                    namespace='',
                    parameters=[params_file],
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                ),
                LifecycleNode(
                    package='miracle_resiliency',
                    executable='checkpoint_manager',
                    name='checkpoint_manager',
                    namespace='',
                    parameters=[params_file],
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                ),
                LifecycleNode(
                    package='miracle_resiliency',
                    executable='recovery_orchestrator',
                    name='recovery_orchestrator',
                    namespace='',
                    parameters=[params_file],
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                ),
                LifecycleNode(
                    package='miracle_resiliency',
                    executable='chaos_injector',
                    name='chaos_injector',
                    namespace='',
                    parameters=[params_file],
                    output='screen',
                    respawn=True,
                    respawn_delay=5.0,
                ),
            ]
        ),
    ])
