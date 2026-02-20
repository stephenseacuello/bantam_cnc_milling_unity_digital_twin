"""
MIRACLE Cognitive Layer Launch (L5) - Cognitive autonomy nodes.

Launches all L5 cognitive subsystem nodes under /miracle/cognitive/ namespace.

Subsystems:
    Knowledge Management  - knowledge_graph, ontology_manager, reasoning_engine,
                            causal_inference
    Planning              - goal_manager, behavior_tree_executor, htn_planner,
                            goap_planner
    Multi-Agent           - agent_registry, task_allocator, auction_manager,
                            coalition_former, consensus_protocol
    Learning              - rl_optimizer, federated_coordinator
    Self-X Properties     - self_optimizer, self_healer, self_configurer,
                            self_protector
    Human Interface       - nlp_interface, explanation_generator, human_escalation

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
    """Generate the cognitive layer launch description."""
    bringup_share = get_package_share_directory('miracle_bringup')
    default_params = os.path.join(bringup_share, 'config', 'miracle_params.yaml')

    params_file = LaunchConfiguration('params_file')

    # Helper to build a LifecycleNode for a cognitive subsystem node
    def _cognitive_node(executable, name=None):
        return LifecycleNode(
            package='miracle_cognitive',
            executable=executable,
            name=name or executable,
            namespace='',
            parameters=[params_file],
            output='screen',
            respawn=True,
            respawn_delay=5.0,
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the parameter YAML file',
        ),

        # -----------------------------------------------------------
        # All cognitive nodes under /miracle/cognitive/
        # -----------------------------------------------------------
        GroupAction(
            actions=[
                PushRosNamespace('miracle/cognitive'),

                # Knowledge Management
                _cognitive_node('knowledge_graph'),
                _cognitive_node('ontology_manager'),
                _cognitive_node('reasoning_engine'),
                _cognitive_node('causal_inference'),

                # Planning
                _cognitive_node('goal_manager'),
                _cognitive_node('behavior_tree_executor'),
                _cognitive_node('htn_planner'),
                _cognitive_node('goap_planner'),

                # Multi-Agent Coordination
                _cognitive_node('agent_registry'),
                _cognitive_node('task_allocator'),
                _cognitive_node('auction_manager'),
                _cognitive_node('coalition_former'),
                _cognitive_node('consensus_protocol'),

                # Learning & Adaptation
                _cognitive_node('rl_optimizer'),
                _cognitive_node('federated_coordinator'),

                # Self-X Properties
                _cognitive_node('self_optimizer'),
                _cognitive_node('self_healer'),
                _cognitive_node('self_configurer'),
                _cognitive_node('self_protector'),

                # Human Interface
                _cognitive_node('nlp_interface'),
                _cognitive_node('explanation_generator'),
                _cognitive_node('human_escalation'),
            ]
        ),
    ])
