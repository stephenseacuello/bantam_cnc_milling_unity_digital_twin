from setuptools import setup, find_packages

package_name = 'miracle_cognitive'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/ontologies', ['ontologies/manufacturing.owl']),
        ('share/' + package_name + '/behavior_trees', [
            'behavior_trees/autonomous_job.xml',
            'behavior_trees/anomaly_response.xml',
            'behavior_trees/recovery_sequence.xml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MIRACLE Team',
    maintainer_email='miracle@example.com',
    description='Cognitive layer nodes for MIRACLE system (Level 5 autonomy)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'knowledge_graph = miracle_cognitive.knowledge.knowledge_graph:main',
            'ontology_manager = miracle_cognitive.knowledge.ontology_manager:main',
            'reasoning_engine = miracle_cognitive.knowledge.reasoning_engine:main',
            'causal_inference = miracle_cognitive.knowledge.causal_inference:main',
            'goal_manager = miracle_cognitive.planning.goal_manager:main',
            'behavior_tree_executor = miracle_cognitive.planning.behavior_tree_executor:main',
            'htn_planner = miracle_cognitive.planning.htn_planner:main',
            'goap_planner = miracle_cognitive.planning.goap_planner:main',
            'agent_registry = miracle_cognitive.multi_agent.agent_registry:main',
            'task_allocator = miracle_cognitive.multi_agent.task_allocator:main',
            'auction_manager = miracle_cognitive.multi_agent.auction_manager:main',
            'coalition_former = miracle_cognitive.multi_agent.coalition_former:main',
            'consensus_protocol = miracle_cognitive.multi_agent.consensus_protocol:main',
            'rl_optimizer = miracle_cognitive.learning.rl_optimizer:main',
            'federated_coordinator = miracle_cognitive.learning.federated_coordinator:main',
            'federated_client = miracle_cognitive.learning.federated_client:main',
            'self_optimizer = miracle_cognitive.self_x.self_optimizer:main',
            'self_healer = miracle_cognitive.self_x.self_healer:main',
            'self_configurer = miracle_cognitive.self_x.self_configurer:main',
            'self_protector = miracle_cognitive.self_x.self_protector:main',
            'nlp_interface = miracle_cognitive.interface.nlp_interface:main',
            'explanation_generator = miracle_cognitive.interface.explanation_generator:main',
            'human_escalation = miracle_cognitive.interface.human_escalation:main',
        ],
    },
)
