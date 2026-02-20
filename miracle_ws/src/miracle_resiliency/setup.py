from setuptools import setup, find_packages

package_name = 'miracle_resiliency'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MIRACLE Team',
    maintainer_email='miracle@example.com',
    description='Resiliency and fault tolerance nodes for MIRACLE system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'supervisor_root = miracle_resiliency.supervisor_root:main',
            'heartbeat_aggregator = miracle_resiliency.heartbeat_aggregator:main',
            'failover_coordinator = miracle_resiliency.failover_coordinator:main',
            'checkpoint_manager = miracle_resiliency.checkpoint_manager:main',
            'recovery_orchestrator = miracle_resiliency.recovery_orchestrator:main',
            'chaos_injector = miracle_resiliency.chaos_injector:main',
        ],
    },
)
