from setuptools import setup, find_packages

package_name = 'miracle_twin'

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
    description='Digital twin nodes for MIRACLE system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sync_engine = miracle_twin.sync_engine:main',
            'gazebo_bridge = miracle_twin.gazebo_bridge:main',
            'prediction_runner = miracle_twin.prediction_runner:main',
            'scenario_manager = miracle_twin.scenario_manager:main',
        ],
    },
)
