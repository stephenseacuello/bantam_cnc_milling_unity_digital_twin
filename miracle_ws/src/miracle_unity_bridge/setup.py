from setuptools import find_packages, setup

package_name = 'miracle_unity_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/unity_bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/unity_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MIRACLE Team',
    maintainer_email='miracle@example.com',
    description='ROS-TCP-Endpoint bridge for Unity Digital Twin',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unity_endpoint = miracle_unity_bridge.unity_endpoint:main',
        ],
    },
)
