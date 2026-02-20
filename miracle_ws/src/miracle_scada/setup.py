from setuptools import setup, find_packages

package_name = 'miracle_scada'

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
    description='SCADA/Supervisory nodes for MIRACLE system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'discovery_server = miracle_scada.discovery_server:main',
            'traffic_manager = miracle_scada.traffic_manager:main',
            'alarm_manager = miracle_scada.alarm_manager:main',
            'historian = miracle_scada.historian:main',
            'hmi_bridge = miracle_scada.hmi_bridge:main',
        ],
    },
)
