from setuptools import setup, find_packages

package_name = 'miracle_bridges'

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
    description='Protocol bridge nodes for MIRACLE system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opc_ua_bridge = miracle_bridges.opc_ua_bridge:main',
            'sparkplug_bridge = miracle_bridges.sparkplug_bridge:main',
            'mtconnect_agent = miracle_bridges.mtconnect_agent:main',
            'modbus_bridge = miracle_bridges.modbus_bridge:main',
            'kafka_bridge = miracle_bridges.kafka_bridge:main',
        ],
    },
)
