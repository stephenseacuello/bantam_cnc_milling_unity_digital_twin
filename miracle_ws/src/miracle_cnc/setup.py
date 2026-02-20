from setuptools import setup, find_packages

package_name = 'miracle_cnc'

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
    description='CNC machine control nodes for MIRACLE system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = miracle_cnc.state_publisher:main',
            'gcode_executor = miracle_cnc.gcode_executor:main',
            'sensor_fusion = miracle_cnc.sensor_fusion:main',
            'local_watchdog = miracle_cnc.local_watchdog:main',
            'spc_monitor = miracle_cnc.spc_monitor:main',
            'rosbag_trigger = miracle_cnc.rosbag_trigger:main',
        ],
    },
)
