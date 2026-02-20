import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'miracle_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # World files
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.sdf'))),
        # CNC machine model
        (os.path.join('share', package_name, 'models', 'cnc_machine'),
            glob(os.path.join('models', 'cnc_machine', '*'))),
        # Cobot model
        (os.path.join('share', package_name, 'models', 'cobot'),
            glob(os.path.join('models', 'cobot', '*'))),
        # Sensor array model
        (os.path.join('share', package_name, 'models', 'sensor_array'),
            glob(os.path.join('models', 'sensor_array', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miracle-team',
    maintainer_email='miracle@example.com',
    description='Gazebo simulation assets for the MIRACLE CNC milling digital twin',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
