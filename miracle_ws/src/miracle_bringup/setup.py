import os
from glob import glob

from setuptools import setup, find_packages

package_name = 'miracle_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MIRACLE Team',
    maintainer_email='miracle@example.com',
    description='Launch files and configuration for the MIRACLE CNC Digital Twin system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lifecycle_autostart = miracle_bringup.lifecycle_autostart:main',
        ],
    },
)
