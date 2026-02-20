from setuptools import setup, find_packages

package_name = 'miracle_mes'

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
    description='MES/Orchestration nodes for MIRACLE system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'job_scheduler = miracle_mes.job_scheduler:main',
            'fleet_manager = miracle_mes.fleet_manager:main',
            'digital_thread = miracle_mes.digital_thread:main',
            'resource_manager = miracle_mes.resource_manager:main',
            'oee_calculator = miracle_mes.oee_calculator:main',
        ],
    },
)
