from setuptools import setup, find_packages

package_name = 'miracle_security'

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
    description='Security nodes for MIRACLE system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'intrusion_detection = miracle_security.intrusion_detection:main',
            'attestation_verifier = miracle_security.attestation_verifier:main',
            'threat_response = miracle_security.threat_response:main',
            'access_enforcer = miracle_security.access_enforcer:main',
            'audit_logger = miracle_security.audit_logger:main',
            'sros2_manager = miracle_security.sros2_manager:main',
        ],
    },
)
