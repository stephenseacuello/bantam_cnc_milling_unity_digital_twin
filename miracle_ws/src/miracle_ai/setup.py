from setuptools import setup, find_packages

package_name = 'miracle_ai'

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
    description='AI/ML nodes for MIRACLE system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anomaly_detector = miracle_ai.anomaly_detector:main',
            'phm_predictor = miracle_ai.phm_predictor:main',
            'tool_wear_estimator = miracle_ai.tool_wear_estimator:main',
            'chatter_detector = miracle_ai.chatter_detector:main',
            'model_manager = miracle_ai.model_manager:main',
        ],
    },
)
