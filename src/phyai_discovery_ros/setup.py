from glob import glob
from setuptools import find_packages, setup

setup(
    name='phyai_discovery_ros',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/phyai_discovery_ros']),
        ('share/phyai_discovery_ros', ['package.xml', 'README.md']),
        ('share/phyai_discovery_ros/launch', glob('launch/*.launch.py')),
        ('share/phyai_discovery_ros/config', glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': ['discovery = phyai_discovery_ros.node:main',
                                         'scan = phyai_discovery_ros.scan:main']},
    description='Shared RGB-D discovery ROS adapter for calibrated cameras and PIPER feedback',
    license='Apache-2.0',
)
