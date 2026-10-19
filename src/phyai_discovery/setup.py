from setuptools import find_packages, setup

setup(
    name='phyai_discovery',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/phyai_discovery']),
        ('share/phyai_discovery', ['package.xml', 'README.md']),
    ],
    entry_points={'console_scripts': ['replay_scan = phyai_discovery.recording:main']},
    install_requires=['setuptools', 'numpy'],
    description='Reusable RGB-D discovery and workspace inventory for PhyAI pipelines',
    license='Apache-2.0',
)
