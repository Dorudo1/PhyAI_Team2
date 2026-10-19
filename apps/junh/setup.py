from glob import glob
from setuptools import find_packages, setup

setup(
    name='junh_master',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={'junh_master.viewer': ['workspace.html']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/junh_master']),
        ('share/junh_master', ['package.xml']),
        ('share/junh_master/launch', glob('launch/*.launch.py')),
        ('share/junh_master/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'numpy', 'phyai_discovery'],
    zip_safe=True,
    entry_points={'console_scripts': ['master = junh_master.ros.handoff_node:main',
                                     'search_demo = junh_master.legacy.demo:main',
                                     'discovery = junh_master.ros.discovery_node:main',
                                     'replay_scan = phyai_discovery.recording:main',
                                     'sim_scan = junh_master.ros.sim_scan:main',
                                     'workspace_viewer = junh_master.ros.viewer_node:main']},
)
