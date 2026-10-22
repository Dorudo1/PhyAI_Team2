from glob import glob

from setuptools import setup

package_name = 'piper_host'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='physical_ai_team2',
    maintainer_email='pjh5148@keti.re.kr',
    description='Piper 호스트 → PhyAI_Team2 README 인터페이스 변환',
    license='Apache-2.0',
    entry_points={'console_scripts': ['readme_bridge = piper_host.readme_bridge:main']},
)
