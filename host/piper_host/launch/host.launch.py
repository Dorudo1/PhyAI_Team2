"""README 인터페이스 변환 + robot_state_publisher.

  ros2 launch piper_host host.launch.py

드라이버 · 카메라는 띄우지 않는다 (piper-sensors / piper-record / piper-teleop 이 켜고, 이 launch 도 같이 켠다).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    with open(os.path.join(get_package_share_directory('piper_description'), 'urdf', 'piper_description.urdf')) as f:
        urdf = f.read()
    config = os.path.join(get_package_share_directory('piper_host'), 'config', 'interface.yaml')
    return LaunchDescription([
        Node(package='piper_host', executable='readme_bridge', name='readme_bridge', parameters=[config], output='screen'),
        # /joint_states → base_link … link8 tf, /robot_description (transient local)
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='piper_host_rsp',
             parameters=[{'robot_description': urdf}], output='screen'),
    ])
