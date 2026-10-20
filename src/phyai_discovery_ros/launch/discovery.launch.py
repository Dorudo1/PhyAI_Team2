"""Discovery transport only; the consuming app owns all robot motion."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=PathJoinSubstitution([
            FindPackageShare('phyai_discovery_ros'), 'config', 'discovery.json'])),
        DeclareLaunchArgument('recording_path', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        Node(package='phyai_discovery_ros', executable='discovery', output='screen',
             namespace=LaunchConfiguration('namespace'),
             parameters=[{'config_path': LaunchConfiguration('config_path'),
                          'recording_path': LaunchConfiguration('recording_path')}],
             remappings=[('joint_states', LaunchConfiguration('joint_states_topic'))]),
    ])
