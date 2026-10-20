"""Discovery transport with an optional shared PIPER scan controller."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument('command_topic', default_value='/joint_ctrl_single'),
        DeclareLaunchArgument('auto_scan', default_value='false'),
        Node(package='phyai_discovery_ros', executable='scan', output='screen',
             namespace=LaunchConfiguration('namespace'),
             condition=IfCondition(LaunchConfiguration('auto_scan')),
             parameters=[{'config_path': LaunchConfiguration('config_path'),
                          'command_topic': LaunchConfiguration('command_topic'),
                          'joint_states_topic': LaunchConfiguration('joint_states_topic')}]),
        Node(package='phyai_discovery_ros', executable='discovery', output='screen',
             namespace=LaunchConfiguration('namespace'),
             parameters=[{'config_path': LaunchConfiguration('config_path'),
                          'recording_path': LaunchConfiguration('recording_path')}],
             remappings=[('joint_states', LaunchConfiguration('joint_states_topic'))]),
    ])
