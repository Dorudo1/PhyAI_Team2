from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_path',default_value=PathJoinSubstitution([
            FindPackageShare('junh_master'),'config','discovery.json'])),
        DeclareLaunchArgument('recording_path',default_value=''),
        DeclareLaunchArgument('workspace_port',default_value='0'),
        DeclareLaunchArgument('simulator_port',default_value='0'),
        Node(package='junh_master',executable='discovery',output='screen',parameters=[{
            'config_path':LaunchConfiguration('config_path'),
            'recording_path':LaunchConfiguration('recording_path')}]),
        Node(package='junh_master',executable='master',output='screen'),
        Node(package='junh_master',executable='workspace_viewer',output='screen',parameters=[{
            'http_port':ParameterValue(LaunchConfiguration('workspace_port'),value_type=int),
            'simulator_port':ParameterValue(LaunchConfiguration('simulator_port'),value_type=int)}]),
    ])
