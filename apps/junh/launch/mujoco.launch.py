"""Live rendered-camera discovery with the development-only MuJoCo simulator."""
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('http_port',default_value='23517'),
        DeclareLaunchArgument('auto_search',default_value='true'),
        DeclareLaunchArgument('scatter_on_start',default_value='true'),
        DeclareLaunchArgument('scatter_seed',default_value='-1'),
        DeclareLaunchArgument('scatter_table',default_value='all'),
        DeclareLaunchArgument('workspace_port',default_value=PythonExpression([LaunchConfiguration('http_port'),' + 1'])),
        Node(package='junh_master',executable='sim_scan',output='screen',
             condition=IfCondition(LaunchConfiguration('auto_search'))),
        Node(package='piper_mujoco_sim',executable='sim',output='screen',parameters=[{
            'publish_cameras':True,'camera_on_demand':True,'camera_rate':3.,
            'camera_width':320,'camera_height':240,'view_width':960,'view_height':720,
            'http_port':ParameterValue(LaunchConfiguration('http_port'),value_type=int),
            'scatter_on_start':ParameterValue(LaunchConfiguration('scatter_on_start'),value_type=bool),
            'scatter_seed':ParameterValue(LaunchConfiguration('scatter_seed'),value_type=int),
            'scatter_table':ParameterValue(LaunchConfiguration('scatter_table'),value_type=str),
            'workspace_port':ParameterValue(LaunchConfiguration('workspace_port'),value_type=int)}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('junh_master'),'launch','master.launch.py'])),launch_arguments={
            'config_path':PathJoinSubstitution([FindPackageShare('junh_master'),'config','mujoco.json']),
            'workspace_port':LaunchConfiguration('workspace_port'),
            'simulator_port':LaunchConfiguration('http_port')}.items()),
    ])
