from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('http_port', default_value='23517',
                              description='Browser view port; pick another if a teammate on the same dev PC uses it'),
        DeclareLaunchArgument('scene_config', default_value=
                              get_package_share_directory('piper_mujoco_sim') + '/config/lab_scene.json',
                              description='Measured lab layout and camera poses'),
        DeclareLaunchArgument('model_path', default_value='',
                              description='Optional custom MJCF instead of the lab scene'),
        DeclareLaunchArgument('scene_editing', default_value='false',
                              description='Open the scene editor at / instead of the read-only viewer; /edit always provides the editor'),
        DeclareLaunchArgument('publish_cameras', default_value='false',
                              description='Publish live RGB-D, CameraInfo and camera TF for perception apps'),
        Node(
            package='piper_mujoco_sim',
            executable='sim',
            parameters=[{'http_port': LaunchConfiguration('http_port'),
                         'scene_config': LaunchConfiguration('scene_config'),
                         'scene_editing': ParameterValue(LaunchConfiguration('scene_editing'), value_type=bool),
                         'model_path': LaunchConfiguration('model_path'),
                         'publish_cameras': ParameterValue(LaunchConfiguration('publish_cameras'), value_type=bool)}],
            output='screen',
        ),
    ])
