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
        DeclareLaunchArgument('scatter_on_start', default_value='false',
                              description='Scatter blocks before the first frame; reset restores the aligned scene'),
        DeclareLaunchArgument('scatter_seed', default_value='-1',
                              description='Nonnegative seed for reproducible scattering; -1 chooses a random layout'),
        DeclareLaunchArgument('scatter_table', default_value='all',
                              description='Tables used for scattering: all, 0_0, 0_1, 1_0, or 1_1'),
        Node(
            package='piper_mujoco_sim',
            executable='sim',
            parameters=[{'http_port': LaunchConfiguration('http_port'),
                         'scene_config': LaunchConfiguration('scene_config'),
                         'scene_editing': ParameterValue(LaunchConfiguration('scene_editing'), value_type=bool),
                         'model_path': LaunchConfiguration('model_path'),
                         'scatter_on_start': ParameterValue(LaunchConfiguration('scatter_on_start'), value_type=bool),
                         'scatter_seed': ParameterValue(LaunchConfiguration('scatter_seed'), value_type=int),
                         'scatter_table': ParameterValue(LaunchConfiguration('scatter_table'), value_type=str),
                         'publish_cameras': ParameterValue(LaunchConfiguration('publish_cameras'), value_type=bool)}],
            output='screen',
        ),
    ])
