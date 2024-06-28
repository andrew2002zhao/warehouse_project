import launch
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.substitutions import PythonExpression


def generate_launch_description():

    cartographer_slam_config = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    file_name = PythonExpression('str(', LaunchConfiguration('map_file'), ')')

    
    container = LaunchDescription([
        DeclareLaunchArgument('map_file'),
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', file_name],
            shell=True,
            working_directory=cartographer_slam_config
        )      
    ])

    return container