import launch 
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    cartographer_config_directory = os.path.join(get_package_share_directory("cartographer_slam"), "config")
    cartographer_config_file = "cartographer.lua"

    rviz_file = os.path.join(get_package_share_directory("cartographer_slam"), "rviz", "rviz_config.rviz")

    container = LaunchDescription([
         Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', cartographer_config_directory,
                       '-configuration_basename', cartographer_config_file]
        ),

        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
        Node(
            executable="rviz2",
            name="rviz2",
            package="rviz2",
            output="screen",
            arguments=["-d", rviz_file]
        )
    ])
    return container