import launch 
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    map_server_config = os.path.join(get_package_share_directory('localization_server'), 'config')
    map_file = PythonExpression(["'", map_server_config, "/", LaunchConfiguration("map_file"), "'"])
    
    rviz_file = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'rviz_config.rviz')
    amcl_yaml = os.path.join(map_server_config, "amcl_real.yaml")


   
    
    container = LaunchDescription(
        [
            DeclareLaunchArgument("map_file"),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_broadcaster',
                output='screen',
                parameters=[{'use_sim_time': False}],
                arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'robot_odom']
            ),
           
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server_real',
                output='screen',
                parameters=[{'use_sim_time': False}, 
                                {'yaml_filename': map_file}]
            ),
            Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_yaml]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization_real',
                output='screen',
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names':['map_server_real', 'amcl']}]
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[{'use_sim_time': False}],
                arguments=['-d', rviz_file]
            )
        ]
    
    )
    return container