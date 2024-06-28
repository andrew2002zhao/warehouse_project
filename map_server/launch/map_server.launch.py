import launch 
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    map_server_config = os.path.join(get_package_share_directory('map_server'), 'config')
    map_file = os.path.join(map_server_config, 'warehouse_map_sim.yaml')
    rviz_file = os.path.join(get_package_share_directory('map_server'), 'rviz',' rviz_config.rviz')
    container = LaunchDescription(
    [
        Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            exectuable='lifecycle_manage',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time', True},
                        {'autostart': True},
                        {'node_names':['map_server']}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
        )
    ]
    
    )
    return container