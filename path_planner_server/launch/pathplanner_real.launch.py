import launch 
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

   
    rviz_file = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'pathplanning_real.rviz')
    
    controller_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller_real.yaml')
    planner_server_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server_real.yaml')
    behaviour_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery_real.yaml')
    bt_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_real.yaml')

    container = LaunchDescription(
        [
            Node(
            
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[planner_server_file]
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[controller_file],
            ),
            Node(
            
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[bt_file]),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                parameters=[behaviour_file],
                output='screen'),
            
            
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_planner',
                output='screen',
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names':['planner_server', 'controller_server', 'bt_navigator', 'behavior_server']}]
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