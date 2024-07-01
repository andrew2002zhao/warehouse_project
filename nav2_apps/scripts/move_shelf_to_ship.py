from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg._pose_stamped import PoseStamped

import sys
import threading
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchService


def generate_launch_description():
    container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="nav_container",
        output="screen",
        namespace="",
        composable_node_descriptions=[
            ComposableNode(
                name="attach_server",
                plugin="nav2_apps::ApproachService",
                package="nav2_apps"
            )
        ]
    )
    return container

def navigation():
    nav = BasicNavigator()

    # initial pose
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = nav.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.orientation.z = 0.0
    init_pose.pose.orientation.w = 1.0
    nav.setInitialPose(init_pose)

    nav.waitUntilNav2Active()

    # target pose
    loading_pose = PoseStamped()
    loading_pose.header.frame_id = 'map'
    loading_pose.header.stamp = nav.get_clock().now().to_msg()
    loading_pose.pose.position.x = 5.7
    loading_pose.pose.position.y = 0.0
    loading_pose.pose.orientation.z = -0.70328
    loading_pose.pose.orientation.w = 0.7109129

    #move to loading pose
    nav.goToPose(loading_pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback.navigation_duration > 600:
            nav.cancelTask()

    #make a service call to the attach shelf service

def main():
    rclpy.init()
    launch_thread = threading.Thread(target=navigation)
    launch_thread.start()
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    ls.run()
    

if __name__ == "__main__":
    main()