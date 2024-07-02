from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.costmap_2d import PyCostmap2D

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration


from geometry_msgs.msg._pose_stamped import PoseStamped
from nav_msgs.msg import OccupancyGrid

import sys
import threading
from attach_shelf_interfaces.srv import GoToLoading
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchService

import numpy as np


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GoToLoading, '/approach_shelf')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GoToLoading.Request()

    def send_request(self):
        self.req.attach_to_shelf = True
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


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
    c
    # initial pose
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = nav.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.orientation.z = 0.0
    init_pose.pose.orientation.w = 1.0
    nav.setInitialPose(init_pose)
  
    nav.waitUntilNav2Active(navigator='/bt_navigator', localizer='/amcl')

    # blackout the cone area
    # [1.5, -1.2] [1.5, -0.9] [5, -1.2] [5, -0.9]


    # target pose
    loading_pose = PoseStamped()
    loading_pose.header.frame_id = 'map'
    loading_pose.header.stamp = nav.get_clock().now().to_msg()
    loading_pose.pose.position.x = 5.8
    loading_pose.pose.position.y = 0.0
    loading_pose.pose.orientation.z = -0.707
    loading_pose.pose.orientation.w = 0.707

    #move to loading pose
    nav.goToPose(loading_pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
            print('Navigation has exceeded timeout of 180s, canceling the request.')
            nav.cancelTask()
    print("ARRIVED AT TARGET POSE")
    #make a service call to the attach shelf service
    client = MinimalClientAsync()
    client.send_request()
    client.destroy_node()

    # change the footprint of the drone
    # 0.7wide 0.9 tall approximately

    # set navigation to the bench area

    # change the footprint of the drone back to radius

def main():
    rclpy.init()
    launch_thread = threading.Thread(target=navigation)
    launch_thread.start()
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    ls.run()
   
    

if __name__ == "__main__":
    main()