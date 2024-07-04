from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.costmap_2d import PyCostmap2D

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterValue

from attach_shelf_interfaces.srv import GoToLoading, ChangeFootprintPolygon, MoveForwards
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters

import sys
import threading
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchService

import numpy as np

import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.shelf_cli = self.create_client(GoToLoading, '/approach_shelf')
        while not self.shelf_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.footprint_cli = self.create_client(ChangeFootprintPolygon, '/change_footprint')
        while not self.footprint_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.global_param_cli = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
        while not self.global_param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.move_cli = self.create_client(MoveForwards, "/move_forwards")
        # self.local_param_cli = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        # while not self.local_param_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')

        self.elevator_down_publisher_ = self.create_publisher(String, '/elevator_down', 10)
        
        self.shelf_req = GoToLoading.Request()
        self.footprint_req = ChangeFootprintPolygon.Request()
        self.param_req = SetParameters.Request()
        self.move_req = MoveForwards.Request()

    def send_shelf_request(self):
        self.shelf_req.attach_to_shelf = True
        self.shelf_future = self.shelf_cli.call_async(self.shelf_req)
        rclpy.spin_until_future_complete(self, self.shelf_future)
        return self.shelf_future.result()
    
    def send_footprint_request(self, polygon):
        self.footprint_req.polygon = polygon
        self.footprint_future = self.footprint_cli.call_async(self.footprint_req)
        rclpy.spin_until_future_complete(self, self.footprint_future)
        return self.footprint_future.result()

    def send_param_request(self, name, value):
        self.param_req.parameters = [Parameter(name=name, value=value).to_parameter_msg()]
        # self.param_local_future = self.local_param_cli.call_async(self.param_req.parameters)
        # rclpy.spin_until_future_complete(self, self.param_local_future)
        self.param_global_future = self.global_param_cli.call_async(self.param_req)
        rclpy.spin_until_future_complete(self, self.param_global_future) 
        return self.param_global_future.result()
    
    def send_move_forwards_request(self, value):
        self.move_req.distance = value
        self.move_future = self.move_cli.call_async(self.move_req)
        rclpy.spin_until_future_complete(self, self.move_future)
    
    def generate_rectangle_polygon(self, height, width):
        points = []
        top_left = Point32()
        top_left.x = height / 2
        top_left.y = -width / 2
        points.append(top_left)
        top_right = Point32()
        top_right.x = height / 2
        top_right.y = width / 2
        points.append(top_right)
        bottom_right = Point32()
        bottom_right.x = -height / 2
        bottom_right.y = width / 2
        points.append(bottom_right)
        bottom_left = Point32()
        bottom_left.x = -height / 2
        bottom_left.y = -width / 2
        points.append(bottom_left)
       
        polygon = Polygon()
        polygon.points = points
        return polygon

    def generate_circle_polygon(self, radius):

        points = []

        for i in range(-100, 100):
            j = i / 100 * radius
            point = Point32()
            point.x = j
            point.y = (radius ** 2 - j ** 2) ** (1/2)
            points.append(point)
        for i in range(-100, 100):
            j = i / 100 * radius
            point = Point32()
            point.x = j * -1
            point.y = ((radius ** 2 - j ** 2) ** (1/2)) * -1
            points.append(point)
        polygon = Polygon()
        polygon.points = points
        return polygon
    
    def navigate(self):
        nav = BasicNavigator()   
        # # initial pose
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = nav.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.0
        init_pose.pose.orientation.z = 0.0
        init_pose.pose.orientation.w = 1.0
        nav.setInitialPose(init_pose)
    
        nav.waitUntilNav2Active(navigator='/bt_navigator', localizer='/amcl')


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
        print("arrived at loading area")
        #make a service call to the attach shelf service
        
        self.send_shelf_request()

        # change the footprint of the drone
        # 0.7wide 0.9 tall approximately

        rectangle = self.generate_rectangle_polygon(0.95, 0.9)
        self.send_footprint_request(rectangle)

        self.send_param_request("obstacle_layer.scan.obstacle_min_range", 1.30)
        self.send_param_request("obstacle_layer.scan.raytrace_min_range", 1.30)


        # set navigation to in front of the bench area

        init_bench_pose = PoseStamped()
        init_bench_pose.header.frame_id = 'map'
        init_bench_pose.header.stamp = nav.get_clock().now().to_msg()
        init_bench_pose.pose.position.x = 2.25
        init_bench_pose.pose.position.y = 0.0
        init_bench_pose.pose.orientation.z = 0.0
        init_bench_pose.pose.orientation.w = -1.0
        nav.goToPose(init_bench_pose)
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                print('Navigation has exceeded timeout of 180s, canceling the request.')
                nav.cancelTask()
        print("arrived at bench area")

        # # set navigation to the bench area
        bench_pose = PoseStamped()
        bench_pose.header.frame_id = 'map'
        bench_pose.header.stamp = nav.get_clock().now().to_msg()
        bench_pose.pose.position.x = 2.25
        bench_pose.pose.position.y = 1.5
        bench_pose.pose.orientation.z = -0.707
        bench_pose.pose.orientation.w = 0.707
        nav.goToPose(bench_pose)
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                print('Navigation has exceeded timeout of 180s, canceling the request.')
                nav.cancelTask()
        print("arrived at bench area")

        # # lower the shelf
        down_msg = String()
        self.elevator_down_publisher_.publish(down_msg)

        # need to move forwards 0.3m since its under the shelf

        self.send_move_forwards_request(0.3)
        

        # change the footprint of the drone back to radius

        circle = self.generate_circle_polygon(0.5)
        self.send_footprint_request(circle)
        self.send_param_request("obstacle_layer.scan.obstacle_min_range", 0.00)
        self.send_param_request("obstacle_layer.scan.raytrace_min_range", 0.00)
        

        # send back to initial pose
        nav.goToPose(init_pose)
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                print('Navigation has exceeded timeout of 180s, canceling the request.')
                nav.cancelTask()
        
        

        print("arrived at initial position")




# class ShelfClientAsync(Node):

#     def __init__(self):
#         super().__init__('shelf_client_async')
        
#         self.cli = self.create_client(GoToLoading, '/approach_shelf')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')
#         self.req = GoToLoading.Request()

   

# class GlobalFootprintPublisher(Node):
    
#     def __init__(self):
#         super().__init__('global_footprint_publisher')
#         self.publisher_ = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
    
#     def publish(self, msg):
#         if hasattr(self, "timer"):
#             self.timer.cancel()
#         self.msg = msg
#         self.timer = self.create_timer(1.0, self.callback)
       
#     def callback(self):

#         self.publisher_.publish(self.msg)


# class LocalFootprintPublisher(Node):
    
#     def __init__(self):
#         super().__init__('local_footprint_publisher')
#         self.publisher_ = self.create_publisher(Polygon, '/local_costmap/footprint', 10)

#     def publish(self, msg):
#         if hasattr(self, "timer"):
#             self.timer.cancel()
#         self.msg = msg
#         self.timer = self.create_timer(1.0, self.callback)
#     def callback(self):
#         self.publisher_.publish(self.msg)



# class ElevatorDownPublisher(Node):
#     def __init__(self):
#         super().__init__('elevator_down_publisher')
#         self.publisher_ = self.create_publisher(String, '/elevator_down', 10)

#     def publish(self):
#         msg = String()
#         self.publisher_.publish(msg)

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
            ),
            ComposableNode(
                name="footprint_server",
                plugin="nav2_apps::ChangeFootprintService",
                package="nav2_apps"
            )
        ],
        parameters=[{"thread_num": 5}]
    )
    return container

def navigation():
    navigationNode = NavigationNode()
    navigationNode.navigate()
    

def main():
    rclpy.init()
    launch_thread = threading.Thread(target=navigation)
    launch_thread.start()
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    ls.run()
   
    

if __name__ == "__main__":
    main()