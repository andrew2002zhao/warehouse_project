cmake_minimum_required(VERSION 3.8)
project(nav2_apps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(attach_shelf_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(attach_server SHARED src/attach_server.cpp)
ament_target_dependencies(attach_server 
	rclcpp
	rclcpp_components
	nav_msgs
	sensor_msgs
	geometry_msgs
	attach_shelf_interfaces
	tf2_ros
)
rclcpp_components_register_nodes(attach_server "nav2_apps::ApproachService")

install(
	TARGETS
	attach_server
	DESTINATION
	lib/
)
install(
	DIRECTORY
	launch
	DESTINATION
	share/${PROJECT_NAME}
)

ament_package()



