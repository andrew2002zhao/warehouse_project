cmake_minimum_required(VERSION 3.8)
project(custom_nav2_costmap_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

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



add_library(${PROJECT_NAME}_core SHARED src/filter_layer.cpp)
include_directories(include)
ament_target_dependencies(${PROJECT_NAME}_core
	rclcpp
	nav2_costmap_2d
	pluginlib
)

install(
	DIRECTORY
	include
	DESTINATION
	share/${PROJECT_NAME}
)


install(TARGETS 
		${PROJECT_NAME}_core
        DESTINATION 
		lib
		)


pluginlib_export_plugin_description_file(nav2_costmap_2d filter_layer.xml)

ament_package()
