cmake_minimum_required(VERSION 3.8)
project(tdk_slam_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)

add_executable(laser_angle_filter_node src/laser_angle_filter.cpp)
ament_target_dependencies(laser_angle_filter_node
  rclcpp
  sensor_msgs
)
add_executable(robot_pose_publisher_node src/robot_pose_publisher.cpp)
ament_target_dependencies(robot_pose_publisher_node 
  rclcpp
  tf2_ros
  geometry_msgs
)
add_executable(localization_manager_node src/localization_manager.cpp)
ament_target_dependencies(localization_manager_node
  rclcpp
  geometry_msgs
  std_msgs
  nav2_msgs
  cartographer_ros_msgs
)

install(TARGETS
  laser_angle_filter_node
  robot_pose_publisher_node
  localization_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  sim
  urdf
  cartographer_config
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
