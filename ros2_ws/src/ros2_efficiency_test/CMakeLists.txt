cmake_minimum_required(VERSION 3.8)
project(ros2_efficiency_test)

# Default to RelWithDebInfo if no build type is set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -fno-omit-frame-pointer")
set(CMAKE_C_FLAGS_RELWITHDEBINFO   "${CMAKE_C_FLAGS_RELWITHDEBINFO} -fno-omit-frame-pointer")

add_compile_options(-fno-omit-frame-pointer)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(shm_msgs REQUIRED)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs)

add_executable(shm_camera_node src/shm_camera_node.cpp)
ament_target_dependencies(shm_camera_node rclcpp sensor_msgs shm_msgs)

add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node rclcpp sensor_msgs)

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node rclcpp sensor_msgs nav_msgs message_filters)

add_executable(planning_node src/planning_node.cpp)
ament_target_dependencies(planning_node rclcpp nav_msgs sensor_msgs message_filters)

add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp nav_msgs sensor_msgs message_filters geometry_msgs)

add_executable(giant_nodes_system_v1 src/giant_nodes_system_v1.cpp)
ament_target_dependencies(giant_nodes_system_v1 rclcpp sensor_msgs nav_msgs geometry_msgs message_filters)

add_executable(giant_nodes_system_v2 src/giant_nodes_system_v2.cpp)
ament_target_dependencies(giant_nodes_system_v2 rclcpp sensor_msgs nav_msgs geometry_msgs message_filters)

add_executable(giant_nodes_system_v3 src/giant_nodes_system_v3.cpp)
ament_target_dependencies(giant_nodes_system_v3 rclcpp sensor_msgs nav_msgs geometry_msgs message_filters)

install(TARGETS
  camera_node
  imu_node
  perception_node
  planning_node
  control_node
  shm_camera_node
  giant_nodes_system_v1
  giant_nodes_system_v2
  giant_nodes_system_v3
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

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
