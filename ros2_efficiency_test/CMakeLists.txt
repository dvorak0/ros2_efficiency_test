cmake_minimum_required(VERSION 3.8)
project(ros2_efficiency_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs)

add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node rclcpp sensor_msgs)

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node message_filters nav_msgs rclcpp sensor_msgs)

add_executable(planning_node src/planning_node.cpp)
ament_target_dependencies(planning_node message_filters nav_msgs rclcpp sensor_msgs)

add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node geometry_msgs nav_msgs rclcpp)

add_executable(giant_nodes_system_v1 src/giant_nodes_system_v1.cpp)
ament_target_dependencies(giant_nodes_system_v1 geometry_msgs message_filters nav_msgs rclcpp sensor_msgs)

add_executable(giant_nodes_system_v2 src/giant_nodes_system_v2.cpp)
ament_target_dependencies(giant_nodes_system_v2 geometry_msgs message_filters nav_msgs rclcpp sensor_msgs)

add_executable(giant_nodes_system_v3 src/giant_nodes_system_v3.cpp)
ament_target_dependencies(giant_nodes_system_v3 geometry_msgs message_filters nav_msgs rclcpp sensor_msgs)

install(
  TARGETS
    camera_node
    imu_node
    perception_node
    planning_node
    control_node
    giant_nodes_system_v1
    giant_nodes_system_v2
    giant_nodes_system_v3
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
