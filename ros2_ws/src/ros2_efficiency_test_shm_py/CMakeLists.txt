cmake_minimum_required(VERSION 3.8)
project(ros2_efficiency_test_shm_py)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ShmImage.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  src/camera_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_node
)
install(PROGRAMS
  src/control_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME control_node
)
install(PROGRAMS
  src/imu_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_node
)
install(PROGRAMS
  src/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_node
)
install(PROGRAMS
  src/planning_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME planning_node
)
install(PROGRAMS
  src/shm_image.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
