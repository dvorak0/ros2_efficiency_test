cmake_minimum_required(VERSION 3.8)
project(ros2_efficiency_test_py)

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



install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
