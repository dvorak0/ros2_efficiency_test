# ROS2 Efficiency Test Project

A ROS2 project designed to measure the efficiency of ROS2 communication patterns with realistic sensor data rates and processing pipelines. This project simulates a complete robotics perception and control system with different architectural approaches for performance testing.

## Project Overview

This project implements a realistic robotics pipeline that includes:
- Stereo camera data processing at 20Hz (720p grayscale)
- IMU data generation at 200Hz
- Multi-rate odometry processing
- Path planning at 20Hz
- Control loop at 200Hz

## Architecture

The project provides two architectural approaches:
1. **Distributed Nodes**: Individual nodes for each component
2. **Monolithic Systems**: Consolidated nodes (experimental variants v1, v2, v3)

## Nodes

### Standard Distributed Nodes

1. **camera_node**: Publishes stereo images (left/right) at 20Hz
   - Resolution: 720p grayscale (1280x720)
   - Topic: `/camera/left/image_raw`, `/camera/right/image_raw`

2. **imu_node**: Publishes IMU data at 200Hz
   - Linear acceleration: (0, 0, 9.81)
   - Angular velocity: (0, 0, 0.1)
   - Topic: `/imu/data`

3. **perception_node**: 
   - Uses time synchronization for stereo image processing
   - Outputs disparity maps and odometry at 20Hz from stereo
   - Outputs high-rate odometry at 200Hz from IMU
   - Topics: `/odometry/rate_20hz`, `/odometry/rate_200hz`, `/disparity`

4. **planning_node**: 
   - Uses time synchronization for disparity + odometry processing
   - Outputs path planning results at 20Hz
   - Topic: `/path`

5. **control_node**: 
   - Receives 200Hz odometry and 20Hz path
   - Runs control loop at 200Hz
   - Outputs velocity commands
   - Topic: `/cmd_vel`

### Experimental Monolithic Variants

- **giant_nodes_system_v1**: All nodes in single process with multi-threaded executor
- **giant_nodes_system_v2**: All nodes in single process with multi-threaded executor
- **giant_nodes_system_v3**: All nodes in single process with static single-threaded executor and intra-process communication

## Build Instructions

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Create workspace if needed
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the project (if not already present)
# git clone <repository-url> ros2_efficiency_test

# Build the package
cd ~/ros2_ws
colcon build --packages-select ros2_efficiency_test

# Source the workspace
source install/setup.bash
```

## Run Instructions

### Run Standard Distributed System
```bash
# Launch all standard nodes
ros2 launch ros2_efficiency_test efficiency_test.launch.py

# Run individual nodes
ros2 run ros2_efficiency_test camera_node
ros2 run ros2_efficiency_test imu_node
ros2 run ros2_efficiency_test perception_node
ros2 run ros2_efficiency_test planning_node
ros2 run ros2_efficiency_test control_node
```

### Run Experimental Variants
```bash
# Run monolithic systems
ros2 run ros2_efficiency_test giant_nodes_system_v1
ros2 run ros2_efficiency_test giant_nodes_system_v2
ros2 run ros2_efficiency_test giant_nodes_system_v3
```

## Topics

### Camera Topics
- `/camera/left/image_raw` (sensor_msgs/Image): Left stereo image at 20Hz
- `/camera/right/image_raw` (sensor_msgs/Image): Right stereo image at 20Hz

### Sensor Topics
- `/imu/data` (sensor_msgs/Imu): IMU data at 200Hz

### Perception Topics
- `/odometry/rate_20hz` (nav_msgs/Odometry): Odometry from stereo processing at 20Hz
- `/odometry/rate_200hz` (nav_msgs/Odometry): High-rate odometry at 200Hz
- `/disparity` (sensor_msgs/Image): Disparity maps from stereo processing

### Planning Topics
- `/path` (nav_msgs/Path): Planned path from planning node at 20Hz

### Control Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands from control node at 200Hz

## System Architecture

### Data Flow
```
camera_node (20Hz) → perception_node → planning_node → control_node → cmd_vel
                    ↗                ↗
imu_node (200Hz) → perception_node → control_node
```

### Timing Characteristics
- **Camera**: 50ms period (20Hz)
- **IMU**: 5ms period (200Hz)
- **Perception**: 50ms stereo processing (20Hz), 5ms IMU processing (200Hz)
- **Planning**: 50ms period (20Hz)
- **Control**: 5ms period (200Hz)

## Performance Monitoring

Use ROS2 command-line tools to monitor system performance:

```bash
# Monitor topic frequencies
ros2 topic hz /camera/left/image_raw
ros2 topic hz /imu/data
ros2 topic hz /odometry/rate_200hz
ros2 topic hz /cmd_vel

# Monitor system resources
htop

# Monitor ROS2 node statistics
ros2 node list
ros2 topic list
ros2 service list

# Monitor topic bandwidth
ros2 topic bw /camera/left/image_raw

# Monitor memory usage
ros2 run rqt_graph rqt_graph
```

## Package Structure

```
ros2_efficiency_test/
├── src/                           # Source files
│   ├── camera_node.cpp           # Stereo camera publisher
│   ├── imu_node.cpp              # IMU data publisher
│   ├── perception_node.cpp       # Stereo + IMU processing
│   ├── planning_node.cpp         # Path planning
│   ├── control_node.cpp          # Control loop
│   ├── giant_nodes_system_v1.cpp # Monolithic system v1
│   ├── giant_nodes_system_v2.cpp # Monolithic system v2
│   └── giant_nodes_system_v3.cpp # Monolithic system v3
├── include/                      # Header files
├── launch/                       # Launch files
│   └── efficiency_test.launch.py # Launch all standard nodes
├── CMakeLists.txt               # Build configuration
├── package.xml                  # Package metadata
└── dev/                         # Development container files
    ├── Dockerfile
    ├── build.sh
    └── run.sh
```

## Dependencies

- **ROS2 Humble** or later
- **rclcpp**: ROS2 C++ client library
- **sensor_msgs**: Sensor message definitions
- **nav_msgs**: Navigation message definitions
- **geometry_msgs**: Geometry message definitions
- **message_filters**: Message synchronization
- **cv_bridge**: OpenCV bridge for ROS2
- **OpenCV**: Computer vision library

## Development Environment

This project includes development container support for consistent development environments.

### Using Dev Container
```bash
# Build development container
cd dev/
./build.sh

# Run development container
./run.sh
# Inside container: colcon build
```

