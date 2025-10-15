from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_efficiency_test',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='ros2_efficiency_test',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        Node(
            package='ros2_efficiency_test',
            executable='perception_node',
            name='perception_node',
            output='screen'
        ),
        Node(
            package='ros2_efficiency_test',
            executable='planning_node',
            name='planning_node',
            output='screen'
        ),
        Node(
            package='ros2_efficiency_test',
            executable='control_node',
            name='control_node',
            output='screen'
        )
    ])