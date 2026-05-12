#!/usr/bin/env python3
import os

import rclpy
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer_ = self.create_timer(0.005, self.timer_callback)  # 5ms = 200Hz

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_frame'

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.1

        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()

    if os.environ.get('EVENT_EXECUTOR') == '1':
        executor = EventsExecutor()
        executor.add_node(node)
        executor.spin()
    else:
        rclpy.spin(node)

if __name__ == '__main__':
    main()
