#!/usr/bin/env python3
import os

import rclpy
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.path_sub_ = self.create_subscription(
            Path, 'path', self.path_callback, 10)
        self.odom_sub_ = self.create_subscription(
            Odometry, 'odometry/rate_200hz', self.odom_callback, 10)

        self.cmd_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer_ = self.create_timer(0.005, self.control_callback)  # 5ms = 200Hz

        self.latest_path_ = None
        self.latest_odom_ = None

    def path_callback(self, msg):
        self.latest_path_ = msg

    def odom_callback(self, msg):
        self.latest_odom_ = msg

    def control_callback(self):
        if self.latest_path_ is None or self.latest_odom_ is None:
            return

        cmd_msg = Twist()
        cmd_msg.linear.x = 0.5
        cmd_msg.angular.z = 0.1
        self.cmd_pub_.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    executor = None

    try:
        if os.environ.get('EVENT_EXECUTOR') == '1':
            executor = EventsExecutor()
            executor.add_node(node)
            executor.spin()
        else:
            rclpy.spin(node)
    finally:
        if executor is not None:
            executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
