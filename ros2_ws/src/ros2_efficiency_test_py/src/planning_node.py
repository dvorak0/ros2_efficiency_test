#!/usr/bin/env python3
import os

import rclpy
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path
import message_filters


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        self.odom_sub_ = message_filters.Subscriber(self, Odometry, 'odometry/rate_20hz')
        self.disparity_sub_ = message_filters.Subscriber(self, Image, 'disparity')

        self.sync_ = message_filters.TimeSynchronizer(
            [self.odom_sub_, self.disparity_sub_], 10)
        self.sync_.registerCallback(self.planning_callback)

        self.path_pub_ = self.create_publisher(Path, 'path', 10)

    def planning_callback(self, odom_msg, disparity_msg):
        path_msg = Path()
        path_msg.header.stamp = odom_msg.header.stamp
        path_msg.header.frame_id = 'map'

        self.path_pub_.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
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
