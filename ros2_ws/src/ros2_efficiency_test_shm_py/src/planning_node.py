#!/usr/bin/env python3
import os

import rclpy
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from shm_image import SHMImageSubscriber


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        self.odom_cache_ = {}
        self.disparity_cache_ = {}
        self.odom_sub_ = self.create_subscription(Odometry, 'odometry/rate_20hz', self.odom_callback, 10)
        self.disparity_sub_ = SHMImageSubscriber(self, 'disparity_shm', self.disparity_callback)

        self.path_pub_ = self.create_publisher(Path, 'path', 10)

    def _stamp_key(self, stamp):
        return (stamp.sec, stamp.nanosec)

    def odom_callback(self, odom_msg):
        key = self._stamp_key(odom_msg.header.stamp)
        disparity_msg = self.disparity_cache_.pop(key, None)
        if disparity_msg is None:
            self.odom_cache_[key] = odom_msg
            return
        self.planning_callback(odom_msg, disparity_msg)

    def disparity_callback(self, disparity_msg):
        # Force numpy view materialization for apples-to-apples image receive cost.
        _ = int(disparity_msg.image[0, 0])
        key = self._stamp_key(disparity_msg.header.stamp)
        odom_msg = self.odom_cache_.pop(key, None)
        if odom_msg is None:
            self.disparity_cache_[key] = disparity_msg
            return
        self.planning_callback(odom_msg, disparity_msg)

    def planning_callback(self, odom_msg, disparity_msg):
        path_msg = Path()
        path_msg.header.stamp = odom_msg.header.stamp
        path_msg.header.frame_id = 'map'

        self.path_pub_.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()

    if os.environ.get('EVENT_EXECUTOR') == '1':
        executor = EventsExecutor()
        executor.add_node(node)
        executor.spin()
    else:
        rclpy.spin(node)

if __name__ == '__main__':
    main()
