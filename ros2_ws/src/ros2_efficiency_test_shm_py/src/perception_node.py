#!/usr/bin/env python3
import os

import numpy as np
import rclpy
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from shm_image import SHMImagePublisher, SHMImageSubscriber, stamp_key


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.left_cache_ = {}
        self.right_cache_ = {}
        self.left_sub_ = SHMImageSubscriber(self, 'camera/left/image_raw_shm', self.left_callback)
        self.right_sub_ = SHMImageSubscriber(self, 'camera/right/image_raw_shm', self.right_callback)
        self.imu_sub_ = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)

        self.odom_20hz_pub_ = self.create_publisher(Odometry, 'odometry/rate_20hz', 10)
        self.odom_200hz_pub_ = self.create_publisher(Odometry, 'odometry/rate_200hz', 10)
        self.disparity_pub_ = SHMImagePublisher(self, 'disparity_shm', width=1280, height=720, encoding='mono8', slots=8)
        self.disparity_payload = np.full((720, 1280), 64, dtype=np.uint8)

    def left_callback(self, left_msg):
        _ = int(left_msg.image[0, 0])
        key = stamp_key(left_msg.header.stamp)
        right_msg = self.right_cache_.pop(key, None)
        if right_msg is None:
            self.left_cache_[key] = left_msg
            return
        self.stereo_callback(left_msg, right_msg)

    def right_callback(self, right_msg):
        _ = int(right_msg.image[0, 0])
        key = stamp_key(right_msg.header.stamp)
        left_msg = self.left_cache_.pop(key, None)
        if left_msg is None:
            self.right_cache_[key] = right_msg
            return
        self.stereo_callback(left_msg, right_msg)

    def stereo_callback(self, left_msg, right_msg):
        self.disparity_pub_.publish(self.disparity_payload, header=left_msg.header)

        odom_msg = Odometry()
        odom_msg.header.stamp = left_msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = 0.1
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0

        self.odom_20hz_pub_.publish(odom_msg)
        self.odom_200hz_pub_.publish(odom_msg)

    def imu_callback(self, msg):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = 0.1
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0

        self.odom_200hz_pub_.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    if os.environ.get('EVENT_EXECUTOR') == '1':
        executor = EventsExecutor()
        executor.add_node(node)
        executor.spin()
    else:
        rclpy.spin(node)

if __name__ == '__main__':
    main()
