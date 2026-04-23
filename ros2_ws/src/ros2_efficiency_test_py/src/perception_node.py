#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
import message_filters


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.left_sub_ = message_filters.Subscriber(self, Image, 'camera/left/image_raw')
        self.right_sub_ = message_filters.Subscriber(self, Image, 'camera/right/image_raw')
        self.imu_sub_ = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)

        self.sync_ = message_filters.TimeSynchronizer(
            [self.left_sub_, self.right_sub_], 10)
        self.sync_.registerCallback(self.stereo_callback)

        self.odom_20hz_pub_ = self.create_publisher(Odometry, 'odometry/rate_20hz', 10)
        self.odom_200hz_pub_ = self.create_publisher(Odometry, 'odometry/rate_200hz', 10)
        self.disparity_pub_ = self.create_publisher(Image, 'disparity', 10)

    def stereo_callback(self, left_msg, right_msg):
        disparity_msg = Image()
        disparity_msg.header.stamp = left_msg.header.stamp
        disparity_msg.header.frame_id = 'disparity_frame'
        disparity_msg.height = 720
        disparity_msg.width = 1280
        disparity_msg.encoding = 'mono8'
        disparity_msg.is_bigendian = False
        disparity_msg.step = 1280
        disparity_msg.data = bytes([64] * (1280 * 720))
        self.disparity_pub_.publish(disparity_msg)

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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
