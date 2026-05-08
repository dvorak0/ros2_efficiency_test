#!/usr/bin/env python3
import os

import rclpy
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.left_publisher_ = self.create_publisher(Image, 'camera/left/image_raw', 10)
        self.right_publisher_ = self.create_publisher(Image, 'camera/right/image_raw', 10)
        self.image_payload = b"\x80" * (1280 * 720)
        self.timer_ = self.create_timer(0.05, self.timer_callback)  # 50ms = 20Hz

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        left_msg = Image()
        left_msg.header.stamp = now
        left_msg.header.frame_id = 'left_camera_frame'
        left_msg.height = 720
        left_msg.width = 1280
        left_msg.encoding = 'mono8'
        left_msg.is_bigendian = False
        left_msg.step = 1280
        left_msg.data = self.image_payload
        self.left_publisher_.publish(left_msg)

        right_msg = Image()
        right_msg.header.stamp = now
        right_msg.header.frame_id = 'right_camera_frame'
        right_msg.height = 720
        right_msg.width = 1280
        right_msg.encoding = 'mono8'
        right_msg.is_bigendian = False
        right_msg.step = 1280
        right_msg.data = self.image_payload
        self.right_publisher_.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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
