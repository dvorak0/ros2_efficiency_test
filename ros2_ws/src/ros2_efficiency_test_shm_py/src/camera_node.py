#!/usr/bin/env python3
import os

import numpy as np
import rclpy
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from std_msgs.msg import Header
from shm_image import SHMImagePublisher


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.left_publisher_ = SHMImagePublisher(self, 'camera/left/image_raw_shm', width=1280, height=720, encoding='mono8', slots=8)
        self.right_publisher_ = SHMImagePublisher(self, 'camera/right/image_raw_shm', width=1280, height=720, encoding='mono8', slots=8)
        self.left_payload = np.full((720, 1280), 128, dtype=np.uint8)
        self.right_payload = np.full((720, 1280), 128, dtype=np.uint8)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()

        left_header = Header()
        left_header.stamp = stamp
        left_header.frame_id = 'left_camera'
        self.left_publisher_.publish(self.left_payload, header=left_header)

        right_header = Header()
        right_header.stamp = stamp
        right_header.frame_id = 'right_camera'
        self.right_publisher_.publish(self.right_payload, header=right_header)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    if os.environ.get('EVENT_EXECUTOR') == '1':
        executor = EventsExecutor()
        executor.add_node(node)
        executor.spin()
    else:
        rclpy.spin(node)

if __name__ == '__main__':
    main()
