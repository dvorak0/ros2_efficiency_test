#include <rclcpp/rclcpp.hpp>
#include "shm_msgs/msg/image1m.hpp"
#include <iostream>
#include <cstdint>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using Topic = shm_msgs::msg::Image1m;


class CameraNode : public rclcpp::Node
{
public:
  CameraNode() : Node("camera_node")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    left_publisher_ = this->create_publisher<Topic>(
        "camera/left/image_raw", 10);
    right_publisher_ = this->create_publisher<Topic>(
        "camera/right/image_raw", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CameraNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Left camera_msg publish
    // Borrow_loaned_message
    auto left_loan = left_publisher_->borrow_loaned_message();
    auto &left_msg = left_loan.get();
    // Timestamp
    auto now = this->now();
    left_msg.header.stamp = now;
    // Frame_id
    std::string left_frame_id = "left_camera_frame";
    std::memcpy(left_msg.header.frame_id.data.data(), left_frame_id.data(), left_frame_id.size());
    left_msg.header.frame_id.size = left_frame_id.size();
    // Encoding
    std::string encoding = "mono8";
    std::memcpy(left_msg.encoding.data.data(), encoding.data(), encoding.size());
    left_msg.encoding.size = encoding.size();
    // Resize
    left_msg.height = 720;
    left_msg.width = 1280;
    left_msg.step = 1280; 
    std::fill_n(left_msg.data.begin(), 1280*720, 128);
    // is_bigendian
    left_msg.is_bigendian = 0;
 
    left_publisher_->publish(std::move(left_loan));
    // Right camera publish(same to left)
    auto right_loan = right_publisher_->borrow_loaned_message();
    auto &right_msg = right_loan.get();
    right_msg.header.stamp = now;
    std::string right_frame_id = "right_camera_frame";
    std::memcpy(right_msg.header.frame_id.data.data(), right_frame_id.data(), right_frame_id.size());
    right_msg.header.frame_id.size = right_frame_id.size();
    std::memcpy(right_msg.encoding.data.data(), encoding.data(), encoding.size());
    right_msg.encoding.size = encoding.size();
    right_msg.height = 720;
    right_msg.width = 1280;
    right_msg.is_bigendian = 0;
    right_msg.step = 1280;
    std::fill_n(right_msg.data.begin(), 1280*720, 128);
    right_publisher_->publish(std::move(right_loan));
  }
  rclcpp::Publisher<Topic>::SharedPtr left_publisher_;
  rclcpp::Publisher<Topic>::SharedPtr right_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
