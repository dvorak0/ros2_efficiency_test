#include <rclcpp/rclcpp.hpp>

#include "shm_msgs/msg/image1m.hpp"
#include <iostream>
#include <cstdint>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using Topic = shm_msgs::msg::Image1m;
template <typename T>
inline void set_array_string(T & dst, const std::string & text)
{
  std::memset(dst.data.data(), 0, dst.data.size());
  std::memcpy(dst.data.data(), text.c_str(),
              std::min(text.size(), dst.data.size() - 1));
}
// using Topic =  ros2_efficiency_test::msg::Image1m;
class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("camera_node") {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    left_publisher_ = this->create_publisher<Topic>(
        "camera/left/image_raw", 10);
    right_publisher_ = this->create_publisher<Topic>(
        "camera/right/image_raw", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(50),
                                std::bind(&CameraNode::timer_callback, this));
  }

private:
  void timer_callback() {

    auto left_loan = left_publisher_->borrow_loaned_message();
    auto left_msg = left_loan.get();
    left_msg.header.stamp = this->now();
    set_array_string(left_msg.header.frame_id, "left_camera_frame");
    left_msg.height = 720;
    left_msg.width = 1280;
    set_array_string(left_msg.encoding, "mono8");

    left_msg.is_bigendian = 0;
    left_msg.step = 1280;
    std::fill(left_msg.data.begin(),
          left_msg.data.begin() + 1280 * 720,
          128);
    left_publisher_->publish(std::move(left_loan));

    // right
    auto right_loan = right_publisher_->borrow_loaned_message();
    auto right_msg = right_loan.get();
    right_msg.header.stamp = this->now();
    set_array_string(right_msg.header.frame_id, "right_camera_frame");
    set_array_string(right_msg.encoding, "mono8");
    right_msg.height = 720;
    right_msg.width = 1280;
  
    right_msg.is_bigendian = 0;
    right_msg.step = 1280;
    std::fill(right_msg.data.begin(),
          right_msg.data.begin() + 1280 * 720,
          128);
    right_publisher_->publish(std::move(right_loan));
  }

  rclcpp::Publisher<Topic>::SharedPtr left_publisher_;
  rclcpp::Publisher<Topic>::SharedPtr right_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
