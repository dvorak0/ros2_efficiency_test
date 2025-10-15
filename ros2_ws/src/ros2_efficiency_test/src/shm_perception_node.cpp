#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "ros2_efficiency_test/msg/image1m.hpp"
#include "ros2_efficiency_test/msg/imu1k.hpp"
#include <cstring>
#include <optional>
#include <iostream>
#include <cstdint>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

template <typename T>
inline void set_array_string(T & dst, const std::string & text)
{
  std::memset(dst.data.data(), 0, dst.data.size());
  std::memcpy(dst.data.data(), text.c_str(),
              std::min(text.size(), dst.data.size() - 1));
}
using Topic = ros2_efficiency_test::msg::Image1m;
class PerceptionNode : public rclcpp::Node {
public:
  PerceptionNode() : Node("perception_node") {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    // 左右图像订阅（零拷贝）
    left_sub_.subscribe(this, "camera/left/image_raw");
    right_sub_.subscribe(this, "camera/right/image_raw");

    // IMU 订阅
    imu_sub_ = this->create_subscription<ros2_efficiency_test::msg::Imu1k>(
        "imu/data", 10,
        std::bind(&PerceptionNode::imu_callback, this, std::placeholders::_1));

    // 同步回调
    sync_ = std::make_shared<message_filters::TimeSynchronizer<
        ros2_efficiency_test::msg::Image1m, ros2_efficiency_test::msg::Image1m>>(left_sub_, right_sub_, 10);

    sync_->registerCallback(std::bind(&PerceptionNode::stereo_callback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

    disparity_pub_ =
        this->create_publisher<ros2_efficiency_test::msg::Image1m>("disparity", 10);
    odom_20hz_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>("odometry/rate_20hz", 10);
    odom_200hz_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>("odometry/rate_200hz", 10);
  }

private:

    void stereo_callback(const ros2_efficiency_test::msg::Image1m::ConstSharedPtr &left_msg,
                  const ros2_efficiency_test::msg::Image1m::ConstSharedPtr &right_msg) {
    (void)right_msg;

    auto disparity_loaned = disparity_pub_->borrow_loaned_message();
    auto & disparity_msg = disparity_loaned.get();
    disparity_msg.header.stamp = left_msg->header.stamp;
    set_array_string(disparity_msg.header.frame_id, "disparity_frame");
    disparity_msg.height = 720;
    disparity_msg.width = 1280;
    set_array_string(disparity_msg.encoding, "mono8");

   
    disparity_msg.is_bigendian = false;
    disparity_msg.step = 1280;
    std::fill(disparity_msg.data.begin(),
          disparity_msg.data.begin() + 1280 * 720,
          64);
    disparity_pub_->publish(std::move(disparity_loaned));

    auto odom_loaned = odom_200hz_pub_->borrow_loaned_message();
    auto & odom_msg = odom_loaned.get();
    odom_msg.header.stamp = left_msg->header.stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = 0.1;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_20hz_pub_->publish(std::move(odom_loaned));
    odom_200hz_pub_->publish(std::move(odom_loaned));

  }

  void imu_callback(const ros2_efficiency_test::msg::Imu1k::SharedPtr msg) {
    (void)msg;
    
    auto odom_loaned = odom_200hz_pub_->borrow_loaned_message();
    auto & odom_msg = odom_loaned.get();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    odom_msg.pose.pose.position.x = 0.1;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    
   odom_200hz_pub_->publish(std::move(odom_loaned));
  }

  message_filters::Subscriber<ros2_efficiency_test::msg::Image1m> left_sub_;
  message_filters::Subscriber<ros2_efficiency_test::msg::Image1m> right_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<ros2_efficiency_test::msg::Image1m, ros2_efficiency_test::msg::Image1m>> sync_;

  rclcpp::Subscription<ros2_efficiency_test::msg::Imu1k>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_20hz_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_200hz_pub_;
  rclcpp::Publisher<ros2_efficiency_test::msg::Image1m>::SharedPtr disparity_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
