#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "shm_msgs/msg/image1m.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <cstring>
#include <optional>
#include <iostream>
#include <cstdint>
#include <chrono>
#include <functional>
#include <memory>
#include <string>


using Topic = shm_msgs::msg::Image1m;
class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode() : Node("perception_node")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    // Subscribe img_msg left/right
    left_sub_.subscribe(this, "camera/left/image_raw");
    right_sub_.subscribe(this, "camera/right/image_raw");
    // Subscribe imu_msg
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 10, std::bind(&PerceptionNode::imu_callback, this, std::placeholders::_1));
    sync_ = std::make_shared<message_filters::TimeSynchronizer<shm_msgs::msg::Image1m, shm_msgs::msg::Image1m>>(left_sub_, right_sub_, 10);
    sync_->registerCallback(std::bind(&PerceptionNode::stereo_callback, this, std::placeholders::_1, std::placeholders::_2));
    // Publisher defined
    disparity_pub_ = this->create_publisher<shm_msgs::msg::Image1m>("disparity", 10);
    odom_20hz_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/rate_20hz", 10);
    odom_200hz_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/rate_200hz", 10);
  }

private:
  void stereo_callback(const shm_msgs::msg::Image1m::ConstSharedPtr &left_msg, const shm_msgs::msg::Image1m::ConstSharedPtr &right_msg)
  {
    (void)right_msg;
    // Disparity_msg publish
    auto disparity_loaned = disparity_pub_->borrow_loaned_message();
    auto &disparity_msg = disparity_loaned.get();
    // Synchronize with the timestamp of the left camera
    disparity_msg.header.stamp = left_msg->header.stamp;
    std::string frame_id = "disparity_frame";
    std::memcpy(disparity_msg.header.frame_id.data.data(), frame_id.data(), frame_id.size());
    disparity_msg.header.frame_id.size = frame_id.size();
    //Encoding
    std::string encoding = "mono8";
    std::memcpy(disparity_msg.encoding.data.data(), encoding.data(), encoding.size());
    disparity_msg.encoding.size = encoding.size();
    //resize
    disparity_msg.height = 720;
    disparity_msg.width = 1280;
    disparity_msg.is_bigendian = false;
    disparity_msg.step = 1280;
    std::fill_n(disparity_msg.data.begin(), 1280*720, 128);

    disparity_pub_->publish(std::move(disparity_loaned));

    // odom_msg publish
    auto odom_loaned = odom_200hz_pub_->borrow_loaned_message();
    auto &odom_msg = odom_loaned.get();
    odom_msg.header.stamp = left_msg->header.stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = 0.1;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_20hz_pub_->publish(std::move(odom_loaned));
    odom_200hz_pub_->publish(std::move(odom_loaned));
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    (void)msg;
    //odom_msg publish
    auto odom_loaned = odom_200hz_pub_->borrow_loaned_message();
    auto &odom_msg = odom_loaned.get();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = 0.1;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;

    odom_200hz_pub_->publish(std::move(odom_loaned));
  }

  message_filters::Subscriber<shm_msgs::msg::Image1m> left_sub_;
  message_filters::Subscriber<shm_msgs::msg::Image1m> right_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<shm_msgs::msg::Image1m, shm_msgs::msg::Image1m>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_20hz_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_200hz_pub_;
  rclcpp::Publisher<shm_msgs::msg::Image1m>::SharedPtr disparity_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
