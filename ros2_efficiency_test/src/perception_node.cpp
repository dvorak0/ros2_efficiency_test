#include <cmath>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

class PerceptionNode : public rclcpp::Node {
public:
  PerceptionNode() : Node("perception_node") {
    left_sub_.subscribe(this, "camera/left/image_raw");
    right_sub_.subscribe(this, "camera/right/image_raw");
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10, std::bind(&PerceptionNode::imu_callback, this, std::placeholders::_1));

    sync_ = std::make_shared<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(left_sub_,
                                                           right_sub_, 10);
    sync_->registerCallback(std::bind(&PerceptionNode::stereo_callback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

    odom_20hz_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "odometry/rate_20hz", 10);
    odom_200hz_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "odometry/rate_200hz", 10);
    disparity_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>("disparity", 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "disparity/camera_info", 10);
  }

private:
  void
  stereo_callback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr &right_msg) {
    (void)right_msg;

    auto disparity_msg = sensor_msgs::msg::Image();
    disparity_msg.header.stamp = left_msg->header.stamp;
    disparity_msg.header.frame_id = "disparity_frame";
    disparity_msg.height = 720;
    disparity_msg.width = 1280;
    disparity_msg.encoding = "32FC1";
    disparity_msg.is_bigendian = false;
    disparity_msg.step = disparity_msg.width * sizeof(float);
    disparity_msg.data.resize(disparity_msg.step * disparity_msg.height);

    constexpr float kTwoPi = 6.28318530717958647692f;
    auto *disparity_ptr = reinterpret_cast<float *>(disparity_msg.data.data());
    for (uint32_t v = 0; v < disparity_msg.height; ++v) {
      for (uint32_t u = 0; u < disparity_msg.width; ++u) {
        const float x = static_cast<float>(u) / static_cast<float>(disparity_msg.width) * kTwoPi;
        disparity_ptr[v * disparity_msg.width + u] = 1.0f + std::sin(x);
      }
    }

    disparity_pub_->publish(disparity_msg);

    auto camera_info_msg = sensor_msgs::msg::CameraInfo();
    camera_info_msg.header.stamp = left_msg->header.stamp;
    camera_info_msg.header.frame_id = "disparity_frame";
    camera_info_msg.height = disparity_msg.height;
    camera_info_msg.width = disparity_msg.width;
    camera_info_msg.distortion_model = "plumb_bob";
    camera_info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera_info_msg.k = {1.0, 0.0, static_cast<double>(disparity_msg.width) / 2.0,
                         0.0, 1.0, static_cast<double>(disparity_msg.height) / 2.0,
                         0.0, 0.0, 1.0};
    camera_info_msg.r = {1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0};
    camera_info_msg.p = {1.0, 0.0, static_cast<double>(disparity_msg.width) / 2.0, 0.0,
                         0.0, 1.0, static_cast<double>(disparity_msg.height) / 2.0, 0.0,
                         0.0, 0.0, 1.0, 0.0};
    camera_info_pub_->publish(camera_info_msg);

    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = left_msg->header.stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = 0.1;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;

    odom_20hz_pub_->publish(odom_msg);
    odom_200hz_pub_->publish(odom_msg);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    (void)msg;
    
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    odom_msg.pose.pose.position.x = 0.1;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    
    odom_200hz_pub_->publish(odom_msg);
  }

  message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image,
                                                    sensor_msgs::msg::Image>>
      sync_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_20hz_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_200hz_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}