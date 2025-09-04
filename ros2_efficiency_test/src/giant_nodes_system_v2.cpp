#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

// Camera Node
class CameraNode : public rclcpp::Node {
public:
  CameraNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("camera_node", options) {
    left_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/left/image_raw", 10);
    right_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/right/image_raw", 10);
    timer_ = this->create_wall_timer(
        50ms, std::bind(&CameraNode::timer_callback, this));
  }

private:
  void timer_callback() {
    auto now = this->now();

    auto left_msg = std::make_unique<sensor_msgs::msg::Image>();
    left_msg->header.stamp = now;
    left_msg->header.frame_id = "left_camera_frame";
    left_msg->height = 720;
    left_msg->width = 1280;
    left_msg->encoding = "mono8";
    left_msg->is_bigendian = false;
    left_msg->step = 1280;
    left_msg->data.resize(1280 * 720, 128);

    // print the address of left_msg to verify intra-process communication
    RCLCPP_INFO(this->get_logger(), "Publishing left image at address: %p",
                static_cast<const void *>(left_msg.get()));
    left_publisher_->publish(std::move(left_msg));

    auto right_msg = std::make_unique<sensor_msgs::msg::Image>();
    right_msg->header.stamp = now;
    right_msg->header.frame_id = "right_camera_frame";
    right_msg->height = 720;
    right_msg->width = 1280;
    right_msg->encoding = "mono8";
    right_msg->is_bigendian = false;
    right_msg->step = 1280;
    right_msg->data.resize(1280 * 720, 128);
    right_publisher_->publish(std::move(right_msg));
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// IMU Node
class ImuNode : public rclcpp::Node {
public:
  ImuNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("imu_node", options) {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    timer_ =
        this->create_wall_timer(5ms, std::bind(&ImuNode::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_frame";

    msg.angular_velocity.x = 0.0;
    msg.angular_velocity.y = 0.0;
    msg.angular_velocity.z = 0.1;

    msg.linear_acceleration.x = 0.0;
    msg.linear_acceleration.y = 0.0;
    msg.linear_acceleration.z = 9.81;

    publisher_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Perception Node
class PerceptionNode : public rclcpp::Node {
public:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr latest_left_msg_;
  sensor_msgs::msg::Image::SharedPtr latest_right_msg_;

  PerceptionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("perception_node", options) {
    latest_left_msg_ = nullptr;
    latest_right_msg_ = nullptr;
    odom_20hz_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "odometry/rate_20hz", 10);
    odom_200hz_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "odometry/rate_200hz", 10);
    disparity_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>("disparity", 10);

    left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/left/image_raw", 10,
        std::bind(&PerceptionNode::left_callback, this, std::placeholders::_1));
    right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/right/image_raw", 10,
        std::bind(&PerceptionNode::right_callback, this,
                  std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10,
        std::bind(&PerceptionNode::imu_callback, this, std::placeholders::_1));
  }

private:
  void left_callback(const sensor_msgs::msg::Image::SharedPtr left_msg) {
    RCLCPP_INFO(this->get_logger(), "Received left image at address: %p",
                static_cast<const void *>(left_msg.get()));
    latest_left_msg_ = left_msg;
  }
  void right_callback(const sensor_msgs::msg::Image::SharedPtr right_msg) {
    latest_right_msg_ = right_msg;
    RCLCPP_INFO(this->get_logger(), "Received right image at address: %p",
                static_cast<const void *>(latest_right_msg_.get()));
    stereo_callback();
  }
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    (void)imu_msg;
    // Publish 200Hz odometry from IMU callback
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = 0.1;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_200hz_pub_->publish(odom_msg);
  }
  void stereo_callback() {
    // Create disparity image
    if (!latest_left_msg_ || !latest_right_msg_)
      return;
    RCLCPP_INFO(this->get_logger(),
                "Processing stereo images at addresses: left %p, right %p",
                static_cast<const void *>(latest_left_msg_.get()),
                static_cast<const void *>(latest_right_msg_.get()));
    auto disparity_msg = std::make_unique<sensor_msgs::msg::Image>();
    disparity_msg->header.stamp = latest_left_msg_->header.stamp;
    disparity_msg->header.frame_id = "disparity_frame";
    disparity_msg->height = 720;
    disparity_msg->width = 1280;
    disparity_msg->encoding = "mono8";
    disparity_msg->is_bigendian = false;
    disparity_msg->step = 1280;
    disparity_msg->data.resize(1280 * 720, 64);
    RCLCPP_INFO(this->get_logger(), "Published disparity image at address: %p",
                static_cast<const void *>(disparity_msg.get()));
    disparity_pub_->publish(std::move(disparity_msg));

    // Create odometry
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = latest_right_msg_->header.stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = 0.1;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;

    odom_20hz_pub_->publish(odom_msg);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_20hz_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_200hz_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;
};

// Planning Node
class PlanningNode : public rclcpp::Node {
public:
  PlanningNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("planning_node", options) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/rate_20hz", 10,
        std::bind(&PlanningNode::odom_callback, this, std::placeholders::_1));
    disparity_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "disparity", 10,
        std::bind(&PlanningNode::disparity_callback, this,
                  std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = msg;
  }

  void disparity_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    latest_disparity_ = msg;
    RCLCPP_INFO(this->get_logger(), "Received disparity image at address: %p",
                static_cast<const void *>(latest_disparity_.get()));
    process_planning();
  }

  void process_planning() {
    if (!latest_odom_ || !latest_disparity_)
      return;

    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.stamp = latest_odom_->header.stamp;
    path_msg.header.frame_id = "map";

    path_pub_->publish(path_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr disparity_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  sensor_msgs::msg::Image::SharedPtr latest_disparity_;
};

// Control Node
class ControlNode : public rclcpp::Node {
public:
  ControlNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("control_node", options) {
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "path", 10,
        std::bind(&ControlNode::path_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/rate_200hz", 10,
        std::bind(&ControlNode::odom_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    latest_path_ = msg;
    control_callback();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = msg;
  }

  void control_callback() {
    if (!latest_path_ || !latest_odom_)
      return;

    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = 0.5;
    cmd_msg.angular.z = 0.1;
    cmd_pub_->publish(cmd_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  nav_msgs::msg::Path::SharedPtr latest_path_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create node options with intra-process communication enabled
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  // Create all nodes with intra-process communication
  auto camera_node = std::make_shared<CameraNode>(options);
  auto imu_node = std::make_shared<ImuNode>(options);
  auto perception_node = std::make_shared<PerceptionNode>(options);
  auto planning_node = std::make_shared<PlanningNode>(options);
  auto control_node = std::make_shared<ControlNode>(options);

  // Create multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 8); // 8 threads for all nodes

  // Add all nodes to executor
  executor.add_node(camera_node);
  executor.add_node(imu_node);
  executor.add_node(perception_node);
  executor.add_node(planning_node);
  executor.add_node(control_node);

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "All nodes started with multi-threaded executor and "
              "intra-process communication");
  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Nodes: Camera (20Hz), IMU (200Hz), Perception (20Hz), Planning "
              "(20Hz), Control (200Hz)");

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
