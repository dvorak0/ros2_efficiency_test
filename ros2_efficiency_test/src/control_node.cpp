#include <geometry_msgs/msg/twist.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class ControlNode : public rclcpp::Node {
public:
  ControlNode() : Node("control_node") {
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "path", 10,
        std::bind(&ControlNode::path_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/rate_200hz", 10,
        std::bind(&ControlNode::odom_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&ControlNode::control_callback, this));
  }

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    latest_path_ = msg;
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
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path::SharedPtr latest_path_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}