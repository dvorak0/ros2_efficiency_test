#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include "shm_msgs/msg/image1m.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


class PlanningNode : public rclcpp::Node {
public:
  PlanningNode() : Node("planning_node") {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    odom_sub_.subscribe(this, "odometry/rate_20hz");
    // subscribe odom_msg
    disparity_sub_.subscribe(this, "disparity");
    sync_ = std::make_shared<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, shm_msgs::msg::Image1m>>(odom_sub_,disparity_sub_, 10);
    sync_->registerCallback(std::bind(&PlanningNode::planning_callback, this, std::placeholders::_1, std::placeholders::_2));
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  }

private:
  void planning_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
    const shm_msgs::msg::Image1m::ConstSharedPtr &disparity_msg) {
    (void)disparity_msg;
    // Path publish
    auto loaned_msg = path_pub_->borrow_loaned_message();
    auto &path_msg = loaned_msg.get();
    path_msg.header.stamp = odom_msg->header.stamp;
    path_msg.header.frame_id = "map";
    path_pub_->publish(std::move(loaned_msg));
  }
  
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  message_filters::Subscriber<shm_msgs::msg::Image1m> disparity_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry,shm_msgs::msg::Image1m>>sync_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningNode>());
  rclcpp::shutdown();
  return 0;
}
