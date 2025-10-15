#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class PlanningNode : public rclcpp::Node {
public:
  PlanningNode() : Node("planning_node") {
    odom_sub_.subscribe(this, "odometry/rate_20hz");
    disparity_sub_.subscribe(this, "disparity");

    sync_ = std::make_shared<message_filters::TimeSynchronizer<
        nav_msgs::msg::Odometry, sensor_msgs::msg::Image>>(odom_sub_,
                                                           disparity_sub_, 10);
    sync_->registerCallback(std::bind(&PlanningNode::planning_callback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  }

private:
  void planning_callback(
      const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr &disparity_msg) {
    (void)disparity_msg;

    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.stamp = odom_msg->header.stamp;
    path_msg.header.frame_id = "map";

    path_pub_->publish(path_msg);
  }

  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> disparity_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry,
                                                    sensor_msgs::msg::Image>>
      sync_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningNode>());
  rclcpp::shutdown();
  return 0;
}