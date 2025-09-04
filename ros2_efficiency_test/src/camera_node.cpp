#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("camera_node") {
    left_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/left/image_raw", 10);
    right_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/right/image_raw", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(50),
                                std::bind(&CameraNode::timer_callback, this));
  }

private:
  void timer_callback() {
    auto now = this->now();
    // print current time
    // RCLCPP_INFO(this->get_logger(), "Publishing images at time: %ld.%09ld",
    // now.seconds(), now.nanoseconds());

    auto left_msg = sensor_msgs::msg::Image();
    left_msg.header.stamp = now;
    left_msg.header.frame_id = "left_camera_frame";
    left_msg.height = 720;
    left_msg.width = 1280;
    left_msg.encoding = "mono8";
    left_msg.is_bigendian = false;
    left_msg.step = 1280;
    left_msg.data.resize(1280 * 720, 128);
    left_publisher_->publish(left_msg);

    auto right_msg = sensor_msgs::msg::Image();
    right_msg.header.stamp = now;
    right_msg.header.frame_id = "right_camera_frame";
    right_msg.height = 720;
    right_msg.width = 1280;
    right_msg.encoding = "mono8";
    right_msg.is_bigendian = false;
    right_msg.step = 1280;
    right_msg.data.resize(1280 * 720, 128);
    right_publisher_->publish(right_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
