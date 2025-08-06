#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("broadcaster_subscriber") // Set node name
  {
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    // Subscribe to "topic" with QoS 10, and call topic_callback when a message arrives
  }

private:
  void topic_callback(const turtlesim::msg::Pose &msg) const
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I got x: '" << msg.x << "', y: '" << msg.y << "', theta: '" << msg.theta << "'");
    // Log the received message
  }
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}