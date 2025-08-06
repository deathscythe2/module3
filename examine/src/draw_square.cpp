#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SquareDrawer : public rclcpp::Node
{
public:
  SquareDrawer()
  : Node("square_drawer"), step_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&SquareDrawer::draw_square, this));
    start_time_ = now();
    RCLCPP_INFO(this->get_logger(), "Turtle is drawing a square...");
  }

private:
  void draw_square()
  
  {
    rclcpp::Time current_time = now();
    rclcpp::Duration elapsed = current_time - start_time_;
    geometry_msgs::msg::Twist msg;
  
    if (step_ >= 8) {
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
      publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Finished drawing square.");
      rclcpp::shutdown();
      return;
    }
  
    if (step_ % 2 == 0) {
      
      msg.linear.x = 1.0;
      msg.angular.z = 0.0;
      if (elapsed.seconds() >= 2.0) {
        step_++;
        start_time_ = current_time;
      }
    } else {
      
      msg.linear.x = 0.0;
      msg.angular.z = 0.785;  
      if (elapsed.seconds() >= 2.0) { 
        step_++;
        start_time_ = current_time;
      }
    }
  
    publisher_->publish(msg);
  }
  
 
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  int step_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SquareDrawer>());
  rclcpp::shutdown();
  return 0;
}
