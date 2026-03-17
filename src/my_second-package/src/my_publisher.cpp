#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>


using namespace std::chrono_literals;


class TurtlesimMove : public rclcpp::Node
{
  public:
    TurtlesimMove()
    : Node("turtlesim_publisher"), count_(0)
    {
      auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos_profile);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&TurtlesimMove::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 2.0;
      message.angular.z = 2.0;
      RCLCPP_INFO(this->get_logger(), "move");
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlesimMove>());
  rclcpp::shutdown();
  return 0;
}