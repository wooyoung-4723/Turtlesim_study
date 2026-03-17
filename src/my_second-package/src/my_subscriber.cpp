#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtlesimSubscriber : public rclcpp::Node
{
public:
    TurtlesimSubscriber()
    : Node("turtlesim_subscriber")
    {
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&TurtlesimSubscriber::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const turtlesim::msg::Pose::SharedPtr msg) const
    {
        std::cout << "X : " << msg->x << " Y : " << msg->y << std::endl;
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlesimSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}