#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "my_second_package/action/dis_turtle.hpp"

using namespace std::placeholders;

class DistTurtleActionServer : public rclcpp::Node
{
public:
    using DisTurtle = my_second_package::action::DisTurtle;
    using GoalHandleDisTurtle = rclcpp_action::ServerGoalHandle<DisTurtle>;

    DistTurtleActionServer()
    : Node("dist_turtle_action_server"),
      total_dist_(0.0),
      is_first_time_(true),
      quantile_time_(0.75),
      almost_goal_time_(0.95)
    {
        current_pose_ = turtlesim::msg::Pose();
        previous_pose_ = turtlesim::msg::Pose();

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&DistTurtleActionServer::pose_callback, this, _1)
        );

        action_server_ = rclcpp_action::create_server<DisTurtle>(
            this,
            "dist_turtle",
            std::bind(&DistTurtleActionServer::handle_goal, this, _1, _2),
            std::bind(&DistTurtleActionServer::handle_cancel, this, _1),
            std::bind(&DistTurtleActionServer::handle_accepted, this, _1)
        );

        this->declare_parameter("quatile_time", 0.75);
        this->declare_parameter("almost_goal_time", 0.95);

        this->get_parameter("quatile_time", quantile_time_);
        this->get_parameter("almost_goal_time", almost_goal_time_);

        RCLCPP_INFO(this->get_logger(), "Dist turtle action server is started.");
        RCLCPP_INFO(
            this->get_logger(),
            "quatile_time: %.2f, almost_goal_time: %.2f",
            quantile_time_,
            almost_goal_time_
        );

        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DistTurtleActionServer::parameter_callback, this, _1)
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp_action::Server<DisTurtle>::SharedPtr action_server_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    double total_dist_;
    bool is_first_time_;
    double quantile_time_;
    double almost_goal_time_;

    turtlesim::msg::Pose current_pose_;
    turtlesim::msg::Pose previous_pose_;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
    }

    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> & params)
    {
        for (const auto & param : params) {
            RCLCPP_INFO(
                this->get_logger(),
                "%s changed to %s",
                param.get_name().c_str(),
                param.value_to_string().c_str()
            );

            if (param.get_name() == "quatile_time") {
                quantile_time_ = param.as_double();
            }

            if (param.get_name() == "almost_goal_time") {
                almost_goal_time_ = param.as_double();
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Current params -> quatile_time: %.2f, almost_goal_time: %.2f",
            quantile_time_,
            almost_goal_time_
        );

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    double calc_diff_pose()
    {
        if (is_first_time_) {
            previous_pose_.x = current_pose_.x;
            previous_pose_.y = current_pose_.y;
            is_first_time_ = false;
        }

        double diff_dist = std::sqrt(
            std::pow(current_pose_.x - previous_pose_.x, 2) +
            std::pow(current_pose_.y - previous_pose_.y, 2)
        );

        previous_pose_ = current_pose_;
        return diff_dist;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const DisTurtle::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(
            this->get_logger(),
            "Received goal -> linear_x: %.2f, angular_z: %.2f, dist: %.2f",
            goal->linear_x,
            goal->angular_z,
            goal->dist
        );

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDisTurtle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDisTurtle> goal_handle)
    {
        std::thread{std::bind(&DistTurtleActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleDisTurtle> goal_handle)
    {
        auto feedback = std::make_shared<DisTurtle::Feedback>();
        auto result = std::make_shared<DisTurtle::Result>();

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = goal_handle->get_goal()->linear_x;
        twist_msg.angular.z = goal_handle->get_goal()->angular_z;

        rclcpp::Rate loop_rate(100);

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                geometry_msgs::msg::Twist stop_msg;
                publisher_->publish(stop_msg);

                result->pos_x = current_pose_.x;
                result->pos_y = current_pose_.y;
                result->pos_theta = current_pose_.theta;
                result->result_dist = total_dist_;

                goal_handle->canceled(result);

                reset_state();
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            total_dist_ += calc_diff_pose();

            feedback->remained_dist = goal_handle->get_goal()->dist - total_dist_;
            goal_handle->publish_feedback(feedback);

            publisher_->publish(twist_msg);

            double tmp = feedback->remained_dist - goal_handle->get_goal()->dist + quantile_time_;
            tmp = std::abs(tmp);

            if (tmp < 0.02) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "The turtle passes the %.2f point. diff=%.5f",
                    quantile_time_,
                    tmp
                );
            }

            if (feedback->remained_dist < 0.2) {
                break;
            }

            loop_rate.sleep();
        }

        geometry_msgs::msg::Twist stop_msg;
        publisher_->publish(stop_msg);

        result->pos_x = current_pose_.x;
        result->pos_y = current_pose_.y;
        result->pos_theta = current_pose_.theta;
        result->result_dist = total_dist_;

        goal_handle->succeed(result);

        RCLCPP_INFO(
            this->get_logger(),
            "Goal succeeded -> x: %.2f, y: %.2f, theta: %.2f, dist: %.2f",
            result->pos_x,
            result->pos_y,
            result->pos_theta,
            result->result_dist
        );

        reset_state();
    }

    void reset_state()
    {
        total_dist_ = 0.0;
        is_first_time_ = true;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DistTurtleActionServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}