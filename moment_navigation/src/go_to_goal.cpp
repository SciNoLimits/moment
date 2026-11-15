#include "moment_navigation/go_to_goal.hpp"

#include <functional>
#include <memory>
#include <string>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::placeholders;

namespace moment_navigation
{
    GoToGoalServer::GoToGoalServer() : Node("go_to_goal_server")
    {

        // Nav2 action client
        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose");

        // Custom action server
        action_server_ = rclcpp_action::create_server<GoToGoal>(
            this,
            "go_to_goal",
            std::bind(&GoToGoalServer::handle_goal, this, _1, _2),
            std::bind(&GoToGoalServer::handle_cancel, this, _1),
            std::bind(&GoToGoalServer::handle_accepted, this, _1));

        RCLCPP_INFO(
            this->get_logger(),
            "GoTOGoal Action Server is ready.");
    }

    rclcpp_action::GoalResponse GoToGoalServer::handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToGoal::Goal> goal)
    {

        (void)uuid;

        std::lock_guard<std::mutex> lock(goal_mutex_);

        const auto & orientation = goal->target_pose.pose.orientation;

        tf2::Quaternion q(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w  
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Reject if another navigation goal is active
        if (active_goal_handle_ != nullptr)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Another navigation goal is already active. Rejecting new goal."
            );

            return rclcpp_action::GoalResponse::REJECT;

        }

        RCLCPP_INFO(
            this->get_logger(),
            "Received navigation goal: x=%.2f, y=%.2, yaw=%.2f rad (%.2f deg)",
            goal->target_pose.pose.position.x,
            goal->target_pose.pose.position.y,
            yaw,
            yaw * 180.0 / M_PI
        );
    }
} // namespace moment_navigation