#include "moment_navigation/go_to_goal.hpp"

#include <functional>
#include <memory>
#include <string>

using namespace std::placeholders;

namespace moment_navigation{
    GoToGoalServer::GoToGoalServer(): Node("go_to_goal_server"){
        
        // Nav2 action client
        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose"
        );

        // Custom action server
        action_server_ = rclcpp_action::create_server<GoToGoal>(
            this,
            "go_to_goal",
            std::bind(&GoToGoalServer::handle_goal, this, _1, _2),
            std::bind(&GoToGoalServer::handle_cancel, this, _1),
            std::bind(&GoToGoalServer::handle_accepted, this, _1)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "GoTOGoal Action Server is ready."
        );
    }
} // namespace moment_navigation