#ifndef MOMENT_NAVIGATION__GO_TO_GOAL_HPP_
#define MOMENT_NAVIGATION__GO_TO_GOAL_HPP_

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "moment_interfaces/action/go_to_goal.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"


namespace moment_navigation
{
    class GoToGoalServer : public rclcpp::Node
    {
        public:
            explicit GoToGoalServer();

            using GoToGoal = moment_interfaces::action::GoToGoal;
            using GoToGoalHandle = rclcpp_action::ServerGoalHandle<GoToGoal>;

            using NavigateToPose = nav2_msgs::action::NavigateToPose;
            using NavigateToPoseHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        private:
            // Custom action server: receives goals from other MOMENT nodes
            rclcpp_action::Server<GoToGoal>::SharedPtr action_server_;

            // Nav2 action client: forwards navigation goals to Nav2
            rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;

    };
} // namespace moment_navigation

#endif // MOMENT_NAVIGATION__GO_TO_GOAL_HPP_