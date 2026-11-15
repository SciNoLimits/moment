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

            // ACTION SERVER AND CLIENT
            // Custom action server: receives goals from other MOMENT nodes
            rclcpp_action::Server<GoToGoal>::SharedPtr action_server_;

            // Nav2 action client: forwards navigation goals to Nav2
            rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;


            // ACTION SERVER CALLBACKS
            // Called when a new goal arrives
            rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const GoToGoal::Goal> goal
            );

            // Called when the client requests cancellation
            rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoToGoalHandle> goal_handle
            );

            // Called after a goal has been accepted
            void handle_accepted(
                const std::shared_ptr<GoToGoalHandle> goal_handle
            );


            // NAV2 COMMUNICATION CALLBACKS
            // Send the accepted goal to Nav2
            void send_navigation_goal(
                const std::shared_ptr<GoToGoalHandle> goal_handle
            );

            // Receive feedback from Nav2
            void nav2_feedback_callback(
                NavigateToPoseHandle::SharedPtr nav2_goal_handle,
                const std::shared_ptr<const NavigateToPose::Feedback> feedback
            );

            // Receive the final result from Nav2
            void nav2_result_callback(
                const NavigateToPoseHandle::WrappedResult & result
            );

            // Receive the response from Nav2
            void nav2_goal_response_callback(
                const NavigateToPoseHandle::SharedPtr & nav2_goal_handle

            );


            // GOAL TRACKING
            // Track the currently active MOMENT goal
            std::shared_ptr<GoToGoalHandle> active_goal_handle_;

            // Track the corresponding Nav2 goal
            NavigateToPoseHandle::SharedPtr nav2_goal_handle_;

            // Protect shared goal state
            std::mutex goal_mutex_;

    };
} // namespace moment_navigation

#endif // MOMENT_NAVIGATION__GO_TO_GOAL_HPP_