#include "moment_navigation/go_to_goal.hpp"

#include <functional>
#include <memory>
#include <string>
#include <chrono>

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

    // Handle incoming goals

    rclcpp_action::GoalResponse GoToGoalServer::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const GoToGoal::Goal> goal)
    {

        (void)uuid;

        std::lock_guard<std::mutex> lock(goal_mutex_);

        const auto &orientation = goal->target_pose.pose.orientation;

        tf2::Quaternion q(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Reject if another navigation goal is active
        if (active_goal_handle_ != nullptr)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Another navigation goal is already active. Rejecting new goal.");

            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Received navigation goal: x=%.2f, y=%.2, yaw=%.2f rad (%.2f deg)",
            goal->target_pose.pose.position.x,
            goal->target_pose.pose.position.y,
            yaw,
            yaw * 180.0 / M_PI);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle cancellation requests

    rclcpp_action::CancelResponse GoToGoalServer::handle_cancel(
        const std::shared_ptr<GoToGoalHandle> goal_handle)
    {
        NavigateToPoseHandle::SharedPtr nav2_handle;

        {
            std::lock_guard<std::mutex> lock(goal_mutex_);

            if (active_goal_handle_ != goal_handle)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Cancellation requested for an inactive goal.");

                return rclcpp_action::CancelResponse::REJECT;
            }

            cancel_requested_ = true;
            nav2_handle = nav2_goal_handle_;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Cancellation requested for the active navigation goal.");

        // If Nav2 has already accepted the goal, cancel it.
        // Otherwise, cancellation will be forwarded later.
        if (nav2_handle)
        {
            nav2_client_->async_cancel_goal(nav2_handle);
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void GoToGoalServer::handle_accepted(
        const std::shared_ptr<GoToGoalHandle> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            active_goal_handle_ = goal_handle;
        }

        // Forward the accepted goal to Nav2
        send_navigation_goal(goal_handle);
    }

    void GoToGoalServer::send_navigation_goal(
        const std::shared_ptr<GoToGoalHandle> goal_handle)
    {
        // Check weather Nav2 is available
        if (!nav2_client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(
                this->get_logger(),

                "Nav2 NavigateToPose action server is not available.");

            GoToGoal::Result result;
            result.success = false;
            result.message = "Nav2 action server is unavailable.";

            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                active_goal_handle_.reset();
            }

            goal_handle->abort(
                std::make_shared<GoToGoal::Result>(result));

            return;
        }

        // Prepare Nav2 goal
        NavigateToPose::Goal nav2_goal;
        nav2_goal.pose = goal_handle->get_goal()->target_pose;

        // Configure callbacks
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

        options.goal_response_callback = std::bind(
            &GoToGoalServer::nav2_goal_response_callback,
            this,
            _1);

        options.feedback_callback = std::bind(
            &GoToGoalServer::nav2_feedback_callback,
            this,
            _1,
            _2);

        options.result_callback = std::bind(
            &GoToGoalServer::nav2_result_callback,
            this,
            _1);

        // Send goal to Nav2
        nav2_client_->async_send_goal(nav2_goal, options);

        RCLCPP_INFO(
            this->get_logger(),
            "Navigation goal forwarded to Nav2.");
    }

    void GoToGoalServer::nav2_goal_response_callback(
        const NavigateToPoseHandle::SharedPtr &nav2_goal_handle)
    {
        if (!nav2_goal_handle)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Nav2 rejected the navigation goal.");

            std::shared_ptr<GoToGoalHandle> goal_handle;

            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                goal_handle = active_goal_handle_;
                active_goal_handle_.reset();
            }

            if (goal_handle && goal_handle->is_active())
            {
                auto result = std::make_shared<GoToGoal::Result>();
                result->success = false;
                result->message = "Nav2 rejected the navigation goal.";

                goal_handle->abort(result);
            }

            return;
        }

        bool should_cancel = false;

        {
            std::lock_guard<std::mutex> lock(goal_mutex_);

            nav2_goal_handle_ = nav2_goal_handle;
            should_cancel = cancel_requested_;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Nav2 accepted the navigation goal.");

        if (should_cancel)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Forwarding pending cancellation request to Nav2.");

            nav2_client_->async_cancel_goal(nav2_goal_handle);
        }
    }

    void GoToGoalServer::nav2_feedback_callback(
        NavigateToPoseHandle::SharedPtr nav2_goal_handle,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        (void)nav2_goal_handle;

        std::shared_ptr<GoToGoalHandle> goal_handle;

        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_handle = active_goal_handle_;
        }

        if (!goal_handle || !goal_handle->is_active())
        {
            return;
        }

        auto feedback_msg = std::make_shared<GoToGoal::Feedback>();

        feedback_msg->distance_remaining = feedback->distance_remaining;

        goal_handle->publish_feedback(feedback_msg);
    }

    void GoToGoalServer::nav2_result_callback(
        const NavigateToPoseHandle::WrappedResult &result)
    {
        std::shared_ptr<GoToGoalHandle> goal_handle;

        {
            std::lock_guard<std::mutex> lock(goal_mutex_);

            goal_handle = active_goal_handle_;

            active_goal_handle_.reset();
            nav2_goal_handle_.reset();
            cancel_requested_ = false;
        }

        if (!goal_handle || !goal_handle->is_active())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Received Nav2 result, but the MOMENT goal is no longer active.");

            return;
        }

        auto result_msg = std::make_shared<GoToGoal::Result>();

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:

            result_msg->success = true;
            result_msg->message = "Navigation completed successfully.";

            goal_handle->succeed(result_msg);

            RCLCPP_INFO(
                this->get_logger(),
                "Navigation completed successfully.");

            break;

        case rclcpp_action::ResultCode::CANCELED:

            result_msg->success = false;
            result_msg->message = "Navigation was canceled.";

            goal_handle->canceled(result_msg);

            RCLCPP_INFO(
                this->get_logger(),
                "Navigation was canceled.");

            break;

        case rclcpp_action::ResultCode::ABORTED:

            result_msg->success = false;
            result_msg->message = "Navigation failed. Nav2 aborted the goal.";

            goal_handle->abort(result_msg);

            RCLCPP_ERROR(
                this->get_logger(),
                "Navigation failed. Nav2 aborted the goal.");

            break;

        default:

            result_msg->success = false;
            result_msg->message = "Navigation returned an unknown result.";

            goal_handle->abort(result_msg);

            RCLCPP_ERROR(
                this->get_logger(),
                "Navigation returned an unknown result.");

            break;
        }
    }

} // namespace moment_navigation