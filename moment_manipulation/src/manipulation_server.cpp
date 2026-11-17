#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moment_interfaces/action/pick.hpp>
#include <moment_interfaces/action/place.hpp>
#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
#include <moveit/move_group_interface/move_group_interface.hpp>
#else
#include <moveit/move_group_interface/move_group_interface.h>
#endif
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using Pick = moment_interfaces::action::Pick;
using Place = moment_interfaces::action::Place;
using PoseStamped = geometry_msgs::msg::PoseStamped;

template<typename Action>
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

bool valid_orientation(const geometry_msgs::msg::Quaternion & q)
{
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  return std::isfinite(norm) && std::abs(norm - 1.0) < 1e-3;
}

bool valid_pose(const PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  return !pose.header.frame_id.empty() &&
         std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         valid_orientation(pose.pose.orientation);
}

void require_target(MoveGroup & group, const std::string & target)
{
  const auto names = group.getNamedTargets();
  if (std::find(names.begin(), names.end(), target) == names.end()) {
    throw std::runtime_error("Missing SRDF target '" + target + "' in group '" +
      group.getName() + "'");
  }
}

void plan_and_execute(MoveGroup & group)
{
  MoveGroup::Plan plan;
  const auto planned = group.plan(plan);
  if (planned.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    throw std::runtime_error("planning failed (MoveIt code " + std::to_string(planned.val) + ")");
  }

  const auto executed = group.execute(plan);
  if (executed.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    throw std::runtime_error(
      "execution failed (MoveIt code " + std::to_string(executed.val) + ")");
  }
}

void move_to_named_target(MoveGroup & group, const std::string & target)
{
  group.clearPoseTargets();
  group.setStartStateToCurrentState();
  if (!group.setNamedTarget(target)) {
    throw std::runtime_error("could not select named target '" + target + "'");
  }
  plan_and_execute(group);
}

class ManipulationServer
{
public:
  explicit ManipulationServer(const rclcpp::Node::SharedPtr & node)
  : node_(node), tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
  {
    const auto arm_group = parameter<std::string>("arm_group", "moment_arm");
    const auto hand_group = parameter<std::string>("hand_group", "moment_gripper");
    const auto tcp_link = parameter<std::string>("tcp_link", "end_effector_link");
    home_target_ = parameter<std::string>("home_target", "home");
    place_target_ = parameter<std::string>("place_target", "place");
    open_target_ = parameter<std::string>("open_target", "open");
    closed_target_ = parameter<std::string>("closed_target", "close");
    approach_distance_ = parameter("approach_distance", 0.10);
    grasp_z_offset_ = parameter("grasp_z_offset", 0.0);
    cartesian_step_ = parameter("cartesian_step", 0.005);
    grasp_orientation_ = parameter("grasp_orientation_xyzw", std::vector<double>{});
    const auto velocity = parameter("velocity_scaling", 0.15);
    const auto acceleration = parameter("acceleration_scaling", 0.15);

    if (arm_group == hand_group || home_target_ == place_target_ ||
      open_target_ == closed_target_ || !std::isfinite(grasp_z_offset_) ||
      !std::isfinite(approach_distance_) || approach_distance_ <= 0.0 ||
      !std::isfinite(cartesian_step_) || cartesian_step_ <= 0.0 ||
      !(velocity > 0.0 && velocity <= 1.0) ||
      !(acceleration > 0.0 && acceleration <= 1.0))
    {
      throw std::runtime_error("Invalid planning groups, target names, motion distances, or scaling");
    }
    if (!grasp_orientation_.empty()) {
      if (grasp_orientation_.size() != 4) {
        throw std::runtime_error("grasp_orientation_xyzw must contain four values");
      }
      fixed_orientation_.x = grasp_orientation_[0];
      fixed_orientation_.y = grasp_orientation_[1];
      fixed_orientation_.z = grasp_orientation_[2];
      fixed_orientation_.w = grasp_orientation_[3];
      if (!valid_orientation(fixed_orientation_)) {
        throw std::runtime_error("grasp_orientation_xyzw must be a unit quaternion");
      }
    }

    arm_ = std::make_unique<MoveGroup>(node_, arm_group, nullptr, rclcpp::Duration::from_seconds(10));
    hand_ = std::make_unique<MoveGroup>(node_, hand_group, nullptr, rclcpp::Duration::from_seconds(10));
    if (!arm_->setEndEffectorLink(tcp_link)) {
      throw std::runtime_error("Could not select TCP link '" + tcp_link + "'");
    }
    arm_->setPoseReferenceFrame(arm_->getPlanningFrame());
    for (auto * group : {arm_.get(), hand_.get()}) {
      group->setMaxVelocityScalingFactor(velocity);
      group->setMaxAccelerationScalingFactor(acceleration);
    }
    require_target(*arm_, home_target_);
    require_target(*arm_, place_target_);
    require_target(*hand_, open_target_);
    require_target(*hand_, closed_target_);
  }

  ~ManipulationServer()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void start()
  {
    pick_server_ = rclcpp_action::create_server<Pick>(
      node_, "pick",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Pick::Goal> goal) {
        if (!valid_pose(goal->object_pose)) {
          RCLCPP_WARN(node_->get_logger(), "Pick requires a frame and a finite pose with a unit quaternion");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return reserve_goal();
      },
      [](const std::shared_ptr<GoalHandle<Pick>>) {return rclcpp_action::CancelResponse::REJECT;},
      [this](const std::shared_ptr<GoalHandle<Pick>> goal) {
        start_worker<Pick>(goal, [this, goal](std::string & stage) {
          return pick(goal, stage);
        });
      });

    place_server_ = rclcpp_action::create_server<Place>(
      node_, "place",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Place::Goal>) {
        return reserve_goal();
      },
      [](const std::shared_ptr<GoalHandle<Place>>) {return rclcpp_action::CancelResponse::REJECT;},
      [this](const std::shared_ptr<GoalHandle<Place>> goal) {
        start_worker<Place>(goal, [this, goal](std::string & stage) {
          return place(goal, stage);
        });
      });

    RCLCPP_INFO(node_->get_logger(), "Pick and place servers ready; planning frame: %s",
      arm_->getPlanningFrame().c_str());
  }

private:
  template<typename T>
  T parameter(const std::string & name, const T & fallback)
  {
    if (node_->has_parameter(name)) {
      return node_->get_parameter(name).get_value<T>();
    }
    return node_->declare_parameter<T>(name, fallback);
  }

  rclcpp_action::GoalResponse reserve_goal()
  {
    if (busy_.exchange(true)) {
      RCLCPP_WARN(node_->get_logger(), "Rejected goal while another manipulation goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  template<typename Action, typename Operation>
  void start_worker(const std::shared_ptr<GoalHandle<Action>> & goal, Operation operation)
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    try {
      worker_ = std::thread([this, goal, operation]() {
        auto result = std::make_shared<typename Action::Result>();
        std::string stage = "starting";
        try {
          result->message = operation(stage);
          result->success = true;
        } catch (const std::exception & error) {
          result->success = false;
          result->message = stage + ": " + error.what();
          RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
        }

        if (rclcpp::ok()) {
          if (result->success) {
            goal->succeed(result);
          } else {
            goal->abort(result);
          }
        }
        busy_.store(false);
      });
    } catch (const std::system_error & error) {
      auto result = std::make_shared<typename Action::Result>();
      result->success = false;
      result->message = error.what();
      goal->abort(result);
      busy_.store(false);
    }
  }

  template<typename Action>
  void feedback(
    const std::shared_ptr<GoalHandle<Action>> & goal, std::string & stage, const char * next)
  {
    stage = next;
    if (!rclcpp::ok()) {
      throw std::runtime_error("ROS is shutting down");
    }
    auto update = std::make_shared<typename Action::Feedback>();
    update->stage = stage;
    goal->publish_feedback(update);
  }

  void move_to_pose(const PoseStamped & pose)
  {
    arm_->setStartStateToCurrentState();
    if (!arm_->setPoseTarget(pose)) {
      throw std::runtime_error("could not select the TCP pose target");
    }
    plan_and_execute(*arm_);
  }

  void move_straight(const PoseStamped & pose)
  {
    arm_->setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::MoveItErrorCodes error;
    const double fraction = arm_->computeCartesianPath(
      {pose.pose}, cartesian_step_, trajectory, true, &error);
    if (error.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
      !(fraction >= 1.0) || trajectory.joint_trajectory.points.empty())
    {
      throw std::runtime_error("Cartesian path incomplete (fraction " + std::to_string(fraction) +
        ", MoveIt code " + std::to_string(error.val) + ")");
    }
    const auto executed = arm_->execute(trajectory);
    if (executed.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      throw std::runtime_error(
        "Cartesian execution failed (MoveIt code " + std::to_string(executed.val) + ")");
    }
  }

  std::string pick(const std::shared_ptr<GoalHandle<Pick>> & goal, std::string & stage)
  {
    feedback<Pick>(goal, stage, "transforming_object_pose");
    auto grasp = goal->get_goal()->object_pose;
    const auto frame = arm_->getPlanningFrame();
    if (grasp.header.frame_id != frame) {
      grasp = tf_buffer_.transform(grasp, frame, tf2::durationFromSec(0.5));
    }
    grasp.pose.position.z += grasp_z_offset_;
    if (!grasp_orientation_.empty()) {
      grasp.pose.orientation = fixed_orientation_;
    }
    auto above = grasp;
    above.pose.position.z += approach_distance_;

    feedback<Pick>(goal, stage, "opening_gripper");
    move_to_named_target(*hand_, open_target_);
    feedback<Pick>(goal, stage, "moving_to_pregrasp");
    move_to_pose(above);
    feedback<Pick>(goal, stage, "approaching_object");
    move_straight(grasp);
    feedback<Pick>(goal, stage, "closing_gripper");
    move_to_named_target(*hand_, closed_target_);
    feedback<Pick>(goal, stage, "lifting_object");
    move_straight(above);
    return "Gripper close and lift completed; object retention was not measured";
  }

  std::string place(const std::shared_ptr<GoalHandle<Place>> & goal, std::string & stage)
  {
    feedback<Place>(goal, stage, "moving_to_place");
    move_to_named_target(*arm_, place_target_);
    feedback<Place>(goal, stage, "opening_gripper");
    move_to_named_target(*hand_, open_target_);
    feedback<Place>(goal, stage, "returning_home");
    try {
      move_to_named_target(*arm_, home_target_);
    } catch (const std::exception & error) {
      throw std::runtime_error(std::string("gripper already opened; ") + error.what());
    }
    feedback<Place>(goal, stage, "closing_gripper");
    move_to_named_target(*hand_, closed_target_);
    return "Gripper opened at the place target and the arm returned home";
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<MoveGroup> arm_;
  std::unique_ptr<MoveGroup> hand_;
  std::string home_target_, place_target_, open_target_, closed_target_;
  double approach_distance_, grasp_z_offset_, cartesian_step_;
  std::vector<double> grasp_orientation_;
  geometry_msgs::msg::Quaternion fixed_orientation_;
  rclcpp_action::Server<Pick>::SharedPtr pick_server_;
  rclcpp_action::Server<Place>::SharedPtr place_server_;
  std::atomic<bool> busy_{false};
  std::thread worker_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("manipulation_server", options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::unique_ptr<ManipulationServer> server;
  int exit_code = 0;

  try {
    server = std::make_unique<ManipulationServer>(node);
    server->start();
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  server.reset();
  return exit_code;
}
