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

bool valid_position(const PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  return !pose.header.frame_id.empty() &&
         std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
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
    approach_distance_ = parameter("approach_distance", 0.03);
    use_pregrasp_ = parameter("use_pregrasp", false);
    planning_time_ = parameter("planning_time", 5.0);
    position_tolerance_ = parameter("position_tolerance", 0.002);
    grasp_z_offset_ = parameter("grasp_z_offset", 0.0);
    const auto velocity = parameter("velocity_scaling", 0.15);
    const auto acceleration = parameter("acceleration_scaling", 0.15);

    if (arm_group == hand_group || home_target_ == place_target_ ||
      open_target_ == closed_target_ || !std::isfinite(grasp_z_offset_) ||
      !std::isfinite(approach_distance_) || approach_distance_ <= 0.0 ||
      !std::isfinite(planning_time_) || planning_time_ <= 0.0 ||
      !std::isfinite(position_tolerance_) || position_tolerance_ <= 0.0 ||
      !(velocity > 0.0 && velocity <= 1.0) ||
      !(acceleration > 0.0 && acceleration <= 1.0))
    {
      throw std::runtime_error("Invalid planning groups, target names, motion distances, or scaling");
    }
    arm_ = std::make_unique<MoveGroup>(node_, arm_group, nullptr, rclcpp::Duration::from_seconds(10));
    hand_ = std::make_unique<MoveGroup>(node_, hand_group, nullptr, rclcpp::Duration::from_seconds(10));
    if (!arm_->setEndEffectorLink(tcp_link)) {
      throw std::runtime_error("Could not select TCP link '" + tcp_link + "'");
    }
    arm_->setPoseReferenceFrame(arm_->getPlanningFrame());
    arm_->setPlanningPipelineId("ompl");
    arm_->setPlanningTime(planning_time_);
    arm_->setGoalPositionTolerance(position_tolerance_);
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
        if (!valid_position(goal->object_pose)) {
          RCLCPP_WARN(node_->get_logger(), "Pick requires a nonempty frame and finite XYZ; orientation is ignored");
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

    RCLCPP_INFO(node_->get_logger(), "Position-only pick and place servers ready; planning frame: %s",
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

  // Every arm XYZ motion uses OMPL with a position-only goal. No Cartesian
  // path interpolation, fixed quaternion, manual IK, or angle sampling.
  void move_to_pose(const PoseStamped & input)
  {
    if (!valid_position(input)) {
      throw std::runtime_error("Invalid position target");
    }
    auto target = input;
    // PoseStamped remains the wire format; use a valid placeholder for TF only.
    target.pose.orientation = geometry_msgs::msg::Quaternion();
    target.pose.orientation.w = 1.0;
    const auto frame = arm_->getPlanningFrame();
    if (target.header.frame_id != frame) {
      target = tf_buffer_.transform(target, frame, tf2::durationFromSec(0.5));
    }
    arm_->clearPoseTargets();
    arm_->clearPathConstraints();
    arm_->clearTrajectoryConstraints();
    arm_->setPoseReferenceFrame(frame);
    arm_->setStartStateToCurrentState();
    const auto & p = target.pose.position;
    RCLCPP_INFO(node_->get_logger(),
      "POSITION-ONLY target input_frame=%s planning_frame=%s tcp=%s "
      "xyz=[%.6f %.6f %.6f]; orientation unconstrained",
      input.header.frame_id.c_str(), frame.c_str(),
      arm_->getEndEffectorLink().c_str(), p.x, p.y, p.z);
    if (!arm_->setPositionTarget(p.x, p.y, p.z, arm_->getEndEffectorLink())) {
      throw std::runtime_error("Could not set position-only TCP target");
    }
    try {
      plan_and_execute(*arm_);
    } catch (...) {
      arm_->clearPoseTargets();
      throw;
    }
    arm_->clearPoseTargets();
    const auto actual = arm_->getCurrentPose();
    RCLCPP_INFO(node_->get_logger(),
      "Reached TCP frame=%s xyz=[%.6f %.6f %.6f] "
      "chosen quaternion xyzw=[%.6f %.6f %.6f %.6f]",
      actual.header.frame_id.c_str(), actual.pose.position.x,
      actual.pose.position.y, actual.pose.position.z,
      actual.pose.orientation.x, actual.pose.orientation.y,
      actual.pose.orientation.z, actual.pose.orientation.w);
  }

  std::string pick(const std::shared_ptr<GoalHandle<Pick>> & goal, std::string & stage)
  {
    feedback<Pick>(goal, stage, "transforming_object_position");
    auto grasp = goal->get_goal()->object_pose;
    grasp.pose.orientation = geometry_msgs::msg::Quaternion();
    grasp.pose.orientation.w = 1.0;
    const auto frame = arm_->getPlanningFrame();
    if (grasp.header.frame_id != frame) {
      grasp = tf_buffer_.transform(grasp, frame, tf2::durationFromSec(0.5));
    }
    // Offset is expressed along planning-frame Z, in metres.
    grasp.pose.position.z += grasp_z_offset_;
    RCLCPP_INFO(node_->get_logger(),
      "Pick position-only mode: grasp_z_offset=%.4f use_pregrasp=%s",
      grasp_z_offset_, use_pregrasp_ ? "true" : "false");

    feedback<Pick>(goal, stage, "opening_gripper");
    move_to_named_target(*hand_, open_target_);
    if (use_pregrasp_) {
      auto above = grasp;
      above.pose.position.z += approach_distance_;
      feedback<Pick>(goal, stage, "moving_to_pregrasp");
      move_to_pose(above);
    }
    feedback<Pick>(goal, stage, "approaching_object");
    move_to_pose(grasp);
    feedback<Pick>(goal, stage, "closing_gripper");
    move_to_named_target(*hand_, closed_target_);
    feedback<Pick>(goal, stage, "returning_home");
    try {
      move_to_named_target(*arm_, home_target_);
    } catch (const std::exception & error) {
      throw std::runtime_error(std::string("not able to go home; ") + error.what());
    }
    return "Position-only pick motion completed; grasp orientation and object retention were not verified";
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
  bool use_pregrasp_;
  double approach_distance_, grasp_z_offset_, planning_time_, position_tolerance_;
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