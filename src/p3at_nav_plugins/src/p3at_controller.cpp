#include "p3at_nav_plugins/p3at_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "angles/angles.h"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace p3at_nav_plugins
{

void P3ATController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Unable to lock lifecycle node in P3ATController::configure()");
  }

  plugin_name_ = std::move(name);
  logger_ = node_->get_logger();
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();

  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(1.25));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".slowdown_distance", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".goal_dist_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".goal_yaw_tolerance", rclcpp::ParameterValue(0.087));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".heading_hold_angle", rclcpp::ParameterValue(0.785));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".heading_gain", rclcpp::ParameterValue(1.8));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".align_gain", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".min_linear_speed", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(0.9));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".collision_lookahead_time", rclcpp::ParameterValue(1.5));

  node_->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node_->get_parameter(plugin_name_ + ".slowdown_distance", slowdown_distance_);
  node_->get_parameter(plugin_name_ + ".goal_dist_tolerance", goal_dist_tolerance_);
  node_->get_parameter(plugin_name_ + ".goal_yaw_tolerance", goal_yaw_tolerance_);
  node_->get_parameter(plugin_name_ + ".heading_hold_angle", heading_hold_angle_);
  node_->get_parameter(plugin_name_ + ".heading_gain", heading_gain_);
  node_->get_parameter(plugin_name_ + ".align_gain", align_gain_);
  node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  node_->get_parameter(plugin_name_ + ".min_linear_speed", min_linear_speed_);
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  node_->get_parameter(plugin_name_ + ".collision_lookahead_time", collision_lookahead_time_);
  nominal_max_linear_speed_ = max_linear_speed_;

  RCLCPP_INFO(logger_, "Configured P3AT controller plugin: %s", plugin_name_.c_str());
}

void P3ATController::cleanup()
{
  global_plan_.poses.clear();
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
}

void P3ATController::activate()
{
  RCLCPP_INFO(logger_, "Activated P3AT controller plugin.");
}

void P3ATController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivated P3AT controller plugin.");
}

void P3ATController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  canceled_ = false;
}

geometry_msgs::msg::TwistStamped P3ATController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();

  if (canceled_) {
    return cmd;
  }
  if (global_plan_.poses.empty()) {
    throw nav2_core::ControllerException("P3AT controller received an empty plan.");
  }

  const auto tracking_frame = pose.header.frame_id.empty() ?
    costmap_ros_->getGlobalFrameID() : pose.header.frame_id;
  auto working_plan = transformPlan(tracking_frame);
  if (working_plan.poses.empty()) {
    throw nav2_core::ControllerException("P3AT controller failed to transform the global plan.");
  }

  prunePlan(working_plan, pose);
  const auto goal_pose = working_plan.poses.back();
  const double goal_dist = distance2D(pose, goal_pose);
  const double current_yaw = tf2::getYaw(pose.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
  const double goal_yaw_error = normalizeAngle(goal_yaw - current_yaw);

  double linear_cmd = 0.0;
  double angular_cmd = 0.0;

  if (goal_dist <= goal_dist_tolerance_) {
    linear_cmd = 0.0;
    angular_cmd = std::clamp(align_gain_ * goal_yaw_error, -max_angular_speed_, max_angular_speed_);
    if (std::abs(goal_yaw_error) <= goal_yaw_tolerance_) {
      angular_cmd = 0.0;
    }
  } else {
    const auto lookahead_pose = selectLookaheadPose(working_plan, pose);
    const double target_bearing = std::atan2(
      lookahead_pose.pose.position.y - pose.pose.position.y,
      lookahead_pose.pose.position.x - pose.pose.position.x);
    const double heading_error = normalizeAngle(target_bearing - current_yaw);

    linear_cmd = max_linear_speed_;
    if (goal_dist < slowdown_distance_) {
      linear_cmd = max_linear_speed_ * (goal_dist / slowdown_distance_);
    }
    linear_cmd = std::clamp(linear_cmd, min_linear_speed_, max_linear_speed_);

    angular_cmd = std::clamp(heading_gain_ * heading_error, -max_angular_speed_, max_angular_speed_);
    if (std::abs(heading_error) > heading_hold_angle_) {
      linear_cmd = 0.0;
    }
  }

  if (std::abs(linear_cmd) > 1e-6 || std::abs(angular_cmd) > 1e-6) {
    if (isCollisionImminent(pose, linear_cmd, angular_cmd)) {
      throw nav2_core::ControllerException("Predicted collision in local costmap.");
    }
  }

  cmd.twist.linear.x = linear_cmd;
  cmd.twist.angular.z = angular_cmd;
  return cmd;
}

bool P3ATController::cancel()
{
  canceled_ = true;
  return true;
}

void P3ATController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_linear_speed_ = nominal_max_linear_speed_ * (speed_limit / 100.0);
  } else {
    max_linear_speed_ = speed_limit;
  }
  max_linear_speed_ = std::max(min_linear_speed_, max_linear_speed_);
}

void P3ATController::reset()
{
  canceled_ = false;
  global_plan_.poses.clear();
  max_linear_speed_ = nominal_max_linear_speed_;
}

nav_msgs::msg::Path P3ATController::transformPlan(const std::string & target_frame) const
{
  if (global_plan_.poses.empty()) {
    return nav_msgs::msg::Path{};
  }

  if (target_frame.empty() || global_plan_.header.frame_id == target_frame) {
    return global_plan_;
  }

  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header = global_plan_.header;
  transformed_plan.header.frame_id = target_frame;
  transformed_plan.poses.reserve(global_plan_.poses.size());

  for (const auto & pose : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    try {
      tf_->transform(pose, transformed_pose, target_frame);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(logger_, "Failed to transform plan pose into %s: %s", target_frame.c_str(), ex.what());
      return nav_msgs::msg::Path{};
    }
    transformed_plan.poses.push_back(transformed_pose);
  }

  return transformed_plan;
}

void P3ATController::prunePlan(
  nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  if (plan.poses.size() < 2) {
    return;
  }

  auto closest_it = std::min_element(
    plan.poses.begin(),
    plan.poses.end(),
    [&pose](const auto & a, const auto & b) {
      return distance2D(a, pose) < distance2D(b, pose);
    });

  if (closest_it != plan.poses.end() && closest_it != plan.poses.begin()) {
    plan.poses.erase(plan.poses.begin(), closest_it);
  }
}

geometry_msgs::msg::PoseStamped P3ATController::selectLookaheadPose(
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  for (const auto & plan_pose : plan.poses) {
    if (distance2D(plan_pose, pose) >= lookahead_distance_) {
      return plan_pose;
    }
  }
  return plan.poses.back();
}

bool P3ATController::isCollisionImminent(
  const geometry_msgs::msg::PoseStamped & pose,
  double linear_vel,
  double angular_vel) const
{
  if (!costmap_) {
    return false;
  }

  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double theta = tf2::getYaw(pose.pose.orientation);
  constexpr double dt = 0.1;

  for (double t = 0.0; t <= collision_lookahead_time_; t += dt) {
    x += linear_vel * dt * std::cos(theta);
    y += linear_vel * dt * std::sin(theta);
    theta = normalizeAngle(theta + angular_vel * dt);

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(x, y, mx, my)) {
      return true;
    }

    const unsigned char cost = costmap_->getCost(mx, my);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      return true;
    }
  }

  return false;
}

double P3ATController::normalizeAngle(double angle)
{
  return angles::normalize_angle(angle);
}

double P3ATController::distance2D(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return std::hypot(dx, dy);
}

}  // namespace p3at_nav_plugins

PLUGINLIB_EXPORT_CLASS(p3at_nav_plugins::P3ATController, nav2_core::Controller)
