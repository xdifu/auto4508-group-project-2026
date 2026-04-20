#ifndef P3AT_NAV_PLUGINS__P3AT_CONTROLLER_HPP_
#define P3AT_NAV_PLUGINS__P3AT_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace p3at_nav_plugins
{

class P3ATController : public nav2_core::Controller
{
public:
  P3ATController() = default;
  ~P3ATController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  bool cancel() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

private:
  nav_msgs::msg::Path transformPlan(const std::string & target_frame) const;
  void prunePlan(nav_msgs::msg::Path & plan, const geometry_msgs::msg::PoseStamped & pose) const;
  geometry_msgs::msg::PoseStamped selectLookaheadPose(
    const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::PoseStamped & pose) const;
  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped & pose,
    double linear_vel,
    double angular_vel) const;

  static double normalizeAngle(double angle);
  static double distance2D(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("p3at_controller")};
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  nav_msgs::msg::Path global_plan_;

  double lookahead_distance_{1.25};
  double slowdown_distance_{1.5};
  double goal_dist_tolerance_{0.25};
  double goal_yaw_tolerance_{0.087};
  double heading_hold_angle_{0.785};
  double heading_gain_{1.8};
  double align_gain_{1.5};
  double max_linear_speed_{0.6};
  double min_linear_speed_{0.05};
  double max_angular_speed_{0.9};
  double collision_lookahead_time_{1.5};

  double nominal_max_linear_speed_{0.6};
  bool canceled_{false};
};

}  // namespace p3at_nav_plugins

#endif  // P3AT_NAV_PLUGINS__P3AT_CONTROLLER_HPP_
