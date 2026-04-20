/*
 * ROS 2 wrapper around AriaCoda for Pioneer-compatible robots.
 * Run with launch or:
 *   ros2 run aria_node ariaNode --ros-args -p odom_topic:=/odom -- -rp /dev/ttyUSB0
 */

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "Aria/Aria.h"

namespace
{

std::atomic_bool g_stop_requested{false};
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

void handle_signal(int)
{
  g_stop_requested = true;
}

}  // namespace

class AriaNode : public rclcpp::Node
{
public:
  explicit AriaNode(ArRobot * robot)
  : Node("aria_node"), robot_(robot)
  {
    declare_parameter("cmd_vel_topic", "/cmd_vel");
    declare_parameter("odom_topic", "/odom");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("publish_tf", false);
    declare_parameter("loop_hz", 20.0);
    declare_parameter("cmd_timeout_sec", 0.5);
    declare_parameter("max_trans_vel_mm_s", 500.0);
    declare_parameter("max_rot_vel_deg_s", 60.0);

    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    loop_hz_ = get_parameter("loop_hz").as_double();
    cmd_timeout_sec_ = get_parameter("cmd_timeout_sec").as_double();
    max_trans_vel_mm_s_ = get_parameter("max_trans_vel_mm_s").as_double();
    max_rot_vel_deg_s_ = get_parameter("max_rot_vel_deg_s").as_double();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 20, std::bind(&AriaNode::cmd_vel_callback, this, std::placeholders::_1));

    last_cmd_time_ = now();
    robot_->setAbsoluteMaxTransVel(max_trans_vel_mm_s_);
    robot_->setAbsoluteMaxRotVel(max_rot_vel_deg_s_);

    RCLCPP_INFO(
      get_logger(),
      "Aria base driver ready. cmd_vel_topic=%s odom_topic=%s publish_tf=%s",
      cmd_vel_topic_.c_str(), odom_topic_.c_str(), publish_tf_ ? "true" : "false");
  }

  double loop_hz() const
  {
    return loop_hz_;
  }

  void step()
  {
    if ((now() - last_cmd_time_).seconds() > cmd_timeout_sec_) {
      commanded_linear_mps_ = 0.0;
      commanded_angular_rps_ = 0.0;
    }

    const double requested_trans_mm_s =
      clamp(commanded_linear_mps_ * 1000.0, -max_trans_vel_mm_s_, max_trans_vel_mm_s_);
    const double requested_rot_deg_s =
      clamp(commanded_angular_rps_ * 180.0 / kPi, -max_rot_vel_deg_s_, max_rot_vel_deg_s_);

    double x_mm = 0.0;
    double y_mm = 0.0;
    double th_deg = 0.0;
    double vel_mm_s = 0.0;
    double rot_vel_deg_s = 0.0;

    robot_->lock();
    robot_->setVel(requested_trans_mm_s);
    robot_->setRotVel(requested_rot_deg_s);
    const ArPose encoder_pose = robot_->getRawEncoderPose();
    x_mm = encoder_pose.getX();
    y_mm = encoder_pose.getY();
    th_deg = encoder_pose.getTh();
    vel_mm_s = robot_->getVel();
    rot_vel_deg_s = robot_->getRotVel();
    robot_->unlock();

    publish_odometry(x_mm, y_mm, th_deg, vel_mm_s, rot_vel_deg_s);
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    commanded_linear_mps_ = msg->linear.x;
    commanded_angular_rps_ = msg->angular.z;
    last_cmd_time_ = now();
  }

  void publish_odometry(
    double x_mm,
    double y_mm,
    double th_deg,
    double vel_mm_s,
    double rot_vel_deg_s)
  {
    const auto stamp = now();
    const double yaw_rad = th_deg * kPi / 180.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    q.normalize();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_mm / 1000.0;
    odom.pose.pose.position.y = y_mm / 1000.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = vel_mm_s / 1000.0;
    odom.twist.twist.angular.z = rot_vel_deg_s * kPi / 180.0;
    odom_pub_->publish(odom);

    if (!publish_tf_ || !tf_broadcaster_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom.header;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = odom.pose.pose.position.x;
    tf_msg.transform.translation.y = odom.pose.pose.position.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  ArRobot * robot_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{false};
  double loop_hz_{20.0};
  double cmd_timeout_sec_{0.5};
  double max_trans_vel_mm_s_{500.0};
  double max_rot_vel_deg_s_{60.0};
  double commanded_linear_mps_{0.0};
  double commanded_angular_rps_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  Aria::init();
  std::vector<std::string> aria_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> aria_argv;
  aria_argv.reserve(aria_args.size());
  for (auto & arg : aria_args) {
    aria_argv.push_back(arg.data());
  }
  int aria_argc = static_cast<int>(aria_argv.size());
  ArArgumentParser parser(&aria_argc, aria_argv.data());
  parser.loadDefaultArguments();

  ArRobot robot;
  ArRobotConnector robot_connector(&parser, &robot);
  if (!robot_connector.connectRobot()) {
    ArLog::log(ArLog::Terse, "aria_node: Could not connect to the robot.");
    if (parser.checkHelpAndWarnUnparsed()) {
      Aria::logOptions();
    }
    Aria::exit(1);
    return 1;
  }

  robot.runAsync(true);
  robot.enableMotors();

  auto node = std::make_shared<AriaNode>(&robot);
  rclcpp::WallRate rate(std::max(5.0, node->loop_hz()));

  while (rclcpp::ok() && !g_stop_requested.load()) {
    rclcpp::spin_some(node);
    node->step();
    rate.sleep();
  }

  robot.lock();
  robot.setVel(0.0);
  robot.setRotVel(0.0);
  robot.unlock();
  robot.disableMotors();
  robot.stopRunning();
  robot.waitForRunExit();

  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  Aria::exit(0);
  return 0;
}
