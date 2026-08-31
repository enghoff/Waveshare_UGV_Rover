// See include/ugv_behaviors/escape_behaviors.hpp for what these are for.

#include "ugv_behaviors/escape_behaviors.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"

namespace ugv_behaviors
{

// --- turning on the spot -----------------------------------------------------

void EscapeSpin::onConfigure()
{
  nav2_behaviors::Spin::onConfigure();
  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  nav2_util::declare_parameter_if_not_declared(
    node, "escape_time_limit", rclcpp::ParameterValue(kDefaultEscapeTimeLimit));
  node->get_parameter("escape_time_limit", escape_time_limit_);
}

ResultStatus EscapeSpin::onRun(const std::shared_ptr<const SpinActionGoal> command)
{
  // Each goal starts with a clean escape clock, or a rover that escaped once
  // would carry that stopwatch into every later turn.
  escaping_ = false;
  return nav2_behaviors::Spin::onRun(command);
}

bool EscapeSpin::inContactNow()
{
  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
      pose, *this->tf_, this->local_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    return false;
  }
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = pose.pose.position.x;
  pose2d.y = pose.pose.position.y;
  pose2d.theta = tf2::getYaw(pose.pose.orientation);
  return !this->local_collision_checker_->isCollisionFree(pose2d, true);
}

ResultStatus EscapeSpin::onCycleUpdate()
{
  const ResultStatus result = nav2_behaviors::Spin::onCycleUpdate();

  if (result.status != Status::FAILED ||
    result.error_code != SpinActionResult::COLLISION_AHEAD)
  {
    escaping_ = false;
    return result;
  }

  // Nav2 refused. If the rover is standing somewhere legal then the obstruction
  // is genuinely in the arc it was about to sweep, and the refusal is correct.
  if (!inContactNow()) {
    escaping_ = false;
    return result;
  }

  const rclcpp::Time now = this->clock_->now();
  if (!escaping_) {
    escaping_ = true;
    escaping_since_ = now;
    RCLCPP_WARN(
      this->logger_,
      "In contact and asked to turn: turning anyway. A circular footprint "
      "rotated about its own centre sweeps no new ground, so this cannot make "
      "the contact worse -- and refusing would leave the rover no way out.");
  } else if ((now - escaping_since_).seconds() > escape_time_limit_) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "Still in contact after %.1f s of turning: giving up rather than grinding.",
      escape_time_limit_);
    return ResultStatus{Status::FAILED, SpinActionResult::COLLISION_AHEAD};
  }

  // Nav2's cycle has already updated relative_yaw_ and prev_yaw_ and then
  // stopped the wheels; all that is left is to command the rotation it would
  // have commanded. The arithmetic is its own, from spin.cpp.
  const double remaining_yaw = std::fabs(cmd_yaw_) - std::fabs(relative_yaw_);
  if (remaining_yaw < 1e-6) {
    this->stopRobot();
    return ResultStatus{Status::SUCCEEDED, SpinActionResult::NONE};
  }
  double vel = std::sqrt(2.0 * rotational_acc_lim_ * remaining_yaw);
  vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.frame_id = this->robot_base_frame_;
  cmd_vel->header.stamp = now;
  cmd_vel->twist.angular.z = std::copysign(vel, cmd_yaw_);
  this->vel_pub_->publish(std::move(cmd_vel));

  return ResultStatus{Status::RUNNING, SpinActionResult::NONE};
}

// --- driving a heading -------------------------------------------------------

template<typename ActionT>
void EscapeDriveOnHeading<ActionT>::onConfigure()
{
  nav2_behaviors::DriveOnHeading<ActionT>::onConfigure();
  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  nav2_util::declare_parameter_if_not_declared(
    node, "escape_time_limit", rclcpp::ParameterValue(kDefaultEscapeTimeLimit));
  node->get_parameter("escape_time_limit", escape_time_limit_);
}

template<typename ActionT>
bool EscapeDriveOnHeading<ActionT>::inContactNow()
{
  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
      pose, *this->tf_, this->local_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    return false;
  }
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = pose.pose.position.x;
  pose2d.y = pose.pose.position.y;
  pose2d.theta = tf2::getYaw(pose.pose.orientation);
  return !this->local_collision_checker_->isCollisionFree(pose2d, true);
}

template<typename ActionT>
bool EscapeDriveOnHeading<ActionT>::projectionEndsClear()
{
  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
      pose, *this->tf_, this->local_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    return false;
  }
  const double theta = tf2::getYaw(pose.pose.orientation);

  // The same projection Nav2 just did, and deliberately the same numbers: the
  // furthest pose it looked at is the one that says whether this motion ends
  // out of contact. Walked from the far end back, so that a motion which clears
  // the obstacle only near the end of the horizon still counts as an escape.
  const int max_cycle_count =
    static_cast<int>(this->cycle_frequency_ * this->simulate_ahead_time_);
  bool fetch_data = true;
  for (int cycle = max_cycle_count - 1; cycle >= 1; --cycle) {
    const double moved =
      this->command_speed_ * (static_cast<double>(cycle) / this->cycle_frequency_);
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = pose.pose.position.x + moved * std::cos(theta);
    pose2d.y = pose.pose.position.y + moved * std::sin(theta);
    pose2d.theta = theta;
    if (this->local_collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return true;
    }
    fetch_data = false;
  }
  return false;
}

template<typename ActionT>
ResultStatus EscapeDriveOnHeading<ActionT>::onCycleUpdate()
{
  const ResultStatus result = nav2_behaviors::DriveOnHeading<ActionT>::onCycleUpdate();

  if (result.status != Status::FAILED ||
    result.error_code != ActionT::Result::COLLISION_AHEAD)
  {
    escaping_ = false;
    return result;
  }

  // Standing somewhere legal means the obstruction really is on the way, and
  // this is the one direction the rover drives into things. Leave it refused.
  if (!inContactNow()) {
    escaping_ = false;
    return result;
  }

  // In contact, so the only question is whether this motion leads out.
  if (!projectionEndsClear()) {
    escaping_ = false;
    return result;
  }

  const rclcpp::Time now = this->clock_->now();
  if (!escaping_) {
    escaping_ = true;
    escaping_since_ = now;
    RCLCPP_WARN(
      this->logger_,
      "In contact, but this heading leads out of it: driving anyway rather "
      "than leaving the rover with no way off the obstacle.");
  } else if ((now - escaping_since_).seconds() > escape_time_limit_) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "Still in contact after %.1f s of driving clear: giving up.",
      escape_time_limit_);
    return ResultStatus{Status::FAILED, ActionT::Result::COLLISION_AHEAD};
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = now;
  cmd_vel->header.frame_id = this->robot_base_frame_;
  cmd_vel->twist.linear.y = 0.0;
  cmd_vel->twist.angular.z = 0.0;
  cmd_vel->twist.linear.x = this->command_speed_;
  this->vel_pub_->publish(std::move(cmd_vel));

  return ResultStatus{Status::RUNNING, ActionT::Result::NONE};
}

template class EscapeDriveOnHeading<nav2_msgs::action::DriveOnHeading>;
template class EscapeDriveOnHeading<nav2_msgs::action::BackUp>;

}  // namespace ugv_behaviors

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ugv_behaviors::EscapeSpin, nav2_core::Behavior)
PLUGINLIB_EXPORT_CLASS(ugv_behaviors::EscapeDriveOnHeadingAction, nav2_core::Behavior)
PLUGINLIB_EXPORT_CLASS(ugv_behaviors::EscapeBackUpAction, nav2_core::Behavior)
