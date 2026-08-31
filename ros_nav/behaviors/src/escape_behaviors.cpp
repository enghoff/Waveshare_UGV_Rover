// See include/ugv_behaviors/escape_behaviors.hpp for what these are for.

#include "ugv_behaviors/escape_behaviors.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
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

  // The same footprint the collision checker is reading, so that "the footprint
  // is a circle" is something this class has measured rather than something the
  // config file asserts and nobody rechecks. Transient-local because the costmap
  // publishes it on a latched-style QoS and this subscription is made late.
  std::string footprint_topic = "local_costmap/published_footprint";
  nav2_util::declare_parameter_if_not_declared(
    node, "local_footprint_topic", rclcpp::ParameterValue(footprint_topic));
  node->get_parameter("local_footprint_topic", footprint_topic);
  footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    footprint_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&EscapeSpin::onFootprint, this, std::placeholders::_1));
}

void EscapeSpin::onFootprint(geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  // The footprint arrives in the robot frame, so a vertex's distance from the
  // origin is its distance from the point the rover turns about. A circle has
  // every vertex the same distance out; Nav2's sixteen-sided stand-in for one
  // has the shortest at cos(pi/16) = 0.98 of the longest.
  if (msg->polygon.points.size() < 3) {
    footprint_roundness_ = -1.0;
    return;
  }
  double shortest = std::numeric_limits<double>::max();
  double longest = 0.0;
  for (const auto & point : msg->polygon.points) {
    const double radius = std::hypot(static_cast<double>(point.x),
                                     static_cast<double>(point.y));
    shortest = std::min(shortest, radius);
    longest = std::max(longest, radius);
  }
  const double roundness = (longest > 1e-6) ? (shortest / longest) : -1.0;
  // Said once, and again only if it changes: the whole justification for
  // turning a refused spin rests on this number, so it should be findable in
  // the log rather than inferred from behaviour.
  if (std::fabs(roundness - footprint_roundness_) > 0.01) {
    RCLCPP_INFO(
      this->logger_,
      "footprint: %zu vertices, shortest %.2f of the longest -- %s",
      msg->polygon.points.size(), roundness,
      roundness >= 0.9 ? "a circle, so a refused turn is always overruled"
                       : "not a circle, so Nav2's spin check is kept");
  }
  footprint_roundness_ = roundness;
}

bool EscapeSpin::footprintIsCircular() const
{
  // 0.9 admits Nav2's 0.98 polygon and any finer one, and excludes this rover's
  // own measured rectangle, whose shortest vertex is 0.14 m against a longest of
  // 0.21 -- a ratio of 0.66. A body that shape really can sweep a corner into a
  // wall while turning, and must keep Nav2's check.
  return footprint_roundness_ >= 0.9;
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

  // Nav2 refused. Whether that refusal means anything depends on the shape of
  // the body, which is why it is measured rather than assumed.
  //
  // `inContactNow` asks the collision checker about a pose, and that throws
  // rather than returning false when it cannot answer -- an unknown costmap, a
  // pose off the edge of it. Uncaught, the exception leaves onCycleUpdate, the
  // server aborts the behaviour, and what reaches the console is the collision
  // refusal we were trying to overturn with no sign of why. So it is caught, and
  // a checker that cannot answer is treated as not-in-contact, which is the
  // conservative reading: it means the escape only proceeds on the footprint
  // being circular, which is a fact about the rover rather than about the map.
  const bool circular = footprintIsCircular();
  bool in_contact = false;
  if (!circular) {
    try {
      in_contact = inContactNow();
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        this->logger_, "Could not test the rover's own pose (%s); "
        "treating it as clear.", error.what());
      in_contact = false;
    }
  }
  // A warning rather than a debug line, and deliberately: it only fires when a
  // spin has been refused, which should be rare, and when one is refused this is
  // the state that explains what happened next. A silent decision here is what
  // made the first version of this take three attempts to understand.
  RCLCPP_WARN(
    this->logger_,
    "spin refused: roundness %.2f circular %d in_contact %d escaping %d "
    "relative_yaw %.3f", footprint_roundness_, circular ? 1 : 0,
    in_contact ? 1 : 0, escaping_ ? 1 : 0, relative_yaw_);
  if (!circular && !in_contact) {
    // A non-circular body standing somewhere legal really can sweep a corner
    // into something, so Nav2 is entitled to this one. (It is also the only
    // branch that ever refuses a turn on this rover, and only if somebody
    // replaces the circular footprint with a polygon.)
    escaping_ = false;
    return result;
  }

  const rclcpp::Time now = this->clock_->now();
  if (!escaping_) {
    escaping_ = true;
    escaping_since_ = now;
    escape_progress_mark_ = std::fabs(relative_yaw_);
    if (circular) {
      RCLCPP_WARN(
        this->logger_,
        "Nav2 refused this turn; turning anyway. The footprint is a circle "
        "about the point the rover rotates about (shortest vertex %.2f of the "
        "longest), so every heading covers the same ground and the refusal can "
        "only be the rasterised outline clipping a cell at one angle and not "
        "another.", footprint_roundness_);
    } else {
      RCLCPP_WARN(
        this->logger_,
        "In contact and asked to turn: turning anyway, because refusing would "
        "leave the rover no way out. The footprint is not a circle "
        "(shortest vertex %.2f of the longest), so this is the weaker case.",
        footprint_roundness_);
    }
  } else if (std::fabs(relative_yaw_) > escape_progress_mark_ + 1e-3) {
    // Still turning, so the clock starts again. The limit is for a rover that
    // has stopped moving, not for one that is taking a while -- a 180 degree
    // turn at the recovery speed is over six seconds and must not be cut off.
    escape_progress_mark_ = std::fabs(relative_yaw_);
    escaping_since_ = now;
  } else if ((now - escaping_since_).seconds() > escape_time_limit_) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "Asked to turn but not turning for %.1f s: giving up rather than "
      "grinding.", escape_time_limit_);
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
  //
  // Both of these ask the collision checker about a pose, which throws rather
  // than returning false when it cannot answer. Uncaught, that would leave the
  // behaviour server aborting on an exception instead of on a decision. A
  // checker that cannot answer leaves the refusal standing, which is the safe
  // way round for a motion that translates.
  try {
    if (!inContactNow() || !projectionEndsClear()) {
      escaping_ = false;
      return result;
    }
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      this->logger_, "Could not test the poses for this heading (%s); leaving "
      "Nav2's refusal alone.", error.what());
    escaping_ = false;
    return result;
  }

  const rclcpp::Time now = this->clock_->now();
  const double travelled = this->feedback_ ?
    static_cast<double>(this->feedback_->distance_traveled) : 0.0;
  if (!escaping_) {
    escaping_ = true;
    escaping_since_ = now;
    escape_progress_mark_ = travelled;
    RCLCPP_WARN(
      this->logger_,
      "In contact, but this heading leads out of it: driving anyway rather "
      "than leaving the rover with no way off the obstacle.");
  } else if (travelled > escape_progress_mark_ + 1e-3) {
    // Moving, so the clock starts again -- the limit is for a rover that has
    // stopped, not for one that is taking a while.
    escape_progress_mark_ = travelled;
    escaping_since_ = now;
  } else if ((now - escaping_since_).seconds() > escape_time_limit_) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "In contact and not moving for %.1f s: giving up rather than grinding.",
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
