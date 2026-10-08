// Exercise the actual plugin's goal initialization without an action server,
// velocity publisher, or motor connection. A BackUp action type alone does not
// reverse DriveOnHeading: Nav2's BackUp subclass supplies that normalization.
#include <cmath>
#include <iostream>
#include <memory>

#include "ugv_behaviors/escape_behaviors.hpp"

template<typename Plugin>
class Probe : public Plugin
{
public:
  Probe()
  {
    this->clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    this->tf_ = std::make_shared<tf2_ros::Buffer>(this->clock_);
    this->local_frame_ = "base_link";
    this->robot_base_frame_ = "base_link";
    this->transform_tolerance_ = 0.0;
  }

  double distance() const {return this->command_x_;}
  double speed() const {return this->command_speed_;}
};

int main()
{
  bool ok = true;
  Probe<ugv_behaviors::EscapeBackUpAction> backup;
  Probe<nav2_behaviors::BackUp> stock;
  for (const double distance : {0.1, 0.25, 0.5}) {
    for (const double sign : {1.0, -1.0}) {
      auto goal = std::make_shared<nav2_msgs::action::BackUp::Goal>();
      goal->target.x = sign * distance;
      goal->speed = sign * 0.2;
      goal->time_allowance.sec = 5;
      const auto reference = stock.onRun(goal);
      const auto result = backup.onRun(goal);
      const bool same = result.status == nav2_behaviors::Status::SUCCEEDED &&
        result.status == reference.status &&
        result.error_code == reference.error_code &&
        backup.distance() == stock.distance() && backup.speed() == stock.speed() &&
        backup.distance() < 0 && backup.speed() < 0;
      std::cout << (same ? "PASS" : "FAIL") << " backup " << goal->target.x <<
        " m: target=" << backup.distance() << " speed=" << backup.speed() <<
        " (stock target=" << stock.distance() << " speed=" << stock.speed() << ")\n";
      ok = same && ok;
    }
  }
  auto invalid = std::make_shared<nav2_msgs::action::BackUp::Goal>();
  invalid->target.x = 0.1;
  invalid->target.y = 0.1;
  invalid->speed = 0.2;
  const auto rejected = backup.onRun(invalid);
  ok = rejected.status == nav2_behaviors::Status::FAILED &&
    rejected.error_code == nav2_msgs::action::BackUp::Result::INVALID_INPUT && ok;

  Probe<ugv_behaviors::EscapeDriveOnHeadingAction> forward;
  auto goal = std::make_shared<nav2_msgs::action::DriveOnHeading::Goal>();
  goal->target.x = 0.1;
  goal->speed = 0.2;
  goal->time_allowance.sec = 5;
  const auto result = forward.onRun(goal);
  const bool ahead = result.status == nav2_behaviors::Status::SUCCEEDED &&
    forward.distance() > 0 && forward.speed() > 0;
  std::cout << (ahead ? "PASS" : "FAIL") << " forward retains positive direction\n";
  ok = ahead && ok;
  return ok ? 0 : 1;
}
