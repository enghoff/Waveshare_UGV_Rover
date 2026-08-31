// Nav2 behaviours that let a rover already touching something drive away from it.
//
// The fault these exist for: put an obstacle close behind this rover and it
// stops being able to move at all -- not backwards, which is right, but also
// not forwards and not on the spot. Reproduced in ros_nav/corridor_sim.py, and
// the cause is one line of arithmetic repeated in three Nav2 behaviours.
//
// `Spin`, `DriveOnHeading` and `BackUp` each project the motion they are about
// to command forward over `simulate_ahead_time` and test the footprint at every
// projected pose. The projection starts at cycle zero, where the simulated
// displacement is zero -- so the first pose tested is the pose the rover is
// standing in *now*, and if that one is in collision the whole behaviour
// returns COLLISION_AHEAD without moving. The direction of travel never enters
// into it. Nav2 1.3.12, which is what this rover runs, has no parameter that
// turns the check off.
//
// The rover's footprint is a 0.20 m circle on `base_link` while the chassis
// reaches 0.16 m behind it (config/nav2.yaml explains at length why a circle),
// so anything inside about 0.17 m behind the sensor freezes it completely.
// Often it really is touching -- and "touching something behind" is exactly
// when a rover most needs to be able to drive away.
//
// ## What these change, and what they deliberately do not
//
// Each class calls Nav2's own implementation first and returns its answer
// unchanged unless it is specifically COLLISION_AHEAD. So every healthy motion
// this rover makes is still Nav2's code, byte for byte.
//
// When Nav2 does refuse, the current pose decides what happens next:
//
//   the current pose is clear   the obstruction is genuinely ahead of the
//                               rover, which is what the check is for. The
//                               refusal stands, untouched.
//
//   the current pose is in      the rover is already in contact, and stock Nav2
//   collision                   will now refuse every motion for ever. This is
//                               the state these classes exist for, and each one
//                               has its own rule for what may be allowed.
//
// `EscapeSpin` allows the rotation. That is sound because the footprint is a
// circle centred on `base_link`: rotating it about its own centre maps it onto
// itself, so a turn cannot sweep ground the rover is not already standing on.
// It is also why Nav2's check can never help here -- with a circular footprint
// it is either vacuous or an unconditional veto, and corridor_sim shows exactly
// that, 90.1 degrees or 0.0 and never anything in between. **If the footprint
// ever stops being a circle this reasoning stops holding**; ros_nav/selftest.py
// fails if nav2.yaml grows a footprint polygon, which is the alarm for it.
//
// `EscapeDriveOnHeading` -- which is also `BackUp`, the same template with the
// sign flipped -- allows the motion only when the far end of the projection is
// clear, meaning the motion leads out of contact rather than deeper into it.
// Driving forward off a rear obstacle passes. Reversing into that same obstacle
// does not, and neither does driving forward into a wall while something is
// behind, which is the wedged case where the honest answer is still no.
//
// Escaping is time limited. A rover that has been trying to escape for
// `escape_time_limit` seconds and is still in contact stops and reports the
// collision, because at that point it is grinding rather than escaping.

#ifndef UGV_BEHAVIORS__ESCAPE_BEHAVIORS_HPP_
#define UGV_BEHAVIORS__ESCAPE_BEHAVIORS_HPP_

#include <memory>
#include <string>

#include "nav2_behaviors/plugins/spin.hpp"
#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_behaviors/plugins/back_up.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/spin.hpp"

namespace ugv_behaviors
{

using nav2_behaviors::ResultStatus;
using nav2_behaviors::Status;

/// How long a behaviour may go on escaping before it gives up and reports the
/// collision. Long enough to drive clear of a body-length of obstacle at
/// recovery speed; short enough that a rover which cannot escape says so.
constexpr double kDefaultEscapeTimeLimit = 3.0;

/// Turning on the spot, which is never refused once the rover is in contact.
class EscapeSpin : public nav2_behaviors::Spin
{
public:
  ResultStatus onCycleUpdate() override;
  void onConfigure() override;
  ResultStatus onRun(const std::shared_ptr<const SpinActionGoal> command) override;

protected:
  /// True when the footprint at the rover's own pose is in collision, which is
  /// the only state in which any of this changes Nav2's answer.
  bool inContactNow();

  double escape_time_limit_{kDefaultEscapeTimeLimit};
  rclcpp::Time escaping_since_;
  bool escaping_{false};
};

/// Driving a heading, allowed while it leads out of contact rather than into it.
///
/// A template because Nav2's is: `BackUp` is this same behaviour with a negative
/// speed, and both are instantiated at the bottom of the .cpp.
template<typename ActionT>
class EscapeDriveOnHeading : public nav2_behaviors::DriveOnHeading<ActionT>
{
public:
  ResultStatus onCycleUpdate() override;
  void onConfigure() override;

protected:
  bool inContactNow();

  /// Whether the far end of the look-ahead projection is clear -- that is,
  /// whether this motion ends with the rover out of contact.
  bool projectionEndsClear();

  double escape_time_limit_{kDefaultEscapeTimeLimit};
  rclcpp::Time escaping_since_;
  bool escaping_{false};
};

class EscapeDriveOnHeadingAction
  : public EscapeDriveOnHeading<nav2_msgs::action::DriveOnHeading>
{
};

class EscapeBackUpAction : public EscapeDriveOnHeading<nav2_msgs::action::BackUp>
{
};

}  // namespace ugv_behaviors

#endif  // UGV_BEHAVIORS__ESCAPE_BEHAVIORS_HPP_
