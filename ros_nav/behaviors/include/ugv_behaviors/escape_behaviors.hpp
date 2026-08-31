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
// Turning and driving are then treated quite differently, because the geometry
// is quite different.
//
// **Turning** is never refused, and that is not a relaxation of safety but a
// correction of it. The footprint is a circle centred on `base_link`, which is
// the point the rover rotates about, so a rotation maps the body exactly onto
// itself: the ground covered is the same at every heading. If the rover fits
// where it stands it fits at every heading, and if it does not, no heading
// helps. Nav2's check cannot add information here -- it can only agree with the
// pose the rover is already in, or disagree with it wrongly, which is what it
// does. It does not test a circle: a radius becomes a sixteen-sided polygon and
// the test walks that outline across a 5 cm grid, so rotating it crosses a
// slightly different set of cells and one marginal cell becomes "turning that
// way would sweep through something" on a rover standing in open floor. Watched
// here on a 180 degree turn. `EscapeSpin` measures the footprint's roundness off
// the same topic the collision checker reads and only claims this when the
// footprint really is a circle.
//
// **Driving** genuinely changes the ground the body covers, so its check is
// real and is kept. `EscapeDriveOnHeading` -- which is also `BackUp`, the same
// template with the sign flipped -- changes Nav2's answer only when the rover is
// already in contact *and* the far end of the projection is clear, meaning the
// motion leads out of the contact rather than deeper into it. Driving forward
// off a rear obstacle passes. Reversing into that same obstacle does not, and
// neither does driving forward into a wall while something is behind: the wedged
// case, where no is still the honest answer.
//
// Escaping is limited by *lack of progress*, not by the clock -- a 180 degree
// turn at the recovery speed takes over six seconds, so a stopwatch would cut
// exactly the manoeuvre this exists to allow. A rover that has not moved for
// `escape_time_limit` seconds is grinding rather than escaping, and stops.

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
#include "geometry_msgs/msg/polygon_stamped.hpp"

namespace ugv_behaviors
{

using nav2_behaviors::ResultStatus;
using nav2_behaviors::Status;

/// How long a behaviour may go on escaping **without making progress** before it
/// gives up and reports the collision.
///
/// Against elapsed time this would be wrong: a 180 degree turn at the recovery
/// speed of 0.5 rad/s takes over six seconds, so a limit on the clock would cut
/// exactly the manoeuvre this exists to allow. Measured against progress
/// instead, it means what it is for -- a rover that is grinding rather than
/// escaping -- and a turn that is still turning is never cut off.
constexpr double kDefaultEscapeTimeLimit = 3.0;

/// Turning on the spot, which a circular-footprint rover is never refused.
///
/// **Nav2's collision check on a spin is unsound for this rover in both
/// directions, and that is the whole reason this class exists.**
///
/// The footprint is a circle centred on `base_link`, which is the point the
/// rover rotates about. Rotating a circle about its own centre maps it exactly
/// onto itself, so the ground covered is identical at every heading: if the
/// rover fits where it is standing, it fits at every heading, and if it does not
/// fit, no heading helps. The check can therefore only ever agree with the pose
/// the rover is already in -- it cannot add information.
///
/// It can still *disagree*, and does, because Nav2 does not check a circle. A
/// radius becomes a polygon of sixteen vertices, and the collision test walks
/// that polygon's outline across a 5 cm grid; rotate it and the outline crosses
/// a slightly different set of cells. So a rover standing legally is told that
/// turning "would sweep through something" on the strength of one marginal cell
/// that the rasterised outline clips at one projected heading and not at the
/// current one. Watched on the rover: a 180 degree turn refused outright with
/// Spin's COLLISION_AHEAD while the rover sat in open floor.
///
/// So when the footprint is a circle this class lets the rotation proceed
/// whichever pose Nav2 objected to. When it is not a circle the reasoning above
/// does not hold -- a long body really can sweep a corner into a wall -- and it
/// falls back to allowing the turn only when the rover is already in contact,
/// which is the case where refusing traps it with no way out.
///
/// Whether the footprint is a circle is measured here rather than assumed:
/// `onConfigure` subscribes to the same footprint topic the collision checker
/// uses and compares the shortest vertex to the longest.
class EscapeSpin : public nav2_behaviors::Spin
{
public:
  ResultStatus onCycleUpdate() override;
  void onConfigure() override;
  ResultStatus onRun(const std::shared_ptr<const SpinActionGoal> command) override;

protected:
  /// True when the footprint at the rover's own pose is in collision.
  bool inContactNow();

  /// True when the published footprint is a circle about the rotation centre,
  /// which is what makes a rotation provably safe. False until one has arrived.
  bool footprintIsCircular() const;

  void onFootprint(geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  /// -1 until a footprint has been seen; then the shortest vertex divided by the
  /// longest, which is 1.0 for a true circle and 0.98 for Nav2's sixteen-sided
  /// approximation of one.
  double footprint_roundness_{-1.0};

  double escape_time_limit_{kDefaultEscapeTimeLimit};
  rclcpp::Time escaping_since_;
  double escape_progress_mark_{0.0};
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
  double escape_progress_mark_{0.0};
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
