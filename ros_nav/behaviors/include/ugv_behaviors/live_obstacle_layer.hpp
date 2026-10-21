// What the lidar sees now and the map does not have, on the planner's costmap,
// and nothing that it saw before.
//
// The global costmap has no obstacle layer, and config/nav2.yaml says why: on a
// rover running SLAM, Nav2's obstacle layer remembers its marks in the `map`
// frame, every loop closure moves that frame under them, and the ghosts of
// walls they leave behind closed 61% of the mapped floor to the planner. So the
// planner could not see a person standing in the way, and drew its route
// through them; on 2026-10-08 the rover was given a way round of its own,
// straight legs driven blind, and the second-to-last of them drifted nine
// degrees into the person it was going round (docs/progress/2026-10-08-blocking-trials.md).
//
// This layer has no memory, so it cannot leave ghosts. It keeps no grid: on
// each update it lays the latest scan, transformed with the current `map`
// transform, straight onto the master grid, and its update bounds always take
// in where it marked last time, so that the costmap's own reset of those bounds
// wipes the old marks before the static layer repaints the map under them.
// A point within `wall_margin_m` of a cell the map already holds as lethal is
// left alone: that is the map's wall, seen a few centimetres off, not something
// new, and marking it would narrow every doorway. The margin grows by
// `wall_margin_per_m` with the point's range, because the rover's heading on the
// map is a few degrees out from one moment to the next and an error of angle
// lands further off the further away it is: on 2026-10-09, 7% of the returns
// within a metre lay more than 10 cm from a mapped wall, 47% of those at two to
// three metres, and those are the orange marks seen along the walls on the
// console (docs/progress/2026-10-09-live-layer-marks.md).
//
// **A margin cannot cover a rover that is off by its own width.** Between the
// mapper's corrections the rover's place on the map is 10 to 30 cm out one time
// in ten (the recorded sessions of 2026-10-17 to 21), and parked or turning on
// the spot nothing corrects it. On 2026-10-21 the rover stood 20 cm and 3.5
// degrees off in an 80 cm doorway; the layer drew the furniture beside it as a
// line of new obstacles 20 cm in front of where the map has it, the planner
// found no way through, and the rover could not move far enough for the mapper
// to put it right. So before marking, the scan as a whole is laid on the map at
// the small shift that puts the most of it on walls (`best_shift`); when that
// shift is clearly better than the believed pose, each point is judged new or
// mapped there instead. Replayed, that took the doorway's 14 marks to none, and
// marked all 22 returns of a person put in the doorway's middle where the old
// rule marked 11; on a drive with two bean bags newly set down it kept their
// marks (31.6 a scan against 31.7), the shift being believed on 37 scans in 890.
//
// `<name>.enabled` can be set while the planner runs, and is the way to take
// the layer back out without a redeploy: switched off, it clears its marks on
// the next update and then marks nothing.

#ifndef UGV_BEHAVIORS__LIVE_OBSTACLE_LAYER_HPP_
#define UGV_BEHAVIORS__LIVE_OBSTACLE_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace ugv_behaviors
{

// A point in the costmap's global frame.
using Point = std::pair<double, double>;

// The arithmetic, apart from ROS, so that it can be tested without a node.
namespace live_layer
{

// Mark each point lethal on `grid` unless a cell within `wall_cells` of it is
// already lethal there. Returns how many cells were marked.
int mark_fresh(nav2_costmap_2d::Costmap2D & grid, const std::vector<Point> & points,
               int wall_cells);

// The same, with each point's own margin: `margin_m` plus `per_m` for every metre
// of its range, `ranges[i]` being the range of `points[i]`.
int mark_fresh(nav2_costmap_2d::Costmap2D & grid, const std::vector<Point> & points,
               const std::vector<double> & ranges, double margin_m, double per_m);

// The box round `points`, as (min_x, min_y, max_x, max_y); false when empty.
bool bounds_of(const std::vector<Point> & points, double & min_x, double & min_y,
               double & max_x, double & max_y);

// How far the whole scan may be off the map, searched for before marking.
struct Align
{
  double window_m = 0.0;      // 0 switches the search off
  double step_m = 0.05;
  double window_deg = 6.0;
  double step_deg = 1.5;
  int min_gain = 20;          // returns the shift must lay on walls beyond the pose's
  double min_gain_frac = 0.10;  // ...and this share of the scan
};

// What the search found: the shift (about `origin`), how many returns lie on a
// wall at the believed pose and at the shift, and whether the shift was believed.
struct Shift
{
  double dx = 0.0, dy = 0.0, dyaw = 0.0;
  int on_wall_here = 0, on_wall_shifted = 0;
  bool believed = false;
};

// The shift of the whole scan, rotated about `origin` (the laser), that lays the
// most returns within a cell of a lethal cell on `grid`, the smallest such when
// several tie. Believed only when it lays `min_gain` and `min_gain_frac` of the
// scan on walls beyond what the believed pose does.
Shift best_shift(const nav2_costmap_2d::Costmap2D & grid, const std::vector<Point> & points,
                 const Point & origin, const Align & align);

// `mark_fresh` with ranges, except that when the scan as a whole lies clearly
// better on the map shifted (`best_shift`), each point is judged at the shift
// instead of where the believed pose puts it. Marks are still put where the
// believed pose puts them, which is where the planner thinks they are.
int mark_fresh(nav2_costmap_2d::Costmap2D & grid, const std::vector<Point> & points,
               const std::vector<double> & ranges, double margin_m, double per_m,
               const Point & origin, const Align & align, Shift * found = nullptr);

}  // namespace live_layer

class LiveObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  LiveObstacleLayer() = default;

  void onInitialize() override;
  void reset() override {}
  bool isClearable() override {return true;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  void onScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  std::mutex mutex_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_parameters_;

  // What this update will mark, and the box it was marked in last time.
  std::vector<Point> points_;
  std::vector<double> ranges_;
  Point origin_{0.0, 0.0};
  bool marked_before_ = false;
  bool said_shift_ = false;
  double before_min_x_ = 0.0, before_min_y_ = 0.0, before_max_x_ = 0.0, before_max_y_ = 0.0;

  bool active_ = true;
  std::string topic_ = "/scan";
  double max_range_m_ = 3.0;
  double min_range_m_ = 0.15;
  double max_age_s_ = 0.5;
  double wall_margin_m_ = 0.10;
  double wall_margin_per_m_ = 0.087;
  live_layer::Align align_;
};

}  // namespace ugv_behaviors

#endif  // UGV_BEHAVIORS__LIVE_OBSTACLE_LAYER_HPP_
