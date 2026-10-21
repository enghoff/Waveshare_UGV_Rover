// See include/ugv_behaviors/live_obstacle_layer.hpp for what this is for.

#include "ugv_behaviors/live_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace ugv_behaviors
{

namespace live_layer
{

int mark_fresh(nav2_costmap_2d::Costmap2D & grid, const std::vector<Point> & points,
               int wall_cells)
{
  const double margin = wall_cells * grid.getResolution();
  return mark_fresh(grid, points, std::vector<double>(points.size(), 0.0),
                    margin - 1e-9, 0.0);
}

namespace
{

// Is a cell within `wall_cells` of (mx, my) lethal on `grid`?
bool mapped_near(const nav2_costmap_2d::Costmap2D & grid, int mx, int my, int wall_cells)
{
  const int width = static_cast<int>(grid.getSizeInCellsX());
  const int height = static_cast<int>(grid.getSizeInCellsY());
  for (int dy = -wall_cells; dy <= wall_cells; ++dy) {
    for (int dx = -wall_cells; dx <= wall_cells; ++dx) {
      if (dx * dx + dy * dy > wall_cells * wall_cells) {
        continue;
      }
      const int cx = mx + dx;
      const int cy = my + dy;
      if (cx < 0 || cy < 0 || cx >= width || cy >= height) {
        continue;
      }
      if (grid.getCost(cx, cy) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        return true;
      }
    }
  }
  return false;
}

int wall_cells_for(double margin_m, double per_m, double range, double resolution)
{
  return static_cast<int>(std::ceil((margin_m + per_m * range) / resolution));
}

}  // namespace

Shift best_shift(const nav2_costmap_2d::Costmap2D & grid, const std::vector<Point> & points,
                 const Point & origin, const Align & align)
{
  Shift out;
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
  if (!bounds_of(points, x0, y0, x1, y1)) {
    return out;
  }
  const double resolution = grid.getResolution();
  const double gx = grid.getOriginX();
  const double gy = grid.getOriginY();
  const int width = static_cast<int>(grid.getSizeInCellsX());
  const int height = static_cast<int>(grid.getSizeInCellsY());
  const double window_m = std::max(0.0, align.window_m);
  const double window_rad = window_m > 0.0 ? std::max(0.0, align.window_deg) * M_PI / 180.0 : 0.0;

  // Everything a candidate can move a point to, with a cell to spare: the
  // window, plus what the turn does to the furthest point.
  double furthest = 0.0;
  for (const auto & p : points) {
    furthest = std::max(furthest, std::hypot(p.first - origin.first, p.second - origin.second));
  }
  const double reach = window_m + furthest * std::sin(window_rad) + 2.0 * resolution;
  const int c0 = std::max(0, static_cast<int>(std::floor((x0 - reach - gx) / resolution)));
  const int r0 = std::max(0, static_cast<int>(std::floor((y0 - reach - gy) / resolution)));
  const int c1 = std::min(width - 1, static_cast<int>(std::floor((x1 + reach - gx) / resolution)));
  const int r1 = std::min(height - 1, static_cast<int>(std::floor((y1 + reach - gy) / resolution)));
  if (c1 < c0 || r1 < r0) {
    return out;
  }
  // On a wall: within a cell of a lethal one, the 3x3 round it.
  const int bw = c1 - c0 + 1;
  const int bh = r1 - r0 + 1;
  std::vector<unsigned char> near(static_cast<std::size_t>(bw) * bh, 0);
  for (int r = r0; r <= r1; ++r) {
    for (int c = c0; c <= c1; ++c) {
      if (grid.getCost(c, r) != nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          const int rr = r + dr - r0;
          const int cc = c + dc - c0;
          if (rr >= 0 && cc >= 0 && rr < bh && cc < bw) {
            near[static_cast<std::size_t>(rr) * bw + cc] = 1;
          }
        }
      }
    }
  }

  const std::size_t n = points.size();
  std::vector<double> rx(n), ry(n);
  auto on_walls = [&](double dx, double dy) {
      int count = 0;
      for (std::size_t i = 0; i < n; ++i) {
        const int c = static_cast<int>(std::floor((origin.first + rx[i] + dx - gx) / resolution)) - c0;
        const int r = static_cast<int>(std::floor((origin.second + ry[i] + dy - gy) / resolution)) - r0;
        if (c >= 0 && r >= 0 && c < bw && r < bh && near[static_cast<std::size_t>(r) * bw + c]) {
          ++count;
        }
      }
      return count;
    };
  auto turn = [&](double dyaw) {
      const double c = std::cos(dyaw);
      const double s = std::sin(dyaw);
      for (std::size_t i = 0; i < n; ++i) {
        const double px = points[i].first - origin.first;
        const double py = points[i].second - origin.second;
        rx[i] = c * px - s * py;
        ry[i] = s * px + c * py;
      }
    };

  turn(0.0);
  out.on_wall_here = on_walls(0.0, 0.0);
  out.on_wall_shifted = out.on_wall_here;
  if (window_m <= 0.0 || align.step_m <= 0.0) {
    return out;
  }
  const int steps = static_cast<int>(std::lround(window_m / align.step_m));
  const int turns = align.step_deg > 0.0 ?
    static_cast<int>(std::lround(std::max(0.0, align.window_deg) / align.step_deg)) : 0;
  int best = out.on_wall_here;
  double best_size = 0.0;
  for (int k = -turns; k <= turns; ++k) {
    const double dyaw_deg = k * align.step_deg;
    turn(dyaw_deg * M_PI / 180.0);
    for (int i = -steps; i <= steps; ++i) {
      for (int j = -steps; j <= steps; ++j) {
        const double dx = i * align.step_m;
        const double dy = j * align.step_m;
        const int count = on_walls(dx, dy);
        // The smallest of several that tie, so that a corridor, which fits
        // equally well anywhere along itself, is not slid down it.
        const double size = std::hypot(dx, dy) + 0.05 * std::abs(dyaw_deg);
        if (count > best || (count == best && size < best_size)) {
          best = count;
          best_size = size;
          out.dx = dx;
          out.dy = dy;
          out.dyaw = dyaw_deg * M_PI / 180.0;
        }
      }
    }
  }
  out.on_wall_shifted = best;
  const double needed = std::max(static_cast<double>(align.min_gain),
                                 align.min_gain_frac * static_cast<double>(n));
  out.believed = (best - out.on_wall_here) >= needed;
  return out;
}

int mark_fresh(nav2_costmap_2d::Costmap2D & grid, const std::vector<Point> & points,
               const std::vector<double> & ranges, double margin_m, double per_m)
{
  return mark_fresh(grid, points, ranges, margin_m, per_m, Point{0.0, 0.0}, Align{});
}

int mark_fresh(nav2_costmap_2d::Costmap2D & grid, const std::vector<Point> & points,
               const std::vector<double> & ranges, double margin_m, double per_m,
               const Point & origin, const Align & align, Shift * found)
{
  // **Decided against the map as it stands before any of them is marked.**
  // Marking as it went, the first point on a person would make every point
  // beside it look like "near something already lethal" and the rest of the
  // person would be skipped as a wall.
  const Shift shift = align.window_m > 0.0 ? best_shift(grid, points, origin, align) : Shift{};
  if (found != nullptr) {
    *found = shift;
  }
  const double c = std::cos(shift.dyaw);
  const double s = std::sin(shift.dyaw);
  std::vector<std::pair<unsigned int, unsigned int>> fresh;
  fresh.reserve(points.size());
  const double resolution = grid.getResolution();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto & point = points[i];
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!grid.worldToMap(point.first, point.second, mx, my)) {
      continue;
    }
    const double range = i < ranges.size() ? ranges[i] : 0.0;
    const int wall_cells = wall_cells_for(margin_m, per_m, range, resolution);
    // Judged where the scan fits, which is the believed pose unless the shift
    // was believed instead -- and then *only* at the shift, because the pose is
    // the thing known to be wrong: judged at both, a person 40 cm from a
    // doorway's wall seen 20 cm off would be the wall at the pose.
    unsigned int jx = mx;
    unsigned int jy = my;
    if (shift.believed) {
      const double px = point.first - origin.first;
      const double py = point.second - origin.second;
      unsigned int sx = 0;
      unsigned int sy = 0;
      if (grid.worldToMap(origin.first + c * px - s * py + shift.dx,
                          origin.second + s * px + c * py + shift.dy, sx, sy))
      {
        jx = sx;
        jy = sy;
      }
    }
    const bool mapped = mapped_near(grid, static_cast<int>(jx), static_cast<int>(jy), wall_cells);
    if (!mapped) {
      fresh.emplace_back(mx, my);
    }
  }
  int marked = 0;
  for (const auto & cell : fresh) {
    if (grid.getCost(cell.first, cell.second) != nav2_costmap_2d::LETHAL_OBSTACLE) {
      grid.setCost(cell.first, cell.second, nav2_costmap_2d::LETHAL_OBSTACLE);
      ++marked;
    }
  }
  return marked;
}

bool bounds_of(const std::vector<Point> & points, double & min_x, double & min_y,
               double & max_x, double & max_y)
{
  if (points.empty()) {
    return false;
  }
  min_x = min_y = std::numeric_limits<double>::max();
  max_x = max_y = std::numeric_limits<double>::lowest();
  for (const auto & point : points) {
    min_x = std::min(min_x, point.first);
    min_y = std::min(min_y, point.second);
    max_x = std::max(max_x, point.first);
    max_y = std::max(max_y, point.second);
  }
  return true;
}

}  // namespace live_layer

void LiveObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("scan_topic", rclcpp::ParameterValue(topic_));
  declareParameter("max_range_m", rclcpp::ParameterValue(max_range_m_));
  declareParameter("min_range_m", rclcpp::ParameterValue(min_range_m_));
  declareParameter("max_age_s", rclcpp::ParameterValue(max_age_s_));
  declareParameter("wall_margin_m", rclcpp::ParameterValue(wall_margin_m_));
  declareParameter("wall_margin_per_m", rclcpp::ParameterValue(wall_margin_per_m_));
  declareParameter("align_window_m", rclcpp::ParameterValue(align_.window_m));
  declareParameter("align_window_deg", rclcpp::ParameterValue(align_.window_deg));
  declareParameter("align_min_gain", rclcpp::ParameterValue(align_.min_gain));
  declareParameter("align_min_gain_frac", rclcpp::ParameterValue(align_.min_gain_frac));
  node->get_parameter(getFullName("enabled"), active_);
  node->get_parameter(getFullName("scan_topic"), topic_);
  node->get_parameter(getFullName("max_range_m"), max_range_m_);
  node->get_parameter(getFullName("min_range_m"), min_range_m_);
  node->get_parameter(getFullName("max_age_s"), max_age_s_);
  node->get_parameter(getFullName("wall_margin_m"), wall_margin_m_);
  node->get_parameter(getFullName("wall_margin_per_m"), wall_margin_per_m_);
  node->get_parameter(getFullName("align_window_m"), align_.window_m);
  node->get_parameter(getFullName("align_window_deg"), align_.window_deg);
  node->get_parameter(getFullName("align_min_gain"), align_.min_gain);
  node->get_parameter(getFullName("align_min_gain_frac"), align_.min_gain_frac);

  // The layer as the costmap sees it is always on, and the switch is ours:
  // a layer the costmap thinks is off is never asked for its bounds again, so
  // the marks it made last would stay on the grid until something else
  // happened to cover them.
  enabled_ = true;
  current_ = true;

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    topic_, rclcpp::SensorDataQoS(),
    std::bind(&LiveObstacleLayer::onScan, this, std::placeholders::_1), options);
  on_parameters_ = node->add_on_set_parameters_callback(
    std::bind(&LiveObstacleLayer::onParameters, this, std::placeholders::_1));
  RCLCPP_INFO(
    logger_, "%s: marking what %s sees within %.1f m and the map has not, "
    "%s; nothing is kept between updates", name_.c_str(), topic_.c_str(),
    max_range_m_, active_ ? "on" : "off");
}

void LiveObstacleLayer::onScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  std::lock_guard<std::mutex> lock(mutex_);
  scan_ = scan;
}

rcl_interfaces::msg::SetParametersResult LiveObstacleLayer::onParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // Every parameter of the costmap comes through here, other layers' too, so
  // anything not ours is let through untouched rather than refused.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == getFullName("enabled") &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
    {
      active_ = parameter.as_bool();
      RCLCPP_INFO(logger_, "%s switched %s", name_.c_str(), active_ ? "on" : "off");
    } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == getFullName("max_range_m")) {
        max_range_m_ = parameter.as_double();
      } else if (name == getFullName("min_range_m")) {
        min_range_m_ = parameter.as_double();
      } else if (name == getFullName("max_age_s")) {
        max_age_s_ = parameter.as_double();
      } else if (name == getFullName("wall_margin_m")) {
        wall_margin_m_ = parameter.as_double();
      } else if (name == getFullName("wall_margin_per_m")) {
        wall_margin_per_m_ = parameter.as_double();
      } else if (name == getFullName("align_window_m")) {
        align_.window_m = parameter.as_double();
      } else if (name == getFullName("align_window_deg")) {
        align_.window_deg = parameter.as_double();
      } else if (name == getFullName("align_min_gain_frac")) {
        align_.min_gain_frac = parameter.as_double();
      }
    } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
      name == getFullName("align_min_gain"))
    {
      align_.min_gain = static_cast<int>(parameter.as_int());
    }
  }
  return result;
}

void LiveObstacleLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  bool active = false;
  double max_range = 0.0;
  double min_range = 0.0;
  double max_age = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scan = scan_;
    active = active_;
    max_range = max_range_m_;
    min_range = min_range_m_;
    max_age = max_age_s_;
  }

  std::vector<Point> points;
  std::vector<double> ranges;
  Point origin{0.0, 0.0};
  if (active && scan) {
    const double age = (clock_->now() - rclcpp::Time(scan->header.stamp, clock_->get_clock_type()))
      .seconds();
    if (age <= max_age) {
      // The transform as it is now, in the frame the costmap is in now: that
      // is what makes these marks land where the map is, however the last
      // loop closure moved it.
      try {
        const auto transform = tf_->lookupTransform(
          layered_costmap_->getGlobalFrameID(), scan->header.frame_id, tf2::TimePointZero);
        const double tx = transform.transform.translation.x;
        const double ty = transform.transform.translation.y;
        // Yaw from the quaternion directly: tf2::getYaw on a message type
        // needs tf2_geometry_msgs's conversions, which this library does not
        // otherwise link.
        const auto & q = transform.transform.rotation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        origin = Point{tx, ty};
        const double nearest = std::max(min_range, static_cast<double>(scan->range_min));
        const double furthest = std::min(max_range, static_cast<double>(scan->range_max));
        points.reserve(scan->ranges.size());
        ranges.reserve(scan->ranges.size());
        for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
          const double range = scan->ranges[i];
          if (!std::isfinite(range) || range < nearest || range > furthest) {
            continue;
          }
          const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
          const double lx = range * std::cos(angle);
          const double ly = range * std::sin(angle);
          points.emplace_back(tx + c * lx - s * ly, ty + s * lx + c * ly);
          ranges.push_back(range);
        }
      } catch (const tf2::TransformException & error) {
        RCLCPP_DEBUG(logger_, "%s: no transform for the scan (%s)", name_.c_str(), error.what());
        points.clear();
        ranges.clear();
      }
    }
  }

  // Where it marked last time is always inside the bounds, so that the
  // costmap's reset of them takes those marks away before anything is painted.
  if (marked_before_) {
    *min_x = std::min(*min_x, before_min_x_);
    *min_y = std::min(*min_y, before_min_y_);
    *max_x = std::max(*max_x, before_max_x_);
    *max_y = std::max(*max_y, before_max_y_);
  }
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
  const bool have = live_layer::bounds_of(points, x0, y0, x1, y1);
  if (have) {
    *min_x = std::min(*min_x, x0);
    *min_y = std::min(*min_y, y0);
    *max_x = std::max(*max_x, x1);
    *max_y = std::max(*max_y, y1);
  }
  marked_before_ = have;
  before_min_x_ = x0;
  before_min_y_ = y0;
  before_max_x_ = x1;
  before_max_y_ = y1;
  std::lock_guard<std::mutex> lock(mutex_);
  points_ = std::move(points);
  ranges_ = std::move(ranges);
  origin_ = origin;
}

void LiveObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  std::vector<Point> points;
  std::vector<double> ranges;
  Point origin{0.0, 0.0};
  double margin = 0.0;
  double per_m = 0.0;
  live_layer::Align align;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    points = points_;
    ranges = ranges_;
    origin = origin_;
    margin = wall_margin_m_;
    per_m = wall_margin_per_m_;
    align = align_;
  }
  if (points.empty()) {
    return;
  }
  live_layer::Shift shift;
  live_layer::mark_fresh(master_grid, points, ranges, margin, per_m, origin, align, &shift);
  // Said once when the scan starts lying better off the pose than on it and
  // once when it stops: it is the rover's place on the map being wrong, which
  // the planner is now working round rather than drawing as obstacles.
  if (shift.believed != said_shift_) {
    said_shift_ = shift.believed;
    if (shift.believed) {
      RCLCPP_INFO(
        logger_, "%s: the scan lies %.0f cm and %.1f degrees off the map (%d returns on "
        "walls there against %d here); what the map has there is not marked",
        name_.c_str(), 100.0 * std::hypot(shift.dx, shift.dy), shift.dyaw * 180.0 / M_PI,
        shift.on_wall_shifted, shift.on_wall_here);
    } else {
      RCLCPP_INFO(logger_, "%s: the scan lies on the map where the rover thinks it is again",
                  name_.c_str());
    }
  }
}

}  // namespace ugv_behaviors

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ugv_behaviors::LiveObstacleLayer, nav2_costmap_2d::Layer)
