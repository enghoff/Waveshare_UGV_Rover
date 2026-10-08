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
  // **Decided against the map as it stands before any of them is marked.**
  // Marking as it went, the first point on a person would make every point
  // beside it look like "near something already lethal" and the rest of the
  // person would be skipped as a wall.
  const int width = static_cast<int>(grid.getSizeInCellsX());
  const int height = static_cast<int>(grid.getSizeInCellsY());
  std::vector<std::pair<unsigned int, unsigned int>> fresh;
  fresh.reserve(points.size());
  for (const auto & point : points) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!grid.worldToMap(point.first, point.second, mx, my)) {
      continue;
    }
    bool mapped = false;
    for (int dy = -wall_cells; dy <= wall_cells && !mapped; ++dy) {
      for (int dx = -wall_cells; dx <= wall_cells; ++dx) {
        if (dx * dx + dy * dy > wall_cells * wall_cells) {
          continue;
        }
        const int cx = static_cast<int>(mx) + dx;
        const int cy = static_cast<int>(my) + dy;
        if (cx < 0 || cy < 0 || cx >= width || cy >= height) {
          continue;
        }
        if (grid.getCost(cx, cy) == nav2_costmap_2d::LETHAL_OBSTACLE) {
          mapped = true;
          break;
        }
      }
    }
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
  node->get_parameter(getFullName("enabled"), active_);
  node->get_parameter(getFullName("scan_topic"), topic_);
  node->get_parameter(getFullName("max_range_m"), max_range_m_);
  node->get_parameter(getFullName("min_range_m"), min_range_m_);
  node->get_parameter(getFullName("max_age_s"), max_age_s_);
  node->get_parameter(getFullName("wall_margin_m"), wall_margin_m_);

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
      }
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
        const double nearest = std::max(min_range, static_cast<double>(scan->range_min));
        const double furthest = std::min(max_range, static_cast<double>(scan->range_max));
        points.reserve(scan->ranges.size());
        for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
          const double range = scan->ranges[i];
          if (!std::isfinite(range) || range < nearest || range > furthest) {
            continue;
          }
          const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
          const double lx = range * std::cos(angle);
          const double ly = range * std::sin(angle);
          points.emplace_back(tx + c * lx - s * ly, ty + s * lx + c * ly);
        }
      } catch (const tf2::TransformException & error) {
        RCLCPP_DEBUG(logger_, "%s: no transform for the scan (%s)", name_.c_str(), error.what());
        points.clear();
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
}

void LiveObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  std::vector<Point> points;
  double margin = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    points = points_;
    margin = wall_margin_m_;
  }
  if (points.empty()) {
    return;
  }
  const int wall_cells = static_cast<int>(std::ceil(margin / master_grid.getResolution()));
  live_layer::mark_fresh(master_grid, points, wall_cells);
}

}  // namespace ugv_behaviors

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ugv_behaviors::LiveObstacleLayer, nav2_costmap_2d::Layer)
