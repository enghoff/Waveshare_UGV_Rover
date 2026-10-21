// The live obstacle layer's arithmetic, on a costmap with no node behind it:
// a person in open floor is marked, every cell of them; the map's own wall seen
// a few centimetres off is not, nor seen 20 cm off by a rover that is.
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "ugv_behaviors/live_obstacle_layer.hpp"

int main()
{
  using nav2_costmap_2d::FREE_SPACE;
  using nav2_costmap_2d::LETHAL_OBSTACLE;
  namespace live = ugv_behaviors::live_layer;
  bool ok = true;
  auto check = [&ok](const char * what, bool got) {
      std::cout << (got ? "  ok   " : "  FAIL ") << what << std::endl;
      ok = ok && got;
    };

  // Two metres square at 5 cm, with a mapped wall along y = 1.0 m.
  nav2_costmap_2d::Costmap2D grid(40, 40, 0.05, 0.0, 0.0, FREE_SPACE);
  for (unsigned int x = 0; x < 40; ++x) {
    grid.setCost(x, 20, LETHAL_OBSTACLE);
  }
  const std::vector<ugv_behaviors::Point> points = {
    {0.52, 1.02},                              // on the wall
    {0.52, 1.08},                              // the wall, seen 6 cm off
    {1.02, 0.52}, {1.07, 0.52}, {1.02, 0.57},  // a person, three cells together
  };
  const int marked = live::mark_fresh(grid, points, 2);
  check("a person in open floor is marked, every cell of them", marked == 3);
  unsigned int mx = 0;
  unsigned int my = 0;
  grid.worldToMap(1.07, 0.52, mx, my);
  check("...including the cell beside one just marked",
        grid.getCost(mx, my) == LETHAL_OBSTACLE);
  grid.worldToMap(0.52, 1.08, mx, my);
  check("the map's own wall seen 6 cm off is left alone",
        grid.getCost(mx, my) == FREE_SPACE);
  check("a point off the grid is ignored, not a crash",
        live::mark_fresh(grid, {{5.0, 5.0}}, 2) == 0);

  // **The margin grows with range** (2026-10-09). Four metres square, a wall
  // along y = 2.0 m, the default 0.10 m plus 0.087 m a metre (5 degrees).
  {
    nav2_costmap_2d::Costmap2D room(80, 80, 0.05, 0.0, 0.0, FREE_SPACE);
    for (unsigned int x = 0; x < 80; ++x) {
      room.setCost(x, 40, LETHAL_OBSTACLE);
    }
    const std::vector<ugv_behaviors::Point> far_wall = {{1.0, 2.22}};
    check("the old fixed margin marks a wall seen 0.2 m off",
          live::mark_fresh(room, far_wall, 2) == 1);
    room.setCost(20, 44, FREE_SPACE);
    check("...and the margin for 3 m of range leaves it alone",
          live::mark_fresh(room, far_wall, {3.0}, 0.10, 0.087) == 0);
    check("a person half a metre from that wall at 3 m is still marked",
          live::mark_fresh(room, {{2.0, 1.47}}, {3.0}, 0.10, 0.087) == 1);
    check("a point 0.2 m from the wall at 0.5 m range is still marked",
          live::mark_fresh(room, {{3.0, 1.82}}, {0.5}, 0.10, 0.087) == 1);
  }

  // **A rover 20 cm off in a doorway** (2026-10-21). Walls along y = 1.0 and
  // 1.8 m and an end wall at x = 3.5 m; the laser really at (1.0, 1.6), believed
  // at (1.0, 1.4). Every return is the map's own wall, and the old rule drew a
  // line of them across the doorway.
  {
    nav2_costmap_2d::Costmap2D door(80, 80, 0.05, 0.0, 0.0, FREE_SPACE);
    for (unsigned int x = 0; x <= 70; ++x) {
      door.setCost(x, 20, LETHAL_OBSTACLE);
      door.setCost(x, 36, LETHAL_OBSTACLE);
    }
    for (unsigned int y = 20; y <= 36; ++y) {
      door.setCost(70, y, LETHAL_OBSTACLE);
    }
    const ugv_behaviors::Point truth{1.0, 1.6};
    const ugv_behaviors::Point believed{1.0, 1.4};
    std::vector<ugv_behaviors::Point> walls;
    for (double x = 0.2; x <= 3.4; x += 0.05) {
      walls.push_back({x, 1.02});
      walls.push_back({x, 1.82});
    }
    for (double y = 1.06; y <= 1.78; y += 0.05) {
      walls.push_back({3.52, y});
    }
    // As the believed pose places them, with their true ranges.
    auto seen = [&](const std::vector<ugv_behaviors::Point> & at, const ugv_behaviors::Point & from) {
        std::vector<ugv_behaviors::Point> out;
        std::vector<double> ranges;
        for (const auto & p : at) {
          const double range = std::hypot(p.first - truth.first, p.second - truth.second);
          if (range > 3.0) {
            continue;
          }
          out.push_back({p.first - truth.first + from.first, p.second - truth.second + from.second});
          ranges.push_back(range);
        }
        return std::make_pair(out, ranges);
      };
    live::Align align;
    align.window_m = 0.40;
    {
      auto [points, ranges] = seen(walls, believed);
      nav2_costmap_2d::Costmap2D copy(door);
      check("the old rule marks the doorway's walls seen 20 cm off",
            live::mark_fresh(copy, points, ranges, 0.10, 0.087) > 5);
      nav2_costmap_2d::Costmap2D again(door);
      live::Shift shift;
      check("...and laid on the map at the shift first, none of them",
            live::mark_fresh(again, points, ranges, 0.10, 0.087, believed, align, &shift) == 0);
      // To a cell: "on a wall" is within a cell of one, so 15, 20 and 25 cm tie
      // and the smallest is kept.
      check("...the shift found being the 20 cm the pose is out, to a cell",
            shift.believed && std::abs(shift.dy - 0.20) <= 0.05 + 1e-9 &&
            std::abs(shift.dx) < 1e-9 && shift.dyaw == 0.0);
    }
    {
      // A person in the doorway's middle, 0.4 m from both walls.
      std::vector<ugv_behaviors::Point> with = walls;
      const std::vector<ugv_behaviors::Point> legs = {{1.8, 1.40}, {1.85, 1.40}, {1.8, 1.45}};
      with.insert(with.end(), legs.begin(), legs.end());
      auto [points, ranges] = seen(with, believed);
      nav2_costmap_2d::Costmap2D copy(door);
      live::Shift shift;
      const int marked = live::mark_fresh(copy, points, ranges, 0.10, 0.087, believed, align,
                                          &shift);
      // The legs are the last three points, where the believed pose put them.
      int on_legs = 0;
      for (std::size_t i = points.size() - legs.size(); i < points.size(); ++i) {
        unsigned int lx = 0;
        unsigned int ly = 0;
        copy.worldToMap(points[i].first, points[i].second, lx, ly);
        on_legs += copy.getCost(lx, ly) == LETHAL_OBSTACLE;
      }
      check("a person in the doorway is still marked, where the rover sees them",
            on_legs == 3 && marked == 3 && shift.believed);
      if (on_legs != 3 || marked != 3 || !shift.believed) {
        std::cout << "       legs " << on_legs << " marked " << marked << " shift "
                  << shift.dx << "," << shift.dy << "," << shift.dyaw << " believed "
                  << shift.believed << " on walls " << shift.on_wall_here << "->"
                  << shift.on_wall_shifted << std::endl;
      }
    }
    {
      // The pose right: nothing to shift, and a person is marked as before.
      std::vector<ugv_behaviors::Point> with = walls;
      with.push_back({1.8, 1.40});
      auto [points, ranges] = seen(with, truth);
      nav2_costmap_2d::Costmap2D copy(door);
      live::Shift shift;
      const int marked = live::mark_fresh(copy, points, ranges, 0.10, 0.087, truth, align, &shift);
      check("with the pose right the shift is not believed", !shift.believed);
      check("...and only the person is marked", marked == 1);
    }
    {
      align.window_m = 0.0;
      auto [points, ranges] = seen(walls, believed);
      nav2_costmap_2d::Costmap2D copy(door);
      check("a window of 0 is the old rule",
            live::mark_fresh(copy, points, ranges, 0.10, 0.087, believed, align) > 5);
    }
  }

  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
  check("no points, no bounds", !live::bounds_of({}, x0, y0, x1, y1));
  live::bounds_of(points, x0, y0, x1, y1);
  check("the bounds take in every point",
        x0 == 0.52 && y0 == 0.52 && x1 == 1.07 && y1 == 1.08);
  return ok ? 0 : 1;
}
