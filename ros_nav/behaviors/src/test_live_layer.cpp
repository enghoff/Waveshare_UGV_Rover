// The live obstacle layer's arithmetic, on a costmap with no node behind it:
// a person in open floor is marked, every cell of them; the map's own wall seen
// a few centimetres off is not.
#include <iostream>
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

  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
  check("no points, no bounds", !live::bounds_of({}, x0, y0, x1, y1));
  live::bounds_of(points, x0, y0, x1, y1);
  check("the bounds take in every point",
        x0 == 0.52 && y0 == 0.52 && x1 == 1.07 && y1 == 1.08);
  return ok ? 0 : 1;
}
