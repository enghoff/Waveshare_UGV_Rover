# The orange marks along the walls: the map's own walls, seen off by the rover's heading

**The planner's live layer marked walls and furniture the map already had, 10 to
30 cm off, because the rover's heading on the map was out by a few degrees, and
by about 14 degrees after it had been moved by hand.** Standing still by the
charger, 36% of the lidar's returns within 3 m lay more than 10 cm from a mapped
wall and the layer kept 56 cells marked; a navigation restart refitted the pose,
the same rover in the same place then had 1% of its returns that far out, and the
old rule would have marked about one cell a scan. The scan's timing, which the
layer's code made the obvious suspect, was not the cause. The layer's wall
margin now grows with range (0.10 m plus 0.087 m a metre, 5 degrees), deployed at
c46eeeb; with the refitted pose the layer marks nothing standing there. A person
in open floor or in a doorway's middle is still marked at any range.

## What the marks were

`experiments/live_layer/scan_record.py` records every scan with the transform the
layer used for it and the transform at the scan's own time, beside the cells the
layer had marked (`nav_grid`'s `live`, 2 Hz). `replay.py` recomputes the marks.
It explained 263 of the rover's 264 marks parked, 98% of 25,246 over run 3's drive
(`run/b21221da/1`), and all 1,130 in the parked check below, so it is the layer.

- **Parked, 11:30**: 35 distinct cells, each seen in nearly every sample, all
  0.11 to 0.32 m from a mapped wall or piece of furniture: the cabinet's front,
  the armchair, the dining chairs' legs.
- **Driving, 11:49-11:54**: returns more than 10 cm from a mapped wall were 7%
  within a metre, 29% at 1-2 m and 47% at 2-3 m. An error that grows with range
  is one of angle.
- **Not the timing.** The layer places a scan with the transform as it is when
  the costmap updates, up to 0.5 s later. Placed instead where the laser was when
  it scanned, the marks per scan were 37.3 against 36.3 turning and 49.3 either
  way standing: the cells move by one or two, the count does not.
- **The wall's angle.** Fitting the cabinet wall in the scans and comparing it
  with the mapped wall gave -0.8 degrees median, -5 to +6.5 degrees at the 10th
  and 90th percentiles, over 29 scans of the drive.
- **The parked rover after it was put on the charger, 12:05**: 56 marks a sample,
  36% of returns beyond 10 cm. After the navigation restart for the deploy,
  without the rover moving, the pose came back 14.5 degrees round and 5 cm over,
  1% of returns beyond 10 cm, and `map_drift` "agrees, within 3 cm and 0.0
  degrees". The owner had turned the rover to plug it in; a turn made with the
  wheels still is the case [R-NAV-13](../requirements/navigation.md#r-nav-13)
  says dead reckoning does not measure. Whether the parked check of
  [R-NAV-14](../requirements/navigation.md#r-nav-14) had reported it is not
  known: the restart replaced the log and the pre-run check did not read
  `map_drift`.

## The change

A point is left alone when a mapped wall cell is within `wall_margin_m +
wall_margin_per_m x range`. Replayed on the drive's recording: 58.9 marks a scan
before, 10.9 with 5 degrees (20.8 with 3; cutting the range to 2 m gives 21.0 but
stops the planner seeing a person 2-3 m away). The C++ test
(`test_live_layer.cpp`) has the old margin mark a wall seen 0.2 m off at 3 m and
the new one leave it; a person 0.5 m from that wall at 3 m and a point 0.2 m from
it at 0.5 m are still marked.

Deployed at c46eeeb (ros_nav, built and tested on the rover; navigation
restarted, position trusted and map settled after); the running costmap reads
`wall_margin_per_m` 0.087. Parked after: no cells marked in 21 samples, against
56 before -- most of that the refit, not the margin, which on the refitted pose
took the old rule's 1.2 a scan to none.

## What it means

The marks were not noise. They were the rover saying its heading disagreed with
the map, and a large patch of them along walls still will: 5 degrees covers the
everyday wander, not a rover turned by hand. A run should not open on such a
pose. Reading `map_drift` before opening one is the cheap first step, proposed
here and not built; this session's pre-checks did not.
