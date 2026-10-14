"""How far away a region is by the lidar, where the depth camera could not say.

**Half of all regions have no range, and a region with no range can be filed to
the wrong thing by its bearing alone.** The depth camera sees 65 degrees of the
fisheye's 130, and it sleeps when the rover has stood still for half a minute.
In M5 session 1 of 2026-10-14 five looks at the armchair with the throw, taken
from the stop by the balcony door with the depth camera asleep, were filed to a
record placed 0.95 m along their line of sight, beside the other armchair. The
armchair they showed was 2.7 m away. A range would have refused every one
(docs/progress/2026-10-14-m5-session-1-continued.md).

The lidar sweeps all round, about 20 cm above the floor and 3.5 cm under the
gimbal camera, so it speaks for things that reach down into its plane: a region
whose lower edge lies no more than `REACH_DEG` above the horizontal
(`reaches_down`). Its range is the 25th percentile of the lidar's returns across
the middle half of the region's width, never narrower than `MIN_HALF_DEG` either
side. On M5 sessions 1 and 2 of 2026-10-14, 173 regions had both: 51% landed
within 0.3 m of the depth camera's own range for the same region, 85% within
1 m, and 15% were more than 0.8 m short, where something nearer stood in the
lidar's plane. A rule on the box's place in the picture did as well on the
looks tilted up and could not see the level ones, which is where the five
armchair looks were taken. So it is kept as a range with `SIGMA_M` of
uncertainty, read from the scan navigation's position check measured the look's
pose with (`nav_map.measure_pose`), and only for a look taken standing still.
The store calls it `lidar` in `range_from`.
"""
from __future__ import annotations

import math
from typing import Any

#: How far above the horizontal a region's lower edge may lie and still reach the
#: lidar's plane, in degrees: the plane is under a degree below the camera's
#: horizontal across a room, and an elevation is good to 2.2 degrees.
REACH_DEG = 2.0
#: The narrowest half-width read, in degrees either side of a region's middle.
MIN_HALF_DEG = 1.5
#: Which of the returns across the region is its range.
PERCENTILE = 25
#: How far a lidar range may be out, in metres.
SIGMA_M = 0.4
#: Returns nearer or farther than these are not read.
MIN_M = 0.15
MAX_M = 6.0
#: What the store's `range_from` calls a range read this way.
METHOD = "lidar"


def reaches_down(lower_edge_deg) -> bool:
    """A region whose lower edge, in degrees above the horizontal with the
    tilt's bias out, comes down to the lidar's plane."""
    try:
        return float(lower_edge_deg) <= REACH_DEG
    except (TypeError, ValueError):
        return False


def _percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    at = (len(ordered) - 1) * q / 100.0
    low = int(math.floor(at))
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (at - low)


def range_along(scan: dict[str, Any], left_deg: float, span_deg: float) -> float | None:
    """The lidar's range across a region `left_deg` left of the rover's nose and
    `span_deg` wide, or None where it returned nothing there."""
    try:
        start = float(scan["angle_min"])
        step = float(scan["angle_increment"])
        ranges = scan["ranges"]
    except (KeyError, TypeError, ValueError):
        return None
    if not step or not ranges:
        return None
    half = max(MIN_HALF_DEG, (float(span_deg) if span_deg else 0.0) / 2.0)
    got = []
    for index, value in enumerate(ranges):
        if value is None:
            continue
        angle = math.degrees(start + step * index)
        off = (angle - left_deg + 180.0) % 360.0 - 180.0
        if abs(off) <= half and MIN_M < float(value) < MAX_M:
            got.append(float(value))
    if not got:
        return None
    return round(_percentile(got, PERCENTILE), 3)
