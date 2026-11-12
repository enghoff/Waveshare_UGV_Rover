"""What a person driving with the game controller is allowed to send the wheels.

No ROS in here, so the selftest can run it on a workstation; `nav_manual.py`
calls it with every manual command, against the newest `/scan`.

**The rule: nothing within STOP_M of the rover in the way it is driving.** Ahead
of the front edge while going forward, behind the rear edge while reversing, and
only inside the rover's own width -- something beside it is not in its way, and
something behind it never stops it going forward. Turning on the spot is never
cut, so a rover stopped at a wall can always be turned away from it.

**STOP_M is where it stops, not where it starts braking.** The command is cut
REACTION_S of travel before that line: the scan can be a tenth of a second old,
the controller's loop and the bridge add a little, and the wheels coast. And the
speed used for that is never below the chassis' slowest real speed, because a
gentle squeeze of the trigger still drives at 0.33 m/s (drive_mixer.py).

**The body is not measured.** `base_link` is the lidar, and how far the chassis
runs in front of it and behind it has never been taped -- `config/nav2.yaml`
says as much about `robot_radius`. Both are taken as the 0.20 m the costmap's
circle uses, which over-estimates them, so until they are measured the rover
stops a little early rather than late. The width is measured: 34 cm
(docs/reference/d500-lidar.md).
"""

import math

#: How close anything may get in the direction of travel, from the body's edge.
STOP_M = 0.20
#: The lidar to the chassis' front and rear edges. Not measured; see above.
BODY_FRONT_M = 0.20
BODY_BACK_M = 0.20
#: Half the chassis' 34 cm, and a little either side of it.
BODY_HALF_WIDTH_M = 0.17
CORRIDOR_MARGIN_M = 0.03
#: Travel allowed for between a return being seen and the wheels stopping.
REACTION_S = 0.30
#: The slowest the chassis actually drives, which a light trigger still gets.
MIN_DRIVE_MS = 0.33
#: A scan older than this is no scan: driving is cut, turning is not.
SCAN_STALE_S = 0.5


def clearance(ranges, angle_min, angle_increment, range_min, range_max,
              half_width=BODY_HALF_WIDTH_M + CORRIDOR_MARGIN_M):
    """The nearest return in front of the front edge and behind the rear edge.

    Only returns inside the rover's own width count. Metres from the edge, None
    when there is nothing in range that way. A return inside the assumed body is
    0: nothing outside the rover can be there, so it is either something very
    close or a body longer than assumed, and both are reasons not to drive at it.
    """
    front = rear = math.inf
    for index, distance in enumerate(ranges):
        if not (math.isfinite(distance) and range_min <= distance <= range_max):
            continue
        angle = angle_min + index * angle_increment
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        if abs(y) > half_width:
            continue
        if x >= 0.0:
            front = min(front, max(0.0, x - BODY_FRONT_M))
        else:
            rear = min(rear, max(0.0, -x - BODY_BACK_M))
    return (None if math.isinf(front) else front,
            None if math.isinf(rear) else rear)


def stop_at(linear):
    """How near something may be before a command at `linear` m/s is cut."""
    return STOP_M + max(abs(linear), MIN_DRIVE_MS) * REACTION_S


def gate(linear, angular, front, rear, scan_age):
    """The command that may go to the wheels, and why any of it was cut.

    `front` and `rear` are `clearance`'s answers; `scan_age` is how old the scan
    behind them is, or None when there has been none.
    """
    if not linear:
        return 0.0, angular, ""
    if scan_age is None or scan_age > SCAN_STALE_S:
        return 0.0, angular, "no fresh scan"
    if linear > 0 and front is not None and front <= stop_at(linear):
        return 0.0, angular, "ahead"
    if linear < 0 and rear is not None and rear <= stop_at(linear):
        return 0.0, angular, "behind"
    return linear, angular, ""
