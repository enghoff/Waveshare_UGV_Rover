"""The gyro check in calibrate_chassis.py: against the walls, not against itself.

Until 2026-10-10 the check read slam_toolbox's `map` -> `base_link` through each
turning burst. slam_toolbox 2.8 takes no scan into its graph until the rover has
moved `minimum_travel_distance`, whatever it has turned, so on the spot that yaw
was odometry's plus a constant and the check found 1.0, 0.96, 1.01 and 1.03 for a
gyro twelve still fits put 6.2% long (captures/2026-10-10-turns). The rover here
turns 100 degrees for every 106.2 its gyro reports, and its mapper's yaw follows
the gyro exactly, as it did on the spot.
"""
from test_harness import check, section
from calibrate_chassis import Chassis

GYRO_LONG = 1.062
TRUE_TURN_DEG = 150.0


class OnTheSpot:
    """Enough of a Chassis for `measure_gyro`: a rover that turns on the spot."""

    def __init__(self):
        self.truth = 40.0             # degrees, what a still fit would say
        self.odom = -100.0            # degrees, what the gyro has integrated
        self.args = None

    def nearest_anything(self):
        return 1.0

    def spin_for(self, seconds):
        pass

    def yaw(self):
        return (self.odom + 180.0) % 360.0 - 180.0

    def map_yaw(self):
        # slam_toolbox on the spot: odometry's yaw plus a constant.
        return ((self.odom + 140.0) + 180.0) % 360.0 - 180.0

    def still_fit(self):
        return (self.truth + 180.0) % 360.0 - 180.0, None

    def burst(self, left, right, sample):
        direction = 1.0 if right > left else -1.0
        samples = []
        steps = 30
        for i in range(steps):
            # Most of the turn inside the sampled window, the rest is ramp and coast.
            step = direction * TRUE_TURN_DEG / steps
            self.truth += step
            self.odom += step * GYRO_LONG
            if 3 <= i < steps - 5:
                samples.append((i * 0.05, sample()))
        return samples, None


def test_the_gyro_is_checked_against_still_fits():
    section("calibrate_chassis: the gyro is checked against the walls, not itself")
    for direction, name in ((+1, "anticlockwise"), (-1, "clockwise")):
        rover = OnTheSpot()
        result, why = Chassis.measure_gyro(rover, 120, direction)
        check("%s: a burst is measured (%s)" % (name, why), result is not None)
        if result is None:
            continue
        odom_deg, map_deg = result
        check("%s: the walls say the true turn, start and coast included" % name,
              abs(map_deg), TRUE_TURN_DEG, tolerance=0.01)
        check("%s: the ratio is the gyro's error, not 1.0" % name,
              odom_deg / map_deg, GYRO_LONG, tolerance=0.001)


TESTS = (test_the_gyro_is_checked_against_still_fits,)
