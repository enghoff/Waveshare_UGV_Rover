"""A range is dropped when the rover turned between the picture and the depth frame.

On 2026-10-01 a tissue box 0.95 m away read 2.61 m. The look was taken while the
rover turned 54.5 degrees across the shutter, and the depth frame, fetched after
the encoders, stood about 17 degrees of turning away from the picture. The box
landed on the furniture behind, and that one look placed the box 1.77 m out with
0.17 m claimed.
"""
from __future__ import annotations

from test_harness import check
from world_state.depth_client import TURNING, Ranged
from world_state.inspection_ranges import (RANGE_TURN_LIMIT_DEG, SHUTTER_UNKNOWN_S,
                                           InspectionRanges)


class Ranges(InspectionRanges):
    def __init__(self, heading_now):
        self.heading_now = heading_now

    def _pose(self):
        if self.heading_now is None:
            return None
        return {"x_m": 0.0, "y_m": 0.0, "heading_deg": self.heading_now}


def ranged(age_s=0.12):
    return Ranged(range_m=0.95, sigma_m=0.03, age_s=age_s)


def test_a_rover_that_turned_away_from_the_depth_frame_loses_the_range() -> None:
    capture = {"shutter_heading_deg": 90.0, "turn_dps": 0.0, "taken_at": 100.0}
    found = [ranged(), None, Ranged(absent="outside the depth camera's view")]
    check("a rover standing still keeps its range",
          (Ranges(90.3)._drop_turned(found, capture, now=100.4),
           found[0].range_m), (0, 0.95))

    found = [ranged()]
    check("a rover that turned 5 degrees since the shutter loses it",
          (Ranges(95.0)._drop_turned(found, capture, now=100.4),
           found[0].absent, found[0].range_m), (1, TURNING, None))

    found = [ranged()]
    check("across the half-circle the turn is counted the short way",
          Ranges(-179.5)._drop_turned(
              found, dict(capture, shutter_heading_deg=179.8), now=100.4), 0)

    # The tissue box: 136 deg/s across the shutter, a depth frame 0.12 s old read
    # 0.5 s after the picture, so 0.38 s apart and some 51 degrees of turning.
    found = [ranged(0.123)]
    turning = {"shutter_heading_deg": None, "turn_dps": 136.0, "taken_at": 100.0}
    check("a fast turn across the shutter drops a range read after it",
          (Ranges(None)._drop_turned(found, turning, now=100.5),
           found[0].absent), (1, TURNING))

    found = [ranged(0.123)]
    check("a slow one inside the limit keeps it",
          Ranges(None)._drop_turned(
              found, dict(turning, turn_dps=RANGE_TURN_LIMIT_DEG / 0.5),
              now=100.5), 0)

    found = [ranged(None)]
    check("a range whose age is unknown is judged on the heading alone",
          Ranges(None)._drop_turned(found, turning, now=100.5), 0)


def test_a_depth_frame_of_the_pictures_moment_survives_ordinary_turning() -> None:
    """Asked for at the shutter and turned back by the turn, a range is dropped only
    when the turn is too fast for the shutter's own unknown moment -- 33 degrees a
    second -- rather than whenever the rover was turning."""
    matched = lambda: Ranged(range_m=0.95, sigma_m=0.03, age_s=0.4, off_s=0.02)
    moderate = {"shutter_heading_deg": 90.0, "turn_dps": 29.0, "taken_at": 100.0}
    found = [matched()]
    check("at the median turn rate of a drive, a matched range is kept",
          (Ranges(110.0)._drop_turned(found, moderate, now=100.5), found[0].range_m),
          (0, 0.95))
    found = [Ranged(range_m=0.95, sigma_m=0.03, age_s=0.12)]
    check("...where the newest frame read half a second later is dropped",
          Ranges(110.0)._drop_turned(found, moderate, now=100.5), 1)
    fast = dict(moderate, turn_dps=1.5 * RANGE_TURN_LIMIT_DEG / SHUTTER_UNKNOWN_S)
    found = [matched()]
    check("a turn too fast for the shutter's unknown moment drops it all the same",
          (Ranges(110.0)._drop_turned(found, fast, now=100.5), found[0].absent),
          (1, TURNING))
    check("the rule's own number is the shutter's", SHUTTER_UNKNOWN_S, 0.03)


TESTS = (test_a_rover_that_turned_away_from_the_depth_frame_loses_the_range,
         test_a_depth_frame_of_the_pictures_moment_survives_ordinary_turning)
