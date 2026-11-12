"""Leaving the map by hand, and coming back, against made-up localizer steps.

No ROS: `off_map.py` is a counter over the steps `localizer_node.py` publishes.
What matters is that only a person's drive is held, that a moment of clutter is
not leaving, that scans the localizer did not search say nothing, and that the
only way back is a run of good matches and then the wide search agreeing.
"""
import off_map as o
from test_harness import check

HOME = (1.0, 2.0, 90.0)


def feed(latch, steps, manual=True, start=0.0):
    """Feed `steps` as (searched, accepted, score); return what was said."""
    said = []
    for i, (searched, accepted, score) in enumerate(steps):
        now = start + 0.2 * i
        said.append(latch.step(manual, searched, accepted, score, HOME, now,
                               1700000000.0 + now))
    return [s for s in said if s]


MATCHED = (True, True, 0.9)
LOST = (True, False, 0.2)
STILL = (False, True, 0.0)


def test_leaving_is_noticed_only_on_a_drive_by_hand():
    latch = o.OffMap()
    check("matching scans are the rover on its map",
          (feed(latch, [MATCHED] * 20), latch.off), ([], False))
    check("...and where it was last matched is kept", latch.left_at, HOME)
    check("a moment of clutter is not leaving",
          (feed(latch, [LOST] * (o.LEAVE_STEPS - 1) + [MATCHED]), latch.off),
          ([], False))
    check("scans not searched -- standing still, turning fast -- say nothing",
          (feed(latch, [LOST] * 5 + [STILL] * 30 + [LOST] * 4), latch.off),
          ([], False))
    auto = o.OffMap()
    check("a drive the rover makes itself is not held: it maps as it goes",
          (feed(auto, [LOST] * 30, manual=False), auto.off), ([], False))
    said = feed(latch, [LOST] * o.LEAVE_STEPS)
    check("two seconds of scans that cannot be placed, by hand, is off the map",
          (said, latch.off), (["left"], True))
    check("...said with when, and where it was last on the map",
          (latch.as_dict()["since"] > 0, latch.as_dict()["left_at"]),
          (True, {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0}))
    check("a rover on its map says nothing", o.OffMap().as_dict(), None)


def test_coming_back_is_shown_by_the_wide_search():
    latch = o.OffMap()
    feed(latch, [LOST] * o.LEAVE_STEPS)
    check("matching again, but only just, earns nothing",
          feed(latch, [(True, True, o.BACK_SCORE - 0.05)] * 30), [])
    check("a run of good matches earns one wide search",
          feed(latch, [MATCHED] * (o.BACK_STEPS + 5), start=100.0), ["check"])
    check("...and is still off the map until it answers", latch.off, True)
    check("...and is not asked again while it runs",
          feed(latch, [MATCHED] * 20, start=102.0), [])
    latch.checked(False)
    check("a search that finds the rover elsewhere keeps it off",
          (latch.off, latch.hits), (True, 0))
    check("...and the next waits out the gap, however well it matches",
          feed(latch, [MATCHED] * 20, start=104.0), [])
    check("...then is asked again",
          feed(latch, [MATCHED] * 20, start=112.0 + o.CHECK_GAP_S), ["check"])
    latch.checked(True)
    check("a search that agrees puts it back on its map",
          (latch.off, latch.as_dict()), (False, None))
    check("...and it can leave again later",
          feed(latch, [LOST] * o.LEAVE_STEPS, start=200.0), ["left"])
    latch.clear()
    check("clearing forgets the lot",
          (latch.off, latch.misses, latch.hits, latch.checking), (False, 0, 0, False))


TESTS = (test_leaving_is_noticed_only_on_a_drive_by_hand,
         test_coming_back_is_shown_by_the_wide_search)
