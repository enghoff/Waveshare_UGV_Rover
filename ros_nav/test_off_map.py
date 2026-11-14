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
    """Feed `steps` as (searched, score); return what was said."""
    said = []
    for i, (searched, score) in enumerate(steps):
        now = start + 0.2 * i
        said.append(latch.step(manual, searched, score, HOME, now,
                               1700000000.0 + now))
    return [s for s in said if s]


MATCHED = (True, 0.9)
LOST = (True, 0.2)
SKIPPED = (False, 0.0)
#: What a parked rover's searched scans say: not accepted -- no correction was
#: taken -- and the map agrees at 82-87% (the rover, 2026-11-13).
PARKED = (True, 0.86)


def test_leaving_is_noticed_only_on_a_drive_by_hand():
    latch = o.OffMap()
    check("matching scans are the rover on its map",
          (feed(latch, [MATCHED] * 20), latch.off), ([], False))
    check("...and where it was last matched is kept", latch.left_at, HOME)
    check("a moment of clutter is not leaving",
          (feed(latch, [LOST] * (o.LEAVE_STEPS - 1) + [MATCHED]), latch.off),
          ([], False))
    check("scans not searched -- turning fast, too few returns -- say nothing",
          (feed(latch, [LOST] * 5 + [SKIPPED] * 30 + [LOST] * 4), latch.off),
          ([], False))
    check("a rover stopped under the controller, which the map agrees with, "
          "is on its map however long it stands",
          (feed(latch, [PARKED] * 60), latch.off), ([], False))
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
          feed(latch, [(True, o.BACK_SCORE - 0.05)] * 30), [])
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


def test_the_map_is_held_only_where_there_is_a_map_to_lose():
    held = o.hold_reason
    check("a person driving holds it", held(True, False, True, True, False),
          "manual driving")
    check("...and so does a drive of theirs that left the map, after it",
          held(False, True, True, True, False), "off the map")
    check("a kept map the rover's place on is not confirmed holds it -- a rover "
          "switched on somewhere else", held(False, False, True, False, False),
          "position unconfirmed")
    check("...as do the seconds before a restore has been checked",
          held(False, False, True, True, True), "position unconfirmed")
    check("a kept map the rover is confirmed on is not held",
          held(False, False, True, True, False), "")
    check("no kept map -- none saved, cleared, or still loading -- is mapped "
          "from scratch, confirmed or not",
          [held(False, False, False, settled, unchecked)
           for settled in (False, True) for unchecked in (False, True)],
          ["", "", "", ""])


def test_a_carried_rover_is_off_its_map_until_the_scan_shows_it_back():
    """2026-11-14: picked up and turned, twice in one evening. The base calls
    the gyro faulty while it lasts (base_node.recover); the bridge holds the map
    from that moment, whoever was or was not driving."""
    latch = o.OffMap()
    feed(latch, [MATCHED] * 5, manual=False)
    latch.carried(1234.0)
    check("a rover turned with neither track moving is off its map at once",
          (latch.off, latch.as_dict()["since"], latch.left_at), (True, 1234.0, HOME))
    check("...and stays so while its scans cannot be placed, nobody driving",
          (feed(latch, [LOST] * 40, manual=False), latch.off), ([], True))
    check("set down where the map agrees, it earns the wide search",
          feed(latch, [MATCHED] * o.BACK_STEPS, manual=False, start=50.0), ["check"])
    latch.checked(True)
    check("...which gives the map back", latch.off, False)
    feed(latch, [LOST] * o.LEAVE_STEPS)
    since = latch.as_dict()["since"]
    latch.carried(since + 60.0)
    check("picked up while already off the map, it has been off since it left",
          latch.as_dict()["since"], since)

    fault = {"gyro_fault": "the gyro reported the rover turning"}
    check("under a gyro fault a turn is refused, with the base's reason",
          o.gyro_refusal("turn", fault),
          "the rover cannot measure its own turning: the gyro reported the rover "
          "turning")
    check("...and so is everything that steers by the heading",
          [bool(o.gyro_refusal(op, fault)) for op in ("goto", "explore")],
          [True, True])
    check("...but not a straight drive, which goes by the tracks",
          o.gyro_refusal("drive", fault), "")
    check("with no fault, or no word from the base yet, nothing is refused",
          [o.gyro_refusal("turn", base) for base in ({"gyro_fault": ""}, {}, None)],
          ["", "", ""])


TESTS = (test_leaving_is_noticed_only_on_a_drive_by_hand,
         test_coming_back_is_shown_by_the_wide_search,
         test_the_map_is_held_only_where_there_is_a_map_to_lose,
         test_a_carried_rover_is_off_its_map_until_the_scan_shows_it_back)
