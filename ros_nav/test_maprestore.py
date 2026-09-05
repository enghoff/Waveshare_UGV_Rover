"""Waking up on the map the rover was switched off with.

`nav_map.load_graph` is the one place that decides whether a restore worked, and
it decides it by watching the transform tree rather than by asking: the
deserialise service on this rover answers with nothing at all, so a graph that
could not be read and a mapper that would not anchor look exactly like a success
from the caller. What separates them is whether the rover's own pose arrives
where the map says it was parked.

**That is the right test and it was on too short a fuse.** The module needs
`slam_toolbox` to import, so what runs here is the real function with a stand-in
node under it -- a fake mapper, a fake clock and a pose that appears when the
test says it does. Everything the decision turns on is in that loop, and none of
it needs a radio.
"""
import sys
import types

from test_harness import check, section


def _slam_toolbox():
    """The two service types `nav_map` imports, and nothing else.

    Stubbed rather than skipped. Skipping is what left this function untested on
    every machine that is not the rover, which is every machine anybody develops
    on, and the decision it makes is arithmetic over a clock.
    """
    if "slam_toolbox.srv" in sys.modules:
        return

    class Request:
        START_AT_GIVEN_POSE = 3

        def __init__(self):
            self.filename = ""
            self.match_type = 0
            self.initial_pose = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)

    package = types.ModuleType("slam_toolbox")
    srv = types.ModuleType("slam_toolbox.srv")
    for name in ("DeserializePoseGraph", "SerializePoseGraph"):
        setattr(srv, name, type(name, (), {"Request": Request}))
    package.srv = srv
    sys.modules["slam_toolbox"] = package
    sys.modules["slam_toolbox.srv"] = srv


_slam_toolbox()

import threading                                            # noqa: E402

import nav_map                                              # noqa: E402


class Mapper:
    """A node with just enough on it for `load_graph`, and a clock it obeys.

    `poses` is what the transform tree answers, one entry per look: None for a
    tree that is not publishing yet, a pose for one that is. The clock advances
    by `tick` on every sleep, so a wait of a minute costs nothing to run.
    """

    def __init__(self, poses, tick=0.1, answers=True):
        self.poses = list(poses)
        self.tick = tick
        self.answers = answers
        self.clock = 0.0
        self.looks = 0
        self.map_lock = threading.RLock()
        self._lock = threading.Lock()
        self.trail = types.SimpleNamespace(cleared=lambda *a: None)
        self.saved = types.SimpleNamespace(stem="/tmp/current")
        self.deserialize_client = types.SimpleNamespace(
            wait_for_service=lambda timeout_sec=None: True,
            call_async=lambda request: object())

    # --- what nav_map calls -------------------------------------------------
    def wait(self, _future, _limit_s):
        return self.answers

    def correction(self):
        return None

    def dead_reckoned(self):
        return None

    def pose_deg(self):
        self.looks += 1
        return self.poses[min(self.looks - 1, len(self.poses) - 1)]


def _restore(poses, **fields):
    """`load_graph` run against that stand-in, with time made to pass."""
    node = Mapper(poses, **fields)
    real_monotonic, real_sleep = nav_map.time.monotonic, nav_map.time.sleep

    def sleep(seconds):
        node.clock += seconds

    nav_map.time.monotonic = lambda: node.clock
    nav_map.time.sleep = sleep
    try:
        return nav_map.NavMap.load_graph(node, (2.0, 1.0, 30.0), drop_trail=True)
    finally:
        nav_map.time.monotonic, nav_map.time.sleep = real_monotonic, real_sleep


#: Where the map says the rover was parked, and where it actually lands -- a
#: little short of it, because the mapper matches the next scan within its own
#: correlation window and never returns exactly what it was handed.
PARKED = (2.0, 1.0, 30.0)
LANDED = (2.05, 0.94, 31.5)


def test_a_restore_that_lands_is_a_restore() -> None:
    """The ordinary case, and the one that has always worked."""
    section("waking up on the saved map")
    ok, why = _restore([LANDED])
    check("a pose that is already there is the map loaded", ok, True)
    check("...and says so", why, "the map is loaded")

    ok, why = _restore([None, None, LANDED])
    check("a pose that arrives a scan later is still the map loaded", ok, True)


def test_a_cold_boot_is_not_a_map_that_could_not_be_read() -> None:
    """**The fault of 2026-09-05, and it cost a store its whole session.**

    On a cold boot the graph is eleven megabytes off cold cache, the lidar is
    still enumerating and the clock is not yet set, so nothing publishes a
    `map -> base_link` transform for tens of seconds. The wait was eight, after
    which the bridge reported a map it could not read, minted a new map
    identity, and the world state followed it onto what it believed was a new
    room -- hiding 256 things that were measured against the map on the screen
    and still perfectly good on it.

    The rover was standing in the restored map the whole time. Every failure in
    the log is stamped within a minute of boot with the clock unset; every
    restart of the stack on a machine already running restores cleanly and
    reports the rover to within three centimetres.
    """
    section("a cold boot is slow, not broken")
    silent_for = int(20.0 / 0.1)                # 20 s with no transform at all
    ok, why = _restore([None] * silent_for + [LANDED])
    check("a transform tree that takes twenty seconds still restores", ok, True)
    check("...and is not reported as a graph that could not be read",
          "could not" in why, False)

    # And the generosity has an end, or a mapper that never answers would hang
    # the boot rather than report anything.
    ok, why = _restore([None])
    check("a pose that never arrives at all is a failure", ok, False)
    check("...that says the position never arrived, rather than blaming the scan",
          "never" in why and "transform" in why, True)


def test_a_pose_that_lands_somewhere_else_is_still_refused() -> None:
    """The failure the wait exists to catch, which must survive the longer fuse.

    A mapper that read the graph and matched the scan in the wrong room puts the
    rover somewhere real and wrong. That is not a slow boot, and reporting it as
    one would restore a map the rover is not standing in -- which is worse than
    what this whole change is fixing.
    """
    section("a rover that lands in the wrong room")
    ok, why = _restore([(9.0, 9.0, 200.0)])
    check("a pose far from where the map says is refused", ok, False)
    check("...and says the rover was put somewhere else",
          "where the map says it was left" in why, True)
    check("...with how far off it was, because that is the evidence",
          "m and" in why, True)

    ok, _why = _restore([(2.4, 1.0, 30.0)])
    check("inside the mapper's own correlation window still counts as landed",
          ok, True)


def test_a_mapper_that_never_answers_is_reported_as_itself() -> None:
    """The deserialise call failing is a different sentence from the pose."""
    section("a mapper that does not answer")
    ok, why = _restore([LANDED], answers=False)
    check("a deserialise that times out is a failure", ok, False)
    check("...named as the load rather than as the pose",
          "loading the graph" in why, True)


TESTS = (
    test_a_restore_that_lands_is_a_restore,
    test_a_cold_boot_is_not_a_map_that_could_not_be_read,
    test_a_pose_that_lands_somewhere_else_is_still_refused,
    test_a_mapper_that_never_answers_is_reported_as_itself,
)
