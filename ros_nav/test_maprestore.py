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

    def __init__(self, poses, tick=0.1, answers=True, odom=(0.0, 0.0, 0.0)):
        self.poses = list(poses)
        self.tick = tick
        self.answers = answers
        self.odom = odom
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

    def travelled_deg(self):
        """What the wheels have done, which is the baseline a restore takes."""
        return self.odom

    # --- and the rest, for `map_restore` ------------------------------------
    def get_logger(self):
        return types.SimpleNamespace(
            info=self.said.append, warn=self.said.append)

    def load_graph(self, *args, **fields):
        return nav_map.NavMap.load_graph(self, *args, **fields)

    def map_trustworthy(self):
        return nav_map.NavMap.map_trustworthy(self)


#: Where the map says the rover was parked, and where it actually lands -- a
#: little short of it, because the mapper matches the next scan within its own
#: correlation window and never returns exactly what it was handed.
PARKED = (2.0, 1.0, 30.0)
LANDED = (2.05, 0.94, 31.5)

#: Where the mapper anchored the graph on 2026-09-06 on a rover nobody had
#: touched: 41 cm and 81 degrees from the parked pose, because it matches the
#: first scan after a deserialise against the graph itself and loop closure had
#: snapped it round. The rover was standing on the map the whole time.
ANCHORED = (PARKED[0] + 0.29, PARKED[1] - 0.29, PARKED[2] + 81.0)

#: The note beside the graph on disk, with the identity that has to survive.
NOTE = {"map_id": "map-one", "saved_at": 1788600000.0,
        "pose": {"x_m": PARKED[0], "y_m": PARKED[1],
                 "heading_deg": PARKED[2]}}


class Saved(types.SimpleNamespace):
    """The note on disk, as much of it as a restore touches.

    `baselined` records what `map_restore` handed to `SavedMap.restored`, which
    is the call that stops the boot writing its anchor over the parked pose.
    """

    def restored(self, odom, now=None):
        self.baselined.append(odom)
        if odom is not None:
            self.posed_odom = tuple(odom)
        return odom is not None


def _restorer(poses, note=NOTE, parked=None, **fields):
    """A stand-in ready for `map_restore`, with a note already on disk."""
    node = Mapper(poses, **fields)
    node.said = []
    node.map_id = None
    node.map_restored = False
    node.map_settled = False
    node.map_note = ""
    node.map_saved_at = None
    node.map_parked_at = None
    node.map_anchored_at = None
    node.saved = Saved(
        stem="/tmp/current",
        held=lambda: note,
        start_pose=lambda: PARKED if parked is None else parked,
        posed_odom=None, baselined=[])
    return node


def _restore_map(node):
    """`map_restore` run against that stand-in, with time made to pass."""
    real_monotonic, real_sleep = nav_map.time.monotonic, nav_map.time.sleep

    def sleep(seconds):
        node.clock += seconds

    nav_map.time.monotonic = lambda: node.clock
    nav_map.time.sleep = sleep
    try:
        nav_map.NavMap.map_restore(node)
    finally:
        nav_map.time.monotonic, nav_map.time.sleep = real_monotonic, real_sleep
    return node


def _source():
    """`nav_map.py` itself, for the rules about what does *not* happen.

    A stand-in node can show what a call returns; it cannot show that no call
    was made from a loop the test never runs.
    """
    with open(nav_map.__file__, encoding="utf-8") as fh:
        return fh.read()


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


def test_a_restore_that_lands_is_a_restore() -> None:
    """The ordinary case, and the one that has always worked."""
    section("waking up on the saved map")
    ok, why, landed = _restore([LANDED])
    check("a pose that is already there is the map loaded", ok, True)
    check("...and says so", why, "the map is loaded")
    check("...and hands back where it landed", landed, LANDED)

    ok, why, _landed = _restore([None, None, LANDED])
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
    ok, why, _landed = _restore([None] * silent_for + [LANDED])
    check("a transform tree that takes twenty seconds still restores", ok, True)
    check("...and is not reported as a graph that could not be read",
          "could not" in why, False)

    # And the generosity has an end, or a mapper that never answers would hang
    # the boot rather than report anything.
    ok, why, landed = _restore([None])
    check("a pose that never arrives at all is a failure", ok, False)
    check("...that says the position never arrived, rather than blaming the scan",
          "never" in why and "transform" in why, True)
    check("...and lands nowhere, which is what says the graph was never read",
          landed, None)


def test_a_pose_that_lands_somewhere_else_is_still_refused() -> None:
    """The failure the wait exists to catch, which must survive the longer fuse.

    A mapper that read the graph and anchored it in the wrong room puts the rover
    somewhere real and wrong, and `load_graph` still says so -- but it now also
    hands back *where*, because that is what tells `map_restore` the graph was
    read at all. See the docstring there: an anchor is not a map that could not be
    loaded, and treating it as one is what threw the map away three times.
    """
    section("a rover that lands in the wrong room")
    ok, why, landed = _restore([(9.0, 9.0, 200.0)])
    check("a pose far from where the map says is refused", ok, False)
    check("...and says the mapper anchored it somewhere else",
          "anchored it" in why, True)
    check("...with how far off it was, because that is the evidence",
          "m and" in why, True)
    check("...and hands back the anchor, which says the graph was read",
          landed, (9.0, 9.0, 200.0))

    ok, _why, _landed = _restore([(2.4, 1.0, 30.0)])
    check("inside the mapper's own correlation window still counts as landed",
          ok, True)


def test_a_badly_anchored_graph_is_still_the_map_the_rover_is_standing_on() -> None:
    """**The fault of 2026-09-06, and it recurred until the map compounded away.**

    slam_toolbox anchors a deserialised graph with its own scan matcher, so where
    the rover lands is its answer and not the rover's belief about anything.
    Measured on the rover, standing still and untouched: 41 cm and 81 degrees from
    where the map said it was parked, with loop closure able to make that 180.

    The rover was on the old map the whole time -- real coordinates, real walls,
    a 15-metre house grid on screen -- and the bridge called it a map that could
    not be loaded, minted a new identity, and told the world state to start a new
    session. What must happen instead is the plain truth: a pose arrived, so the
    graph was read, so this is the old map. The rover is then taken to be where
    it was parked, because nobody drove it while it was switched off -- the boot
    says so and waits, rather than searching for it unasked.
    """
    section("a graph the mapper anchored somewhere else")
    node = _restore_map(_restorer([ANCHORED]))
    check("the map is kept rather than thrown away", node.map_restored, True)
    check("...with the identity the world state is holding coordinates under",
          node.map_id, "map-one")
    check("...and the age of the graph it came from", node.map_saved_at,
          NOTE["saved_at"])
    check("...and it says the mapper anchored it, not that the map was lost",
          "anchored it" in node.map_note and "from scratch" not in node.map_note,
          True)

    check("...and nothing goes looking for the rover, which nobody asked for",
          "refit" in node.map_note, True)
    check("...saying instead that the rover is taken to be where it was parked",
          "parked where the map left it" in node.map_note, True)

    check("the parked pose is kept, as what a fit would search around",
          node.map_parked_at, PARKED)
    check("...and so is the anchor, which is the error rather than a belief",
          node.map_anchored_at, ANCHORED)
    check("and until somebody asks for a fit, nothing is written back to disk",
          node.map_trustworthy(), False)


def test_a_boot_does_not_write_its_anchor_over_the_parked_pose() -> None:
    """**The pose on disk is the better number, and the boot leaves it alone.**

    A restore used to be followed within a second by the keeper writing down
    where the rover was -- which on a fresh boot is the mapper's anchor for a
    graph it read a moment ago, allowed to be half a metre and twenty degrees
    out. The pose it replaced was measured by a session that had driven there.
    A rover nobody has moved is still at the older one, so the newer one is not
    news; written boot after boot it walks the parking spot across the room.

    So the restore hands the wheels' reading to `SavedMap.restored`, and the
    gates behave as though this session had already written: the first thing it
    writes is written because the rover drove.
    """
    section("what a boot writes down")
    node = _restore_map(_restorer([LANDED]))
    check("the wheels' reading becomes the baseline for driving",
          node.saved.baselined, [node.odom])
    check("...so nothing is written until the rover has moved from it",
          node.saved.posed_odom, node.odom)

    node = _restore_map(_restorer([LANDED], odom=None))
    check("wheels that are not answering yet leave no baseline",
          node.saved.posed_odom, None)
    check("...which the keeper's loop is what asks again for",
          "self.saved.restored(odom)" in _source(), True)

    node = _restore_map(_restorer([None]))
    check("and a map this session is drawing from scratch takes none",
          node.saved.baselined, [])


def test_only_a_graph_that_was_never_read_starts_a_new_map() -> None:
    """The other half: a real failure still has to mint a new identity.

    Keeping the map on a mis-anchored restore is only safe because a restore that
    genuinely produced nothing is still told apart from it. No pose ever reaching
    the transform tree means nothing here can say the graph was read, and
    claiming the old map's identity then would hand the world state coordinates in
    a frame that may not exist.
    """
    section("a graph that really was not read")
    node = _restore_map(_restorer([None]))
    check("no pose at all starts a new map", node.map_restored, False)
    check("...under a new identity", node.map_id not in (None, "map-one"), True)
    check("...and says the rover is mapping from scratch",
          "from scratch" in node.map_note, True)
    check("...with no parked pose kept, because there is nothing to fit to",
          node.map_parked_at, None)
    check("...and a map this session drew needs no permission to be saved",
          node.map_trustworthy(), True)

    node = _restore_map(_restorer([LANDED], answers=False))
    check("a deserialise that never answers starts a new map too",
          node.map_restored, False)

    node = _restore_map(_restorer([LANDED], note=None))
    check("no saved note at all starts a new map", node.map_restored, False)
    check("...and does not go looking for a pose to fit",
          node.map_parked_at, None)


def test_a_restore_that_landed_cleanly_needs_nothing_further() -> None:
    """The ordinary restore: the mapper agrees, so the rover is where it was.

    The rover was parked here and cannot have driven while it was off, so an
    anchor that lands on the saved pose is that belief confirmed by the only
    other thing with an opinion. Nothing is searched for, and the keeper is free
    to write over the saved pose again -- which is the whole of what a boot has
    to do on the day nothing went wrong.
    """
    section("the restore that always worked")
    node = _restore_map(_restorer([LANDED]))
    check("the map is kept", node.map_restored, True)
    check("...and says the rover is where it was parked",
          "where it was parked" in node.map_note, True)
    check("...with the parked pose kept, should a fit be asked for later",
          node.map_parked_at, PARKED)
    check("...and is trusted for writing straight away, with nothing to await",
          node.map_trustworthy(), True)
    check("...though the note on disk already describes a rover nobody moved",
          node.saved.baselined, [node.odom])


def test_a_boot_never_fits_the_rover_by_itself() -> None:
    """**No boot runs a search nobody asked for, and that is the rule here.**

    A fit moves the rover on one scan matched against a stored graph, and the
    case it exists for -- somebody carried the rover while it was off -- is the
    case where that scan agrees with the map least. So it belongs to the console
    button and the daemon call that a person presses, and the keeper's job on a
    restore ends at saying what it found.

    Checked against the source because it is a rule about what does *not*
    happen, and nothing a stand-in node returns can demonstrate that.
    """
    section("what a boot does on its own")
    source = _source()
    keeper = source[source.index("def _map_loop"):source.index("def map_trustworthy")]
    check("the keeper's loop never calls the fit", "self.refit(" in keeper, False)
    restore = source[source.index("def map_restore"):source.index("def load_graph")]
    check("...and neither does the restore", "self.refit(" in restore, False)
    check("...nor is there anything left to schedule one",
          "def map_settle(" in source, False)


class Prompted:
    """A node with just enough on it for `refit` to choose a search centre.

    The fit itself is a recorder: what is under test is which pose the search is
    centred on, not what the search then finds, which is `test_refit.py`'s.
    """

    def __init__(self, pose, settled=False, parked=PARKED, anchored=ANCHORED):
        self._pose = pose
        self.map_lock = threading.RLock()
        self.move_mutex = threading.Lock()
        self.map_restored = True
        self.map_settled = settled
        self.map_fit = None
        self.map_parked_at = parked
        self.map_anchored_at = anchored
        self.asked = []

    def pose_deg(self):
        return self._pose

    def map_fit_now(self, _window_m, _window_deg, _min_score, around):
        self.asked.append(around)
        return {"fitted": False, "why": "measured, not moved"}

    def map_trustworthy(self):
        return nav_map.NavMap.map_trustworthy(self)

    def still_parked(self):
        return nav_map.NavMap.still_parked(self)


def _asked_around(pose, around=None, **fields):
    node = Prompted(pose, **fields)
    nav_map.NavMap.refit(node, around=around)
    return node.asked[0]


def test_a_fit_asked_for_looks_where_the_map_was_left() -> None:
    """**Where the search is centred, which is what made every refit refuse.**

    On 2026-09-06 the rover woke anchored 81 degrees from where it was parked,
    and each refit searched a window around that anchor -- with the truth
    outside it, since the anchor was the error. The rover's own pose is not
    evidence about the rover until something has confirmed it; the pose the map
    was left at is, because nobody drove the rover while it was switched off.

    That holds only while it is still standing there. Once it has driven, the
    parked pose is history and the search goes back to where the rover is.
    """
    section("where a prompted fit looks")
    check("a rover woken badly anchored is searched for where it was parked",
          _asked_around(ANCHORED), PARKED)
    check("...and not around the anchor, which is the error being corrected",
          _asked_around(ANCHORED) == ANCHORED, False)

    check("a rover whose anchor was confirmed is searched for where it is",
          _asked_around(LANDED, settled=True), None)

    driven = (ANCHORED[0] + 1.5, ANCHORED[1], ANCHORED[2])
    check("...and so is one that has driven since it woke",
          _asked_around(driven), None)
    nudged = (ANCHORED[0] + 0.02, ANCHORED[1], ANCHORED[2] + 1.0)
    check("...where drifting on the spot is not driving",
          _asked_around(nudged), PARKED)

    check("a caller that names a centre is still obeyed",
          _asked_around(ANCHORED, around=LANDED), LANDED)
    check("...and a restore that kept no parked pose falls back to the rover",
          _asked_around(ANCHORED, parked=None), None)


def test_an_unsettled_map_is_never_written_over_the_saved_one() -> None:
    """**The rule that stops one bad session poisoning the next, and it is new.**

    The keeper writes where the rover is far more often than it writes the graph,
    and the first of those writes lands seconds after a restore. So a restore
    anchored 81 degrees out used to overwrite the good saved pose with the bad
    one straight away -- and the next boot restored *at* it, anchored worse, and
    wrote that down in turn. The rover's log has three consecutive restores at
    20, 180 and 29 degrees doing exactly that, each starting a new world-state
    session as it went.
    """
    section("what may be written back to disk")
    node = _restorer([ANCHORED])
    node.map_restored, node.map_settled = True, False
    check("a restored map whose pose no scan has confirmed is not written",
          node.map_trustworthy(), False)
    node.map_settled = True
    check("...and is written once the anchor or a fit has confirmed it",
          node.map_trustworthy(), True)

    node = _restorer([ANCHORED])
    node.map_restored, node.map_settled = False, False
    check("a map this session drew is always written, having nothing to confirm",
          node.map_trustworthy(), True)

    # And the keeper has to actually ask, or the rule is decoration.
    source = _source()
    loop = source[source.index("def _map_loop"):source.index("def map_trustworthy")]
    check("the keeper asks before it saves anything",
          "self.map_trustworthy()" in loop, True)
    check("...before either the graph or the pose",
          loop.index("map_trustworthy") < loop.index("self.saved.due"), True)


def test_a_mapper_that_never_answers_is_reported_as_itself() -> None:
    """The deserialise call failing is a different sentence from the pose."""
    section("a mapper that does not answer")
    ok, why, landed = _restore([LANDED], answers=False)
    check("a deserialise that times out is a failure", ok, False)
    check("...named as the load rather than as the pose",
          "loading the graph" in why, True)
    check("...and lands nowhere, so no map is claimed from it", landed, None)


TESTS = (
    test_a_restore_that_lands_is_a_restore,
    test_a_cold_boot_is_not_a_map_that_could_not_be_read,
    test_a_pose_that_lands_somewhere_else_is_still_refused,
    test_a_badly_anchored_graph_is_still_the_map_the_rover_is_standing_on,
    test_only_a_graph_that_was_never_read_starts_a_new_map,
    test_a_restore_that_landed_cleanly_needs_nothing_further,
    test_a_boot_never_fits_the_rover_by_itself,
    test_a_boot_does_not_write_its_anchor_over_the_parked_pose,
    test_a_fit_asked_for_looks_where_the_map_was_left,
    test_an_unsettled_map_is_never_written_over_the_saved_one,
    test_a_mapper_that_never_answers_is_reported_as_itself,
)
