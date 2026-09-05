#!/usr/bin/env python3
"""Keeping the map between sessions, and putting the rover back on it.

Two jobs that are really one. The pose graph is written to disk while the rover
drives and loaded again when the stack starts, so the map the rover wakes up with
is the map it was switched off with -- and because the one thing it cannot know
is whether somebody moved it while it was off, the same machinery answers "where
am I on this map, really?" on demand. Both go through `slam_toolbox`'s own
serialisation services, so nothing here has an opinion about the graph's format
and nothing here publishes a transform.

**`map -> odom` still has exactly one owner and it is still slam_toolbox.** That
rule is why the answer is applied the way it is: this works out where the rover
is, hands that pose to the mapper, and the mapper decides where the rover ends
up. What it does with it is `ProcessAgainstNodesNearBy`, which matches the next
scan against the graph near the pose it was given -- and which, in the version on
this rover, does *not* add that scan to the graph. So a refit cannot damage the
map. The worst a wrong one can do is move the rover, and pressing it again fixes
that.

A mixin on `NavBridge` like `NavMoves` and `NavExplore`, because it needs the
node's pose, its map and its service clients. The parts with no ROS in them are
next door and are what the selftest argues with: `mapstore.py` owns where the
files are and when they are written, and `refit.py` owns the search.

## The one timing trap, which is not obvious and is load-bearing

slam_toolbox will not fold a scan into its graph until the rover has driven
`minimum_travel_distance`, which is why `clear_map` leaves a parked rover in the
frame it just threw away until the wheels turn -- trail.py's docstring has that
story. A restore would have exactly the same problem, and a refit would be a
button that appears to do nothing until somebody drives.

It does not, and the reason is one line in slam_toolbox: deserialising sets
`first_measurement_`, and the next scan after that is processed whatever the
rover has or has not done. So the pose lands within a scan or two of the request,
parked or not. That is checked here rather than assumed -- `load_graph` waits
until the rover's own transform says it has arrived, and reports it if it never
does.
"""

import math
import threading
import time

from slam_toolbox.srv import DeserializePoseGraph, SerializePoseGraph

import frontier
import mapstore
import refit

#: How often the keeper wakes up. It does one thing per tick -- restore, then
#: settle, then save -- so this is also how long a boot takes to work through
#: those, and a second is far below the minute between saves.
TICK_S = 1.0

#: How long to wait for the mapper to answer a serialise or a deserialise. Both
#: hold the mapper's own mutex while they read or write the whole graph, so on a
#: large map they are seconds rather than milliseconds.
GRAPH_TIMEOUT_S = 30.0

#: How long to wait for a deserialised pose to actually reach the transform tree,
#: and how close counts as arrived. The mapper matches the next scan against the
#: graph near the pose it was handed, so what lands is within its own correlation
#: window of what was asked for -- a quarter of a metre by config/slam_toolbox.yaml
#: -- and never exactly it. Half a metre and twenty degrees is that window with
#: room to spare, and a pose that never gets that close is the mapper having
#: refused, which is worth saying out loud.
LANDED_S = 8.0
LANDED_M = 0.5
LANDED_DEG = 20.0

#: How long after a restore to keep waiting for the map and a scan before giving
#: up on fitting the rover to it. Thirty seconds covers a lidar that enumerates
#: late; past that, something is wrong that a fit cannot fix.
SETTLE_WAIT_S = 30.0


class NavMap:
    """The half of `NavBridge` that owns the map on disk."""

    # --- setting up -----------------------------------------------------------

    def map_startup(self):
        """Called once from the node's `__init__`, before anything is served."""
        self.saved = mapstore.SavedMap()
        # Re-entrant because `refit` holds it across a save and a load, and both
        # of those take it themselves -- they are also called on their own.
        self.map_lock = threading.RLock()
        #: None until the keeper has decided what map this is, which needs
        #: slam_toolbox to be answering. Everything downstream reads "no answer
        #: yet" from that None rather than being told a map that might change.
        self.map_id = None
        self.map_restored = False
        self.map_note = "the map keeper has not run yet"
        self.map_saved_at = None
        self.map_fit = None
        self._map_settle_from = None

        self.serialize_client = self.create_client(
            SerializePoseGraph, "/slam_toolbox/serialize_map",
            callback_group=self.group)
        self.deserialize_client = self.create_client(
            DeserializePoseGraph, "/slam_toolbox/deserialize_map",
            callback_group=self.group)

        # Its own thread rather than a node timer, and that is not a preference:
        # every call here waits on a service future, `wait` blocks the calling
        # thread on purpose (see nav_moves.py), and a timer that blocks is one of
        # the executor's three threads gone for the length of a graph write.
        self._map_stop = threading.Event()
        threading.Thread(target=self._map_loop, name="nav-map",
                         daemon=True).start()

    def map_status(self):
        """What the console and the daemon are told about the map on disk.

        `map_id` is the one anything holding coordinates cares about: it survives
        a restore of the same graph and changes when the map does, which is how
        the semantic world state knows whether its positions still mean anything.
        """
        return {
            "map_id": self.map_id,
            "map_kept": self.map_restored,
            "map_note": self.map_note,
            "map_saved_age_s": (None if self.map_saved_at is None
                                else round(time.time() - self.map_saved_at, 1)),
            "map_fit": self.map_fit,
        }

    def map_forgotten(self):
        """The pose graph has been thrown away, so the copy on disk goes too.

        Otherwise the next boot would load the map somebody had just deleted,
        which is the one outcome a person pressing "clear map" cannot have meant.
        The identity changes with it, because everything measured in the old
        frame -- the trail, every position in the world state -- is now measured
        against a map that no longer exists.
        """
        with self.map_lock:
            self.saved.forget()
            self.map_id = mapstore.new_id()
            self.map_restored = False
            self.map_fit = None
            self.map_saved_at = None
            self.map_note = ("the map was cleared, so the saved one went with it "
                             "and the rover is mapping the room again")

    # --- the keeper -----------------------------------------------------------

    def _map_loop(self):
        """One thing per tick: find the map, then settle on it, then keep it."""
        while not self._map_stop.wait(TICK_S):
            try:
                if self.map_id is None:
                    self.map_restore()
                    continue
                if self._map_settle_from is not None:
                    self.map_settle()
                    continue
                if self.saved.due(self.travelled_deg()):
                    self.save_graph()
            except Exception as error:              # never past here: it is a loop
                self.get_logger().warn("map keeper: %s: %s"
                                       % (type(error).__name__, error))

    def travelled_deg(self):
        """`(x_m, y_m, heading_deg)` in the *odom* frame, or None.

        What the wheels and the gyro have done since the ROS stack started, with
        no map correction on top -- which is what says whether the rover has
        actually moved. See `mapstore.due`, and `dead_reckoned` in nav_bridge.py
        for why the two frames answer different questions.
        """
        where = self.dead_reckoned()
        if where is None:
            return None
        return (where[0], where[1], math.degrees(where[2]))

    def pose_deg(self):
        """`(x_m, y_m, heading_deg)` in the map frame, or None.

        The node's `pose` in the units everything outside the transform tree
        uses. One conversion here rather than four at the callers, which is the
        same trade `refit.py` makes for the same reason.
        """
        where = self.pose()
        if where is None:
            return None
        return (where[0], where[1], math.degrees(where[2]))

    def map_restore(self):
        """Find out what map this is, and load it if there is one. Once, at start.

        Does nothing at all until slam_toolbox is answering, and that is
        deliberate rather than defensive: "there is no saved map, so this is a
        new one" is a statement about a running mapper, and made while the mapper
        is still starting it would hand the rest of the rover a map identity that
        the actual mapper then has nothing to do with.
        """
        if not self.deserialize_client.wait_for_service(timeout_sec=0.5):
            self.map_note = ("waiting for slam_toolbox before looking for a "
                             "saved map")
            return
        note = self.saved.held()
        if note is None:
            with self.map_lock:
                self.map_id = mapstore.new_id()
                self.map_restored = False
                self.map_note = ("no map was saved, so the rover is mapping the "
                                 "room from scratch")
            self.get_logger().info(self.map_note)
            return
        pose = self.saved.start_pose()
        ok, why = self.load_graph(pose, drop_trail=True)
        with self.map_lock:
            if not ok:
                self.map_id = mapstore.new_id()
                self.map_restored = False
                self.map_note = ("the saved map could not be loaded (%s), so the "
                                 "rover is mapping the room from scratch" % (why,))
            else:
                self.map_id = str(note.get("map_id"))
                self.map_restored = True
                self.map_saved_at = note.get("saved_at")
                self.map_note = ("the map from the last session is back, and the "
                                 "rover is where it was parked")
                self._map_settle_from = time.monotonic()
        self.get_logger().info(self.map_note)

    def map_settle(self):
        """Check the restored rover really is where the map was left, once.

        The saved pose is where the rover *was* when the graph was written, which
        is where it still is unless somebody moved it -- and somebody moving it is
        the whole reason this exists. So a restore is followed by one fit against
        the scan the lidar can actually see, applied if it is trustworthy and
        reported either way.

        Nothing here waits for a person, because the alternative is a rover that
        comes up believing a wall is a doorway and stays that way until somebody
        opens the console. The safety is in the fit rather than in the asking:
        `refit.py` will not move the rover further than its window, and refuses
        outright when the scan fits the map in more than one place.
        """
        waited = time.monotonic() - self._map_settle_from
        with self._lock:
            have_map = self.map_msg is not None
            have_scan = self.scan_msg is not None
        if not (have_map and have_scan):
            if waited < SETTLE_WAIT_S:
                return
            self._map_settle_from = None
            self.map_note += ("; it could not be checked against what the lidar "
                              "sees, because no %s arrived"
                              % ("map" if not have_map else "scan",))
            self.get_logger().warn(self.map_note)
            return
        self._map_settle_from = None
        answer = self.refit()
        self.map_note = "the map from the last session is back: " + str(
            answer.get("why") or "")
        self.get_logger().info(self.map_note)

    # --- the graph on disk ----------------------------------------------------

    def save_graph(self):
        """Write the pose graph and the note that says where the rover was.

        Returns `(ok, why)`. Written under a second name and renamed into place
        by `mapstore.commit`, so that a power cut during a write costs the last
        minute of mapping rather than the whole map.
        """
        with self.map_lock:
            pose = self.pose_deg()
            with self._lock:
                mapped = self.map_msg is not None
            if not mapped:
                # An empty graph is worse than no saved map, and not by a little.
                # A restore of one comes back saying the map was kept, so the
                # world state keeps coordinates it recorded in a frame that has
                # gone -- while the rover, having nothing to anchor on, quietly
                # starts a new map at wherever odometry happens to begin. Nothing
                # to map, nothing to save.
                return False, ("slam_toolbox has not published a map yet, so "
                               "there is no graph worth keeping")
            if pose is None:
                return False, ("there is no position, so there is nothing to "
                               "record as where the map was left")
            if not self.serialize_client.wait_for_service(timeout_sec=2.0):
                return False, "slam_toolbox is not answering"
            self.saved.make()
            request = SerializePoseGraph.Request()
            request.filename = self.saved.staging_stem
            future = self.serialize_client.call_async(request)
            if not self.wait(future, GRAPH_TIMEOUT_S):
                return False, ("slam_toolbox did not finish writing the graph in "
                               "%.0f seconds" % (GRAPH_TIMEOUT_S,))
            result = future.result()
            if result is None or result.result != result.RESULT_SUCCESS:
                return False, "slam_toolbox could not write the graph"
            try:
                note = self.saved.commit(self.map_id, pose,
                                         odom=self.travelled_deg())
            except OSError as error:
                return False, ("the graph was written but could not be put in "
                               "place: %s" % (error,))
            self.map_saved_at = note["saved_at"]
            return True, "the map is saved"

    def load_graph(self, pose, drop_trail=False):
        """Load the saved graph and put the rover on it near `pose`.

        Returns `(ok, why)`. `pose` is `(x_m, y_m, heading_deg)` in the map frame.

        **The wait at the end is the whole of the checking that can be done.**
        The service this rover has answers with nothing at all -- no result code,
        unlike its serialising twin -- so a file that could not be read and a
        mapper that would not anchor look exactly like a success from here. What
        distinguishes them is the transform: the rover's own pose moves to what
        was asked for within a scan or two, or it does not.

        `drop_trail` is for the restore at startup and not for a refit. At
        startup the track holds a point or two recorded at odometry's origin
        before the graph arrived, and a restore that jumps the rover across the
        house would draw a straight line from there to here -- which is the fault
        trail.py exists to describe. A refit moves the rover by less than its own
        window, which is a few steps of the track and not a lie about where it
        has been, so the track it has drawn all session is kept.
        """
        if pose is None:
            return False, "the saved map does not say where the rover was left"
        with self.map_lock:
            if not self.deserialize_client.wait_for_service(timeout_sec=2.0):
                return False, "slam_toolbox is not answering"
            if drop_trail:
                with self._lock:
                    self.trail.cleared(self.correction(), self.dead_reckoned())
            request = DeserializePoseGraph.Request()
            request.filename = self.saved.stem
            request.match_type = request.START_AT_GIVEN_POSE
            request.initial_pose.x = float(pose[0])
            request.initial_pose.y = float(pose[1])
            request.initial_pose.theta = math.radians(float(pose[2]))
            future = self.deserialize_client.call_async(request)
            if not self.wait(future, GRAPH_TIMEOUT_S):
                return False, ("slam_toolbox did not finish loading the graph in "
                               "%.0f seconds" % (GRAPH_TIMEOUT_S,))
            deadline = time.monotonic() + LANDED_S
            while time.monotonic() < deadline:
                where = self.pose_deg()
                if where is not None and _near(where, pose):
                    return True, "the map is loaded"
                time.sleep(0.1)
            return False, ("slam_toolbox did not put the rover where the map "
                           "says it was left, which is either a graph it could "
                           "not read or a scan it could not match")

    # --- fitting the rover to the map -----------------------------------------

    def refit(self, window_m=None, window_deg=None, min_score=None):
        """Find where the rover actually is on the map it has, and go there.

        Refused while a move is running, for `clear_map`'s reason: the route
        being followed is a list of places in coordinates this is about to move
        the rover within, and stopping is never refused.

        The order is deliberate and the save is not optional. Committing a fit
        means loading the graph again, and the graph on disk is up to a minute
        behind the one in memory -- so it is written first, and what comes back is
        what the rover had a moment ago rather than what it had a minute ago.
        """
        if not self.move_mutex.acquire(blocking=False):
            return {"fitted": False, "why":
                    "the rover is moving, and it has to be still to be measured "
                    "against the map -- stop it first"}
        try:
            with self.map_lock:
                answer = self.map_fit_now(window_m, window_deg, min_score)
                self.map_fit = answer
                return answer
        finally:
            self.move_mutex.release()

    def map_fit_now(self, window_m=None, window_deg=None, min_score=None):
        """The fit and its consequence, with the mutex already held."""
        with self._lock:
            grid_msg, scan = self.map_msg, self.scan_msg
        where = self.pose_deg()
        if grid_msg is None:
            return {"fitted": False,
                    "why": "there is no map yet, so there is nothing to fit to"}
        if scan is None:
            return {"fitted": False,
                    "why": "no scan has arrived, so there is nothing to fit"}
        if where is None:
            return {"fitted": False,
                    "why": "the rover has no position, so there is nowhere to "
                           "look for it -- this searches around where the rover "
                           "thinks it is rather than the whole house"}
        grid = frontier.Grid(
            grid_msg.info.width, grid_msg.info.height, grid_msg.info.resolution,
            grid_msg.info.origin.position.x, grid_msg.info.origin.position.y,
            grid_msg.data)
        points = refit.points_of(scan.ranges, scan.angle_min,
                                 scan.angle_increment, scan.range_min,
                                 scan.range_max)
        started = time.monotonic()
        fit = refit.fit(grid, points, where,
                        window_m=refit.WINDOW_M if window_m is None else window_m,
                        window_deg=(refit.WINDOW_DEG if window_deg is None
                                    else window_deg),
                        min_score=(refit.MIN_SCORE if min_score is None
                                   else min_score))
        answer = dict(fit.as_dict())
        answer["took_s"] = round(time.monotonic() - started, 2)
        answer["was"] = {"x_m": round(where[0], 3), "y_m": round(where[1], 3),
                         "heading_deg": round(where[2], 1)}
        if not fit.ok or fit.settled:
            answer["fitted"] = False
            return answer

        saved, why = self.save_graph()
        if not saved:
            answer["fitted"] = False
            answer["why"] = ("the rover is %.0f cm and %.1f degrees from where it "
                             "thinks it is, and it was left there: %s, and moving "
                             "it means loading the graph again"
                             % (100.0 * fit.moved_m, fit.turned_deg, why))
            return answer
        loaded, why = self.load_graph((fit.x_m, fit.y_m, fit.heading_deg))
        if not loaded:
            answer["fitted"] = False
            answer["why"] = "the fit was found but not applied: %s" % (why,)
            return answer
        # Where the rover actually ended up, which is the mapper's answer and not
        # this one. It matches the next scan against the graph near the pose it
        # was handed and keeps its own result, so what is reported is what
        # happened rather than what was asked for -- measured on the rover, a
        # 2.5-degree correction handed over came back as no move at all, because
        # the mapper matched the scan against the node it had just made from it.
        landed = self.pose_deg() or (fit.x_m, fit.y_m, fit.heading_deg)
        answer["pose"] = {"x_m": round(landed[0], 3), "y_m": round(landed[1], 3),
                          "heading_deg": round(landed[2], 1)}
        answer["moved_m"] = round(math.hypot(landed[0] - where[0],
                                             landed[1] - where[1]), 3)
        answer["turned_deg"] = round(
            (landed[2] - where[2] + 180.0) % 360.0 - 180.0, 1)
        if (answer["moved_m"] < refit.SETTLED_M
                and abs(answer["turned_deg"]) < refit.SETTLED_DEG):
            answer["fitted"] = False
            answer["why"] = ("the scan fits the map %.0f cm and %.1f degrees "
                             "from here, but the mapper matched it against its "
                             "own graph and kept the rover where it was"
                             % (100.0 * fit.moved_m, fit.turned_deg))
            return answer
        answer["fitted"] = True
        answer["why"] = ("the rover was %.0f cm and %.1f degrees from where it "
                         "thought it was, and has been moved onto the map -- "
                         "%.0f%% of the scan now lies on a wall against %.0f%% "
                         "before" % (100.0 * answer["moved_m"],
                                     answer["turned_deg"], 100.0 * fit.score,
                                     100.0 * fit.guess_score))
        return answer


def _near(where, pose):
    return (math.hypot(where[0] - pose[0], where[1] - pose[1]) <= LANDED_M
            and abs((where[2] - pose[2] + 180.0) % 360.0 - 180.0) <= LANDED_DEG)
