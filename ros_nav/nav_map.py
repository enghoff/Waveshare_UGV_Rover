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

## The other trap, which was live for weeks and cost the map three times over

**Where the mapper anchors the graph it has just read is not where it was asked
to, and it is not a mistake either.** Deserialising hands slam_toolbox a pose and
slam_toolbox then matches the next scan against the graph near it, so the anchor
that results is its own matcher's answer -- ordinarily within a quarter of a
metre and twenty degrees, and much further when loop closure fires in a room
whose two ends look alike. Measured on the rover on 2026-09-06, standing still
and untouched: the graph came back anchored 41 cm and 81 degrees from where the
map said the rover was parked, and the scan fitted the map at 97% at the parked
pose against 30% at the anchored one.

Three things then went wrong in a row, and together they made that permanent:

1. **The anchor was read as a map that could not be loaded.** The graph was in
   fact loaded -- the rover was standing on it, in its coordinates, driving on
   its walls -- but a new map identity was minted, which moved the semantic world
   state onto a new session and took everything measured in the old one off the
   console. The map on screen was the right map; only the rover's place in it and
   the label on it were wrong.
2. **The fit that exists to correct exactly this never ran**, because it was
   conditional on the restore having been called a success.
3. **The bad pose was written to disk within seconds**, over the good one, so the
   next boot restored *at* it, anchored somewhere worse again, and wrote that
   down in turn. The log has three consecutive restores at 20, 180 and 29 degrees
   doing this.

So a pose arriving at all now means the map was read and is kept. What happens
after that is a decision about what to believe rather than a search: **a rover
switched off parked is still parked, so the pose the map was left at is the
truth, and an anchor that disagrees with it is the mapper being wrong rather
than news about the rover.** An anchor that agrees is the rover's place on the
map confirmed and the keeper may write over the saved pose; an anchor that does
not leaves the map kept, the disagreement in the note, and the saved pose
exactly where it was. `map_trustworthy` is that last rule.

## A saved map is only ever replaced by itself, or by somebody clearing it

The trap above cost the map three times and was survivable each time, because
the graph was still on disk. The same evening it was lost outright, and this is
the rule that stops that. A restore that produced no pose at all -- as opposed to
one anchored badly -- used to mint a new identity and let the rover's own scratch
graph be the map. The world state adopted that identity within seconds, so 489
placed things stopped being anywhere; and because a map the session drew itself
needs no permission to be written down, the keeper wrote the scratch graph over
the saved one as soon as the wheels turned, so the frame those 489 positions were
measured in ceased to exist and nothing could carry them across.

Every way of failing to read a saved graph is the mapper being slow or busy: a
deserialise still holding its own mutex after thirty seconds, no transform after
sixty, a node that has not finished coming up. None of them is news about the
room. So `map_restore` claims nothing, writes nothing and asks again on a later
tick, for as long as that takes -- `map_id` stays None, which everything
downstream already reads as "no answer yet", and which the keeper reads as a tick
to spend on the restore rather than on a write. `mapstore.commit` then refuses
outright to put a graph over a saved map recorded under another identity, so the
rule holds at the point the bytes move even if some future caller forgets it.

The cost is a rover that drives on an unnamed scratch map until its own comes
back, and `rover_world._world_pose` is where that is paid: no map identity means
no pose, so looks are still recorded and still kept, and none of them gets a
bearing it would have to measure in a frame that is about to be thrown away.

## Nothing goes looking for the rover unless somebody asks

**The fit is prompted and never automatic**, which is the console's "refit to
map" button and the daemon's `refit_pose`. It moves the rover on the strength of
one scan matched against a stored graph, and the single case it exists for --
somebody carried the rover while it was off -- is also the case where that scan
has least to do with the map. Run at every boot it is a search nobody asked for,
argued against a prior that is right almost every time; asked for, it is a
person saying "I moved it", which is the one piece of information none of this
can get for itself.

What the boot owes that person instead is an honest sentence and a saved map
left intact, so that pressing the button is still an option an hour later.
`refit.fit`'s `was` argument is what keeps the call honest whenever it comes: it
separates where to look from what the correction is measured against, without
which an 81 degree error reads as "nothing was moved". And while the rover has
not moved since it woke, a fit looks around the pose the map was left at rather
than around the anchor -- centring on the anchor is what made every refit on
2026-09-06 search a window with the truth outside it.
"""

import math
import threading
import time

from slam_toolbox.srv import DeserializePoseGraph, SerializePoseGraph

import frontier
import mapstore
import refit

#: How often the keeper wakes up. It does one thing per tick -- restore, then
#: save -- so this is also how long a boot takes to work through those, and a
#: second is far below the minute between saves.
TICK_S = 1.0

#: How long to wait for the mapper to answer a serialise or a deserialise. Both
#: hold the mapper's own mutex while they read or write the whole graph, so on a
#: large map they are seconds rather than milliseconds.
GRAPH_TIMEOUT_S = 30.0

#: How long to wait for a deserialised pose to actually reach the transform tree,
#: and how close counts as arrived cleanly.
#:
#: **These no longer decide whether the map is kept, and that is the fix of
#: 2026-09-06.** A pose arriving at all means the graph was read, so `map_restore`
#: keeps the map either way. What they decide is whether the mapper's anchor
#: agrees with the pose the map was left at -- which, on a rover nobody moved, is
#: the same question as whether the anchor is right, and so whether the keeper
#: may write over the saved pose. Disagreement now costs a sentence and a wait
#: for somebody to press refit. What it used to cost was the map, and the
#: numbers made that easy -- `correlation_search_space_dimension`
#: is 0.5 in config/slam_toolbox.yaml, so the mapper's own matcher can move the
#: anchor a quarter of a metre against the half here, but
#: `coarse_search_angle_offset` is 0.349 rad, which is *exactly* the twenty
#: degrees here with no room at all. So an anchor at the edge of the mapper's
#: ordinary search window read as a refusal, and the log has one at 20 degrees
#: doing precisely that. Loop closure can then move it much further again -- 180
#: degrees in a room whose two ends match, which the log also has.
#:
#: **A minute, and it used to be eight seconds.** Eight is right for the wait it
#: was written for -- a mapper that is already publishing takes a scan or two to
#: anchor -- and it is nowhere near enough for the wait that actually happens. On
#: a cold boot nothing publishes `map -> base_link` for tens of seconds: an
#: eleven-megabyte graph is being read off cold cache, the lidar is still
#: enumerating, and the clock is not yet set. On 2026-09-05 that reported a map
#: the rover had in fact restored and was standing in as a map it could not read,
#: which minted a new map identity, which moved the world state onto a new map
#: session, which took 256 placed things off the console's map. Every failure in
#: the log is within a minute of a boot; every restore after a restart of the
#: stack on a running machine succeeds.
#:
#: What the length costs is only how long a genuine failure takes to be reported,
#: once, at startup -- and against that, a restore wrongly abandoned throws away
#: the map and everything measured in it.
LANDED_S = 60.0
LANDED_M = 0.5
LANDED_DEG = 20.0

#: How long to leave between attempts at a saved map that has not loaded yet.
#:
#: **A restore that produced nothing is asked again rather than replaced**, and
#: the spacing is the whole of what makes that affordable. Every attempt asks
#: slam_toolbox to deserialise the entire graph under its own mutex, so a keeper
#: retrying every tick would be competing for the mapper with the very work it is
#: waiting on. Thirty seconds is far below how long anybody minds waiting at a
#: boot and far above the cost of one attempt.
RESTORE_RETRY_S = 30.0

#: How far the rover may have moved on the map since it woke and still have the
#: pose the map was left at used as the centre of a fit.
#:
#: That pose is a prior about a rover standing where somebody left it, and it
#: stops being one the moment the rover drives: a fit asked for after a lap of
#: the house has to look around where the rover is now, wrong as that may be. A
#: quarter of a metre and ten degrees is well above what a parked rover's
#: believed pose drifts with the gyro over the minutes it takes somebody to open
#: the console, and well below a deliberate move. A rover that was driven while
#: badly anchored is past what any window can find, and the wide-window call in
#: the README is what it needs.
STILL_M = 0.25
STILL_DEG = 10.0


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
        #: Whether the rover's place on a restored map is believed. True when the
        #: mapper anchored the graph where the map says the rover was parked, and
        #: when a fit somebody asked for has since put it right. False on a map
        #: this session drew itself, where there is nothing to believe or doubt:
        #: the rover built those coordinates as it went. It gates writing over
        #: the saved map -- see `map_trustworthy` and `save_graph`.
        self.map_settled = False
        self.map_note = "the map keeper has not run yet"
        self.map_saved_at = None
        self.map_fit = None
        #: Where the note said the rover was parked, and where the mapper's own
        #: matcher actually anchored the graph, both kept from the restore. The
        #: first is the better prior for a fit while the rover has not moved; the
        #: second is how `still_parked` knows whether it has. Both None until
        #: there is a restored map to have them.
        self.map_parked_at = None
        self.map_anchored_at = None
        #: When the last attempt at a saved map began, and how many there have
        #: been. A saved map that will not load is asked for again rather than
        #: written over, so this is what spaces those attempts out and what lets
        #: the note say how long it has been trying.
        self.map_restore_at = None
        self.map_restore_tries = 0

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
            # Whether the rover's place on that map is believed. A kept map with
            # this false is a rover standing on real coordinates it cannot vouch
            # for its position in and is waiting to be refitted, which is worth
            # telling apart from both a fresh map and a settled one -- and it is
            # also the state in which nothing is written back to disk. None
            # before the keeper has decided what map this is, for `map_id`'s
            # reason: "nothing is confirmed yet" and "nothing needs confirming"
            # are both false-ish and only one of them is a rover to worry about.
            "map_settled": (None if self.map_id is None
                            else self.map_trustworthy()),
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
            self.map_settled = False
            self.map_fit = None
            self.map_saved_at = None
            self.map_parked_at = None
            self.map_anchored_at = None
            self.map_note = ("the map was cleared, so the saved one went with it "
                             "and the rover is mapping the room again")

    # --- the keeper -----------------------------------------------------------

    def _map_loop(self):
        """One thing per tick: find the map, then keep it.

        There is deliberately no third job here. A restore used to be followed by
        a fit this loop ran on its own; the rover is now taken to be where it was
        parked, and looking for it anywhere else is something a person asks for.
        """
        while not self._map_stop.wait(TICK_S):
            try:
                if self.map_id is None:
                    self.map_restore()
                    continue
                odom = self.travelled_deg()
                if self.map_restored and self.saved.posed_odom is None:
                    # The restore has not been given a baseline to measure
                    # driving from yet, because the wheels were not answering
                    # when it ran. Until they do, the note on disk stands as it
                    # is -- see `SavedMap.restored`, which is the rule that keeps
                    # the parked pose from being overwritten by the anchor.
                    self.saved.restored(odom)
                    continue
                if not self.map_trustworthy():
                    continue
                if self.saved.due(odom):
                    self.save_graph()
                elif self.saved.pose_due(odom):
                    self.keep_pose(odom)
            except Exception as error:              # never past here: it is a loop
                self.get_logger().warn("map keeper: %s: %s"
                                       % (type(error).__name__, error))

    def map_trustworthy(self):
        """Whether the rover's own pose is fit to be written down as where it is.

        **The one guard that stops a bad session poisoning the next one, and it
        was missing.** The keeper writes where the rover is far more often than it
        writes the graph, and the first of those writes happens seconds after a
        restore. So a restore that anchored the graph 81 degrees out used to
        overwrite the good saved pose with the bad one within seconds -- and then
        the next boot restored *at* the bad pose, anchored somewhere worse again,
        and wrote that down in turn. Measured in the log on 2026-09-06: three
        restores in a row, 20 degrees out, then 180, then 29, each inheriting the
        last one's error and each starting a new map session in the world state.

        A map this session drew needs no permission: its coordinates are the
        rover's own and there is nothing to disagree with. A map that came off
        disk does, because the rover's place in it is the mapper's anchor until
        something confirms it -- either the anchor landing where the map says the
        rover was parked, which on a rover nobody moved is confirmation, or a fit
        somebody asked for putting it right. An unconfirmed anchor must not be
        the thing the next boot trusts.
        """
        return self.map_settled or not self.map_restored

    def still_parked(self):
        """Whether the rover is still standing where the restore left it.

        The pose the map was left at is only a prior about a rover nobody has
        driven, so this is what says whether `refit` may still use it. Asked of
        the map frame rather than the wheels because at the moment it matters --
        the seconds after a boot -- the map pose is the one thing known to exist:
        a pose reaching the transform tree is how the restore decided the graph
        had been read at all.
        """
        if self.map_anchored_at is None:
            return False
        where = self.pose_deg()
        if where is None:
            return False
        return (math.hypot(where[0] - self.map_anchored_at[0],
                           where[1] - self.map_anchored_at[1]) <= STILL_M
                and abs((where[2] - self.map_anchored_at[2] + 180.0) % 360.0
                        - 180.0) <= STILL_DEG)

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
        """Find out what map this is, and load it if there is one.

        Does nothing at all until slam_toolbox is answering, and that is
        deliberate rather than defensive: "there is no saved map, so this is a
        new one" is a statement about a running mapper, and made while the mapper
        is still starting it would hand the rest of the rover a map identity that
        the actual mapper then has nothing to do with.

        **Asked again for as long as it takes, rather than once at start.** A
        saved graph that has not loaded has not gone anywhere: every way of
        failing to read one is the mapper being slow or busy -- a deserialise
        still holding its mutex after thirty seconds, no transform after sixty, a
        node still coming up. Minting a new identity there was how this rover
        lost its map on 2026-09-06, twice over: the world state adopted the new
        identity within seconds and 489 placed things stopped being anywhere, and
        the keeper then wrote the scratch graph over the saved one as soon as the
        wheels turned, so the map those positions were measured in was gone from
        the disk and no re-anchoring could ask it anything. The saved map is now
        only ever forgotten by somebody clearing it.

        So a failure claims nothing, writes nothing, and says so. `map_id` stays
        None, which everything downstream already reads as "no answer yet" -- and
        which the keeper reads as a tick to spend here instead of on a write.
        """
        if not self.deserialize_client.wait_for_service(timeout_sec=0.5):
            self.map_note = ("waiting for slam_toolbox before looking for a "
                             "saved map")
            return
        note = self.saved.held()
        if note is None:
            # Nothing on disk is not a restore that failed. There is no map to
            # abandon and nothing anywhere holds coordinates measured against
            # one, so the rover names the graph it is about to draw and gets on
            # with it. This is the only new identity nobody asks for.
            with self.map_lock:
                self.map_id = mapstore.new_id()
                self.map_restored = False
                self.map_note = ("no map was saved, so the rover is mapping the "
                                 "room from scratch")
            self.get_logger().info(self.map_note)
            return
        now = time.monotonic()
        if (self.map_restore_at is not None
                and now - self.map_restore_at < RESTORE_RETRY_S):
            return
        self.map_restore_at = now
        self.map_restore_tries += 1
        pose = self.saved.start_pose()
        ok, why, landed = self.load_graph(pose, drop_trail=True)
        with self.map_lock:
            if landed is None:
                self.map_note = ("the saved map has not loaded yet (%s); it is "
                                 "untouched on disk and will be asked for "
                                 "again, attempt %d"
                                 % (why, self.map_restore_tries))
                self.get_logger().warn(self.map_note)
                return
            # A pose arrived, so the graph was read and the rover is standing
            # on the old map whether or not the mapper anchored it where it
            # was asked. Keeping the identity is the whole point: it is what
            # tells the semantic world state that its coordinates still mean
            # something, and minting a new one here is what used to throw a
            # good map and everything measured in it away.
            self.map_id = str(note.get("map_id"))
            self.map_restored = True
            self.map_saved_at = note.get("saved_at")
            # Where the map says the rover physically is, and where the
            # mapper actually put it. The first is what the rover is taken to
            # believe about itself, because it was parked there and nobody
            # can have driven it while it was off; the second is only the
            # mapper's own matcher answering, and is kept so that a fit
            # asked for later can tell a rover that has stayed put from one
            # somebody has since driven. See `refit.fit`'s `was`.
            self.map_parked_at = pose
            self.map_anchored_at = landed
            # An anchor that agrees with the parked pose is that belief
            # confirmed, and the keeper may write over the saved pose. One
            # that disagrees is the mapper wrong about a rover that has not
            # moved, so the saved pose stays exactly as it is until a fit
            # somebody asked for settles the argument. Nothing goes looking
            # on its own -- that search is what used to run here.
            self.map_settled = bool(ok)
            self.map_note = (
                "the map from the last session is back, and the rover is "
                "where it was parked" if ok else
                "the map from the last session is back, but %s -- the rover "
                "is taken to be parked where the map left it, and a refit "
                "is what moves it if it is not" % (why,))
            # And the pose in the note is where it is taken to be, so this
            # session has nothing to add until the wheels turn. Without
            # this the keeper's first tick writes the anchor over it.
            self.saved.restored(self.travelled_deg())
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
            if note is None:
                # The map on disk belongs to another session's room, so this one
                # is not allowed to land on top of it. Nothing above should ever
                # reach here -- a restore that failed claims no identity to save
                # under -- and it is refused rather than trusted to stay true.
                return False, ("the saved map was recorded under a different "
                               "identity, so this graph was not written over it")
            self.map_saved_at = note["saved_at"]
            return True, "the map is saved"

    def keep_pose(self, odom=None):
        """Write down where the rover is, without writing the graph again.

        The cheap half of saving, and the half that a boot after a power cut
        actually needs. Writing the graph means asking slam_toolbox to serialise
        thirteen megabytes under its own mutex, which is why it happens about once
        a minute; where the rover is inside that graph changes every time a wheel
        turns, costs a rename to record, and is the one thing the next boot cannot
        work out for itself.

        Measured on this rover on 2026-09-05: the stack went down mid-drive four
        seconds into a graph write, so the note that survived was the one from the
        save before it. The boot put the rover back where that note said, the scan
        fitted the map there at 56% against the 90% a fit needs, and the rover
        came up outside the window `refit.py` can search -- unable to find its way
        back onto a map it had kept perfectly well.

        Under the map lock, so that a pose never lands in the note while `refit`
        is between writing the graph and loading it back.
        """
        with self.map_lock:
            return self.saved.note_pose(self.map_id, self.pose_deg(), odom)

    def load_graph(self, pose, drop_trail=False):
        """Load the saved graph and put the rover on it near `pose`.

        Returns `(ok, why, landed)`. `pose` is `(x_m, y_m, heading_deg)` in the
        map frame, and `landed` is where the rover's own transform actually
        arrived -- None if it never arrived at all.

        **`landed` is the difference between a map that could not be read and a
        map that was read and anchored badly, and conflating those two cost the
        rover its map repeatedly.** A pose arriving at all means slam_toolbox
        read the graph: it is publishing `map -> base_link` in that graph's own
        coordinates, and the rover is on that map whatever this function returns.
        Where it landed is the mapper's own scan matcher's answer and can be a
        long way from what was asked for -- so "not where I asked" is a rover
        that needs fitting to the map it has, and only "nothing ever arrived" is
        a rover with no map. See `map_restore`, which is where that mattered.

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
            return False, "the saved map does not say where the rover was left", None
        with self.map_lock:
            if not self.deserialize_client.wait_for_service(timeout_sec=2.0):
                return False, "slam_toolbox is not answering", None
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
                               "%.0f seconds" % (GRAPH_TIMEOUT_S,)), None
            # **Two failures, and they were one sentence.** A transform tree that
            # never says anything and one that puts the rover in the wrong room
            # are different faults with different cures, and the message that
            # covered both -- "either a graph it could not read or a scan it
            # could not match" -- named neither. The first is what a cold boot
            # looks like and is usually not a fault at all; the second is a real
            # refusal and the reason this check exists. So the last pose seen is
            # kept, and it decides which of the two is reported.
            deadline = time.monotonic() + LANDED_S
            last = None
            while time.monotonic() < deadline:
                where = self.pose_deg()
                if where is not None:
                    if _near(where, pose):
                        return True, "the map is loaded", where
                    last = where
                time.sleep(0.1)
            if last is None:
                return False, ("the rover's own position never reached the "
                               "transform tree in %.0f seconds, so nothing here "
                               "can say whether the graph was read"
                               % (LANDED_S,)), None
            # Phrased as a clause, because `map_restore` reads it out after "the
            # map from the last session is back, but ..." and the first draft of
            # this said "but slam_toolbox read the graph but anchored it".
            return False, ("the mapper anchored it %.2f m and %.0f degrees from "
                           "where the map says the rover was left, which is its "
                           "own scan matcher's answer rather than anything the "
                           "rover believes"
                           % (math.hypot(last[0] - pose[0], last[1] - pose[1]),
                              abs((last[2] - pose[2] + 180.0) % 360.0 - 180.0))), last

    # --- fitting the rover to the map -----------------------------------------

    def refit(self, window_m=None, window_deg=None, min_score=None, around=None):
        """Find where the rover actually is on the map it has, and go there.

        **Only ever because somebody asked.** Nothing in this file calls it: it is
        the console's "refit to map" button and the daemon's `refit_pose`, and
        pressing one of those is a person saying they moved the rover. A boot
        takes the rover to be where the map says it was parked instead -- see the
        module docstring for why a search nobody asked for is the wrong default.

        `around` is where to centre the search, and it defaults to where the
        rover thinks it is. The exception is a restored map whose anchor nothing
        has confirmed, on a rover that has not moved since: there the rover's own
        pose *is* the mapper's anchor, which is the error being corrected rather
        than evidence about it, and the pose the map was left at is the honest
        centre. Measured on 2026-09-06, an anchor 81 degrees out left the truth
        outside any window centred on it and every refit was refused.

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
                if (around is None and not self.map_trustworthy()
                        and self.still_parked()):
                    around = self.map_parked_at
                answer = self.map_fit_now(window_m, window_deg, min_score,
                                          around)
                self.map_fit = answer
                return answer
        finally:
            self.move_mutex.release()

    def map_fit_now(self, window_m=None, window_deg=None, min_score=None,
                    around=None):
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
        fit = refit.fit(grid, points, where if around is None else around,
                        window_m=refit.WINDOW_M if window_m is None else window_m,
                        window_deg=(refit.WINDOW_DEG if window_deg is None
                                    else window_deg),
                        min_score=(refit.MIN_SCORE if min_score is None
                                   else min_score),
                        was=where)
        answer = dict(fit.as_dict())
        answer["took_s"] = round(time.monotonic() - started, 2)
        answer["was"] = {"x_m": round(where[0], 3), "y_m": round(where[1], 3),
                         "heading_deg": round(where[2], 1)}
        if not fit.ok or fit.settled:
            answer["fitted"] = False
            # A fit that agrees with where the rover already is has confirmed the
            # pose just as firmly as one that corrects it, so the keeper may write
            # it down. A refusal has confirmed nothing and must not.
            if fit.ok:
                self.map_settled = True
            return answer

        saved, why = self.save_graph()
        if not saved:
            answer["fitted"] = False
            answer["why"] = ("the rover is %.0f cm and %.1f degrees from where it "
                             "thinks it is, and it was left there: %s, and moving "
                             "it means loading the graph again"
                             % (100.0 * fit.moved_m, fit.turned_deg, why))
            return answer
        loaded, why, _landed = self.load_graph(
            (fit.x_m, fit.y_m, fit.heading_deg))
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
        # The rover's place on the map has now been checked against a scan, so
        # the keeper may write it down again -- and it must, straight away. The
        # save above recorded where the rover was *before* the fit, which is the
        # pose this has just proved wrong, and nothing else would rewrite it
        # until the wheels turned: a rover that refitted and then sat still used
        # to leave the wrong pose on disk for the next boot to restore at.
        self.map_settled = True
        self.keep_pose(self.travelled_deg())
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
