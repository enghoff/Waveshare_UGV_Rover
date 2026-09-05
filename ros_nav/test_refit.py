"""Keeping the map between sessions, and finding the rover on it again.

Two modules with no ROS in them, so what runs here is what runs on the rover:
`refit.py` decides where the rover is and `mapstore.py` decides when the pose
graph is written. The fitting is argued against the occupancy grid the recorded
`kitchen-loop` drive produced, because the only honest test of a scan matcher is
a room with the mess in it -- a hand-drawn rectangle is a room where everything
fits everywhere.

The scans here are cast from that map rather than recorded, and that limits what
these checks can prove: they can show that the search finds a pose the map itself
explains, and they cannot show what a real lidar's noise does to the score. That
is what the thresholds in `refit.py` were measured against and what the first
press of the console button on the rover is for.
"""
import math
import os
import tempfile
import time

import mapstore
import refit
from frontier import Grid, read_pgm
from test_harness import HERE, check, section

#: Three places on the kitchen-loop map with floor around them, in different
#: rooms. Metres in the map's own frame, which for a PGM read back is measured
#: from its bottom-left corner.
SPOTS = ((2.48, 3.53), (5.28, 9.12), (8.78, 11.23))

#: What the rover is told it is at when it is really at one of those: a third of
#: a metre and fifteen degrees out, which is a rover somebody has nudged and
#: turned while it was switched off.
NUDGE = (0.35, -0.25, 15.0)


def _map():
    return read_pgm(os.path.join(HERE, "fixtures", "kitchen-loop.pgm.gz"))


def _scan(grid, pose, drop=0, move=0):
    """What the lidar would see at `pose`, optionally with the room changed.

    `drop` and `move` are one in N: a dropped return is something the sensor got
    no echo from -- a black sofa, a glass door -- and a moved one is something
    that is not where the map has it, which is a chair pushed out or a person
    standing in the room. Both are what separate a scan taken today from the map
    drawn last week.
    """
    rays = refit.cast(grid, pose)
    out = []
    for i, r in enumerate(rays):
        if not math.isfinite(r):
            out.append(r)
        elif drop and i % drop == 0:
            out.append(float("inf"))
        elif move and i % move == 0:
            out.append(max(0.25, r * 0.6))
        else:
            out.append(r)
    return refit.points_of(out, -math.pi, 2.0 * math.pi / len(out), 0.12, 8.0)


def _corridor():
    """A two-metre corridor forty metres long: the same room everywhere along it.

    The one shape a planar lidar genuinely cannot place itself in, and the reason
    `refit.py` reports a rival at all. Nothing about this is a caricature -- a
    house has hallways, and this is what one looks like to a scan matcher when
    both ends are out of range.
    """
    width, height, res = 800, 40, 0.05
    data = [0] * (width * height)
    for col in range(width):
        data[col] = 100
        data[(height - 1) * width + col] = 100
    return Grid(width, height, res, 0.0, 0.0, data)


def test_a_nudged_rover_is_found_again():
    """The headline: told it is somewhere it is not, it works out where it is.

    A third of a metre and fifteen degrees is the case this exists for -- a rover
    pushed aside and turned a little while it was switched off, coming back up on
    the map it was parked on. What is checked is the answer rather than the
    score: within a few centimetres of the truth, in every room tried.
    """
    section("finding a nudged rover on the map it already has")
    grid = _map()
    for x, y in SPOTS:
        for heading in (20.0, -140.0):
            truth = (x, y, heading)
            guess = (x + NUDGE[0], y + NUDGE[1], heading + NUDGE[2])
            fit = refit.fit(grid, _scan(grid, truth), guess)
            check("at %.1f,%.1f facing %.0f the fit is trusted" % truth, fit.ok)
            check("...and lands within 6 cm of where the rover really is",
                  math.hypot(fit.x_m - x, fit.y_m - y) < 0.06)
            check("...and within two degrees of its real heading",
                  abs((fit.heading_deg - heading + 180) % 360 - 180) < 2.0)
            check("...and says the rover was about 43 cm out",
                  fit.moved_m, 0.43, tolerance=0.04)


def test_what_the_sensor_never_heard_does_not_stop_it_fitting():
    """About one return in six from this lidar is a no-echo, and that is normal.

    A black sofa and a glass door give nothing back, so a scan is always missing
    a share of itself. Those are dropped rather than laid on the map at 8 m, and
    what is checked here is that dropping them costs nothing: an eighth of the
    scan silent fits exactly as well as all of it.
    """
    section("a room with things the lidar cannot hear")
    grid = _map()
    x, y = SPOTS[1]
    truth = (x, y, 65.0)
    guess = (x + NUDGE[0], y + NUDGE[1], 65.0 + NUDGE[2])
    fit = refit.fit(grid, _scan(grid, truth, drop=8), guess)
    check("an eighth of the scan silent still fits", fit.ok)
    check("...to within 10 cm", math.hypot(fit.x_m - x, fit.y_m - y) < 0.10)
    check("...at very nearly the score a whole scan gets",
          fit.score > 0.95, True)


def test_a_room_that_has_changed_is_refused_rather_than_guessed_at():
    """The honest limit of the feature, and the one worth pinning down.

    A quarter of the returns landing somewhere the map has nothing is not a rover
    that has moved, it is a room that has -- and the score cannot tell those two
    apart, because both look like a scan that does not lie on the map. So it says
    so rather than picking the best of a bad set: the refusal names both
    possibilities and gives the number, which is what a person needs to decide
    which of them they are looking at.

    **This is why the thresholds are set against real recorded lidar and not
    against this.** A cast scan whose returns have been dragged to six tenths of
    their range is a harsher room than any real one -- the real map's walls are
    several cells thick and a real chair that has moved is usually still near
    something -- so the score here (0.83 to 0.88) sits below what the same rover
    measures against a real map of a real room (0.94 to 0.98).
    """
    section("a room with a quarter of it moved since the map was drawn")
    grid = _map()
    x, y = SPOTS[1]
    truth = (x, y, 65.0)
    guess = (x + NUDGE[0], y + NUDGE[1], 65.0 + NUDGE[2])
    fit = refit.fit(grid, _scan(grid, truth, drop=8, move=4), guess)
    check("a quarter of the room moved is not answered", fit.ok, False)
    check("...and the sentence offers both readings of it, because the number "
          "cannot tell them apart",
          "room that has changed" in fit.why and "moved further" in fit.why, True)
    check("...with the number it fell short by in it",
          "%.0f%%" % (100 * refit.MIN_SCORE) in fit.why, True)


def test_a_rover_that_has_not_moved_is_left_alone():
    """Agreement is not a correction, and saying so is the point.

    Applying a fit means writing the pose graph and reading it back, which jumps
    the rover on the console and costs a second or two. Doing that to move the
    rover a centimetre would make the button feel like it had gone wrong.
    """
    section("a rover standing where it thinks it is")
    grid = _map()
    x, y = SPOTS[0]
    fit = refit.fit(grid, _scan(grid, (x, y, 10.0)), (x, y, 10.0))
    check("the fit is trusted", fit.ok)
    check("...and reports that there was nothing to move", fit.settled)
    check("...having found the rover within a centimetre or two of itself",
          fit.moved_m < 0.05, True)


def test_a_corridor_is_refused_rather_than_guessed_at():
    """Two places that fit equally well are not an answer, they are a coin toss.

    This is the guard that stops the feature being dangerous. A wrong fit that is
    *reported* costs a person a sentence; a wrong fit that is applied costs them a
    rover driving on a map it is not on.
    """
    section("a corridor that looks the same everywhere along it")
    grid = _corridor()
    truth = (20.0, 1.0, 0.0)
    fit = refit.fit(grid, _scan(grid, truth), (20.4, 1.0, 0.0))
    check("the corridor is not answered", fit.ok, False)
    check("...because something else fits it about as well",
          fit.rival > fit.score * 0.8, True)
    check("...and the sentence says which of the two refusals this is",
          "two different places" in fit.why, True)


def test_a_fit_can_never_move_the_rover_further_than_its_window():
    """The promise the whole feature rests on.

    A rover that has been *carried* cannot be found by a search around where it
    thinks it is, and the honest response to that is a bounded error rather than a
    clever one: whatever this answers, it is inside the window it searched. So the
    worst a wrong answer can do is the same order as the error it exists to
    remove, and pressing the button again fixes it.
    """
    section("a rover that was carried, which is outside what this can do")
    grid = _map()
    x, y = SPOTS[2]
    for away in (2.0, -3.0):
        guess = (x + away, y - away / 2.0, 40.0)
        fit = refit.fit(grid, _scan(grid, (x, y, 40.0)), guess)
        moved = math.hypot(fit.x_m - guess[0], fit.y_m - guess[1])
        check("carried %.0f m: the answer stays inside the window" % (away,),
              moved <= refit.WINDOW_M + 0.01, True)
        check("...and inside the angle it searched",
              abs((fit.heading_deg - guess[2] + 180) % 360 - 180)
              <= refit.WINDOW_DEG + 0.01, True)


def test_a_scan_with_nothing_in_it_is_not_matched():
    """A revolution this thin is a blocked sensor, not an empty room."""
    section("a scan with almost nothing in it")
    grid = _map()
    fit = refit.fit(grid, [(1.0, 0.0)] * 4, (SPOTS[0][0], SPOTS[0][1], 0.0))
    check("four returns are not a scan", fit.ok, False)
    check("...and it says so in the words a person would use",
          "not enough" in fit.why, True)


def test_the_scan_arrives_in_the_rovers_own_frame():
    """x forward, y left, and nothing beyond the sensor's own limits.

    Worth checking rather than assuming, because a sign flip here is a rover that
    fits its scan to the map back to front and is confident about it.
    """
    section("a laser scan as points")
    quarter = math.pi / 2.0
    points = refit.points_of([2.0, 3.0, 4.0, 5.0], -math.pi, quarter, 0.12, 8.0)
    check("four returns become four points", len(points), 4)
    check("...the one straight behind is behind", round(points[0][0], 3), -2.0)
    check("...the one to the right is to the right", round(points[1][1], 3), -3.0)
    check("...the one straight ahead is ahead", round(points[2][0], 3), 4.0)
    check("...and the one to the left is to the left", round(points[3][1], 3), 5.0)
    thin = refit.points_of([float("inf"), 0.05, 40.0, 1.0], -math.pi, quarter,
                           0.12, 8.0)
    check("no echo, too near and too far are all dropped rather than clamped",
          len(thin), 1)


def test_the_map_is_smeared_so_the_search_can_find_the_peak():
    """A scan point near a wall has to score more than one nowhere near it.

    Against bare occupied cells the score is zero everywhere except exactly on
    the answer, and a search stepping in 10 cm walks straight over it.
    """
    section("the map as something a scan can be scored against")
    data = [0] * (20 * 20)
    data[10 * 20 + 10] = 100
    for cell in range(15 * 20, 20 * 20):
        data[cell] = -1                     # five rows nobody has mapped
    walls, known = refit.field(Grid(20, 20, 0.05, 0.0, 0.0, data))
    check("the wall itself scores full", round(float(walls[10][10]), 3), 1.0)
    check("...five centimetres off scores less but not nothing",
          0.5 < float(walls[10][11]) < 1.0, True)
    check("...and a quarter of a metre off scores nothing at all",
          float(walls[10][15]), 0.0)
    # And the other half of the map: where it has an opinion at all, which is what
    # decides whether a scan point is scored or passed over.
    check("mapped floor is somewhere a scan point counts",
          float(known[2][2]), 1.0)
    check("...and so is the far side of a wall, within the smear, because a "
          "point a cell out is what a fit is correcting",
          float(known[16][2]), 1.0)
    check("...but open unmapped ground is not",
          float(known[19][19]), 0.0)


def test_the_saved_map_is_only_usable_once_all_three_files_are_there():
    """The note is written last, and it is what says the pair is complete.

    A power cut during a write leaves a truncated graph, and a rover that loaded
    one would come up with half a house. What it does instead is come up with no
    saved map at all, which is exactly what it did before any of this existed.
    """
    section("what makes a saved map loadable")
    with tempfile.TemporaryDirectory() as directory:
        saved = mapstore.SavedMap(directory)
        check("nothing saved yet", saved.held(), None)
        saved.make()
        for path in saved.graph_paths(saved.staging_stem):
            open(path, "w").write("graph")
        check("a graph written but not yet committed is not offered",
              saved.held(), None)
        note = saved.commit("map-one", (1.5, -2.5, 90.0))
        check("committing it makes it loadable", bool(saved.held()), True)
        check("...under the identity it was given",
              saved.held()["map_id"], "map-one")
        check("...remembering where the rover was standing",
              saved.start_pose(), (1.5, -2.5, 90.0))
        check("...and the staging copy is gone rather than left to be found",
              os.path.exists(saved.graph_paths(saved.staging_stem)[0]), False)
        check("...with the note carrying when it happened",
              note["saved_at"] > 0, True)

        os.remove(saved.graph_paths()[1])
        check("a note whose graph has lost a file is not loadable",
              saved.held(), None)


def test_a_cleared_map_does_not_come_back_at_the_next_boot():
    """The one outcome somebody pressing "clear map" cannot have meant."""
    section("clearing the map clears the saved one")
    with tempfile.TemporaryDirectory() as directory:
        saved = mapstore.SavedMap(directory)
        saved.make()
        for path in saved.graph_paths(saved.staging_stem):
            open(path, "w").write("graph")
        saved.commit("map-one", (0.0, 0.0, 0.0))
        saved.forget()
        check("nothing is left to load", saved.held(), None)
        check("...and nothing is left on disk either",
              [n for n in os.listdir(directory)], [])
        check("forgetting a map that is not there is not an error",
              saved.forget(), None)


def test_the_graph_is_written_for_driving_rather_than_for_time():
    """A parked rover adds no nodes, so a second copy of the same graph is bytes
    for nothing -- `minimum_travel_distance` in the mapper's config is the same
    rule, one layer down.

    And it is the wheels that are asked, not the rover's belief about where it is
    on the map. Those differ for a parked rover: `map -> odom` is only corrected
    when a scan is folded in, so between scans the gyro's residual bias walks the
    believed heading round at 0.8 degrees a minute -- measured on the rover -- and
    half an hour of standing still would otherwise look like a rover that had
    turned twenty degrees and get the drifted heading written down as where the
    map was left.
    """
    section("when the graph is worth writing again")
    with tempfile.TemporaryDirectory() as directory:
        saved = mapstore.SavedMap(directory)
        now = time.monotonic()
        check("with nothing saved, anything is worth saving",
              saved.due((0.0, 0.0, 0.0), now), True)
        check("...but not without a reading to compare against",
              saved.due(None, now), False)
        saved.make()
        for path in saved.graph_paths(saved.staging_stem):
            open(path, "w").write("graph")
        saved.commit("map-one", (3.0, 4.0, 90.0), odom=(0.0, 0.0, 0.0))
        later = saved.saved_at + mapstore.SAVE_EVERY_S + 1.0
        check("a rover that has just been saved is not saved again",
              saved.due((5.0, 5.0, 0.0), saved.saved_at + 1.0), False)
        check("...nor a parked one a minute later",
              saved.due((0.0, 0.0, 0.0), later), False)
        check("but one that has driven half a metre since is",
              saved.due((0.6, 0.0, 0.0), later), True)
        check("...and so is one that has only turned",
              saved.due((0.0, 0.0, 30.0), later), True)
        check("and what was written down is where the rover is on the map, "
              "which is not what decided to write it",
              saved.start_pose(), (3.0, 4.0, 90.0))


def test_where_the_rover_is_survives_a_power_cut_between_graph_writes():
    """The pose is written at the pace the rover moves, not the graph's.

    This is the fault of 2026-09-05, in a temporary directory. The rover went
    down mid-drive four seconds into a graph write; the serialisation never
    finished, so what a boot found was the note from the save before it, up to a
    minute of driving earlier. It came back where that note said, matched the
    map there at 56% where a fit needs 90%, and stood a metre outside the window
    `refit.py` is allowed to search -- lost on a map it had kept intact.
    """
    section("where the rover is, between graph writes")
    with tempfile.TemporaryDirectory() as directory:
        saved = mapstore.SavedMap(directory)
        check("with no map saved there is nowhere to write a pose",
              saved.note_pose("map-one", (1.0, 2.0, 30.0)), False)
        saved.make()
        for path in saved.graph_paths(saved.staging_stem):
            open(path, "w").write("graph")
        check("...and a graph not yet committed is not a map either",
              saved.note_pose("map-one", (1.0, 2.0, 30.0)), False)

        saved.commit("map-one", (0.0, 0.0, 0.0), odom=(0.0, 0.0, 0.0))
        graph_written = os.path.getmtime(saved.graph_paths()[0])
        check("a rover that has just been written down is left alone",
              saved.pose_due((0.0, 0.0, 0.0), saved.posed_at + 0.1), False)
        later = saved.posed_at + mapstore.POSE_EVERY_S + 0.1
        check("...and a second later, still, if it has not moved",
              saved.pose_due((0.0, 0.0, 0.0), later), False)
        check("but ten centimetres of driving is worth writing down",
              saved.pose_due((0.11, 0.0, 0.0), later), True)
        check("...and so is a few degrees of turning",
              saved.pose_due((0.0, 0.0, 4.0), later), True)
        check("long before the graph itself is worth writing again",
              saved.due((0.11, 0.0, 4.0), later), False)

        check("writing it says so", saved.note_pose(
            "map-one", (3.5, -1.25, 91.4), odom=(0.11, 0.0, 4.0)), True)
        check("a boot now starts where the rover last was",
              saved.start_pose(), (3.5, -1.25, 91.4))
        check("...under the identity it always had",
              saved.held()["map_id"], "map-one")
        check("...with the graph underneath it untouched",
              os.path.getmtime(saved.graph_paths()[0]), graph_written)
        check("...and the pose written down as its own moment",
              saved.held()["pose_at"] > 0, True)

        check("a pose for a map this is not is refused",
              saved.note_pose("map-two", (9.0, 9.0, 0.0)), False)
        check("...leaving the one that belongs to this map",
              saved.start_pose(), (3.5, -1.25, 91.4))
        check("and nothing to write to once the map is cleared",
              (saved.forget(), saved.note_pose("map-one", (1.0, 1.0, 0.0)))[1],
              False)


def test_the_keeper_writes_the_pose_when_it_is_not_writing_the_graph():
    """The rule above is only worth having if the keeper actually asks.

    Read as text, like the check below it and for the same reason: the keeper
    needs rclpy and this file runs on a workstation.
    """
    section("what the map keeper does with a tick")
    path = os.path.join(HERE, "nav_map.py")
    if not os.path.exists(path):
        return
    with open(path, encoding="utf-8") as fh:
        source = fh.read()
    loop = source[source.index("def _map_loop"):source.index("def travelled_deg")]
    check("the tick that does not write a graph writes the pose",
          "self.saved.pose_due(odom)" in loop, True)
    check("...and it is one or the other rather than both",
          "elif" in loop, True)
    keeping = source[source.index("def keep_pose"):source.index("def load_graph")]
    check("...from the map frame, for the map the rover has now",
          "self.saved.note_pose(self.map_id, self.pose_deg(), odom)" in keeping,
          True)


def test_an_empty_graph_is_never_written_down():
    """A saved empty map is worse than no saved map, and it reads as a success.

    Read as text because the keeper needs rclpy and this file runs on a
    workstation. The rule is worth pinning even so: a restore of an empty graph
    comes back saying the map was kept, so the world state keeps coordinates it
    recorded in a frame that has gone -- while the rover, with nothing to anchor
    on, quietly starts a new map wherever odometry happens to begin. The two
    together are exactly the fault map sessions exist to prevent.
    """
    section("what the keeper refuses to write")
    path = os.path.join(HERE, "nav_map.py")
    if not os.path.exists(path):
        return
    with open(path, encoding="utf-8") as fh:
        source = fh.read()
    saving = source[source.index("def save_graph"):source.index("def load_graph")]
    check("saving asks whether there is a map at all",
          "self.map_msg is not None" in saving, True)
    check("...before it asks slam_toolbox for anything",
          saving.index("self.map_msg is not None")
          < saving.index("serialize_client.wait_for_service"), True)


TESTS = (
    test_a_nudged_rover_is_found_again,
    test_what_the_sensor_never_heard_does_not_stop_it_fitting,
    test_a_room_that_has_changed_is_refused_rather_than_guessed_at,
    test_a_rover_that_has_not_moved_is_left_alone,
    test_a_corridor_is_refused_rather_than_guessed_at,
    test_a_fit_can_never_move_the_rover_further_than_its_window,
    test_a_scan_with_nothing_in_it_is_not_matched,
    test_the_scan_arrives_in_the_rovers_own_frame,
    test_the_map_is_smeared_so_the_search_can_find_the_peak,
    test_the_saved_map_is_only_usable_once_all_three_files_are_there,
    test_a_cleared_map_does_not_come_back_at_the_next_boot,
    test_the_graph_is_written_for_driving_rather_than_for_time,
    test_where_the_rover_is_survives_a_power_cut_between_graph_writes,
    test_the_keeper_writes_the_pose_when_it_is_not_writing_the_graph,
    test_an_empty_graph_is_never_written_down,
)
