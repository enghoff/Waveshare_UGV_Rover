"""Offline checks for the ROS 2 navigation backend, with no ROS anywhere near it.

Everything here runs against `ros_navigator.RosNavigator`, which is the daemon's
half of the bridge and is deliberately plain sockets and arithmetic. A fake bridge
on a real loopback port stands in for the ROS side, which is worth the few lines:
the protocol is the interface between two processes in two different Pythons, and
a mismatch in it is exactly the class of fault that cannot be caught by reading
either side on its own.

What is *not* covered is whether Nav2 drives well. That needs a rover, and
[README.md](../ros_nav/README.md) says how to check it there.

The one thing here that is worth the closest reading is the grid placement. Two
conversions happen between what slam_toolbox publishes and what the map renderer
expects -- an axis order and an origin offset -- and both are invisible when
wrong: a transposed map is still a plausible-looking room, and a mis-placed one
still draws walls. The tests build a map with a deliberately asymmetric mark in
it, so that a transpose cannot pass.
"""
from __future__ import annotations

import base64
import json
import os
import socket
import socketserver
import sys
import threading
import time
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from test_harness import SKIP, check


# --- a bridge that is not there ------------------------------------------------
def _closed_port() -> int:
    """A port with nothing listening on it, for the down-bridge tests.

    Bound and released rather than picked, because a hard-coded number is a test
    that fails on whichever machine happens to be running something there.
    """
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class FakeBridge:
    """The ROS side of the protocol, as a script of canned replies.

    `answers` maps an op to either a dict -- one reply, which is what the real
    bridge does for a read -- or a list of dicts, written in order, which is how a
    move streams progress and then an outcome. The string `"close"` anywhere in
    that list hangs up instead of writing, and it is there for one case: a bridge
    killed mid-move. The real one holds its connection open waiting for more
    requests, so a fake that merely stopped writing would leave the client
    blocked until its own quiet timeout four minutes later -- which is a hung test
    suite rather than a failing one.
    """

    def __init__(self, answers):
        self.answers = answers
        self.seen = []
        bridge = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                for raw in self.rfile:
                    raw = raw.strip()
                    if not raw:
                        continue
                    request = json.loads(raw)
                    bridge.seen.append(request)
                    reply = bridge.answers.get(request.get("op"))
                    if reply is None:
                        reply = {"kind": "reply", "ok": False,
                                 "error": "no such op"}
                    for line in (reply if isinstance(reply, list) else [reply]):
                        if line == "close":
                            return
                        self.wfile.write(json.dumps(line).encode() + b"\n")
                        self.wfile.flush()

        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        self.server = Server(("127.0.0.1", 0), Handler)
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.server.shutdown()
        self.server.server_close()


# --- the grid ------------------------------------------------------------------
def _payload(width, height, cells, resolution, origin_x, origin_y, values):
    """A `map` reply shaped exactly as nav_bridge.grid() builds one."""
    raw = bytes(bytearray((v & 0xFF) for v in values))
    return {"ok": True, "width": width, "height": height,
            "resolution_m": resolution, "origin_x_m": origin_x,
            "origin_y_m": origin_y, "cells": cells,
            "data": base64.b64encode(zlib.compress(raw, 6)).decode("ascii"),
            "pose": {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0},
            "trail": []}


def test_a_ros_map_lands_where_it_belongs():
    """The map goes into the square grid at the right place and the right way up.

    The mark is a single occupied cell one step along ROS's x and two along its
    y, in a map whose origin is one metre behind and one metre to the right of
    where the rover started. So the answer is arithmetic that can be done by
    hand: the cell sits at (-1.0 + 0.05, -1.0 + 0.10) in metres, which is 19 cells
    back and 18 cells right of the middle of the grid.

    A transpose would put it at 18 back and 19 right, which is why the offsets
    along the two axes are different numbers. Getting this wrong draws a room that
    looks entirely reasonable and is reflected about its own diagonal.
    """
    try:
        import numpy  # noqa: F401
    except ImportError:
        SKIP.append("the ROS grid adapter needs numpy")
        return
    import ros_navigator

    width, height, cells = 5, 4, 100
    values = [-1] * (width * height)
    values[2 * width + 1] = 100          # ROS is row-major: [y][x], so x=1, y=2
    values[0] = 0                        # and one free cell at the very corner
    grid = ros_navigator._GridSlam(
        _payload(width, height, cells, 0.05, -1.0, -1.0, values))

    array = grid.grid()
    check("the square grid is the size the renderer was told",
          array.shape, (cells, cells))
    middle = cells // 2
    check("the occupied cell is 19 cells behind the origin",
          int(array[middle - 19, middle - 18]), ros_navigator.GRID_OCCUPIED)
    check("...and not at the transpose of that, which would look just as sane",
          int(array[middle - 18, middle - 19]), 0)
    check("the free cell is free, not unknown",
          int(array[middle - 20, middle - 20]), ros_navigator.GRID_FREE)
    check("everything the map did not mention is never-seen",
          int(array[0, 0]), 0)


def test_the_three_occupancy_states_survive_the_trip():
    """ROS says -1, 0 and 100; the renderer wants nothing, negative and high.

    The middle band matters least and is the one most likely to be got wrong: a
    cell slam_toolbox is unsure about must not come out as either free or
    occupied, because the renderer paints those as the room and the walls.
    """
    try:
        import numpy  # noqa: F401
    except ImportError:
        SKIP.append("the occupancy coding needs numpy")
        return
    import ros_navigator

    values = [-1, 0, 24, 25, 64, 65, 100]
    grid = ros_navigator._GridSlam(
        _payload(len(values), 1, 60, 0.05, 0.0, 0.0, values))
    array = grid.grid()
    middle = 60 // 2
    got = [int(array[middle + i, middle]) for i in range(len(values))]
    check("unknown, free, free, dim, dim, occupied, occupied",
          got, [0, ros_navigator.GRID_FREE, ros_navigator.GRID_FREE,
                ros_navigator.GRID_DIM, ros_navigator.GRID_DIM,
                ros_navigator.GRID_OCCUPIED, ros_navigator.GRID_OCCUPIED])
    check("and the threshold the renderer compares against is above the dim band",
          ros_navigator.GRID_DIM < grid.config.occupied_at
          <= ros_navigator.GRID_OCCUPIED, True)


def test_a_map_bigger_than_the_grid_is_clipped_not_fatal():
    """A room mapped past 40 m across is a real thing, and drawing it must not
    raise in the middle of a picture somebody asked for."""
    try:
        import numpy  # noqa: F401
    except ImportError:
        SKIP.append("the clipping check needs numpy")
        return
    import ros_navigator

    # 30 cells at 5 cm placed 20 m out on a grid only 1 m across: entirely off it.
    grid = ros_navigator._GridSlam(
        _payload(30, 30, 20, 0.05, 20.0, 20.0, [100] * 900))
    check("a map wholly off the grid draws an empty one rather than raising",
          int(grid.grid().max()), 0)

    # Straddling the edge: half in, half out.
    grid = ros_navigator._GridSlam(
        _payload(30, 30, 20, 0.05, -0.5, -0.5, [100] * 900))
    check("a map straddling the edge keeps the part that fits",
          int(grid.grid().max()), ros_navigator.GRID_OCCUPIED)


def test_a_map_that_lies_about_its_size_is_refused():
    """The width and height are the sender's claim about bytes somebody else
    compressed, and believing them would reshape into whatever numpy allowed."""
    try:
        import numpy  # noqa: F401
    except ImportError:
        SKIP.append("the size check needs numpy")
        return
    import ros_navigator

    try:
        ros_navigator._GridSlam(_payload(10, 10, 40, 0.05, 0.0, 0.0, [0] * 7))
    except ValueError:
        check("a map whose byte count contradicts its dimensions is refused",
              True, True)
    else:
        check("a map whose byte count contradicts its dimensions is refused",
              False, True)


# --- the protocol --------------------------------------------------------------
def test_an_offset_becomes_a_place_before_it_crosses_the_socket():
    """`drive_to(ahead, left)` is converted here, using the pose it asks for.

    Converting on the far side would measure the offset from wherever the rover
    had got to by the time the request arrived, which for a rover already moving
    is most of a metre. The rover is put at (2, 1) facing 90 degrees, so "one
    metre ahead" is one metre along +y and "one metre left" is one metre along
    -x: the answer is (1, 2), and a version that forgot the heading would say
    (3, 2).
    """
    import ros_navigator

    with FakeBridge({
        "status": {"kind": "reply", "ok": True,
                   "pose": {"x_m": 2.0, "y_m": 1.0, "heading_deg": 90.0}},
        "goto": [{"kind": "outcome", "reason": "arrived", "travelled_m": 1.41,
                  "turned_deg": 0.0}],
    }) as bridge:
        nav = ros_navigator.RosNavigator(port=bridge.port)
        outcome = nav.drive_to(ahead_m=1.0, left_m=1.0)

    check("the move arrives", outcome.reason, "arrived")
    sent = [r for r in bridge.seen if r.get("op") == "goto"]
    check("exactly one goal was sent", len(sent), 1)
    check("the offset was rotated into the map frame (x)",
          round(sent[0]["x_m"], 3), 1.0)
    check("...and (y)", round(sent[0]["y_m"], 3), 2.0)


def test_a_place_on_the_map_is_sent_untouched():
    """A tap on the console's map already names a point, so nothing is added to
    it -- and it must not pick up the rover's pose on the way past."""
    import ros_navigator

    with FakeBridge({
        "goto": [{"kind": "outcome", "reason": "arrived", "travelled_m": 2.0,
                  "turned_deg": 0.0}],
    }) as bridge:
        nav = ros_navigator.RosNavigator(port=bridge.port)
        nav.drive_to(x_m=-1.5, y_m=4.25)
        sent = [r for r in bridge.seen if r.get("op") == "goto"]

    check("a point goes as it was given", (sent[0]["x_m"], sent[0]["y_m"]),
          (-1.5, 4.25))
    check("...and no pose was asked for on the way",
          [r["op"] for r in bridge.seen], ["goto"])
    check("a click on the map says nothing about which way to face",
          sent[0]["yaw_deg"], None)

    # And the one caller that does. The console's world popup sends the rover to
    # look at something, so where it ends up pointing is the whole point of the
    # move: with no yaw the goal faces along the way it travelled, which for a
    # viewpoint chosen off to one side is the rover arriving with its back to the
    # thing it went to see.
    with FakeBridge({
        "goto": [{"kind": "outcome", "reason": "arrived", "travelled_m": 2.0,
                  "turned_deg": 90.0}],
    }) as bridge:
        nav = ros_navigator.RosNavigator(port=bridge.port)
        nav.drive_to(x_m=-1.5, y_m=4.25, heading_deg=135.0)
        sent = [r for r in bridge.seen if r.get("op") == "goto"]
        asked = nav.report.snapshot()["asked"]

    check("a viewpoint carries the way to be facing on arrival",
          sent[0]["yaw_deg"], 135.0)
    check("...and the commentary a console polls says so too",
          asked, {"x_m": -1.5, "y_m": 4.25, "heading_deg": 135.0})


def test_a_move_narrates_itself_while_it_runs():
    """The progress lines become the commentary a console polls for.

    This is the whole reason a move holds its connection. Without it the only
    thing either console could show for a minute-long drive is a stopwatch, and a
    route Nav2 refused looks exactly like one still being driven.
    """
    import ros_navigator

    with FakeBridge({
        "goto": [
            {"kind": "progress", "phase": "planning", "why": "the goal is with Nav2"},
            {"kind": "progress", "phase": "driving", "remaining_m": 2.4},
            {"kind": "progress", "phase": "driving", "recoveries": 1},
            {"kind": "progress", "phase": "driving", "remaining_m": 0.2},
            {"kind": "outcome", "reason": "arrived", "travelled_m": 2.5,
             "turned_deg": -3.0},
        ],
    }) as bridge:
        nav = ros_navigator.RosNavigator(port=bridge.port)
        # Watched from another thread, the way a console does: the move blocks and
        # the status poll must see the phases go by.
        seen = []
        stop = threading.Event()

        def watch():
            while not stop.is_set():
                seen.append(nav.report.snapshot().get("phase"))
        watcher = threading.Thread(target=watch, daemon=True)
        watcher.start()
        outcome = nav.drive_to(x_m=2.5, y_m=0.0)
        stop.set()
        watcher.join(timeout=2.0)

    check("the outcome is the last line, not the first", outcome.reason, "arrived")
    check("...and carries what the move measured", outcome.travelled_m, 2.5)
    final = nav.report.snapshot()
    check("the report ends on the outcome", final.get("phase"), "ended")
    check("...naming the same reason the caller was given",
          final.get("reason"), "arrived")
    check("a recovery is reported as a replan, which is what it is from outside",
          final.get("replans"), 1)
    check("the commentary was visible while the move ran, not only after it",
          "driving" in seen or "planning" in seen or "replanning" in seen, True)


def test_the_move_report_says_what_was_asked_for():
    """`asked` and `kind` are what the console turns into a sentence, and the
    vocabulary is the tool's own name so the panel and the transcript agree."""
    import ros_navigator

    with FakeBridge({
        "turn": [{"kind": "outcome", "reason": "arrived", "travelled_m": 0.0,
                  "turned_deg": 90.0}],
    }) as bridge:
        nav = ros_navigator.RosNavigator(port=bridge.port)
        nav.turn_in_place(90.0)
        snapshot = nav.report.snapshot()

    check("a turn is reported as turn_in_place", snapshot.get("kind"),
          "turn_in_place")
    check("...with the angle it was asked for", snapshot.get("asked"),
          {"angle_deg": 90.0})


def test_two_moves_at_once_is_refused_rather_than_queued():
    """Queueing would have the second caller drive to somewhere the first has
    since made wrong. The bridge refuses too; this is the near end of it."""
    import ros_navigator

    started = threading.Event()
    release = threading.Event()

    class Slow(ros_navigator.RosNavigator):
        def stream(self, request, phase):
            started.set()
            release.wait(5.0)
            from nav_types import Outcome
            return Outcome("arrived", 1.0, 0.0)

    nav = Slow(port=_closed_port())
    first = threading.Thread(target=lambda: nav.drive(1.0), daemon=True)
    first.start()
    started.wait(2.0)
    second = nav.drive(1.0)
    release.set()
    first.join(timeout=5.0)

    check("the second move is refused", second.reason, "busy")
    check("...and says why in words", "already running" in second.detail, True)


def test_a_bridge_that_is_not_there_is_a_sentence_not_a_traceback():
    """Every one of these results goes either into a model's context or onto a
    console panel, and both need words. An exception reaching the daemon's
    dispatcher would come back as a type name."""
    import ros_navigator

    nav = ros_navigator.RosNavigator(port=_closed_port())

    outcome = nav.drive(1.0)
    check("a drive against a dead bridge is blocked, not an exception",
          outcome.reason, "blocked")
    check("...and the detail says how to bring it back",
          "restart.sh" in outcome.detail, True)

    room = nav.describe()
    check("describe always carries the text the tools splice into a move",
          isinstance(room.get("text"), str), True)
    check("...and always carries clear_ahead_m, which a move reads by name",
          "clear_ahead_m" in room, True)

    status = nav.status(since_seq=None)
    for key in ("driving", "estop", "pose", "speed_ms", "turn_dps",
                "clearance_m", "steering_deg", "remaining_m", "match_score",
                "position_trusted", "scans", "dropped_scans", "pwm",
                "lidar_ok", "lidar_live", "scan_age_s", "lidar_resets",
                "move"):
        check("a dead bridge still fills the console's '%s' row" % key,
              key in status, True)
    check("...and does not claim the position is trusted",
          status["position_trusted"], False)


def test_a_stop_is_never_reported_as_a_failure():
    """The one reply this interface must not get wrong.

    Whoever is reading it has already decided the rover should not be moving, and
    "could not stop" is the worst sentence available. It is also not the truth: the
    driver board halts itself when the commands stop arriving, and a bridge that
    is not answering is a bridge that is not sending any.
    """
    import ros_navigator

    nav = ros_navigator.RosNavigator(port=_closed_port())
    got = nav.stop()
    check("a stop against a dead bridge still reports stopped",
          got.get("stopped"), True)
    check("...and explains what is actually stopping the wheels",
          "stops itself" in str(got.get("note")), True)


def test_a_bridge_that_dies_mid_move_stops_the_rover():
    """The dangerous case: the connection carrying a move goes away while the
    wheels are turning. Nothing else would then send a stop, so this does -- and
    it has to be on a fresh connection, because the one it would have used is the
    one that just died."""
    import ros_navigator

    with FakeBridge({
        # One progress line and then the connection goes away with no outcome ever
        # sent, which is what a bridge being killed mid-move looks like.
        "drive": [{"kind": "progress", "phase": "driving"}, "close"],
        "stop": {"kind": "reply", "ok": True, "stopped": True, "latched": False},
    }) as bridge:
        nav = ros_navigator.RosNavigator(port=bridge.port)
        outcome = nav.drive(1.0)
        ops = [r["op"] for r in bridge.seen]

    check("the move fails rather than claiming to have arrived",
          outcome.reason, "failed")
    check("...and a stop was sent on a new connection", "stop" in ops, True)


def test_exploring_does_not_wedge_every_other_tool():
    """Starting an explore has to answer at once, and it is not a convenience.

    Every client of this daemon holds one connection with one lock on it -- see
    `RoverClient` in voice_chat/rover_tools.py. So a tool call that waits for a
    ten-minute run holds that lock for ten minutes, and `stop_driving` queues
    behind it: the voice model would set the rover off and then be unable to stop
    it, with somebody in the room asking it to. The run therefore goes on a
    thread of its own and the call comes straight back.

    The fake bridge here never answers the explore, which is exactly what the
    real one does for ten minutes.
    """
    import ros_navigator

    with FakeBridge({
        # An explore that says it is planning and then says nothing more, which
        # is a run in progress as far as this end can tell.
        "explore": [{"kind": "progress", "phase": "choosing"}],
        "stop": {"kind": "reply", "ok": True, "stopped": True, "latched": False},
        "status": {"kind": "reply", "ok": True, "pose": None},
    }) as bridge:
        nav = ros_navigator.RosNavigator(port=bridge.port)

        began = time.monotonic()
        first = nav.explore_in_background(budget_s=600.0)
        took = time.monotonic() - began

        check("starting an explore answers at once rather than in ten minutes",
              took < 2.0, True)
        check("...and says it started one", first.get("started"), True)

        # The run is now in flight on its thread. The thing that must still work
        # is everything else, and stopping above all.
        deadline = time.monotonic() + 3.0
        while not nav.exploring and time.monotonic() < deadline:
            time.sleep(0.02)
        check("...and the navigator says it is exploring", nav.exploring, True)
        check("...and counts that as driving, so a drive_to is refused as busy",
              nav.driving, True)

        again = nav.explore_in_background(budget_s=600.0)
        check("asking again does not start a second run",
              again.get("started"), False)
        check("...and reports the one already going rather than stopping it",
              again.get("exploring"), True)

        began = time.monotonic()
        stopped = nav.stop()
        check("a stop gets through while the run is in flight",
              stopped.get("stopped"), True)
        check("...promptly, because it is not queued behind the run",
              time.monotonic() - began < 2.0, True)
        check("...and it reached the bridge",
              "stop" in [r["op"] for r in bridge.seen], True)


def test_a_trip_to_one_place_is_not_waited_for_either():
    """`go_to_thing` starts a drive that lasts a minute over the one connection a
    voice model has, so it goes through the same machinery exploring does.

    What has to hold on top of that is that the two are told apart: an errand is
    not an exploring run, `explore` must refuse while one is in flight rather
    than starting a second move, and what the trip was *for* has to survive it so
    that asking again is answered about the sofa rather than about a coordinate.
    """
    import ros_navigator

    with FakeBridge({
        # A goto that says it is planning and then says nothing more, which is a
        # trip in progress as far as this end can tell.
        "goto": [{"kind": "progress", "phase": "planning"}],
        "stop": {"kind": "reply", "ok": True, "stopped": True, "latched": False},
        "status": {"kind": "reply", "ok": True, "pose": None},
    }) as bridge:
        nav = ros_navigator.RosNavigator(port=bridge.port)

        began = time.monotonic()
        started = nav.drive_to_in_background(3.2, 3.0, heading_deg=0.0,
                                             for_what={"said": "the sofa"})
        check("setting off answers at once rather than in a minute",
              time.monotonic() - began < 2.0, True)
        check("...and says it started", started.get("started"), True)

        deadline = time.monotonic() + 3.0
        while nav.away != "errand" and time.monotonic() < deadline:
            time.sleep(0.02)
        check("...and the navigator says it is on an errand", nav.away, "errand")
        check("...which is not an exploring run", nav.exploring, False)
        check("...but is driving, so an ordinary move is refused as busy",
              nav.driving, True)
        check("...and remembers what the trip is for",
              nav.errand.get("said"), "the sofa")
        check("...and where it is going, so a console can draw it",
              (nav.errand.get("x_m"), nav.errand.get("y_m")), (3.2, 3.0))
        # The thread is running by the time `away` says so, which is a moment
        # before its socket has carried the request. Waited for rather than
        # assumed: this is the one check that the trip actually left the daemon.
        deadline = time.monotonic() + 3.0
        goto = [r for r in bridge.seen if r["op"] == "goto"]
        while not goto and time.monotonic() < deadline:
            time.sleep(0.02)
            goto = [r for r in bridge.seen if r["op"] == "goto"]
        check("...having asked the bridge to go there",
              (goto[0]["x_m"], goto[0]["yaw_deg"]) if goto else None, (3.2, 0.0))

        exploring = nav.explore_in_background(budget_s=600.0)
        check("exploring will not start on top of it",
              (exploring.get("started"), exploring.get("busy")), (False, True))
        check("...and does not claim the rover is exploring",
              exploring.get("exploring"), False)

        began = time.monotonic()
        stopped = nav.stop()
        check("a stop gets through while the trip is in flight",
              stopped.get("stopped"), True)
        check("...promptly, because it is not queued behind it",
              time.monotonic() - began < 2.0, True)


def test_the_map_is_refused_in_words_when_there_is_none():
    """`ValueError` on purpose: the daemon's dispatcher reports those as the
    sentence alone, and everything else with the exception class in front."""
    import ros_navigator

    with FakeBridge({
        "map": {"kind": "reply", "ok": False,
                "error": "slam_toolbox has not published a map yet"},
    }) as bridge:
        nav = ros_navigator.RosNavigator(port=bridge.port)
        try:
            nav.map_png()
        except ValueError as exc:
            check("no map is a ValueError carrying the reason",
                  "not published a map" in str(exc), True)
        except Exception as exc:
            check("no map is a ValueError carrying the reason",
                  "%s: %s" % (type(exc).__name__, exc), "a ValueError")
        else:
            check("no map is a ValueError carrying the reason", "no raise",
                  "a ValueError")


def test_resolution_is_readable_before_any_map_has_arrived():
    """`map_png` on the daemon reads `nav.slam.config.resolution_m` to work out
    the zoom *before* it asks for the picture, so the placeholder has to answer
    rather than being None."""
    import ros_navigator

    nav = ros_navigator.RosNavigator(port=_closed_port())
    check("the placeholder knows the resolution",
          nav.slam.config.resolution_m, ros_navigator.DEFAULT_RESOLUTION_M)
    check("...and the grid size the console's zoom limits assume",
          nav.slam.config.grid_cells, ros_navigator.GRID_CELLS)
    check("...and reports a pose rather than raising", nav.slam.pose,
          (0.0, 0.0, 0.0))


TESTS = (
    test_a_ros_map_lands_where_it_belongs,
    test_the_three_occupancy_states_survive_the_trip,
    test_a_map_bigger_than_the_grid_is_clipped_not_fatal,
    test_a_map_that_lies_about_its_size_is_refused,
    test_an_offset_becomes_a_place_before_it_crosses_the_socket,
    test_a_place_on_the_map_is_sent_untouched,
    test_a_move_narrates_itself_while_it_runs,
    test_the_move_report_says_what_was_asked_for,
    test_two_moves_at_once_is_refused_rather_than_queued,
    test_a_bridge_that_is_not_there_is_a_sentence_not_a_traceback,
    test_a_stop_is_never_reported_as_a_failure,
    test_a_bridge_that_dies_mid_move_stops_the_rover,
    test_exploring_does_not_wedge_every_other_tool,
    test_a_trip_to_one_place_is_not_waited_for_either,
    test_the_map_is_refused_in_words_when_there_is_none,
    test_resolution_is_readable_before_any_map_has_arrived,
)
