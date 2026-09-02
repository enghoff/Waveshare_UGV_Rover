"""Frames: waiting for the last one, and not replaying it.

A picture is the most expensive thing the console sends, so a browser that is
behind must not be sent a queue of stale ones, and a frame already seen must not
arrive twice.
"""
from __future__ import annotations

import io
import time

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import SKIP, check


def test_pictures_wait_for_the_last_one() -> None:
    """When the console asks the rover for the next map and the next frame.

    Both used to run on a clock started when the request went out, and the map's
    was two seconds. A map takes the rover longer than that to draw, so by the
    time one arrived its own timer had already expired and the console asked for
    the next in the same breath -- an unbroken queue of renders on the core that
    is also running SLAM, and the reason a stop could take a moment to be heard.

    The clock now starts when the picture lands, so whatever the rover charged
    for it is spent before the gap begins and the gap is really a gap. What is
    checked here is that ordering: nothing goes out while one is in flight,
    nothing goes out in the moment one arrives, and something goes out once the
    gap has passed since it arrived.

    And which gap, because there are two. Half a second of 28 kB camera frames and
    7 kB map renders is most of a megabit a second of somebody's wi-fi, and a rover
    standing still spends all of it redrawing the same picture: the map is drawn
    around a pose that has not moved. So the fast gap belongs to a move in flight
    and a parked rover is asked more slowly -- the camera more slowly, since a room
    can change while a rover does not, and the map slowest of all.
    """
    try:
        import drive_web
        from console_model import (PARKED_FRAME_GAP_S, PARKED_MAP_GAP_S,
                                   PICTURE_GAP_S, Reply)
    except ImportError as exc:
        SKIP.append(f"pictures wait for the last one ({type(exc).__name__})")
        return

    class Fake:
        """A channel that records rather than connects."""

        def __init__(self):
            self.sent = []

        def submit(self, name, arguments=None):
            self.sent.append(name)

    session = drive_web.Session(None, 3.0, 480)
    session.picture, session.camera = Fake(), Fake()
    # Enough of a link that the pump does not spend the test looking for a rover:
    # a channel in the list and an answer recently enough not to count as lost.
    session.channels = [session.picture]
    session.answered_at = time.monotonic()

    session.pump()
    check("a console with nothing on screen asks for a map at once",
          session.picture.sent, ["map_png"])
    check("...and for a frame", session.camera.sent, ["camera_jpeg"])
    session.pump()
    session.pump()
    check("neither is asked for again while the first is still coming",
          (len(session.picture.sent), len(session.camera.sent)), (1, 1))

    # The rover answers, and both took longer than the gap. Under the old clock
    # that alone made the next one due; under this one the gap has not started.
    session.handle(Reply("map_png", {}, {"ok": True, "png_base64": ""}, 2.4))
    session.handle(Reply("camera_jpeg", {}, {"ok": True, "jpeg_base64": ""}, 0.9))
    session.pump()
    check("a picture that took the rover seconds does not buy the next one early",
          (len(session.picture.sent), len(session.camera.sent)), (1, 1))

    # Half the gap after they landed: still too soon for either.
    session.map_done_at -= PICTURE_GAP_S / 2
    session.frame_done_at -= PICTURE_GAP_S / 2
    session.pump()
    check("...and neither goes out half way through the gap",
          (len(session.picture.sent), len(session.camera.sent)), (1, 1))

    # Past the fast gap, and still nothing: this rover is parked, and the fast gap
    # is for a rover that is doing something.
    session.map_done_at -= PICTURE_GAP_S
    session.frame_done_at -= PICTURE_GAP_S
    session.pump()
    check("a parked rover is not asked for a picture every half second",
          (len(session.picture.sent), len(session.camera.sent)), (1, 1))

    # Past the camera's own parked gap. The room can change while the rover does
    # not, so this one is slowed rather than stopped.
    session.frame_done_at -= PARKED_FRAME_GAP_S
    session.pump()
    check("...but its camera still comes, a couple of seconds at a time",
          session.camera.sent, ["camera_jpeg", "camera_jpeg"])
    check("...while the map, which cannot have changed, waits longer",
          session.picture.sent, ["map_png"])
    session.map_done_at -= PARKED_MAP_GAP_S
    session.pump()
    check("...until its own gap has passed", session.picture.sent,
          ["map_png", "map_png"])

    # A move in flight puts both back on the fast gap, which is the whole point:
    # now the pose is changing, so the next picture is a different picture.
    session.handle(Reply("map_png", {}, {"ok": True, "png_base64": ""}, 0.1))
    session.handle(Reply("camera_jpeg", {}, {"ok": True, "jpeg_base64": ""}, 0.1))
    session.busy_since, session.busy_name = time.monotonic(), "drive"
    session.map_done_at -= PICTURE_GAP_S * 1.5
    session.frame_done_at -= PICTURE_GAP_S * 1.5
    session.pump()
    check("a move in flight is worth a picture every half second again",
          (len(session.picture.sent), len(session.camera.sent)), (3, 3))


def test_pictures_are_not_replayed() -> None:
    """Two consoles must never publish a picture at the same URL.

    They did, and it was the worst-looking bug in this thing. Each map is served at
    `/map.png?gen=N` with N counting from 1 and a year of `immutable` on it, and N
    starts again at 1 in every new process -- so the second console handed a browser
    exactly the URLs the first had already filled its cache with, in the same order.
    The browser never asked about them again and drew the earlier run's pictures back
    frame by frame, over a live rover, the same run every time. Restarting did not
    help and neither did rebooting: the pictures were on disk in the browser profile.

    Reproduced by pointing one console at the mock rover and the next at the real one
    and logging what the server was asked for: the second console served a different
    picture at that URL and the browser fetched it zero times. So the guard is that
    the name of a picture belongs to the run that drew it.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"pictures are not replayed ({type(exc).__name__})")
        return

    one = drive_web.Session(None, 3.0, 480)
    two = drive_web.Session(None, 3.0, 480)

    check("a console with no picture yet publishes no name", one.tag(0), "")
    check("...which is what the page reads as nothing to show", bool(one.tag(0)),
          False)
    check("the first picture of a run is named", bool(one.tag(1)), True)
    check("...and pictures within one run differ", one.tag(1) != one.tag(2), True)

    # The whole point: same counter, different run, different URL.
    check("two consoles do not name their first picture the same",
          one.tag(1) != two.tag(1), True)
    check("...nor their tenth", one.tag(10) != two.tag(10), True)

    # And the header that made it permanent is only honest once that holds.
    check("a picture is still cacheable for a year",
          "immutable" in io.open(drive_web.__file__, encoding="utf-8").read(), True)


def test_tracking_while_the_rover_drives() -> None:
    """The camera can be started and stopped mid-move, and the panel says which
    of the two things it is doing with nobody in view.

    Both halves used to be wrong in the same direction. The rover took face
    tracking away from itself the moment the wheels turned, so the console greyed
    the two buttons out beside the driving ones and the panel only ever had a
    sweep to report. Neither is true now: tracking runs through a move, and it
    stops sweeping and watches where the rover is going instead.

    The buttons themselves are enabled by the page's own script, which is not
    reachable from here -- what this pins down is that the console sends the call
    on the connection that is free during a move rather than the one the move is
    occupying.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"tracking while driving ({type(exc).__name__})")
        return

    class Fake:
        def __init__(self):
            self.sent = []

        def submit(self, name, arguments=None):
            self.sent.append((name, arguments or {}))

    session = drive_web.Session(None, 3.0, 480)
    session.moves, session.watch = Fake(), Fake()
    session.tools = ["drive", "turn_in_place", "start_tracking", "stop_tracking"]
    session.can_drive = True
    session.busy_since, session.busy_name = time.monotonic(), "drive"

    session.act({"do": "track", "name": "start_tracking"})
    check("tracking starts while a move is running",
          [name for name, _ in session.watch.sent], ["start_tracking"])
    check("...on the status connection, not the one the move holds",
          session.moves.sent, [])

    session.show_tracking({"ok": True, "tracking": True, "following_someone": False,
                           "searching": "sweeping", "faces_in_view": 0})
    check("a parked rover with nobody in view is sweeping", session.track_text,
          "on, sweeping, nobody yet, 0 in view")
    session.show_tracking({"ok": True, "tracking": True, "following_someone": False,
                           "searching": "watching ahead", "faces_in_view": 0})
    check("...and a driving one is watching where it is going",
          session.track_text, "on, watching ahead, nobody yet, 0 in view")
    session.show_tracking({"ok": True, "tracking": True, "following_someone": True,
                           "faces_in_view": 1})
    check("somebody in view outranks both", session.track_text,
          "on, following someone, 1 in view")
    session.show_tracking({"ok": True, "tracking": True, "following_someone": False,
                           "faces_in_view": 0})
    check("a rover too old to say is taken as sweeping", session.track_text,
          "on, sweeping, nobody yet, 0 in view")


TESTS = (
    test_pictures_wait_for_the_last_one,
    test_pictures_are_not_replayed,
    test_tracking_while_the_rover_drives,
)
