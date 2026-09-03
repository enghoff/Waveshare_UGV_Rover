"""One console at a time: taking over, losing a browser, and finding the rover.

The console is a process that lives from boot and a browser that comes and goes,
so what is checked here is the seam between them -- who is being served, what
happens when a second browser clicks, and what a slow one is shown when it
finally reads.
"""
from __future__ import annotations

import io
import os
import re
import sys
import time

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import SKIP, check


def test_web_console() -> None:
    """The browser console's model, with no browser and no rover.

    Everything the page draws comes out of `Session`, so these are the panels
    themselves: the alarms that make a silent lidar unmissable, which networks are
    offered a join button, and how the map's two ladders answer a resized window.
    None of it needs a socket -- `Session` connects when its pump runs, and the pump
    is not started here.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"web console ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)

    # The status panel. The formatting lives in console_model.py; what is tested
    # here is that the alarm flag reaches the page, because a lidar that has
    # gone silent under motor load makes every other number on that panel a lie.
    session.show_status({"ok": True, "lidar_live": False, "lidar_ok": False,
                         "estop": False, "position_trusted": True, "speed_ms": 0.0,
                         "pose": {"x_m": 1.0, "y_m": -0.5, "heading_deg": 90.0}})
    rows = dict((row[0], row) for row in session.status_rows)
    check("a silent lidar says so in capitals", rows["lidar"][1], "SILENT")
    check("...and is flagged so the page can colour it", rows["lidar"][2], True)
    check("a stale match is not an alarm on its own", rows["matched"][2], False)
    check("and the pose reads as a place",
          session.pose_text, "x +1.00  y -0.50  +90.0 deg")

    # A status the rover could not answer must blank the numbers rather than leave
    # the last good ones on screen looking current.
    session.show_status({"ok": False, "error": "no navigator"})
    check("a refused status blanks the rows",
          set(row[1] for row in session.status_rows), {"-"})
    check("...and says why", session.status_error, "no navigator")

    # The network list. Joinable means configured and not the one already in use --
    # a network the rover holds no passphrase for is worth seeing in the list and is
    # not worth a button.
    session.show_wifi({"ok": True, "connected": "Sonic", "level_dbm": -42,
                       "address": "192.168.1.47", "list_age_s": 12.0,
                       "networks": [
                           {"ssid": "Sonic", "signal": 80, "in_use": True,
                            "configured": True},
                           {"ssid": "Sonic5", "signal": 61, "in_use": False,
                            "configured": True},
                           {"ssid": "next door", "signal": 44, "in_use": False,
                            "configured": False}]})
    check("a stale neighbour list is dated rather than crashing the pump",
          "list heard" in session.wifi["where"], True)

    session.show_battery({"ok": True, "volts": 12.28, "percent": 90,
                          "state": "ok", "reading_age_s": 25.0})
    check("a battery reading is shown", "12.28 V" in session.battery["text"], True)
    check("...and an old one is dated rather than crashing the pump",
          "ago" in session.battery["note"], True)
    offered = [n["ssid"] for n in session.wifi_networks if n["joinable"]]
    check("only a network it has a passphrase for is offered", offered, ["Sonic5"])
    check("the one it is on is named as such",
          session.wifi_networks[0]["note"], "on it")
    # The list is fetched rather than pushed, so the state carries a count and the
    # page asks again when it moves. Answering the same list twice must not move it,
    # or every poll would have the browser fetch three and a half kB again.
    gen = session.wifi_networks_gen
    session.set_networks(list(session.wifi_networks))
    check("an unchanged list is not a new list", session.wifi_networks_gen, gen)
    session.set_networks([])
    check("...and a changed one is", session.wifi_networks_gen, gen + 1)
    check("and the strong link is coloured as one", session.wifi["verdict"], "good")

    # An older daemon has none of these calls. Say so once and stop asking, rather
    # than painting the panel red every five seconds for the rest of the session.
    quiet = drive_web.Session(None, 3.0, 480)
    quiet.show_wifi({"ok": False, "error": "no such tool: wifi_status"})
    check("a daemon too old for the network calls is asked once",
          quiet.wifi_ok, False)

    # A scan that never reached the radio used to come back as "heard 1 network
    # in 0 s" with the daemon's explanation discarded. The panel has to keep
    # that sentence, or the next Banana Pi looks like a neighbourhood of one.
    from console_model import Reply
    explained = drive_web.Session(None, 3.0, 480)
    explained.handle(Reply("wifi_status", {"scan": True}, {
        "ok": True, "connected": "TheGreatLord",
        "networks": [{"ssid": "TheGreatLord", "signal": -35, "in_use": True,
                      "configured": True}],
        "note": "/usr/local/sbin/wifi_ctl.sh is not installed on this rover; "
                "run wifi_roam/install.sh",
    }, 0.2))
    check("a scan that could not look still says how many rows came back",
          "heard 1 network" in explained.wifi["note"], True)
    check("...and keeps the daemon's reason",
          "install" in explained.wifi["note"], True)

    # Zooming changes how much room is in the picture and never how big the picture
    # is. The rover derives pixels per cell from the two, so a zoom that resized the
    # picture would leave every cell the same size on screen, which is not zooming.
    before = session.map_size
    session.map_settings({"zoom": 1})
    check("widening the view shows more room", session.half_extent > 3.0, True)
    check("...in a picture that is still the same size", session.map_size, before)
    session.map_settings({"zoom": -1})
    check("...and closing it again comes back to the rung it left", session.half_extent, 3.0)

    # The map is square and the daemon says how big it came out, but a mock or an
    # older daemon may not -- and the page sets the panel's aspect ratio from this
    # number, so a wrong one puts a click somewhere else in the room.
    header = b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR" + (484).to_bytes(4, "big")
    check("the size can always be read off the picture itself",
          drive_web._png_width(header), 484)


def test_stopping_an_unwatched_rover() -> None:
    """The browser console's answer to a tab being closed mid-move.

    A desktop window sends a stop from its close handler. A browser tab that goes
    away says nothing at all and the server outlives it, so the promise is kept from
    the server's side instead: the event stream is the browser being present, and
    losing the last one while a move is running is treated as closing the window.

    The grace is the part worth testing. A reload tears the stream down and puts it
    back inside a few hundred milliseconds, and a console that stopped the rover for
    that would be unreloadable during the only minute it is interesting.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"stopping an unwatched rover ({type(exc).__name__})")
        return

    class Fake:
        """A channel that records rather than connects."""

        def __init__(self):
            self.sent = []

        def submit(self, name, arguments=None):
            self.sent.append(name)

    session = drive_web.Session(None, 3.0, 480)
    session.halt = Fake()
    session.busy_since = 100.0
    session.busy_name = "drive"

    # Somebody is looking, so nothing happens however long the move runs.
    session.listeners = 1
    session.mind_the_watchers(100.0)
    session.mind_the_watchers(200.0)
    check("a watched move is left alone", session.halt.sent, [])

    # The stream goes. Inside the grace this is a reload, not a departure.
    session.listeners = 0
    session.mind_the_watchers(200.0)
    session.mind_the_watchers(200.0 + drive_web.ORPHAN_GRACE_S / 2)
    check("a reload does not stop the rover", session.halt.sent, [])

    session.mind_the_watchers(200.0 + drive_web.ORPHAN_GRACE_S + 0.1)
    check("but a closed tab does", session.halt.sent, ["stop_driving"])
    # Once, not once per tick: the pump runs ten times a second, and a stop resent
    # ten times a second would bury the transcript meant to explain it.
    session.mind_the_watchers(260.0)
    check("...and only once", session.halt.sent, ["stop_driving"])

    # A rover doing nothing is not stopped for being unwatched. There is nothing to
    # stop, and the line it would write in the transcript would be a lie.
    idle = drive_web.Session(None, 3.0, 480)
    idle.halt = Fake()
    idle.mind_the_watchers(300.0)
    idle.mind_the_watchers(400.0)
    check("an idle rover is not stopped for being alone", idle.halt.sent, [])


def test_idle_console_waits_for_a_browser() -> None:
    """A console hosted on the rover must not be a client overnight.

    The desk process is started to drive and killed when you are done, so it
    connects at once and polls for as long as it lives. The same process started
    from boot would otherwise ask for nav_status three times a second and a map
    every two, for nobody, on the same machine that is running SLAM. `--idle`
    waits for a browser and drops the rover once the last tab has been gone for
    the orphan grace -- long enough that a reload is not a disconnect.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"idle console waits for a browser ({type(exc).__name__})")
        return

    class Fake:
        def __init__(self):
            self.sent = []

        def submit(self, name, arguments=None):
            self.sent.append(name)

        def close(self):
            pass

    quiet = drive_web.Session(None, 3.0, 480, idle=True)
    quiet.watch = Fake()
    quiet.picture = Fake()
    quiet.channels = [quiet.watch]
    quiet.poll_at = 0.0
    quiet.map_done_at = 0.0
    quiet.pump()
    check("an idle console does not poll with nobody watching",
          quiet.watch.sent, [])
    check("...nor ask for a map", quiet.picture.sent, [])
    check("...and still has the rover during a reload",
          bool(quiet.channels), True)

    quiet.alone_since = 100.0
    quiet.pump()
    # pump() uses time.monotonic(), so a stale alone_since from 100 is "long ago"
    # and rest() runs. The channels list is replaced, not mutated.
    check("once the last tab has been gone, it drops the rover",
          quiet.channels, [])
    check("...and says it is waiting", quiet.link_text, "waiting for a browser")

    watched = drive_web.Session(None, 3.0, 480, idle=True)
    watched.watch = Fake()
    watched.channels = [watched.watch]
    watched.listeners = 1
    watched.poll_at = 0.0
    watched.answered_at = time.monotonic()
    watched.pump()
    check("a watched idle console polls",
          "nav_status" in watched.watch.sent, True)

    drive_web.Handler.session = watched
    watched.address = "127.0.0.1:8769"
    body = drive_web.health()
    check("health is ok while it is serving", body["ok"], True)
    check("...and says how many browsers", body["watching"], 1)
    check("...and where the rover is", body["rover"], "127.0.0.1:8769")
    check("...and that it is the hosted kind", body["idle"], True)
    drive_web.Handler.session = None


def test_finding_the_rover_again() -> None:
    """A console that has lost its rover goes looking, without being asked.

    The button is the thing being tested away. A rover on wifi that has driven
    behind the boiler, or been power-cycled, or come back on another address, used
    to leave the page reading "no daemon answered" until somebody noticed and
    clicked -- which is the wrong thing to require at the moment the stop button has
    stopped working. So the clock and the decision are both here, driven with an
    explicit `now` rather than a sleep.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"finding the rover again ({type(exc).__name__})")
        return

    def recording(session):
        """A session whose `connect` records instead of opening sockets."""
        session.tried = []
        session.connect = lambda: session.tried.append("connect")
        return session

    # --- a link that is up ---------------------------------------------------
    live = recording(drive_web.Session(None, 3.0, 480))
    live.channels = ["a channel"]          # only its emptiness is ever read
    live.answered_at = 100.0
    live.mind_the_link(100.0 + drive_web.LINK_LOST_S / 2)
    check("a rover that is answering is left alone", live.tried, [])

    live.mind_the_link(100.0 + drive_web.LINK_LOST_S + 0.1)
    check("one that has gone quiet is reconnected", live.tried, ["connect"])

    # A move in flight owns the link: the move channel waits longer than this does,
    # and pulling the connections out from under it would throw away the one reply
    # that says what the rover did.
    driving = recording(drive_web.Session(None, 3.0, 480))
    driving.channels = ["a channel"]
    driving.answered_at = 100.0
    driving.busy_since = 100.0
    driving.mind_the_link(200.0)
    check("a move in flight is not interrupted to reconnect", driving.tried, [])

    # A join takes the rover off this network on purpose, and `rejoined` is already
    # scheduled to pick the pieces up.
    joining = recording(drive_web.Session(None, 3.0, 480))
    joining.channels = ["a channel"]
    joining.answered_at = 100.0
    joining.wifi_joining = "upstairs"
    joining.mind_the_link(200.0)
    check("a network join is left to finish", joining.tried, [])

    # --- a link that is down -------------------------------------------------
    down = recording(drive_web.Session(None, 3.0, 480))
    down.find_at = 100.0
    down.find_tries = 1
    down.mind_the_link(100.0 + drive_web.RECONNECT_S / 2)
    check("a search is not repeated the instant it fails", down.tried, [])
    down.mind_the_link(100.0 + drive_web.RECONNECT_S + 0.1)
    check("...and is repeated once the wait is up", down.tried, ["connect"])

    # One at a time. The search runs on a thread and takes seconds on a name that
    # does not resolve; a retry per tick would be ten threads a second.
    flying = recording(drive_web.Session(None, 3.0, 480))
    flying.find_at = 100.0
    flying.find_outstanding = True
    flying.mind_the_link(500.0)
    check("a search already running is not started again", flying.tried, [])

    # Backing off, and stopping backing off. A rover switched off for the evening
    # should not be dialled every two seconds all night, and one switched off for a
    # moment should not take a minute to be noticed.
    waits = [recording(drive_web.Session(None, 3.0, 480)) for _ in range(3)]
    for tries, session in zip((0, 1, 50), waits):
        session.find_tries = tries
    check("the first wait is the short one",
          waits[0].retry_in(), drive_web.RECONNECT_S)
    check("...and so is the wait after one failure",
          waits[1].retry_in(), drive_web.RECONNECT_S)
    check("...and it never grows past the ceiling",
          waits[2].retry_in(), drive_web.RECONNECT_MAX_S)

    # --- what the notice and the link line say -------------------------------
    talking = drive_web.Session("bpi-m4zero.local:8769", 3.0, 480)
    talking.connected = lambda address: None          # no sockets in a selftest
    for _ in range(3):
        talking.handle(drive_web.Reply(
            "__found__", {}, {"ok": False, "address": None}, 0.0))
    check("a rover that is not there is reported once, not once a try",
          talking.notice_seq, 1)
    check("...and what it says is that it is still looking",
          "keep looking" in talking.notice["text"], True)
    check("...as does the link", "looking again" in talking.link_text, True)
    check("...and the tries were counted", talking.find_tries, 3)

    talking.handle(drive_web.Reply(
        "__found__", {}, {"ok": True, "address": "bpi-m4zero.local:8769"}, 0.0))
    check("coming back is worth saying",
          "answered again" in talking.notice["text"], True)
    check("...and the count starts over", talking.find_tries, 0)


def test_a_browser_leaving() -> None:
    """A closed tab is not an error, and everything else still is.

    `socketserver` prints a full traceback for anything that reaches it out of a
    handler, and a browser closing a kept-alive connection reaches it as one --
    `ConnectionAbortedError [WinError 10053]` from the read of the next request
    line. Every reload printed twenty lines about it. That is worth a test rather
    than a comment because the fix is a suppression, and a suppression that grows
    to cover a real fault is how a console stops reporting the thing it is for.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"a browser leaving ({type(exc).__name__})")
        return

    def printed(error):
        """What the server would write to stderr while `error` is being handled."""
        caught = io.StringIO()
        was, sys.stderr = sys.stderr, caught
        try:
            try:
                raise error
            except type(error):
                # An instance without its __init__, so no socket is bound to ask
                # the question of -- the whole decision is which exception is in
                # flight, and `super()` inside it needs a real instance to reach
                # the printing it falls back to.
                server = drive_web.Console.__new__(drive_web.Console)
                server.handle_error(None, ("127.0.0.1", 1))
        except Exception as exc:        # the real handler's own failure, if any
            caught.write(f"handle_error raised {type(exc).__name__}")
        finally:
            sys.stderr = was
        return caught.getvalue()

    for error in (ConnectionAbortedError(10053, "aborted"),
                  ConnectionResetError(10054, "reset"),
                  BrokenPipeError(32, "broken pipe"),
                  TimeoutError("the handler's idle timeout")):
        check(f"{type(error).__name__} is a tab closing, not an error",
              printed(error), "")

    # And the other half, which is the half that matters: a genuine fault in a
    # handler still lands in the window somebody is watching.
    shouted = printed(ValueError("the map arrived as a duck"))
    check("a real fault is still printed", "ValueError" in shouted, True)
    check("...with the traceback that says where it came from",
          "Traceback" in shouted, True)


def test_one_console_at_a_time() -> None:
    """A second drive console must not start, on any port.

    Two consoles are not two windows onto one rover, they are two clients of it:
    each polls three times a second and each asks for a map that, on the Pi 1, cost
    the single core two and a half seconds to draw. Measured with three attached, the
    daemon sat at 48% of the core drawing maps for windows nobody was looking at.
    Worse on Windows, where `SO_REUSEADDR` means *share* rather than *reclaim*, so
    the second one binds the same port happily and the browser is served its page by
    one console while its buttons post to the other -- which reads as a rover that
    has stopped listening and a map from some earlier session.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"one console at a time ({type(exc).__name__})")
        return

    import tempfile

    path = os.path.join(tempfile.mkdtemp(prefix="rover-lock-"), "console.lock")
    first, second = drive_web.OnlyOne(path), drive_web.OnlyOne(path)
    try:
        check("the first console gets the lock", first.claim(), "")
        refused = second.claim()
        check("...and the second is refused", bool(refused), True)
        check("...and told which process to close",
              str(os.getpid()) in refused, True)

        # A lock the kernel holds, not a file somebody has to remember to delete:
        # the console that matters here is the one that died without tidying up, and
        # a stale lock nobody can clear is a console nobody can run.
        first.release()
        check("once the first goes, the next one starts", second.claim(), "")
    finally:
        first.release()
        second.release()

    # The port guard is the other half, and it is a property of the class rather
    # than of a running server: on Windows the default would let two consoles share
    # one port without either of them finding out.
    check("Windows refuses to share the port; Linux reclaims TIME_WAIT",
          drive_web.Console.allow_reuse_address, os.name != "nt")


def test_a_second_click_takes_over() -> None:
    """Clicking somewhere else while the rover is driving sends it there instead.

    It used to be refused. The console saw a move in flight and answered "drive_to
    is still running; stop it or wait", which is a console arguing with the only
    instruction it has -- somebody clicking a second time on the map is saying the
    rover is going to the wrong place, and the answer to that is not to make them
    press STOP first and then click again.

    Two things make it work and both are tested here. The click is sent as a point
    on the map rather than as an offset from the rover, so it keeps its meaning
    across the second it takes to stop what is running; an offset would be measured
    from wherever the rover had got to by then. And the new move is held until the
    old one answers, because the running call occupies the move connection and the
    daemon would refuse a second one as busy -- the stop that frees it goes out on
    the connection that carries nothing else.
    """
    try:
        import drive_web
        from console_model import Reply, asked_for, tap_to_point
    except ImportError as exc:
        SKIP.append(f"a second click takes over ({type(exc).__name__})")
        return

    class Fake:
        """A channel that records rather than connects."""

        def __init__(self):
            self.sent = []

        def submit(self, name, arguments=None):
            self.sent.append((name, arguments or {}))

    # The map as the rover last drew it: 3 m each way at 4 px per cell, with the
    # rover two metres out along x and facing 90 degrees left of where it started.
    # The rover's own pixel is then (240, 240), and 20 cells is a metre.
    view = {"half_extent_m": 3.0, "scale": 4, "rover_up": False,
            "pose": {"x_m": 2.0, "y_m": -1.0, "heading_deg": 90.0}}

    def console():
        """A session wired to fakes, with a map on screen and a rover that drives."""
        session = drive_web.Session(None, 3.0, 480)
        session.moves, session.halt = Fake(), Fake()
        session.tools = ["drive_to", "drive", "stop_driving"]
        session.can_drive = True
        session.map_view = dict(view)
        return session

    if tap_to_point(240, 240, view) is None:
        SKIP.append("a second click takes over (no mapimg beside us)")
        return

    # A click on an idle rover goes straight out, and it goes as a place. The pixel
    # is 20 cells up the page from the rover, which with the page held to the start
    # heading is a metre further along +x -- the rover's position plus a metre, not
    # "a metre ahead of it", which for a rover facing +y would be somewhere else.
    session = console()
    session.act({"do": "tap", "col": 240, "row": 160})
    check("a click on an idle rover is sent at once",
          [name for name, _ in session.moves.sent], ["drive_to"])
    first = session.moves.sent[0][1]
    check("...as a place on the map rather than an offset",
          sorted(first), ["x_m", "y_m"])
    check("...and it is the place under the cursor",
          (first["x_m"], first["y_m"]), (3.0, -1.0))

    # Now the same again while that move is still running. The move connection is
    # occupied, so nothing may go out on it yet -- but the stop must.
    session.act({"do": "tap", "col": 160, "row": 240})
    check("a click during a move stops what is running",
          [name for name, _ in session.halt.sent], ["stop_driving"])
    check("...and does not try to overtake it on the move connection",
          len(session.moves.sent), 1)
    check("...and says so, naming where it will go instead",
          "new target" in session.notice["text"], True)

    # The rover answers the move it was told to stop. That is the wheels coming
    # free, and the click that was waiting goes then.
    session.handle(Reply("drive_to", first, {"ok": True, "reason": "stopped",
                                             "travelled_m": 0.4}, 1.2))
    check("the waiting click goes once the old move has answered",
          [name for name, _ in session.moves.sent], ["drive_to", "drive_to"])
    second = session.moves.sent[1][1]
    check("...to the second place, not the first",
          (second["x_m"], second["y_m"]), (2.0, 0.0))
    check("...and nothing is left waiting", session.pending_target, None)
    check("...and the console counts itself busy again",
          session.busy_name, "drive_to")

    # A third place clicked before the handover replaces the waiting one, and sends
    # no second stop: the first one is already on its way.
    session = console()
    session.busy_since, session.busy_name = 100.0, "drive_to"
    session.act({"do": "tap", "col": 240, "row": 160})
    session.act({"do": "tap", "col": 160, "row": 240})
    check("a further click replaces the waiting one rather than queueing behind it",
          (session.pending_target or {}).get("y_m"), 0.0)
    check("...without a second stop", len(session.halt.sent), 1)

    # STOP after a click must not be followed by the rover setting off for it. This
    # is the one that would be unforgivable.
    session.act({"do": "stop"})
    check("pressing STOP throws the waiting click away", session.pending_target,
          None)
    session.handle(Reply("drive_to", {}, {"ok": True, "reason": "stopped"}, 0.3))
    check("...so the move that answers is the end of it, not the start of another",
          session.moves.sent, [])

    # The same goes for the stop that follows the last browser leaving: the target
    # was queued by a tab that has since been closed.
    session = console()
    session.busy_since, session.busy_name = 100.0, "drive"
    session.act({"do": "tap", "col": 240, "row": 160})
    session.listeners = 0
    session.mind_the_watchers(200.0)
    session.mind_the_watchers(200.0 + drive_web.ORPHAN_GRACE_S + 0.1)
    check("a closed tab takes its waiting click with it", session.pending_target,
          None)

    # A stop that never landed leaves the move channel silent for its own four
    # minutes, and a click acted on that late is an intention that has expired.
    session = console()
    session.busy_since, session.busy_name = 100.0, "drive_to"
    session.act({"do": "tap", "col": 240, "row": 160})
    held = session.pending_until
    session.mind_the_target(held - 0.1)
    check("a waiting click is held while there is still hope",
          session.pending_target is not None, True)
    session.mind_the_target(held + 0.1)
    check("...and dropped once there is not", session.pending_target, None)
    check("...out loud, because a click that evaporates looks like a bug",
          "dropped" in session.notice["text"], True)

    # A reconnect throws the move connection away, so the reply that would have
    # handed the wheels over is never coming.
    session = console()
    session.busy_since, session.busy_name = 100.0, "drive_to"
    session.act({"do": "tap", "col": 240, "row": 160})
    session.wanted_address = ""
    session.connect()
    check("a remade link does not leave a click waiting for a dead connection",
          session.pending_target, None)

    # The transcript has to say where a move is going in the units it was asked
    # for. "ahead +0.00 m" for a place the rover has already driven past would be a
    # sentence about the wrong thing entirely.
    check("a route to a place on the map reads as one",
          asked_for({"kind": "drive_to", "asked": {"x_m": 3.0, "y_m": -1.0}}),
          "the point x +3.00, y -1.00 on the map")
    check("...and an offset still reads as an offset",
          asked_for({"kind": "drive_to", "asked": {"ahead_m": 1.0, "left_m": -0.4}}),
          "ahead +1.00 m, left -0.40 m")


def test_a_click_while_exploring_takes_over() -> None:
    """Clicking the map while the rover is off exploring sends it there instead.

    The same instruction as clicking during a move -- go to this place, stop
    whatever you were doing -- and until now the only one the console got wrong.
    Exploring holds the wheels through the same mutex a move does, so the click
    went out as a `drive_to`, came back refused as busy, and the rover carried on
    mapping the house; the way to do it by hand was STOP, wait, then find the same
    pixel again.

    What makes it a separate path is that nothing waits on an exploring run. It is
    started by a call that answers in a moment and then runs for ten minutes, so
    there is no reply for the handover to hang off the way a move's outcome is.
    The console watches the rover's own `exploring` instead, and sends the waiting
    click when it goes false.
    """
    try:
        import drive_web
        from console_model import tap_to_point
        from drive_actions import EXPLORE_HANDOVER_S, TARGET_HANDOVER_S
    except ImportError as exc:
        SKIP.append(f"a click while exploring takes over ({type(exc).__name__})")
        return

    class Fake:
        def __init__(self):
            self.sent = []

        def submit(self, name, arguments=None):
            self.sent.append((name, arguments or {}))

    # The same map as the test above: 3 m each way at 4 px per cell, the rover two
    # metres out along x, so the pixel 80 rows up the page is a metre further on.
    view = {"half_extent_m": 3.0, "scale": 4, "rover_up": False,
            "pose": {"x_m": 2.0, "y_m": -1.0, "heading_deg": 90.0}}

    def console(exploring: bool = True):
        session = drive_web.Session(None, 3.0, 480)
        session.moves, session.halt = Fake(), Fake()
        session.tools = ["drive_to", "drive", "explore", "stop_driving"]
        session.can_drive = True
        session.map_view = dict(view)
        # Read off the rover in the ordinary way, not set by any button here.
        session.exploring = exploring
        return session

    if tap_to_point(240, 240, view) is None:
        SKIP.append("a click while exploring takes over (no mapimg beside us)")
        return

    exploring = {"ok": True, "exploring": True, "lidar_live": True}
    finished = {"ok": True, "exploring": False, "lidar_live": True}

    session = console()
    session.act({"do": "tap", "col": 240, "row": 160})
    check("a click while exploring stops the run",
          [name for name, _ in session.halt.sent], ["stop_driving"])
    check("...rather than sending a drive the rover would refuse as busy",
          session.moves.sent, [])
    check("...and says which of the two it stopped",
          "exploring run is being stopped" in session.notice["text"], True)
    check("...and waits longer than it would for a move, because a run between "
          "goals is pricing frontiers and cannot answer until it has",
          EXPLORE_HANDOVER_S > TARGET_HANDOVER_S, True)

    # The run has not stopped yet. Nothing may go out on the move connection while
    # it holds the wheels, however many status polls arrive.
    session.show_status(exploring)
    session.show_status(exploring)
    check("the click waits while the rover is still exploring",
          (session.moves.sent, session.pending_target is not None), ([], True))

    # And now it has. That is the only announcement there is that the wheels are
    # free, and the click goes on it.
    session.show_status(finished)
    check("the waiting click goes as soon as the run says it has stopped",
          [name for name, _ in session.moves.sent], ["drive_to"])
    where = session.moves.sent[0][1]
    check("...to the place under the cursor, in map coordinates",
          (where["x_m"], where["y_m"]), (3.0, -1.0))
    check("...and nothing is left waiting", session.pending_target, None)
    check("...and the console counts itself busy with it",
          session.busy_name, "drive_to")

    # An exploring run that ends by itself is the ordinary case, and there is
    # nothing waiting to send when it does.
    session = console()
    session.show_status(finished)
    check("a run that ends with nothing clicked sends nothing",
          session.moves.sent, [])

    # A stop that never landed. The click is dropped rather than acted on minutes
    # later, and the notice says which thing failed to let go.
    session = console()
    session.act({"do": "tap", "col": 240, "row": 160})
    held = session.pending_until
    session.mind_the_target(held - 0.1)
    check("a click behind an exploring run is held while there is still hope",
          session.pending_target is not None, True)
    session.mind_the_target(held + 0.1)
    check("...and dropped once there is not", session.pending_target, None)
    check("...saying it was the exploring run that would not stop",
          "exploring run did not stop" in session.notice["text"], True)


def test_a_slow_browser_is_shown_the_newest_state() -> None:
    """A browser on a link that cannot keep up gets fewer states, all of them now.

    Reproduced on the rover on 2026-08-26. Its radio had begun losing every packet
    over about 1100 bytes -- nothing lost at 500, a third at 1000, all of them at
    1200 -- which left the link carrying some 20 kB/s while this console writes a
    full state ten times a second, about 55 kB/s. The difference went into the
    kernel's send buffer, which Linux grows into the hundreds of kilobytes, and TCP
    then owes the browser every stale update in order before it may deliver the
    current one. Measured on the running rover: 580 kB stood queued for one browser,
    the page was drawing map generation 2473 while the console served 2502, and photo
    3229 against 3286 -- twenty-nine maps and fifty-seven photos, about a minute of
    rover either way. The page's own age readout said the map had been drawn 0.8 s
    ago, because that number is taken here as we publish and cannot see the minute
    that follows.

    What is pinned here is the decision, because the effect needs a kernel that grows
    a send buffer the way Linux does and cannot be seen on a desk that does not: with
    no room on the socket, a state is dropped rather than queued, `seen_version` stays
    where it was, and whatever is current when there is room next goes out in its
    place. The size of what that leaves is measured on the rover, where the fault was.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"a slow browser is shown the newest state ({type(exc).__name__})")
        return

    import json
    import threading

    class Jammed:
        """One browser, its link full until it is not, and what reached it."""

        def __init__(self) -> None:
            self.room = False
            self.heard: list[bytes] = []

        def write(self, data: bytes) -> None:
            self.heard.append(data)

        def flush(self) -> None:
            pass

        def setsockopt(self, *_args) -> None:
            pass

        def settimeout(self, *_args) -> None:
            pass

        def states(self) -> list[int]:
            """The state numbers this browser was actually sent, in order."""
            whole = b"".join(self.heard)
            return [int(n) for n in re.findall(rb'"n":(\d+)', whole)]

    def publish(session, n: int) -> None:
        with session.lock:
            session.published = json.dumps({"n": n}, separators=(",", ":"))
            session.version += 1
            session.lock.notify_all()

    session = drive_web.Session(None, 3.0, 480)
    page = Jammed()
    stream = drive_web.Handler.__new__(drive_web.Handler)
    stream.session = session
    stream.connection = page
    stream.wfile = page
    stream.send_response = lambda *_a, **_k: None
    stream.send_header = lambda *_a, **_k: None
    stream.end_headers = lambda: None
    stream._room_for_one_more = lambda: page.room

    watching = threading.Thread(target=stream._events, daemon=True)
    watching.start()
    try:
        publish(session, 1)
        time.sleep(0.3)
        check("a state is not written to a browser whose link is full",
              page.states(), [])

        for n in range(2, 51):
            publish(session, n)
        time.sleep(0.3)
        check("...nor are the forty-nine that came along behind it",
              page.states(), [])

        page.room = True
        for _ in range(40):
            if page.states():
                break
            time.sleep(0.05)
        check("when the link clears, what goes out is the newest state",
              page.states()[:1], [50])
        check("...and the fifty it was holding are never sent",
              max(page.states()), 50)

        # And the other half of the same rule. A picture is asked for by the name it
        # had in some state, and that state may be a minute old by the time the
        # browser acts on it -- so the name says which run drew it and nothing more,
        # and what comes back is the picture the rover has now.
        session.map_png = b"\x89PNG the newest one"
        shot = drive_web.Handler.__new__(drive_web.Handler)
        shot.session = session
        shot.wfile = Jammed()
        shot.send_response = lambda *_a, **_k: None
        shot.send_header = lambda *_a, **_k: None
        shot.end_headers = lambda: None
        shot.path = "/map.png?gen=deadbeef-1"
        shot.do_GET()
        check("a picture asked for by an old name still comes back current",
              b"".join(shot.wfile.heard), b"\x89PNG the newest one")
    finally:
        session.running = False
        with session.lock:
            session.lock.notify_all()
        page.room = True
        watching.join(timeout=2.0)


TESTS = (
    test_web_console,
    test_stopping_an_unwatched_rover,
    test_idle_console_waits_for_a_browser,
    test_finding_the_rover_again,
    test_a_browser_leaving,
    test_one_console_at_a_time,
    test_a_second_click_takes_over,
    test_a_click_while_exploring_takes_over,
    test_a_slow_browser_is_shown_the_newest_state,
)
