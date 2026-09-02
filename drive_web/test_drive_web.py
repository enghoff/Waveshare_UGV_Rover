"""Checks for the browser console, with no browser and no rover."""
from __future__ import annotations

import io
import os
import re
import sys
import tempfile
import time

import _paths  # noqa: F401 — drive_web, console_model, mock_rover, test_harness
from test_harness import FAIL, PASS, SKIP, check


def test_choosing_a_network() -> None:
    """Which access point the rover is on, and moving it to another.

    Over a real socket rather than against the class, because the shape of these
    two answers is the whole contract between the daemon and the console's network
    panel, and the mock is the only place that shape can be checked without a Pi
    and three routers.

    The unscanned case is the one worth pinning down. Nothing polls for a scan --
    it takes the radio off channel for seconds, and the caller is very likely
    asking through it -- so what a console normally sees is a list of one, and a panel that treated
    that as "there is nothing else out there" would be wrong in the ordinary case
    rather than the rare one.
    """
    try:
        import mock_rover
        import rover_tools
    except ImportError as exc:
        SKIP.append(f"choosing a network ({type(exc).__name__})")
        return

    rover = mock_rover.Rover(None, None)
    server = mock_rover.serve(rover, "127.0.0.1", 0, quiet=True)
    client = rover_tools.RoverClient(f"127.0.0.1:{server.server_address[1]}")
    try:
        quiet = client.call("wifi_status", {})
        check("it says which network it is on",
              quiet.get("connected"), "TheGreatLord")
        check("...with a signal from the driver, in dBm",
              -90 <= quiet.get("level_dbm", 0) <= -20, True)
        check("...and an address, since being associated is not being online",
              quiet.get("address"), "192.168.1.47")
        check("without a scan the list is only what was last heard",
              [n["ssid"] for n in quiet["networks"]], ["TheGreatLord"])
        check("...and that row is marked as the one in use",
              quiet["networks"][0]["in_use"], True)

        looked = client.call("wifi_status", {"scan": True})
        check("a scan finds the neighbours", len(looked["networks"]) > 1, True)
        check("...and says which of them this rover has a passphrase for",
              [n["ssid"] for n in looked["networks"] if n["configured"]],
              ["TheGreatLord", "TheMaharaja", "TheGreatViking"])

        refused = client.call("wifi_join", {"ssid": "Alister"})
        check("a network with no passphrase is refused", refused.get("ok"), False)
        check("...by name, so the panel can say why",
              "no passphrase for Alister" in refused.get("error", ""), True)
        check("and so is a join with no network at all",
              client.call("wifi_join", {}).get("ok"), False)

        moved = client.call("wifi_join", {"ssid": "TheMaharaja"})
        check("a configured network is accepted", moved.get("joining"), "TheMaharaja")
        check("...and the answer warns that the link is about to go",
              "drop" in moved.get("note", ""), True)
        after = client.call("wifi_status", {})
        check("...and afterwards it is on it", after.get("connected"), "TheMaharaja")
        check("...with the outcome kept for whoever reconnects",
              after.get("last_join", {}).get("ok"), True)
    finally:
        client.close()
        server.shutdown()
        server.server_close()


def test_signal_verdict() -> None:
    """One word for a dBm reading, which is what gets the colour in the panel."""
    try:
        import console_model
    except ImportError as exc:
        SKIP.append(f"signal verdict ({type(exc).__name__})")
        return

    verdict = console_model.wifi_verdict
    check("a strong link", verdict(-41), "good")
    check("a fading one", verdict(-68), "fair")
    check("one the wifi keeper is about to act on", verdict(-77), "poor")
    # No reading at all is the interface not reporting a signal, which is not good
    # news and must not be coloured as though it were.
    check("and no reading at all", verdict(None), "poor")


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


def test_the_network_panel() -> None:
    """The network panel, and what pressing `join` costs on a rover with one radio.

    It costs the page. The rover takes its link down and brings another up, so
    every connection this console holds dies with it -- which is why the panel
    warns before it happens, why a reconnect is scheduled rather than waited
    for, and why nothing here ever chooses a network on its own.
    """
    import drive_web

    session = drive_web.Session(None, 3.0, 480)
    session.show_wifi({"ok": True, "connected": "TheGreatViking",
                       "level_dbm": -38, "address": "192.168.1.80",
                       "list_age_s": 0.0,
                       "networks": [
                           {"ssid": "TheGreatViking", "signal": 84,
                            "in_use": True, "configured": True},
                           {"ssid": "TheGreatLord", "signal": 50,
                            "in_use": False, "configured": True},
                           {"ssid": "Alister", "signal": 40,
                            "in_use": False, "configured": False}]})
    check("the reading is the network the rover is on",
          session.wifi["text"].startswith("TheGreatViking"), True)
    check("...and the address to reach it at",
          "192.168.1.80" in session.wifi["where"], True)
    # The other two house networks are the ones a person may choose; the
    # neighbour's is shown so the list is honest and is not offered.
    offered = [n["ssid"] for n in session.wifi_networks if n["joinable"]]
    check("the house networks it is not on are the ones offered",
          offered, ["TheGreatLord"])
    check("...and a network with no passphrase says why it is not",
          [n["note"] for n in session.wifi_networks if n["ssid"] == "Alister"],
          ["no passphrase"])

    session.watch = object()
    calls: list[tuple] = []
    session.watch_call = lambda name, args: calls.append((name, args))
    session.wifi_join("TheGreatLord")
    check("a join is sent", calls[0][0], "wifi_join")
    check("...and says the rover is about to go away",
          "unreachable" in session.wifi["note"], True)
    check("...so the page schedules its own reconnect",
          session.rejoin_at > 0.0, True)

    # ...and it stops claiming to be joining once the rover answers from the
    # network it was sent to, rather than saying so for the rest of the session.
    session.show_wifi({"ok": True, "connected": "TheGreatLord",
                       "level_dbm": -55, "address": "192.168.1.80",
                       "list_age_s": 0.0, "networks": []})
    check("the join is seen to have landed", session.wifi["joining"], None)
    check("...and said so", "on TheGreatLord now" in session.wifi["note"], True)


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


def test_the_audio_socket() -> None:
    """The framing under the microphone, with no browser and no model.

    This is a protocol implemented by hand -- see [wsframe.py](wsframe.py) for
    why it is not a library -- and the two rules worth pinning are the ones whose
    failure is silent. A length that crosses one of the header's size boundaries
    comes out as a frame that reads fine and is one byte long; masking applied on
    the wrong side is a connection the browser closes without saying anything.
    Neither shows up as an exception in the place that caused it.
    """
    import wsframe

    # RFC 6455's own example, so this is checked against the standard rather than
    # against itself.
    check("the handshake matches the standard's worked example",
          wsframe.accept("dGhlIHNhbXBsZSBub25jZQ=="),
          "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=")

    for size in (0, 1, 125, 126, 127, 65535, 65536, 70000):
        raw = bytes(range(256)) * (size // 256) + bytes(size % 256)
        wire = wsframe.frame(wsframe.BINARY, raw, mask=True)
        opcode, back = wsframe.read_message(io.BytesIO(wire))
        check(f"a {size} byte frame survives the wire", (opcode, back),
              (wsframe.BINARY, raw))

    # The server's own frames are unmasked, and a client reading them says so.
    wire = wsframe.frame(wsframe.TEXT, b"hello")
    check("a server frame reads back on the client side",
          wsframe.read_message(io.BytesIO(wire), from_client=False),
          (wsframe.TEXT, b"hello"))

    for name, wire, from_client in (
            ("an unmasked frame from a client is refused",
             wsframe.frame(wsframe.TEXT, b"x"), True),
            ("a masked frame from a server is refused",
             wsframe.frame(wsframe.TEXT, b"x", mask=True), False)):
        try:
            wsframe.read_message(io.BytesIO(wire), from_client=from_client)
            check(name, "accepted", "refused")
        except wsframe.ProtocolError:
            check(name, "refused", "refused")

    # Fragments, because a browser may send them even though this never does.
    first = wsframe.frame(wsframe.BINARY, b"one", mask=True)
    first = bytes([first[0] & 0x7F]) + first[1:]        # clear FIN
    rest = wsframe.frame(wsframe.CONT, b"-two", mask=True)
    check("a fragmented message is put back together",
          wsframe.read_message(io.BytesIO(first + rest)),
          (wsframe.BINARY, b"one-two"))

    # A close reason longer than a control frame may carry. The rover has been on
    # the receiving end of this one: Alibaba's service once refused a session with
    # a reason too long to be legal, and every conformant client discarded it, so
    # the actual message was invisible.
    payload = wsframe.close_frame(1000, "x" * 400)
    check("a close reason is cut to what a control frame may hold",
          len(payload) <= 125, True)


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


def test_what_the_browser_heard() -> None:
    """The playback accounting an interruption depends on.

    When somebody talks over the rover, the model believes it said the whole
    reply and the only correction available is the number of milliseconds that
    were actually audible. On a desk that number comes off a sound card; here it
    comes back over the wifi from a browser, which means it can be late, stale, or
    about a reply that has already been replaced -- and each of those has a wrong
    answer that sounds like a fault rather than looking like a bug.
    """
    try:
        import numpy as np

        from omni_bridge import BrowserSpeaker
    except Exception as error:                         # noqa: BLE001
        SKIP.append(f"the browser speaker ({type(error).__name__}: {error})")
        return

    sent: list[bytes] = []
    control: list[dict] = []
    speaker = BrowserSpeaker(sent.append, control.append)

    speaker.begin()
    check("a reply starts by telling the page so", control[0]["t"], "begin")
    # A second of audio at the rate the service speaks.
    speaker.write(np.zeros(24000, dtype=np.float32))
    check("what was sent is a second of PCM16", len(sent[0]), 48000)

    speaker.note_played(control[0]["gen"], 400)
    check("what the page says it played is what is reported",
          speaker.played_ms(), 400)
    speaker.note_played(control[0]["gen"], 5000)
    check("...but never more than was actually sent", speaker.played_ms(), 1000)

    # A report about the previous reply, arriving after the next one began.
    speaker.begin()
    speaker.write(np.zeros(2400, dtype=np.float32))
    speaker.note_played(control[0]["gen"], 900)
    check("a report about a finished reply is ignored", speaker.played_ms(), 0)

    speaker.note_played(control[-1]["gen"], 40)
    dropped = speaker.flush()
    check("an interruption tells the page to drop what is queued",
          control[-1]["t"], "flush")
    check("...and says how much of the reply went unheard",
          round(dropped, 3), 0.06)
    check("...after which what was queued is what was heard",
          speaker.played_ms(), 40)



def test_a_second_conversation_starts_at_once() -> None:
    """Pressing start again straight after a refresh must reach the model.

    Reproduced on the rover on 2026-08-27. Refreshing the console ends the
    conversation on purpose -- the page says so on `pagehide`, so a tab closed at
    bedtime cannot quietly spend the account's free quota -- and pressing start
    again a few seconds later failed with `[Errno 98] Address already in use`.
    The port that was in use is not the model's. It is the little loopback
    receiver the daemon posts `look`'s pictures to, which a conversation built
    before it dialled anything, so the failure landed before a single word
    reached the model and read as a rover that had stopped answering.

    What holds the port is the daemon: it keeps one connection to that receiver
    and is never told the conversation has ended, so an open connection on the
    port outlives the receiver that accepted it and the next `bind` is refused.
    A rebind is refused whether or not `SO_REUSEADDR` is set -- that forgives a
    port left in TIME_WAIT and this one is not -- so the fix is to stop
    rebinding, and the receiver now lives as long as the console does.
    """
    try:
        import http.client
        import json
        import socket

        import omni_bridge
        import talk_frames
    except Exception as error:                         # noqa: BLE001
        SKIP.append(f"a second conversation ({type(error).__name__}: {error})")
        return

    # A JPEG only in the sense the receiver checks for.
    jpeg = b"\xff\xd8\xff\xe0\x00\x10JFIF" + b"\x00" * 32 + b"\xff\xd9"

    def free_port() -> int:
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        return port

    def post(connection) -> dict:
        """One picture, over a connection the caller keeps -- as the daemon does."""
        connection.request("POST", "/frame", body=jpeg,
                           headers={"Content-Type": "image/jpeg",
                                    "Content-Length": str(len(jpeg))})
        return json.loads(connection.getresponse().read())

    # 1. The trap, walked into exactly as a conversation used to walk into it.
    port = free_port()
    receiver = talk_frames.Frames(port=port, host="127.0.0.1")
    receiver.serve_in_background()
    daemon = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    try:
        check("the daemon files a picture with the receiver",
              post(daemon).get("ok"), True)
        # The conversation ends. The daemon is not told and does not let go.
        receiver.shutdown()
        receiver.server_close()
        try:
            talk_frames.Frames(port=port, host="127.0.0.1").server_close()
            refused = ""
        except OSError as error:
            refused = str(error)
        if sys.platform.startswith("linux"):
            check("...and the next conversation cannot have the port back",
                  "in use" in refused, True)
        else:
            SKIP.append("the port a finished receiver leaves behind "
                        f"(this kernel hands it straight back: {refused or 'ok'})")
    finally:
        daemon.close()

    # 2. And the console no longer asks for it back, because it never let go.
    port = free_port()
    said: list[str] = []
    omni = omni_bridge.Omni("127.0.0.1:1", lambda text, err=False: said.append(text),
                            frame_port=port)
    daemon = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    try:
        first = omni._frame_server()
        picture = post(daemon).get("image")
        check("a conversation has somewhere for the pictures to go",
              bool(picture), True)

        # The conversation ends -- a refresh, the button, or the idle watch --
        # and the next one starts while the daemon's connection is still open.
        second = omni._frame_server()
        check("the next conversation is served by the same receiver",
              second is first, True)
        check("...so the daemon's kept-open connection is still good",
              post(daemon).get("ok"), True)
        check("...and a picture from the conversation before is gone",
              first.take(picture), None)
    finally:
        daemon.close()
        omni.close()

    check("closing the console is what finally gives the port up",
          omni._frames, None)


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


def test_the_world_state_popup() -> None:
    """The console's side of the semantic world, with no rover and no browser.

    What the page draws comes out of `Session` and out of two URLs, so those are
    what is checked here: the counts and the tag that rides in every pushed state,
    the payload the popup fetches when that tag moves, and the two failures the
    viewer has to survive -- a rover with no world-state component at all, and an
    observation whose stored frame is not there any more.

    The drawing itself is JavaScript in a browser and is not covered by anything
    here; the repository has no browser in its test loop. What that costs is
    written down in world_state/README.md rather than left to be discovered.
    """
    try:
        import json

        import drive_web
        from console_model import Reply
    except ImportError as exc:
        SKIP.append(f"the world-state popup ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)

    # Nothing has been asked yet, so the button says nothing about a rover it has
    # not spoken to. `available: None` is that third state, and it matters: a
    # console that assumed either answer would either hide a working feature or
    # offer a broken one.
    world = session.world_state()
    check("the world starts unknown rather than absent", world["available"], None)
    check("...and shut", world["open"], False)
    check("...with no picture to fetch yet", world["gen"], "")

    # An empty world. The popup opens on a rover that has never inspected
    # anything, and that has to read as "press the button", not as an error.
    session.world_handle("world_state_entities", {
        "ok": True, "entities": [], "recent": [], "unmatched": [],
        "summary": {"entities": 0, "observations": 0, "inspections": 0,
                    "map_session": 1}}, 0.1)
    check("an empty world is available rather than broken",
          session.world_state()["available"], True)
    check("...and says so in the counts", session.world_state()["entities"], 0)
    check("...and the popup has something to fetch",
          bool(session.world_state()["gen"]), True)

    # A populated one.
    observation = {"id": 1, "entity_id": "furniture:1", "observed_at": 1.0,
                   "source": "cosmos_visual", "frame_id": "20260901-120000-abc123",
                   "label": "grey sofa", "description": "a grey three-seat sofa",
                   "location_hint": "ahead-left", "bbox": [0.1, 0.3, 0.5, 0.9],
                   "observer_pan_deg": 20.0, "observer_tilt_deg": -5.0,
                   "pose": {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0},
                   "map_session": 1, "model_id": "cosmos", "prompt_version": "2",
                   "raw": {"label": "grey sofa"}, "note": None}
    session.world_handle("world_state_entities", {
        "ok": True,
        "entities": [{"id": "furniture:1", "kind": "furniture", "label": "grey sofa",
                      "canonical_description": "a grey three-seat sofa",
                      "created_at": 1.0, "last_seen_at": 2.0,
                      "observation_count": 2, "last_map_session": 1,
                      "last_frame_id": "20260901-120000-abc123",
                      "distinct_labels": 1,
                      "rays": [{"x_m": 1.0, "y_m": 2.0, "bearing_deg": 70.0,
                                "span_deg": 26.0, "length_m": 2.5}]}],
        "recent": [observation], "unmatched": [],
        "summary": {"entities": 1, "observations": 2, "inspections": 1,
                    "map_session": 1}}, 0.2)
    check("a populated world reaches the counts",
          (session.world_state()["entities"], session.world_state()["observations"]),
          (1, 2))
    check("...and the payload carries the ray the map is drawn from",
          session.world_payload["entities"][0]["rays"][0]["bearing_deg"], 70.0)
    check("...and the whole observation stream, so duplicates are visible",
          len(session.world_payload["recent"]), 1)

    # The frame behind an observation. The console fetches the newest few and
    # serves them at their own URL; one it has not fetched is a name, not a broken
    # picture, and the page is told which by the list of frames it holds.
    check("a frame that has not arrived is not offered to the page",
          session.world_payload["frames"], [])
    session.world_handle("world_state_frame", {
        "ok": True, "frame_id": "20260901-120000-abc123",
        "jpeg_base64": "/9j/4AAQSkZJRg=="}, 0.1)
    check("one that has is", session.world_payload["frames"],
          ["20260901-120000-abc123"])
    check("...and is held where the URL can find it",
          "20260901-120000-abc123" in session.world_frames, True)

    # An inspection, and the one line the popup's header gets out of it.
    session.world_handle("world_inspect", {
        "ok": True, "created": 2, "matched": 1, "rejected": 0,
        "detail": ""}, 61.0)
    check("an inspection says what it did, in a line",
          session.world_state()["note"], "2 new, 1 recognised -- 61 s")
    check("...and is no longer running", session.world_state()["busy"], False)

    session.world_handle("world_inspect", {
        "ok": True, "created": 0, "matched": 0, "rejected": 0}, 58.0)
    check("...and an inspection that found nothing says that rather than nothing",
          session.world_state()["note"],
          "nothing worth recording in view -- 58 s")

    # A failure. The popup has to be able to say the model failed, because the
    # alternative reading of an unchanged world -- nothing was in view -- is fixed
    # somewhere else entirely.
    session.world_handle("world_inspect", {
        "ok": False, "error": "the Cosmos sidecar at http://127.0.0.1:8775 is not "
                              "answering"}, 3.0)
    check("a failed inspection is reported as one",
          "not answering" in session.world_state()["error"], True)
    check("...and the button comes back", session.world_state()["busy"], False)

    # Clearing. Armed first, and separate from the map's clear in both directions.
    session.world_link = _Recorder()
    session.world_clear()
    check("the first press arms rather than clears", session.world_link.calls, [])
    check("...and says what the second one will do",
          "map is not touched" in session.world_state()["note"], True)
    session.world_clear()
    check("the second press clears the semantic world",
          [name for name, _ in session.world_link.calls], ["world_state_clear"])
    check("...and nothing about it went near the navigator",
          [name for name, _ in session.world_link.calls
           if "map" in name or "nav" in name], [])

    # And the other direction: clearing the SLAM map starts a new map session and
    # deletes no semantic state.
    session.world_link.calls.clear()
    session.world_map_cleared()
    check("clearing the map starts a new session in the store",
          [name for name, _ in session.world_link.calls], ["world_map_session"])

    # A rover with no world-state component says so once. Anything else is a
    # popup that shows the same error every few seconds for the rest of the day.
    session.world_handle("world_state_summary", {
        "ok": False, "error": "this rover has no world_state component installed"},
        0.1)
    check("a rover without the component is remembered as such",
          session.world_state()["available"], False)
    check("...and the reason is kept for the popup to show",
          "no world_state component" in session.world_state()["error"], True)


class _Recorder:
    """A channel that writes down what it was asked for instead of asking."""

    def __init__(self) -> None:
        self.calls = []

    def submit(self, name, arguments=None) -> None:
        self.calls.append((name, arguments))


def test_finding_a_thing_from_the_console() -> None:
    """Typing a phrase, and the two answers it can get.

    The interesting case is the third one: the model takes several seconds, so an
    answer can arrive after the person has typed something else, and drawing it
    would read as the search having got the wrong thing. It is dropped instead.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"the console search ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    sent = []
    session.world_link = type("Link", (), {
        "submit": lambda _self, name, arguments=None: sent.append((name, arguments)),
    })()

    session.world_act({"what": "search", "query": "  a spray bottle  "})
    check("the phrase is sent, trimmed", sent[-1][0], "world_state_search")
    check("...as the rover's own argument", sent[-1][1]["query"], "a spray bottle")
    check("...and the box says it is working",
          session.world_state()["searching"], True)

    session.world_handle("world_state_search", {
        "ok": True, "query": "a spray bottle", "confident": True,
        "best": 0.13, "considered": 31, "skipped": 0,
        "detail": "the best match scores 0.130 against that description",
        "matches": [{"score": 0.13, "observation_id": 4, "entity_id": "object-1",
                     "label": "a spray bottle", "frame_id": "f1",
                     "placement": {"x_m": 2.0, "y_m": 1.0}}]}, 4.0)
    check("the answer stops the working state",
          session.world_state()["searching"], False)
    check("...and is kept for the popup to draw",
          session.world_payload["search"]["matches"][0]["label"], "a spray bottle")

    # An answer to the phrase before last, arriving after the box has moved on.
    session.world_act({"what": "search", "query": "a bicycle"})
    session.world_handle("world_state_search", {
        "ok": True, "query": "a spray bottle", "confident": True,
        "matches": [{"score": 0.9, "label": "the wrong answer"}]}, 4.0)
    check("a late answer to an older phrase is dropped",
          session.world_payload["search"]["matches"][0]["label"], "a spray bottle")

    # An empty box clears the pane rather than asking the rover about nothing.
    before = len(sent)
    session.world_act({"what": "search", "query": "   "})
    check("an empty phrase asks the rover nothing", len(sent), before)
    check("...and takes the last answer off the screen",
          "search" in session.world_payload, False)

    # A rover that cannot answer says so without leaving the box spinning.
    session.world_act({"what": "search", "query": "a mirror"})
    session.world_handle("world_state_search",
                         {"ok": False, "error": "no perception sidecar"}, 0.2)
    check("a refusal stops the working state",
          session.world_state()["searching"], False)
    check("...and is shown", session.world_state()["error"], "no perception sidecar")


def test_the_switch_for_building_the_world_state() -> None:
    """On by default on the rover, and this panel never guesses at it.

    What it shows comes back from the rover rather than from what this console
    last asked for, because the voice session, another console or a script can
    turn it off -- and a panel showing its own past would leave a rover that had
    quietly stopped recording still looking busy.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"the world-building switch ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    sent = []
    session.watch = type("Link", (), {
        "submit": lambda _self, name, arguments=None: sent.append((name, arguments)),
    })()

    check("nothing is claimed before the rover has been asked",
          session.world_state()["building"], None)

    session.world_act({"what": "build", "on": False})
    check("the switch is sent to the rover", sent[-1][0], "world_building")
    check("...as what was asked for", sent[-1][1], {"on": False})
    check("...and the panel still does not guess",
          session.world_state()["building"], None)

    session.world_handle("world_building",
                         {"ok": True, "building": False, "looks": 7}, 0.0)
    check("the panel shows what the rover said",
          session.world_state()["building"], False)
    check("...and how much it has recorded",
          session.world_state()["built_looks"], 7)

    session.world_handle("world_building",
                         {"ok": True, "building": True, "looks": 8}, 0.0)
    check("and again when it is turned back on",
          session.world_state()["building"], True)

    # The loop's own complaint is the only place a rover that has stopped
    # recording would ever say so.
    session.world_handle("world_building",
                         {"ok": True, "building": True, "looks": 8,
                          "error": "the perception sidecar is not running"}, 0.0)
    check("a loop that is failing says why", session.world_state()["error"],
          "the perception sidecar is not running")

    # A rover with no world state at all stops being asked, rather than showing
    # an error every ten seconds for the rest of the session.
    session.world_handle("world_building",
                         {"ok": False, "error": "no world_state component"}, 0.0)
    check("a rover without it is marked absent",
          session.world_state()["available"], False)
    check("...and the poll is not left outstanding",
          session.world_build_outstanding, False)


def test_the_page_draws_every_pane_its_tabs_offer() -> None:
    """A tab whose pane is never unhidden is a tab that does nothing.

    Cheap to get wrong when a tab is added, invisible until somebody clicks it,
    and there is no browser in this repository's test loop to catch it.
    """
    import os
    import re

    page = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "drive_web.html")
    with open(page, encoding="utf-8") as handle:
        html = handle.read()
    tabs = set(re.findall(r'data-wtab="([a-z]+)"', html))
    check("the popup offers four tabs", len(tabs), 4)
    for tab in sorted(tabs):
        pane = f'id="wPane{tab.capitalize()}"'
        check(f"the {tab} tab has a pane", pane in html, True)
        check(f"...that something unhides",
              f'$("wPane{tab.capitalize()}").hidden' in html, True)
    # Every element the script reaches for by name has to be in the markup, which
    # is the other half of the same mistake.
    for name in sorted(set(re.findall(r'\$\("(w[A-Za-z]+)"\)', html))):
        check(f"the page has an element called {name}",
              f'id="{name}"' in html, True)


def test_the_world_urls() -> None:
    """`/world.json` and `/world_frame.jpg`, over a real socket.

    Both exist for the same reason the network list and the map do: the payload is
    tens of kilobytes and the frames are whole JPEGs, and the state they would
    otherwise ride in goes out ten times a second.
    """
    try:
        import http.client
        import json
        import threading

        import drive_web
    except ImportError as exc:
        SKIP.append(f"the world URLs ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    session.world_payload = {"summary": {"entities": 1}, "frames": ["a-frame"]}
    session.world_frames["a-frame"] = b"\xff\xd8\xff\xe0not really a jpeg\xff\xd9"

    # The handler reaches its session through a class attribute rather than
    # through the server, which is how the console itself wires it up.
    was, drive_web.Handler.session = drive_web.Handler.session, session
    server = drive_web.Console(("127.0.0.1", 0), drive_web.Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    port = server.server_address[1]
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    try:
        connection.request("GET", "/world.json?gen=x")
        reply = connection.getresponse()
        body = json.loads(reply.read())
        check("the world payload is served", reply.status, 200)
        check("...as the popup's own copy", body["summary"]["entities"], 1)

        connection.request("GET", "/world_frame.jpg?id=a-frame")
        reply = connection.getresponse()
        frame = reply.read()
        check("a stored frame is served as a picture", reply.status, 200)
        check("...with the bytes the rover kept", frame.startswith(b"\xff\xd8"), True)

        # The row outlives the file. A viewer that fell over here would be hiding
        # exactly the observations it was opened to look at.
        connection.request("GET", "/world_frame.jpg?id=gone")
        reply = connection.getresponse()
        reply.read()
        check("a frame that is not held answers plainly rather than failing",
              reply.status, 404)

        connection.request("GET", "/world_frame.jpg")
        reply = connection.getresponse()
        reply.read()
        check("...and so does asking for no frame at all", reply.status, 404)
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        drive_web.Handler.session = was
