"""Checks for the browser console, with no browser and no rover."""
from __future__ import annotations

import io
import os
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
    it takes the dongle off channel for seconds, on a bus it shares with the camera
    -- so what a console normally sees is a list of one, and a panel that treated
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


def test_camera_rate() -> None:
    """How often the camera panel asks for a frame.

    The rate arrives from a page, so it is snapped to a rung rather than believed:
    a nought coming out of a hand-written POST would have the pump asking the rover
    for a picture on every tick of its own loop, on the core that is also running
    SLAM. There is deliberately no rung that means off -- the panel is a view of
    what the rover can see -- and the pump additionally waits for each frame to
    arrive before asking for the next, so the fastest rung is a floor on the
    interval and not a promise about it.
    """
    try:
        import drive_web
        from console_model import CAMERA_AUTO_S, CAMERA_RATES_S
    except ImportError as exc:
        SKIP.append(f"camera rate ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    check("the console starts on a rung the drop-down offers",
          session.frame_every_s in CAMERA_RATES_S, True)
    session.act({"do": "camera_rate", "seconds": 0.5})
    check("a rate from the drop-down is taken", session.frame_every_s, 0.5)
    session.act({"do": "camera_rate", "seconds": 0.0})
    check("...and one that is not on the ladder is snapped to the nearest",
          session.frame_every_s, min(CAMERA_RATES_S))
    session.act({"do": "camera_rate"})
    check("...and a rate with no number in it falls back to the default",
          session.frame_every_s, CAMERA_AUTO_S)
    check("nothing on the page can switch the camera off",
          0 in CAMERA_RATES_S, False)


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
    offered = [n["ssid"] for n in session.wifi["networks"] if n["joinable"]]
    check("only a network it has a passphrase for is offered", offered, ["Sonic5"])
    check("the one it is on is named as such",
          session.wifi["networks"][0]["note"], "on it")
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


def test_two_radios() -> None:
    """The panel when the rover has a spare radio, which changes what a join means.

    Everything the panel showed with one radio still has to be right -- the
    reading at the top is the link the traffic is going through, and the address
    is where to reach the rover -- while three things become true that were not:
    there are two rows above the network list, the address is one that survives a
    failover, and pressing `join` no longer takes the page down with it.
    """
    import drive_web

    session = drive_web.Session(None, 3.0, 480)
    dual = {
        "active": "wlan0", "standby": "wlan1", "switches": 2,
        "since_switch_s": 300.0, "service_ip": "192.168.1.80",
        "service_on": "wlan0", "surrendered": False, "note": "",
        "radios": [
            {"iface": "wlan0", "kind": "onboard", "role": "active",
             "ssid": "TheGreatLord 5G", "router": "TheGreatLord", "band": "5",
             "dbm": -38, "rtt_ms": 2.4, "loss_pct": 0.0, "usable": True,
             "address": "192.168.1.139", "asked": None,
             "seen": [{"ssid": "TheMaharaja 5G", "dbm": -70, "band": "5",
                       "router": "TheMaharaja"}]},
            {"iface": "wlan1", "kind": "usb", "role": "standby",
             "ssid": "TheMaharaja", "router": "TheMaharaja", "band": "2.4",
             "dbm": -69, "rtt_ms": 6.0, "loss_pct": 0.0, "usable": True,
             "address": "192.168.1.144", "asked": None,
             "seen": [{"ssid": "TheGreatViking", "dbm": -75, "band": "2.4",
                       "router": "TheGreatViking"}]},
        ],
    }
    session.show_wifi({"ok": True, "connected": "TheGreatLord 5G",
                       "level_dbm": -38, "address": "192.168.1.80",
                       "list_age_s": 0.0, "dual": dual,
                       "networks": [
                           {"ssid": "TheGreatLord 5G", "signal": 84,
                            "in_use": True, "configured": True},
                           {"ssid": "TheGreatViking", "signal": 50,
                            "in_use": False, "configured": True}]})
    check("both radios reach the panel", len(session.wifi["radios"]), 2)
    check("the one carrying traffic is named as such",
          session.wifi["radios"][0]["role"], "active")
    check("...and the spare as the spare", session.wifi["radios"][1]["role"],
          "standby")
    check("a radio's row says what it is on and how good it is",
          "2.4 GHz" in session.wifi["radios"][1]["detail"], True)
    check("the address shown is the one that survives a failover",
          "192.168.1.80" in session.wifi["service"], True)
    check("...and how many times it has moved",
          "2 failovers" in session.wifi["service"], True)
    check("choosing a network is offered as free", session.wifi["safe_join"], True)

    # A join with a spare radio must not schedule the reconnect that a join with
    # one radio has to: the page is not going to lose anything, and tearing six
    # working connections down to prove it would be the fault it exists to avoid.
    session.watch = object()
    calls: list[tuple] = []
    session.watch_call = lambda name, args: calls.append((name, args))
    session.wifi_join("TheGreatViking")
    check("a join is still sent", calls[0][0], "wifi_join")
    check("...but nothing is scheduled to reconnect", session.rejoin_at, 0.0)
    check("...and the panel says the page should not notice",
          "should not notice" in session.wifi["note"], True)

    # ...and it stops saying that once the rover is actually there, rather than
    # claiming to be joining for the rest of the session.
    dual["radios"][1]["role"] = "active"
    dual["radios"][1]["ssid"] = "TheGreatViking"
    dual["radios"][0]["role"] = "standby"
    dual["active"] = "wlan1"
    session.show_wifi({"ok": True, "connected": "TheGreatViking",
                       "level_dbm": -75, "address": "192.168.1.80",
                       "list_age_s": 0.0, "dual": dual, "networks": []})
    check("the join is seen to have landed", session.wifi["joining"], None)
    check("...and said so", "nothing dropped" in session.wifi["note"], True)

    # A rover with one radio must be unaffected by all of the above.
    old = drive_web.Session(None, 3.0, 480)
    old.show_wifi({"ok": True, "connected": "Sonic", "level_dbm": -42,
                   "address": "192.168.1.47", "list_age_s": 1.0,
                   "networks": [{"ssid": "Sonic", "signal": 80,
                                 "in_use": True, "configured": True}]})
    check("one radio draws no radio rows", old.wifi["radios"], [])
    check("...and keeps the old warning about what a join costs",
          old.wifi["safe_join"], False)


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
    quiet.map_at = 0.0
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

    # --- what the log and the link line say ----------------------------------
    talking = drive_web.Session("bpi-m4zero.local:8769", 3.0, 480)
    talking.connected = lambda address: None          # no sockets in a selftest
    for _ in range(3):
        talking.handle(drive_web.Reply(
            "__found__", {}, {"ok": False, "address": None}, 0.0))
    said = [line["text"] for line in talking.log]
    check("a rover that is not there is reported once, not once a try",
          sum("no rover daemon answered" in text for text in said), 1)
    check("...and the line says it is still looking",
          "keep looking" in " ".join(said), True)
    check("...as does the link", "looking again" in talking.link_text, True)
    check("...and the tries were counted", talking.find_tries, 3)

    talking.handle(drive_web.Reply(
        "__found__", {}, {"ok": True, "address": "bpi-m4zero.local:8769"}, 0.0))
    check("coming back is worth a line", any("answered again" in line["text"]
                                             for line in talking.log), True)
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

    # Now the same again while that move is still running. The wheels must not
    # stop: the new place goes out as a `retarget`, which Nav2 takes as a
    # replacement goal without halting the behaviour tree, so the rover keeps
    # driving the route it has until the planner produces the new one.
    session.act({"do": "tap", "col": 160, "row": 240})
    check("a click during a drive redirects it instead of stopping it",
          [name for name, _ in session.halt.sent], ["retarget"])
    check("...to the place under the cursor",
          (session.halt.sent[0][1]["x_m"], session.halt.sent[0][1]["y_m"]),
          (2.0, 0.0))
    check("...and does not try to overtake the move on its own connection",
          len(session.moves.sent), 1)
    check("...and says so without promising a stop",
          "without stopping" in session.log[-2]["text"], True)

    # The rover takes it. The move already in flight is the one that reports the
    # arrival, so nothing new is sent and nothing is left waiting.
    session.handle(Reply("retarget", session.halt.sent[0][1],
                         {"ok": True, "reason": "handed over"}, 0.1))
    check("a taken redirect starts no second move", len(session.moves.sent), 1)
    check("...and leaves nothing waiting for the wheels",
          session.pending_target, None)
    check("...and the console is still busy with the move it began with",
          session.busy_name, "drive_to")

    # A redirect the rover will not take falls back to what the console did
    # before there was one: stop, and send the click when the wheels come free.
    session = console()
    session.busy_since, session.busy_name = 100.0, "drive_to"
    session.act({"do": "tap", "col": 240, "row": 160})
    session.handle(Reply("retarget", session.halt.sent[0][1],
                         {"ok": False, "reason": "blocked",
                          "detail": "there is no route to that place"}, 0.1))
    check("a refused redirect stops the move instead",
          [name for name, _ in session.halt.sent], ["retarget", "stop_driving"])
    check("...keeping the click, which goes when the move answers",
          (session.pending_target or {}).get("x_m"), 3.0)
    session.handle(Reply("drive_to", {}, {"ok": True, "reason": "stopped"}, 1.2))
    check("...as an ordinary move to the place that was clicked",
          [(name, a.get("x_m")) for name, a in session.moves.sent],
          [("drive_to", 3.0)])

    # And if the move ended while the redirect was in flight, the wheels are free
    # and there was never anything to redirect: it is just a drive.
    session = console()
    session.busy_since, session.busy_name = 100.0, "drive_to"
    session.act({"do": "tap", "col": 240, "row": 160})
    session.handle(Reply("retarget", session.halt.sent[0][1],
                         {"ok": False, "reason": "idle"}, 0.1))
    check("a redirect that arrived too late sends nothing on its own",
          [name for name, _ in session.moves.sent], [])
    check("...and no stop for a rover that has already stopped",
          [name for name, _ in session.halt.sent], ["retarget"])
    check("...but keeps the click", (session.pending_target or {}).get("x_m"), 3.0)
    session.handle(Reply("drive_to", {}, {"ok": True, "reason": "arrived"}, 2.0))
    check("...which goes as an ordinary drive when the move finally answers",
          [(name, a.get("x_m")) for name, a in session.moves.sent],
          [("drive_to", 3.0)])

    # A third place clicked while a redirect is outstanding replaces the waiting
    # one rather than queueing, and still never stops the rover.
    session = console()
    session.busy_since, session.busy_name = 100.0, "drive_to"
    session.act({"do": "tap", "col": 240, "row": 160})
    session.act({"do": "tap", "col": 160, "row": 240})
    check("a further click replaces the waiting one rather than queueing behind it",
          (session.pending_target or {}).get("y_m"), 0.0)
    check("...without a second redirect while the first is unanswered",
          len(session.halt.sent), 1)
    session.handle(Reply("retarget", session.halt.sent[0][1],
                         {"ok": True, "reason": "handed over"}, 0.1))
    check("...which goes as soon as the first is answered",
          [(name, a.get("y_m")) for name, a in session.halt.sent],
          [("retarget", -1.0), ("retarget", 0.0)])
    check("...and the wheels were never stopped for any of it",
          [name for name, _ in session.halt.sent if name == "stop_driving"], [])

    # STOP after a click must not be followed by the rover setting off for it. This
    # is the one that would be unforgivable.
    session.act({"do": "stop"})
    check("pressing STOP throws the waiting click away", session.pending_target,
          None)
    session.handle(Reply("drive_to", {}, {"ok": True, "reason": "stopped"}, 0.3))
    check("...so the move that answers is the end of it, not the start of another",
          session.moves.sent, [])
    # The redirect that was in flight when STOP was pressed answers afterwards,
    # and this is the one that would be unforgivable: a rover that stopped and
    # then set off again on its own.
    session.handle(Reply("retarget", {"x_m": 3.0, "y_m": -1.0},
                         {"ok": True, "reason": "handed over"}, 0.4))
    check("...and a redirect answering after STOP starts nothing either",
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
          "dropped" in session.log[-1]["text"], True)

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
