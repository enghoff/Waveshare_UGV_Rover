"""The line under the header while a game controller drives, and its X button.

X is counted on the rover and toggled here, so the one thing that must not
happen is a toggle the person did not ask for: the count a console first sees is
whatever it had reached before the page opened.
"""
from __future__ import annotations

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import SKIP, check


def test_the_manual_line_and_the_talk_button() -> None:
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"manual line ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    toggles = []
    session.voice_toggle = lambda: toggles.append(1) or "X: starting the conversation"

    session.show_status({"ok": True, "manual": {"on": False, "voice": 4}})
    check("no controller is no line", session.snapshot()["manual"]["on"], False)
    check("...and the count already reached is not a press", toggles, [])
    check("...nor does it make the console poll faster", session.moving(), False)

    session.show_status({"ok": True, "manual": {
        "on": True, "controller": "Xbox Wireless Controller", "blocked": "",
        "front_m": 0.6, "voice": 4, "note": ""}})
    check("a controller is a line naming it",
          session.manual["text"], "manual: Xbox Wireless Controller")
    check("...but one nobody is touching is a parked rover to the pictures",
          session.moving(), False)
    session.show_manual({"on": True, "controller": "Xbox Wireless Controller",
                         "linear": 0.2, "angular": 0.0, "voice": 4})
    check("...and one that is driving keeps them at a move's pace",
          session.moving(), True)
    import drive_session
    session.manual_driven_at -= drive_session.MANUAL_SETTLE_S + 0.1
    check("...until a few seconds after it stops", session.moving(), False)

    session.show_manual({"on": True, "controller": "Xbox Wireless Controller",
                         "blocked": "ahead", "front_m": 0.27, "voice": 5,
                         "note": "lights on"})
    check("the lidar's stop says how close, and the line turns to a warning",
          (session.manual["text"], session.manual["blocked"]),
          ("manual: Xbox Wireless Controller -- stopped, 27 cm ahead -- lights on", True))
    check("X toggles the conversation once", toggles, [1])
    check("...and says so", session.notice["text"], "X: starting the conversation")
    session.show_manual({"on": True, "controller": "pad", "blocked": "", "voice": 5})
    check("...and only once per press", toggles, [1])

    session.show_manual({"on": True, "controller": "pad", "voice": 5,
                         "error": "navigation is not answering"})
    check("navigation down is said, since the stick then does nothing",
          session.manual["text"], "manual: pad -- navigation is not answering")


def test_the_lights_and_tracking_follow_the_rover() -> None:
    """Y, B and the voice model change these on the rover without this console
    asking; the status poll carries them, and the panels follow it."""
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"lights and tracking ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    session.light_level = 0
    session.track_at = 1e12          # the tracking poll just went
    session.show_status({"ok": True, "lights": 255, "tracking": True})
    snap = session.snapshot()
    check("lights switched on elsewhere show as on",
          snap["lights"]["level"], 255)
    check("...tracking started elsewhere shows at once",
          (session.tracking_on, snap["tracking"]), (True, "on"))
    check("...and asks for the rest of the tracking panel now, not at its poll",
          session.track_at, 0.0)

    session.track_at = 1e12
    session.show_status({"ok": True, "lights": 255, "tracking": True})
    check("no change is no extra question", session.track_at, 1e12)
    session.show_status({"ok": True, "lights": 0, "tracking": False})
    check("and both go off the same way",
          (session.light_level, session.tracking_on, session.snapshot()["tracking"]),
          (0, False, "off"))
    session.show_status({"ok": True})
    check("a daemon that does not say leaves them as they were",
          (session.light_level, session.tracking_on), (0, False))


def test_off_the_map_stays_on_the_line() -> None:
    """A drive by hand that left the map holds it until the rover is back,
    controller or not (ros_nav/off_map.py); the line says so meanwhile."""
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"off the map ({type(exc).__name__})")
        return
    import time

    session = drive_web.Session(None, 3.0, 480)
    since = time.mktime((2026, 11, 13, 23, 10, 0, 0, 0, -1))
    off = {"since": since, "left_at": {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0}}
    session.show_status({"ok": True, "manual": {"on": True, "controller": "pad",
                                                "voice": 0}, "off_map": off})
    check("driving off the map is said after the controller's name, as a warning",
          (session.manual["text"], session.manual["blocked"]),
          ("manual: pad -- off the map since 23:10 -- map and store paused", True))
    session.show_status({"ok": True, "manual": {"on": False, "voice": 0},
                         "off_map": off})
    check("...and stays when the controller has gone",
          (session.manual["on"], session.manual["text"]),
          (True, "off the map since 23:10 -- map and store paused"))
    check("...without keeping the pictures at a moving rover's pace",
          session.moving(), False)
    session.show_status({"ok": True, "manual": {"on": False, "voice": 0},
                         "off_map": None, "map_held": ""})
    check("back on the map, the line goes", session.manual["on"], False)
    session.show_status({"ok": True, "manual": {"on": False, "voice": 0},
                         "off_map": None, "map_held": "position unconfirmed"})
    check("a kept map the rover is not confirmed on -- switched on somewhere "
          "else -- holds it too, and says so",
          (session.manual["on"], session.manual["text"]),
          (True, "position unconfirmed -- map and store paused"))
    session.show_status({"ok": True, "manual": {"on": True, "controller": "pad",
                                                "voice": 0},
                         "off_map": None, "map_held": "manual driving"})
    check("...while driving by hand the line is enough on its own",
          session.manual["text"], "manual: pad")


TESTS = (test_the_manual_line_and_the_talk_button,
         test_the_lights_and_tracking_follow_the_rover,
         test_off_the_map_stays_on_the_line)
