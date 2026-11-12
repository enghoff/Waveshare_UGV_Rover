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
    check("...and the console keeps up with it, as it does with a move",
          session.moving(), True)

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


TESTS = (test_the_manual_line_and_the_talk_button,)
