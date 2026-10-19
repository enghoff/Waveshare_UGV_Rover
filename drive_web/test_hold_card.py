"""The "keep it on" card: which agents want the rover kept switched on.

What the card must never do is say "nobody" when it does not know, because
nobody is the answer the owner switches the rover off on. So the checks are of
every answer the daemon can give landing somewhere honest, and of a daemon too
old to keep holds taking the card off the page rather than leaving it empty.
"""
from __future__ import annotations

import time

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import SKIP, check


def test_the_card_shows_every_hold_and_says_when_it_cannot() -> None:
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"keep it on card ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    sent = []
    session.watch = type("Fake", (), {"submit": lambda _s, name, args=None:
                                      sent.append((name, args))})()
    check("the card waits for the rover's first answer",
          session.snapshot()["holds"]["supported"], None)
    session.pump()
    check("the rover is asked who holds it", [n for n, _ in sent].count("power_holds"), 1)
    session.pump()
    check("...and not again while that question is out",
          [n for n, _ in sent].count("power_holds"), 1)

    now = time.time()
    session.handle(drive_web.Reply("power_holds", {}, {"ok": True, "holds": [
        {"who": "ugv-rover-d7", "what": "benchmark day 2", "when": "planned",
         "since": now - 600, "until": None},
        {"who": "codex", "what": "a replay", "when": "now",
         "since": now - 60, "until": now + 3600}]}, 0.01))
    rows = session.snapshot()["holds"]["rows"]
    check("every hold is a row, with who, when and what",
          [(r["who"], r["when"], r["what"]) for r in rows],
          [("ugv-rover-d7", "planned", "benchmark day 2"),
           ("codex", "now", "a replay")])
    check("...and when it was set and lapses, in the rover's own clock",
          (rows[0]["times"].startswith("since "), ", until " in rows[1]["times"],
           ", until " in rows[0]["times"]), (True, True, False))
    check("...and the card is on the page", session.holds["supported"], True)
    check("...and the question is free to be asked again",
          session.holds_outstanding, False)

    session.show_holds({"ok": True, "holds": []})
    check("no holds is nobody, with nothing wrong",
          (session.holds["rows"], session.holds["note"]), ([], ""))

    session.show_holds({"ok": False, "error": "the holds file could not be read: x"})
    check("a file the rover could not read is said, never shown as nobody",
          (session.holds["rows"], session.holds["note"]),
          ([], "the holds file could not be read: x"))

    session.show_holds({"ok": False, "error": "no such tool: power_holds"})
    check("a daemon that keeps no holds takes the card off the page",
          session.holds["supported"], False)
    session.holds_at = 0.0
    sent.clear()
    session.pump()
    check("...and is not asked again", [n for n, _ in sent].count("power_holds"), 0)


TESTS = (test_the_card_shows_every_hold_and_says_when_it_cannot,)
