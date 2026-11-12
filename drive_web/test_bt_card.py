"""The Bluetooth card: what the rover is paired with, and pairing from the page.

Pairing and scanning answer before they have done anything, so the card has to
keep saying what is under way until a later status says how it went -- and say
that once, not again on every poll, and not at all for an outcome the rover
already had before this console first asked.
"""
from __future__ import annotations

import time

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import SKIP, check

PAD = {"address": "AA:BB:CC:00:11:22", "name": "Xbox Wireless Controller",
       "kind": "controller", "paired": True, "connected": True, "input": "js0"}
HEADSET = {"address": "77:88:99:AA:BB:CC", "name": "Headset", "kind": "headset",
           "paired": True, "connected": False}
FOUND = {"address": "C0:FF:EE:00:00:01", "name": "8BitDo Pro 2",
         "kind": "controller", "paired": False, "connected": False}


def test_the_card_pairs_and_says_how_it_went() -> None:
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"bluetooth card ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    sent = []
    fake = type("Fake", (), {"submit": lambda _s, name, args=None:
                             sent.append((name, args))})()
    session.watch = fake

    def pump() -> None:
        # A pump with no rover connected goes looking for one and replaces the
        # status connection, so the fake is put back after every tick.
        session.pump()
        session.watch = fake
    reply = lambda name, body, args=None: session.handle(  # noqa: E731
        drive_web.Reply(name, args or {}, body, 0.01))
    asked = lambda: [n for n, _ in sent].count("bt_status")  # noqa: E731

    check("the card waits for the rover's first answer",
          session.snapshot()["bt"]["supported"], None)
    pump()
    pump()
    check("the rover is asked once, and not again while that is out", asked(), 1)

    old = {"what": "pair", "name": "Old pad", "ok": True, "connected": True,
           "at": time.time() - 3600}
    reply("bt_status", {"ok": True, "devices": [PAD, HEADSET, FOUND],
                        "heard": 9, "scanning": False, "last": old})
    card = session.snapshot()["bt"]
    check("every device is a row, with what it is and where it stands",
          [(r["name"], r["kind"], r["state"], r["act"]) for r in card["rows"]],
          [("Xbox Wireless Controller", "controller", "connected js0", "unpair"),
           ("Headset", "headset", "paired", "unpair"),
           ("8BitDo Pro 2", "controller", "found", "pair")])
    check("...and an outcome from before this console asked is not news",
          card["note"], "")

    session.act({"do": "bt_scan"})
    check("look sends a scan", sent[-1], ("bt_scan", None))
    check("...and the button greys at once", session.bt["scanning"], True)
    reply("bt_scan", {"ok": True, "scanning": True, "seconds": 30})
    reply("bt_status", {"ok": True, "devices": [PAD], "heard": 3,
                        "scanning": True, "last": old})
    before = asked()
    session.bt_at = time.monotonic() - 1.5
    pump()
    check("while a scan runs the rover is asked every second",
          asked(), before + 1)
    reply("bt_status", {"ok": True, "devices": [PAD, FOUND], "heard": 23,
                        "scanning": False, "last": old})
    check("a scan that ends says how much it heard",
          session.bt["note"], "heard 23 devices")

    session.act({"do": "bt_pair", "address": FOUND["address"]})
    check("pair sends the address", sent[-1],
          ("bt_pair", {"address": FOUND["address"]}))
    reply("bt_pair", {"ok": True, "pairing": FOUND["address"],
                      "name": FOUND["name"]})
    check("...and says it is pairing, greying every button",
          (session.bt["note"], session.bt["busy"]),
          ("pairing 8BitDo Pro 2", "8BitDo Pro 2"))
    busy = {"what": "pair", "address": FOUND["address"], "name": FOUND["name"]}
    reply("bt_status", {"ok": True, "devices": [PAD, FOUND], "heard": 23,
                        "scanning": False, "busy": busy, "last": old})
    check("...for as long as the rover is pairing", session.bt["busy"],
          "8BitDo Pro 2")
    done = {"what": "pair", "name": "8BitDo Pro 2", "ok": True,
            "connected": True, "at": time.time()}
    paired = dict(FOUND, paired=True, connected=True, input="js1")
    reply("bt_status", {"ok": True, "devices": [PAD, paired], "heard": 23,
                        "scanning": False, "last": done})
    check("then it says it worked",
          (session.bt["note"], session.bt["verdict"], session.bt["busy"]),
          ("paired 8BitDo Pro 2", "good", ""))
    reply("bt_status", {"ok": True, "devices": [PAD, paired], "heard": 23,
                        "scanning": False, "last": done})
    check("...and an outcome read twice is said once", session.bt["note"],
          "paired 8BitDo Pro 2")

    failed = {"what": "pair", "name": "Headset", "ok": False,
              "said": "AuthenticationFailed", "at": time.time() + 1}
    reply("bt_status", {"ok": True, "devices": [PAD], "last": failed})
    check("a pair that failed says why",
          (session.bt["note"], session.bt["verdict"]),
          ("could not pair Headset: AuthenticationFailed", "bad"))

    reply("bt_unpair", {"ok": False, "error": "still pairing Headset"},
          {"address": HEADSET["address"]})
    check("a refused button says why in the card",
          (session.bt["note"], session.bt["verdict"]),
          ("still pairing Headset", "bad"))

    reply("bt_status", {"ok": False, "error": "no such tool: bt_status"})
    check("a daemon that cannot pair takes the card off the page",
          session.bt["supported"], False)
    session.bt_at = 0.0
    before = asked()
    pump()
    check("...and is not asked again", asked(), before)


TESTS = (test_the_card_pairs_and_says_how_it_went,)
