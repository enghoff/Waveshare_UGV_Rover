"""Agents' holds on the rover's power: set, replaced, lapsed, released, kept.

What matters is that the list the console draws is the truth: one hold per
agent, gone when it lapses or is released, still there after a restart, and an
error rather than an empty list when the file cannot be read.
"""
from __future__ import annotations

import os
import tempfile
import time

from test_fakes import FakeLink
from test_harness import check


def _rover(root):
    import rover_daemon

    os.environ["UGV_HOLDS_FILE"] = os.path.join(root, "ugv", "power_holds.json")
    return rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")


def test_holds_are_set_replaced_released_and_outlive_a_restart():
    import rover_holds

    was = os.environ.get("UGV_HOLDS_FILE")
    with tempfile.TemporaryDirectory() as root:
        try:
            rover = _rover(root)
            check("nobody holds a new rover", rover.call("power_holds", {}),
                  {"ok": True, "holds": []})
            start = time.time() + 3 * 3600
            set_ = rover.call("power_hold", {"who": "ugv-rover-d7",
                                             "what": "multi-day benchmark, day 2",
                                             "when": "planned", "from": start})
            check("a hold is set", [(h["who"], h["when"], h["from"], h["until"])
                                    for h in set_["holds"]],
                  [("ugv-rover-d7", "planned", round(start), None)])
            rover.call("power_hold", {"who": "codex", "what": "a replay",
                                      "hours": 2})
            again = rover.call("power_hold", {"who": "ugv-rover-d7",
                                              "what": "driving now"})
            mine = [h for h in again["holds"] if h["who"] == "ugv-rover-d7"]
            check("setting again replaces the agent's hold rather than adding one",
                  [(h["what"], h["when"]) for h in mine], [("driving now", "now")])
            theirs = [h for h in again["holds"] if h["who"] == "codex"][0]
            check("...and a lapsing hold says when",
                  round((theirs["until"] - time.time()) / 3600, 1), 2.0)

            rover = _rover(root)
            check("a restarted daemon still has both",
                  sorted(h["who"] for h in rover.call("power_holds", {})["holds"]),
                  ["codex", "ugv-rover-d7"])
            left = rover.call("power_hold", {"who": "codex", "release": True})
            check("a release takes only that agent's hold",
                  [h["who"] for h in left["holds"]], ["ugv-rover-d7"])
            gone = rover.call("power_hold", {"who": "nobody", "release": True})
            check("releasing a hold nobody set is not an error", gone["ok"], True)

            rover.call("power_hold", {"who": "old", "what": "x", "hours": 1})
            path = os.environ["UGV_HOLDS_FILE"]
            import json
            with open(path) as handle:
                holds = json.load(handle)
            for hold in holds:
                if hold["who"] == "old":
                    hold["until"] = time.time() - 1
            with open(path, "w") as handle:
                json.dump(holds, handle)
            check("a lapsed hold is not shown",
                  [h["who"] for h in rover.call("power_holds", {})["holds"]],
                  ["ugv-rover-d7"])

            tomorrow = time.strftime("%Y-%m-%d %H:%M",
                                     time.localtime(time.time() + 86400))
            planned = rover.call("power_hold", {"who": "p", "what": "a run",
                                                "when": "planned",
                                                "from": tomorrow, "hours": 2})
            mine = [h for h in planned["holds"] if h["who"] == "p"][0]
            check("a planned hold starts at a time in the rover's own clock, "
                  "and its hours count from then",
                  (round((mine["from"] - time.time()) / 3600),
                   round((mine["until"] - mine["from"]) / 3600, 1)), (24, 2.0))
            bare = rover.call("power_hold", {"who": "x", "what": "y",
                                             "when": "planned"})
            check("a planned hold that does not say when is refused",
                  (bare["ok"], "`from`" in bare["error"]), (False, True))
            for name, arguments in (
                    ("one already past", {"when": "planned",
                                          "from": time.time() - 60}),
                    ("one with no date in it", {"when": "planned",
                                                "from": "tomorrow at nine"}),
                    ("a hold for now with a later start",
                     {"when": "now", "from": time.time() + 3600})):
                check(f"...and so is {name}",
                      rover.call("power_hold", {"who": "x", "what": "y",
                                                **arguments})["ok"], False)

            refused = rover.call("power_hold", {"who": "x", "what": "y",
                                                "when": "soon"})
            check("a when that is neither now nor planned is refused",
                  refused["ok"], False)
            check("...and so is a hold that says nothing",
                  rover.call("power_hold", {"who": "x"})["ok"], False)
            check("...and one with no name",
                  rover.call("power_hold", {"what": "y"})["ok"], False)
            check("...and one longer than a line",
                  rover.call("power_hold", {"who": "x",
                                            "what": "y" * 200})["ok"], False)

            real = time.time
            rover_holds.time.time = lambda: 86400.0
            try:
                early = rover.call("power_hold", {"who": "x", "what": "y"})
            finally:
                rover_holds.time.time = real
            check("a hold is refused while the clock still reads 1970",
                  (early["ok"], "clock" in early["error"]), (False, True))

            with open(path, "w") as handle:
                handle.write("{not json")
            broken = rover.call("power_holds", {})
            check("an unreadable file is an error, not an empty list",
                  (broken["ok"], "could not be read" in broken["error"]),
                  (False, True))
            check("...and nobody's hold is written over it",
                  rover.call("power_hold", {"who": "x", "what": "y"})["ok"], False)
        finally:
            if was is None:
                os.environ.pop("UGV_HOLDS_FILE", None)
            else:
                os.environ["UGV_HOLDS_FILE"] = was


TESTS = (test_holds_are_set_replaced_released_and_outlive_a_restart,)
