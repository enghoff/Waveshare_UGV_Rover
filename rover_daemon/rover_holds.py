"""Who wants the rover kept switched on, and for what (R-CTL-12).

The owner switches the rover off by hand -- to carry it, to stop the battery
draining overnight -- and agents working on it from elsewhere had no way to say
"not now, I am using it" or "have it on at nine tomorrow" except in a
conversation the owner might not be reading. On 2026-10-18 and again on 10-19 a
session found the rover gone and spent its time working out why.

So an agent puts a hold here, and the console draws every hold in a card of its
own where the owner looks before reaching for the switch. A hold is a request,
not a lock: nothing on the rover refuses to power down because of one, and the
owner is free to ignore it.

    power_hold   {who, what, when: "now", hours?}                set or replace
    power_hold   {who, what, when: "planned", from, hours?}      ...for later
    power_hold   {who, release: true}                            take it back
    power_holds  {}                                              every live hold

One hold per `who`, so setting again replaces it rather than stacking a second.
`hours` makes it lapse by itself, which is what keeps a session that died
without releasing from holding the rover forever; without it a hold stands until
it is released. Both answer with the whole list.

A planned hold says when the rover is needed -- `from`, "2026-10-21 09:00" in
the rover's own clock or seconds since the epoch -- and its `hours` count from
then. Until then the owner may switch the rover off. On 10-20 a planned hold had
no start, so a run the next morning read as "keep it on until 11:00 tomorrow",
which is a night of battery spent on nothing.

Kept in a file under `~/.ugv`, because a planned hold has to outlive the daemon
restarts every deploy makes -- `UGV_HOLDS_FILE` moves it for the tests. Neither
call is a model tool: whether the rover may be switched off is the owner's
question, answered between people and the agents working for them.
"""
from __future__ import annotations

import json
import os
import threading
import time
from typing import Any

ENV_FILE = "UGV_HOLDS_FILE"
WHO_MAX = 40
WHAT_MAX = 160
WHENS = ("now", "planned")
#: Earlier than this and the clock has not been set since power-on -- the Orin
#: reads 1970 for about a minute after a cold boot -- so a hold's times would be
#: wrong by fifty years and every expiry meaningless.
CLOCK_SET_AFTER = 1.7e9

_LOCK = threading.Lock()


def holds_file() -> str:
    return os.environ.get(ENV_FILE) or os.path.expanduser("~/.ugv/power_holds.json")


def _read() -> list[dict[str, Any]]:
    """Every hold on disk. An unreadable file is an error, never an empty list:
    "nobody needs it" said over a file nobody could read would be the console
    inventing the answer the owner switches the rover off on."""
    try:
        with open(holds_file(), encoding="utf-8") as handle:
            holds = json.load(handle)
    except FileNotFoundError:
        return []
    except (OSError, ValueError) as error:
        raise ValueError(f"the holds file could not be read: {error}") from None
    if not isinstance(holds, list):
        raise ValueError("the holds file is not a list of holds")
    return [h for h in holds if isinstance(h, dict) and h.get("who")]


def _live(holds: list[dict[str, Any]], now: float) -> list[dict[str, Any]]:
    """The holds that have not lapsed. On an unset clock nothing is judged
    lapsed, because nothing can be."""
    if now < CLOCK_SET_AFTER:
        return holds
    return [h for h in holds if not h.get("until") or h["until"] > now]


def _write(holds: list[dict[str, Any]]) -> None:
    path = holds_file()
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    temporary = f"{path}.{os.getpid()}.tmp"
    with open(temporary, "w", encoding="utf-8") as handle:
        json.dump(holds, handle, indent=1)
    os.replace(temporary, path)


def _text(value: Any, name: str, limit: int) -> str:
    text = " ".join(str(value or "").split())
    if not text:
        raise ValueError(f"`{name}` is needed")
    if len(text) > limit:
        raise ValueError(f"`{name}` is {len(text)} characters; {limit} at most")
    return text


def _start(value: Any, now: float) -> float:
    """When a planned hold begins, as seconds since the epoch. A plan without
    one is refused: "planned" alone can only be read as "from now", which is
    the owner leaving the rover on all night for a run in the morning."""
    if value is None or value == "":
        raise ValueError("a planned hold needs `from`, when the rover is needed: "
                         "\"2026-10-21 09:00\" in the rover's clock, or epoch seconds")
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        start = float(value)
    else:
        text = str(value).strip().replace("T", " ")
        try:
            start = time.mktime(time.strptime(text, "%Y-%m-%d %H:%M"))
        except ValueError:
            raise ValueError(f"`from` {value!r} is not \"YYYY-MM-DD HH:MM\"") from None
    if start <= now:
        raise ValueError("`from` has already passed; a hold from now is "
                         "`when: \"now\"`")
    if start > now + 14 * 86400:
        raise ValueError("`from` is more than 14 days away")
    return start


class RoverHolds:
    """The two calls, mixed into Rover."""

    def _tool_power_hold(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Set, replace or release the caller's hold on the rover's power."""
        who = _text(arguments.get("who"), "who", WHO_MAX)
        now = time.time()
        with _LOCK:
            holds = [h for h in _live(_read(), now) if h["who"] != who]
            if not arguments.get("release"):
                if now < CLOCK_SET_AFTER:
                    raise ValueError("the rover's clock is not set yet -- it reads "
                                     "1970 for a minute after power-on; try again")
                what = _text(arguments.get("what"), "what", WHAT_MAX)
                when = arguments.get("when") or "now"
                if when not in WHENS:
                    raise ValueError(f"`when` is one of {', '.join(WHENS)}")
                hold = {"who": who, "what": what, "when": when,
                        "since": round(now), "until": None}
                start = now
                if when == "planned":
                    start = _start(arguments.get("from"), now)
                    hold["from"] = round(start)
                elif arguments.get("from") is not None:
                    raise ValueError("a hold for now starts now; a later one is "
                                     "`when: \"planned\"`")
                hours = arguments.get("hours")
                if hours is not None:
                    hours = float(hours)
                    if not 0 < hours <= 24 * 14:
                        raise ValueError("`hours` is more than 0 and at most 336")
                    hold["until"] = round(start + hours * 3600)
                holds.append(hold)
            _write(holds)
        return {"ok": True, "holds": holds}

    def _tool_power_holds(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Every hold that has not lapsed, oldest first."""
        with _LOCK:
            holds = _live(_read(), time.time())
        return {"ok": True, "holds": sorted(holds, key=lambda h: h.get("since") or 0)}
