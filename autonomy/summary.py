"""An episode in a few lines, for a person or for a model to read.

Both audiences want the same thing, which is why there is one function rather
than two: what the rover was trying to do, what it chose and why, what it
actually called, how it ended, and what the record can no longer stand behind. A
model given a page of JSON will summarise it into something like this and get it
wrong sometimes; a model given this does not have to.

**What is missing is part of the summary and not an appendix to it.** An episode
whose evidence has been deleted says so on the face of it, because the failure
this component exists to prevent is a confident account of something nobody can
check any more.
"""
from __future__ import annotations

import time
from typing import Any

import replay as replay_mod
import store as store_mod


def of(store: store_mod.EpisodeStore, episode_ref: str, *,
       live_world_generation: str | None = None) -> str:
    """One episode, as prose. Between four and a dozen lines."""
    got = replay_mod.reconstruct(store, episode_ref,
                                 live_world_generation=live_world_generation)
    return render(got)


def render(got: dict[str, Any]) -> str:
    """The same, from a reconstruction somebody already has."""
    lines = [f"{_name(got['ref'])} -- opened {_when(got['opened_at'])}, "
             f"triggered by {got['trigger']}"]

    where = [f"world state {_generation(got['world_generation'])}"]
    if got["map_session"] is not None:
        where.append(f"map session {got['map_session']}")
    lines.append("  " + ", ".join(where))

    if got["note"]:
        lines.append(f"  {got['note']}")

    decision = got["decision"]
    if decision is None:
        lines.append("  decided nothing")
    else:
        considered = len(got["candidates"])
        counted = ("considered nothing it wrote down" if not considered
                   else f"considered {considered} "
                        f"{'goal' if considered == 1 else 'goals'}")
        lines.append(f"  {counted}, chose {decision['chose']}: {decision['why']}")
        if decision["inputs"] is None:
            lines.append("  the world it chose from is no longer in this "
                         "database, so the choice cannot be checked")

    for body in got["calls"]:
        verdict = "ok" if body.get("ok") else f"failed: {body.get('error', '')}"
        lines.append(f"  called {body.get('call')}"
                     f"{_params(body.get('params'))} -- {verdict}")
    if not got["calls"]:
        lines.append("  called nothing")

    # What the occasion did to the world. On an episode with no decision in it
    # -- every episode a shadow run records -- this is the whole of the content,
    # and a summary that left it out would say almost nothing.
    for step in got["steps"]:
        if step["kind"] == "world_change":
            lines.append(f"  {step['body'].get('what')}")

    # What a move actually did. On a move episode the phases are the whole of
    # the content, the way the world_change line is on a look -- without them a
    # move reads as four lines saying nothing happened.
    phases = [step["body"] for step in got["steps"]
              if step["kind"] == "measured" and step["body"].get("phase")]
    if phases:
        asked = (got["trigger_detail"] or {}).get("asked")
        kind = (got["trigger_detail"] or {}).get("kind")
        lines.append(f"  {kind}{_params(asked)}, and it went "
                     + " -> ".join(_runs([one["phase"] for one in phases])))
        for one in phases:
            if one.get("why"):
                lines.append(f"    {one['phase']}: {one['why']}")
        shape = _shape(phases)
        if shape:
            lines.append("  " + shape)

    for body in got["models"]:
        lines.append(f"  asked {body.get('provider')}/{body.get('model')} "
                     f"for {body.get('purpose')}")

    outcome = got["outcome"]
    if outcome is None:
        lines.append("  still open")
    else:
        detail = outcome.get("detail", "")
        lines.append(f"  closed: {outcome['outcome']}"
                     + (f" -- {detail}" if detail else ""))

    held = [one for one in got["evidence"] if one["state"] == store_mod.HELD]
    if held:
        lines.append(f"  {_count(len(held), 'piece')} of evidence kept, "
                     f"{_bytes(sum(one.get('bytes') or 0 for one in held))}")

    for gone in got["missing"]:
        lines.append("  " + _gone(gone))

    for ref in got["references"]:
        if ref["now_called"]:
            lines.append(f"  {ref['local'] or ref['ref']} is now "
                         f"{', '.join(_name(one) for one in ref['now_called'])}")
    unlookable = [one for one in got["references"]
                  if one["why_not"] and not one["resolvable"]]
    if unlookable:
        first = unlookable[0]
        rest = (f" (and {len(unlookable) - 1} more)"
                if len(unlookable) > 1 else "")
        lines.append(f"  {first['local'] or first['ref']} cannot be looked up: "
                     f"{first['why_not']}{rest}")
    return "\n".join(lines)


def recent(store: store_mod.EpisodeStore, *, limit: int = 10) -> str:
    """The last few episodes, one line each, newest first."""
    rows = store.episodes(limit=limit)
    if not rows:
        return "no episodes recorded"
    return "\n".join(line(store, row) for row in rows)


def line(store: store_mod.EpisodeStore, episode: dict[str, Any]) -> str:
    outcome = store.outcome(episode["ref"])
    ended = "open" if outcome is None else outcome["outcome"]
    return (f"{_name(episode['ref']):<14} {_when(episode['opened_at'])}  "
            f"{episode['trigger']:<20} {ended}")


# --- wording -----------------------------------------------------------------

def _name(ref: str) -> str:
    """The readable tail of a reference. Full references are for the database."""
    return ref.rsplit("/", 1)[-1] if "/" in ref else ref


def _generation(generation: str) -> str:
    return ("not known -- nothing in it can be looked up"
            if generation == "unknown" else generation)


def _when(stamp: float | None) -> str:
    if not stamp:
        return "at an unknown time"
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(stamp))


def _params(params: Any) -> str:
    if not isinstance(params, dict) or not params:
        return ""
    inside = ", ".join(f"{k}={v}" for k, v in sorted(params.items()))
    return f"({inside})"


def _runs(phases: list) -> list[str]:
    """Collapse repeats, so eleven polls of "driving" read as one step."""
    out: list[str] = []
    for phase in phases:
        if out and out[-1].split(" x")[0] == phase:
            first = out[-1].split(" x")[0]
            times = int(out[-1].split(" x")[1]) if " x" in out[-1] else 1
            out[-1] = f"{first} x{times + 1}"
        else:
            out.append(str(phase))
    return out


def _shape(phases: list) -> str:
    """The numbers a move leaves behind, where it left any."""
    said = []
    route = next((one["route_m"] for one in reversed(phases)
                  if one.get("route_m")), None)
    if route:
        said.append(f"{route:.2f} m of route")
    points = next((one["waypoints"] for one in reversed(phases)
                   if one.get("waypoints")), None)
    if points:
        said.append(f"{points} waypoints")
    replans = max([one.get("replans") or 0 for one in phases] or [0])
    if replans:
        said.append(f"{replans} replan{'' if replans == 1 else 's'}")
    watched = sum(1 for one in phases if one.get("watched"))
    said.append(f"{watched} of {len(phases)} steps seen as they happened, "
                f"the rest read back afterwards"
                if watched < len(phases) else
                f"all {len(phases)} steps seen as they happened")
    return ", ".join(said)


def _count(number: int, thing: str) -> str:
    return f"{number} {thing}{'' if number == 1 else 's'}"


def _bytes(total: int) -> str:
    if total < 1024:
        return f"{total} bytes"
    if total < 1024 * 1024:
        return f"{total / 1024:.0f} kB"
    return f"{total / (1024 * 1024):.1f} MB"


def _gone(missing: dict[str, Any]) -> str:
    """Say which kind of gone, because they call for different reactions."""
    what = missing["what"]
    if missing["state"] == store_mod.DELETED:
        when = f" on {_when(missing.get('at'))}" if missing.get("at") else ""
        return (f"the {what} was deleted{when}: {missing.get('why', '')}"
                " -- this episode can no longer be replayed in full")
    return (f"the {what} is missing: {missing.get('why', 'not in this database')}"
            " -- this episode can no longer be replayed in full")
