#!/usr/bin/env python3
"""Read the record back: what the rover did, when, and what it was looking at.

    ssh orin 'cd ~/ugv/autonomy && python3 review.py'            the last 20
    ssh orin 'cd ~/ugv/autonomy && python3 review.py --moves'    only the driving
    ssh orin 'cd ~/ugv/autonomy && python3 review.py episode:481'
    ssh orin 'cd ~/ugv/autonomy && python3 review.py --stats'
    ssh orin 'cd ~/ugv/autonomy && python3 review.py episode:481 --save ~/look'

**Nothing here touches the rover.** It opens the record and reads it, so it works
just as well against a copy of the database on a desk as against the live one --
pass `--dir`. That is the same property `replay` has and for the same reason: an
account of what the rover did should be readable without the rover.

An episode is named by its short form, `episode:481`, because that is what the
listings print and what somebody reads out loud. The full reference with its
generation is still what the database holds, and `--full` prints it.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Any

import refs
import replay
import retention
import store as store_mod
import summary as summary_mod


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read back what the rover did. Touches nothing.")
    parser.add_argument("episode", nargs="?", default=None,
                        help="an episode to show in full, e.g. episode:481")
    parser.add_argument("--dir", default=None,
                        help="a record other than ~/.ugv/autonomy")
    parser.add_argument("--limit", type=int, default=20,
                        help="how many to list (default 20)")
    parser.add_argument("--moves", action="store_true",
                        help="only episodes where the rover moved")
    parser.add_argument("--looks", action="store_true",
                        help="only episodes where the rover looked")
    parser.add_argument("--world", default=None,
                        help="only one filling of the world state, by generation")
    parser.add_argument("--pinned", action="store_true",
                        help="only episodes kept from retention")
    parser.add_argument("--broken", action="store_true",
                        help="only episodes that can no longer be replayed in full")
    parser.add_argument("--full", action="store_true",
                        help="print whole references rather than short names")
    parser.add_argument("--stats", action="store_true",
                        help="what the record holds, and what it cost")
    parser.add_argument("--save", default=None,
                        help="write an episode's pictures into this directory")
    parser.add_argument("--live", default=None,
                        help="the world generation live now, so that references "
                             "can be reported as resolvable or not")
    args = parser.parse_args(argv)

    store = store_mod.EpisodeStore(args.dir)
    try:
        if args.stats:
            return stats(store)
        if args.episode:
            return show(store, args.episode, save=args.save, live=args.live)
        return listing(store, args)
    finally:
        store.close()


# --- the listing -------------------------------------------------------------

def listing(store: store_mod.EpisodeStore, args: Any) -> int:
    rows = store.episodes(limit=4000, world_generation=args.world)
    if args.moves:
        rows = [one for one in rows if one["trigger"] == "the rover moved"]
    if args.looks:
        rows = [one for one in rows if one["trigger"] == "the rover looked"]
    if args.pinned:
        rows = [one for one in rows if store.is_pinned(one["ref"])]
    if args.broken:
        rows = [one for one in rows
                if not replay.reconstruct(store, one["ref"])["replayable"]]
    if not rows:
        print("nothing in the record matches that")
        return 0

    shown = rows[:args.limit]
    print(f"{len(rows)} episode(s) match; showing {len(shown)}, newest first")
    print()
    for one in shown:
        print(line(store, one, full=args.full))
    if len(rows) > len(shown):
        print()
        print(f"...and {len(rows) - len(shown)} more. --limit takes a number.")
    print()
    print(f"one of them in full:  python3 review.py "
          f"{_short(shown[0]['ref'])}")
    return 0


def line(store: store_mod.EpisodeStore, episode: dict[str, Any], *,
         full: bool = False) -> str:
    """One episode on one line: when, what, how it went, and what it was about."""
    outcome = store.outcome(episode["ref"])
    ended = "open" if outcome is None else outcome["outcome"]
    name = episode["ref"] if full else _short(episode["ref"])
    detail = episode["trigger_detail"] or {}
    what = (f"{detail.get('kind') or 'a move'}"
            if episode["trigger"] == "the rover moved"
            else f"{detail.get('regions', '?')} region(s)"
                 f" at pan {_deg(detail.get('pan_deg'))}")
    marks = "".join((
        "*" if store.is_pinned(episode["ref"]) else " ",
        "!" if not replay.reconstruct(store, episode["ref"])["replayable"]
        else " ",
    ))
    return (f"{marks}{name:<13} {_when(episode['opened_at'])}  "
            f"{_trigger(episode['trigger']):<7} {what:<26} {ended}")


# --- one episode -------------------------------------------------------------

def show(store: store_mod.EpisodeStore, wanted: str, *, save: str | None,
         live: str | None) -> int:
    ref = _find(store, wanted)
    if ref is None:
        print(f"no episode called {wanted!r}. "
              f"`python3 review.py` lists what there is.")
        return 1

    print(summary_mod.of(store, ref, live_world_generation=live))
    print()

    got = replay.reconstruct(store, ref, live_world_generation=live)
    detail = got["trigger_detail"] or {}
    if detail.get("builds"):
        print("  produced by: "
              + ", ".join(f"{k} {v}" for k, v in sorted(detail["builds"].items())))
    print("  pinned against retention" if store.is_pinned(ref)
          else "  not pinned, so retention may take its pictures")

    print()
    print("  step by step:")
    for step in got["steps"]:
        after = " (added after it closed)" if step["after_close"] else ""
        print(f"    {step['seq']:>3} {_clock(step['at'])} {step['kind']}"
              f"{after}: {_body(step)}")
        for corrected in step["corrected_by"]:
            print(f"        corrected by step {corrected}")

    if got["references"]:
        print()
        print("  things it named:")
        for one in got["references"]:
            state = ("resolvable in the store live now" if one["resolvable"]
                     else one["why_not"] or "not resolvable")
            print(f"    {one['local']}: {state}")

    if got["evidence"]:
        print()
        print("  pictures:")
        for one in got["evidence"]:
            print(f"    {one['digest'][:19]}... {one['state']}"
                  + (f", {one['bytes']} bytes" if one.get("bytes") else "")
                  + (f" -- {one['why']}" if one.get("why") else ""))

    if save:
        print()
        print(_save(store, got, save))
    elif got["evidence"]:
        print()
        print(f"  to look at them:  python3 review.py {_short(ref)} "
              f"--save ~/look")
    return 0


def _save(store: store_mod.EpisodeStore, got: dict[str, Any],
          where: str) -> str:
    """Write an episode's pictures out, so somebody can actually look at them."""
    where = os.path.expanduser(where)
    os.makedirs(where, exist_ok=True)
    written, missing = [], []
    for one in got["evidence"]:
        data = store.evidence_bytes(one["digest"])
        if data is None:
            missing.append(one)
            continue
        name = f"{_short(got['ref']).replace(':', '-')}-{one['digest'][7:19]}.jpg"
        with open(os.path.join(where, name), "wb") as handle:
            handle.write(data)
        written.append(name)
    said = [f"  wrote {len(written)} picture(s) into {where}"]
    said += [f"    {name}" for name in written]
    for one in missing:
        said.append(f"    {one['digest'][:19]}... is {one['state']}"
                    + (f": {one['why']}" if one.get("why") else "")
                    + " and cannot be written out")
    return "\n".join(said)


# --- what the record holds ---------------------------------------------------

def stats(store: store_mod.EpisodeStore) -> int:
    got = store.summary()
    rows = store.episodes(limit=100000)
    print(f"the record at {got['dir']}")
    print(f"  {got['episodes']} episodes, {got['events']} events, "
          f"{got['snapshots']} snapshots")
    print(f"  {got['evidence']} pictures held, "
          f"{retention.size(got['evidence_bytes'])}, "
          f"{got['deletions']} deleted")
    print(f"  {len(store.pinned())} episodes pinned against retention")
    print(f"  schema {got['schema_version']}, this record's own id "
          f"{got['generation']}")

    kinds: dict[str, int] = {}
    worlds: dict[str, int] = {}
    for one in rows:
        kinds[one["trigger"]] = kinds.get(one["trigger"], 0) + 1
        worlds[one["world_generation"]] = worlds.get(one["world_generation"], 0) + 1
    print()
    print("  what happened:")
    for name, count in sorted(kinds.items(), key=lambda kv: -kv[1]):
        print(f"    {count:>5}  {name}")
    print()
    print("  which filling of the world state they belong to:")
    for name, count in sorted(worlds.items(), key=lambda kv: -kv[1]):
        print(f"    {count:>5}  {name}")
    print("         (an episode can only be looked up against its own)")

    if rows:
        first, last = rows[-1]["opened_at"], rows[0]["opened_at"]
        print()
        print(f"  spanning {_when(first)} to {_when(last)}")
    print()
    print(f"  growth: {retention.would_fill(store)}")
    print(f"  policy: {retention.DEFAULT.describe()}")
    return 0


# --- wording -----------------------------------------------------------------

def _find(store: store_mod.EpisodeStore, wanted: str) -> str | None:
    if refs.parse(wanted):
        try:
            store.episode(wanted)
            return wanted
        except KeyError:
            return None
    for one in store.episodes(limit=100000):
        if _short(one["ref"]) == wanted:
            return one["ref"]
    return None


def _short(ref: str) -> str:
    return ref.rsplit("/", 1)[-1]


def _trigger(trigger: str) -> str:
    return "moved" if "moved" in trigger else "looked"


def _deg(value: Any) -> str:
    return "?" if value is None else f"{value:+.0f}"


def _when(stamp: float) -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(stamp))


def _clock(stamp: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(stamp))


def _body(step: dict[str, Any]) -> str:
    body = dict(step["body"])
    if step["kind"] == "measured" and body.get("phase"):
        said = [str(body["phase"])]
        if body.get("why"):
            said.append(str(body["why"]))
        for key in ("route_m", "waypoints", "replans", "reason"):
            if body.get(key):
                said.append(f"{key}={body[key]}")
        return ", ".join(said)
    for key in ("what", "text", "outcome", "chose", "goal", "call"):
        if body.get(key):
            rest = {k: v for k, v in body.items()
                    if k != key and v not in (None, "", [], {})}
            trimmed = ", ".join(f"{k}={v}" for k, v in sorted(rest.items()))
            return f"{body[key]}" + (f" ({trimmed})" if trimmed else "")
    return ", ".join(f"{k}={v}" for k, v in sorted(body.items()))


if __name__ == "__main__":
    sys.exit(main())
