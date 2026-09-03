#!/usr/bin/env python3
"""What knowing how high a thing is buys, on the same recording, either way.

    python world_state/bench_height.py /tmp/run.db --frames /tmp/frames \
        --map /tmp/map.json

**The vertical half of a ray was measured and thrown away on every box this
rover has ever drawn**, and this is what putting it back is worth. The lens
returns a direction in three dimensions; the bearing uses two of them. So a
crossing is a plan view, in which a bearing at a picture on the wall and a
bearing at the sideboard beneath it cross exactly as convincingly as two
bearings at one thing, and nothing in the resolver could tell the difference --
appearance cannot separate two objects on this rover, and geometry seen from
above cannot separate two heights.

Two arms, one name apart, the way [bench_cluster.py](bench_cluster.py) compares
the two discovery passes:

    flat        `locate.rise_disagreement` silenced, so every crossing is
                accepted on its plan view alone. This is the rover as it was.
    vertical    as shipped: two rays have to agree about how high the thing is
                as well as about where on the floor it is.

`mixed` and `stray` are [replay.py](replay.py)'s own scores, unchanged, because
a change must not mark its own homework. Three numbers are new. **refused** is
how many crossings the gate threw out, which is the size of the thing being
claimed. **heights** is how many placed things came out with one. And **rise
miss** is how far an attached look's own elevation puts a thing from where its
entity says it is, in metres -- the vertical twin of `bench_cluster.misses`, and
the number that says whether the heights are coherent rather than merely
present.

## What it measured, on the drive of 2026-09-04

459 regions over 114 looks, every one of them carrying an elevation once it is
worked out, and 77 off a box the frame had cut.

| | things | attached | still waiting | stray | a look against its entity | worst | over a metre |
|---|---:|---:|---:|---:|---:|---:|---:|
| flat | 41 | 373 | 75 | 29 | 0.11 | 3.42 | 12 of 41 |
| **vertical** | **45** | **377** | **71** | **26** | **0.10** | **1.33** | **11 of 45** |

Metres. **More things, more looks attached to them, fewer bearings straying, and
the vertical tail cut by two thirds** -- no entity spans more than 1.75 m of
height now where one spanned 3.81, and no single look sits more than 1.33 m from
the middle of its own entity where one sat 3.42. The gate refuses 1,136
crossings on the way.

The reason it separates is worth stating on its own, because it is the thing
appearance cannot do on this rover. Over the 12,299 pairs that clear every other
gate:

| pairs whose crops | how many | median vertical gap | refused |
|---|---:|---:|---:|
| look like one object (dino >= 0.70) | 1,128 | **0.16 m** | 2% |
| do not (dino < 0.70) | 11,171 | **0.62 m** | 30% |

**Four times the gap, and it costs 2% of the pairs that were genuinely the same
thing.** That is a discriminator the component did not have: the appearance floor
lets through 13% of pairs known to be different things, and a plan-view crossing
cannot see a height at all.

What the tolerance is made of, on those same alike pairs, in metres: the angle
0.07, the range 0.10, where the ray started 0.02, and **how tall the thing itself
is 0.34** -- which is most of it, and is the term that has to be there. Two looks
at either end of a doorway centre their boxes a metre apart and both are pointing
at the doorway.

Runs at a desk against a recording. **The recording must be replayed with the
bearings recomputed**, which this does for you: the elevation column did not
exist when the rover wrote these rows, so replayed as they stand every ray
abstains and both arms are the same run. The box, the pose, the gimbal angles
and the lens are still the rover's own -- what is worked out again is only the
arithmetic between them.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time

if __package__ in (None, ""):                       # run as a script
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "world_state"

from . import locate                                # noqa: E402
from . import replay as replay_module               # noqa: E402


def rise_misses(entities: list[dict], observations: list[dict]
                ) -> tuple[list[float], list[float]]:
    """How far apart an entity's own looks put it vertically: each look's miss
    from the middle of its entity, and each entity's top-to-bottom spread.

    The vertical twin of `bench_cluster.misses`, and **derived rather than read
    off the placement, so that both arms are scored the same way**. Only one arm
    writes a height, and marking it against its own stated figure while the
    other is marked against nothing would be a comparison of one thing. So the
    middle of what an entity's own looks say is taken as its height in both, and
    what is measured is how far its looks are from agreeing with each other.

    A look that measured no elevation, and an entity with no position, say
    nothing here.
    """
    placed: dict = {}
    for entity in entities:
        try:
            got = json.loads(entity["placement_json"] or "null")
        except (TypeError, ValueError):
            got = None
        if got:
            placed[entity["id"]] = got
    rises: dict = {}
    for one in observations:
        where = placed.get(one["entity_id"])
        if not where:
            continue
        try:
            pose = json.loads(one["observer_pose_json"])
        except (TypeError, ValueError):
            continue
        got = locate.rise_m(where, {"x_m": pose["x_m"], "y_m": pose["y_m"],
                                    "elevation_deg": one.get("elevation_deg")})
        if got is not None:
            rises.setdefault(one["entity_id"], []).append(got)
    misses, spreads = [], []
    for group in rises.values():
        middle = statistics.median(group)
        misses.extend(abs(one - middle) for one in group)
        if len(group) > 1:
            spreads.append(max(group) - min(group))
    return sorted(misses), sorted(spreads)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("database", help="a world.db copied off the rover")
    parser.add_argument("--frames", default="")
    parser.add_argument("--map", default="")
    parser.add_argument("--frame-size", default="640x480")
    parser.add_argument("--only", default="",
                        help="run one arm by name: flat or vertical")
    parser.add_argument("--detail", action="store_true")
    args = parser.parse_args()

    if not os.path.exists(args.database):
        print(f"no such recording: {args.database}", file=sys.stderr)
        return 2
    skip = (replay_module.blank_ids(args.database, args.frames)
            if args.frames else set())
    reach = replay_module.reach_from(args.map) if args.map else None
    groups = replay_module.inspections(args.database)
    width, height = (int(part) for part in args.frame_size.lower().split("x"))
    count, median = replay_module.remeasure(groups, (width, height))
    print(f"  {count} bearings worked out again, median move {median:.2f} deg")
    measured = sum(1 for group in groups for row in group
                   if row.get("elevation_deg") is not None)
    clipped = sum(1 for group in groups for row in group
                  if _clipped(row))
    print(f"  {measured} of them carry an elevation, "
          f"{clipped} off a box the frame cut")
    real = locate.rise_disagreement
    for name in ("flat", "vertical"):
        if args.only and args.only != name:
            continue
        refused = [0]

        def counting(point, first, second, _real=real, _tally=refused):
            got = _real(point, first, second)
            if got is not None and got[0] > got[1]:
                _tally[0] += 1
            return got

        # The flat arm silences the height everywhere rather than only in
        # `fix`: with no height written, `locate.stands_as_high` abstains of its
        # own accord and the join path is exactly what it was before any of
        # this. Silencing one and not the other would measure half a change.
        real_height = locate.height_over
        if name == "flat":
            locate.rise_disagreement = lambda *a, **k: None
            locate.height_over = lambda *a, **k: None
        else:
            locate.rise_disagreement = counting
        began = time.time()
        try:
            entities, observations = replay_module.replay(
                args.database, skip=skip, reach=reach, groups=groups)
        finally:
            locate.rise_disagreement = real
            locate.height_over = real_height
        took = time.time() - began
        print(f"\n=== {name}   ({took:.1f} s)")
        replay_module.score(entities, observations, detail=args.detail)
        heights = []
        for entity in entities:
            try:
                got = json.loads(entity["placement_json"] or "null")
            except (TypeError, ValueError):
                got = None
            if got and got.get("height_m") is not None:
                heights.append(float(got["height_m"]))
        print(f"  crossings the vertical test refused:  {refused[0]}")
        print(f"  placed things carrying a height:      {len(heights)}"
              + (f"   from {min(heights):+.2f} to {max(heights):+.2f} m "
                 f"about the camera" if heights else ""))
        misses, spreads = rise_misses(entities, observations)
        if misses:
            print(f"  a look against its entity's middle:   "
                  f"median {statistics.median(misses):.2f}  "
                  f"90th {misses[int(len(misses) * 0.9)]:.2f}  "
                  f"worst {misses[-1]:.2f} m")
        if spreads:
            over = sum(1 for one in spreads if one > 1.0)
            print(f"  an entity top to bottom:              "
                  f"median {statistics.median(spreads):.2f}  "
                  f"worst {spreads[-1]:.2f} m   "
                  f"{over} of {len(spreads)} disagree by over a metre")
    return 0


def _clipped(row: dict) -> bool:
    try:
        return _view().clipped_vertically(json.loads(row["bbox_json"]))
    except (TypeError, ValueError, KeyError):
        return False


def _view():
    from . import view
    return view


if __name__ == "__main__":
    sys.exit(main())
