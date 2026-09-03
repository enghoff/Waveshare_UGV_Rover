#!/usr/bin/env python3
"""Two ways of turning a pool of bearings into things, on the same recording.

    python world_state/bench_cluster.py /tmp/run.db --frames /tmp/frames \
        --map /tmp/map.json

**This is what the choice in `resolve.DISCOVERY` rests on**, and it exists
because the two passes are not comparable by argument. `_pair_up` searches over
pairs of bearings and commits to the best-supported crossing; `_cluster_up` fits
every leftover bearing at once and lets the association settle itself by
expectation-maximisation. Both fill the same slot in the resolver, so the whole
comparison is one name and a replay.

Three numbers come out and only one of them is new. `mixed` and `stray` are
[replay.py](replay.py)'s own scores, unchanged, because a pass must not mark its
own homework. The third is **how far each attached bearing actually misses the
thing it was attached to, in degrees** -- and it is here because `stray` divides
by the entity's own stated uncertainty, so a pass that reports a tighter figure
scores worse on it while being more nearly right. An angle is scale-free and
settles that.

## What it measured, on the drive of 2026-09-03

The recording holds 406 usable regions of which only 131 carry a bearing at all,
from 35 looks standing in 26 distinct places; the other 275 have no pose stored,
which is the fault the shutter fix addressed and which no estimator can reach
back and repair.

| | things | attached | mixed | stray | median miss | worst | time |
|---|---:|---:|---:|---:|---:|---:|---:|
| `_pair_up`, `refine` crossing a pair | 15 | 66 | 0 | 1 | 1.56 | 48.9 | 1.6 s |
| **`_pair_up`, `refine` fitting** | **15** | **65** | **0** | **1** | **1.41** | **21.1** | **1.9 s** |
| `_cluster_up`, one arrangement | 10 | 52 | 0 | 1 | 1.71 | 26.4 | 32 s |
| `_cluster_up`, soft weights | — | — | — | — | — | — | >30 min |

Misses are degrees. The soft-weight row is not filled in because it did not
finish: enumerating every feasible arrangement of every look, once per candidate
dropped, is not a thing that can run after every look on the rover. That is a
result rather than a gap — see **Cost** below.

Two conclusions, and they point opposite ways.

**The fit is a better placer, and it ships.** Given the same rays and the same
associations, it puts things where the bearings actually agree: the median miss
is 1.41 degrees against 1.56, and the worst is 21.1 against 48.9. That is the
whole of the second row, and it is `locate.refine` alone -- the association
machinery is untouched.

**The fit is a worse discoverer, and it does not ship.** Not because the
arithmetic is worse but because discovery here is *incremental*: `_pair_up` sees
the pool again after every look and offers every waiting ray to everything
already placed, through the wide `locate.match_tolerance` gate. `_cluster_up` has
to find things from crossings inside one pass's leftovers, and with 35 usable
looks in the whole recording no single pass holds enough. What it does place it
places cleanly — no crop belonging to something else, one straying bearing, the
same as the greedy pass — it simply places two thirds as much.

**And the leftovers really are nothing**, which is worth recording because it was
the question that started this. Of the 65 bearings the greedy pass leaves
unattached, 30 pairs cross admissibly on geometry alone; the best of those 30
scores 0.48 on appearance and the median 0.30, against the 0.55 floor and against
the 0.69 that two regions of *the same frame* reach at the 95th percentile. They
are different things, and refusing them is correct.

## What would change it, measured rather than hoped

**A range on each ray.** The reason `_cluster_up` cannot separate things is not
noise, it is that a crossing where no object is lies *exactly* on rays belonging
to the real objects either side of it, and fits them exactly as well. There is
nothing in the angles to prefer the truth. Three objects in a row seen from two
places: bearings alone place one thing and it is the phantom; the same rays each
carrying a range place all three exactly. `test_cluster.py` holds both as checks.

The OAK-D-Lite on the front of this rover has served stereo depth on loopback
8770 since 2026-08-31 and nothing reads it. `locate.residuals` is where a range
would enter and `RANGE_SIGMA_M` is the one constant it needs; what stands in the
way is extrinsics rather than arithmetic, and
[oak_depth/README.md](../oak_depth/README.md) lists what has to be settled.

**Cost, and it is the other reason.** One expectation step enumerates every
feasible arrangement of every look, and the pruning runs one of those per
candidate dropped. Taking the single best arrangement instead costs 32 seconds
where the greedy pass takes 1.9; enumerating them all did not finish in half an
hour. The rover resolves after every look, so neither is affordable as it stands.
Before this could run there the candidate set would have to be bounded — which
a range also does, by collapsing each ray's candidates from a handful to one.
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

from . import replay as replay_module               # noqa: E402
from . import resolve as resolver                   # noqa: E402


def misses(entities: list[dict], observations: list[dict]) -> list[float]:
    """How far each attached bearing misses its own entity, in degrees.

    The scale-free score. `replay.score`'s `stray` counts a bearing as straying
    when it misses by more than the entity's *own stated* uncertainty allows, so
    a pass that states a tighter figure is charged for its honesty; this is the
    same question asked in a unit neither pass gets to choose.
    """
    placed: dict = {}
    for entity in entities:
        try:
            got = json.loads(entity["placement_json"] or "null")
        except (TypeError, ValueError):
            got = None
        if got:
            placed[entity["id"]] = got
    out = []
    for one in observations:
        where = placed.get(one["entity_id"])
        if not where or one["bearing_deg"] is None:
            continue
        try:
            pose = json.loads(one["observer_pose_json"])
        except (TypeError, ValueError):
            continue
        dx, dy = where["x_m"] - pose["x_m"], where["y_m"] - pose["y_m"]
        out.append(abs(replay_module._wrap(
            math.degrees(math.atan2(dy, dx)) - one["bearing_deg"])))
    return sorted(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("database", help="a world.db copied off the rover")
    parser.add_argument("--frames", default="")
    parser.add_argument("--map", default="")
    parser.add_argument("--frame-size", default="640x480")
    parser.add_argument("--only", default="",
                        help="run one pass by name: pairs, soft or hard")
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

    from . import cluster

    arms = (("pairs", resolver._pair_up, None),
            ("soft", resolver._cluster_up, True),
            ("hard", resolver._cluster_up, False))
    for name, pass_fn, soft in arms:
        if args.only and args.only != name:
            continue
        resolver.DISCOVERY = pass_fn
        real = cluster.discover
        if soft is not None:
            cluster.discover = (
                lambda *a, **k: real(*a, **{**k, "soft": soft}))
        began = time.time()
        try:
            entities, observations = replay_module.replay(
                args.database, skip=skip, reach=reach, groups=groups)
        finally:
            cluster.discover = real
            resolver.DISCOVERY = resolver._pair_up
        took = time.time() - began
        print(f"\n=== {name}   ({took:.1f} s)")
        replay_module.score(entities, observations)
        got = misses(entities, observations)
        if got:
            print(f"  how far an attached bearing actually misses:  "
                  f"median {statistics.median(got):.2f}  "
                  f"90th {got[int(len(got) * 0.9)]:.2f}  "
                  f"worst {got[-1]:.2f} deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
