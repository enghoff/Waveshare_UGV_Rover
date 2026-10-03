#!/usr/bin/env python3
"""Whole-session EM against the deployed resolver, on one recorded map session.

    python world_state/bench_whole.py /tmp/world.db --session 67 --map /tmp/map.json

**What it asks.** [bench_cluster.py](bench_cluster.py) put
expectation-maximisation in the resolver's discovery slot, where it only ever
sees one pass's leftovers, and on 2026-09-03 it lost there. It never got to try
the thing EM is for: going back over everything already decided. The resolver
commits a look the moment it is offered and never revisits it, which is how one
door became `object:96` and `object:102` on 2026-10-03 -- the first was founded
on a picture with a chair in front of it and refused the door's clear views,
and nothing ever looks at two placed things again. So this replays a session
through the deployed resolver, then hands every ray of that session to
[cluster.py](cluster.py)'s EM at once, seeded with what the resolver ended up
holding, and scores both.

**The EM is cluster.py's, with four changes, and each is here for a reason.**

* Seeded from the resolver's things rather than from every pairwise crossing.
  This is a consolidation pass over what the rover holds; 2,400 rays would be
  2.9 million crossings to seed from.
* The E-step is the same likelihood, `cluster._likelihood` times the share,
  computed for every ray and place at once with numpy. The look constraint is
  kept: each look's single best arrangement, solved as an assignment, which is
  `cluster.discover`'s default.
* A ray may not claim a place behind the wall in front of it, nor one nearer
  than `locate.MIN_RANGE_M` or further than `locate.MAX_RANGE_M`. cluster.py
  asks those questions at the end, of every ray that claims a thing, and drops
  the whole thing if any one fails. That is harmless on a pass's leftovers and
  ruinous over a session: a thing seen 81 times from 15 places nearly always has
  one look taken from 0.7 m, and on the first run of this bench that one look
  dropped it. Asked of the ray, one bad look costs that look.
* A thing seen from one place survives if a ray claiming it measured its range,
  which is what `resolve._place_from_range` already accepts on the rover.

Three variants, because the first result needed explaining before it could be
believed: cluster.py exactly (shares re-estimated, fits within
`cluster.SAME_PLACE_M` merged); the same with a merge refused unless the two
never shared a look and their crops look alike in the median; and that with the
shares held equal.

**What it measured is in
[the progress entry](../docs/progress/2026-10-03-whole-session-em.md).** In
short: it finds real duplicates the resolver leaves -- the door, five
paintings, both ceiling fans -- in a few seconds for a whole session, and it
loses too much to be worth having. As cluster.py has it, 117 things become 24;
with the shares held equal 70, with two of its fifteen merges plainly two
different objects and the share of crops in the wrong thing up from 0 to 7%.

The scores are `replay.score`'s and `bench_cluster.misses`'s, unchanged, plus
one that neither asks: how many pairs of placed things stand within
`resolve.SAME_PLACE_M` of each other, were never in one picture together, and
look alike. That is the duplicate this bench exists to find, and it is a
candidate list rather than a verdict -- identical dining chairs stand closer
than that.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from collections import Counter

if __package__ in (None, ""):                       # run as a script
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "world_state"

from . import cluster, locate, resolve             # noqa: E402
from . import replay as replay_module               # noqa: E402
from .bench_cluster import misses                   # noqa: E402
from .store import _readable                        # noqa: E402

_ROOT_2PI = math.sqrt(2.0 * math.pi)


def _wrap(degrees):
    return (degrees + 180.0) % 360.0 - 180.0


class Session:
    """Every ray of one map session, held as arrays for the E-step."""

    def __init__(self, rows, reach):
        import numpy as np

        self.rows = rows
        self.rays = []
        for row in rows:
            ray = resolve.ray_of(_readable(dict(row), vectors=True), reach)
            if ray is not None:
                self.rays.append((ray, row))
        rays = [ray for ray, _row in self.rays]
        self.index = {ray["observation_id"]: i for i, ray in enumerate(rays)}

        def column(key, missing=np.nan):
            return np.array([missing if ray.get(key) is None else float(ray[key])
                             for ray in rays])

        self.x, self.y = column("x_m"), column("y_m")
        self.bearing = column("bearing_deg")
        self.sigma = np.array([locate.sigma_of(ray) for ray in rays])
        self.origin = column("origin_sigma_m", 0.0)
        self.range = column("range_m")
        self.ranged = ~np.isnan(self.range)
        stated = column("range_sigma_m", 0.0)
        self.range_sigma = np.where(stated > 0.0, stated, locate.RANGE_SIGMA_M)
        self.reach = column("reach_m", np.inf) + locate.SEE_PAST_M
        self.look = [ray.get("inference_id") for ray in rays]
        looks: dict = {}
        for index, key in enumerate(self.look):
            looks.setdefault(key, []).append(index)
        self.looks = list(looks.values())
        nothing = cluster.CLUTTER_PER_DEG * cluster.CLUTTER_PRIOR
        self.clutter = np.where(self.ranged, nothing * cluster.CLUTTER_PER_M, nothing)

        vectors = [np.frombuffer(row.get("dino_blob") or b"", dtype="<f4")
                   for _ray, row in self.rays]
        width = max((len(v) for v in vectors), default=0)
        self.has_vector = np.array([len(v) == width and width > 0 for v in vectors])
        unit = np.zeros((len(vectors), max(width, 1)))
        for index, vector in enumerate(vectors):
            if self.has_vector[index]:
                unit[index] = vector / max(float(np.linalg.norm(vector)), 1e-9)
        self.similar = unit @ unit.T

    def ray(self, index):
        return self.rays[index][0]

    def scores(self, places):
        """`cluster._likelihood` times share, every ray against every place."""
        import numpy as np

        px = np.array([p["x_m"] for p in places])
        py = np.array([p["y_m"] for p in places])
        extent = np.array([max(0.0, p.get("extent_m") or 0.0) for p in places])
        share = np.array([p.get("share") or 1.0 for p in places])
        dx = px[None, :] - self.x[:, None]
        dy = py[None, :] - self.y[:, None]
        distance = np.hypot(dx, dy)
        near = np.maximum(distance, locate.MIN_RANGE_M)
        noise = np.hypot(self.sigma[:, None],
                         np.degrees(np.arctan2(self.origin[:, None], near)))
        half = np.degrees(np.arctan2(extent[None, :] / 2.0, near))
        off = np.abs(_wrap(np.degrees(np.arctan2(dy, dx)) - self.bearing[:, None]))
        missed = np.maximum(0.0, off - half)
        got = np.exp(-0.5 * (missed / noise) ** 2) / (noise * _ROOT_2PI)
        short = np.maximum(0.0, np.abs(distance - np.nan_to_num(self.range)[:, None])
                           - extent[None, :] / 2.0)
        sigma_m = self.range_sigma[:, None]
        along = np.exp(-0.5 * (short / sigma_m) ** 2) / (sigma_m * _ROOT_2PI)
        got = np.where(self.ranged[:, None], got * along, got) * share[None, :]
        # Asked of the ray rather than of the thing; see the module docstring.
        got[(distance > self.reach[:, None]) | (distance < locate.MIN_RANGE_M)
            | (distance > locate.MAX_RANGE_M)] = 0.0
        # The appearance veto as cluster.py asks it: refused only where the ray
        # looks like none of the thing's known crops.
        for column, place in enumerate(places):
            known = [i for i in place["exemplars"] if self.has_vector[i]]
            if known:
                best = self.similar[:, known].max(axis=1)
                got[self.has_vector & (best < resolve.DIFFERENT_THING), column] = 0.0
        return got

    def weigh(self, places):
        """The E-step: each look's single best arrangement, as 0/1 weights."""
        import numpy as np
        from scipy.optimize import linear_sum_assignment

        got = self.scores(places)
        weights = np.zeros_like(got)
        for members in self.looks:
            mine, nothing = got[members], self.clutter[members][:, None]
            gain = np.where(mine > nothing,
                            np.log(np.maximum(mine, 1e-300)) - np.log(nothing),
                            -np.inf)
            columns = np.where(np.isfinite(gain).any(axis=0))[0]
            if not len(columns):
                continue
            gain = gain[:, columns]
            top = float(gain[np.isfinite(gain)].max()) + 1.0
            cost = np.where(np.isfinite(gain), top - gain, top)
            for row, picked in zip(*linear_sum_assignment(cost)):
                if cost[row, picked] < top:
                    weights[members[row], columns[picked]] = 1.0
        return weights


def _share(places, weights):
    """`cluster._share`, on a weight matrix."""
    totals = weights.sum(axis=0)
    average = float(totals.mean()) if len(totals) else 0.0
    for place, total in zip(places, totals):
        place["share"] = ((total + cluster.SHARE_PRIOR) / (average + cluster.SHARE_PRIOR)
                          if average > 0.0 else 1.0)


def _stands(session, place, claimed):
    """`cluster._survives`, less the per-ray checks the E-step now makes."""
    claiming = [session.ray(i) for i in claimed]
    if not claiming:
        return False
    if locate.standing_places(claiming) < 2:
        if not any(ray.get("range_m") is not None for ray in claiming):
            return False
    elif len(claiming) < cluster.MIN_CLAIMING:
        return False
    return place.get("error_major_m", 0.0) <= cluster.MAX_UNCERTAINTY_M


def _merge(session, places, lookalike):
    """`cluster._merge`, or the same refusing two things that do not look alike."""
    import numpy as np

    kept = []
    for place in sorted(places, key=lambda one: one["error_major_m"]):
        twin = None
        for other in kept:
            if math.hypot(place["x_m"] - other["x_m"],
                          place["y_m"] - other["y_m"]) >= cluster.SAME_PLACE_M:
                continue
            if lookalike:
                if ({session.look[i] for i in place["claimers"]}
                        & {session.look[i] for i in other["claimers"]}):
                    continue
                mine = [i for i in place["claimers"] if session.has_vector[i]]
                theirs = [i for i in other["claimers"] if session.has_vector[i]]
                if (mine and theirs and float(np.median(
                        session.similar[np.ix_(mine, theirs)])) < replay_module.JOIN):
                    continue
            twin = other
            break
        if twin is None:
            kept.append(place)
        else:
            twin["exemplars"] = sorted(set(twin["exemplars"]) | set(place["exemplars"]))
            twin["absorbed"] = twin.get("absorbed", []) + [place["id"]] + place.get("absorbed", [])
    return kept


def consolidate(session, seeds, *, shares=True, lookalike=False):
    """cluster.discover's loop, from the resolver's things, over every ray."""
    import numpy as np

    places = [dict(seed) for seed in seeds]
    rounds = 0
    for rounds in range(1, cluster.MAX_ROUNDS + 1):
        weights = session.weigh(places)
        if shares:
            _share(places, weights)
            places = [p for p in places if p["share"] >= cluster.MIN_SHARE]
        weights = session.weigh(places)
        moved, fitted = 0.0, []
        for column, place in enumerate(places):
            claimed = np.where(weights[:, column] > 0)[0]
            if not len(claimed):
                continue
            rays = [session.ray(i) for i in claimed]
            extent = cluster._extent(place, rays, [1.0] * len(rays))
            got = locate.fit_over(rays, [1.0] * len(rays),
                                  (place["x_m"], place["y_m"]), extent)
            if got is None:
                continue
            moved = max(moved, math.hypot(got["x_m"] - place["x_m"],
                                          got["y_m"] - place["y_m"]))
            got.update(id=place["id"], extent_m=extent, share=place.get("share", 1.0),
                       exemplars=sorted(set(place["exemplars"]) | set(claimed.tolist())),
                       claimers=claimed.tolist(), absorbed=place.get("absorbed", []))
            fitted.append(got)
        places = _merge(session, fitted, lookalike)
        if moved < cluster.SETTLED_M:
            break
    # Backward selection, as cluster.discover does it.
    while places:
        weights = session.weigh(places)
        if shares:
            _share(places, weights)
        failing = [(place.get("share", 1.0), column) for column, place in enumerate(places)
                   if not _stands(session, place, np.where(weights[:, column] > 0)[0])]
        if not failing:
            break
        places.pop(min(failing)[1])
    return places, session.weigh(places), rounds


def _tables(session, places, weights, map_session):
    """The answer in the shape `replay.score` reads."""
    import numpy as np

    entities, owner = [], {}
    for column, place in enumerate(places):
        claimed = np.where(weights[:, column] > 0)[0]
        if not len(claimed):
            continue
        entities.append({"id": place["id"], "placement_map_session": map_session,
                         "placement_json": json.dumps({
                             "x_m": place["x_m"], "y_m": place["y_m"],
                             "uncertainty_m": place["error_major_m"]})})
        for index in claimed:
            owner[session.ray(index)["observation_id"]] = place["id"]
    observations = []
    for row in session.rows:
        one = {key: row.get(key) for key in (
            "id", "inference_id", "bearing_deg", "span_deg", "elevation_deg",
            "observer_pose_json", "map_session", "dino_blob")}
        one["entity_id"] = owner.get(row["id"])
        observations.append(one)
    return entities, observations


def neighbours(entities, observations, *, radius_m=resolve.SAME_PLACE_M,
               join=replay_module.JOIN, best=False):
    """Placed things standing together, never in one picture, that look alike."""
    import numpy as np

    held: dict = {}
    for one in observations:
        if one["entity_id"]:
            held.setdefault(one["entity_id"], []).append(one)
    placed = []
    for entity in entities:
        where = json.loads(entity["placement_json"] or "null")
        if not where or entity["id"] not in held:
            continue
        vectors = [v for v in (replay_module._unit(one["dino_blob"])
                               for one in held[entity["id"]] if one["dino_blob"])
                   if v is not None]
        placed.append((where["x_m"], where["y_m"], np.array(vectors),
                       {one["inference_id"] for one in held[entity["id"]]}))
    found = 0
    for index, (ax, ay, av, al) in enumerate(placed):
        for bx, by, bv, bl in placed[index + 1:]:
            if (math.hypot(ax - bx, ay - by) > radius_m or al & bl
                    or not len(av) or not len(bv)):
                continue
            cross = av @ bv.T
            if float(cross.max() if best else np.median(cross)) >= join:
                found += 1
    return found


def report(name, entities, observations, watch=()):
    print(f"\n=== {name}")
    replay_module.score(entities, observations)
    got = misses(entities, observations)
    if got:
        print(f"  how far an attached bearing actually misses:  "
              f"median {statistics.median(got):.2f}  "
              f"90th {got[int(len(got) * 0.9)]:.2f}  worst {got[-1]:.2f} deg")
    print(f"  look-alike pairs within {resolve.SAME_PLACE_M} m: "
          f"{neighbours(entities, observations)} by the median crop, "
          f"{neighbours(entities, observations, join=resolve.RECOGNISED, best=True)} "
          f"by the best")
    if watch:
        where: dict = {}
        for one in observations:
            if one["id"] in watch:
                where.setdefault(one["entity_id"], []).append(one["id"])
        print(f"  the watched looks ended in: {where}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("database", help="a world.db copied off the rover")
    parser.add_argument("--session", type=int, required=True,
                        help="the map session to replay and re-solve")
    parser.add_argument("--map", default="",
                        help="a map.json fetched from the nav bridge, for reach")
    parser.add_argument("--watch", default="",
                        help="comma-separated observation ids to follow, "
                             "e.g. the looks of a known duplicate")
    args = parser.parse_args()
    if not os.path.exists(args.database):
        print(f"no such recording: {args.database}", file=sys.stderr)
        return 2
    watch = {int(one) for one in args.watch.split(",") if one.strip()}
    reach = replay_module.reach_from(args.map) if args.map else None
    groups = [group for group in replay_module.inspections(args.database)
              if group[0]["map_session"] == args.session]
    rows = [row for group in groups for row in group]
    print(f"  map session {args.session}: {len(groups)} looks, {len(rows)} regions")

    began = time.time()
    entities, observations = replay_module.replay(args.database, reach=reach,
                                                  groups=groups)
    took = time.time() - began
    # The replay numbers its rows from 1 in the order it inserted them, and
    # the scores and the watch list want the recording's own ids.
    for one in observations:
        one["id"] = rows[one["id"] - 1]["id"]
    report(f"pairs, the deployed resolver look by look ({took:.0f} s)",
           entities, observations, watch)

    session = Session(rows, reach)
    print(f"\n  {len(session.rays)} rays, {int(session.ranged.sum())} of them "
          f"ranged, from {len(session.looks)} looks")
    seeds = []
    for entity in entities:
        where = json.loads(entity["placement_json"] or "null")
        if not where:
            continue
        members = [session.index[one["id"]] for one in observations
                   if one["entity_id"] == entity["id"] and one["id"] in session.index]
        seeds.append({"id": entity["id"], "x_m": where["x_m"], "y_m": where["y_m"],
                      "extent_m": where.get("extent_m") or 0.0,
                      "error_major_m": where.get("error_major_m",
                                                 where.get("uncertainty_m", 0.0)),
                      "exemplars": members})
    size = {seed["id"]: len(seed["exemplars"]) for seed in seeds}

    for name, shares, lookalike in (
            ("whole, as cluster.py: shares re-estimated, merged on distance", True, False),
            ("whole, merged only where they look alike", True, True),
            ("whole, merged where they look alike, shares held equal", False, True)):
        began = time.time()
        places, weights, rounds = consolidate(session, seeds, shares=shares,
                                              lookalike=lookalike)
        took = time.time() - began
        report(f"{name} ({took:.0f} s, {rounds} rounds)",
               *_tables(session, places, weights, args.session), watch)
        alive = {place["id"] for place in places}
        merged = {gone for place in places for gone in place.get("absorbed", [])}
        lost = [one for one in size if one not in alive]
        dropped = sorted((size[one], one) for one in lost if one not in merged)
        print(f"  of the resolver's {len(size)} things: {len(lost) - len(dropped)} "
              f"merged into another, {len(dropped)} dropped, "
              f"{sum(1 for n, _ in dropped if n >= 20)} of those seen 20 times or more")
        print("  merges:", [(place["id"], place["absorbed"]) for place in places
                            if place.get("absorbed")])
    return 0


if __name__ == "__main__":
    sys.exit(main())
