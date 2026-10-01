"""Replay hypothesis checks over a real recording, at a desk.

    python world_state/bench_inspection.py captures/m0-2026-10-01/world-2026-10-01-acceptance.db \\
        --frames captures/m0-2026-10-01/frames

For every claim -- each placed thing in the recording, or each thing in a labels
file crossed from its own looks -- and every look that kept its depth and did
not help make the claim, `hypothesis_check.check` is asked whether that look
shows something where the claim says. Looks that cannot see the place at all
are counted and left out of the listing.

This is what M0a's replay criterion runs on real looks rather than made-up ones:
a claim made of two objects' looks that a later look sees straight through, a
place in the open, a place behind something nearer. **It is a replay of the
check, not of a drive**: the looks were taken where the drive happened to stand,
not from a viewpoint chosen for the claim, and their headings are whatever the
drive believed. A fresh supervised run is what measures how often the answers
are right.

`--shift X,Y` moves every claim by that much in map metres, which turns each real
thing into a place where nothing stands: the absent-target case, read through
real depth.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import sqlite3
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from world_state import hypothesis_check  # noqa: E402

CALIBRATION = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "captures", "2026-09-30-oak-rail",
    "oak-health.json")


def rows_of(db, query, args=()):
    out = []
    for row in db.execute(query, args):
        one = dict(row)
        text = one.get("observer_pose_json")
        one["pose"] = json.loads(text) if text else None
        out.append(one)
    return out


def claims_from_store(db):
    out = []
    for row in db.execute("SELECT id, placement_json FROM entities "
                          "WHERE placement_json IS NOT NULL"):
        place = json.loads(row["placement_json"])
        if place.get("x_m") is None:
            continue
        looks = [r[0] for r in db.execute(
            "SELECT id FROM observations WHERE entity_id = ?", (row["id"],))]
        out.append({"name": row["id"], "verdict": None, "looks": looks,
                    "claim": {k: place.get(k) for k in
                              ("x_m", "y_m", "height_m", "height_sigma_m",
                               "uncertainty_m")}})
    return out


def claims_from_labels(db, path):
    """Each labelled thing, placed where its own looks cross."""
    things = json.load(open(path, encoding="utf-8"))["things"]
    out = []
    for name, thing in things.items():
        looks = rows_of(db, "SELECT * FROM observations WHERE id IN (%s)"
                        % ",".join("?" * len(thing["looks"])),
                        tuple(thing["looks"]))
        looks = [one for one in looks
                 if one["pose"] and one.get("bearing_deg") is not None]
        crossed = _crossing(looks)
        if crossed is None:
            continue
        height = _height(looks, crossed)
        claim = {"x_m": round(crossed[0], 3), "y_m": round(crossed[1], 3),
                 "uncertainty_m": thing.get("uncertainty_m") or 0.3}
        if height is not None:
            claim.update({"height_m": round(height[0], 3),
                          "height_sigma_m": round(height[1], 3)})
        out.append({"name": name, "verdict": thing.get("verdict"),
                    "what": thing.get("what"),
                    "looks": [one["id"] for one in looks], "claim": claim})
    return out


def _crossing(looks):
    import numpy as np

    if len({one["frame_id"] for one in looks}) < 2:
        return None
    a = np.zeros((2, 2))
    b = np.zeros(2)
    for one in looks:
        theta = math.radians(float(one["bearing_deg"]))
        normal = np.array([-math.sin(theta), math.cos(theta)])
        where = np.array([one["pose"]["x_m"], one["pose"]["y_m"]])
        a += np.outer(normal, normal)
        b += np.outer(normal, normal) @ where
    if np.linalg.cond(a) > 1e4:
        return None
    return tuple(float(v) for v in np.linalg.solve(a, b))


def _height(looks, place):
    rises = []
    for one in looks:
        if one.get("elevation_deg") is None:
            continue
        across = math.hypot(place[0] - one["pose"]["x_m"],
                            place[1] - one["pose"]["y_m"])
        rises.append(across * math.tan(math.radians(float(one["elevation_deg"]))))
    if len(rises) < 2:
        return None
    rises.sort()
    middle = rises[len(rises) // 2]
    spread = (rises[-1] - rises[0]) / 2.0
    return middle, max(0.05, min(spread, 0.5))


def depth_of(frames, frame_id):
    import gzip

    try:
        with gzip.open(os.path.join(frames, frame_id + ".depth.gz"), "rb") as f:
            body = f.read()
    except OSError:
        return None
    size = int.from_bytes(body[:4], "little")
    return body[4 + size:], json.loads(body[4:4 + size])


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("db")
    parser.add_argument("--frames", required=True)
    parser.add_argument("--labels", help="claims from a labels file, crossed "
                                         "from each thing's own looks")
    parser.add_argument("--lens", default=CALIBRATION)
    parser.add_argument("--shift", default="",
                        help="move every claim by X,Y metres: the absent case")
    parser.add_argument("--only", default="", help="comma-separated claims")
    parser.add_argument("--json", help="write every verdict here")
    args = parser.parse_args(argv)

    lens = SimpleNamespace(**json.load(open(args.lens))["colour"]["intrinsics"])
    db = sqlite3.connect(f"file:{args.db}?mode=ro", uri=True)
    db.row_factory = sqlite3.Row
    claims = (claims_from_labels(db, args.labels) if args.labels
              else claims_from_store(db))
    if args.only:
        wanted = set(args.only.split(","))
        claims = [one for one in claims if one["name"] in wanted]
    shift = (tuple(float(v) for v in args.shift.split(","))
             if args.shift else (0.0, 0.0))
    frames = sorted(name[:-len(".depth.gz")] for name in os.listdir(args.frames)
                    if name.endswith(".depth.gz"))
    looks = {}
    for frame_id in frames:
        found = rows_of(db, "SELECT * FROM observations WHERE frame_id = ?",
                        (frame_id,))
        if found:
            looks[frame_id] = found

    tally = collections.Counter()
    out = []
    for one in claims:
        claim = dict(one["claim"])
        claim["x_m"] = round(claim["x_m"] + shift[0], 3)
        claim["y_m"] = round(claim["y_m"] + shift[1], 3)
        own = set(one["looks"])
        source = rows_of(db, "SELECT * FROM observations WHERE id IN (%s)"
                         % ",".join("?" * len(own)), tuple(own)) if own else []
        for frame_id, look in looks.items():
            if own & {row["id"] for row in look}:
                continue
            got = hypothesis_check.check(look, claim, depth_of(args.frames,
                                                               frame_id),
                                         lens, source=source)
            tally[(one.get("verdict"), got["outcome"], got["code"])] += 1
            if got["code"] in ("outside", "direction", "range", "too loose"):
                continue
            entry = {"claim": one["name"], "verdict": one.get("verdict"),
                     "what": one.get("what"), "frame_id": frame_id,
                     "outcome": got["outcome"], "code": got["code"],
                     "why": got["why"],
                     "distance_m": got["evidence"].get("distance_m"),
                     "depth": got["evidence"].get("depth")}
            out.append(entry)
            print(f"{one['name']:11s} {str(one.get('verdict') or ''):8s} "
                  f"{frame_id}  {got['outcome']:12s} {got['code']:12s} "
                  f"{got['evidence'].get('distance_m')} m  {got['why'][:80]}")
    print()
    for key, count in sorted(tally.items(), key=lambda kv: str(kv[0])):
        print(key, count)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump({"claims": len(claims), "frames": len(looks),
                       "shift": shift, "verdicts": out,
                       "tally": {" ".join(map(str, k)): v
                                 for k, v in tally.items()}}, handle, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
