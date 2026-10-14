"""Does a lidar range on regions the depth camera missed stop a record taking
two identical armchairs? (experiments/identity_lidar/README.md)

    python replay_identity.py WORLD.db LABELS.json OUT.json [--lidar REC.npz ...]
        [--window REC.npz ...]

Run from the repository root. Replays map session 67 of WORLD.db through the
live resolver (world_state/replay.py), as the rover filed it, and reports which
records hold the labelled regions of each physical thing, and any record that
holds two things. With --lidar, a furniture-sized region with a pose and no
depth range first takes the lidar's range across its width at its shutter from
whichever of the given recordings covers it (experiments/what_where/lidar.py),
with LIDAR_SIGMA_M as its uncertainty. A region qualifies as the rover's own
rule has it (world_state/lidar_ranges.py): its lower edge, tilt bias out, no
more than REACH_DEG above the horizontal. --window replays only the looks taken
while the given recordings ran, from an empty world.
"""
import argparse
import json
import math
import os
import sqlite3
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(HERE, "..", "what_where"))

from world_state import locate  # noqa: E402
from world_state import replay as wsreplay  # noqa: E402

#: How far a lidar range may be out: half of them land within 0.3 m of the
#: depth camera's own on the controls of 2026-10-11, one in ten 0.8 m short.
LIDAR_SIGMA_M = 0.4
REACH_DEG = 2.0


def reaches_down(row):
    e = locate.elevation_of(row)
    if e is None:
        return False
    return e - float(row.get("elevation_span_deg") or 0.0) / 2.0 <= REACH_DEG


def add_lidar(groups, db, recordings):
    """Give unranged, posed regions reaching the lidar's plane a lidar range."""
    import lidar
    import recorded
    recs = [recorded.Recording(p) for p in recordings]
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    taken = dict(con.execute("select id, taken_at from frames"))
    added = 0
    for group in groups:
        for row in group:
            if row.get("range_m") is not None or row.get("bearing_deg") is None:
                continue
            pose = row.get("observer_pose_json")
            pose = json.loads(pose) if isinstance(pose, str) else pose
            bbox = row.get("bbox_json")
            bbox = json.loads(bbox) if isinstance(bbox, str) else bbox
            if not pose or not bbox or not reaches_down(row):
                continue
            t = taken.get(row.get("frame_id")) or row.get("observed_at")
            span = row.get("span_deg") or 0.0
            p = lidar.place(recs, t, float(row["bearing_deg"]), float(span))
            if p is None:
                continue
            row["range_m"] = round(math.hypot(p[0] - pose["x_m"], p[1] - pose["y_m"]), 3)
            row["range_sigma_m"] = LIDAR_SIGMA_M
            row["range_from"] = "lidar"
            added += 1
    return added


def main():
    a = argparse.ArgumentParser()
    a.add_argument("world")
    a.add_argument("labels")
    a.add_argument("out")
    a.add_argument("--lidar", nargs="*", default=[])
    a.add_argument("--session", type=int, default=67)
    a.add_argument("--window", nargs="*", default=[])
    args = a.parse_args()
    groups = wsreplay.inspections(args.world)
    groups = [g for g in groups if g[0]["map_session"] == args.session]
    if args.window:
        import recorded
        spans = [(r.scan_t[0], r.scan_t[-1]) for r in map(recorded.Recording, args.window)]
        groups = [g for g in groups
                  if any(lo <= g[0]["observed_at"] <= hi for lo, hi in spans)]
    print("looks", len(groups), "regions", sum(len(g) for g in groups), flush=True)
    if args.lidar:
        print("lidar ranges added:", add_lidar(groups, args.world, args.lidar), flush=True)
    # The replay writes into a fresh store, so a region's id there is its place
    # in the order it was inserted.
    original = [row["id"] for group in groups for row in group]
    entities, observations = wsreplay.replay(args.world, groups=groups)
    new_to_old = {k + 1: oid for k, oid in enumerate(original)}
    labels = json.load(open(args.labels))
    things = labels["things"] if "things" in labels else labels["subjects"]
    by_id = {}
    for o in observations:
        by_id[new_to_old.get(o.get("id"))] = o.get("entity_id")
    held = {}
    out = {"things": {}}
    for thing, ids in things.items():
        recs = {}
        for rid in ids:
            e = by_id.get(rid)
            recs[e or "-"] = recs.get(e or "-", 0) + 1
            if e:
                held.setdefault(e, {}).setdefault(thing, []).append(rid)
        out["things"][thing] = recs
        print("%-40s %s" % (thing[:40], recs), flush=True)
    out["two_things"] = {e: t for e, t in held.items() if len(t) > 1}
    out["records"] = len(entities)
    out["filed"] = sum(1 for o in observations if o.get("entity_id"))
    out["regions"] = len(observations)
    print("records", out["records"], "regions filed", out["filed"], "of", out["regions"])
    for e, t in out["two_things"].items():
        print("record", e, "holds", {k: len(v) for k, v in t.items()})
    json.dump(out, open(args.out, "w"), indent=1)


if __name__ == "__main__":
    main()
