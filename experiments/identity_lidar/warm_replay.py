"""The rover's world as a snapshot left it, with the looks taken since fed back
through the live resolver: does a lidar range stop a record taking both
armchairs? (experiments/identity_lidar/README.md)

    python warm_replay.py SNAPSHOT.db LATER.db LABELS.json GRID.json OUT.json
        [--lidar REC.npz ...]

Run from the repository root. SNAPSHOT.db is a world store from
~/.ugv/snapshots; LATER.db a copy of the live store taken afterwards, whose
observations past the snapshot's last are replayed, look by look, into a copy of
the snapshot -- so the records they meet are the ones the rover had, under
their own ids. GRID.json bounds bearings as the daemon's map does. With --lidar,
an unranged region reaching the lidar's plane first takes the lidar's range
across it from whichever recording covers its shutter (replay_identity.add_lidar).
"""
import argparse
import json
import os
import shutil
import sqlite3
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "what_where"))

from world_state import replay as wsreplay  # noqa: E402
from world_state import resolve as resolver  # noqa: E402
from world_state.store import WorldStore  # noqa: E402

import replay_identity  # noqa: E402


def later_looks(snapshot, later, session):
    con = sqlite3.connect(f"file:{snapshot}?mode=ro", uri=True)
    last = con.execute("SELECT MAX(id) FROM observations").fetchone()[0]
    con.close()
    groups = [g for g in wsreplay.inspections(later)
              if g[0]["map_session"] == session and min(r["id"] for r in g) > last]
    return last, groups


def main():
    a = argparse.ArgumentParser()
    a.add_argument("snapshot")
    a.add_argument("later")
    a.add_argument("labels")
    a.add_argument("grid")
    a.add_argument("out")
    a.add_argument("--lidar", nargs="*", default=[])
    a.add_argument("--session", type=int, default=67)
    args = a.parse_args()
    last, groups = later_looks(args.snapshot, args.later, args.session)
    print("snapshot ends at observation", last, "; looks after it", len(groups),
          "regions", sum(len(g) for g in groups), flush=True)
    if args.lidar:
        print("lidar ranges added:",
              replay_identity.add_lidar(groups, args.later, args.lidar), flush=True)
    reach = wsreplay.reach_from(args.grid)
    directory = tempfile.mkdtemp(prefix="world-warm-")
    try:
        shutil.copy(args.snapshot, os.path.join(directory, "world.db"))
        store = WorldStore(directory)
        store.db.execute("REPLACE INTO meta(key, value) VALUES('map_session', ?)",
                         (str(args.session),))
        store.db.commit()
        columns = ("id",) + tuple(wsreplay.COLUMNS)
        started = time.time()
        for n, group in enumerate(groups):
            with store._lock, store.db:
                for row in group:
                    store.db.execute(
                        "INSERT INTO observations(entity_id, " + ",".join(columns)
                        + ") VALUES(NULL, " + ",".join("?" * len(columns)) + ")",
                        tuple(row.get(name) for name in columns))
            resolver.resolve(store, reach=reach)
            if n % 50 == 0:
                print(f"  {n} of {len(groups)} looks, {time.time() - started:.0f} s",
                      flush=True)
        filed = dict(store.db.execute(
            "SELECT id, entity_id FROM observations WHERE id > ?", (last,)).fetchall())
        notes = dict(store.db.execute(
            "SELECT id, note FROM observations WHERE id > ?", (last,)).fetchall())
        created = store.db.execute(
            "SELECT COUNT(*) FROM entities").fetchone()[0]
        store.close()
    finally:
        shutil.rmtree(directory, ignore_errors=True)
    labels = json.load(open(args.labels))
    things = labels["things"] if "things" in labels else labels["subjects"]
    held = {}
    out = {"things": {}, "records": created, "replayed": len(filed),
           "filed": sum(1 for e in filed.values() if e)}
    for thing, ids in things.items():
        recs = {}
        for rid in ids:
            e = filed.get(rid)
            recs[e or "-"] = recs.get(e or "-", 0) + 1
            if e:
                held.setdefault(e, {}).setdefault(thing, []).append(rid)
        out["things"][thing] = recs
        print("%-40s %s" % (thing[:40], recs), flush=True)
    out["two_things"] = {e: t for e, t in held.items() if len(t) > 1}
    for e, t in out["two_things"].items():
        print("record", e, "holds", {k: len(v) for k, v in t.items()})
        for k, v in t.items():
            for rid in v:
                print("   ", rid, k[:30], (notes.get(rid) or "")[:110])
    print("records", created, "regions filed", out["filed"], "of", out["replayed"])
    json.dump(out, open(args.out, "w"), indent=1)


if __name__ == "__main__":
    main()
