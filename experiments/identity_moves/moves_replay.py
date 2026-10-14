"""The rover's world from the 10-11 snapshot, with the looks since fed back
through the live resolver and step 3's moved events acted on the minute they
would have been known (relocate.py).

    python moves_replay.py SNAPSHOT.db LATER.db GRID.json OUT.db
        --moves MOVES.json ... [--lidar REC.npz ...] [--no-moves]

Run from the repository root. MOVES are timeline.py's outputs. --no-moves feeds
the same looks without acting on the events, the baseline. OUT.db is the
replayed store, for experiments/m5_changes/identity_count.py. Each decision is
printed with its reason.
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
sys.path.insert(0, os.path.join(HERE, "..", "identity_lidar"))
sys.path.insert(0, os.path.join(HERE, "..", "what_where"))

from world_state import replay as wsreplay  # noqa: E402
from world_state import resolve as resolver  # noqa: E402
from world_state.store import WorldStore  # noqa: E402

import relocate  # noqa: E402
import replay_identity  # noqa: E402
import warm_replay  # noqa: E402


def main():
    a = argparse.ArgumentParser()
    a.add_argument("snapshot")
    a.add_argument("later")
    a.add_argument("grid")
    a.add_argument("out")
    a.add_argument("--moves", nargs="*", default=[])
    a.add_argument("--lidar", nargs="*", default=[])
    a.add_argument("--no-moves", action="store_true")
    a.add_argument("--session", type=int, default=67)
    args = a.parse_args()
    last, groups = warm_replay.later_looks(args.snapshot, args.later, args.session)
    print("looks", len(groups), "regions", sum(len(g) for g in groups), flush=True)
    if args.lidar:
        print("lidar ranges added:", replay_identity.add_lidar(groups, args.later, args.lidar),
              flush=True)
    moves = sorted((e for path in args.moves for e in json.load(open(path))
                    if e["kind"] == "moved"), key=lambda e: e["known_at"])
    print("moved events", len(moves), "acted on" if not args.no_moves else "not acted on",
          flush=True)
    resolver._replace_placement = relocate.refit_since(resolver._replace_placement)
    reach = wsreplay.reach_from(args.grid)
    directory = tempfile.mkdtemp(prefix="world-moves-")
    decisions = []
    try:
        shutil.copy(args.snapshot, os.path.join(directory, "world.db"))
        store = WorldStore(directory)
        store.db.execute("REPLACE INTO meta(key, value) VALUES('map_session', ?)",
                         (str(args.session),))
        store.db.commit()
        columns = ("id",) + tuple(wsreplay.COLUMNS)
        pending = list(moves)
        started = time.time()
        for n, group in enumerate(groups):
            at = group[0]["observed_at"]
            while pending and pending[0]["known_at"] <= at:
                e = pending.pop(0)
                if args.no_moves:
                    continue
                done, why = relocate.relocate(store, e, args.session, e["known_at"])
                decisions.append({"from": e["from"], "to": e["to"], "known_at": e["known_at"],
                                  "session": e.get("session"), "done": done, "why": why})
                print("  %s %s -> %s: %s, %s" % (time.strftime("%m-%d %H:%M", time.localtime(
                    e["known_at"])), e["from"], e["to"], done, why), flush=True)
            with store._lock, store.db:
                for row in group:
                    store.db.execute(
                        "INSERT INTO observations(entity_id, " + ",".join(columns)
                        + ") VALUES(NULL, " + ",".join("?" * len(columns)) + ")",
                        tuple(row.get(name) for name in columns))
            resolver.resolve(store, reach=reach)
            if n % 50 == 0:
                print(f"  {n} of {len(groups)} looks, {time.time() - started:.0f} s", flush=True)
        kept = sqlite3.connect(args.out)
        store.db.backup(kept)
        kept.close()
        print("records", store.db.execute("SELECT COUNT(*) FROM entities").fetchone()[0],
              "regions filed", store.db.execute(
                  "SELECT COUNT(*) FROM observations WHERE id > ? AND entity_id IS NOT NULL",
                  (last,)).fetchone()[0], "of", sum(len(g) for g in groups))
        store.close()
    finally:
        shutil.rmtree(directory, ignore_errors=True)
    json.dump(decisions, open(args.out + ".decisions.json", "w"), indent=1)


if __name__ == "__main__":
    main()
