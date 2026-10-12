"""Criterion 4 counted from labelled crops, by the rules declared before the
sessions (docs/decisions/m5-sessions-are-counted-before-they-start.md).

    python identity_count.py RECORD.json STAGED.json WORLD.db LABELS.json
        [--bad HH:MM:SS-HH:MM:SS ...]

LABELS is {"things": {thing: [region ids]}, "before_records": {thing: [record
ids]}}: the regions that show each staged thing, labelled from
identity_sheets.py's sheets, and optionally the records that held a thing before
the session when its history is known from elsewhere (an earlier labelled
review). Region memberships are read from WORLD, a SQLite backup of the store
taken after the session.

Per staged move: a trial when the thing had at least two regions assigned before
the move (or named records); kept when the record holding most of its assigned
regions after the move held it before; split when not; unresolved with fewer than
two assigned after. Regions in a --bad window are left out and listed. Any record
holding regions of two things, taken as ordinary matches, is a confident wrong
merge, which fails the criterion.
"""
import argparse
import json
import math
import sqlite3
import time
from collections import Counter

#: The resolver's note on an ordinary match begins so (world_state/locate.py).
MATCH_NOTE = "the bearing points at"


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return (max(0.0, centre - half), min(1.0, centre + half))


def main():
    p = argparse.ArgumentParser()
    for name in ("record", "staged", "world", "labels"):
        p.add_argument(name)
    p.add_argument("--bad", action="append", default=[])
    a = p.parse_args()
    record, staged = json.load(open(a.record)), json.load(open(a.staged))
    labels = json.load(open(a.labels))
    entries = record["entries"]
    day = time.localtime(entries[0]["at"])
    clock = lambda s: time.mktime(day[:3] + tuple(int(v) for v in s.split(":")) + (0, 0, -1))
    staged_at = next(e["at"] for e in entries if e["step"] == "wait" and e.get("end"))
    bad = [tuple(clock(s) for s in w.split("-")) for w in a.bad]
    db = sqlite3.connect("file:%s?mode=ro" % a.world, uri=True)

    regions = {}
    for thing, ids in labels["things"].items():
        for rid in ids:
            row = db.execute("select entity_id, observed_at, note from observations where id = ?",
                             (rid,)).fetchone()
            if row is None:
                continue
            regions[rid] = {"thing": thing, "record": row[0], "at": row[1], "note": row[2] or "",
                            "bad": any(lo <= row[1] <= hi for lo, hi in bad)}

    movers = [m["what"] for m in staged.get("moves", [])]
    movers += [m["what"] for m in staged.get("also_moved", []) if m["what"] not in movers]
    out = {"trials": [], "not_trials": [], "wrong_merges": [], "left_out_bad_pose": 0}
    for thing in labels["things"]:
        mine = [r for r in regions.values() if r["thing"] == thing]
        later = next((m for m in staged.get("also_moved", []) if m["what"] == thing), None)
        moved_at = staged_at
        if thing not in [m["what"] for m in staged.get("moves", [])] and later:
            # Moved during the session, not in the staging: its move is the gap
            # in its sightings the counts were told of, "before" the step named.
            step = later.get("before")
            moved_at = next((e["at"] for e in entries if e["step"] == step and e.get("begin")),
                            staged_at)
            if thing in [x["what"] for x in staged.get("added", [])]:
                mine_before = [r for r in mine if staged_at <= r["at"] < moved_at]
            else:
                mine_before = [r for r in mine if r["at"] < moved_at]
        else:
            mine_before = [r for r in mine if r["at"] < moved_at]
        mine_after = [r for r in mine if r["at"] >= moved_at]
        out["left_out_bad_pose"] += sum(r["bad"] for r in mine)
        before = Counter(r["record"] for r in mine_before if r["record"] and not r["bad"])
        named = set(labels.get("before_records", {}).get(thing, []))
        after = Counter(r["record"] for r in mine_after if r["record"] and not r["bad"])
        row = {"thing": thing, "before": dict(before), "named_before": sorted(named),
               "after": dict(after),
               "after_unassigned": sum(1 for r in mine_after if not r["record"] and not r["bad"])}
        if thing not in movers or (sum(before.values()) < 2 and not named):
            row["why"] = ("not a staged move" if thing not in movers
                          else "fewer than two regions assigned before the move")
            out["not_trials"].append(row)
            continue
        if sum(after.values()) < 2:
            row["outcome"] = "unresolved"
        else:
            top = after.most_common(1)[0][0]
            row["outcome"] = "kept" if (top in before or top in named) else "split"
            row["record_after"] = top
        out["trials"].append(row)

    held = {}
    for rid, r in regions.items():
        if r["record"] and r["at"] >= staged_at and MATCH_NOTE in r["note"]:
            held.setdefault(r["record"], {}).setdefault(r["thing"], []).append(rid)
    for record_id, things in sorted(held.items()):
        if len(things) > 1:
            out["wrong_merges"].append({"record": record_id, "regions": things})

    kept = sum(t["outcome"] == "kept" for t in out["trials"])
    n = len(out["trials"])
    lo, hi = wilson(kept, n)
    for t in out["trials"]:
        print("%-32s %-10s before %s%s  after %s, %d unassigned" % (
            t["thing"][:32], t["outcome"], t["before"],
            (" named %s" % t["named_before"]) if t["named_before"] else "", t["after"],
            t["after_unassigned"]))
    for t in out["not_trials"]:
        print("%-32s not a trial: %s; after %s" % (t["thing"][:32], t["why"], t["after"]))
    for m in out["wrong_merges"]:
        print("confident wrong merge: %s holds %s" % (m["record"], {k: v for k, v in m["regions"].items()}))
    print("kept %d of %d trials (95%%: %.0f-%.0f%%); %d split, %d unresolved; %d wrong merges%s; %d regions left out for a bad pose"
          % (kept, n, 100 * lo, 100 * hi, sum(t["outcome"] == "split" for t in out["trials"]),
             sum(t["outcome"] == "unresolved" for t in out["trials"]), len(out["wrong_merges"]),
             " -- criterion 4 fails" if out["wrong_merges"] else "", out["left_out_bad_pose"]))
    print(json.dumps(out))


if __name__ == "__main__":
    main()
