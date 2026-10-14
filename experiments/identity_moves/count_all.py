"""Criterion 4 on the shakedown and M5 sessions 1 and 2, from replayed stores.

    python count_all.py NAME=STORE.db ...

Run from the repository root of the main checkout (it reads the sessions'
records and labels from tmp/ and experiments/). Prints, for each store and
session, the trials kept, split and unresolved and any record holding two
labelled things, by experiments/m5_changes/identity_count.py's rules -- and,
stricter than those, any record taking regions of two labelled things during
the session however they were filed, so that a region moved with a record
counts as a match would.

With --across, a thing's record "before" is any record holding a labelled
picture of the same physical object from an earlier session too (OBJECTS: the
armchair by the charger on the shakedown is the one with the throw, told by
its photographs; the one by the balcony is the plain one), passed to the count
as `before_records`.
"""
import json
import os
import sqlite3
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SESSIONS = [
    ("shakedown", "tmp/ww-recordings/shakedown-1012.json",
     "experiments/m5_changes/staged/shakedown-1012.json",
     os.path.join(HERE, "labels-shakedown.json")),
    ("session 1", "tmp/session1-1014/session1-1014.json",
     "experiments/m5_changes/staged/session1-1014.json",
     "experiments/m5_changes/labels-session1-1014.json"),
    ("session 2", "tmp/session2-1014/session2-1014.json",
     "experiments/m5_changes/staged/session2-1014.json",
     "experiments/m5_changes/labels-session2-1014.json"),
]


#: Which physical object each session's labelled thing is.
OBJECTS = {
    "black armchair by the charger": "throw", "black armchair by the balcony": "plain",
    "red bean bag": "red bean bag", "black bean bag": "black bean bag",
    "black armchair, to the balcony door": "plain", "black armchair, to the black cabinet": "throw",
    "plain black armchair, toward wall A": "plain", "black cabinet, along wall A": "cabinet",
    "black armchair with the throw (not moved in session 2)": "throw",
}


def earlier_records(db, upto):
    """Per physical object, the records holding its labelled pictures from the
    sessions before index `upto` in SESSIONS."""
    con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    out = {}
    for _name, _record, _staged, labels in SESSIONS[:upto]:
        for thing, ids in json.load(open(labels))["things"].items():
            for rid in ids:
                row = con.execute("select entity_id from observations where id = ?", (rid,)).fetchone()
                if row and row[0]:
                    out.setdefault(OBJECTS[thing], set()).add(row[0])
    con.close()
    return out


def main():
    across = "--across" in sys.argv
    for arg in [a for a in sys.argv[1:] if a != "--across"]:
        name, db = arg.split("=", 1)
        total = {"kept": 0, "split": 0, "unresolved": 0, "merges": 0}
        print("==", name)
        for k, (session, record, staged, labels) in enumerate(SESSIONS):
            used = labels
            if across:
                before = earlier_records(db, k)
                lab = json.load(open(labels))
                lab["before_records"] = {t: sorted(before.get(OBJECTS[t], ())) for t in lab["things"]}
                handle, used = tempfile.mkstemp(suffix=".json")
                with os.fdopen(handle, "w") as f:
                    json.dump(lab, f)
            out = subprocess.run([sys.executable, "experiments/m5_changes/identity_count.py",
                                  record, staged, db, used], capture_output=True, text=True)
            lines = out.stdout.strip().splitlines()
            if not lines:
                print("  ", session, "failed:", out.stderr[-300:])
                continue
            result = json.loads(lines[-1])
            for t in result["trials"]:
                total[t["outcome"]] += 1
                print("   %-9s %-32s %-10s before %s after %s, %d unassigned" % (
                    session, t["thing"][:32], t["outcome"], t["before"], t["after"],
                    t["after_unassigned"]))
            for m in result["wrong_merges"]:
                total["merges"] += 1
                print("   %-9s wrong merge: %s holds %s" % (
                    session, m["record"], {k: len(v) for k, v in m["regions"].items()}))
            things = json.load(open(labels))["things"]
            entries = json.load(open(record))["entries"]
            staged_at = next(e["at"] for e in entries if e["step"] == "wait" and e.get("end"))
            con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
            held = {}
            for thing, ids in things.items():
                for rid in ids:
                    row = con.execute("select entity_id, observed_at from observations"
                                      " where id = ?", (rid,)).fetchone()
                    if row and row[0] and row[1] >= staged_at:
                        held.setdefault(row[0], {}).setdefault(thing, 0)
                        held[row[0]][thing] += 1
            con.close()
            for record_id, by in sorted(held.items()):
                if len(by) > 1 and record_id not in [m["record"] for m in result["wrong_merges"]]:
                    total["any_merges"] = total.get("any_merges", 0) + 1
                    print("   %-9s holds two things however filed: %s %s" % (session, record_id, by))
        total.setdefault("any_merges", 0)
        print("   total: kept %(kept)d, split %(split)d, unresolved %(unresolved)d;"
              " wrong merges %(merges)d, and %(any_merges)d more however filed" % total)


if __name__ == "__main__":
    main()
