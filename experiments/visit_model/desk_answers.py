"""The multi-day benchmark's questions put here to a filing of every picture up
to the end of each benchmark day, as find_thing puts them on the rover
(experiments/multiday/answer.py, `ask`): every record ranked by its average
look against the phrase, near-ties within NEAR_TIE ordered standing first and
newest first, the first over the floor answering. Right within the
benchmark's 1.0 m; an answer from a record seen gone or set aside is
unresolved, as score.py counts it.

    python desk_answers.py LOOKS.npz PHRASES.json STORE.db RECORDING ...

The model is this directory's, with whatever switches the environment sets.
PHRASES is {question: base64 vector} from the rover's sidecar. STORE is a
SQLite copy of the rover's store, which holds each look's vector.

It answers after all of each day's pictures, both arms together, not each arm
alone as the benchmark did: a way to compare filings, not a rerun of the
benchmark.
"""
import base64
import json
import math
import os
import shutil
import sqlite3
import sys
import tempfile
import threading
import time

import data
import model
import recorded
import truth

sys.path.insert(0, truth.MAIN)
from world_state import search, visit_things  # noqa: E402

BENCH = json.load(open(os.path.join(truth.MAIN, "experiments", "multiday", "benchmark.json")))
NEAR_TIE = 0.01
#: The end of each day's session record (tmp/md-days/md-dayN-*.json) and five
#: minutes for its last visit to close.
DAY_ENDS = {"1": 1792377243.3 + 300, "2": 1792462840.1 + 300}
#: The filling of the store the rover's records were of on those days.
FILLING = {"map_session": 67, "map_id": "7da19bef3888", "store_generation": "07dec15b41240130"}


class Store:
    """What visit_things.VisitStore reads of the world store (answer.Store)."""

    def __init__(self, directory, payload, db_path):
        self.dir = directory
        self.db = sqlite3.connect("file:%s?mode=ro" % db_path, uri=True, check_same_thread=False)
        self._lock = threading.Lock()
        self._p = payload

    def map_session(self):
        return self._p["map_session"]

    def map_id(self):
        return self._p["map_id"]

    def generation(self):
        return self._p["store_generation"]


def things(filing, mem):
    """The records as the rover's job writes them (job.things)."""
    members = {}
    for rid, oid in filing.items():
        members.setdefault(mem.resolve(oid), []).append(int(rid))
    out = []
    for o in mem.objects:
        ids = sorted(members.get(o["id"], []))
        if not ids:
            continue
        out.append({"id": o["id"], "x_m": round(float(o["x"]), 3), "y_m": round(float(o["y"]), 3),
                    "spread_m": round(math.sqrt(float(o["var"])), 3), "extent_m": round(float(o["extent"]), 3),
                    "height_m": None if not math.isfinite(float(o["z"])) else round(float(o["z"]), 3),
                    "state": o["state"], "seen_at": o["seen"], "since": o["since"], "gone_at": o["gone_at"],
                    "visits": len(o["visits"]), "looked_for": o.get("looked_for", 0),
                    "path": o["path"], "observations": ids})
    return out


def ask(records_payload, db_path, vectors):
    payload = {"generation": "desk", **FILLING, "written_at": time.time(), "things": records_payload}
    tmp = tempfile.mkdtemp()
    try:
        os.makedirs(os.path.join(tmp, "visit_model"))
        with open(os.path.join(tmp, visit_things.THINGS_FILE), "w") as f:
            json.dump(payload, f)
        records = visit_things.VisitStore(Store(tmp, payload, db_path))
        if records.fresh():
            raise SystemExit("the records cannot answer: %s" % records.fresh())
        index = search.ThingIndex(records)
        out = {}
        for question, vector in vectors.items():
            answer = search.rank_things(vector, index)
            matches = answer.get("matches") or []
            if matches:
                best = float(matches[0]["score"])
                near = [m for m in matches if float(m["score"]) >= max(answer["floor"], best - NEAR_TIE)]
                rest = [m for m in matches if m not in near]

                def key(m):
                    ent = records.entity(m["entity_id"]) or {}
                    return (2 if ent.get("gone_at") else 1 if ent.get("set_aside") else 0,
                            -float(ent.get("last_seen_at") or 0.0))
                matches = sorted(near, key=key) + rest
            got = None
            for m in matches:
                if float(m["score"]) < answer["floor"]:
                    break
                ent = records.entity(m["entity_id"])
                if ent is None:
                    continue
                p = ent["placement"]
                got = {"record": m["entity_id"], "score": round(float(m["score"]), 4),
                       "at": [round(p["x_m"], 2), round(p["y_m"], 2)],
                       "gone": bool(ent.get("gone_at")), "set_aside": bool(ent.get("set_aside"))}
                break
            out[question] = got
        return out
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def truth_of(day, question):
    if question in BENCH["items"]:
        return BENCH["spots"][BENCH["days"][day]["put"][question]]["at"]
    return BENCH["controls"][question]["at"]


def main():
    a = data.load(sys.argv[1])
    vectors = {k: base64.b64decode(v) for k, v in json.load(open(sys.argv[2])).items()}
    db_path = sys.argv[3]
    recs = [recorded.Recording(p) for p in sys.argv[4:]]
    tally = {"right": 0, "wrong": 0, "unresolved": 0}
    for day, end in DAY_ENDS.items():
        filing, mem, _ = model.run(model._until(a, end), recs)
        answers = ask(things(filing, mem), db_path, vectors)
        print("day %s:" % day)
        for question, got in answers.items():
            want = truth_of(day, question)
            if got is None or got["gone"] or got["set_aside"]:
                verdict = "unresolved"
            else:
                verdict = "right" if math.hypot(got["at"][0] - want[0], got["at"][1] - want[1]) <= BENCH["correct_m"] else "wrong"
            tally[verdict] += 1
            print("  %-28s %-10s %s" % (question, verdict, got and "%s at (%.2f, %.2f) %.3f%s" % (
                got["record"], got["at"][0], got["at"][1], got["score"],
                " gone" if got["gone"] else " set aside" if got["set_aside"] else "")))
    print("in all: %s" % tally)


if __name__ == "__main__":
    main()
