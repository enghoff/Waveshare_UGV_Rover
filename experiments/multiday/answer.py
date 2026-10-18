"""The day's questions, answered from the world model as each session alone
would have left it. On the rover, from ~/multiday, after day.py run NAME.

    python3 answer.py NAME [OUT.json]
    python3 answer.py --check            the plumbing: the live world model, filed
                                          on with nothing added, answers as find_thing does

Three states of the world model, each filed by the rover's own engine
(world_state/visit_model) from the morning copy day.py kept (NAME-start/):
`base`, with nothing added; `A`, with the autonomy arm's pictures alone; `B`,
with the fixed loop's alone, each from its window in the session's record, with
the lidar the model had that day (NAME-lidar/). So the arm that drove second
does not inherit what the first saw. Each question is put as find_thing puts
it (rover_daemon/rover_recall.py, `_recall`): the phrase's SigLIP2 vector from
the perception sidecar, every record ranked by its average look
(world_state/search.py), near-ties ordered present and recent first, the first
over the floor answering -- with the record, its place, and whether it is
standing, set aside or seen gone.
"""
import base64
import http.client
import json
import os
import pickle
import shutil
import sqlite3
import sys
import tempfile
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
UGV = os.path.expanduser("~/ugv")
sys.path.insert(0, os.path.join(UGV, "world_state", "visit_model"))
sys.path.insert(0, UGV)
import engine  # noqa: E402
import job  # noqa: E402
import model  # noqa: E402
import recorded  # noqa: E402
from world_state import search, visit_things  # noqa: E402

BENCH = json.load(open(os.path.join(HERE, "benchmark.json")))
SESSIONS = os.path.expanduser("~/.ugv/sessions")
DB = job.DB
STATE = job.STATE
#: rover_daemon/rover_recall.py
NEAR_TIE = 0.01


def phrases():
    out = dict(BENCH["items"])
    out.update({k: v["phrase"] for k, v in BENCH["controls"].items()})
    return out


def embed(texts):
    """Each phrase as the daemon embeds it: lower-cased, through the sidecar."""
    c = http.client.HTTPConnection("127.0.0.1", 8776, timeout=60)
    c.request("POST", "/embed", body=json.dumps({"phrases": [t.lower() for t in texts]}).encode(),
              headers={"Content-Type": "application/json"})
    r = json.loads(c.getresponse().read())
    if not r.get("ok"):
        raise SystemExit("the sidecar would not embed: %s" % r.get("error"))
    return [base64.b64decode(v) for v in r["vectors"]]


class Store:
    """What visit_things.VisitStore reads of the world store: the records file
    in its directory, the database read-only, and which filling it is."""

    def __init__(self, directory, payload):
        self.dir = directory
        self.db = sqlite3.connect("file:%s?mode=ro" % DB, uri=True, check_same_thread=False)
        self._lock = threading.Lock()
        self._p = payload

    def map_session(self):
        return self._p["map_session"]

    def map_id(self):
        return self._p["map_id"]

    def generation(self):
        return self._p["store_generation"]


def lidar(directory):
    """The lidar files day.py kept, as one recording's arrays (job.lidar
    without its half-hour window)."""
    parts = []
    for name in sorted(n for n in os.listdir(directory) if n.endswith(".npz")):
        try:
            with np.load(os.path.join(directory, name)) as d:
                parts.append(job.clean({k: d[k] for k in ("ranges", "scan_meta", "odom_tf", "map_tf")}))
        except Exception:  # noqa: BLE001 - as job.lidar: one bad file is skipped
            continue
    if not parts:
        return None
    width = max(p["ranges"].shape[1] for p in parts)
    return {"ranges": np.concatenate([np.pad(p["ranges"], ((0, 0), (0, width - p["ranges"].shape[1])),
                                             constant_values=np.nan) for p in parts]),
            "scan_meta": np.concatenate([p["scan_meta"] for p in parts]),
            "odom_tf": np.concatenate([p["odom_tf"] for p in parts]),
            "map_tf": np.concatenate([p["map_tf"] for p in parts])}


def file_onto(snapshot, window, arrays):
    """The morning copy with the regions taken inside `window` filed on, every
    visit closed. window None files only what the copy had waiting."""
    e = engine.Engine.load(snapshot)
    recs = [recorded.Recording(arrays=arrays)] if arrays is not None else []
    a = engine.prepare(DB, recs, after_id=e.last_id, session=e.filling["map_session"],
                       wall_cells=job.walls_for(e, STATE, lambda *_: None))
    if len(a["id"]):
        keep = a["t"] >= job.CLOCK_SET_AFTER
        if window is None:
            keep &= False
        else:
            # By when the shutter opened: a region is stored a second or so
            # after, which put the last look of an arm outside its window.
            keep &= (a["shutter"] >= window[0]) & (a["shutter"] <= window[1])
        e.add({k: v[keep] for k, v in a.items()})
    decisions = []
    end = (window[1] if window else time.time()) + 10 * model.WINDOW_S
    filed = e.step(end, recs, decisions)
    return e, decisions, filed


def ask(e, vectors):
    """{question: answer} as find_thing would give it from this engine's records."""
    payload = {"generation": e.generation, "map_session": e.filling["map_session"],
               "map_id": e.filling["map_id"], "store_generation": e.filling["store_generation"],
               "written_at": time.time(), "things": job.things(e)}
    tmp = tempfile.mkdtemp()
    try:
        os.makedirs(os.path.join(tmp, "visit_model"))
        with open(os.path.join(tmp, visit_things.THINGS_FILE), "w") as f:
            json.dump(payload, f)
        records = visit_things.VisitStore(Store(tmp, payload))
        if records.fresh():
            raise SystemExit("the replayed records cannot answer: %s" % records.fresh())
        index = search.ThingIndex(records)
        out = {}
        for question, vector in vectors.items():
            answer = search.rank_things(vector, index)
            matches = answer.get("matches") or []
            if matches:
                best = float(matches[0]["score"])
                floor = answer["floor"]
                near = [m for m in matches if float(m["score"]) >= max(floor, best - NEAR_TIE)]
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
                       "gone": bool(ent.get("gone_at")), "set_aside": bool(ent.get("set_aside")),
                       "last_seen_at": ent.get("last_seen_at")}
                break
            out[question] = got
        return out
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def chosen_goals(t0, t1):
    """The goals autonomous turns chose and set off on between t0 and t1, from
    the episode store (as experiments/m10_outcomes/inventory.py reads it)."""
    db = sqlite3.connect("file:%s?mode=ro" % os.path.expanduser("~/.ugv/autonomy/episodes.db"), uri=True)
    try:
        out = []
        for (body,) in db.execute("select body_json from events where kind = 'decision' and at between ? and ?",
                                  (t0, t1)):
            chose = (json.loads(body) or {}).get("chose")
            if chose:
                out.append(chose)
        return out
    finally:
        db.close()


def stale_targets(name, window):
    """The plan's rule that stale coordinates cannot drive movement: every goal
    the autonomy arm chose, against the morning's records -- none may be at a
    record seen gone or set aside as not found again."""
    with open(os.path.join(SESSIONS, name + "-start", "things.json")) as f:
        payload = json.load(f)
    payload["written_at"] = time.time()
    tmp = tempfile.mkdtemp()
    try:
        os.makedirs(os.path.join(tmp, "visit_model"))
        with open(os.path.join(tmp, visit_things.THINGS_FILE), "w") as f:
            json.dump(payload, f)
        records = visit_things.VisitStore(Store(tmp, payload))
        records.load()
        goals, stale = chosen_goals(*window), []
        for goal in goals:
            target = goal.split(":", 1)[1].rsplit("@", 1)[0] if ":" in goal else None
            ent = records.entity(target) if target else None
            if ent is not None and (ent.get("gone_at") or ent.get("set_aside")):
                stale.append(goal)
        return {"goals": goals, "at_gone_or_set_aside": stale}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def windows(name):
    rec = json.load(open(os.path.join(SESSIONS, name + ".json")))
    out, began = {}, {}
    for e in rec["entries"]:
        step = e["step"]
        if step.startswith("arm ") and e.get("begin"):
            began[step[4:]] = e["at"]
        if step.startswith("arm ") and e.get("end") and step[4:] in began:
            out[step[4:]] = [began[step[4:]], e["at"]]
    return rec, out


def check():
    """With nothing added, the live engine's records answer as the daemon does."""
    qs = phrases()
    vectors = dict(zip(qs, embed(list(qs.values()))))
    snap = os.path.join(tempfile.mkdtemp(), "engine.pkl")
    shutil.copy2(os.path.join(STATE, "engine.pkl"), snap)
    e, _, _ = file_onto(snap, None, None)
    replayed = ask(e, vectors)
    import socket
    live = {}
    for q, phrase in qs.items():
        s = socket.create_connection(("127.0.0.1", 8769), 60)
        f = s.makefile("rwb")
        f.write((json.dumps({"call": "find_thing", "arguments": {"description": phrase}}) + "\n").encode())
        f.flush()
        r = json.loads(f.readline())
        s.close()
        live[q] = None if not r.get("found") else [r["map_x_m"], r["map_y_m"]]
    same = True
    for q in qs:
        a = None if replayed[q] is None else [round(v, 1) for v in replayed[q]["at"]]
        b = None if live[q] is None else [round(v, 1) for v in live[q]]
        print("%-30s replayed %-16s live %s" % (q, a, b))
        same &= (a == b) or (a is not None and b is not None and max(abs(a[0] - b[0]), abs(a[1] - b[1])) <= 0.1)
    print("the replay answers as find_thing does:", same)
    return same


def main():
    if sys.argv[1:] == ["--check"]:
        sys.exit(0 if check() else 1)
    name = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(SESSIONS, name + "-answers.json")
    rec, wins = windows(name)
    # Every region of the day in the store, read by the live job and none of
    # them waiting: the replay reads the store itself, and this says perception
    # has finished with the day's pictures. (The job's `filed_until` is the
    # newest region filed, which a parked rover taking no picture never moves.)
    with open(os.path.join(STATE, "things.json")) as f:
        live = json.load(f)
    last = max(w[1] for w in wins.values())
    con = sqlite3.connect("file:%s?mode=ro" % DB, uri=True)
    newest = con.execute("select max(id) from observations where observed_at <= ?", (last,)).fetchone()[0] or 0
    con.close()
    if time.time() < last + 2 * job.READ_DELAY_S or (live.get("last_observation") or 0) < newest \
            or live.get("waiting"):
        raise SystemExit("the live world model has not yet read every picture of the day; wait")
    qs = phrases()
    vectors = dict(zip(qs, embed(list(qs.values()))))
    snap = os.path.join(SESSIONS, name + "-start", "engine.pkl")
    arrays = lidar(os.path.join(SESSIONS, name + "-lidar"))
    result = {"name": name, "windows": wins, "states": {}}
    for state, window in [("base", None)] + sorted(wins.items()):
        e, decisions, filed = file_onto(snap, window, arrays)
        result["states"][state] = {"answers": ask(e, vectors), "filed": int(filed),
                                   "decisions": len(decisions)}
        print(state, json.dumps(result["states"][state]["answers"])[:400])
    if "A" in wins:
        result["autonomy_goals"] = stale_targets(name, wins["A"])
    day = next((e for e in rec["entries"] if e["step"] == "day" and "day" in e), {})
    result["day"] = day.get("day")
    result["put"] = day.get("put")
    # What each arm cost and whether anyone had to step in, from the record.
    cost = {}
    for which in wins:
        ends = [e for e in rec["entries"] if e["step"] == "arm " + which]
        b0 = next((e.get("battery") for e in ends if e.get("begin")), None)
        b1 = next((e.get("battery") for e in ends if e.get("end")), None)
        cost[which] = {"minutes": next((e.get("minutes") for e in ends if e.get("end")), None),
                       "battery_used": None if b0 is None or b1 is None else b0 - b1}
    result["cost"] = cost
    result["stops"] = [e for e in rec["entries"] if "stop" in e]
    with open(out_path, "w") as f:
        json.dump(result, f, indent=1)
    print("written", out_path)


if __name__ == "__main__":
    main()
