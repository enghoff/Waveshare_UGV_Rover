"""M5's counts for one session, by the rules proposed in
docs/decisions/m5-sessions-are-counted-before-they-start.md.

    python3 score_session.py NAME STAGED.json [--sessions DIR] [--log LOG.jsonl]

NAME's record is `session.py`'s (`~/.ugv/sessions/NAME.json`); the log is the
change watch's (`~/.ugv/changes/<map>.log.jsonl`). STAGED.json is written from
what the owner said was moved, with the places read off the loop's scans and
photographs where the owner's words leave them open:

    {"staged_in": "wait",
     "moves": [{"what": "black armchair", "from": [-18.6, -16.6], "to": [-17.8, -15.3]}],
     "added": [{"what": "pink bucket", "at": [-17.0, -13.8]}],
     "removed": [],
     "doors": [{"what": "bathroom door", "at": [-14.5, -13.4]}],
     "rooms": ["living"],
     "reset_rooms": ["living"]}

`rooms` were touched while the rover waited, `reset_rooms` between this session
and the last; neither is a control on its first visit after.

**Found** is a furniture-sized change of the right kind logged within FOUND_M of
the place, in the first step after the staging whose stops came within the
lidar's reach of it; a smaller one there is **found small**. **A room-control**
is a room a step visited that nobody touched since the rover's last visit; a
furniture-sized change logged in one, not at a door, is a **false alarm**, and it
**held up** if a later step's report still has it.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import control_count  # noqa: E402

FOUND_M = 0.6
#: How near a stop must come to a place for the step to have seen it: the change
#: watch counts the scan within 4.5 m (ros_nav/change_watch.py, TALLY_MAX_M).
SEEN_M = 4.5
#: A change this far from everything staged, in a staged room, is a false alarm.
STAGED_CLEAR_M = 1.0
#: The four doors the owner keeps shut, as marked on 2026-10-11, with 0.2 m
#: round each: a change in one is a door, named and not counted.
DOORS = [((-10.72, -12.59), 0.70, 0.68), ((-12.36, -12.76), 0.69, 0.55),
         ((-13.94, -14.32), 0.77, 0.59), ((-14.52, -13.43), 0.62, 0.52)]
STEPS = ("loop A", "run", "loop B")


def room_of(x, y):
    """Which room a point is in, by the map of 2026-10-11."""
    if x < -15.4:
        return "living"
    if x < -9.8 and y < -12.0:
        return "corridor"
    return "bedroom"


def at_door(x, y):
    return any(abs(x - cx) <= hx and abs(y - cy) <= hy for (cx, cy), hx, hy in DOORS)


def windows(entries):
    """Each step's (name, begin, end, stops, end report), in order."""
    out, open_at = [], {}
    for e in entries:
        step = e["step"]
        if e.get("begin"):
            open_at[step] = {"from": e["at"], "stops": []}
        elif step in open_at:
            pose = e.get("at_pose") or ([e["pose"].get("x_m"), e["pose"].get("y_m")]
                                        if isinstance(e.get("pose"), dict) else None)
            if pose and pose[0] is not None:
                open_at[step]["stops"].append((float(pose[0]), float(pose[1])))
            if e.get("end"):
                w = open_at.pop(step)
                out.append({"step": step, "from": w["from"], "to": e["at"],
                            "stops": w["stops"],
                            "report": (e.get("report") or {}).get("changes") or []})
    return out


def near(rows, kind, x, y, metres=FOUND_M):
    return [r for r in rows if r["kind"] == kind
            and math.hypot(r["x_m"] - x, r["y_m"] - y) <= metres]


def score(record, staged, log_rows):
    steps = windows(record["entries"])
    names = [w["step"] for w in steps]
    after = names.index(staged.get("staged_in", "wait")) + 1 if staged.get("staged_in", "wait") in names else 0
    events = []
    for move in staged.get("moves", []):
        events.append((move["what"], "GONE", tuple(move["from"])))
        events.append((move["what"], "NEW", tuple(move["to"])))
    for one in staged.get("added", []):
        events.append((one["what"], "NEW", tuple(one["at"])))
    for one in staged.get("removed", []):
        events.append((one["what"], "GONE", tuple(one["at"])))
    out = {"events": [], "room_controls": [], "doors": []}
    for what, kind, (x, y) in events:
        result = {"what": what, "kind": kind, "at": [x, y], "outcome": "missed",
                  "step": None, "change": None}
        for w in steps[after:]:
            if w["step"] not in STEPS or not any(
                    math.hypot(sx - x, sy - y) <= SEEN_M for sx, sy in w["stops"]):
                continue
            kept, _ = control_count.counted(
                [r for r in log_rows if w["from"] <= r["logged_at"] <= w["to"]])
            hits = near(kept, kind, x, y)
            big = [r for r in hits if r["large"]]
            result["step"] = w["step"]
            if big or hits:
                best = min(big or hits, key=lambda r: math.hypot(r["x_m"] - x, r["y_m"] - y))
                result["outcome"] = "found" if big else "found small"
                result["change"] = {k: best[k] for k in ("x_m", "y_m", "extent_m")}
            break
        out["events"].append(result)

    touched_after = set(staged.get("rooms", []))
    reset = set(staged.get("reset_rooms", []))
    visited_since = set()
    staged_places = [p for _, _, p in events]
    for i, w in enumerate(steps):
        if w["step"] not in STEPS:
            continue
        rooms = {room_of(x, y) for x, y in w["stops"]}
        kept, _ = control_count.counted(
            [r for r in log_rows if w["from"] <= r["logged_at"] <= w["to"]])
        for room in sorted(rooms):
            first_after_reset = room in reset and room not in visited_since
            first_after_staging = (room in touched_after and i >= after
                                   and (room, "staged") not in visited_since)
            control = not (first_after_reset or first_after_staging)
            alarms = []
            for r in kept:
                if not r["large"] or room_of(r["x_m"], r["y_m"]) != room:
                    continue
                if at_door(r["x_m"], r["y_m"]):
                    out["doors"].append({"step": w["step"], "kind": r["kind"],
                                         "at": [r["x_m"], r["y_m"]]})
                    continue
                if not control and any(math.hypot(r["x_m"] - px, r["y_m"] - py)
                                       <= STAGED_CLEAR_M for px, py in staged_places):
                    continue
                later = [v for v in steps[i + 1:] if v["step"] in STEPS
                         and room in {room_of(x, y) for x, y in v["stops"]}]
                held = None if not later else bool(
                    near(later[0]["report"], r["kind"], r["x_m"], r["y_m"]))
                alarms.append({"kind": r["kind"], "at": [r["x_m"], r["y_m"]],
                               "extent_m": r["extent_m"], "held_up": held})
            out["room_controls"].append({"step": w["step"], "room": room,
                                         "control": control, "false_alarms": alarms})
            visited_since.add(room)
            if first_after_staging:
                visited_since.add((room, "staged"))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("name")
    p.add_argument("staged")
    p.add_argument("--sessions", default=os.path.expanduser("~/.ugv/sessions"))
    p.add_argument("--log")
    a = p.parse_args()
    with open(os.path.join(a.sessions, a.name + ".json")) as f:
        record = json.load(f)
    with open(a.staged) as f:
        staged = json.load(f)
    log = a.log or os.path.join(control_count.CHANGES,
                                control_count.latest_map() + ".log.jsonl")
    with open(log) as f:
        rows = [json.loads(line) for line in f]
    got = score(record, staged, rows)
    for e in got["events"]:
        print("%-22s %-4s at (%6.2f, %6.2f): %-11s %s" % (
            e["what"][:22], e["kind"], e["at"][0], e["at"][1], e["outcome"],
            ("in %s, %.2f m change at (%.2f, %.2f)" % (e["step"], e["change"]["extent_m"],
                                                        e["change"]["x_m"], e["change"]["y_m"])
             if e["change"] else ("by %s" % e["step"] if e["step"] else "never in reach"))))
    found = sum(e["outcome"] == "found" for e in got["events"])
    small = sum(e["outcome"] == "found small" for e in got["events"])
    print("found %d of %d events, %d more found small" % (found, len(got["events"]), small))
    controls = [c for c in got["room_controls"] if c["control"]]
    alarms = [a for c in controls for a in c["false_alarms"]]
    print("%d room-controls, %d false alarms, %d held up" % (
        len(controls), len(alarms), sum(1 for a in alarms if a["held_up"])))
    for c in got["room_controls"]:
        for alarm in c["false_alarms"]:
            print("  false alarm in %s, %s%s: %s (%.2f, %.2f) %.2f m, held up: %s" % (
                c["room"], c["step"], "" if c["control"] else " (staged room)",
                alarm["kind"], alarm["at"][0], alarm["at"][1], alarm["extent_m"],
                alarm["held_up"]))
    for d in got["doors"]:
        print("  door: %s %s at (%.2f, %.2f)" % (d["step"], d["kind"], d["at"][0], d["at"][1]))
    print(json.dumps(got))


if __name__ == "__main__":
    main()
