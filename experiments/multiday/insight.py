"""The fixes of 2026-10-20 on the floor, before a second round of the
multi-day benchmark: a thing moved while the parked rover watches is reported
as it drives off, and an autonomy run goes to look and waits for the change
watch before it ends. On the rover, from ~/multiday with ~/m5 beside it.

    python3 insight.py brief          what the owner is asked to do, and when
    python3 insight.py run NAME       detached by insight.sh: parked on the charger until
                                      NAME.go, the settling wait, off to living-3, the run, home
    python3 insight.py score NAME     the criteria below, from the change log, the run's
                                      decisions and the session's record (NAME-insight.json)

**The arrangement before the rover is switched on** is day 2's: the red bean
bag at P1, the black one at P5, the stool at P3. The change watch calls a place
by what its last two minutes say only when they differ from the rest of the
visit, and a visit begins when the place comes into sight, so the things must
stand there from the start of the rover's watch (docs/progress/2026-10-20-a-move-made-in-the-watchs-sight.md).
**With the rover watching from its charger** the owner moves the red bean bag
from P1 to P2 and the black one from P5 to P4, beside the charger -- where day
2's was set down unreported -- unplugs the rover and leaves. The stool stays:
it is the control. From the charger the lidar sees P2, P4 and P5 and barely P1
or P3 (day 2's recording), so the three changes it watches are a new thing at
P2, a new thing at P4 and P5 emptied.

**The watch reports nothing parked**: a change needs seeing from places 0.5 m
apart. So the session drives to living-3, which sees all five spots, and
dwells there before the run starts; an autonomy run with nothing reported ends
where it stands (day 2).

Criteria, fixed before the session (`score`):

1. **Seen in its sight** (aa3a76f): logged from the owner's "done" to a minute
   after the dwell, a new change within MATCH_M of P2 and of P4 and a gone one
   within MATCH_M of P5.
2. **Goes to look** (9c0e499): the run chooses a goal at a change within
   GOAL_M of P1, P2, P4 or P5.
3. **Waits for the watch** (9c0e499): the run waits for the change watch's
   report on where it last stopped at least once, or spends its whole budget.
4. **Nothing less safe**: no stop file, home within 0.3 m with the position
   trusted, nobody needed to step in.

And the control: nothing logged within MATCH_M of P3 all session.
"""
import glob
import json
import math
import os
import shutil
import sqlite3
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
for path in (os.path.expanduser("~/m5"), os.path.join(HERE, "..", "m5_changes")):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
import control_count  # noqa: E402
import session  # noqa: E402

BENCH = json.load(open(os.path.join(HERE, "benchmark.json")))
SPOTS = {k: tuple(v["at"]) for k, v in BENCH["spots"].items()}
STATE = os.path.expanduser("~/.ugv/world/visit_model")
START = {"red bean bag": "P1", "black bean bag": "P5", "stool": "P3"}
MOVES = {"red bean bag": ("P1", "P2"), "black bean bag": ("P5", "P4")}
#: What the watch can see from the charger: NEW at P2 and P4, GONE at P5.
WATCHED = [("NEW", "P2"), ("NEW", "P4"), ("GONE", "P5")]
CONTROL = "P3"
#: The loop's middle stop: on the map it sees all five spots, P2 1.0 m away.
OFF = ("living-3", -18.1, -14.1)
#: Two reports of the watch, with its 20 s of evidence from the second viewpoint.
DWELL_S = 60.0
#: The watch's last two minutes all after the moves: its recent windows roll
#: every 60 s, two of them kept.
SETTLE_S = 180.0
#: How long the rover waits on its charger for the owner's "done".
PARK_MAX_MIN = 30.0
RUN_MINUTES = BENCH["arm_minutes"]
#: A change logged this near a spot is the thing there: the watch places a
#: group by the cells it sees, the near face of a bean bag 0.3 m short of its
#: middle (the day-2 replay found them within 0.5 m).
MATCH_M = 0.6
#: A goal at a change this near a spot is a look at the thing moved.
GOAL_M = 1.0
HOME_M = 0.3
EPISODES = os.path.expanduser("~/.ugv/autonomy/episodes.db")


def brief():
    s = BENCH["spots"]
    print("\n".join([
        "Before you switch the rover on (or now, if it is already on and they are there since this morning):",
        "- the red bean bag: %s (P1)" % s["P1"]["in_words"],
        "- the black bean bag: %s (P5)" % s["P5"]["in_words"],
        "- the stool: %s (P3)" % s["P3"]["in_words"],
        "Then switch it on, on its charger, and leave it a few minutes.",
        "When I say, with the rover on its charger and watching:",
        "- the red bean bag over to P2: %s" % s["P2"]["in_words"],
        "- the black bean bag over to P4, beside the charger: %s" % s["P4"]["in_words"],
        "- leave the stool where it is",
        "- unplug the rover, leave the living room and say done.",
        "About 3 minutes later it drives off, looks around on its own for up to %g minutes and "
        "comes home; please keep the living room, corridor and bedroom clear until it is back."
        % RUN_MINUTES]))


def keep_lidar(name):
    """The world model's lidar over the session, and the drive recording
    stopped: as day.py keeps them."""
    lidar = session.path_of(name, "-lidar")
    os.makedirs(lidar, exist_ok=True)
    t0 = json.load(open(session.path_of(name)))["entries"][0]["at"] - 600
    for f in glob.glob(os.path.join(STATE, "lidar", "*.npz")):
        if os.path.getmtime(f) >= t0:
            shutil.copy2(f, lidar)
    bag = os.path.expanduser("~/ugv/ros_nav/recordings/bags/session-" + name)
    subprocess.run(["pkill", "-INT", "-f", "ros2 [b]ag record --output " + bag], check=False)
    return lidar


def pause(name, seconds):
    """Sleep, but give way to the stop file."""
    t0 = time.time()
    while time.time() - t0 < seconds and not session.stopped(name):
        time.sleep(2)


def run(name):
    session.start(name)
    report = session.report()
    session.note(name, "parked", begin=True, battery=session.battery(), report=report,
                 start=START, moves=MOVES, until="%s or %g min" % (session.path_of(name, ".go"), PARK_MAX_MIN))
    if not report.get("counting"):
        session.note(name, "parked", stop="the change watch is not counting: nothing to test")
        open(session.path_of(name, ".stop"), "w").close()
    t0 = time.time()
    while time.time() - t0 < 60 * PARK_MAX_MIN and not os.path.exists(session.path_of(name, ".go")) \
            and not session.stopped(name):
        time.sleep(2)
    went = os.path.exists(session.path_of(name, ".go"))
    session.note(name, "parked", end=True, go=went, waited_s=round(time.time() - t0),
                 battery=session.battery(), report=session.report())
    if went and not session.held(name, "settle"):
        session.note(name, "settle", begin=True, seconds=SETTLE_S)
        pause(name, SETTLE_S)
        session.note(name, "settle", end=True, report=session.report())
    if went and not session.held(name, "off") and not session.low_battery(name, "off"):
        label, x, y = OFF
        session.face(name, "off", x, y)
        session.drive(name, "off", label, x, y)
        pause(name, DWELL_S)
        session.note(name, "off", dwelt_s=DWELL_S, report=session.report(), **session.pose_check())
        session.note(name, "arm A", begin=True, battery=session.battery())
        began = time.time()
        session.run(name, RUN_MINUTES)
        session.note(name, "arm A", end=True, minutes=round((time.time() - began) / 60.0, 2),
                     battery=session.battery())
        session.home(name)
    lidar = keep_lidar(name)
    session.note(name, "insight", end=True, lidar_kept_in=lidar, battery=session.battery())


def _at(entries, step, **match):
    for e in entries:
        if e["step"] == step and all(e.get(k) == v for k, v in match.items()):
            return e
    return None


def _near(x, y, spot, within):
    return math.hypot(x - SPOTS[spot][0], y - SPOTS[spot][1]) <= within


def run_events(t0, t1):
    """The run's decisions and how its episodes closed, from the episode store."""
    db = sqlite3.connect("file:%s?mode=ro" % EPISODES, uri=True)
    try:
        rows = db.execute("select at, kind, body_json from events where kind in ('decision', 'closed')"
                          " and at between ? and ? order by at", (t0, t1)).fetchall()
    finally:
        db.close()
    chose, closed = [], []
    for at, kind, body in rows:
        b = json.loads(body) or {}
        if kind == "decision" and b.get("chose"):
            chose.append({"at": at, "chose": b["chose"]})
        elif kind == "closed":
            closed.append({"at": at, "outcome": b.get("outcome"), "detail": str(b.get("detail") or "")[:300]})
    return chose, closed


def score(name):
    entries = json.load(open(session.path_of(name)))["entries"]
    go = _at(entries, "parked", end=True)
    off = _at(entries, "off", dwelt_s=DWELL_S)
    run_begin, run_end = _at(entries, "run", begin=True), _at(entries, "run", end=True)
    home = next((e for e in reversed(entries) if e["step"] == "home" and "trusted" in e), None)
    out = {"name": name, "criteria": {}}
    map_id = control_count.latest_map()
    first, last = entries[0]["at"], entries[-1]["at"]
    logged, _ = control_count.counted(control_count.logged_between(map_id, first, last + 120))
    out["logged"] = [{k: r[k] for k in ("kind", "x_m", "y_m", "extent_m", "large", "logged_at")} for r in logged]

    # 1. Seen in its sight.
    if go and off:
        window = [r for r in logged if go["at"] <= r["logged_at"] <= off["at"] + 60]
        found = {}
        for kind, spot in WATCHED:
            hit = [r for r in window if r["kind"] == kind and _near(r["x_m"], r["y_m"], spot, MATCH_M)]
            found["%s at %s" % (kind, spot)] = [[r["x_m"], r["y_m"], r["extent_m"], r["large"],
                                                round(r["logged_at"] - off["at"])] for r in hit]
        parked = [r for r in logged if go["at"] <= r["logged_at"] < off["at"] - DWELL_S]
        out["criteria"]["1 seen in its sight"] = {"pass": all(found.values()), "found": found,
                                                  "logged while still parked": len(parked)}
    else:
        out["criteria"]["1 seen in its sight"] = {"pass": False, "why": "the session did not reach the dwell"}

    # 2 and 3, from the run's own record.
    if run_begin and run_end:
        chose, closed = run_events(run_begin["at"], run_end["at"] + 5)
        at_moved = []
        for c in chose:
            goal = c["chose"]
            if not goal.startswith("investigate_change:") or "@" not in goal:
                continue
            x, y = (float(v) for v in goal.rsplit("@", 1)[1].split(","))
            spots = [s for s in ("P1", "P2", "P4", "P5") if _near(x, y, s, GOAL_M)]
            if spots:
                at_moved.append({"goal": goal, "spots": spots, "after_s": round(c["at"] - run_begin["at"])})
        out["criteria"]["2 goes to look"] = {"pass": bool(at_moved), "goals_at_moved_things": at_moved,
                                             "every_goal": [c["chose"] for c in chose]}
        waits = [c for c in closed if "change watch" in c["detail"]]
        spent = run_end.get("spent") or {}
        whole = (spent.get("seconds") or 0) >= 60 * RUN_MINUTES - 5
        out["criteria"]["3 waits for the watch"] = {"pass": bool(waits) or whole, "waits": len(waits),
                                                    "whole_budget": whole, "why_it_ended": run_end.get("why")}
    else:
        out["criteria"]["2 goes to look"] = {"pass": False, "why": "no run"}
        out["criteria"]["3 waits for the watch"] = {"pass": False, "why": "no run"}

    # 4. Nothing less safe.
    stops = [e for e in entries if e.get("stop")]
    home_ok = bool(home and home.get("trusted") and home.get("pose") and math.hypot(
        home["pose"]["x_m"] - session.HOME[0], home["pose"]["y_m"] - session.HOME[1]) <= HOME_M)
    out["criteria"]["4 nothing less safe"] = {"pass": not stops and not session.stopped(name) and home_ok,
                                              "stops": [e["stop"] for e in stops], "home": home and home.get("pose")}

    control = [r for r in logged if _near(r["x_m"], r["y_m"], CONTROL, MATCH_M)]
    out["control"] = {"pass": not control, "logged_at_%s" % CONTROL: [[r["kind"], r["x_m"], r["y_m"]] for r in control]}
    with open(session.path_of(name, "-insight.json"), "w") as f:
        json.dump(out, f, indent=1)
    for k, v in out["criteria"].items():
        print("%-24s %s  %s" % (k, "PASS" if v["pass"] else "FAIL",
                                json.dumps({a: b for a, b in v.items() if a != "pass"})[:400]))
    print("%-24s %s  %s" % ("control", "PASS" if control == [] else "FAIL", out["control"]))
    print("logged in the session: %d (%d furniture-sized)" % (len(logged), sum(r["large"] for r in logged)))


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else ""
    if what == "brief":
        brief()
    elif what == "run" and len(sys.argv) > 2:
        run(sys.argv[2])
    elif what == "score" and len(sys.argv) > 2:
        score(sys.argv[2])
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
