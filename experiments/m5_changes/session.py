"""One supervised session, as planned in
docs/plans/sessions-that-serve-several-milestones.md. Run on the rover.

    python3 session.py dry                    plan every leg of the loop, nothing moves
    python3 session.py start NAME             recording on; battery and pose checked
    python3 session.py loop NAME A|B          the survey loop, from whichever end is nearer
    python3 session.py wait NAME [MINUTES]    to the bedroom, until NAME.go appears or the minutes pass
    python3 session.py run NAME [MINUTES]     an autonomy run over the flat, a loop's time by default
    python3 session.py home NAME              back to the charger, for the owner to plug in
    python3 session.py count NAME             the change log counted step by step
    python3 session.py sweep NAME             the camera aimed all round where the rover stands
    python3 session.py seen NAME LABEL:X,Y .. the records the session's looks went to at each place
    python3 session.py all NAME [--b-first]   the steps in order: start, loop A, wait,
                                              run and loop B (swapped with --b-first), home, count

Everything a step does is appended to ~/.ugv/sessions/NAME.json, so a session
can be stopped and taken up again a step at a time. Touching NAME.stop beside it
ends the step in progress after its current leg and keeps every later step from
driving; a loop leaves one itself when its position check disagrees at two stops
running, so that a rover that has lost its place is not driven home on it.

**The loop is the instrument** (the plan's rule 3): the same viewpoints in the
same order every time, two or more a room so that the change watch's
two-places rule can be met, driven here and not by the executive. Every
viewpoint is one the rover has stood on in the runs of 2026-10-10 and 11, and
none is within reach of the rug under the dining table, which the rover stuck on
on 2026-10-11, or through the four doors the owner keeps shut. A goal without a
heading faces the way the rover travelled, so the loop spends no time turning;
the lidar sees all round, and since session 4 the camera is aimed all round
from every stop inside its dwell (`sweep`).
"""
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import control_count  # noqa: E402

SESSIONS = os.path.expanduser("~/.ugv/sessions")
VIEWS = [
    ("living-1", -19.4, -15.4),
    ("living-2", -19.6, -14.4),
    ("living-3", -18.1, -14.1),
    ("living-4", -16.5, -14.1),
    ("corridor-1", -13.4, -13.7),
    ("corridor-2", -11.3, -13.2),
    ("bedroom-1", -9.1, -12.3),
    ("bedroom-2", -8.5, -11.4),
]
#: Where the rover waits while the owner stages a scene, the loop's last
#: viewpoint: more than 6 m, the lidar's reach, from the living room.
WAIT = VIEWS[-1][1:]
#: Parked on the charger, 2026-10-11.
HOME = (-17.20, -15.93, 135.0)
#: Long enough for the change watch's 20 s of evidence from each viewpoint.
DWELL_S = 20.0
#: Where the camera is aimed at every stop, in degrees left of the way the
#: rover faces. The rover's own looking takes the picture straight ahead as it
#: comes to rest; with these four the 130-degree camera has seen all round, and
#: the 65-degree depth camera all but 50 degrees behind.
SWEEP_LEFT_DEG = (60.0, 120.0, -60.0, -120.0)
#: How far out the point aimed at is put; any distance gives the same pan.
SWEEP_M = 2.0
#: How long a sweep's look waits for the rover's own look to let go of the
#: camera. In session 4, 22 of 64 were refused at 5 s, the rover's own having
#: held it for up to 12 s.
SWEEP_WAIT_S = 15.0
#: A record stands at a place for `seen` when placed this near it: the
#: record-moving step's own limit (world_state/relocate.py, FROM_M).
SEEN_M = 1.0
#: The rug under the dining table, min/max x and y with the chairs round it.
RUG = (-19.8, -17.1, -13.5, -11.9)
#: The owner's floor for supervised drives, at rest.
FLOOR_PERCENT = 5
RECORD_S = 4500
#: How far apart the points of a run's path are kept.
PATH_STEP_M = 0.5
#: A stop's position check disagrees beyond these; LOST_STOPS in a row end the
#: session where the rover stands (`disagrees`).
LOST_M = 0.3
LOST_DEG = 10.0
LOST_STOPS = 2
#: The autonomy arm gets the time a loop takes, as the counts' decision has it:
#: the shakedown's two loops took 3 min 22 s and 3 min 24 s.
RUN_MINUTES = 3.5


def call(name, arguments=None, timeout=600):
    s = socket.create_connection(("127.0.0.1", 8769), timeout)
    f = s.makefile("rwb")
    f.write(json.dumps({"call": name, "arguments": arguments or {}}).encode() + b"\n")
    f.flush()
    return json.loads(f.readline())


def bridge(op, timeout=120):
    s = socket.create_connection(("127.0.0.1", 8773), timeout)
    f = s.makefile("rwb")
    f.write(json.dumps(op).encode() + b"\n")
    f.flush()
    while True:
        r = json.loads(f.readline())
        if r.get("kind") != "progress":
            return r


def path_of(name, ext=".json"):
    return os.path.join(SESSIONS, name + ext)


def note(name, step, **what):
    """Append one entry to the session's record and print it."""
    os.makedirs(SESSIONS, exist_ok=True)
    try:
        with open(path_of(name)) as f:
            record = json.load(f)
    except FileNotFoundError:
        record = {"name": name, "entries": []}
    entry = dict(step=step, at=round(time.time(), 1), **what)
    record["entries"].append(entry)
    with open(path_of(name) + ".tmp", "w") as f:
        json.dump(record, f, indent=1)
    os.replace(path_of(name) + ".tmp", path_of(name))
    print(time.strftime("%H:%M:%S"), step, json.dumps(what)[:300], flush=True)
    return record


def report():
    """The change watch's current report, trimmed to the changes: kept at the end
    of every step, because whether a change held up once its place was seen from
    another viewpoint is read from the next step's (the counts' decision)."""
    r = bridge({"op": "changes"})
    return {"at": r.get("at"), "counting": r.get("counting"),
            "changes": [{k: c.get(k) for k in ("kind", "x_m", "y_m", "extent_m", "large",
                                               "seen_from", "seen_until")}
                        for c in r.get("changes") or []]}


def stopped(name):
    return os.path.exists(path_of(name, ".stop"))


def where():
    nav = call("nav_status")
    p = nav.get("pose") or {}
    return p, nav


def battery():
    return call("battery").get("percent")


def pose_check():
    """Navigation's own say on the pose, and the drift check's (R-NAV-14)."""
    p, nav = where()
    m = bridge({"op": "measure"})
    return {"pose": p, "trusted": nav.get("position_trusted"), "settled": nav.get("map_settled"),
            "drift_m": m.get("moved_m"), "drift_deg": m.get("turned_deg"),
            "fit": m.get("score"), "fit_here": m.get("guess_score")}


def drive(name, step, label, x, y, heading=None):
    t = time.time()
    args = {"x_m": x, "y_m": y}
    if heading is not None:
        args["heading_deg"] = heading
    r = call("drive_to", args)
    p, _ = where()
    note(name, step, leg=label, goal=[x, y], seconds=round(time.time() - t, 1),
         outcome=r.get("reason") or r.get("error") or r.get("ok"),
         travelled_m=r.get("travelled_m"), detail=str(r.get("detail") or "")[:200],
         at_pose=[p.get("x_m"), p.get("y_m"), p.get("heading_deg")])
    return r


def face(name, step, x, y):
    """Turn on the spot towards (x, y). Nav2's lattice planner will not turn
    sharply where it is tight, and from a viewpoint facing away from the next it
    planned a loop of 3.7 m for 1 m, through the rug; facing it, the route is
    the straight line."""
    p, _ = where()
    if not p:
        return
    bearing = math.degrees(math.atan2(y - p["y_m"], x - p["x_m"]))
    angle = (bearing - p["heading_deg"] + 180.0) % 360.0 - 180.0
    if abs(angle) > 20.0:
        r = call("turn_in_place", {"angle_deg": round(angle, 1)})
        note(name, step, turn=round(angle, 1), outcome=r.get("reason") or r.get("error"))


def low_battery(name, step):
    pct = battery()
    if pct is not None and pct <= FLOOR_PERCENT:
        note(name, step, stop="battery %s%% at rest, at the floor" % pct)
        return True
    return False


def loop(name, which):
    """The survey loop, from whichever end is nearer the rover."""
    step = "loop " + which
    p, _ = where()
    views = list(VIEWS)
    if p and math.hypot(p["x_m"] - views[-1][1], p["y_m"] - views[-1][2]) < \
            math.hypot(p["x_m"] - views[0][1], p["y_m"] - views[0][2]):
        views.reverse()
    note(name, step, begin=[v[0] for v in views], battery=battery())
    lost = 0
    for label, x, y in views:
        if stopped(name) or low_battery(name, step):
            break
        p, _ = where()
        if not p or math.hypot(p["x_m"] - x, p["y_m"] - y) > 0.3:
            face(name, step, x, y)
            drive(name, step, label, x, y)
        began = time.time()
        sweep(name, step, label)
        time.sleep(max(0.0, DWELL_S - (time.time() - began)))
        check = pose_check()
        note(name, step, viewpoint=label, **check)
        lost = lost + 1 if disagrees(check) else 0
        if lost >= LOST_STOPS:
            note(name, step, stop="the position check disagreed at %d stops in a row: "
                                  "stopped where it stands for a person to look" % lost)
            open(path_of(name, ".stop"), "w").close()
            break
    note(name, step, end=True, battery=battery(), report=report())


def sweep(name, step, label):
    """The camera aimed all round from a stop, one look a direction, inside the
    stop's dwell. **A loop that drives facing the way it travels photographs
    only what is ahead**: in session 3 none of the three things staged had a
    picture from loop A, the armchair by the charger being behind the rover at
    every stop, so the record-moving step had nothing at their old places to
    go on. The gimbal turns the camera and leaves the chassis, the lidar and
    the pose alone; the lidar's change watch sees all round either way. The
    looks are recorded without a resolver pass (`settle`), which took 15 s a
    look on the charger; the looking loop's own pass files them. `seen`
    counts records near a place, not the thing's own: in session 4 it passed
    the red bean bag on a record of the empty floor where it had been."""
    p, _ = where()
    if not p:
        return
    looks = []
    for left in SWEEP_LEFT_DEG:
        a = math.radians(p["heading_deg"] + left)
        r = call("world_inspect", {"aim_at": {"x_m": p["x_m"] + SWEEP_M * math.cos(a),
                                              "y_m": p["y_m"] + SWEEP_M * math.sin(a)},
                                   "wait": True, "wait_s": SWEEP_WAIT_S, "settle": False},
                 timeout=30)
        looks.append({"left": left, "pan": (r.get("aimed") or {}).get("pan_deg"),
                      "frame": r.get("frame_id"), "error": r.get("error"), "s": r.get("took_s")})
    note(name, step, sweep=label, looks=looks)


def seen(name, places):
    """What the store has filed since the session began at each place: the
    records placed within SEEN_M of it and how many of the session's looks
    each holds. Run after loop A, before the owner stages anything: a thing
    to be moved wants its record to have taken looks here."""
    import sqlite3
    with open(path_of(name)) as f:
        since = json.load(f)["entries"][0]["at"]
    db = sqlite3.connect("file:%s?mode=ro" % os.path.expanduser("~/.ugv/world/world.db"), uri=True)
    rows = db.execute("SELECT e.id, e.placement_json, COUNT(o.id) FROM observations o"
                      " JOIN entities e ON e.id = o.entity_id WHERE o.observed_at >= ?"
                      " AND e.placement_json IS NOT NULL GROUP BY e.id", (since,)).fetchall()
    out = {}
    for label, x, y in places:
        near = []
        for rid, placement, n in rows:
            p = json.loads(placement)
            if p.get("x_m") is not None and math.hypot(p["x_m"] - x, p["y_m"] - y) <= SEEN_M:
                near.append([rid, n, round(math.hypot(p["x_m"] - x, p["y_m"] - y), 2)])
        out[label] = sorted(near, key=lambda r: -r[1])
        print("%-16s %s" % (label, out[label] or "nothing filed here this session"))
    note(name, "seen", places=out)


def disagrees(check):
    """Whether a stop's position check says the rover is not where it thinks.
    One stop is not enough: furniture staged beside a stop fills the lidar's
    view, and on 2026-10-12 the check at such a stop said 0.6-0.7 m and up to
    39 degrees out three times with the next stop agreeing to 5 cm; the pose
    really 1.35 m out disagreed at two stops running."""
    moved, turned = check.get("drift_m"), check.get("drift_deg")
    return (moved is not None and moved > LOST_M) or \
        (turned is not None and abs(turned) > LOST_DEG)


def start(name):
    if os.path.exists(path_of(name)):
        raise SystemExit("a session called %s is already recorded; pick another name" % name)
    check = pose_check()
    pct = battery()
    watch = bridge({"op": "changes"})
    note(name, "start", battery=pct, watch_counting=watch.get("counting"),
         watch_paused=watch.get("paused"), **check)
    if not (check["trusted"] and check["settled"]):
        raise SystemExit("navigation does not trust its position: refit or turn first")
    # The map as the session found it, so that a pose that goes wrong can be
    # replayed into the mapper from the graph it really had: on 2026-10-11 the
    # bedroom's could not be, the graph having been saved over since.
    kept = path_of(name, "-map")
    os.makedirs(kept, exist_ok=True)
    for one in ("current.posegraph", "current.data", "current.json"):
        source = os.path.join(os.path.expanduser("~/.ugv/map"), one)
        if os.path.exists(source):
            shutil.copy2(source, kept)
    # And the occupancy grid, which the map gate judges scans against when an
    # arm is replayed on its own scans (arm_replay.py).
    grid = call("nav_grid")
    if grid.get("ok"):
        with open(path_of(name, "-grid.json"), "w") as f:
            json.dump({k: v for k, v in grid.items() if k not in ("trail", "live")}, f)
    note(name, "start", map_kept_in=kept, grid_kept=bool(grid.get("ok")))
    rec = "%s/ugv/ros_nav/record_drive.sh" % os.path.expanduser("~")
    log = open(path_of(name, ".recording.log"), "w")
    subprocess.Popen([rec, "--seconds", str(RECORD_S), "--name", "session-" + name],
                     stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                     start_new_session=True)
    time.sleep(4)
    note(name, "start", recording="session-" + name, seconds=RECORD_S)


def held(name, step):
    """A stop file left by a loop that lost its position, or by a person, keeps
    every later step from driving: the rover stays where it is to be looked at."""
    if stopped(name):
        note(name, step, stop="not started: %s is there" % path_of(name, ".stop"))
        return True
    return False


def wait(name, minutes=20.0):
    if held(name, "wait"):
        return
    p, _ = where()
    if not p or math.hypot(p["x_m"] - WAIT[0], p["y_m"] - WAIT[1]) > 0.3:
        drive(name, "wait", "wait", *WAIT)
    # The world as it stands before anything is staged, for replaying the
    # session's looks from it (experiments/identity_moves/README.md, the test).
    # Ten seconds parked; a failure is noted and the session goes on.
    saved = call("snapshot_save", {"name": "%s before staging" % name})
    note(name, "wait", begin=True, battery=battery(), until="%s or %g min" % (path_of(name, ".go"), minutes),
         snapshot=(saved.get("snapshot") or {}).get("id") or saved.get("error") or saved)
    t0 = time.time()
    while time.time() - t0 < 60 * minutes and not os.path.exists(path_of(name, ".go")) \
            and not stopped(name):
        time.sleep(5)
    # What the watch knows when the owner has finished staging, which is where
    # both arms of the benchmark start from when each is replayed on its own
    # scans (the counts' decision).
    nav = call("nav_status")
    tallies = os.path.expanduser("~/.ugv/changes/%s.npz" % nav.get("map_id"))
    if os.path.exists(tallies):
        shutil.copy2(tallies, path_of(name, "-watch-after-wait.npz"))
    note(name, "wait", end=True, waited_s=round(time.time() - t0), battery=battery(),
         report=report())


def run(name, minutes=None):
    minutes = RUN_MINUTES if minutes is None else minutes
    if held(name, "run") or low_battery(name, "run"):
        return
    budget = {"seconds": int(60 * minutes), "travel_m": None, "actions": None}
    r = call("autonomy_start", {"purpose": "M5 session %s: the autonomy arm" % name, "budget": budget})
    run_id = (r.get("run") or {}).get("id")
    note(name, "run", begin=True, ok=r.get("ok"), error=r.get("error"), run=run_id, battery=battery())
    if not r.get("ok"):
        return
    # Where the run went, a point every PATH_STEP_M, so that the counts can tell
    # which places it came within reach of: a loop has its stops, a run only this.
    t0, low, path = time.time(), 0, []
    while time.time() - t0 < 60 * minutes + 120:
        st = call("autonomy_status")
        nav = call("nav_status")
        p = nav.get("pose") or {}
        if p.get("x_m") is not None and (not path or math.hypot(
                p["x_m"] - path[-1][0], p["y_m"] - path[-1][1]) >= PATH_STEP_M):
            path.append([round(p["x_m"], 2), round(p["y_m"], 2)])
        run_now = st.get("run") or {}
        if not st.get("enabled") or run_now.get("ended"):
            break
        if stopped(name):
            call("autonomy_stop", {"by": "the session script", "why": "its stop file"})
            break
        if not nav.get("driving"):
            pct = battery()
            low = low + 1 if (pct is not None and pct <= FLOOR_PERCENT) else 0
            if low >= 3:
                call("stop_driving")
                break
        time.sleep(5)
    st = call("autonomy_status")
    note(name, "run", end=True, run=run_id, report=report(), path=path,
         why=str(st.get("why") or (st.get("run") or {}).get("ended"))[:300],
         spent=(st.get("run") or {}).get("spent"), battery=battery())


def home(name):
    if held(name, "home"):
        return
    face(name, "home", *HOME[:2])
    drive(name, "home", "home", *HOME)
    note(name, "home", battery=battery(), **pose_check())


def count(name):
    with open(path_of(name)) as f:
        entries = json.load(f)["entries"]
    windows, open_at = [], {}
    for e in entries:
        if e.get("begin"):
            open_at[e["step"]] = e["at"]
        if e.get("end") and e["step"] in open_at:
            windows.append((e["step"], open_at.pop(e["step"]), e["at"]))
    first, last = entries[0]["at"], entries[-1]["at"]
    windows.append(("session", first, last))
    map_id = control_count.latest_map()
    out = {}
    for step, a, b in windows:
        kept, _ = control_count.counted(control_count.logged_between(map_id, a, b))
        large = [{k: r[k] for k in ("kind", "x_m", "y_m", "extent_m", "logged_at")}
                 for r in kept if r["large"]]
        out[step] = {"from": a, "to": b, "furniture": large, "smaller": len(kept) - len(large)}
        print("%-8s %s-%s  %d furniture-sized, %d smaller" % (
            step, time.strftime("%H:%M:%S", time.localtime(a)),
            time.strftime("%H:%M:%S", time.localtime(b)), len(large), len(kept) - len(large)))
        for r in large:
            print("          %-4s (%.2f, %.2f) %.2f m" % (r["kind"], r["x_m"], r["y_m"], r["extent_m"]))
    note(name, "count", map=map_id, counts=out)


def dry():
    """Every leg planned by Nav2's own planner, from where the last one ends,
    with how close each route comes to the rug. Nothing moves."""
    import rclpy
    from nav2_msgs.action import ComputePathToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    rclpy.init()
    node = Node("session_dry_run")
    planner = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    if not planner.wait_for_server(timeout_sec=20.0):
        raise SystemExit("no planner: source ~/ugv/ros_nav/env.sh and dds.sh first")

    def plan(a, b):
        # The lattice planner minds headings: the rover turns to face each goal
        # before it sets off (face), and a goal without a heading faces the way
        # it travels.
        goal = ComputePathToPose.Goal()
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        for pose, xy in ((goal.start, a), (goal.goal, b)):
            pose.header.frame_id = "map"
            pose.pose.position.x, pose.pose.position.y = xy
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
        goal.use_start = True
        sent = planner.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=5.0)
        result = sent.result().get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=10.0)
        return [(q.pose.position.x, q.pose.position.y) for q in result.result().result.path.poses]

    def rug_gap(x, y):
        dx = max(RUG[0] - x, 0.0, x - RUG[1])
        dy = max(RUG[2] - y, 0.0, y - RUG[3])
        return math.hypot(dx, dy)

    stops = ([("home", HOME[:2])] + [(v[0], v[1:]) for v in VIEWS]
             + [(v[0], v[1:]) for v in reversed(VIEWS[:-1])] + [("home", HOME[:2])])
    total = 0.0
    for (la, a), (lb, b) in zip(stops, stops[1:]):
        route = plan(a, b)
        if not route:
            print("%-11s -> %-11s NO ROUTE" % (la, lb))
            continue
        length = sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(route, route[1:]))
        total += length
        gap = min(rug_gap(*q) for q in route)
        print("%-11s -> %-11s route %5.2f m, straight %5.2f m, nearest the rug %.2f m%s" % (
            la, lb, length, math.hypot(b[0] - a[0], b[1] - a[1]), gap,
            "  <- too near" if gap < 0.35 else ""))
    print("both loops and the drives between: %.1f m" % total)
    rclpy.shutdown()


def main():
    args = sys.argv[1:]
    if not args:
        raise SystemExit(__doc__)
    what = args[0]
    if what == "dry":
        return dry()
    name = args[1]
    if what == "start":
        start(name)
    elif what == "loop":
        loop(name, args[2])
    elif what == "wait":
        wait(name, float(args[2]) if len(args) > 2 else 20.0)
    elif what == "run":
        run(name, float(args[2]) if len(args) > 2 else None)
    elif what == "home":
        home(name)
    elif what == "count":
        count(name)
    elif what == "sweep":
        sweep(name, "sweep", "where it stands")
    elif what == "seen":
        places = []
        for one in args[2:]:
            label, xy = one.rsplit(":", 1)
            places.append((label, *(float(v) for v in xy.split(","))))
        seen(name, places)
    elif what == "all":
        start(name)
        loop(name, "A")
        wait(name)
        if "--b-first" in args:
            loop(name, "B")
            run(name)
        else:
            run(name)
            loop(name, "B")
        home(name)
        time.sleep(40)
        count(name)
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
