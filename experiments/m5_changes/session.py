"""One supervised session, as planned in
docs/plans/sessions-that-serve-several-milestones.md. Run on the rover.

    python3 session.py dry                    plan every leg of the loop, nothing moves
    python3 session.py start NAME             recording on; battery and pose checked
    python3 session.py loop NAME A|B          the survey loop, from whichever end is nearer
    python3 session.py wait NAME [MINUTES]    to the bedroom, until NAME.go appears or the minutes pass
    python3 session.py run NAME [MINUTES]     an autonomy run over the flat
    python3 session.py home NAME              back to the charger, for the owner to plug in
    python3 session.py count NAME             the change log counted step by step
    python3 session.py all NAME [--b-first]   the steps in order: start, loop A, wait,
                                              run and loop B (swapped with --b-first), home, count

Everything a step does is appended to ~/.ugv/sessions/NAME.json, so a session
can be stopped and taken up again a step at a time. Touching NAME.stop beside it
ends the step in progress after its current leg.

**The loop is the instrument** (the plan's rule 3): the same viewpoints in the
same order every time, two or more a room so that the change watch's
two-places rule can be met, driven here and not by the executive. Every
viewpoint is one the rover has stood on in the runs of 2026-10-10 and 11, and
none is within reach of the rug under the dining table, which the rover stuck on
on 2026-10-11, or through the four doors the owner keeps shut. A goal without a
heading faces the way the rover travelled, so the loop spends no time turning;
the lidar sees all round.
"""
import json
import math
import os
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
#: The rug under the dining table, min/max x and y with the chairs round it.
RUG = (-19.8, -17.1, -13.5, -11.9)
#: The owner's floor for supervised drives, at rest.
FLOOR_PERCENT = 5
RECORD_S = 4500


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
    for label, x, y in views:
        if stopped(name) or low_battery(name, step):
            break
        face(name, step, x, y)
        drive(name, step, label, x, y)
        time.sleep(DWELL_S)
        note(name, step, viewpoint=label, **pose_check())
    note(name, step, end=True, battery=battery())


def start(name):
    if os.path.exists(path_of(name)):
        raise SystemExit("a session called %s is already recorded; pick another name" % name)
    check = pose_check()
    pct = battery()
    report = bridge({"op": "changes"}).get("report", {})
    note(name, "start", battery=pct, watch_counting=report.get("counting"),
         watch_paused=report.get("paused"), **check)
    if not (check["trusted"] and check["settled"]):
        raise SystemExit("navigation does not trust its position: refit or turn first")
    rec = "%s/ugv/ros_nav/record_drive.sh" % os.path.expanduser("~")
    log = open("/tmp/%s.recording.log" % name, "w")
    subprocess.Popen([rec, "--seconds", str(RECORD_S), "--name", "session-" + name],
                     stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                     start_new_session=True)
    time.sleep(4)
    note(name, "start", recording="session-" + name, seconds=RECORD_S)


def wait(name, minutes=20.0):
    p, _ = where()
    if not p or math.hypot(p["x_m"] - WAIT[0], p["y_m"] - WAIT[1]) > 0.3:
        drive(name, "wait", "wait", *WAIT)
    note(name, "wait", begin=True, battery=battery(), until="%s or %g min" % (path_of(name, ".go"), minutes))
    t0 = time.time()
    while time.time() - t0 < 60 * minutes and not os.path.exists(path_of(name, ".go")) \
            and not stopped(name):
        time.sleep(5)
    note(name, "wait", end=True, waited_s=round(time.time() - t0), battery=battery())


def run(name, minutes=10.0):
    if low_battery(name, "run"):
        return
    budget = {"seconds": int(60 * minutes), "travel_m": None, "actions": None}
    r = call("autonomy_start", {"purpose": "M5 session %s: the autonomy arm" % name, "budget": budget})
    run_id = (r.get("run") or {}).get("id")
    note(name, "run", begin=True, ok=r.get("ok"), error=r.get("error"), run=run_id, battery=battery())
    if not r.get("ok"):
        return
    t0, low = time.time(), 0
    while time.time() - t0 < 60 * minutes + 120:
        st = call("autonomy_status")
        nav = call("nav_status")
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
    note(name, "run", end=True, run=run_id, why=str(st.get("why") or (st.get("run") or {}).get("ended"))[:300],
         spent=(st.get("run") or {}).get("spent"), battery=battery())


def home(name):
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
        run(name, float(args[2]) if len(args) > 2 else 10.0)
    elif what == "home":
        home(name)
    elif what == "count":
        count(name)
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
