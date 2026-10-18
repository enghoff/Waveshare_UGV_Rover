"""One day of the multi-day usefulness benchmark, on the rover, from ~/multiday
with ~/m5 (experiments/m5_changes) beside it.

    python3 day.py brief DAY              what the owner is asked to set out today
    python3 day.py restart NAME           day 2: the daemon and the world model's job
                                          restarted with the rover parked, and every
                                          question asked before and after (NAME-restart.json)
    python3 day.py run NAME DAY           the day's two sessions, in the day's order,
                                          then home; run detached by launch.sh

The driving is session.py's: the survey loop with its sweeps (the fixed
schedule, arm B), an autonomy run with a time budget (arm A), the position
check at every stop, the stop file and the battery floor, and home. What this
adds is the morning copy of the world model (`NAME-start/`), taken after the
owner has set things out and before either arm drives, each arm's window in the
session's record, and the lidar the world model had over the day
(`NAME-lidar/`), from which answer.py files each arm's pictures alone onto the
morning copy.
"""
import glob
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
for path in (os.path.expanduser("~/m5"), os.path.join(HERE, "..", "m5_changes")):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
import session  # noqa: E402

BENCH = json.load(open(os.path.join(HERE, "benchmark.json")))
STATE = os.path.expanduser("~/.ugv/world/visit_model")


def brief(day):
    d = BENCH["days"][str(day)]
    spots = BENCH["spots"]
    lines = ["Day %s of 3." % day]
    for item, spot in d["put"].items():
        lines.append("- the %s: %s" % (item, spots[spot]["in_words"]))
    lines.append("Nothing else moves; nobody in the living room, corridor or bedroom while it drives.")
    lines.append("Order today: %s." % " then ".join(
        {"A": "its own choice of where to look", "B": "the fixed loop"}[a] for a in d["order"]))
    print("\n".join(lines))


def phrases():
    out = dict(BENCH["items"])
    out.update({k: v["phrase"] for k, v in BENCH["controls"].items()})
    return out


def ask_live():
    """Every question asked of the running daemon, as the voice would ask it."""
    out = {}
    for name, phrase in phrases().items():
        r = session.call("find_thing", {"description": phrase}, timeout=60)
        out[name] = None if not r.get("found") else [r.get("map_x_m"), r.get("map_y_m")]
    return out


def restart(name):
    """The plan's restart: a process restart with retained evidence, answered
    the same before and after. The map is not reset (agreed 2026-10-18)."""
    name += "-restart"
    before = ask_live()
    session.note(name, "restart", asked_before=before)
    for script in ("~/ugv/restart.sh", "~/ugv/world_state/visit_model/restart_job.sh"):
        subprocess.run([os.path.expanduser(script)], check=False, timeout=300)
    for _ in range(60):
        try:
            if session.call("list_tools", timeout=5).get("ok"):
                break
        except OSError:
            pass
        time.sleep(5)
    # The job writes its records within a pass of starting.
    for _ in range(36):
        try:
            with open(os.path.join(STATE, "things.json")) as f:
                if time.time() - json.load(f)["written_at"] < 30:
                    break
        except (OSError, ValueError, KeyError):
            pass
        time.sleep(5)
    after = ask_live()
    _, nav = session.where()
    session.note(name, "restart", asked_after=after, same=before == after,
                 settled=nav.get("map_settled"), trusted=nav.get("position_trusted"))
    print("answers the same after the restart:", before == after)


def arm(name, which, minutes):
    session.note(name, "arm " + which, begin=True, battery=session.battery())
    began = time.time()
    if which == "A":
        session.run(name, minutes)
    else:
        session.loop(name, "A")
        session.loop(name, "B")
    session.note(name, "arm " + which, end=True, minutes=round((time.time() - began) / 60.0, 2),
                 battery=session.battery())


def run(name, day):
    d = BENCH["days"][str(day)]
    session.start(name)
    session.note(name, "day", day=int(day), order=d["order"], put=d["put"])
    keep = session.path_of(name, "-start")
    os.makedirs(keep, exist_ok=True)
    for one in ("engine.pkl", "things.json"):
        shutil.copy2(os.path.join(STATE, one), keep)
    session.note(name, "snapshot", kept_in=keep)
    for which in d["order"]:
        if session.stopped(name):
            break
        arm(name, which, BENCH["arm_minutes"])
    session.home(name)
    began = session.path_of(name)
    lidar = session.path_of(name, "-lidar")
    os.makedirs(lidar, exist_ok=True)
    t0 = json.load(open(began))["entries"][0]["at"] - 600
    for f in glob.glob(os.path.join(STATE, "lidar", "*.npz")):
        if os.path.getmtime(f) >= t0:
            shutil.copy2(f, lidar)
    bag = os.path.expanduser("~/ugv/ros_nav/recordings/bags/session-" + name)
    subprocess.run(["pkill", "-INT", "-f", "ros2 [b]ag record --output " + bag], check=False)
    session.note(name, "day", end=True, lidar_kept_in=lidar, battery=session.battery())


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else ""
    if what == "brief":
        brief(sys.argv[2])
    elif what == "restart":
        restart(sys.argv[2])
    elif what == "run":
        run(sys.argv[2], sys.argv[3])
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
