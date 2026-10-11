"""What the change watch logged during a run: M5's count for a control or a scene.

    python3 control_count.py START [END] [--map MAP_ID]

START and END are HH:MM[:SS] today on the rover's clock, or seconds since the
epoch; END defaults to now. Run on the rover, after the run, once the watch has
written its next report (it reports every 30 s). Reads
~/.ugv/changes/<map_id>.log.jsonl, where the watch appends each change the first
time it finds it (ros_nav/change_node.py), and its current report.

On a control -- nothing moved, the owner saying so before the run -- every
furniture-sized change is a false alarm unless it is shown to be something that
did move, such as a door, which is said beside it, never dropped from the count.
"""
import argparse
import datetime as dt
import glob
import json
import math
import os
import time

CHANGES = os.path.expanduser("~/.ugv/changes")


def when(text):
    try:
        return float(text)
    except ValueError:
        pass
    parts = [int(p) for p in text.split(":")]
    today = dt.datetime.now().replace(microsecond=0)
    return today.replace(hour=parts[0], minute=parts[1],
                         second=parts[2] if len(parts) > 2 else 0).timestamp()


def clock(t):
    # A boot after the power was off starts the clock at 1970 until it syncs, and
    # a scan the watch counted then carries that time (2026-10-11).
    if t is None:
        return "?"
    if t < 1e9:
        return "unset clock"
    return dt.datetime.fromtimestamp(t).strftime("%m-%d %H:%M:%S")


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("start")
    p.add_argument("end", nargs="?")
    p.add_argument("--map")
    a = p.parse_args()
    start = when(a.start)
    end = when(a.end) if a.end else time.time()
    if a.map:
        map_id = a.map
    else:
        reports = glob.glob(os.path.join(CHANGES, "*.json"))
        map_id = os.path.basename(max(reports, key=os.path.getmtime))[:-len(".json")]
    with open(os.path.join(CHANGES, map_id + ".json")) as f:
        report = json.load(f)
    logged = []
    try:
        with open(os.path.join(CHANGES, map_id + ".log.jsonl")) as f:
            for line in f:
                row = json.loads(line)
                if start <= row["logged_at"] <= end:
                    logged.append(row)
    except FileNotFoundError:
        pass

    print("map %s, %s to %s" % (map_id, clock(start), clock(end)))
    print("watch now: %s, %d scans in all, %d refused as not fitting; report written %s"
          % ("counting" if report.get("counting") else "paused: %s" % report.get("paused"),
             report.get("scans", 0), report.get("refused", 0), clock(report.get("at"))))
    if report.get("at", 0) < end:
        print("  (its last report is older than the end of the window: run again in 30 s)")
    # The watch logs a group again when it drops out of one report and comes back
    # in a later one, so the same change, the same kind within 0.3 m, counts once.
    counted = []
    for row in logged:
        again = any(c["kind"] == row["kind"] and math.hypot(c["x_m"] - row["x_m"],
                                                            c["y_m"] - row["y_m"]) < 0.3
                    for c in counted)
        if not again:
            counted.append(row)
        print("%s  %-4s %-9s at (%6.2f, %6.2f)  %.2f m across, %.3f m2; seen so from %s, "
              "otherwise at %s%s"
              % (clock(row["logged_at"]), row["kind"],
                 "FURNITURE" if row["large"] else "small", row["x_m"], row["y_m"],
                 row["extent_m"], row["area_m2"], clock(row["seen_from"]),
                 clock(row.get("was_otherwise_at")), "  (again)" if again else ""))
    logged = counted
    large = [r for r in logged if r["large"]]
    print("%d furniture-sized change%s logged, %d smaller"
          % (len(large), "" if len(large) == 1 else "s", len(logged) - len(large)))


if __name__ == "__main__":
    main()
