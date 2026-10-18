"""M10's look trial: the survey loop, with a look at records in reach at every
stop, each look's predicted chance of landing written down before it is taken.

    python3 look_trial.py dry NAME          the looks each stop would take, from
                                            the live records; nothing moves
    python3 look_trial.py run NAME          record, drive the loop out and back,
                                            look at each stop, go home

Run on the rover from ~/m10, with ~/m5 (experiments/m5_changes) and
~/ugv/autonomy beside it. The driving is session.py's -- the same eight stops,
turning to face each, a position check at each, the stop file and the
battery floor -- so nothing here moves the rover in a way a session has not.

**At each stop the looks are drawn at random** among the records in reach:
placed, a height known, seen on two visits or more, not gone or set aside, 0.5
to 2.5 m away with nothing mapped in between, and inside the depth camera's
view at a calibrated tilt. Random, so the test is not of the looks the
predictor would have chosen. Up to LOOKS_PER_STOP each. Each is aimed with
`world_inspect` at the record's place and recorded, with its predicted chance
of landing (`autonomy/landing.py`, from where the rover stands), the features
that chance came from, the predictor's digest and the picture's frame, in
`~/.ugv/sessions/NAME-looks.jsonl`. `landed.py` scores them once the world model
has filed the pictures.
"""
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
for path in (os.path.expanduser("~/m5"), os.path.join(HERE, "..", "m5_changes"),
             os.path.expanduser("~/ugv/autonomy"), os.path.join(HERE, "..", "..", "autonomy")):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
import client as client_mod  # noqa: E402
import goals  # noqa: E402
import landing  # noqa: E402
import session  # noqa: E402
import situation as situation_mod  # noqa: E402

LOOKS_PER_STOP = 6
MIN_VISITS = 2
LOOK_WAIT_S = 15.0


def eligible(here, x, y):
    """The records a look from (x, y) could be aimed at, with what the
    predictor says of each."""
    reach = here.reach
    every = list(here.body.get("entities") or [])
    out = []
    for e in here.entities:
        p = e.get("placement") or {}
        if e.get("kind") != "record" or p.get("x_m") is None or p.get("height_m") is None:
            continue
        if int(p.get("viewpoints") or 0) < MIN_VISITS:
            continue
        rng = math.hypot(float(p["x_m"]) - x, float(p["y_m"]) - y)
        if not goals.BAND_NEAR_M <= rng <= goals.BAND_FAR_M:
            continue
        if reach is not None and not reach.clear_line(x, y, float(p["x_m"]), float(p["y_m"]),
                                                      ignore_within_m=goals.SURFACE_M):
            continue
        got = goals._from_viewpoint(p, e, x, y)
        if got["in_depth_view"] is False:
            continue
        near = landing.neighbours(e, every)
        chance, source = landing.lands(e, near, here.at, rng, False, goals.RECORD_LOOK_LANDS)
        seen = e.get("last_seen_at")
        filed = goals._from_viewpoint(p, e, x, y, lands=1.0)
        out.append({"record": e["id"], "x_m": p["x_m"], "y_m": p["y_m"],
                    "tilt_deg": got["look_tilt_deg"], "lands": round(chance, 4), "lands_from": source,
                    # What the planner's arithmetic says the look gains if it
                    # lands: the old ranking is by this, the new by this times
                    # the chance (landed.py's comparison at each stop).
                    "gain_if_filed_m": filed["gain_m"],
                    "visits": p.get("viewpoints"), "spread_m": p.get("uncertainty_m"),
                    "rival": near["rival"], "crowd": near["crowd"], "nearest_m": round(near["nearest_m"], 3),
                    "since_h": None if seen is None else round((here.at - seen) / 3600.0, 3),
                    "range_m": round(rng, 3)})
    return out


def read():
    return situation_mod.Situation.read(client_mod.ReadOnly())


def looks_at_stop(name, step, label, log, dry=False, at=None):
    here = read()
    if at is None:
        pose = here.where
        if pose is None:
            session.note(name, step, looks=label, error="no pose")
            return
        x, y = pose
    else:
        x, y = at
    pool = eligible(here, x, y)
    rng = random.Random("%s/%s/%s" % (name, step, label))
    chosen = rng.sample(pool, min(LOOKS_PER_STOP, len(pool)))
    if dry:
        print("%-11s %2d in reach, looking at %s" % (label, len(pool), ", ".join(
            "%s (%.2f)" % (c["record"], c["lands"]) for c in chosen)))
        return chosen
    for c in chosen:
        args = {"aim_at": {"x_m": c["x_m"], "y_m": c["y_m"]}, "wait": True,
                "wait_s": LOOK_WAIT_S, "settle": False}
        if c["tilt_deg"] is not None:
            args["tilt_deg"] = c["tilt_deg"]
        t = time.time()
        r = session.call("world_inspect", args, timeout=40)
        row = dict(c, step=step, stop=label, t=round(t, 2), frame=r.get("frame_id"),
                   error=r.get("error"), model=landing.digest(), took_s=round(time.time() - t, 1))
        log.write(json.dumps(row) + "\n")
        log.flush()
    session.note(name, step, looks=label, taken=len(chosen), in_reach=len(pool))
    return chosen


def loop(name, which, log):
    """session.loop, with the looks in place of its sweep."""
    step = "loop " + which
    p, _ = session.where()
    views = list(session.VIEWS)
    if p and math.hypot(p["x_m"] - views[-1][1], p["y_m"] - views[-1][2]) < \
            math.hypot(p["x_m"] - views[0][1], p["y_m"] - views[0][2]):
        views.reverse()
    session.note(name, step, begin=[v[0] for v in views], battery=session.battery())
    lost = 0
    for label, x, y in views:
        if session.stopped(name) or session.low_battery(name, step):
            break
        p, _ = session.where()
        if not p or math.hypot(p["x_m"] - x, p["y_m"] - y) > 0.3:
            session.face(name, step, x, y)
            session.drive(name, step, label, x, y)
        looks_at_stop(name, step, label, log)
        check = session.pose_check()
        session.note(name, step, viewpoint=label, **check)
        lost = lost + 1 if session.disagrees(check) else 0
        if lost >= session.LOST_STOPS:
            session.note(name, step, stop="the position check disagreed at %d stops in a row" % lost)
            open(session.path_of(name, ".stop"), "w").close()
            break
    session.note(name, step, end=True, battery=session.battery())


def main():
    what, name = sys.argv[1], sys.argv[2]
    if what == "dry":
        total = 0
        for label, x, y in session.VIEWS:
            total += len(looks_at_stop(name, "dry", label, None, dry=True, at=(x, y)))
        print("%d looks a loop; the predictor is %s" % (total, landing.digest()))
        return
    if what != "run":
        raise SystemExit(__doc__)
    session.start(name)
    with open(session.path_of(name, "-looks.jsonl"), "a") as log:
        loop(name, "A", log)
        if not session.stopped(name):
            loop(name, "B", log)
    session.home(name)


if __name__ == "__main__":
    main()
