"""Step 3: what changed, as events that join the lidar's geometry to the camera's
appearance.

- **Geometry** is the change watch the rover runs (ros_nav/change_watch.py),
  replayed from the drive recordings the way change_node feeds it: every third
  scan, none taken turning faster than 40 degrees a second, a report every 30 s
  and only what is news logged. GONE is a place solid before and seen through
  now; NEW the other way round.
- **Appearance** is the what stream of step 1. A NEW place takes the regions
  standing at it (depth or lidar place, streams3.places) in the visit; a GONE
  place takes the remembered regions that stood at it before the cutoff.

The events:
- **moved**: a GONE and a NEW in one visit whose regions look alike, at least
  PAIR_COS, within PAIR_M of each other.
- **new thing**: a NEW whose regions are new to the house (what over its
  threshold in two looks), or, with no NEW near it, a group of regions new to
  the house in two looks at one place (things the lidar plane misses or calls
  small).
- **arrived** / **gone**: the rest of the watch's furniture-sized NEW and GONE,
  named by their regions where they have any.
Small watch changes are reported only when the camera confirms a new thing at
them; on their own they are the noise the watch already leaves out.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for _where in (os.path.join(HERE, "..", "..", "ros_nav"),):
    sys.path.insert(0, _where)
sys.path.insert(0, os.path.join(HERE, "..", "m5_changes"))
import change_watch as cw  # noqa: E402
from arm_replay import grid_of  # noqa: E402
from change_detect import compose, interp_pose, last_pose  # noqa: E402

import streams  # noqa: E402
import streams3  # noqa: E402

EVERY = 3
MAX_TURN_DPS = 40.0
REPORT_EVERY_S = 30.0
#: Regions stand at a watch change when within this of its middle, plus half
#: its extent.
ATTACH_M = 0.5
#: A GONE and a NEW are one thing moved when their regions look this alike and
#: they are this close (set on development; see step3.md).
PAIR_COS = 0.7
PAIR_M = 4.0
#: Regions new to the house are one new thing when they look this alike and
#: stand within GROUP_M of each other.
GROUP_COS = 0.5
GROUP_M = 0.6
#: A camera-only new thing this near a watch NEW is that NEW's.
NEAR_NEW_M = 0.6
#: Changes of one kind logged within this of each other in a visit are one
#: change (the watch logs a place again when it grows to furniture size).
SAME_CHANGE_M = 0.5
#: A GONE and a NEW nearer than this are a thing that changed in place, not
#: one that moved.
MIN_MOVE_M = 0.5


def merged(changes):
    """One change per place and kind: the largest of those within SAME_CHANGE_M."""
    out = []
    for c in sorted(changes, key=lambda c: -c["extent_m"]):
        if not any(o["kind"] == c["kind"] and
                   math.hypot(o["x_m"] - c["x_m"], o["y_m"] - c["y_m"]) <= SAME_CHANGE_M
                   for o in out):
            out.append(c)
    return out


def replay(recordings, grid_json, skip=()):
    """Every change the watch would have logged, in time order, over the
    recordings fed oldest first; scans inside a (start, end) in skip are not
    fed (a window with the pose known to be wrong)."""
    import json
    bounds = (-24.0, -22.0, 400, 360)
    watch = cw.ChangeWatch(*bounds)
    if grid_json:
        watch.set_map(*grid_of(json.load(open(grid_json))))
    logged, previous, last_report = [], [], None
    for rec in sorted(recordings, key=lambda r: r.start):
        meta, ranges = rec.meta, rec.ranges
        last_yaw, seen = None, 0
        for k in range(len(ranges)):
            t = float(meta[k][0])
            if any(a <= t <= b for a, b in skip):
                continue
            seen += 1
            if last_report is None:
                last_report = t
            if t - last_report >= REPORT_EVERY_S:
                last_report = t
                now = watch.changes()
                for group in cw.ChangeWatch.news(now, previous, logged, t):
                    logged.append(dict(group, logged_at=t))
                previous = now
            if seen % EVERY:
                continue
            ob, mo = interp_pose(rec.odom, t), last_pose(rec.map, t)
            if ob is None or mo is None:
                continue
            x, y, yaw = compose(mo, ob)
            if last_yaw is not None:
                then, was = last_yaw
                gap = t - then
                if gap > 0 and abs(math.degrees(math.atan2(
                        math.sin(yaw - was), math.cos(yaw - was)))) / gap > MAX_TURN_DPS:
                    last_yaw = (t, yaw)
                    continue
            last_yaw = (t, yaw)
            r = ranges[k]
            watch.add_scan(x, y, yaw, r, meta[k][2] + meta[k][3] * np.arange(len(r)), t)
    return logged


def _near(xs, ys, x, y, r):
    return np.flatnonzero(np.isfinite(xs) & (np.hypot(xs - x, ys - y) <= r))


def build(changes, query, memory, recs, t_what):
    """The events of one visit: watch changes logged in it (`changes`), the
    visit's regions (`query`) and memory before its cutoff."""
    changes = merged(changes)
    what = streams.what_scores(query, memory)
    fl = query.floor_sized()
    qx, qy, _src = streams3.places(query, recs)
    mx, my, _msrc = streams3.places(memory, recs)
    novel = fl & (what > t_what)
    events, used_novel = [], set()
    news = [c for c in changes if c["kind"] == "NEW"]
    gones = [c for c in changes if c["kind"] == "GONE"]
    for c in news:
        rows = [i for i in _near(qx, qy, c["x_m"], c["y_m"], ATTACH_M + c["extent_m"] / 2) if fl[i]]
        nov = [i for i in rows if novel[i]]
        c["regions"] = [int(query.id[i]) for i in rows]
        c["novel_looks"] = len({query.frame[i] for i in nov})
        c["_vec"] = query.vec[rows] if rows else np.zeros((0, query.vec.shape[1]))
        used_novel.update(nov)
    for c in gones:
        rows = _near(mx, my, c["x_m"], c["y_m"], ATTACH_M + c["extent_m"] / 2)
        rows = rows[np.argsort(-memory.t[rows])][:20]
        c["remembered"] = [int(memory.id[i]) for i in rows]
        c["_vec"] = memory.vec[rows] if len(rows) else np.zeros((0, memory.vec.shape[1]))
    # Moved: the most alike GONE for each NEW, each used once.
    pairs, taken = [], set()
    cand = []
    for a, n in enumerate(news):
        for b, g in enumerate(gones):
            if len(n["_vec"]) == 0 or len(g["_vec"]) == 0:
                continue
            d = math.hypot(n["x_m"] - g["x_m"], n["y_m"] - g["y_m"])
            if d > PAIR_M or d < MIN_MOVE_M:
                continue
            cos = float((n["_vec"] @ g["_vec"].T).max())
            if cos >= PAIR_COS:
                cand.append((cos, a, b))
    for cos, a, b in sorted(cand, reverse=True):
        if ("n", a) in taken or ("g", b) in taken:
            continue
        if news[a]["novel_looks"] >= 2:
            continue
        taken |= {("n", a), ("g", b)}
        pairs.append((a, b, cos))
    for a, b, cos in pairs:
        n, g = news[a], gones[b]
        events.append({"kind": "moved", "from": [g["x_m"], g["y_m"]], "to": [n["x_m"], n["y_m"]],
                       "x_m": n["x_m"], "y_m": n["y_m"], "cos": round(cos, 3),
                       "large": bool(n["large"] or g["large"]),
                       "regions": n["regions"], "remembered": g["remembered"][:5]})
    for a, n in enumerate(news):
        if ("n", a) in taken:
            continue
        if n["novel_looks"] >= 2:
            events.append({"kind": "new thing", "x_m": n["x_m"], "y_m": n["y_m"],
                           "extent_m": n["extent_m"], "large": n["large"], "by": "lidar and camera",
                           "regions": n["regions"]})
        elif n["large"]:
            events.append({"kind": "arrived", "x_m": n["x_m"], "y_m": n["y_m"],
                           "extent_m": n["extent_m"], "large": True, "regions": n["regions"]})
    for b, g in enumerate(gones):
        if ("g", b) in taken or not g["large"]:
            continue
        events.append({"kind": "gone", "x_m": g["x_m"], "y_m": g["y_m"], "extent_m": g["extent_m"],
                       "large": True, "remembered": g["remembered"][:5]})
    # New things the watch did not log: regions new to the house in two looks
    # at one place, away from every watch NEW.
    rest = [i for i in np.flatnonzero(novel & np.isfinite(qx)) if i not in used_novel
            and not any(math.hypot(qx[i] - n["x_m"], qy[i] - n["y_m"]) <= NEAR_NEW_M for n in news)]
    groups = []
    for i in rest:
        for gr in groups:
            if any(query.vec[i] @ query.vec[j] >= GROUP_COS and
                   math.hypot(qx[i] - qx[j], qy[i] - qy[j]) <= GROUP_M for j in gr):
                gr.append(i)
                break
        else:
            groups.append([i])
    for gr in groups:
        if len({query.frame[i] for i in gr}) < 2:
            continue
        events.append({"kind": "new thing", "x_m": round(float(np.mean(qx[gr])), 2),
                       "y_m": round(float(np.mean(qy[gr])), 2), "large": False, "by": "camera",
                       "regions": [int(query.id[i]) for i in gr]})
    for c in changes:
        c.pop("_vec", None)
    return events, changes
