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


#: A thing's evidence is persistent across looks this far apart, as the
#: watch's own (change_watch.PERSIST_S).
PERSIST_S = 20.0
#: What stood at a place before: the regions within HERE_M of it in the
#: newest HERE_LOOKS remembered looks that viewed it (camera within HERE_VIEW_M,
#: the place within HERE_HALF_DEG of the camera's axis). A NEW is a new thing
#: when its regions are unlike all of them; a thing brought back from another
#: room is new here even if the house has seen one like it.
HERE_LOOKS = 5
HERE_M = 1.0
HERE_VIEW_M = 4.0
HERE_HALF_DEG = 45.0
#: Regions are attached to a NEW from this long before the watch first saw it
#: until a GONE is first seen at the same place (or the visit ends), so that
#: successive rearrangements in one visit stay apart.
ATTACH_S = 60.0


def merged(changes):
    """One change per place, kind and visit: the largest of those within
    SAME_CHANGE_M whose evidence overlaps in time or falls within the watch's
    visit gap of each other (a place logged again when it grows)."""
    out = []
    for c in sorted(changes, key=lambda c: (-c["extent_m"], c["logged_at"])):
        if not any(o["kind"] == c["kind"] and
                   math.hypot(o["x_m"] - c["x_m"], o["y_m"] - c["y_m"]) <= SAME_CHANGE_M and
                   c["seen_from"] <= o["seen_until"] + cw.VISIT_GAP_S and
                   o["seen_from"] <= c["seen_until"] + cw.VISIT_GAP_S
                   for o in out):
            out.append(c)
    return out


def replay(recordings, grid_json, skip=()):
    """Every change the watch would have logged, in time order, over the
    recordings fed oldest first. Scans inside a (start, end) in skip are not
    fed (a window with the pose known to be wrong). An approximation of the
    live watch: navigation's own trust in the pose is not in a recording, and
    the history starts with the first recording given."""
    import json
    watch = cw.ChangeWatch(-24.0, -22.0, 400, 360)
    if grid_json:
        watch.set_map(*grid_of(json.load(open(grid_json))))
    logged, previous, last_report = [], [], None
    received = 0

    def report(t):
        nonlocal previous
        now = watch.changes()
        for group in cw.ChangeWatch.news(now, previous, logged, t):
            logged.append(dict(group, logged_at=t))
        previous = now

    for rec in sorted(recordings, key=lambda r: r.start):
        meta, ranges = rec.meta, rec.ranges
        last_yaw, t = None, None
        for k in range(len(ranges)):
            t = float(meta[k][0])
            if any(a <= t <= b for a, b in skip):
                continue
            received += 1
            if last_report is None:
                last_report = t
            if t - last_report >= REPORT_EVERY_S:
                last_report = t
                report(t)
            if received % EVERY:
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
        if t is not None:
            # The node's timer keeps reporting after the last scan.
            last_report = t
            report(t)
    return logged


def _near(xs, ys, x, y, r):
    return np.flatnonzero(np.isfinite(xs) & (np.hypot(xs - x, ys - y) <= r))


def persistent(query, rows):
    """Two or more looks at least PERSIST_S apart among these regions."""
    if len(rows) < 2:
        return False
    times = sorted({float(query.shutter[i]) for i in rows})
    return times[-1] - times[0] >= PERSIST_S


def _components(rows, linked):
    """Connected components of rows under a symmetric `linked(i, j)`, in a
    stable order whatever order the rows came in."""
    rows = sorted(rows)
    parent = {i: i for i in rows}

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for a, i in enumerate(rows):
        for j in rows[a + 1:]:
            if linked(i, j):
                parent[root(j)] = root(i)
    groups = {}
    for i in rows:
        groups.setdefault(root(i), []).append(i)
    return sorted(groups.values(), key=lambda g: g[0])


class Before:
    """Remembered looks, for asking what the latest views of a place showed."""

    def __init__(self, memory, mx, my):
        frames, first = np.unique(memory.frame, return_index=True)
        posed = memory.posed[first]
        self.frames = frames[posed]
        first = first[posed]
        self.t = memory.shutter[first]
        self.cx, self.cy, self.yaw = memory.cx[first], memory.cy[first], memory.yaw[first]
        self.rows = {f: np.flatnonzero(memory.frame == f) for f in self.frames}
        self.mx, self.my = mx, my

    def here(self, x, y):
        """Rows of what the newest looks that viewed (x, y) had within HERE_M of it."""
        d = np.hypot(self.cx - x, self.cy - y)
        b = np.degrees(np.arctan2(y - self.cy, x - self.cx))
        off = np.abs((b - self.yaw + 180.0) % 360.0 - 180.0)
        viewed = np.flatnonzero((d <= HERE_VIEW_M) & (off <= HERE_HALF_DEG))
        newest = viewed[np.argsort(-self.t[viewed])][:HERE_LOOKS]
        out = []
        for k in newest:
            rows = self.rows[self.frames[k]]
            near = np.isfinite(self.mx[rows]) & (np.hypot(self.mx[rows] - x, self.my[rows] - y) <= HERE_M)
            out += list(rows[near])
        return out


def build(changes, query, memory, recs, t_what):
    """The events of one visit: watch changes logged in it (`changes`), the
    visit's regions (`query`) and memory before its cutoff."""
    changes = merged(changes)
    what = streams.what_scores(query, memory)
    fl = query.floor_sized()
    qx, qy, _src = streams3.places(query, recs)
    mx, my, _msrc = streams3.places(memory, recs)
    before = Before(memory, mx, my)
    novel = fl & (what > t_what)
    events, used_novel = [], set()
    news = [c for c in changes if c["kind"] == "NEW"]
    gones = [c for c in changes if c["kind"] == "GONE"]
    end_of_visit = float(query.shutter.max()) if query.n else 0.0
    for c in news:
        later = [g["seen_from"] for g in gones if g["seen_from"] > c["seen_from"] and
                 math.hypot(g["x_m"] - c["x_m"], g["y_m"] - c["y_m"]) <= SAME_CHANGE_M]
        until = min(later) if later else end_of_visit
        when = (query.shutter >= c["seen_from"] - ATTACH_S) & (query.shutter <= until)
        rows = [i for i in _near(qx, qy, c["x_m"], c["y_m"], ATTACH_M + c["extent_m"] / 2)
                if fl[i] and when[i]]
        # New here: regions unlike everything the latest views of the place
        # showed, alike to one another, persistent.
        was = before.here(c["x_m"], c["y_m"])
        if was:
            here = 1.0 - (query.vec[rows] @ memory.vec[was].T).max(axis=1) if rows else np.zeros(0)
        else:
            here = np.ones(len(rows))
        alike = lambda i, j: query.vec[i] @ query.vec[j] >= GROUP_COS
        nov = [i for i, h in zip(rows, here) if h > t_what]
        best = max(_components(nov, alike), key=len, default=[])
        c["rows"] = rows
        # New to the whole house: such a thing cannot have moved here.
        house = max(_components([i for i in rows if novel[i]], alike), key=len, default=[])
        c["novel_house"] = persistent(query, house)
        # A furniture-sized arrival is new when new to this place; a small one,
        # whose geometry is weak evidence, only when new to the whole house.
        c["novel"] = persistent(query, best) if c["large"] else c["novel_house"]
        used_novel.update(i for i in rows if novel[i])
    for c in gones:
        # What stood there: the newest remembered regions at the place, and
        # anything seen there earlier in this visit (a thing that arrived and
        # left again within it).
        rows = _near(mx, my, c["x_m"], c["y_m"], ATTACH_M + c["extent_m"] / 2)
        c["mrows"] = [("m", int(i)) for i in rows[np.argsort(-memory.t[rows])][:20]]
        seen = [int(i) for i in _near(qx, qy, c["x_m"], c["y_m"], ATTACH_M + c["extent_m"] / 2)
                if fl[i] and query.shutter[i] < c["seen_from"] - ATTACH_S]
        c["mrows"] += [("q", i) for i in seen]
    # Moved: a GONE and a NEW whose evidence looks alike in two looks at least
    # PERSIST_S apart, one of them furniture-sized. Each region, seen or
    # remembered, supports one move only.
    used_q, used_m, taken, pairs = set(), set(), set(), []
    while True:
        best = None
        for a, n in enumerate(news):
            if ("n", a) in taken or n["novel_house"]:
                continue
            qrows = [i for i in n["rows"] if i not in used_q]
            for b, g in enumerate(gones):
                if ("g", b) in taken or not (n["large"] or g["large"]):
                    continue
                d = math.hypot(n["x_m"] - g["x_m"], n["y_m"] - g["y_m"])
                if d > PAIR_M or d < MIN_MOVE_M:
                    continue
                mrows = [r for r in g["mrows"] if r not in used_m and r[1] not in used_q]
                if not qrows or not mrows:
                    continue
                gvec = np.stack([(memory if src == "m" else query).vec[i] for src, i in mrows])
                sims = query.vec[qrows] @ gvec.T
                support = [qrows[k] for k in np.flatnonzero(sims.max(axis=1) >= PAIR_COS)]
                if not persistent(query, support):
                    continue
                cos = float(sims.max())
                if best is None or cos > best[0]:
                    best = (cos, a, b, support,
                            [mrows[k] for k in np.flatnonzero(sims.max(axis=0) >= PAIR_COS)])
        if best is None:
            break
        cos, a, b, support, remembered = best
        taken |= {("n", a), ("g", b)}
        used_q.update(support)
        used_m.update(remembered)
        pairs.append((a, b, cos, support, remembered))
    for a, b, cos, support, remembered in pairs:
        n, g = news[a], gones[b]
        events.append({"kind": "moved", "from": [g["x_m"], g["y_m"]], "to": [n["x_m"], n["y_m"]],
                       "x_m": n["x_m"], "y_m": n["y_m"], "cos": round(cos, 3),
                       "large": bool(n["large"] or g["large"]),
                       "regions": [int(query.id[i]) for i in support],
                       "remembered": [int((memory if src == "m" else query).id[i])
                                      for src, i in remembered[:5]]})
    for a, n in enumerate(news):
        if ("n", a) in taken:
            continue
        ids = [int(query.id[i]) for i in n["rows"]]
        if n["novel"]:
            events.append({"kind": "new thing", "x_m": n["x_m"], "y_m": n["y_m"],
                           "extent_m": n["extent_m"], "large": n["large"], "by": "lidar and camera",
                           "regions": ids})
        elif n["large"]:
            events.append({"kind": "arrived", "x_m": n["x_m"], "y_m": n["y_m"],
                           "extent_m": n["extent_m"], "large": True, "regions": ids})
    for b, g in enumerate(gones):
        if ("g", b) in taken or not g["large"]:
            continue
        events.append({"kind": "gone", "x_m": g["x_m"], "y_m": g["y_m"], "extent_m": g["extent_m"],
                       "large": True, "remembered": [int((memory if src == "m" else query).id[i])
                                                     for src, i in g["mrows"][:5]]})
    # New things the watch did not log: regions new to the house, alike and at
    # one place, in looks at least PERSIST_S apart, away from every watch NEW.
    rest = [int(i) for i in np.flatnonzero(novel & np.isfinite(qx)) if i not in used_novel
            and not any(math.hypot(qx[i] - n["x_m"], qy[i] - n["y_m"]) <= NEAR_NEW_M for n in news)]
    groups = _components(rest, lambda i, j: query.vec[i] @ query.vec[j] >= GROUP_COS and
                         math.hypot(qx[i] - qx[j], qy[i] - qy[j]) <= GROUP_M)
    for gr in groups:
        if not persistent(query, gr):
            continue
        events.append({"kind": "new thing", "x_m": round(float(np.mean(qx[gr])), 2),
                       "y_m": round(float(np.mean(qy[gr])), 2), "large": False, "by": "camera",
                       "regions": [int(query.id[i]) for i in gr]})
    for c in changes:
        for k in ("rows", "mrows"):
            c.pop(k, None)
    return events, changes
