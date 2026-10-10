"""What has changed since the rover last saw a place, from the lidar alone.

**Why the lidar and not the camera.** The world state's looks were measured for
this on 2026-10-10 and cannot do it: an aimed look that filed nothing, the only
"it is not there" the rover had, came back that way on half the looks at things
that had not moved, and no looser test told the thing from another object in
its place (docs/progress/2026-10-10-m5-what-reports-a-change.md). The scan does
not have to recognise anything. A beam that passes through where something
solid used to be says it has gone, and one that stops where the floor used to
be clear says something has arrived; the armchair moved that day was found
that way, at its record's position, in every comparison that spanned the move.

**What is compared with what.** Every 5 cm cell of the map frame keeps two
tallies, one for the visit in progress and one for the last visit before it: in
how many scans a beam ended in the cell (a hit) and in how many one passed
through it. A visit to a cell ends when the cell has gone `VISIT_GAP_S` without
being seen, so a place the rover left an hour ago and comes back to is compared
with what it was then, and a room it is sitting in is not compared with itself.
A cell is solid when at least `MIN_OBS` scans saw it and `SOLID` of them hit
it, and clear when no more than `CLEAR` did. A change is a cell solid in one
visit and clear in the other, with `SLACK_CELLS` of slack for the pose: nothing
next to something the other visit also had counts. Changed cells are grouped,
and a group is reported only once its evidence spans `PERSIST_S`, so a person
walking past is not a change, though one who stands still for a while is.

**What it does not do.** It moves nothing and decides nothing. Saying which
thing a change is -- the armchair, a door -- is for whoever reads the report
and knows where things are; the clusters carry where and how big. Things above
or below the lidar's plane, about 20 cm up, are invisible to it: paintings,
most of a table, shoes.
"""
import json
import math
import os
import time

import numpy as np

RES_M = 0.05
#: How long a cell has to go unseen for the next sighting to be a new visit.
#: Long enough that a run moving about one room keeps one visit of it; short
#: enough that the next run, or the owner's next arrangement, is a new one.
VISIT_GAP_S = 300.0
MIN_OBS = 6
SOLID = 0.6
CLEAR = 0.1
SLACK_CELLS = 2
#: A group of changed cells is reported once the visit's evidence for it spans
#: this long. On the recordings of 2026-10-10, 20 s kept the armchair and dropped
#: everything a person walking past left.
PERSIST_S = 20.0
MIN_CELLS = 6
#: Groups this long or longer are the size of furniture; smaller ones on those
#: recordings were doors in the wall lines.
LARGE_M = 0.6
MIN_RANGE_M = 0.12
MAX_RANGE_M = 6.0
STEP_M = RES_M / 2.0


def _dilate(mask, cells):
    """`mask` grown by `cells` in every direction, square, without scipy."""
    out = mask.copy()
    for _ in range(cells):
        grown = out.copy()
        grown[1:, :] |= out[:-1, :]
        grown[:-1, :] |= out[1:, :]
        grown[:, 1:] |= out[:, :-1]
        grown[:, :-1] |= out[:, 1:]
        grown[1:, 1:] |= out[:-1, :-1]
        grown[1:, :-1] |= out[:-1, 1:]
        grown[:-1, 1:] |= out[1:, :-1]
        grown[:-1, :-1] |= out[1:, 1:]
        out = grown
    return out


def _groups(mask):
    """Eight-connected groups of the True cells, as lists of (row, column)."""
    todo = set(zip(*np.nonzero(mask)))
    groups = []
    while todo:
        seed = todo.pop()
        group, edge = [seed], [seed]
        while edge:
            r, c = edge.pop()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nb = (r + dr, c + dc)
                    if nb in todo:
                        todo.remove(nb)
                        group.append(nb)
                        edge.append(nb)
        groups.append(group)
    return groups


class ChangeWatch:
    """The two visits' tallies over a fixed patch of the map frame."""

    def __init__(self, x0, y0, nx, ny, res=RES_M):
        self.x0, self.y0, self.nx, self.ny, self.res = (
            float(x0), float(y0), int(nx), int(ny), float(res))
        shape = (self.ny, self.nx)
        self.cur_hits = np.zeros(shape, np.int32)
        self.cur_pass = np.zeros(shape, np.int32)
        self.cur_first = np.full(shape, np.nan)
        self.cur_last = np.full(shape, -np.inf)
        self.ref_hits = np.zeros(shape, np.int32)
        self.ref_pass = np.zeros(shape, np.int32)
        self.ref_time = np.full(shape, np.nan)
        self.scans = 0

    @classmethod
    def around(cls, xmin, ymin, xmax, ymax, margin_m=3.0, res=RES_M):
        """A watch covering a box of the map frame with room to spare."""
        x0, y0 = xmin - margin_m, ymin - margin_m
        return cls(x0, y0, math.ceil((xmax - xmin + 2 * margin_m) / res),
                   math.ceil((ymax - ymin + 2 * margin_m) / res), res)

    def covers(self, xmin, ymin, xmax, ymax):
        return (xmin >= self.x0 and ymin >= self.y0
                and xmax <= self.x0 + self.nx * self.res
                and ymax <= self.y0 + self.ny * self.res)

    def grown_to(self, xmin, ymin, xmax, ymax, margin_m=3.0):
        """A watch over this one's patch and the box given, tallies carried over.

        For a map that has grown past the patch the watch was made for: what was
        learned about the old patch keeps its cells, on the same grid."""
        x0 = min(self.x0, xmin - margin_m)
        y0 = min(self.y0, ymin - margin_m)
        x0 = self.x0 - math.ceil((self.x0 - x0) / self.res) * self.res
        y0 = self.y0 - math.ceil((self.y0 - y0) / self.res) * self.res
        x1 = max(self.x0 + self.nx * self.res, xmax + margin_m)
        y1 = max(self.y0 + self.ny * self.res, ymax + margin_m)
        bigger = ChangeWatch(x0, y0, math.ceil((x1 - x0) / self.res - 1e-9),
                             math.ceil((y1 - y0) / self.res - 1e-9), self.res)
        dx = int(round((self.x0 - x0) / self.res))
        dy = int(round((self.y0 - y0) / self.res))
        for name in ("cur_hits", "cur_pass", "cur_first", "cur_last",
                     "ref_hits", "ref_pass", "ref_time"):
            getattr(bigger, name)[dy:dy + self.ny, dx:dx + self.nx] = getattr(self, name)
        bigger.scans = self.scans
        return bigger

    def _cells(self, xs, ys):
        cx = np.floor((xs - self.x0) / self.res).astype(np.int64)
        cy = np.floor((ys - self.y0) / self.res).astype(np.int64)
        inside = (cx >= 0) & (cx < self.nx) & (cy >= 0) & (cy < self.ny)
        return np.unique(cy[inside] * self.nx + cx[inside])

    def add_scan(self, x, y, heading, ranges, angles, t):
        """One scan, taken from map pose (x, y, heading in radians) at time t.

        `angles` are each beam's direction in the rover's frame. Each cell is
        counted once per scan however many beams fell in it, so that a near wall
        does not outvote a far one."""
        r = np.asarray(ranges, dtype=float)
        a = np.asarray(angles, dtype=float)
        ok = np.isfinite(r) & (r > MIN_RANGE_M) & (r < MAX_RANGE_M)
        r, a = r[ok], a[ok] + heading
        if r.size == 0:
            return
        hits = self._cells(x + r * np.cos(a), y + r * np.sin(a))
        steps = np.arange(int(np.max(r) / STEP_M) + 1) * STEP_M + STEP_M
        inside = steps[None, :] < (r - self.res)[:, None]
        px = x + steps[None, :] * np.cos(a)[:, None]
        py = y + steps[None, :] * np.sin(a)[:, None]
        passes = self._cells(px[inside], py[inside])
        touched = np.union1d(hits, passes)
        # A cell not seen for VISIT_GAP_S starts a new visit, and the one it
        # ends becomes what the next is compared with -- if it saw enough.
        lapsed = touched[(t - self.cur_last.flat[touched]) > VISIT_GAP_S]
        if lapsed.size:
            seen = self.cur_hits.flat[lapsed] + self.cur_pass.flat[lapsed]
            judged = lapsed[seen >= MIN_OBS]
            self.ref_hits.flat[judged] = self.cur_hits.flat[judged]
            self.ref_pass.flat[judged] = self.cur_pass.flat[judged]
            self.ref_time.flat[judged] = self.cur_last.flat[judged]
            self.cur_hits.flat[lapsed] = 0
            self.cur_pass.flat[lapsed] = 0
            self.cur_first.flat[lapsed] = t
        self.cur_hits.flat[hits] += 1
        self.cur_pass.flat[passes] += 1
        self.cur_last.flat[touched] = t
        self.scans += 1

    def _states(self, hits, passes):
        seen = hits + passes
        share = hits / np.maximum(seen, 1)
        judged = seen >= MIN_OBS
        return judged & (share >= SOLID), judged & (share <= CLEAR)

    def changes(self, large_only=False):
        """What differs between each cell's last visit and its present one.

        A list of groups, largest first: `kind` GONE (solid then, seen through
        now) or NEW (the other way round), where, how big, when the present
        evidence was gathered and when the place was last seen otherwise."""
        was_solid, was_clear = self._states(self.ref_hits, self.ref_pass)
        is_solid, is_clear = self._states(self.cur_hits, self.cur_pass)
        found = []
        for kind, mask in (
                ("GONE", was_solid & is_clear & ~_dilate(is_solid, SLACK_CELLS)),
                ("NEW", is_solid & was_clear & ~_dilate(was_solid, SLACK_CELLS))):
            for group in _groups(mask):
                if len(group) < MIN_CELLS:
                    continue
                rows = np.array([g[0] for g in group])
                cols = np.array([g[1] for g in group])
                first = float(np.nanmin(self.cur_first[rows, cols]))
                last = float(np.max(self.cur_last[rows, cols]))
                if last - first < PERSIST_S:
                    continue
                extent = self.res * max(np.ptp(cols) + 1, np.ptp(rows) + 1)
                if large_only and extent < LARGE_M:
                    continue
                found.append({
                    "kind": kind,
                    "x_m": round(self.x0 + (cols.mean() + 0.5) * self.res, 2),
                    "y_m": round(self.y0 + (rows.mean() + 0.5) * self.res, 2),
                    "extent_m": round(float(extent), 2),
                    "area_m2": round(len(group) * self.res * self.res, 3),
                    "large": bool(extent >= LARGE_M),
                    "seen_from": round(first, 1),
                    "seen_until": round(last, 1),
                    "was_otherwise_at": round(float(np.nanmax(self.ref_time[rows, cols])), 1),
                })
        found.sort(key=lambda g: -g["area_m2"])
        return found

    @staticmethod
    def first_seen(found, already, near_m=0.3):
        """The groups in `found` not already reported: no group in `already` of
        the same kind within `near_m`. A group grows and shifts a little as the
        visit goes on, and the same armchair must not be news every report."""
        fresh = []
        for g in found:
            if not any(o["kind"] == g["kind"]
                       and math.hypot(o["x_m"] - g["x_m"], o["y_m"] - g["y_m"]) <= near_m
                       for o in already):
                fresh.append(g)
        return fresh

    def save(self, path, **meta):
        """Write the tallies, atomically, with whatever names their map."""
        tmp = path + ".tmp.npz"
        np.savez_compressed(
            tmp, cur_hits=self.cur_hits, cur_pass=self.cur_pass,
            cur_first=self.cur_first, cur_last=self.cur_last,
            ref_hits=self.ref_hits, ref_pass=self.ref_pass, ref_time=self.ref_time,
            meta=np.array(json.dumps({
                "x0": self.x0, "y0": self.y0, "nx": self.nx, "ny": self.ny,
                "res": self.res, "scans": self.scans, "saved_at": time.time(),
                **meta})))
        os.replace(tmp, path)

    @classmethod
    def load(cls, path):
        """(watch, meta) from `save`, or (None, None) if there is none."""
        if not os.path.exists(path):
            return None, None
        with np.load(path) as data:
            meta = json.loads(str(data["meta"]))
            watch = cls(meta["x0"], meta["y0"], meta["nx"], meta["ny"], meta["res"])
            for name in ("cur_hits", "cur_pass", "cur_first", "cur_last",
                         "ref_hits", "ref_pass", "ref_time"):
                getattr(watch, name)[...] = data[name]
            watch.scans = int(meta.get("scans", 0))
        return watch, meta
