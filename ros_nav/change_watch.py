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

A visit lasts as long as the place stays in sight, so a thing moved beside the
parked rover leaves its place solid for part of the visit and seen through for
the rest. Such a cell is called what the visit's last two `RECENT_S` windows say
when they differ from the rest of the visit by `FLIP` of the beams, and is
compared with the visit before; the visit's own first half is never the
reference.

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
#: How far a scan's time may run back behind the newest the watch holds -- a
#: network clock correction -- before it is taken for a clock not yet set.
CLOCK_SLACK_S = 60.0
#: A change logged is not logged again, the same kind at the same place, for
#: this long: a run is counted from the log.
RELOG_S = 3600.0
MIN_OBS = 6
SOLID = 0.6
CLEAR = 0.1
SLACK_CELLS = 2
#: A group of changed cells is reported once the visit's evidence for it spans
#: this long. On the recordings of 2026-10-10, 20 s kept the armchair and dropped
#: everything a person walking past left.
PERSIST_S = 20.0
#: **And from places this far apart.** On 2026-10-11 the rover stood 27 s with
#: its pose 10 cm and 3 degrees out, and beams running along a hall wall passed
#: through it: 0.7 m of wall reported gone, every scan behind it taken within
#: 0.30 m. The armchair moved the day before was seen from across the room. From
#: one spot a change is a reason to look again (`changes(unconfirmed=True)`,
#: the report's `unconfirmed`); from two it is reported.
VIEW_SPREAD_M = 0.5
VIEW_ARRAYS = ("hit_x0", "hit_x1", "hit_y0", "hit_y1",
               "pass_x0", "pass_x1", "pass_y0", "pass_y1")
#: **A visit that disagrees with itself is judged on its recent evidence.** A
#: visit lasts as long as a place stays in sight, so a thing moved in front of
#: the parked rover leaves its place solid for part of the visit and seen
#: through for the rest, which is neither, and nothing was reported: on
#: 2026-10-20 the black bean bag was moved 2 m beside the charger with the rover
#: watching, and neither its old place nor its new one was ever logged. So each
#: cell also keeps its last two windows of this length, and a cell the whole
#: visit cannot call solid or clear is called what those windows say.
RECENT_S = 60.0
#: How far apart the share of beams ending in a cell has to be, between its
#: recent windows and the visit before them, for it to have flipped. On
#: 2026-10-20 the black bean bag's new place read 0.10 to 0.25 before the
#: windows, since it arrived a minute before they began, and 0.66 to 1.0 in
#: them.
FLIP = 0.5
RECENT_ARRAYS = ("win_hits", "win_pass", "win_first", "old_hits", "old_pass", "old_first")
MIN_CELLS = 6
#: Groups this long or longer are the size of furniture; smaller ones on those
#: recordings were doors in the wall lines.
LARGE_M = 0.6
MIN_RANGE_M = 0.12
MAX_RANGE_M = 6.0
#: **Only the scan within this range is counted.** Further out a degree and a
#: quarter of heading moves a beam past SLACK_CELLS, and a wall seen edge-on
#: from across a room is "seen through": on 2026-10-11 a corridor wall was
#: reported gone from 4.5 m away. Replayed through both days at 4.5 m, every
#: scan still judged against the map out to MAX_RANGE_M, the controls stayed
#: clean and scene 1 gave up that wall and gained two of its true changes at
#: furniture size, among them where an armchair had stood; at 3.5 m control 1
#: reported a wall gone, at 5 m the corridor wall came back.
TALLY_MAX_M = 4.5
STEP_M = RES_M / 2.0
#: **A scan has to fit what the watch already knows before it is counted.** Of
#: its hits that land where something is already known -- solid, or seen
#: through -- at least FIT_MIN must land on (or SLACK_CELLS from) what is solid.
#: An armchair moved is a small part of a scan; a scan laid at the wrong pose is
#: most of it on floor already seen clear. On 2026-10-10 at 18:06 a restore 170
#: degrees out, with the map still called settled, logged the charger room as
#: changed; navigation's own say-so is not enough.
FIT_MIN = 0.6
#: Fewer known hits than this and there is nothing to judge the scan by: a place
#: seen for the first time is taken as it is.
FIT_MIN_KNOWN = 30
#: How often the masks a scan is judged against are worked out again.
FIT_REFRESH_SCANS = 30
#: **And it has to fit the mapper's map.** The check above judges a scan by what
#: the watch has seen, and a room seen all day is solid near enough everywhere:
#: on 2026-10-11 the rover's pose went 0.5 m and more out entering the bedroom
#: and the watch logged a wall new and two lengths of bed gone. Of the hits
#: beyond MAP_FIT_FAR_M -- walls, mostly; what is near is what moves -- at least
#: MAP_FIT_MIN must land within MAP_FIT_SLACK_CELLS of what the mapper calls
#: occupied. On the two days' recordings good poses fitted at a median of 0.94
#: to 1.0, with 1.5% of scans under 0.7; armchairs moved to beside the rover at
#: 0.93 and above. The bad bedroom pose fitted at 0.63 to 0.67 for the half minute before
#: the mapper's correction and 0.45 after it, and the reports came from the
#: half minute before: a gate at 0.6, as first set, let them through.
MAP_FIT_MIN = 0.7
MAP_FIT_FAR_M = 1.5
MAP_FIT_SLACK_CELLS = 2
MAP_FIT_MIN_HITS = 20


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


def _flipped(hits, passes, recent_hits, recent_passes):
    """(solid, clear): cells whose visit is neither as a whole but whose recent
    windows are the one, and whose share of beams ending in them is FLIP more
    or less than in the visit before those windows -- a place that changed
    while in sight. A visit mixed throughout, as a thin thing hit from one
    place and seen past from another is, is not called anything."""
    def share_of(h, p):
        seen = h + p
        return h / np.maximum(seen, 1), seen >= MIN_OBS
    share, judged = share_of(hits, passes)
    mixed = judged & (share < SOLID) & (share > CLEAR)
    r_share, r_judged = share_of(recent_hits, recent_passes)
    e_share, e_judged = share_of(hits - recent_hits, passes - recent_passes)
    both = mixed & r_judged & e_judged
    return (both & (r_share >= SOLID) & (r_share - e_share >= FLIP),
            both & (r_share <= CLEAR) & (e_share - r_share >= FLIP))


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
        # Where the visit in progress saw each cell from: the box round the
        # rover's positions, for the scans that ended in it and those that
        # passed through it.
        for name in VIEW_ARRAYS:
            setattr(self, name, np.full(shape, np.inf if name.endswith("0") else -np.inf,
                                        np.float32))
        # The last two RECENT_S windows of the visit in progress, the newer
        # first: what a cell the whole visit cannot call is judged on.
        self.win_hits = np.zeros(shape, np.int32)
        self.win_pass = np.zeros(shape, np.int32)
        self.win_first = np.full(shape, np.nan)
        self.old_hits = np.zeros(shape, np.int32)
        self.old_pass = np.zeros(shape, np.int32)
        self.old_first = np.full(shape, np.nan)
        self.scans = 0
        self.refused = 0
        self.off_map = 0
        self._map = None
        self.stale = 0
        self._newest = None
        self._known = None
        self._known_age = 0

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
                     "ref_hits", "ref_pass", "ref_time") + VIEW_ARRAYS + RECENT_ARRAYS:
            getattr(bigger, name)[dy:dy + self.ny, dx:dx + self.nx] = getattr(self, name)
        bigger.scans = self.scans
        bigger._map = self._map
        return bigger

    def set_map(self, occupied, x0, y0, res):
        """The mapper's occupancy to judge scans against (MAP_FIT_MIN): a boolean
        grid, rows along y, whose cell (0, 0) has its corner at (x0, y0)."""
        self._map = (_dilate(np.asarray(occupied, bool), MAP_FIT_SLACK_CELLS),
                     float(x0), float(y0), float(res))

    def _on_map(self, xs, ys):
        """Whether these far hits fit the mapper's map; True with no map or too
        few of them to judge."""
        if self._map is None:
            return True
        near, x0, y0, res = self._map
        cx = np.floor((xs - x0) / res).astype(np.int64)
        cy = np.floor((ys - y0) / res).astype(np.int64)
        inside = (cx >= 0) & (cx < near.shape[1]) & (cy >= 0) & (cy < near.shape[0])
        if inside.sum() < MAP_FIT_MIN_HITS:
            return True
        return near[cy[inside], cx[inside]].mean() >= MAP_FIT_MIN

    def _cells(self, xs, ys):
        cx = np.floor((xs - self.x0) / self.res).astype(np.int64)
        cy = np.floor((ys - self.y0) / self.res).astype(np.int64)
        inside = (cx >= 0) & (cx < self.nx) & (cy >= 0) & (cy < self.ny)
        return np.unique(cy[inside] * self.nx + cx[inside])

    def add_scan(self, x, y, heading, ranges, angles, t):
        """One scan, taken from map pose (x, y, heading in radians) at time t.

        `angles` are each beam's direction in the rover's frame. Each cell is
        counted once per scan however many beams fell in it, so that a near wall
        does not outvote a far one.

        A scan dated more than CLOCK_SLACK_S before the newest one the watch
        holds is not counted (None, and counted in `stale`): the rover's clock
        reads 1970 for the first minute or so after its power has been off, and
        such a scan would join the visit before it and date that visit 1970."""
        if self._newest is None:
            seen = self.cur_last[np.isfinite(self.cur_last)]
            self._newest = float(seen.max()) if seen.size else -np.inf
        if t < self._newest - CLOCK_SLACK_S:
            self.stale += 1
            return None
        self._newest = max(self._newest, t)
        r = np.asarray(ranges, dtype=float)
        a = np.asarray(angles, dtype=float)
        ok = np.isfinite(r) & (r > MIN_RANGE_M) & (r < MAX_RANGE_M)
        r, a = r[ok], a[ok] + heading
        if r.size == 0:
            return None
        far = r > MAP_FIT_FAR_M
        if not self._on_map(x + r[far] * np.cos(a[far]), y + r[far] * np.sin(a[far])):
            self.off_map += 1
            self.refused += 1
            return False
        hits = self._cells(x + r * np.cos(a), y + r * np.sin(a))
        if not self._fits(hits):
            self.refused += 1
            return False
        if TALLY_MAX_M < MAX_RANGE_M:
            near = r < TALLY_MAX_M
            hits = self._cells(x + r[near] * np.cos(a[near]), y + r[near] * np.sin(a[near]))
        steps = np.arange(int(min(np.max(r), TALLY_MAX_M) / STEP_M) + 1) * STEP_M + STEP_M
        inside = steps[None, :] < np.minimum(r - self.res, TALLY_MAX_M)[:, None]
        px = x + steps[None, :] * np.cos(a)[:, None]
        py = y + steps[None, :] * np.sin(a)[:, None]
        passes = self._cells(px[inside], py[inside])
        touched = np.union1d(hits, passes)
        # A cell not seen for VISIT_GAP_S starts a new visit, and the one it
        # ends becomes what the next is compared with -- if it saw enough. One
        # that flipped while in sight is taken as its recent windows say, so the
        # next visit is compared with how the place was left.
        lapsed = touched[(t - self.cur_last.flat[touched]) > VISIT_GAP_S]
        if lapsed.size:
            hits_was = self.cur_hits.flat[lapsed]
            pass_was = self.cur_pass.flat[lapsed]
            recent_hits = self.old_hits.flat[lapsed] + self.win_hits.flat[lapsed]
            recent_pass = self.old_pass.flat[lapsed] + self.win_pass.flat[lapsed]
            r_solid, r_clear = _flipped(hits_was, pass_was, recent_hits, recent_pass)
            recent = r_solid | r_clear
            whole = (hits_was + pass_was >= MIN_OBS) & ~recent
            for take, h, p in ((whole, hits_was, pass_was), (recent, recent_hits, recent_pass)):
                cells = lapsed[take]
                self.ref_hits.flat[cells] = h[take]
                self.ref_pass.flat[cells] = p[take]
                self.ref_time.flat[cells] = self.cur_last.flat[cells]
            self.cur_hits.flat[lapsed] = 0
            self.cur_pass.flat[lapsed] = 0
            self.cur_first.flat[lapsed] = t
            for name in VIEW_ARRAYS:
                getattr(self, name).flat[lapsed] = np.inf if name.endswith("0") else -np.inf
            for name in ("win_hits", "win_pass", "old_hits", "old_pass"):
                getattr(self, name).flat[lapsed] = 0
            self.win_first.flat[lapsed] = t
            self.old_first.flat[lapsed] = np.nan
        # The recent windows move on: the newer becomes the older, or, if the
        # cell went unseen for longer than both, neither is kept.
        began = self.win_first.flat[touched]
        rolled = touched[np.isnan(began) | ((t - began) > RECENT_S)]
        if rolled.size:
            gone_by = np.isnan(self.win_first.flat[rolled]) | (
                (t - self.win_first.flat[rolled]) > 2 * RECENT_S)
            keep = rolled[~gone_by]
            drop = rolled[gone_by]
            self.old_hits.flat[keep] = self.win_hits.flat[keep]
            self.old_pass.flat[keep] = self.win_pass.flat[keep]
            self.old_first.flat[keep] = self.win_first.flat[keep]
            self.old_hits.flat[drop] = 0
            self.old_pass.flat[drop] = 0
            self.old_first.flat[drop] = np.nan
            self.win_hits.flat[rolled] = 0
            self.win_pass.flat[rolled] = 0
            self.win_first.flat[rolled] = t
        for kind, cells in (("hit", hits), ("pass", passes)):
            for axis, at in (("x", x), ("y", y)):
                low = getattr(self, "%s_%s0" % (kind, axis))
                high = getattr(self, "%s_%s1" % (kind, axis))
                low.flat[cells] = np.minimum(low.flat[cells], at)
                high.flat[cells] = np.maximum(high.flat[cells], at)
        self.cur_hits.flat[hits] += 1
        self.cur_pass.flat[passes] += 1
        self.win_hits.flat[hits] += 1
        self.win_pass.flat[passes] += 1
        self.cur_last.flat[touched] = t
        self.scans += 1
        self._known_age += 1
        return True

    def _fits(self, hits):
        """Whether a scan's hits lie on what this watch already knows is there.

        Judged against both visits: the last one, and the one in progress, so a
        pose that goes wrong part way through a visit is caught against the
        visit's own first half as well."""
        if self._known is None or self._known_age >= FIT_REFRESH_SCANS:
            was_solid, was_clear = self._states(self.ref_hits, self.ref_pass)
            is_solid, is_clear = self._states(self.cur_hits, self.cur_pass)
            solid = _dilate(was_solid | is_solid, SLACK_CELLS)
            self._known = (solid, (was_clear | is_clear) & ~solid)
            self._known_age = 0
        solid, clear = self._known
        on = int(solid.flat[hits].sum())
        off = int(clear.flat[hits].sum())
        return on + off < FIT_MIN_KNOWN or on >= FIT_MIN * (on + off)

    def _states(self, hits, passes):
        seen = hits + passes
        share = hits / np.maximum(seen, 1)
        judged = seen >= MIN_OBS
        return judged & (share >= SOLID), judged & (share <= CLEAR)

    def _present(self):
        """(solid, clear, since): each cell's state in the visit in progress,
        and since when the evidence for it was gathered. The whole visit's,
        unless it disagrees with itself because the place flipped: then its
        last two windows'."""
        solid, clear = self._states(self.cur_hits, self.cur_pass)
        r_solid, r_clear = _flipped(self.cur_hits, self.cur_pass,
                                    self.old_hits + self.win_hits,
                                    self.old_pass + self.win_pass)
        recent_since = np.where(self.old_hits + self.old_pass > 0,
                                self.old_first, self.win_first)
        since = np.where(r_solid | r_clear, recent_since, self.cur_first)
        return solid | r_solid, clear | r_clear, since

    def changes(self, large_only=False, unconfirmed=False):
        """What differs between each cell's last visit and its present one.

        A list of groups, largest first: `kind` GONE (solid then, seen through
        now) or NEW (the other way round), where, how big, when the present
        evidence was gathered, when the place was last seen otherwise, and
        from where (`viewed_from`, the middle of the viewpoints, and `spread_m`,
        how far apart they were). With `unconfirmed`, instead the groups seen
        from one place only, closer together than VIEW_SPREAD_M: not reported
        as changes, but a reason to look from somewhere else. A parked rover
        sees nothing else, and on 2026-10-21 two bean bags set down beside it
        were among them for two minutes before it drove off, and reported only
        once it had."""
        was_solid, was_clear = self._states(self.ref_hits, self.ref_pass)
        is_solid, is_clear, since = self._present()
        found = []
        for kind, mask in (
                ("GONE", was_solid & is_clear & ~_dilate(is_solid, SLACK_CELLS)),
                ("NEW", is_solid & was_clear & ~_dilate(was_solid, SLACK_CELLS))):
            for group in _groups(mask):
                if len(group) < MIN_CELLS:
                    continue
                rows = np.array([g[0] for g in group])
                cols = np.array([g[1] for g in group])
                first = float(np.nanmin(since[rows, cols]))
                last = float(np.max(self.cur_last[rows, cols]))
                if last - first < PERSIST_S:
                    continue
                # Seen gone from where the beams passed through, new from where
                # they ended.
                # Only cells whose viewpoints are known: one loaded from a file
                # older than them has none, which is not yet seen at all.
                by = "pass" if kind == "GONE" else "hit"
                x0 = getattr(self, by + "_x0")[rows, cols]
                known = np.isfinite(x0)
                if not known.any():
                    continue
                vx0, vx1 = float(x0[known].min()), float(getattr(self, by + "_x1")[rows, cols][known].max())
                vy0 = float(getattr(self, by + "_y0")[rows, cols][known].min())
                vy1 = float(getattr(self, by + "_y1")[rows, cols][known].max())
                spread = math.hypot(vx1 - vx0, vy1 - vy0)
                if (spread < VIEW_SPREAD_M) != unconfirmed:
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
                    "viewed_from": [round((vx0 + vx1) / 2.0, 2), round((vy0 + vy1) / 2.0, 2)],
                    "spread_m": round(spread, 2),
                })
        found.sort(key=lambda g: -g["area_m2"])
        return found

    @staticmethod
    def first_seen(found, already, near_m=0.3):
        """The groups in `found` not already reported: no group in `already` of
        the same kind within `near_m`. A group grows and shifts a little as the
        visit goes on, and the same armchair must not be news every report.
        But one reported small that has grown to furniture size is news again:
        on 2026-10-10 an armchair's old place was logged small and had grown to
        0.95 m by the end of the run, and the count, which takes the
        furniture-sized changes from the log, never saw it."""
        fresh = []
        for g in found:
            if not any(o["kind"] == g["kind"]
                       and (o.get("large", True) or not g.get("large", False))
                       and math.hypot(o["x_m"] - g["x_m"], o["y_m"] - g["y_m"]) <= near_m
                       for o in already):
                fresh.append(g)
        return fresh

    @staticmethod
    def news(found, previous, logged, now, within_s=None):
        """What to log from a report: groups neither in the report before
        (`previous`) nor logged in the last `within_s` (`logged`, rows carrying
        `logged_at`). A group near its thresholds drops out of one report and
        comes back in the next, and in control 1 on 2026-10-11 two changes were
        logged twice that way, minutes apart."""
        within_s = RELOG_S if within_s is None else within_s
        recent = [r for r in logged if now - r["logged_at"] < within_s]
        return ChangeWatch.first_seen(found, list(previous) + recent)

    def save(self, path, **meta):
        """Write the tallies, atomically, with whatever names their map."""
        tmp = path + ".tmp.npz"
        np.savez_compressed(
            tmp, cur_hits=self.cur_hits, cur_pass=self.cur_pass,
            cur_first=self.cur_first, cur_last=self.cur_last,
            ref_hits=self.ref_hits, ref_pass=self.ref_pass, ref_time=self.ref_time,
            **{name: getattr(self, name) for name in VIEW_ARRAYS + RECENT_ARRAYS},
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
                         "ref_hits", "ref_pass", "ref_time") + VIEW_ARRAYS + RECENT_ARRAYS:
                # A file from before 2026-10-11 has no viewpoints: the visit in
                # progress has to be seen from two places again.
                if name in data.files:
                    getattr(watch, name)[...] = data[name]
            watch.scans = int(meta.get("scans", 0))
        return watch, meta
