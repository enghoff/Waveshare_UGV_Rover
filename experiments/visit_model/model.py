"""A world model kept per visit, replayed over every look since 2026-10-03.

    python model.py LOOKS.npz OUT.json [--test] [RECORDING.npz ...]

Looks are taken in time order and cut into visits (`windows`): a run of looks
with no gap over GAP_S, at most WINDOW_S long. In each visit:

1. **The visit's objects** (`cluster`). Two regions of different pictures are
   linked when they look alike and their geometry agrees: two measured places
   near each other (for their size), a measured place on the other's sight
   line, or two sight lines that cross ahead of both. Links are joined
   strongest first, never putting two regions of one picture in one cluster,
   nor two clusters whose measured places stand apart.
2. **Matched against memory once** (`reconcile`). Every cluster with a place
   is compared with every remembered object still standing, by place, size,
   height and likeness, in one assignment. Two standing objects that the same
   cluster fits, never seen in one picture together, are one object and are
   merged: the duplicate a stray picture starts would otherwise split every
   later sighting between the two. A cluster left over is matched to an object
   seen gone, by likeness alone, when one is clearly the likeliest: that is a
   move. Otherwise it is new. A cluster with no place (sight lines that never
   crossed) is filed to a standing object its sight lines pass, or left
   unfiled.
3. **Absence** (`absent`). Where a drive recording covers the visit, an object
   the lidar hit when it was seen, and that was not seen in this visit, is gone
   once most of the visit's scans that judge its place pass through it.

What it keeps per object: one current place with its spread, its size and
height, up to EXEMPLARS masked DINOv2 vectors chosen to differ from one
another, the pictures it holds a region of, whether the lidar hits it, and the
places it has stood.
"""
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import data  # noqa: E402
import lidar  # noqa: E402
import recorded  # noqa: E402

# --- visits
GAP_S = 30.0
WINDOW_S = 150.0
# --- links between regions of one visit
LINK_COS = 0.6          # likeness for a link with a measured place
RAY_LINK_COS = 0.75     # likeness for a link between two sight lines alone
NEAR_M = 0.4            # two measured places, plus a quarter of their widths
RANGE_SLACK = 0.05      # ... plus this per metre of range
RAY_M = 0.3             # a measured place this near a sight line, plus BEARING_SLACK
BEARING_SLACK = math.tan(math.radians(4.0))
SAME_SPOT_M = 0.2       # cameras this close see the same thing at the same bearing
SAME_BEARING_DEG = 6.0
AHEAD_M = (0.3, 8.0)
HEIGHT_M = 0.5          # measured heights (range x tan elevation) this far apart are two things
CLUSTER_SPREAD_M = 0.6  # two clusters' measured centres this far apart (plus size) never join
MAX_EXTENT_M = 3.0
RAY_EXTENT_M = 0.5
LIDAR_SHORT_M = 0.3     # a lidar range puts a thing no nearer than this short of it
SAME_SPOT_RANGE_M = 0.6 # two lidar ranges from one spot, one bearing, this apart are two things
WALL_AGREE_M = 0.6      # a depth range this near the wall's lets the wall place a raised thing
WALL_MIN_M = 2.0        # ... and only a wall this far away (nearer, wall places land 0.41 m out)
AGREE_K = 2.5           # two places agree within this many of their combined spreads
OUTLIER_K = 2.5         # a cluster's place leaves out places this many spreads from its median
HERE_K = 2.5            # a cluster and an object can be one within this many spreads (plus size)
SYSTEMATIC_M = 0.1      # what averaging many places cannot remove: pose, lens
# --- memory
EXEMPLARS = 12
HERE_M = 0.5            # a cluster and a standing object this near (plus size) can be one
HERE_COS = 0.55
SIGHTED_COS = 0.75      # a cluster with sight lines only is filed to an object this alike
DISTANCE_COST = 0.3     # likeness given up per metre apart, in the assignment
LONE_EXTRA_COS = 0.1    # a cluster of one region needs this much more likeness
MERGE_M = 0.4           # two standing objects this near (plus size) can be one ...
MERGE_COS = 0.75        # ... when the cluster that fits both is this like the other
CONSOLIDATE_COS = 0.75  # two standing objects together, never in one picture, this alike are one
CLOSE_COS = 0.6         # ... or this alike when within one spread of each other
PENDING_S = 600.0       # a new object can still turn out to be a gone one, moved, this long
PENDING_COS = 0.75      # ... when it looks this like it, and clearly more than any other
COLOUR_MIN = 0.35       # two things whose colours match less than this are two things
#: A record unseen this long is "here as before" only on the likeness a move
#: needs: hours later, standing where it stood says little about identity in a
#: home that gets rearranged (M5 session 5: the black bean bag, 0.6 m from the
#: red one's record of the day before, was taken for it).
STALE_S = float(os.environ.get("STALE_S", 3 * 3600))
#: A departure seen this recently is matched to an arrival before departures
#: of days ago: in M5 session 5 the red bean bag's own record, seen gone at its
#: old place minutes before, tied with two older records of it gone for days,
#: and the move went unpaired.
RECENT_GONE_S = float(os.environ.get("RECENT_GONE_S", 1800))
#: An arrival is paired at once only with a recent departure; with an older
#: one it waits, pending, until the visit's absences are in, so that its own
#: departure seen in the same visit can win (M5 session 5: the red bean bag
#: paired with a record of it gone for days while its record of minutes ago
#: was marked gone later in the same visit).
DEFER_OLD_MOVES = os.environ.get("DEFER_OLD_MOVES", "1") == "1"
#: Two pictures whose colours match less than this are not linked into one
#: cluster: single pictures' colours separate objects weakly (half of all
#: different-object pairs score over 0.48) but this cuts 3% of same-object
#: links and 18% of different ones, and links chain. A cluster of two rug
#: pictures and one of the red bean bag went to the rug's record (session 3).
LINK_COLOUR_MIN = float(os.environ.get("LINK_COLOUR_MIN", 0.2))
COLOUR_MOSTLY = os.environ.get("COLOUR_MOSTLY", "0") == "1"  # tried 2026-10-16, not adopted (README)
PATH_S = 3600.0         # a record seen gone this recently and a look-alike found standing ...
PATH_COS = 0.7          # ... this alike, never seen in one visit, are one object at two places
PATH_STALE_S = 1800.0   # ... when the one found standing had not been seen for this long before
MOVE_LEAD_COS = 0.68    # a move on less likeness than MOVE_COS needs ...
MOVE_LEAD = 0.1         # ... this much lead over every other gone look-alike
MOVE_COS = 0.75         # a cluster and a gone object this alike can be a move
MOVE_MARGIN = 0.04      # ... when no other gone object is within this
PLACE_FLOOR_M2 = 0.01
NEW_MIN_REGIONS = 2     # an object is first recorded from this many pictures of it
#: Every bar that joins two things raised by this much, so that a likeness just
#: over a bar starts a record of its own instead of joining (version 4,
#: 2026-10-16). Version 3's wrong merges came and went with 1% of the pictures
#: taken away (0-2 over seven sessions); with every bar 0.06 higher there were
#: none in eleven such runs, at 0.03 three in six. Sparing the move bars
#: followed more moves but let a painting's record "move" (one in eleven), so
#: they are raised too.
JOIN_MARGIN = float(os.environ.get("JOIN_MARGIN", 0.06))
LINK_COS += JOIN_MARGIN
RAY_LINK_COS += JOIN_MARGIN
HERE_COS += JOIN_MARGIN
SIGHTED_COS += JOIN_MARGIN
MERGE_COS += JOIN_MARGIN
CONSOLIDATE_COS += JOIN_MARGIN
CLOSE_COS += JOIN_MARGIN
PENDING_COS += JOIN_MARGIN
COLOUR_MIN += JOIN_MARGIN
LINK_COLOUR_MIN += JOIN_MARGIN
if os.environ.get("MARGIN_ON_MOVES", "1") == "1":
    PATH_COS += JOIN_MARGIN
    MOVE_LEAD_COS += JOIN_MARGIN
    MOVE_COS += JOIN_MARGIN
# --- absence
GONE_MIN_SCANS = 5
GONE_SHARE = 0.66
GONE_REACH_M = 4.0
#: Absence from the camera, for what the lidar does not judge (version 5): a
#: visit with a standing object's depth place within CAM_GONE_M of the camera
#: and CAM_CONE_DEG of its axis in CAM_PICTURES pictures or more, that files
#: nothing to it and has no region of any record whose sight line passes within
#: CAM_THERE_M (or its own half-width) of it without its range running on past
#: it or its being filed to a record standing elsewhere, is an empty visit. CAM_EMPTY_VISITS of them in a row
#: and it is set aside as not found again (`camera_absent`) -- not called gone:
#: called gone, the same rule took two records of things that were there
#: (2026-10-18, README). Only an object with a depth place: one from sight lines
#: alone may be a metre out. Off (0) unless set: the filing is version 4's
#: either way.
CAM_GONE_M = 2.5
CAM_NEAR_M = 0.5
CAM_CONE_DEG = 45.0
CAM_PICTURES = 3
CAM_EMPTY_VISITS = int(os.environ.get("CAM_EMPTY_VISITS", 0))
CAM_THERE_M = 0.5
#: Candidates for version 5, off unless set (2026-10-20, R-WS-17).
#: SOLID_BY_VISIT: whether the lidar hit an object filed in a visit is judged
#: over every scan while its pictures were taken (SOLID_PAD_S either side), not
#: one scan at one picture's time, which can pass between a stool's legs or
#: find no return and counts either as empty. On the multi-day benchmark's day 1
#: the stool's place was hit 23 to 75 times in each visit and judged not solid
#: every time, so the lidar's 400 passes through it on day 2 never made the
#: record gone. A visit judged on fewer than SOLID_MIN scans is not counted.
SOLID_BY_VISIT = os.environ.get("SOLID_BY_VISIT", "0") == "1"
SOLID_PAD_S = 5.0
SOLID_MIN = 3
#: ONE_THING_RIVALS: a gone record that looks like the leading candidate for a
#: move by DOUBLE_COS or more, with matching colours and never in one picture
#: with it, is the same thing recorded twice, not a rival, and the lead is
#: taken over the best candidate that is not. On day 1 the red bean bag's
#: arrival scored 0.809, 0.772 and 0.770 against its own three gone records and
#: was made a new record for want of a 0.04 lead. Over every labelled record
#: to 2026-10-20 (pair_likeness), records of two things never in one picture,
#: of one height and colour, scored 0.63 at most, and 27 of 98 pairs of
#: records of one thing 0.70 or more.
ONE_THING_RIVALS = os.environ.get("ONE_THING_RIVALS", "0") == "1"
DOUBLE_COS = float(os.environ.get("DOUBLE_COS", 0.70))
#: The end of development: M5 session 2's last labelled look was before this
#: (2026-10-14 13:00 local). Development runs see nothing after it.
DEVELOPMENT_END = 1791963000.0


#: Use the likeness learned from the rover's own pictures (metric.py) where
#: the looks carry it, mapped onto the image vectors' scale so that every bar
#: above keeps its rate of false matches.
USE_LEARNED = True
_SIM = {"map": None}
#: The scale from the learned likeness to the image vectors', fitted once on
#: the pictures taken before SCALE_FITTED_BEFORE (the shakedown to M5 session
#: 4, the pictures metric-1015.npz was learned from) and kept beside it. Fitted
#: over whatever looks a run was given, it moved with every picture added, later
#: ones included: in M5 session 6 the 44 pictures taken after session 5 shifted
#: a score of 2026-10-03 by 0.001 and two pictures of one painting went to
#: another's record in session 5.
SCALE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scale-1015.npz")
SCALE_FITTED_BEFORE = 1792089000.0  # 2026-10-16 00:00 local


def sim(x):
    """A likeness on the scale the bars were set on."""
    m = _SIM["map"]
    if m is None:
        return x
    return np.interp(x, m[0], m[1])


def learn_scale(a, learned):
    """The mapping from the learned likeness to the image vectors' own, matched
    on pairs of regions of one picture -- two things by construction -- so
    that a bar lets through the same share of two-thing pairs either way."""
    fr = a["frame"].astype(str)
    order = np.argsort(fr, kind="stable")
    bounds = np.flatnonzero(np.r_[True, fr[order][1:] != fr[order][:-1], True])
    i0, i1 = [], []
    for s0, e0 in zip(bounds[:-1], bounds[1:]):
        idx = order[s0:e0]
        for p in range(len(idx)):
            for q in range(p + 1, len(idx)):
                i0.append(idx[p])
                i1.append(idx[q])
    i0, i1 = np.array(i0), np.array(i1)
    old = np.sum(a["vec"][i0] * a["vec"][i1], axis=1)
    new = np.sum(learned[i0] * learned[i1], axis=1)
    qs = np.linspace(0.0, 1.0, 201)
    xo, xn = np.quantile(old, qs), np.quantile(new, qs)
    # Above every two-thing pair, straight on to 1 = 1.
    xn = np.append(xn, 1.0)
    xo = np.append(xo, 1.0)
    keep = np.r_[True, np.diff(xn) > 1e-9]
    return xn[keep], xo[keep]


def windows(t, gap=GAP_S, longest=WINDOW_S):
    order = np.argsort(t, kind="stable")
    start = 0
    for k in range(1, len(order) + 1):
        if k == len(order) or t[order[k]] - t[order[k - 1]] > gap or \
                t[order[k]] - t[order[start]] > longest:
            yield order[start:k]
            start = k


def diverse(vecs, k=EXEMPLARS):
    """Up to k rows of vecs, each as unlike those already chosen as possible."""
    if len(vecs) <= k:
        return vecs
    chosen = [0]
    best = vecs @ vecs[0]
    for _ in range(k - 1):
        i = int(np.argmin(best))
        chosen.append(i)
        best = np.maximum(best, vecs @ vecs[i])
    return vecs[chosen]


class Looks:
    """The arrays of data.py, with each region's sight line, width and height.

    **A lidar range is a floor, not a place.** The lidar answers with the
    nearest thing in its plane along the bearing, so a wardrobe behind a bed
    takes the bed's range: the deployed store found 15% of lidar ranges more
    than 0.8 m short of the depth camera's (world_state/lidar_ranges.py). So
    only depth places a region (`placed`); a lidar range says the thing is no
    nearer than it, less LIDAR_SHORT_M (`nearest`), and places a cluster that
    has nothing better, loosely."""

    def __init__(self, a):
        a = dict(a)
        if USE_LEARNED and "lvec" in a:
            m = np.load(SCALE_PATH)
            _SIM["map"] = (m["learned"], m["image"])
            a["vec"] = a["lvec"]
        src = np.array([str(s) for s in a["place_from"]])
        if "wx" in a:
            # A raised thing hangs where its sight line meets a wall
            # (wallplace.py), unless a depth range puts it well short of it.
            depth = np.isin(src, ("outline", "box")) & np.isfinite(a["px"])
            on_wall = np.isfinite(a["wx"]) & (np.nan_to_num(a["wr"]) >= WALL_MIN_M) & \
                (~depth | (np.abs(np.nan_to_num(a["range"]) - np.nan_to_num(a["wr"])) <= WALL_AGREE_M))
            for k, v in (("px", "wx"), ("py", "wy"), ("range", "wr")):
                a[k] = np.where(on_wall, a[v], a[k])
            src = np.where(on_wall, "wall", src)
        self.a = a
        self.n = len(a["id"])
        self.posed = np.array([p is not None for p in a["pose_from"]])
        b = np.radians(np.nan_to_num(a["bearing"]))
        self.ux, self.uy = np.cos(b), np.sin(b)
        ranged = np.isfinite(a["px"])
        self.placed = ranged & np.isin(src, ("outline", "box", "wall"))
        self.lidar = ranged & ~self.placed
        r = np.nan_to_num(a["range"])
        # How far each kind of place lands from its object, measured on the
        # development labels (calibrate.py): depth outlines 0.24 m at the
        # median under 2.5 m and 0.32 beyond, depth boxes 0.51, walls 0.08
        # from beyond 2.5 m.
        self.sigma = np.select([src == "outline", src == "wall"],
                               [np.where(r < 2.5, 0.25, 0.32), np.where(r >= 2.5, 0.1, 0.3)], 0.5)
        self.nearest = np.where(self.lidar, np.nan_to_num(a["range"]) - LIDAR_SHORT_M, 0.0)
        self.z = np.where(self.placed & np.isfinite(a["elevation"]),
                          a["range"] * np.tan(np.radians(np.nan_to_num(a["elevation"]))), np.nan)
        self.width = np.where(ranged, np.nan_to_num(a["range"]) *
                              np.radians(np.nan_to_num(a["span"])), np.nan)
        self.frame = a["frame"].astype(str)
        self.colour = a.get("colour")
        self.has_colour = a.get("has_colour")


def _links(L, idx):
    """Candidate links (likeness, i, j) between regions of different pictures."""
    a = L.a
    idx = idx[L.posed[idx]]
    if len(idx) < 2:
        return []
    V = a["vec"][idx]
    S = sim(V @ V.T)
    out = []
    frames = L.frame[idx]
    for p in range(len(idx)):
        js = np.flatnonzero((S[p, p + 1:] >= LINK_COS) & (frames[p + 1:] != frames[p])) + p + 1
        for q in js:
            i, j = idx[p], idx[q]
            if L.has_colour is not None and L.has_colour[i] and L.has_colour[j] and                     float(L.colour[i] @ L.colour[j]) < LINK_COLOUR_MIN:
                continue
            if _agree(L, i, j, S[p, q]):
                out.append((float(S[p, q]), int(i), int(j)))
    return out


def _on_ray(L, r, x, y, extent):
    """Whether a point (with a size) lies on region r's sight line, no nearer
    than its lidar floor."""
    a = L.a
    dx, dy = x - a["cx"][r], y - a["cy"][r]
    along = dx * L.ux[r] + dy * L.uy[r]
    across = abs(-dx * L.uy[r] + dy * L.ux[r])
    return max(AHEAD_M[0], L.nearest[r] - extent / 2.0) <= along <= AHEAD_M[1] and         across <= RAY_M + along * BEARING_SLACK + extent / 2.0


def _agree(L, i, j, cos):
    a = L.a
    pi, pj = L.placed[i], L.placed[j]
    if pi and pj:
        d = math.hypot(a["px"][i] - a["px"][j], a["py"][i] - a["py"][j])
        reach = AGREE_K * math.hypot(L.sigma[i], L.sigma[j]) + \
            (min(L.width[i], MAX_EXTENT_M) + min(L.width[j], MAX_EXTENT_M)) / 4.0
        if d > reach:
            return False
        zi, zj = L.z[i], L.z[j]
        return not (math.isfinite(zi) and math.isfinite(zj) and abs(zi - zj) > HEIGHT_M)
    if pi or pj:
        p, r = (i, j) if pi else (j, i)
        return _on_ray(L, r, a["px"][p], a["py"][p], min(L.width[p], MAX_EXTENT_M))
    if cos < RAY_LINK_COS:
        return False
    if math.hypot(a["cx"][i] - a["cx"][j], a["cy"][i] - a["cy"][j]) < SAME_SPOT_M:
        if L.lidar[i] and L.lidar[j] and abs(a["range"][i] - a["range"][j]) > SAME_SPOT_RANGE_M:
            return False
        return abs((a["bearing"][i] - a["bearing"][j] + 180.0) % 360.0 - 180.0) <= SAME_BEARING_DEG
    hit = _cross(L, i, j)
    return hit is not None and _on_ray(L, i, hit[0], hit[1], 0.0) and _on_ray(L, j, hit[0], hit[1], 0.0)


def _cross(L, i, j):
    """Where two sight lines cross, ahead of both, or None."""
    a = L.a
    det = L.ux[i] * (-L.uy[j]) - L.uy[i] * (-L.ux[j])
    if abs(det) < math.sin(math.radians(2.0)):
        return None
    bx, by = a["cx"][j] - a["cx"][i], a["cy"][j] - a["cy"][i]
    ti = (bx * (-L.uy[j]) - by * (-L.ux[j])) / det
    tj = (L.ux[i] * by - L.uy[i] * bx) / det
    if not (AHEAD_M[0] <= ti <= AHEAD_M[1] and AHEAD_M[0] <= tj <= AHEAD_M[1]):
        return None
    return a["cx"][i] + ti * L.ux[i], a["cy"][i] + ti * L.uy[i]


def place_of(L, members):
    """{x, y, var, extent, z} of a cluster, or None: its depth places weighted
    by their spread, else the least-squares crossing of its sight lines."""
    a = L.a
    m = np.asarray(members)
    pl = m[L.placed[m]]
    if len(pl):
        # Robustly: a weighted mean of the places within OUTLIER_K spreads of
        # their median, since a place's error has a long tail (calibrate.py:
        # depth places 0.28 m out at the median, 0.79 m at the 90th percentile).
        px, py = a["px"][pl], a["py"][pl]
        keep = np.hypot(px - np.median(px), py - np.median(py)) <= OUTLIER_K * L.sigma[pl] + \
            np.minimum(L.width[pl], MAX_EXTENT_M) / 2.0
        pl = pl[keep] if keep.any() else pl
        w = 1.0 / L.sigma[pl] ** 2
        x = float(np.sum(w * a["px"][pl]) / w.sum())
        y = float(np.sum(w * a["py"][pl]) / w.sum())
        spread = float(np.max(np.hypot(a["px"][pl] - x, a["py"][pl] - y)))
        extent = min(MAX_EXTENT_M, max(float(np.median(L.width[pl])), 2.0 * spread))
        z = L.z[pl][np.isfinite(L.z[pl])]
        return {"x": x, "y": y, "var": max(PLACE_FLOOR_M2, 1.0 / w.sum()), "extent": extent,
                "z": float(np.median(z)) if len(z) else float("nan")}
    if len(m) >= 2:
        A = np.zeros((2, 2))
        bvec = np.zeros(2)
        for i in m:
            nx, ny = -L.uy[i], L.ux[i]
            A += np.outer((nx, ny), (nx, ny))
            bvec += np.array((nx, ny)) * (nx * a["cx"][i] + ny * a["cy"][i])
        if np.linalg.cond(A) <= 1e3:
            x, y = np.linalg.solve(A, bvec)
            if all(_on_ray(L, i, x, y, 0.0) for i in m):
                return {"x": float(x), "y": float(y), "var": 0.15, "extent": RAY_EXTENT_M,
                        "z": float("nan"), "rays": True}
    # Lidar ranges alone do not place a cluster: they land 0.66 m from their
    # object at the median (calibrate.py), which put the shakedown's two
    # armchairs, 1.1 m apart, into one object. Such a cluster is filed by its
    # sight lines to an object already standing, or not at all.
    return None


def consistent(L, members):
    """Whether every sight line in a cluster passes its place. A link between
    two sight lines alone says little -- two lines in a plane nearly always
    cross -- so a cluster joined that way is checked whole: on session 2 three
    looks at the village painting joined the blue painting 3 m away so."""
    m = np.asarray(members)
    rays = m[~L.placed[m]]
    if not len(rays):
        return True
    p = place_of(L, m)
    if p is None:
        return False
    return all(_on_ray(L, r, p["x"], p["y"], p["extent"]) for r in rays)



def cluster(L, idx):
    """The visit's clusters: lists of region indices."""
    parent = {int(i): int(i) for i in idx}
    frames = {int(i): {L.frame[i]} for i in idx}
    members = {int(i): [int(i)] for i in idx}

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for _cos, i, j in sorted(_links(L, idx), reverse=True):
        ri, rj = root(i), root(j)
        if ri == rj or frames[ri] & frames[rj]:
            continue
        if L.placed[members[ri]].any() and L.placed[members[rj]].any():
            pi, pj = place_of(L, members[ri]), place_of(L, members[rj])
            if math.hypot(pi["x"] - pj["x"], pi["y"] - pj["y"]) > \
                    CLUSTER_SPREAD_M + (pi["extent"] + pj["extent"]) / 4.0:
                continue
        if not consistent(L, members[ri] + members[rj]):
            continue
        parent[rj] = ri
        frames[ri] |= frames.pop(rj)
        members[ri] += members.pop(rj)
    return [m for r, m in members.items() if root(r) == r]


def _heights_agree(z0, z1):
    return not (math.isfinite(z0) and math.isfinite(z1) and abs(z0 - z1) > HEIGHT_M)


class Memory:
    def __init__(self):
        self.objects = []
        self.alias = {}
        self.visit = 0

    def new(self, p, vecs, frames, t, cols=None):
        o = {"id": "v%d" % (len(self.objects) + len(self.alias)), "x": p["x"], "y": p["y"],
             "var": p["var"], "extent": p["extent"], "z": p["z"], "ex": diverse(vecs),
             "col": diverse(cols) if cols is not None else np.zeros((0, 1)),
             "frames": set(frames), "visits": {self.visit}, "state": "present", "seen": t,
             "since": t, "gone_at": None,
             "solid_hits": 0, "solid_judged": 0, "path": [[round(p["x"], 2), round(p["y"], 2), round(t)]]}
        self.objects.append(o)
        return o

    def likeness(self, vecs, o):
        return float(sim((vecs @ o["ex"].T).max()))

    def colour(self, cols, o):
        """How well a cluster's colours match an object's: the mean of each
        region's best match. 1 when there is nothing to compare."""
        if cols is None or not len(cols) or not len(o["col"]):
            return 1.0
        if COLOUR_MOSTLY:
            # Against what the record mostly looks like: the median over its
            # samples. The samples are chosen to differ, so the best of them
            # is whichever stray picture a record once took (M5 session 5:
            # two dark samples let the black bean bag into the red one's).
            return float(np.median(cols @ o["col"].T, axis=1).mean())
        return float((cols @ o["col"].T).max(axis=1).mean())

    def gate(self, o, p):
        return HERE_K * math.sqrt(p["var"] + o["var"] + SYSTEMATIC_M ** 2) + \
            (o["extent"] + p["extent"]) / 4.0

    def update(self, o, p, vecs, frames, t, moved=False, cols=None):
        if p is not None and moved:
            o["x"], o["y"], o["var"] = p["x"], p["y"], p["var"]
            o["path"].append([round(p["x"], 2), round(p["y"], 2), round(t)])
        elif p is not None:
            w0, w1 = 1.0 / o["var"], 1.0 / p["var"]
            o["x"] = (o["x"] * w0 + p["x"] * w1) / (w0 + w1)
            o["y"] = (o["y"] * w0 + p["y"] * w1) / (w0 + w1)
            o["var"] = max(PLACE_FLOOR_M2, 1.0 / (w0 + w1))
        if p is not None:
            o["extent"] = 0.8 * o["extent"] + 0.2 * p["extent"]
            if math.isfinite(p["z"]):
                o["z"] = p["z"] if not math.isfinite(o["z"]) else 0.8 * o["z"] + 0.2 * p["z"]
        o["ex"] = diverse(np.vstack([vecs, o["ex"]]))
        if cols is not None and len(cols):
            o["col"] = diverse(np.vstack([cols, o["col"]]) if len(o["col"]) else cols)
        o["frames"] |= set(frames)
        if self.visit not in o["visits"]:
            o["seen_before"] = o["seen"]
        o["visits"].add(self.visit)
        o["state"] = "present"
        o["seen"] = t

    def merge(self, keep, gone):
        """Fold one object into another: the same thing, recorded twice."""
        keep["ex"] = diverse(np.vstack([keep["ex"], gone["ex"]]))
        if len(gone["col"]):
            keep["col"] = diverse(np.vstack([keep["col"], gone["col"]]) if len(keep["col"]) else gone["col"])
        keep["frames"] |= gone["frames"]
        keep["visits"] |= gone["visits"]
        keep["solid_hits"] += gone["solid_hits"]
        keep["solid_judged"] += gone["solid_judged"]
        self.alias[gone["id"]] = keep["id"]
        self.objects.remove(gone)

    def resolve(self, oid):
        while oid in self.alias:
            oid = self.alias[oid]
        return oid


def reconcile(L, mem, clusters, t, decisions):
    """File each cluster to an object; returns {region index: object id}."""
    a = L.a
    filed = {}
    placed, loose = [], []
    for m in clusters:
        p = place_of(L, m)
        (placed if p else loose).append((m, p))
    standing = [o for o in mem.objects if o["state"] == "present"]
    used = set()
    # 1. Here as before: one assignment over placed clusters and standing objects.
    if placed and standing:
        ox = np.array([o["x"] for o in standing])
        oy = np.array([o["y"] for o in standing])
        cost = np.full((len(placed), len(standing)), 1e6)
        fits = {}
        for ci, (m, p) in enumerate(placed):
            d = np.hypot(ox - p["x"], oy - p["y"])
            vecs = a["vec"][m]
            for oi in np.flatnonzero(d <= HERE_M + MAX_EXTENT_M / 2.0 + 1.0):
                o = standing[oi]
                if d[oi] > mem.gate(o, p) or not _heights_agree(o["z"], p["z"]):
                    continue
                # A place from sight lines crossing is loose (the gate reaches
                # 1.3 m); the lines themselves are not. On the shakedown two
                # looks at the blue painting crossed 1.3 m from the snowy-tree
                # painting's record and were filed to it.
                if p.get("rays") and not all(_on_ray(L, i, o["x"], o["y"], o["extent"]) for i in m):
                    continue
                cos = mem.likeness(vecs, o)
                if mem.colour(cols_of(L, m), o) < COLOUR_MIN:
                    continue
                # ... and the lines alone need the likeness sight lines alone
                # need: along a wall they pass every painting on it.
                need = SIGHTED_COS if p.get("rays") else HERE_COS + (LONE_EXTRA_COS if len(m) == 1 else 0.0)
                if t - o["seen"] > STALE_S:
                    need = max(need, MOVE_COS)
                if cos >= need:
                    cost[ci, oi] = -(cos - DISTANCE_COST * d[oi])
                    fits.setdefault(ci, []).append((oi, cos))
        rows, cols = linear_sum_assignment(cost)
        for ci, oi in zip(rows, cols):
            if cost[ci, oi] >= 1e5:
                continue
            m, p = placed[ci]
            o = standing[oi]
            frames = set(L.frame[m])
            decisions.append({"t": round(t), "here": o["id"], "at": [round(p["x"], 2), round(p["y"], 2)],
                              "was": [round(o["x"], 2), round(o["y"], 2)], "n": len(m)})
            mem.update(o, p, a["vec"][m], frames, t, cols=cols_of(L, m))
            filed.update({i: o["id"] for i in m})
            used.add(ci)
            o = link_path(mem, o, t, decisions)
            # Two standing objects this cluster fits, never in one picture: one.
            for other, cos in fits.get(ci, []):
                dup = standing[other]
                if dup is o or dup not in mem.objects or dup["frames"] & o["frames"] or \
                        cos < MERGE_COS or not _heights_agree(o["z"], dup["z"]) or \
                        math.hypot(o["x"] - dup["x"], o["y"] - dup["y"]) > \
                        MERGE_M + min(o["extent"], dup["extent"]) / 4.0 + \
                        math.sqrt(o["var"] + dup["var"]):
                    continue
                decisions.append({"t": round(t), "merged": dup["id"], "into": o["id"],
                                  "at": [round(dup["x"], 2), round(dup["y"], 2)]})
                mem.merge(o, dup)
    # 2. Left over: a move from where an object was seen gone, or new.
    gone = [o for o in mem.objects if o["state"] == "gone"]
    for ci, (m, p) in enumerate(placed):
        if ci in used:
            continue
        vecs = a["vec"][m]
        frames = set(L.frame[m])
        cols = cols_of(L, m)
        best = sorted(((mem.likeness(vecs, o), k) for k, o in enumerate(gone)
                       if _heights_agree(o["z"], p["z"]) and mem.colour(cols, o) >= COLOUR_MIN),
                      reverse=True)
        rivals = _without_doubles(best, gone)
        lead = rivals[0][0] - rivals[1][0] if len(rivals) > 1 else 1.0
        pick = _move_choice(best, gone, t) if len(m) >= 2 else None
        if pick is None and not DEFER_OLD_MOVES and len(m) >= 2 and best and (
                (best[0][0] >= MOVE_COS and lead >= MOVE_MARGIN) or
                (best[0][0] >= MOVE_LEAD_COS and lead >= MOVE_LEAD)):
            pick = best[0][1]
        if pick is not None:
            o = gone[pick]
            decisions.append({"t": round(t), "moved": o["id"], "from": [round(o["x"], 2), round(o["y"], 2)],
                              "to": [round(p["x"], 2), round(p["y"], 2)], "cos": round(best[0][0], 3),
                              "runner_up": round(best[1][0], 3) if len(best) > 1 else None})
            mem.update(o, p, vecs, frames, t, moved=True, cols=cols)
            gone.remove(o)
        elif len(m) < NEW_MIN_REGIONS:
            # One picture is not an object: left unfiled rather than starting
            # one that competes with the real thing for every later sighting.
            continue
        else:
            o = mem.new(p, vecs, frames, t, cols)
            decisions.append({"t": round(t), "new": o["id"], "at": [round(p["x"], 2), round(p["y"], 2)],
                              "n": len(m), "gone_alike": [[gone[k]["id"], round(c, 3)] for c, k in best[:3]]})
        filed.update({i: o["id"] for i in m})
    # 3. Clusters with no place: a standing object their sight lines pass.
    standing = [o for o in mem.objects if o["state"] == "present"]
    if standing:
        ox = np.array([o["x"] for o in standing])
        oy = np.array([o["y"] for o in standing])
        ext = np.array([o["extent"] for o in standing])
        for m, _p in loose:
            ok = np.ones(len(standing), bool)
            for i in m:
                dx, dy = ox - a["cx"][i], oy - a["cy"][i]
                along = dx * L.ux[i] + dy * L.uy[i]
                across = np.abs(-dx * L.uy[i] + dy * L.ux[i])
                ok &= (along >= AHEAD_M[0]) & (along <= AHEAD_M[1]) & \
                    (across <= RAY_M + along * BEARING_SLACK + ext / 2.0)
            frames = set(L.frame[m])
            cand = [k for k in np.flatnonzero(ok) if not standing[k]["frames"] & frames]
            if not cand:
                continue
            vecs = a["vec"][m]
            cols = cols_of(L, m)
            cand = [k for k in cand if mem.colour(cols, standing[k]) >= COLOUR_MIN]
            if not cand:
                continue
            score = {k: mem.likeness(vecs, standing[k]) for k in cand}
            k = max(score, key=score.get)
            if score[k] >= SIGHTED_COS:
                decisions.append({"t": round(t), "sighted": standing[k]["id"], "n": len(m),
                                  "from": [round(float(a["cx"][m[0]]), 2), round(float(a["cy"][m[0]]), 2)],
                                  "bearing": round(float(a["bearing"][m[0]]), 1), "cos": round(score[k], 3)})
                mem.update(standing[k], None, vecs, frames, t, cols=cols)
                filed.update({i: standing[k]["id"] for i in m})
    return filed


def solid_votes(L, mem, filed, recs):
    """Whether the lidar hit each object filed in this visit, where recorded."""
    a = L.a
    by = {}
    for i, oid in filed.items():
        by.setdefault(mem.resolve(oid), []).append(i)
    index = {o["id"]: o for o in mem.objects}
    for oid, members in by.items():
        o = index.get(oid)
        if o is None:
            continue
        t = float(a["t"][members[len(members) // 2]])
        if recorded.for_time(recs, t) is None:
            continue
        if SOLID_BY_VISIT:
            ts = a["t"][members]
            passes, hits = lidar.outcomes(recs, np.array([o["x"]]), np.array([o["y"]]),
                                          float(ts.min()) - SOLID_PAD_S, float(ts.max()) + SOLID_PAD_S)
            if passes[0] + hits[0] < SOLID_MIN:
                continue
            o["solid_judged"] += 1
            o["solid_hits"] += int(hits[0] > passes[0])
            continue
        o["solid_judged"] += 1
        o["solid_hits"] += int(lidar.solid(recs, t, o["x"], o["y"]))


def camera_absent(L, mem, idx, filed, decisions):
    """Objects the lidar does not judge, looked for close up in this visit's
    pictures and not found, CAM_EMPTY_VISITS visits in a row: set aside as not
    found again (`unfound`, the time), not called gone. Nothing else reads the
    flag, so the filing is version 4's; the next look filed to the object takes
    it off."""
    if not CAM_EMPTY_VISITS:
        return
    a = L.a
    seen = {mem.resolve(v) for v in filed.values()}
    t1 = float(a["t"][idx].max())
    for o in mem.objects:
        if o["id"] in seen:
            o["empty"] = 0
            if o.get("unfound"):
                o["unfound"] = None
                decisions.append({"t": round(t1), "found again": o["id"]})
    pics = {}
    for i in idx:
        if L.posed[i] and a["frame"][i] not in pics:
            pics[a["frame"][i]] = (a["cx"][i], a["cy"][i], a["heading"][i] - a["pan"][i])
    if len(pics) < CAM_PICTURES:
        return
    P = np.array(list(pics.values()))
    idx = np.asarray(idx)
    rays = idx[L.posed[idx] & np.isfinite(a["bearing"][idx])]
    RX, RY = a["cx"][rays], a["cy"][rays]
    UX, UY = np.cos(np.radians(a["bearing"][rays])), np.sin(np.radians(a["bearing"][rays]))
    RR = a["range"][rays]
    # Where the record each region was filed to stands, if it was filed: a
    # region filed to a record standing elsewhere is accounted for there.
    byid = {o["id"]: o for o in mem.objects}
    FX = np.full(len(rays), np.nan)
    FY = np.full(len(rays), np.nan)
    for k, i in enumerate(rays):
        oid = filed.get(int(i))
        f = byid.get(mem.resolve(oid)) if oid is not None else None
        if f is not None:
            FX[k], FY[k] = f["x"], f["y"]
    for o in mem.objects:
        if o["state"] != "present" or o.get("unfound") or o["id"] in seen or not math.isfinite(o["z"]):
            continue
        if o["solid_hits"] > 0 and o["solid_hits"] * 2 >= o["solid_judged"]:
            continue  # the lidar's to judge (absent)
        d = np.hypot(P[:, 0] - o["x"], P[:, 1] - o["y"])
        off = (np.degrees(np.arctan2(o["y"] - P[:, 1], o["x"] - P[:, 0])) - P[:, 2] + 180.0) % 360.0 - 180.0
        n = int(np.sum((d >= CAM_NEAR_M) & (d <= CAM_GONE_M) & (np.abs(off) <= CAM_CONE_DEG)))
        if n < CAM_PICTURES:
            continue
        # A sight line past its place, of any record's region, says the place
        # is not empty unless its range ran on beyond the place: one that
        # stopped there, or short of it, or was not ranged at all -- a look
        # filed to a second record of the same thing, a painting above the
        # depth camera's view, something standing in front -- may be it.
        vx, vy = o["x"] - RX, o["y"] - RY
        along = vx * UX + vy * UY
        across = np.abs(vx * UY - vy * UX)
        near = max(CAM_THERE_M, o["extent"] / 2.0)
        through = np.isfinite(RR) & (RR > along + near)
        elsewhere = np.isfinite(FX) & (np.hypot(FX - o["x"], FY - o["y"]) > near + CAM_THERE_M)
        if np.any((along > 0) & (across <= near) & ~through & ~elsewhere):
            continue
        o["empty"] = o.get("empty", 0) + 1
        if o["empty"] >= CAM_EMPTY_VISITS:
            o["unfound"] = t1
            decisions.append({"t": round(t1), "unfound": o["id"], "at": [round(o["x"], 2), round(o["y"], 2)],
                              "empty_visits": o["empty"], "pictures": n})


def absent(L, mem, idx, filed, recs, decisions):
    """Objects not seen this visit whose place the lidar saw through."""
    a = L.a
    t0, t1 = float(a["t"][idx].min()), float(a["t"][idx].max())
    if recorded.for_time(recs, t0) is None and recorded.for_time(recs, t1) is None:
        return
    seen = {mem.resolve(v) for v in filed.values()}
    cx, cy = a["cx"][idx][L.posed[idx]], a["cy"][idx][L.posed[idx]]
    if not len(cx):
        return
    cand = [o for o in mem.objects if o["state"] == "present" and o["id"] not in seen
            and o["solid_hits"] > 0 and o["solid_hits"] * 2 >= o["solid_judged"]
            and np.min(np.hypot(cx - o["x"], cy - o["y"])) <= GONE_REACH_M]
    if not cand:
        return
    xs = np.array([o["x"] for o in cand])
    ys = np.array([o["y"] for o in cand])
    after = np.array([o["seen"] for o in cand])
    passes, hits = lidar.outcomes(recs, xs, ys, t0, t1, after=after)
    judged = passes + hits
    for o, p, j in zip(cand, passes, judged):
        if j >= GONE_MIN_SCANS and p >= GONE_SHARE * j:
            o["state"] = "gone"
            o["gone_at"] = t1
            decisions.append({"t": round(t1), "gone": o["id"], "at": [round(o["x"], 2), round(o["y"], 2)],
                              "passes": int(p), "judged": int(j)})


def cols_of(L, m):
    """The colours of a cluster's regions that have one."""
    if L.colour is None:
        return None
    m = np.asarray(m)
    m = m[L.has_colour[m]]
    return L.colour[m] if len(m) else None


def _object_colour(o0, o1):
    if not len(o0["col"]) or not len(o1["col"]):
        return 1.0
    s = o0["col"] @ o1["col"].T
    return 0.5 * (float(s.max(axis=1).mean()) + float(s.max(axis=0).mean()))


def link_path(mem, o, t, decisions):
    """One object at two places: a record found standing in a visit in which
    a look-alike record was seen gone, the two never seen in one visit, are
    the same thing, moved back to a place it stood before. In session 3 the
    cabinet's arrival found the record left from session 2 while its main
    record was being seen gone. Joined into the older; returns the object
    the cluster is now filed to."""
    if t - o.get("seen_before", t) < PATH_STALE_S:
        return o
    recent = [g for g in mem.objects if g["state"] == "gone" and g["gone_at"] is not None
              and t - g["gone_at"] <= PATH_S and not (g["visits"] & o["visits"])
              and _heights_agree(o["z"], g["z"]) and _object_colour(o, g) >= COLOUR_MIN]
    if not recent:
        return o
    best = sorted(((_object_likeness(o, g), k) for k, g in enumerate(recent)), reverse=True)
    if best[0][0] < PATH_COS or (len(best) > 1 and best[0][0] - best[1][0] < MOVE_MARGIN):
        return o
    g = recent[best[0][1]]
    keep, fold = (g, o) if g["since"] <= o["since"] else (o, g)
    decisions.append({"t": round(t), "path": fold["id"], "into": keep["id"], "from": [round(g["x"], 2), round(g["y"], 2)],
                      "to": [round(o["x"], 2), round(o["y"], 2)], "cos": round(best[0][0], 3)})
    if keep is g:
        g["x"], g["y"], g["var"], g["extent"], g["z"] = o["x"], o["y"], o["var"], o["extent"], o["z"]
        g["path"].append([round(o["x"], 2), round(o["y"], 2), round(t)])
        g["state"], g["seen"], g["gone_at"] = "present", o["seen"], None
    mem.merge(keep, fold)
    return keep


def _object_likeness(o0, o1):
    """How alike two objects look: the mean of each one's exemplars' best
    match among the other's, both ways."""
    s = sim(o0["ex"] @ o1["ex"].T)
    return 0.5 * (float(s.max(axis=1).mean()) + float(s.max(axis=0).mean()))


def consolidate(mem, touched, t, decisions):
    """Standing objects that are one thing recorded twice: at one place, of
    one height, alike, and never in one picture together. Run on the objects
    a visit touched, so that a duplicate made on an earlier day is folded in
    the first time the thing is seen again."""
    for oid in touched:
        o = next((x for x in mem.objects if x["id"] == oid), None)
        if o is None or o["state"] != "present":
            continue
        for other in list(mem.objects):
            if other is o or other["state"] != "present" or other["frames"] & o["frames"]:
                continue
            d = math.hypot(o["x"] - other["x"], o["y"] - other["y"])
            if d > MERGE_M + min(o["extent"], other["extent"]) / 4.0 + \
                    math.sqrt(o["var"] + other["var"]) or \
                    not _heights_agree(o["z"], other["z"]):
                continue
            if _object_colour(o, other) < COLOUR_MIN:
                continue
            cos = _object_likeness(o, other)
            # Two objects nearly on top of each other need less likeness: the
            # place is the stronger evidence (pictures of one painting from two
            # sides score 0.65 at the median, calibrate.py).
            close = d <= min(o["extent"], other["extent"]) / 4.0 + math.sqrt(o["var"] + other["var"])
            if cos < (CLOSE_COS if close else CONSOLIDATE_COS):
                continue
            keep, fold = (o, other) if o["since"] <= other["since"] else (other, o)
            decisions.append({"t": round(t), "merged": fold["id"], "into": keep["id"],
                              "at": [round(fold["x"], 2), round(fold["y"], 2)], "cos": round(cos, 3),
                              "by": "consolidation"})
            mem.merge(keep, fold)
            if fold is o:
                break
            o = keep


def _one_thing(o0, o1):
    """Two gone records of one thing: never in one picture, of one height and
    colour, and alike."""
    return not (o0["frames"] & o1["frames"]) and _heights_agree(o0["z"], o1["z"]) and \
        _object_colour(o0, o1) >= COLOUR_MIN and _object_likeness(o0, o1) >= DOUBLE_COS


def _without_doubles(best, cands):
    """best, [(likeness, k)] sorted with k indexing cands, less every later
    candidate that is the leader's double (ONE_THING_RIVALS)."""
    if not ONE_THING_RIVALS or len(best) < 2:
        return best
    leader = cands[best[0][1]]
    return best[:1] + [b for b in best[1:] if not _one_thing(leader, cands[b[1]])]


def _clear_winner(best, strong, margin, cands):
    """The index of a candidate that wins: alike enough and ahead of the rest."""
    if not best:
        return None
    rivals = _without_doubles(best, cands)
    lead = rivals[0][0] - rivals[1][0] if len(rivals) > 1 else 1.0
    if (best[0][0] >= strong and lead >= margin) or (best[0][0] >= MOVE_LEAD_COS and lead >= MOVE_LEAD):
        return best[0][1]
    return None


def _move_choice(best, gone, t):
    """Recent departures first, then all of them. best is [(likeness, k)],
    sorted, k indexing gone."""
    if RECENT_GONE_S > 0:
        recent = [b for b in best if gone[b[1]].get("gone_at") is not None
                  and t - gone[b[1]]["gone_at"] <= RECENT_GONE_S]
        k = _clear_winner(recent, MOVE_COS, MOVE_MARGIN, gone)
        if k is not None:
            return k
    return None


def settle_pending(mem, t, decisions):
    """A new object is pending for PENDING_S: if, with everything seen of it
    so far, it is clearly an object seen gone, it is that object, moved. The
    arrival may be seen before the old place is seen empty, or first only in
    one picture."""
    gone = [o for o in mem.objects if o["state"] == "gone"]
    if not gone:
        return
    for o in [x for x in mem.objects if x["state"] == "present" and not x.get("settled")
              and t - x["since"] <= PENDING_S and len(x["frames"]) >= 2]:
        best = sorted(((_object_likeness(o, g), k) for k, g in enumerate(gone)
                       if g["since"] < o["since"] and _heights_agree(o["z"], g["z"])
                       and _object_colour(o, g) >= COLOUR_MIN), reverse=True)
        rivals = _without_doubles(best, gone)
        lead = rivals[0][0] - rivals[1][0] if len(rivals) > 1 else 1.0
        k = _move_choice(best, gone, t)
        if k is None and best and ((best[0][0] >= PENDING_COS and lead >= MOVE_MARGIN) or
                                   (best[0][0] >= MOVE_LEAD_COS and lead >= MOVE_LEAD)):
            k = best[0][1]
        if k is not None:
            g = gone[k]
            decisions.append({"t": round(t), "moved": g["id"], "from": [round(g["x"], 2), round(g["y"], 2)],
                              "to": [round(o["x"], 2), round(o["y"], 2)], "cos": round(best[0][0], 3),
                              "runner_up": round(best[1][0], 3) if len(best) > 1 else None,
                              "by": "pending", "was": o["id"]})
            g["x"], g["y"], g["var"], g["extent"], g["z"] = o["x"], o["y"], o["var"], o["extent"], o["z"]
            g["path"].append([round(o["x"], 2), round(o["y"], 2), round(t)])
            g["state"], g["seen"] = "present", o["seen"]
            mem.merge(g, o)
            gone.remove(g)
        elif t - o["since"] > PENDING_S / 2.0 and best and best[0][0] < PENDING_COS - 0.1:
            o["settled"] = True


def run(a, recs):
    L = Looks(a)
    mem = Memory()
    filing, decisions = {}, []
    for idx in windows(a["t"]):
        mem.visit += 1
        clusters = cluster(L, idx)
        t = float(a["t"][idx].max())
        filed = reconcile(L, mem, clusters, t, decisions)
        if recs:
            solid_votes(L, mem, filed, recs)
            absent(L, mem, idx, filed, recs, decisions)
        camera_absent(L, mem, idx, filed, decisions)
        # Sorted: a set of names iterates in an order that changes from run to
        # run, and the order of merges changed the result (2026-10-16).
        consolidate(mem, sorted({mem.resolve(v) for v in filed.values()}), t, decisions)
        settle_pending(mem, t, decisions)
        filing.update({int(a["id"][i]): oid for i, oid in filed.items()})
    return {k: mem.resolve(v) for k, v in filing.items()}, mem, decisions


def _until(a, t):
    keep = a["t"] < t
    return {k: v[keep] for k, v in a.items()}


def fit_scale(looks_path):
    """`python model.py --fit-scale LOOKS.npz`: the scale, from the looks taken
    before SCALE_FITTED_BEFORE, written to SCALE_PATH."""
    a = _until(data.load(looks_path), SCALE_FITTED_BEFORE)
    learned, image = learn_scale(a, a["lvec"])
    np.savez(SCALE_PATH, learned=learned, image=image, before=SCALE_FITTED_BEFORE, regions=len(a["id"]))
    print("scale from %d regions, %d points" % (len(a["id"]), len(learned)))


if __name__ == "__main__":
    args = sys.argv[1:]
    if args[0] == "--fit-scale":
        fit_scale(args[1])
        sys.exit()
    test = "--test" in args
    args = [x for x in args if x != "--test"]
    a = data.load(args[0])
    if not test:
        a = _until(a, DEVELOPMENT_END)
    recs = [recorded.Recording(p) for p in args[2:]]
    filing, mem, decisions = run(a, recs)
    json.dump(filing, open(args[1], "w"))
    json.dump(decisions, open(args[1] + ".decisions.json", "w"), indent=0)
    print("%d regions filed of %d, %d objects (%d gone), %d moves, %d merges, %d gone events" % (
        len(filing), len(a["id"]), len(mem.objects), sum(o["state"] == "gone" for o in mem.objects),
        sum("moved" in d for d in decisions), sum("merged" in d for d in decisions),
        sum("gone" in d for d in decisions)))
