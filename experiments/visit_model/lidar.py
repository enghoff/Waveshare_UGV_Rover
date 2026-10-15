"""The lidar's part in step 3: where a region stands, and whether a remembered
place has since been seen empty.

The lidar sweeps one plane about 20 cm above the floor, so it speaks only for
things that reach down into it. A remembered place is judged by it only if the
lidar hit something there when the place was seen (`solid`).
"""
import math

import numpy as np

import recorded

#: Returns across the middle half of a region's width, never under 1.5 degrees
#: either side; the 25th percentile of them is where the region stands (on the
#: controls of 2026-10-11, 57% of these land within 0.3 m of the depth camera's
#: own range for the same region, against 48% for the nearest within 3 degrees).
MIN_HALF_DEG = 1.5
PLACE_PERCENTILE = 25
MAX_M = 6.0
#: A remembered place is solid when a return lands within this of it.
SOLID_M = 0.3
#: A beam passes a place when every beam within FREE_HALF_DEG of it returns
#: from at least FREE_BEYOND_M farther; it hits it when one ends within
#: HIT_M. Places nearer than NEAR_M or farther than FAR_M are not judged.
FREE_HALF_DEG = 1.5
FREE_BEYOND_M = 0.3
HIT_M = 0.25
NEAR_M = 0.4
FAR_M = 4.0
#: Every this-many scans is enough: the lidar turns ten times a second.
EVERY = 3


def _scan_near(rec, t):
    k = int(np.argmin(np.abs(rec.scan_t - t)))
    if abs(rec.scan_t[k] - t) > 0.3:
        return None
    return k


def _returns(rec, k, bearing_deg, half_deg):
    p = rec.pose(rec.scan_t[k])
    if p is None:
        return None, None
    _s, _r, amin, ainc, _ti, _st = rec.meta[k]
    r = rec.ranges[k]
    ang = np.degrees(amin + ainc * np.arange(len(r))) + p[2]
    near = np.abs((ang - bearing_deg + 180.0) % 360.0 - 180.0) <= half_deg
    ok = near & np.isfinite(r) & (r > 0.15) & (r < MAX_M)
    return p, r[ok]


def place(recs, t, bearing_deg, span_deg):
    """Where a region stands by the lidar at its shutter: (x, y) or None."""
    rec = recorded.for_time(recs, t)
    if rec is None or not math.isfinite(bearing_deg):
        return None
    k = _scan_near(rec, t)
    if k is None:
        return None
    half = max(MIN_HALF_DEG, (span_deg if math.isfinite(span_deg) else 0.0) / 2.0)
    p, r = _returns(rec, k, bearing_deg, half)
    if p is None or len(r) == 0:
        return None
    d = float(np.percentile(r, PLACE_PERCENTILE))
    b = math.radians(bearing_deg)
    return p[0] + d * math.cos(b), p[1] + d * math.sin(b)


def solid(recs, t, x, y):
    """Did the lidar hit something within SOLID_M of (x, y) at time t?"""
    rec = recorded.for_time(recs, t)
    if rec is None:
        return False
    k = _scan_near(rec, t)
    if k is None:
        return False
    p = rec.pose(rec.scan_t[k])
    if p is None:
        return False
    d = math.hypot(x - p[0], y - p[1])
    b = math.degrees(math.atan2(y - p[1], x - p[0]))
    _p, r = _returns(rec, k, b, FREE_HALF_DEG)
    return r is not None and len(r) > 0 and bool(np.any(np.abs(r - d) <= SOLID_M))


def _beam_view(rec, k):
    """One scan as (x, y, heading, angle_min, increment, ranges with invalid NaN)."""
    p = rec.pose(rec.scan_t[k])
    if p is None:
        return None
    _s, _r, amin, ainc, _ti, _st = rec.meta[k]
    r = rec.ranges[k].astype(np.float64)
    r[~(np.isfinite(r) & (r > 0.15) & (r < MAX_M + 2))] = np.nan
    return p[0], p[1], p[2], amin, ainc, r


def outcomes(recs, xs, ys, start, end, after=None, newest_first=False, limit=None):
    """Pass and hit counts at each place over the scans in [start, end].

    after (per place) skips scans before it. With newest_first and limit, only
    each place's latest `limit` judged scans count: the latest state."""
    n = len(xs)
    passes = np.zeros(n, int)
    hits = np.zeros(n, int)
    if n == 0:
        return passes, hits
    after = np.full(n, -np.inf) if after is None else np.asarray(after)
    for rec in sorted(recs, key=lambda r: r.start, reverse=newest_first):
        ks = np.flatnonzero((rec.scan_t >= start) & (rec.scan_t <= end))[::EVERY]
        if newest_first:
            ks = ks[::-1]
        for k in ks:
            t = rec.scan_t[k]
            live = after < t
            if limit is not None:
                live &= (passes + hits) < limit
            if not live.any():
                if newest_first:
                    break
                continue
            view = _beam_view(rec, k)
            if view is None:
                continue
            x0, y0, th, amin, ainc, r = view
            idx = np.flatnonzero(live)
            dx, dy = xs[idx] - x0, ys[idx] - y0
            d = np.hypot(dx, dy)
            near = (d >= NEAR_M) & (d <= FAR_M)
            idx, d = idx[near], d[near]
            if len(idx) == 0:
                continue
            ang = np.arctan2(dy[near], dx[near]) - math.radians(th)
            beam = np.round(((ang - amin) % (2 * math.pi)) / ainc).astype(int)
            w = max(1, int(round(math.radians(FREE_HALF_DEG) / ainc)))
            m = len(r)
            cols = (beam[:, None] + np.arange(-w, w + 1)[None, :]) % m
            rr = r[cols]
            valid = np.isfinite(rr)
            hit = (valid & (np.abs(rr - d[:, None]) <= HIT_M)).any(axis=1)
            beyond = valid.all(axis=1) & (np.nan_to_num(rr, nan=0.0) >= d[:, None] + FREE_BEYOND_M).all(axis=1)
            passes[idx[beyond & ~hit]] += 1
            hits[idx[hit]] += 1
    return passes, hits
