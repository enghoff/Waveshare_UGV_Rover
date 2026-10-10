"""What has changed between two recordings, from the lidar alone.

Each recording (an npz from bag_extract.py) is turned into counts per 5 cm cell
of the map frame: how often a beam ended there (hit) and how often one passed
through it (pass). The rover's pose at each scan is map -> odom (the mapper's
correction) composed with odom -> base_link, both as recorded.

A cell is GONE when the reference saw it solid and the test sees through it; it
is NEW when the reference saw through it and the test sees it solid. Both
allow for the poses being a few centimetres out by dilating the reference's
solid cells (a NEW hit next to something the reference had is not new) and the
test's solid cells (a GONE cell next to something the test still hits is not
gone). Changed cells are grouped into clusters, and a cluster is kept only if
its evidence spans more than PERSIST_S of the test recording, so a person
walking past is not a change.

    python change_detect.py REF.npz TEST.npz [--png out.png]
"""
import math, sys

import numpy as np
from scipy import ndimage

RES = 0.05
X0, Y0, NX, NY = -24.0, -22.0, 400, 360          # the flat with room to spare
MAX_RANGE = 6.0
MIN_RANGE = 0.12
SCAN_EVERY = 3                                    # about 3 scans a second
MAX_TURN_DPS = 40.0
STEP = RES / 2.0
SOLID = 0.6                                       # hit share to call a cell solid
CLEAR = 0.1                                       # hit share to call it seen through
MIN_OBS = 6                                       # beams a cell needs to be judged
DILATE = 2                                        # cells of slack for pose error
PERSIST_S = 20.0
MIN_CELLS = 6


def interp_pose(rows, t):
    """(x, y, yaw) from rows (stamp, recv, x, y, yaw) at time t, or None."""
    st = rows[:, 0]
    i = np.searchsorted(st, t)
    if i <= 0 or i >= len(st):
        return None
    a, b = rows[i - 1], rows[i]
    if b[0] - a[0] > 0.5:
        return None
    f = (t - a[0]) / max(1e-9, b[0] - a[0])
    dyaw = math.atan2(math.sin(b[4] - a[4]), math.cos(b[4] - a[4]))
    return a[2] + f * (b[2] - a[2]), a[3] + f * (b[3] - a[3]), a[4] + f * dyaw


def last_pose(rows, t):
    st = rows[:, 0]
    i = np.searchsorted(st, t) - 1
    return None if i < 0 else tuple(rows[i, 2:5])


def compose(a, b):
    x, y, th = a
    bx, by, bth = b
    return (x + bx * math.cos(th) - by * math.sin(th),
            y + bx * math.sin(th) + by * math.cos(th), th + bth)


def counts(path):
    d = np.load(path)
    ranges, meta = d["ranges"], d["scan_meta"]
    odom = d["odom_tf"][np.argsort(d["odom_tf"][:, 0])]
    mapt = d["map_tf"][np.argsort(d["map_tf"][:, 0])]
    hits = np.zeros((NY, NX), np.int32)
    passes = np.zeros((NY, NX), np.int32)
    first = np.full((NY, NX), np.inf)
    last = np.full((NY, NX), -np.inf)
    t0 = meta[0, 0]
    used = 0
    prev = None
    for k in range(0, len(ranges), SCAN_EVERY):
        stamp, _recv, amin, ainc, _tinc, stime = meta[k]
        t = stamp + stime / 2.0
        ob = interp_pose(odom, t)
        mo = last_pose(mapt, t)
        if ob is None or mo is None:
            continue
        if prev is not None and t > prev[0]:
            rate = abs(math.degrees(math.atan2(math.sin(ob[2] - prev[1]),
                                               math.cos(ob[2] - prev[1])))) / (t - prev[0])
            if rate > MAX_TURN_DPS:
                prev = (t, ob[2])
                continue
        prev = (t, ob[2])
        x, y, th = compose(mo, ob)
        r = ranges[k]
        ang = amin + ainc * np.arange(len(r))
        ok = np.isfinite(r) & (r > MIN_RANGE) & (r < MAX_RANGE)
        r, ang = r[ok], ang[ok] + th
        # Hits.
        hx = ((x + r * np.cos(ang) - X0) / RES).astype(int)
        hy = ((y + r * np.sin(ang) - Y0) / RES).astype(int)
        inside = (hx >= 0) & (hx < NX) & (hy >= 0) & (hy < NY)
        np.add.at(hits, (hy[inside], hx[inside]), 1)
        # Passes: samples along each beam short of the end by a cell.
        n = np.floor((r - RES) / STEP).astype(int)
        if n.max(initial=0) > 0:
            s = np.arange(n.max()) * STEP + STEP
            mask = s[None, :] < (r - RES)[:, None]
            px = x + s[None, :] * np.cos(ang)[:, None]
            py = y + s[None, :] * np.sin(ang)[:, None]
            cx = ((px[mask] - X0) / RES).astype(int)
            cy = ((py[mask] - Y0) / RES).astype(int)
            inside = (cx >= 0) & (cx < NX) & (cy >= 0) & (cy < NY)
            cx, cy = cx[inside], cy[inside]
            # Count each cell once per scan, however many samples fell in it.
            flat = np.unique(cy * NX + cx)
            passes.flat[flat] += 1
            first.flat[flat] = np.minimum(first.flat[flat], t - t0)
            last.flat[flat] = np.maximum(last.flat[flat], t - t0)
        used += 1
    # When each hit cell was seen, for persistence.
    return {"hits": hits, "passes": passes, "first": first, "last": last,
            "scans": used, "span_s": float(meta[-1, 0] - meta[0, 0])}


def hit_times(path):
    """For each cell, the first and last time a beam ended in it."""
    d = np.load(path)
    ranges, meta = d["ranges"], d["scan_meta"]
    odom = d["odom_tf"][np.argsort(d["odom_tf"][:, 0])]
    mapt = d["map_tf"][np.argsort(d["map_tf"][:, 0])]
    first = np.full((NY, NX), np.inf)
    last = np.full((NY, NX), -np.inf)
    t0 = meta[0, 0]
    for k in range(0, len(ranges), SCAN_EVERY):
        stamp, _recv, amin, ainc, _tinc, stime = meta[k]
        t = stamp + stime / 2.0
        ob = interp_pose(odom, t)
        mo = last_pose(mapt, t)
        if ob is None or mo is None:
            continue
        x, y, th = compose(mo, ob)
        r = ranges[k]
        ang = amin + ainc * np.arange(len(r))
        ok = np.isfinite(r) & (r > MIN_RANGE) & (r < MAX_RANGE)
        r, ang = r[ok], ang[ok] + th
        hx = ((x + r * np.cos(ang) - X0) / RES).astype(int)
        hy = ((y + r * np.sin(ang) - Y0) / RES).astype(int)
        inside = (hx >= 0) & (hx < NX) & (hy >= 0) & (hy < NY)
        flat = np.unique(hy[inside] * NX + hx[inside])
        first.flat[flat] = np.minimum(first.flat[flat], t - t0)
        last.flat[flat] = np.maximum(last.flat[flat], t - t0)
    return first, last


def judge(c):
    obs = c["hits"] + c["passes"]
    share = np.where(obs > 0, c["hits"] / np.maximum(obs, 1), np.nan)
    solid = (obs >= MIN_OBS) & (share >= SOLID)
    clear = (obs >= MIN_OBS) & (share <= CLEAR)
    return solid, clear


def clusters(mask, tfirst, tlast, label):
    lab, n = ndimage.label(ndimage.binary_dilation(mask, iterations=1) & mask
                           | mask, structure=np.ones((3, 3)))
    out = []
    for i in range(1, n + 1):
        cells = lab == i
        size = int(cells.sum())
        if size < MIN_CELLS:
            continue
        span = float(np.nanmax(np.where(cells, tlast, -np.inf)) -
                     np.nanmin(np.where(cells, tfirst, np.inf)))
        ys, xs = np.nonzero(cells)
        out.append({"kind": label, "cells": size, "area_m2": round(size * RES * RES, 3),
                    "x_m": round(X0 + (xs.mean() + 0.5) * RES, 2),
                    "y_m": round(Y0 + (ys.mean() + 0.5) * RES, 2),
                    "extent_m": round(RES * max(np.ptp(xs) + 1, np.ptp(ys) + 1), 2),
                    "span_s": round(span, 1)})
    return out


def compare(ref_path, test_path):
    ref, test = counts(ref_path), counts(test_path)
    r_solid, r_clear = judge(ref)
    t_solid, t_clear = judge(test)
    grow = np.ones((2 * DILATE + 1, 2 * DILATE + 1), bool)
    r_solid_wide = ndimage.binary_dilation(r_solid, structure=grow)
    t_solid_wide = ndimage.binary_dilation(t_solid, structure=grow)
    gone = r_solid & t_clear & ~t_solid_wide
    new = t_solid & r_clear & ~r_solid_wide
    hf, hl = hit_times(test_path)
    found = (clusters(gone, test["first"], test["last"], "GONE")
             + clusters(new, hf, hl, "NEW"))
    kept = [c for c in found if c["span_s"] >= PERSIST_S]
    return ref, test, gone, new, found, kept, (r_solid, t_solid)


if __name__ == "__main__":
    ref_path, test_path = sys.argv[1], sys.argv[2]
    ref, test, gone, new, found, kept, solids = compare(ref_path, test_path)
    print(f"ref {ref_path}: {ref['scans']} scans used over {ref['span_s']:.0f} s")
    print(f"test {test_path}: {test['scans']} scans used over {test['span_s']:.0f} s")
    print(f"changed cells: gone {int(gone.sum())}, new {int(new.sum())}")
    print("clusters (all, then kept as persistent):")
    for c in sorted(found, key=lambda c: -c["cells"])[:15]:
        print("  ", c, "KEPT" if c in kept else "")
    if "--png" in sys.argv:
        from PIL import Image
        r_solid, t_solid = solids
        img = np.full((NY, NX, 3), 255, np.uint8)
        img[r_solid] = (150, 150, 150)
        img[t_solid & ~r_solid] = (90, 90, 200)
        img[gone] = (220, 30, 30)
        img[new] = (30, 160, 30)
        Image.fromarray(img[::-1]).resize((NX * 2, NY * 2), Image.NEAREST).save(
            sys.argv[sys.argv.index("--png") + 1])
