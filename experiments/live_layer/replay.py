#!/usr/bin/env python3
"""Recompute the live layer's marks from a `scan_record.py` recording, two ways.

    python experiments/live_layer/replay.py rec.json

`latest` places each scan with the transform as it is when the costmap updates,
which is what `live_obstacle_layer.cpp` does today. `at_stamp` places it where
the laser was when the scan was taken. Both then apply the layer's own rule: a
point within `wall_margin_m` of a cell the map already has as a wall is left
alone, any other point within range marks its cell.

What it reports:

* how many of the rover's own marks (`nav_grid`'s `live`, sampled at 2 Hz) each
  model explains -- the check that the model is the layer at all;
* marks per scan against how fast the rover was turning, for both models: if
  turning makes marks that the scan's own pose does not, the transform's timing
  is the cause;
* how far the marks lie from the nearest mapped wall, standing and turning.
"""
from __future__ import annotations

import base64
import json
import math
import sys
import zlib

import numpy as np

MAX_RANGE_M, MIN_RANGE_M, WALL_MARGIN_M = 3.0, 0.15, 0.10
#: The layer's update window: a mark sampled at t came from a scan received in
#: the half second before it (the costmap updates twice a second).
WINDOW_S = 0.6
TURNING_DPS = 15.0


def load(path: str):
    d = json.load(open(path))
    m = d["map"]
    grid = np.frombuffer(zlib.decompress(base64.b64decode(m["data"])), dtype=np.int8)
    grid = grid.reshape(m["height"], m["width"])
    return d, grid, m["resolution_m"], m["origin_x_m"], m["origin_y_m"]


def marks_of(scan: dict, pose, lethal_near: np.ndarray, res, ox, oy) -> set:
    if pose is None:
        return set()
    x0, y0, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    near = max(MIN_RANGE_M, scan["rmin"])
    far = min(MAX_RANGE_M, scan["rmax"])
    out = set()
    h, w = lethal_near.shape
    for i, r in enumerate(scan["ranges"]):
        if r is None or r < near or r > far:
            continue
        a = scan["amin"] + i * scan["ainc"]
        lx, ly = r * math.cos(a), r * math.sin(a)
        x, y = x0 + c * lx - s * ly, y0 + s * lx + c * ly
        col, row = int((x - ox) / res), int((y - oy) / res)
        if 0 <= col < w and 0 <= row < h and not lethal_near[row, col]:
            out.add((col, row))
    return out


def main(path: str) -> int:
    d, grid, res, ox, oy = load(path)
    lethal = grid >= 65
    k = int(round(WALL_MARGIN_M / res))
    near = np.zeros_like(lethal)
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            if dx * dx + dy * dy <= k * k:
                near |= np.roll(np.roll(lethal, dy, 0), dx, 1)
    wy, wx = np.nonzero(lethal)
    wall_xy = np.stack([ox + (wx + 0.5) * res, oy + (wy + 0.5) * res], 1)

    scans = [s for s in d["scans"] if s.get("latest") and s.get("at_stamp")]
    for one in scans:
        one["m_latest"] = marks_of(one, one["latest"], near, res, ox, oy)
        one["m_stamp"] = marks_of(one, one["at_stamp"], near, res, ox, oy)
    for a, b in zip(scans, scans[1:]):
        dt = b["stamp"] - a["stamp"]
        dyaw = math.atan2(math.sin(b["at_stamp"][2] - a["at_stamp"][2]),
                          math.cos(b["at_stamp"][2] - a["at_stamp"][2]))
        b["turn_dps"] = abs(math.degrees(dyaw) / dt) if dt > 0 else 0.0
    scans[0]["turn_dps"] = 0.0

    def cell(x, y):
        return int((x - ox) / res), int((y - oy) / res)

    explained = {"latest": 0, "at_stamp": 0}
    observed_total = 0
    for sample in d["marks"]:
        live = sample.get("live") or []
        if not live:
            continue
        t = sample["wall"]
        window = [s for s in scans if t - WINDOW_S <= s["recv"] <= t]
        if not window:
            continue
        got_l = set().union(*(s["m_latest"] for s in window))
        got_s = set().union(*(s["m_stamp"] for s in window))
        for x, y in live:
            c = cell(x, y)
            ring = {(c[0] + dx, c[1] + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)}
            observed_total += 1
            explained["latest"] += bool(ring & got_l)
            explained["at_stamp"] += bool(ring & got_s)
    print(f"{len(scans)} scans with both poses; {observed_total} of the rover's own marks")
    for name, n in explained.items():
        share = n / observed_total if observed_total else float("nan")
        print(f"  explained by {name:8s} {n:5d}  ({share:.0%})")

    def describe(rows, key):
        counts = [len(s[key]) for s in rows]
        return (f"{np.mean(counts):5.1f} per scan" if counts else "  none")

    still = [s for s in scans if s["turn_dps"] < TURNING_DPS]
    turning = [s for s in scans if s["turn_dps"] >= TURNING_DPS]
    print(f"\nmarks per scan          standing/slow ({len(still)})   turning >= {TURNING_DPS:.0f} deg/s ({len(turning)})")
    for key in ("m_latest", "m_stamp"):
        print(f"  {key[2:]:9s}             {describe(still, key)}            {describe(turning, key)}")

    def wall_gap(cells):
        if not cells:
            return []
        xy = np.array([[ox + (c + 0.5) * res, oy + (r + 0.5) * res] for c, r in cells])
        return [float(np.min(np.hypot(*(wall_xy - p).T))) for p in xy]

    for label, rows in (("standing/slow", still), ("turning", turning)):
        only_latest = set().union(*(s["m_latest"] - s["m_stamp"] for s in rows)) if rows else set()
        both = set().union(*(s["m_latest"] & s["m_stamp"] for s in rows)) if rows else set()
        g1, g2 = wall_gap(only_latest), wall_gap(both)
        print(f"\n{label}: cells marked only with the latest transform {len(only_latest)}"
              + (f", median {np.median(g1):.2f} m from a mapped wall" if g1 else ""))
        print(f"{label}: cells marked either way {len(both)}"
              + (f", median {np.median(g2):.2f} m from a mapped wall" if g2 else ""))
    ages = [s["recv"] - s["stamp"] for s in scans]
    if ages:
        print(f"\nscan age at receipt: median {np.median(ages):.3f} s, max {max(ages):.3f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1]))
