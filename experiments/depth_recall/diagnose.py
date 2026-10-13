"""The development checks run before the freeze, and the diagnostics run after
scoring, so their numbers can be reproduced. Neither decides anything.

    python diagnose.py development --frames <dir> --store <world.db copy>
    python diagnose.py after --frames <dir> --store <world.db copy>

`development` scores the seven pairs among the 21 looks that are not targets
(same rule as pairs.json but gaps from 5 s and baselines from 0.1 m), with
search bounds: target heading -12..+12 deg in 1.5 steps; target tilt -6..+6 in
1.5; a common tilt on both looks -8..+4 in 2; the camera 0.10 m behind or ahead.

`after` is post-hoc and target-assisted, on the six targets: useful area at a
looser 25 cm / 10%; wrong predictions split nearer and farther; accuracy by the
target's depth band; and the best of a heading search -9..+9 in 1.5 steps
crossed with tilt -3, 0, +3, which uses the target's depth and so could only
show what a perfect pose correction would buy.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sqlite3
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geometry as g  # noqa: E402
import predict as p  # noqa: E402
import score as s  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def warp(frames, store, sources, target, dyaw=0.0, dtilt=0.0, common_tilt=0.0):
    mt = p.look(store, target)
    pose = dict(mt["pose"])
    pose["heading_deg"] += dyaw
    truth, header = g.read_depth(os.path.join(frames, f"{target}.depth.gz"))
    r_t, t_t = g.camera_pose(pose, mt["pan"], mt["tilt"] + dtilt + common_tilt, mt["when"])
    canvas = np.full(truth.shape, np.inf)
    for source in sources:
        depth, sh = g.read_depth(os.path.join(frames, f"{source}.depth.gz"))
        ms = p.look(store, source)
        r_s, t_s = g.camera_pose(ms["pose"], ms["pan"], ms["tilt"] + common_tilt, ms["when"])
        p.draw(p.surface_points(depth, sh, r_s, t_s), sh, r_t, t_t, canvas)
    return canvas, truth, header, mt


def development(frames, store):
    targets = {pair["target"] for pair in json.load(open(os.path.join(HERE, "pairs.json")))["pairs"]}
    ids = [os.path.basename(f).split(".")[0] for f in sorted(glob.glob(os.path.join(frames, "*.depth.gz")))]
    looks = {i: p.look(store, i) for i in ids}
    pairs = []
    for a in ids:
        for b in ids:
            if b in targets or a == b or looks[b]["when"] - looks[a]["when"] < 5:
                continue
            pa, pb = looks[a]["pose"], looks[b]["pose"]
            d = math.hypot(pa["x_m"] - pb["x_m"], pa["y_m"] - pb["y_m"])
            dh = abs((pa["heading_deg"] - pb["heading_deg"] + 180) % 360 - 180)
            if 0.1 <= d <= 1.0 and dh <= 20:
                pairs.append((a, b))
    out = {}
    for a, b in pairs:
        pred, truth, _, _ = warp(frames, store, [a], b)
        measured = s.surface_mask(truth)
        copy, _ = g.read_depth(os.path.join(frames, f"{a}.depth.gz"))
        copy[(copy < p.MIN_M) | (copy > p.MAX_M)] = np.inf
        row = {"warped": s.measure(pred, truth, measured)["useful"],
               "copy": s.measure(copy, truth, measured)["useful"]}
        row["best_heading"] = max((s.measure(warp(frames, store, [a], b, dyaw=y)[0], truth, measured)["useful"], float(y))
                                  for y in np.arange(-12, 12.1, 1.5))
        row["best_target_tilt"] = max((s.measure(warp(frames, store, [a], b, dtilt=t)[0], truth, measured)["useful"], float(t))
                                      for t in np.arange(-6, 6.1, 1.5))
        row["common_tilt"] = {int(t): s.measure(warp(frames, store, [a], b, common_tilt=t)[0], truth, measured)["useful"]
                              for t in range(-8, 5, 2)}
        out[f"{a}->{b}"] = row
        print(a[-13:], "->", b[-13:], json.dumps(row))
    return out


def after(frames, store):
    labels = json.load(open(os.path.join(HERE, "labels.json")))["exclude_fisheye_boxes"]
    record = json.load(open(os.path.join(HERE, "predictions", "predictions.json")))
    out = {}
    for target, info in sorted(record["targets"].items()):
        _, truth, header, mt = warp(frames, store, [], target)
        keep = s.label_mask(header, labels.get(target, []), mt["when"])
        measured = s.surface_mask(truth) & keep
        pred = np.load(os.path.join(HERE, "predictions", info["file"]))["warped"]
        have = np.isfinite(pred) & measured
        e, z = (pred - truth)[have], truth[have]
        tol = np.maximum(0.15, 0.05 * z)
        row = {"loose_useful_25cm_10pc": float((np.abs(e) <= np.maximum(0.25, 0.10 * z)).sum() / measured.sum()),
               "wrong_pred_nearer": float((e < -tol).sum() / measured.sum()),
               "wrong_pred_farther": float((e > tol).sum() / measured.sum())}
        for lo, hi in [(0, 2), (2, 3), (3, 4), (4, 6)]:
            band = (z >= lo) & (z < hi)
            row[f"acc_{lo}-{hi}m"] = f"{(np.abs(e[band]) <= tol[band]).mean():.0%} of {band.sum()}" if band.any() else "-"
        best = (0.0, (0.0, 0))
        for dyaw in np.arange(-9, 9.1, 1.5):
            for dtilt in (-3, 0, 3):
                u = s.measure(warp(frames, store, info["sources"], target, dyaw, dtilt)[0], truth, measured)["useful"]
                if u > best[0]:
                    best = (u, (float(dyaw), dtilt))
        row["best_shift_useful"], row["best_shift_yaw_tilt"] = round(best[0], 3), best[1]
        out[target] = row
        print(target[-13:], json.dumps(row))
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("what", choices=["development", "after"])
    parser.add_argument("--frames", required=True)
    parser.add_argument("--store", required=True)
    args = parser.parse_args()
    store = sqlite3.connect(f"file:{args.store}?mode=ro", uri=True)
    result = development(args.frames, store) if args.what == "development" else after(args.frames, store)
    json.dump(result, open(os.path.join(HERE, f"{args.what}.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
