"""Score the frozen predictions against the targets' own depth maps.

Run only after predict.py has written its predictions and they are committed.
The decision rules are protocol.md's and are applied here, not by eye:

* a target counts only its **measured surface**: pixels whose depth lies in
  0.3-6 m and that belong to at least one 2x2 block whose depths agree (the rule
  the sources pass, predict.surface_points), outside the labelled changes;
* a pixel is **useful** when predicted within max(0.15 m, 5% of the target's
  depth); **unavailable** when nothing was predicted there;
* stop if fewer than three of the six targets reach 30% useful area, or if
  warping does not beat copying (better on at least four of six targets and a
  positive median difference). No alignment step exists, so the third stop rule
  (works only when aligned on changed surfaces) cannot be triggered.
"""
from __future__ import annotations

import argparse
import json
import os
import sqlite3
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geometry as g  # noqa: E402
import predict as p  # noqa: E402

TOL_ABS_M, TOL_REL = 0.15, 0.05
USEFUL_BAR = 0.30
TARGETS_NEEDED = 3


def surface_mask(depth: np.ndarray) -> np.ndarray:
    good = (depth >= p.MIN_M) & (depth <= p.MAX_M)
    stack = np.stack([depth[:-1, :-1], depth[:-1, 1:], depth[1:, :-1], depth[1:, 1:]])
    span = stack.max(0) - stack.min(0)
    block = (good[:-1, :-1] & good[:-1, 1:] & good[1:, :-1] & good[1:, 1:]
             & (span <= np.maximum(p.EDGE_ABS_M, p.EDGE_REL * stack.min(0))))
    mask = np.zeros_like(good)
    mask[:-1, :-1] |= block
    mask[:-1, 1:] |= block
    mask[1:, :-1] |= block
    mask[1:, 1:] |= block
    return mask


def label_mask(header: dict, boxes: list[list[float]], when: float) -> np.ndarray:
    """True where a pixel is outside every labelled change (fisheye boxes)."""
    fx, fy = g.oak_pixels_in_fisheye(header, when=when)
    keep = np.ones(fx.shape, dtype=bool)
    for x0, y0, x1, y1 in boxes:
        keep &= ~((fx >= x0) & (fx <= x1) & (fy >= y0) & (fy <= y1))
    return keep


def measure(pred: np.ndarray, truth: np.ndarray, region: np.ndarray) -> dict:
    tol = np.maximum(TOL_ABS_M, TOL_REL * truth)
    have = np.isfinite(pred) & region
    near = have & (np.abs(pred - truth) <= tol)
    n = int(region.sum())
    errors = np.abs(pred - truth)[have]
    return {"pixels": n,
            "useful": near.sum() / n if n else float("nan"),
            "wrong": (have & ~near).sum() / n if n else float("nan"),
            "unavailable": (region & ~np.isfinite(pred)).sum() / n if n else float("nan"),
            "accuracy_where_predicted": near.sum() / have.sum() if have.any() else float("nan"),
            "abs_error_p50_m": float(np.median(errors)) if errors.size else float("nan"),
            "abs_error_p75_m": float(np.percentile(errors, 75)) if errors.size else float("nan")}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", required=True)
    parser.add_argument("--store", required=True)
    parser.add_argument("--labels", required=True)
    parser.add_argument("--predictions", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    store = sqlite3.connect(f"file:{args.store}?mode=ro", uri=True)
    labels = json.load(open(args.labels))["exclude_fisheye_boxes"]
    record = json.load(open(os.path.join(args.predictions, "predictions.json")))
    results = {}
    for target, info in sorted(record["targets"].items()):
        truth, header = g.read_depth(os.path.join(args.frames, f"{target}.depth.gz"))
        if {k: header[k] for k in ("width", "height", "lens")} != record["lens"]:
            raise SystemExit(f"{target} was predicted through another lens")
        when = p.look(store, target)["when"]
        keep = label_mask(header, labels.get(target, []), when)
        measured = surface_mask(truth) & keep
        raw = (truth >= p.MIN_M) & (truth <= p.MAX_M) & keep
        saved = np.load(os.path.join(args.predictions, info["file"]))
        row = {"sources": info["sources"], "copy_from": info["copy_from"],
               "excluded_share": float(1 - keep.mean()),
               "measured_share_of_grid": float(measured.mean()),
               "warped": measure(saved["warped"], truth, measured),
               "copy": measure(saved["copy"], truth, measured),
               "warped_raw_valid": measure(saved["warped"], truth, raw),
               "copy_raw_valid": measure(saved["copy"], truth, raw)}
        for source in info["sources"]:
            row[f"alone_{source}"] = measure(saved[f"alone_{source}"], truth, measured)
        results[target] = row

    useful = {t: r["warped"]["useful"] for t, r in results.items()}
    gain = {t: r["warped"]["useful"] - r["copy"]["useful"] for t, r in results.items()}
    reached = sum(u >= USEFUL_BAR for u in useful.values())
    better = sum(d > 0 for d in gain.values())
    stop = []
    if reached < TARGETS_NEEDED:
        stop.append(f"only {reached} of {len(useful)} targets reach {USEFUL_BAR:.0%} useful area")
    if not (better >= 4 and float(np.median(list(gain.values()))) > 0):
        stop.append(f"warping beats copying on {better} of {len(gain)} targets, "
                    f"median gain {np.median(list(gain.values())):+.1%}")
    verdict = {"targets_reaching_bar": reached, "warp_better_than_copy": better,
               "median_gain": float(np.median(list(gain.values()))),
               "stop": stop, "continue": not stop}
    json.dump({"results": results, "verdict": verdict}, open(args.out, "w"), indent=1)
    for target, r in results.items():
        w, c = r["warped"], r["copy"]
        print(f"{target}: measured {r['measured_share_of_grid']:.0%} of grid; warped useful "
              f"{w['useful']:.0%} (unavailable {w['unavailable']:.0%}, wrong {w['wrong']:.0%}, "
              f"|err| p50 {w['abs_error_p50_m']:.2f} m); copy useful {c['useful']:.0%}")
    print("verdict:", "continue" if verdict["continue"] else "STOP: " + "; ".join(stop))


if __name__ == "__main__":
    main()
