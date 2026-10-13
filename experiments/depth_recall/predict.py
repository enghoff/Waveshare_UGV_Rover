"""Blind depth recall: predict a look's OAK depth from earlier looks alone.

Each source look's depth map is turned into map points and drawn into the
target camera with a z-buffer; the nearest surface wins. **The target's own
depth map is never opened here**: its pose, gimbal angles and moment come from
the world store and its lens from a source's header (one device, one lens), and
that is asserted rather than assumed when scoring.

Densified by sub-sampling each 2x2 block of source pixels whose four depths
agree (the block is one surface), so a target that stands closer than the
source does not see through gaps between splats; a block whose depths disagree
is a depth edge and is dropped. Run:

    python predict.py --frames <dir of .jpg/.depth.gz> --store <world.db copy>
        --pairs pairs.json --out predictions/
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sqlite3
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geometry as g  # noqa: E402

#: Source depth outside this band is not used (passive stereo, no projector).
MIN_M, MAX_M = 0.3, 6.0
#: A 2x2 block is one surface when its depths span no more than this.
EDGE_ABS_M, EDGE_REL = 0.05, 0.05
#: Sub-samples per block side.
SUBDIVIDE = 4
#: Nothing nearer the target camera than this is drawn.
NEAR_M = 0.2


def look(store: sqlite3.Connection, frame_id: str) -> dict:
    row = store.execute(
        "select observer_pose_json, observer_pan_deg, observer_tilt_deg, observed_at"
        " from observations where frame_id=? limit 1", (frame_id,)).fetchone()
    pose, pan, tilt, when = row
    return {"pose": json.loads(pose), "pan": float(pan or 0.0),
            "tilt": float(tilt or 0.0), "when": float(when)}


def surface_points(depth: np.ndarray, header: dict, rotation, translation):
    """Map points sub-sampled over every 2x2 block that is one surface."""
    x, y = g.pixel_grid(header)
    good = (depth >= MIN_M) & (depth <= MAX_M)
    d00, d01, d10, d11 = depth[:-1, :-1], depth[:-1, 1:], depth[1:, :-1], depth[1:, 1:]
    stack = np.stack([d00, d01, d10, d11])
    span = stack.max(0) - stack.min(0)
    block = (good[:-1, :-1] & good[:-1, 1:] & good[1:, :-1] & good[1:, 1:]
             & (span <= np.maximum(EDGE_ABS_M, EDGE_REL * stack.min(0))))
    rows, cols = np.nonzero(block)
    steps = (np.arange(SUBDIVIDE) + 0.5) / SUBDIVIDE
    a, b = np.meshgrid(steps, steps)            # a along columns, b along rows
    a, b = a.reshape(1, -1), b.reshape(1, -1)

    def corner(arr, dr, dc):
        return arr[rows + dr, cols + dc].reshape(-1, 1)

    def blend(arr):
        return ((1 - a) * (1 - b) * corner(arr, 0, 0) + a * (1 - b) * corner(arr, 0, 1)
                + (1 - a) * b * corner(arr, 1, 0) + a * b * corner(arr, 1, 1))

    z, xs, ys = blend(depth), blend(x), blend(y)
    sensor = np.stack([xs * z, ys * z, z], axis=-1).reshape(-1, 3)
    return sensor @ rotation.T + translation


def draw(points: np.ndarray, header: dict, rotation, translation,
         canvas: np.ndarray) -> None:
    """Z-buffer the points into `canvas` (metres, inf where nothing)."""
    column, row, z = g.project(points, header, rotation, translation)
    c, r = np.rint(column).astype(np.int64), np.rint(row).astype(np.int64)
    ok = (z > NEAR_M) & (c >= 0) & (c < header["width"]) & (r >= 0) & (r < header["height"])
    np.minimum.at(canvas, (r[ok], c[ok]), z[ok])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", required=True)
    parser.add_argument("--store", required=True)
    parser.add_argument("--pairs", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    pairs = json.load(open(args.pairs))["pairs"]
    store = sqlite3.connect(f"file:{args.store}?mode=ro", uri=True)
    os.makedirs(args.out, exist_ok=True)
    targets: dict[str, list[str]] = {}
    for pair in pairs:
        targets.setdefault(pair["target"], []).append(pair["source"])

    sources = sorted({s for ss in targets.values() for s in ss})
    if set(sources) & set(targets):
        raise SystemExit("a target is also a source; the pair list is not blind")
    loaded = {}
    lens = None
    for source in sources:
        depth, header = g.read_depth(os.path.join(args.frames, f"{source}.depth.gz"))
        if lens is None:
            lens = {k: header[k] for k in ("width", "height", "lens")}
        elif {k: header[k] for k in ("width", "height", "lens")} != lens:
            raise SystemExit(f"{source} has another lens; the reference bank is not one device")
        meta = look(store, source)
        loaded[source] = (depth, header, meta)

    record = {"lens": lens, "targets": {}}
    for target, its_sources in sorted(targets.items()):
        meta = look(store, target)
        r_t, t_t = g.camera_pose(meta["pose"], meta["pan"], meta["tilt"], meta["when"])
        warped = np.full((lens["height"], lens["width"]), np.inf)
        each = {}
        for source in its_sources:
            depth, header, smeta = loaded[source]
            r_s, t_s = g.camera_pose(smeta["pose"], smeta["pan"], smeta["tilt"], smeta["when"])
            points = surface_points(depth, header, r_s, t_s)
            alone = np.full_like(warped, np.inf)
            draw(points, lens, r_t, t_t, alone)
            each[source] = alone
            np.minimum(warped, alone, out=warped)
        # The copy baseline: the nearest source's own depth map, unmoved.
        nearest = min(its_sources, key=lambda s: np.hypot(
            loaded[s][2]["pose"]["x_m"] - meta["pose"]["x_m"],
            loaded[s][2]["pose"]["y_m"] - meta["pose"]["y_m"]))
        copy = loaded[nearest][0].copy()
        copy[(copy < MIN_M) | (copy > MAX_M)] = np.inf
        path = os.path.join(args.out, f"{target}.npz")
        np.savez_compressed(path, warped=warped.astype(np.float32),
                            copy=copy.astype(np.float32),
                            **{f"alone_{s}": v.astype(np.float32) for s, v in each.items()})
        digest = hashlib.sha256(open(path, "rb").read()).hexdigest()
        record["targets"][target] = {"sources": its_sources, "copy_from": nearest,
                                     "file": os.path.basename(path), "sha256": digest,
                                     "predicted_share": float(np.isfinite(warped).mean())}
        print(target, "from", its_sources, "predicted", f"{np.isfinite(warped).mean():.0%}",
              "of the grid; copy from", nearest)
    json.dump(record, open(os.path.join(args.out, "predictions.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
