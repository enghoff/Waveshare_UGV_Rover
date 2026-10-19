#!/usr/bin/env python3
"""Draw the mask and the settled assignment over their crops, to see what the loop does.

    python experiments/dino_feedback/draw.py <cache.npz> <out.jpg> [look id ...]

Each row is one look: the crop letterboxed as DINOv2 sees it, the rover's mask
share per patch cell in red, and the settled assignment in green. With no look
ids it draws the looks the loop moved most, and a few it moved least.
"""
from __future__ import annotations

import json
import os
import sqlite3
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import settle  # noqa: E402
from replay import DB, DINO_SIZE, GRID, LETTERBOX_GREY, frame_index, letterbox, window  # noqa: E402
from score import labels, load  # noqa: E402

SIDE = 160


def cell_overlay(square, values, colour):
    grid = cv2.resize(values.reshape(GRID, GRID).astype(np.float32), (DINO_SIZE, DINO_SIZE),
                      interpolation=cv2.INTER_NEAREST)
    tint = np.zeros_like(square)
    tint[:] = colour
    alpha = (0.6 * grid)[..., None]
    return (square * (1 - alpha) + tint * alpha).astype(np.uint8)


def main():
    cache, out = sys.argv[1], sys.argv[2]
    wanted = [int(x) for x in sys.argv[3:]]
    looks = load(cache)
    records = labels()
    stray = {o for _, (_, odd) in records.items() for o in odd}
    record_of = {o: r for r, (main, odd) in records.items() for o in main + odd}
    moved = {}
    for oid, look in looks.items():
        got = settle.settle(look["tokens"].astype(np.float32)[1:], look["share"], look["real"])
        look["r"] = got.r
        valid = look["real"] >= settle.REAL
        m, r = look["share"][valid], got.r[valid]
        moved[oid] = 1.0 - float(np.minimum(m, r).sum() / max(np.maximum(m, r).sum(), 1e-9))
    if not wanted:
        ranked = sorted(moved, key=moved.get)
        wanted = ranked[::-1][:14] + ranked[:4]
    frames = frame_index()
    db = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
    rows = []
    for oid in wanted:
        frame_id, bbox = db.execute("select frame_id, bbox_json from observations where id=?",
                                    (oid,)).fetchone()
        image = cv2.imread(frames[frame_id])
        x0, y0, x1, y1 = window(image.shape, json.loads(bbox))
        square = letterbox(image[y0:y1, x0:x1], DINO_SIZE, LETTERBOX_GREY)
        look = looks[oid]
        tiles = [square, cell_overlay(square, look["share"], (0, 0, 255)),
                 cell_overlay(square, look["r"], (0, 200, 0))]
        tiles = [cv2.resize(t, (SIDE, SIDE)) for t in tiles]
        label = np.full((SIDE, 230, 3), 255, np.uint8)
        kind = "STRAY" if oid in stray else "main"
        for k, text in enumerate([f"{oid} {kind}", record_of.get(oid, ""),
                                  f"moved {moved[oid]:.2f}"]):
            cv2.putText(label, text, (4, 30 + 26 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        rows.append(np.hstack([label] + tiles))
    cv2.imwrite(out, np.vstack(rows))
    print(out, len(rows))


if __name__ == "__main__":
    main()
