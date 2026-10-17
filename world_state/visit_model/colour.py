"""Each region's colour, inside its own outline: run on the rover.

    python3 colour.py OUT.npz [MAP_SESSION]

Reads the store read-only (~/.ugv/world/world.db): for every perception region
of the map session, the frame it was cut from and the outline the region
finder drew (world_state/outline.py's format), and writes a colour histogram
per region. Pixels too grey to have a hue (saturation under GREY_S) are
counted by brightness in GREY_BINS bins; the rest by hue in HUE_BINS and two
bands of saturation. The histogram is normalised and square-rooted, so the dot
product of two is the Bhattacharyya coefficient: 1 for the same colours.

The masked DINOv2 vector the store keeps is mostly shape: it scored the red
and the black bean bag 0.74 and 0.73 against one arrival in the test of
2026-10-15. Colour is the other half of what a thing looks like.
"""
import colorsys  # noqa: F401  (documentation of the convention; numpy does the work)
import json
import os
import sqlite3
import struct
import sys
import zlib

import numpy as np
from PIL import Image

HUE_BINS = 12
SAT_SPLIT = 0.55
GREY_S = 0.22
GREY_BINS = 4
WIDTH = HUE_BINS * 2 + GREY_BINS
_HEADER = struct.Struct("<HHHHB")


def decode(blob):
    """(x0, y0, stride, mask) of world_state/outline.py's encoding, or None."""
    if not blob or len(blob) <= _HEADER.size:
        return None
    x0, y0, cols, rows, stride = _HEADER.unpack_from(blob)
    bits = np.unpackbits(np.frombuffer(zlib.decompress(blob[_HEADER.size:]), dtype=np.uint8))
    if cols == 0 or rows == 0 or bits.size < cols * rows:
        return None
    return x0, y0, stride, bits[:cols * rows].reshape(rows, cols).astype(bool)


def hsv(rgb):
    """rgb (n, 3) in 0..1 to hue 0..1, saturation, value."""
    mx, mn = rgb.max(axis=1), rgb.min(axis=1)
    d = mx - mn
    s = np.where(mx > 0, d / np.maximum(mx, 1e-9), 0.0)
    r, g, b = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    h = np.zeros(len(rgb))
    nz = d > 1e-9
    rc = np.where(nz, (mx - r) / np.maximum(d, 1e-9), 0)
    gc = np.where(nz, (mx - g) / np.maximum(d, 1e-9), 0)
    bc = np.where(nz, (mx - b) / np.maximum(d, 1e-9), 0)
    h = np.where(r == mx, bc - gc, np.where(g == mx, 2.0 + rc - bc, 4.0 + gc - rc))
    h = (h / 6.0) % 1.0
    return h, s, mx


def histogram(rgb):
    h, s, v = hsv(rgb)
    out = np.zeros(WIDTH)
    grey = s < GREY_S
    gi = np.minimum((v[grey] * GREY_BINS).astype(int), GREY_BINS - 1)
    np.add.at(out, HUE_BINS * 2 + gi, 1.0)
    hi = np.minimum((h[~grey] * HUE_BINS).astype(int), HUE_BINS - 1)
    si = (s[~grey] >= SAT_SPLIT).astype(int)
    np.add.at(out, hi * 2 + si, 1.0)
    total = out.sum()
    return np.sqrt(out / total) if total > 0 else out


def colours(db_path, session=67, after_id=None):
    """(ids, histograms) of the session's regions, or with `after_id` of those
    the store has added since; each region is coloured on its own."""
    db = sqlite3.connect("file:" + db_path + "?mode=ro", uri=True)
    rows = db.execute("select o.id, o.frame_id, o.bbox_json, o.outline_blob, f.path from observations o"
                      " join frames f on f.id = o.frame_id where o.source = 'perception'"
                      " and o.map_session = ? and o.id > ? order by o.frame_id",
                      (int(session), int(after_id or 0))).fetchall()
    db.close()
    ids, vecs = [], []
    current, img = None, None
    for oid, frame, bbox, blob, path in rows:
        if frame != current:
            current = frame
            try:
                img = np.asarray(Image.open(path).convert("RGB"), dtype=np.float32) / 255.0
            except (OSError, ValueError):
                img = None
        if img is None:
            continue
        H, W = img.shape[:2]
        got = decode(blob)
        if got is not None:
            x0, y0, stride, mask = got
            ys, xs = np.nonzero(mask)
            ys, xs = y0 + ys * stride, x0 + xs * stride
            keep = (ys < H) & (xs < W)
            pix = img[ys[keep], xs[keep]]
        else:
            x0, y0, x1, y1 = json.loads(bbox)
            pix = img[int(y0 * H):max(int(y1 * H), int(y0 * H) + 1),
                      int(x0 * W):max(int(x1 * W), int(x0 * W) + 1)].reshape(-1, 3)
        if len(pix) < 20:
            continue
        ids.append(oid)
        vecs.append(histogram(pix))
    return np.array(ids, dtype=np.int64), np.array(vecs, dtype=np.float32).reshape(-1, WIDTH), len(rows)


def main(out_path, session=67):
    ids, vecs, n = colours(os.path.expanduser("~/.ugv/world/world.db"), session)
    np.savez_compressed(out_path, id=ids, colour=vecs)
    print(len(ids), "regions coloured of", n)


if __name__ == "__main__":
    main(*sys.argv[1:3])
