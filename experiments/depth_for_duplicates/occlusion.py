"""How much of each region something else in the same picture covers, from the
region finder's own outlines: no depth model, no pictures.

    python occlusion.py LOOKS.npz RUN.pkl DUPS.json DB OUT.npz

For every region filed to a never-moved object's main record or one of its
duplicates (dups.json), and every labelled region of those objects:

- `cover`: share of the region's box under other regions' outlines -- what
  stands in front of it, for things on walls and floors;
- `bleed`: share of the region's own outline under other regions' outlines --
  the occluder inside the pixels the vector and the range are read from;

leaving out regions whose box holds most of this one's and is at least twice
its size (a region of the cabinet with the painting on it), and regions
ranged more than BEHIND_M further than this one.
"""
import json
import os
import pickle
import sqlite3
import struct
import sys
import zlib
from collections import defaultdict

import numpy as np

S = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ["VISIT_MODEL"])
import data  # noqa: E402

_HEADER = struct.Struct("<HHHHB")
W, H = 640, 480
BEHIND_M = 0.3


def decode(blob):
    if not blob or len(blob) <= _HEADER.size:
        return None
    x0, y0, cols, rows, stride = _HEADER.unpack_from(blob)
    bits = np.unpackbits(np.frombuffer(zlib.decompress(blob[_HEADER.size:]), dtype=np.uint8))
    if cols == 0 or rows == 0 or bits.size < cols * rows:
        return None
    return x0, y0, stride, bits[:cols * rows].reshape(rows, cols).astype(bool)


def raster(blob):
    """The outline on a half-resolution frame grid (320 x 240)."""
    out = np.zeros((H // 2, W // 2), bool)
    got = decode(blob)
    if got is None:
        return out
    x0, y0, stride, m = got
    ys, xs = np.nonzero(m)
    yy, xx = (y0 + ys * stride) // 2, (x0 + xs * stride) // 2
    k = (yy < H // 2) & (xx < W // 2)
    out[yy[k], xx[k]] = True
    return out


def box_mask(b):
    out = np.zeros((H // 2, W // 2), bool)
    x0, y0, x1, y1 = [int(round(v)) for v in (b[0] * W / 2, b[1] * H / 2, b[2] * W / 2, b[3] * H / 2)]
    out[max(0, y0):max(0, y1), max(0, x0):max(0, x1)] = True
    return out


def area(b):
    return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])


def inter(b, c):
    return area([max(b[0], c[0]), max(b[1], c[1]), min(b[2], c[2]), min(b[3], c[3])])


def main():
    looks, runp, dupsp, db, outp = sys.argv[1:6]
    a = data.load(looks)
    run = pickle.load(open(runp, "rb"))
    dups = json.load(open(dupsp))
    row = {int(i): k for k, i in enumerate(a["id"])}
    filing = run["filing"]
    mem = defaultdict(list)
    for oid, r in filing.items():
        mem[r].append(oid)
    stay = json.load(open(os.path.join(os.environ["VISIT_MODEL"], "labels",
                                       "stayed-put.json")))["objects"]
    want = {}  # region id -> (object, role)
    for name, v in dups.items():
        for i in mem[v["main"]]:
            want[i] = (name, "main")
        for o in v["others"]:
            for i in mem[o["record"]]:
                want.setdefault(i, (name, "dup"))
        for s, ids in stay[name].items():
            if isinstance(ids, list):
                for i in ids:
                    if int(i) in row and int(i) not in want:
                        want[int(i)] = (name, "unfiled" if int(i) not in filing else "other")
    con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    frames = defaultdict(list)
    for i in want:
        frames[a["frame"][row[i]]].append(i)
    # One pass over the table: it has no index on frame_id.
    by_frame = defaultdict(list)
    for oid, fr, bb, rng, blob in con.execute(
            "select id, frame_id, bbox_json, range_m, outline_blob from observations"
            " where source = 'perception' and camera = 'gimbal'"):
        if fr in frames:
            by_frame[fr].append((int(oid), json.loads(bb), rng, blob))
    res = {}
    for n, (frame, ids) in enumerate(frames.items()):
        regs = by_frame.get(frame, [])
        rasters = {}
        for i in ids:
            me = next((r for r in regs if r[0] == i), None)
            if me is None:
                continue
            _, b, rng, blob = me
            mine = raster(blob)
            bm = box_mask(b)
            cover = np.zeros_like(bm)
            for oid, c, crng, cblob in regs:
                if oid == i:
                    continue
                if inter(b, c) >= 0.8 * area(b) and area(c) >= 2 * area(b):
                    continue  # holds this one: a region of what it hangs on
                if inter(b, c) <= 0:
                    continue
                if rng is not None and crng is not None and crng > rng + BEHIND_M:
                    continue
                if oid not in rasters:
                    rasters[oid] = raster(cblob)
                cover |= rasters[oid]
            nb, nm = bm.sum(), mine.sum()
            res[i] = (float((cover & bm).sum() / nb) if nb else np.nan,
                      float((cover & mine).sum() / nm) if nm else np.nan)
        if n % 500 == 0:
            print(n, len(frames), file=sys.stderr)
    ids = np.array(sorted(res))
    np.savez(outp, id=ids, cover=np.array([res[i][0] for i in ids]), bleed=np.array([res[i][1] for i in ids]),
             obj=np.array([want[i][0] for i in ids]), role=np.array([want[i][1] for i in ids]),
             record=np.array([str(filing.get(int(i))) for i in ids]))
    print(len(ids), "regions measured")


if __name__ == "__main__":
    main()
