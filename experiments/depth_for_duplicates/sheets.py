"""Contact sheets of records: each tile a whole picture, the region's outline
tinted and its box drawn, so that whatever stands in front can be seen.

    python sheets.py LOOKS.npz RUN.pkl DB OUTDIR REC[:N] ...
"""
import json
import os
import pickle
import sqlite3
import struct
import sys
import zlib

import numpy as np
from PIL import Image, ImageDraw

S = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ["VISIT_MODEL"])
import data  # noqa: E402

_HEADER = struct.Struct("<HHHHB")
TILE = (320, 240)
PER_ROW = 4


def decode(blob):
    if not blob or len(blob) <= _HEADER.size:
        return None
    x0, y0, cols, rows, stride = _HEADER.unpack_from(blob)
    bits = np.unpackbits(np.frombuffer(zlib.decompress(blob[_HEADER.size:]), dtype=np.uint8))
    if cols == 0 or rows == 0 or bits.size < cols * rows:
        return None
    return x0, y0, stride, bits[:cols * rows].reshape(rows, cols).astype(bool)


def frame_paths():
    out = {}
    for p in open(os.environ["FRAME_PATHS"]).read().split("\n"):
        if p:
            out.setdefault(os.path.basename(p)[:-4], p if os.path.isabs(p) else os.path.join("D:/UGV Rover", p))
    return out


def tile(path, bbox, blob, text):
    img = Image.open(path).convert("RGB")
    W, H = img.size
    arr = np.asarray(img).astype(np.float32)
    got = decode(blob)
    if got is not None:
        x0, y0, stride, m = got
        ys, xs = np.nonzero(m)
        for dy in range(stride):
            for dx in range(stride):
                yy, xx = y0 + ys * stride + dy, x0 + xs * stride + dx
                k = (yy < H) & (xx < W)
                arr[yy[k], xx[k]] = 0.55 * arr[yy[k], xx[k]] + 0.45 * np.array([0, 255, 80])
    img = Image.fromarray(arr.astype(np.uint8))
    d = ImageDraw.Draw(img)
    b = [bbox[0] * W, bbox[1] * H, bbox[2] * W, bbox[3] * H]
    d.rectangle(b, outline=(255, 0, 0), width=3)
    img = img.resize(TILE)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, TILE[0], 14], fill=(0, 0, 0))
    d.text((2, 1), text, fill=(255, 255, 0))
    return img


def main():
    looks, runp, db, outdir = sys.argv[1:5]
    a = data.load(looks)
    run = pickle.load(open(runp, "rb"))
    row = {int(i): k for k, i in enumerate(a["id"])}
    mem = {}
    for oid, r in run["filing"].items():
        mem.setdefault(r, []).append(oid)
    paths = frame_paths()
    con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    os.makedirs(outdir, exist_ok=True)
    for spec in sys.argv[5:]:
        if spec.startswith("@"):
            # @NAME=id,id,...: these regions, in this order.
            rec, _, listed = spec[1:].partition("=")
            ids = [int(x) for x in listed.split(",")]
            ids = [i for i in ids if a["frame"][row[i]] in paths]
            pick = ids
        else:
            rec, _, n = spec.partition(":")
            n = int(n or 8)
            ids = sorted(mem.get(rec, []), key=lambda i: a["t"][row[i]])
            ids = [i for i in ids if a["frame"][row[i]] in paths]
            if not ids:
                print(rec, "no pictures on disk")
                continue
            # The first half in time order (how the record began), the rest spread.
            first = ids[: n // 2]
            rest = ids[n // 2:]
            if rest:
                rest = [rest[int(k)] for k in np.linspace(0, len(rest) - 1, min(n - len(first), len(rest)))]
            pick = list(dict.fromkeys(first + rest))
        tiles = []
        for i in pick:
            k = row[i]
            blob = con.execute("select outline_blob from observations where id = ?", (i,)).fetchone()
            blob = blob[0] if blob else None
            rng = a["range"][k]
            text = "%d %s r%s %s" % (i, str(a["frame"][k])[4:15], "-" if not np.isfinite(rng) else "%.1f" % rng,
                                     (a["place_from"][k] or "")[:7])
            tiles.append(tile(paths[a["frame"][k]], a["bbox"][k], blob, text))
        rows = (len(tiles) + PER_ROW - 1) // PER_ROW
        sheet = Image.new("RGB", (TILE[0] * PER_ROW, TILE[1] * rows + 18), (30, 30, 30))
        ImageDraw.Draw(sheet).text((4, 2), "%s: %d regions, %d on disk" % (rec, len(mem.get(rec, [])), len(ids)),
                                   fill=(255, 255, 255))
        for j, t in enumerate(tiles):
            sheet.paste(t, ((j % PER_ROW) * TILE[0], 18 + (j // PER_ROW) * TILE[1]))
        sheet.save(os.path.join(outdir, "%s.jpg" % rec), quality=85)
        print(rec, len(tiles))


if __name__ == "__main__":
    main()
