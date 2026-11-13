"""Depth Anything V2 Small over the gimbal pictures, read under each region's
own outline.

    python mono_run.py MODEL.onnx FRAMES.txt FRAME_PATHS.txt DB OUT.npz [LIMIT]

For every perception region of every listed frame: the model's relative
disparity (larger is nearer) under the region's outline -- median, 25th and
75th percentiles, pixel count -- and the same over its box. The picture is
the stored 640 x 480 frame, shown to the model at 518 x 392 (multiples of 14)
with ImageNet normalisation, as the model's own preprocessor does.
"""
import json
import os
import sqlite3
import struct
import sys
import time
import zlib

import numpy as np
import onnxruntime as ort
from PIL import Image

_HEADER = struct.Struct("<HHHHB")
IN_W, IN_H = 518, 392
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def decode(blob):
    if not blob or len(blob) <= _HEADER.size:
        return None
    x0, y0, cols, rows, stride = _HEADER.unpack_from(blob)
    bits = np.unpackbits(np.frombuffer(zlib.decompress(blob[_HEADER.size:]), dtype=np.uint8))
    if cols == 0 or rows == 0 or bits.size < cols * rows:
        return None
    return x0, y0, stride, bits[:cols * rows].reshape(rows, cols).astype(bool)


def main():
    model, frames_txt, paths_txt, db, out = sys.argv[1:6]
    limit = int(sys.argv[6]) if len(sys.argv) > 6 else None
    frames = [f for f in open(frames_txt).read().split("\n") if f]
    part = os.environ.get("PART")  # "k/n": every n-th frame from the k-th
    if part:
        k, n = map(int, part.split("/"))
        frames = frames[k::n]
    frames = frames[:limit]
    paths = {}
    for p in open(paths_txt).read().split("\n"):
        if p:
            paths.setdefault(os.path.basename(p)[:-4], []).append(
                p if os.path.isabs(p) else os.path.join("D:/UGV Rover", p))
    want = set(frames)
    con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    regs = {}
    for oid, fr, bb, blob in con.execute("select id, frame_id, bbox_json, outline_blob from observations"
                                         " where source = 'perception' and camera = 'gimbal'"):
        if fr in want:
            regs.setdefault(fr, []).append((int(oid), json.loads(bb), blob))
    so = ort.SessionOptions()
    so.intra_op_num_threads = int(os.environ.get("THREADS", 4))
    sess = ort.InferenceSession(model, so, providers=["CPUExecutionProvider"])
    rows = []
    t0 = time.time()
    for n, fr in enumerate(frames):
        img = None
        for path in paths.get(fr, []):
            try:
                img = Image.open(path).convert("RGB")
                break
            except (OSError, ValueError):
                continue
        if img is None:
            print("unreadable", fr, file=sys.stderr)
            continue
        W, H = img.size
        x = (np.asarray(img.resize((IN_W, IN_H), Image.BICUBIC), np.float32) / 255.0 - MEAN) / STD
        d = sess.run(None, {"pixel_values": x.transpose(2, 0, 1)[None]})[0][0]
        sy, sx = d.shape[0] / H, d.shape[1] / W
        for oid, b, blob in regs.get(fr, []):
            got = decode(blob)
            o = (np.nan, np.nan, np.nan, 0)
            if got is not None:
                x0, y0, stride, m = got
                ys, xs = np.nonzero(m)
                yy = np.clip(((y0 + ys * stride) * sy).astype(int), 0, d.shape[0] - 1)
                xx = np.clip(((x0 + xs * stride) * sx).astype(int), 0, d.shape[1] - 1)
                v = d[yy, xx]
                if v.size:
                    o = (*np.percentile(v, [50, 25, 75]), v.size)
            bx = d[int(b[1] * d.shape[0]):max(int(b[1] * d.shape[0]) + 1, int(b[3] * d.shape[0])),
                   int(b[0] * d.shape[1]):max(int(b[0] * d.shape[1]) + 1, int(b[2] * d.shape[1]))]
            rows.append((oid, *o, float(np.median(bx)), float(np.percentile(bx, 90))))
        if n % 100 == 0:
            print(n, len(frames), "%.2f s a frame" % ((time.time() - t0) / (n + 1)), file=sys.stderr, flush=True)
    r = np.array(rows, dtype=np.float64)
    np.savez(out, id=r[:, 0].astype(np.int64), d50=r[:, 1], d25=r[:, 2], d75=r[:, 3], npx=r[:, 4],
             box50=r[:, 5], box90=r[:, 6])
    print(len(rows), "regions from", len(frames), "frames in %.0f s" % (time.time() - t0))


if __name__ == "__main__":
    main()
