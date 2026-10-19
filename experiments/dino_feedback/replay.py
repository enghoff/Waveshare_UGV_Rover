#!/usr/bin/env python3
"""Re-run DINOv2 over stored looks and keep what the rover throws away.

    python experiments/dino_feedback/replay.py --out <cache.npz>

For each look named by the per-look identity labels this crops the stored frame
exactly as `world_state/perceive.py` does, rebuilds the region's mask from the
outline the rover stored, and runs the fp16 DINOv2 export the rover's TensorRT
engine is built from. It keeps the full token grid -- the summary token the rover
stores and the 256 patch tokens it discards -- with each patch cell's share of
mask, so that `settle.py` can be run over them as often as wanted.

It also runs the masked crop through DINOv2 twice: once with the rover's mask,
to show the replay reproduces the stored `dino_alone`, and once with the mask
`settle.settle` returns, which is the candidate that feeds the agreed mask back
into the encoder.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sqlite3
import sys

import cv2
import numpy as np
import onnxruntime as ort

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import settle  # noqa: E402

ROOT = r"D:\UGV Rover"
MANIFEST = os.path.join(ROOT, "experiments", "perception_bench", "manifest.json")
DB = os.path.join(ROOT, "captures", "2026-10-03-merge-review", "world_now.db")
YOLOE = os.path.join(ROOT, "captures", "2026-10-05-depth-abstention",
                     "yoloe-11s-seg-objectness.onnx")
DINO = os.path.join(ROOT, ".cache", "entity-reader-groups", ".cache",
                    "appearance-probe", "dinov2-small-fp16.onnx")

# perceive.py's own numbers.
CROP_PAD = 0.02
LETTERBOX_GREY = 114
DINO_SIZE = 224
PATCH = 14
GRID = DINO_SIZE // PATCH
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def frame_index():
    found = {}
    for pattern in (os.path.join(ROOT, "tmp", "codex-m5-audit", "frames", "*.jpg"),
                    os.path.join(ROOT, "captures", "**", "*.jpg"),
                    os.path.join(ROOT, "tmp", "**", "*.jpg")):
        for path in glob.glob(pattern, recursive=True):
            found.setdefault(os.path.splitext(os.path.basename(path))[0], path)
    return found


def window(shape, box):
    height, width = shape[:2]
    return (int(max(0.0, box[0] - CROP_PAD) * width),
            int(max(0.0, box[1] - CROP_PAD) * height),
            int(min(1.0, box[2] + CROP_PAD) * width),
            int(min(1.0, box[3] + CROP_PAD) * height))


def letterbox(image, size, fill, interpolation=cv2.INTER_LINEAR):
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    new_w, new_h = int(round(width * scale)), int(round(height * scale))
    resized = cv2.resize(image, (new_w, new_h), interpolation=interpolation)
    canvas = np.full((size, size) + image.shape[2:], fill, dtype=image.dtype)
    top, left = (size - new_h) // 2, (size - new_w) // 2
    canvas[top:top + new_h, left:left + new_w] = resized
    return canvas


YOLOE_SIZE, YOLOE_CONF, YOLOE_OVERLAP, MASK_INSIDE = 512, 0.15, 0.8, 0.5


def suppress(boxes, scores, threshold):
    """perceive._suppress: greedy, overlap over the smaller box."""
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        best = order[0]
        keep.append(best)
        rest = order[1:]
        if not rest.size:
            break
        x1 = np.maximum(boxes[best, 0], boxes[rest, 0])
        y1 = np.maximum(boxes[best, 1], boxes[rest, 1])
        x2 = np.minimum(boxes[best, 2], boxes[rest, 2])
        y2 = np.minimum(boxes[best, 3], boxes[rest, 3])
        overlap = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
        area = lambda b: (b[..., 2] - b[..., 0]) * (b[..., 3] - b[..., 1])
        smaller = np.minimum(area(boxes[best]), area(boxes[rest]))
        order = rest[overlap / np.maximum(smaller, 1e-9) <= threshold]
    return keep


def regions(session, image):
    """perceive._regions and _Masks.of: boxes as fractions and frame-sized masks."""
    height, width = image.shape[:2]
    scale = min(YOLOE_SIZE / width, YOLOE_SIZE / height)
    new_w, new_h = int(round(width * scale)), int(round(height * scale))
    top, left = (YOLOE_SIZE - new_h) // 2, (YOLOE_SIZE - new_w) // 2
    canvas = letterbox(image, YOLOE_SIZE, LETTERBOX_GREY)
    blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    raw, protos = session.run(None, {"images": blob})
    rows = raw[0].T
    rows = rows[rows[:, 4] >= YOLOE_CONF]
    if not len(rows):
        return np.zeros((0, 4)), []
    cx, cy, w, h = rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3]
    boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1)
    keep = suppress(boxes, rows[:, 4], YOLOE_OVERLAP)
    boxes, coeffs = boxes[keep], rows[keep, 5:37]
    flat = protos[0].reshape(protos.shape[1], -1)
    masks = []
    for c in coeffs:
        m = 1.0 / (1.0 + np.exp(-(c @ flat)))
        m = cv2.resize(m.reshape(protos.shape[2:]), (YOLOE_SIZE, YOLOE_SIZE),
                       interpolation=cv2.INTER_LINEAR) > MASK_INSIDE
        inner = m[top:top + int(round(height * scale)), left:left + int(round(width * scale))]
        masks.append(cv2.resize(inner.astype(np.uint8), (width, height),
                                interpolation=cv2.INTER_NEAREST) > 0)
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - left) / scale / width
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - top) / scale / height
    return np.clip(boxes, 0.0, 1.0), masks


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / max(union, 1e-9)


def outline_mask(blob, shape):
    """The frame-sized mask `outline.encode` stored, or None."""
    import struct
    import zlib
    header = struct.Struct("<HHHHB")
    if not blob or len(blob) <= header.size:
        return None
    x0, y0, columns, rows, stride = header.unpack_from(blob)
    bits = np.unpackbits(np.frombuffer(zlib.decompress(blob[header.size:]), dtype=np.uint8))
    piece = bits[:columns * rows].reshape(rows, columns).astype(bool)
    big = np.kron(piece, np.ones((stride, stride), dtype=bool))
    mask = np.zeros(shape[:2], dtype=bool)
    h = min(big.shape[0], shape[0] - y0)
    w = min(big.shape[1], shape[1] - x0)
    mask[y0:y0 + h, x0:x0 + w] = big[:h, :w]
    return mask


def cells(mask_crop):
    """Each patch cell's share of mask and of real picture, on the 16x16 grid."""
    share = letterbox(mask_crop.astype(np.float32), DINO_SIZE, 0.0, cv2.INTER_AREA)
    real = letterbox(np.ones(mask_crop.shape, np.float32), DINO_SIZE, 0.0, cv2.INTER_AREA)
    pool = lambda a: a.reshape(GRID, PATCH, GRID, PATCH).mean(axis=(1, 3)).ravel()
    return pool(share), pool(real)


def pixels(patch):
    square = letterbox(patch, DINO_SIZE, LETTERBOX_GREY)
    return ((square[:, :, ::-1].astype(np.float32) / 255.0 - MEAN) / STD).transpose(2, 0, 1)


def labelled_ids():
    manifest = json.load(open(MANIFEST))
    ids = []
    for item in manifest["items"]:
        if item["kind"] == "per_look_identity":
            truth = item["ground_truth"]
            ids += list(truth["main_looks"]) + list(truth["odd"]) + list(truth["unsure"])
    return sorted(set(ids))


def unit(v):
    return v / max(1e-9, float(np.linalg.norm(v)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--temperature", type=float, default=settle.TEMPERATURE)
    args = parser.parse_args()

    frames = frame_index()
    db = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
    ids = labelled_ids()
    rows = db.execute(
        f"select id, entity_id, frame_id, bbox_json, dino_blob, dino_alone_blob, "
        f"outline_blob, mask_share from observations where id in ({','.join('?' * len(ids))})",
        ids).fetchall()
    session = ort.InferenceSession(DINO, providers=["CPUExecutionProvider"])
    yoloe = ort.InferenceSession(YOLOE, providers=["CPUExecutionProvider"])
    found_regions = {}

    out = {k: [] for k in ("id", "entity", "frame", "tokens", "share", "real",
                           "stored_dino", "stored_alone", "stored_mask_share",
                           "replay_alone", "replay_mask_share", "settled_alone",
                           "settled_r", "box_match")}
    missing = []
    for number, (oid, entity, frame_id, bbox_json, dino, alone, blob, mask_share) in enumerate(rows):
        path = frames.get(frame_id)
        image = None if path is None else cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            missing.append(oid)
            continue
        box = json.loads(bbox_json)
        x0, y0, x1, y1 = window(image.shape, box)
        patch = image[y0:y1, x0:x1]
        # The region model again, for the whole mask: the stored outline is cut to
        # the box, and the rover blanked its masked crop over the padded window.
        if frame_id not in found_regions:
            found_regions[frame_id] = regions(yoloe, image)
        boxes, masks = found_regions[frame_id]
        overlaps = [iou(box, b) for b in boxes]
        best = int(np.argmax(overlaps)) if overlaps else -1
        matched = overlaps[best] if overlaps else 0.0
        mask = masks[best] if matched >= 0.9 else outline_mask(blob, image.shape)
        piece = (np.zeros(patch.shape[:2], bool) if mask is None else mask[y0:y1, x0:x1])
        masked = patch.copy()
        masked[~piece] = LETTERBOX_GREY
        share, real = cells(piece)

        tokens = session.run(None, {"pixel_values": np.stack([pixels(patch), pixels(masked)])})[0]
        result = settle.settle(tokens[0, 1:], share, real, temperature=args.temperature)
        # The agreed mask back at pixel resolution: the rover's mask where the
        # settled cells keep it. Only trims -- see settle.refined_pixels.
        refined = settle.refined_pixels(piece, result.r, real)
        again = patch.copy()
        again[~refined] = LETTERBOX_GREY
        settled_cls = session.run(None, {"pixel_values": pixels(again)[None]})[0][0, 0]

        out["id"].append(oid)
        out["entity"].append(entity)
        out["frame"].append(frame_id)
        out["tokens"].append(tokens[0].astype(np.float16))
        out["share"].append(share.astype(np.float32))
        out["real"].append(real.astype(np.float32))
        out["stored_dino"].append(np.frombuffer(dino, np.float32))
        out["stored_alone"].append(np.frombuffer(alone, np.float32) if alone else np.zeros(384, np.float32))
        out["stored_mask_share"].append(-1.0 if mask_share is None else float(mask_share))
        out["replay_alone"].append(tokens[1, 0].astype(np.float32))
        out["replay_mask_share"].append(float(piece.mean()) if piece.size else -1.0)
        out["settled_alone"].append(settled_cls.astype(np.float32))
        out["settled_r"].append(result.r.astype(np.float32))
        out["box_match"].append(float(matched))
        if number % 50 == 0:
            print(f"{number}/{len(rows)}", flush=True)

    arrays = {k: np.array(v) for k, v in out.items()}
    np.savez_compressed(args.out, **arrays)
    plain = np.array([unit(t[0].astype(np.float32)) for t in arrays["tokens"]])
    stored = np.array([unit(v) for v in arrays["stored_dino"]])
    alone_r = np.array([unit(v) for v in arrays["replay_alone"]])
    alone_s = np.array([unit(v) for v in arrays["stored_alone"]])
    cos_plain = (plain * stored).sum(1)
    cos_alone = (alone_r * alone_s).sum(1)
    print(f"looks {len(arrays['id'])}, frames missing {len(missing)}")
    bm = arrays["box_match"]
    print("region boxes regenerated: IoU median %.4f, below 0.9: %d" % (np.median(bm), int((bm < 0.9).sum())))
    print("replay vs stored, summary token: median %.4f, 5th pct %.4f, min %.4f"
          % (np.median(cos_plain), np.percentile(cos_plain, 5), cos_plain.min()))
    print("replay vs stored, masked token:  median %.4f, 5th pct %.4f, min %.4f"
          % (np.median(cos_alone), np.percentile(cos_alone, 5), cos_alone.min()))
    diff = np.abs(arrays["replay_mask_share"] - arrays["stored_mask_share"])
    print("mask share |replay - stored|: median %.3f, 95th pct %.3f" % (np.median(diff), np.percentile(diff, 95)))


if __name__ == "__main__":
    main()
