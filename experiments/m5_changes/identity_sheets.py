"""The crops a session's staged things may be in, for labelling criterion 4
(docs/decisions/m5-sessions-are-counted-before-they-start.md).

    python identity_sheets.py RECORD.json STAGED.json WORLD.db REC.npz FRAMES OUT
        [--before-h 6] [--bad HH:MM:SS-HH:MM:SS ...] [--date YYYY-MM-DD]

RECORD is session.py's record, STAGED the session's staged file, WORLD a SQLite
backup of the world store (never the live file), REC the session's recording as
bag_extract.py writes it, FRAMES a directory holding the session's frame JPEGs.
Run from the repository root, so that the world state's lens can be imported.

A region is a candidate for a staged place when its line of sight passes within
NEAR_M of the place, between MIN_RANGE_M and MAX_RANGE_M ahead. A look the store
kept no pose for is drawn from the recording's pose at its moment, which is how
the living room's looks of 2026-10-12 come back: the heading check refused 20 of
25 of them. Looks before the staging are candidates for where a thing stood, looks
after for where it went; a look in a window given as --bad is marked, and the
counts leave it out.

Writes OUT/candidates.json, a contact sheet per place (OUT/<place>.png) and
OUT/labels-template.json for the labels identity_count.py reads.
"""
import argparse
import json
import math
import os
import sqlite3
import sys
import time

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.getcwd())
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from world_state import view  # noqa: E402
from change_detect import compose, interp_pose, last_pose  # noqa: E402

NEAR_M = 1.0
MIN_RANGE_M = 0.3
MAX_RANGE_M = 6.0
#: Codex's size floor for a crop worth labelling (shakedown_labels.json).
MIN_AREA = 0.012
MIN_HEIGHT = 0.10
CROP_PX = 120


def places(staged):
    """(name, kind, x, y, phase) for every place a staged thing stood or went."""
    out = []
    for m in staged.get("moves", []):
        out.append((m["what"], "from", m["from"][0], m["from"][1], "before"))
        out.append((m["what"], "to", m["to"][0], m["to"][1], "after"))
    for a in staged.get("added", []):
        out.append((a["what"], "added", a["at"][0], a["at"][1], "after"))
    for m in staged.get("also_moved", []):
        out.append((m["what"], "later to", m["to"][0], m["to"][1], "after"))
    return out


def clock(day, s):
    return time.mktime(day[:3] + tuple(int(v) for v in s.split(":")) + (0, 0, -1))


def main():
    p = argparse.ArgumentParser()
    for name in ("record", "staged", "world", "recording", "frames", "out"):
        p.add_argument(name)
    p.add_argument("--before-h", type=float, default=6.0)
    p.add_argument("--bad", action="append", default=[])
    a = p.parse_args()
    record = json.load(open(a.record))
    staged = json.load(open(a.staged))
    entries = record["entries"]
    start, end = entries[0]["at"], entries[-1]["at"]
    staged_at = next(e["at"] for e in entries if e["step"] == "wait" and e.get("end"))
    day = time.localtime(start)
    bad = [tuple(clock(day, s) for s in w.split("-")) for w in a.bad]
    z = np.load(a.recording)
    odom = z["odom_tf"][np.argsort(z["odom_tf"][:, 0])]
    mapt = z["map_tf"][np.argsort(z["map_tf"][:, 0])]

    db = sqlite3.connect("file:%s?mode=ro" % a.world, uri=True)
    db.row_factory = sqlite3.Row
    rows = db.execute(
        "select id, entity_id, observed_at, frame_id, bbox_json, observer_pan_deg,"
        " observer_tilt_deg, observer_pose_json, bearing_deg, note from observations"
        " where observed_at between ? and ? and frame_id is not null and bbox_json is not null",
        (start - 3600 * a.before_h, end)).fetchall()

    spots = places(staged)
    found = {i: [] for i in range(len(spots))}
    for r in rows:
        box = json.loads(r["bbox_json"])
        w, h = box[2] - box[0], box[3] - box[1]
        if w * h < MIN_AREA or h < MIN_HEIGHT or r["observer_pan_deg"] is None:
            continue
        pose = json.loads(r["observer_pose_json"]) if r["observer_pose_json"] else None
        source = "store"
        if pose is None:
            t = r["observed_at"]
            ob, mo = interp_pose(odom, t), last_pose(mapt, t)
            if ob is None or mo is None:
                continue
            x, y, th = compose(mo, ob)
            pose = {"x_m": x, "y_m": y, "heading_deg": math.degrees(th)}
            source = "recording"
        bearing = r["bearing_deg"]
        if bearing is None or source == "recording":
            got = view._from_box(box, float(r["observer_tilt_deg"] or 0.0), (640, 480), None)
            if got is None:
                continue
            bearing = pose["heading_deg"] - float(r["observer_pan_deg"]) + got[0]
        b = math.radians(bearing)
        ux, uy = math.cos(b), math.sin(b)
        phase = "before" if r["observed_at"] < staged_at else "after"
        for i, (name, kind, px, py, want) in enumerate(spots):
            if phase != want:
                continue
            dx, dy = px - pose["x_m"], py - pose["y_m"]
            ahead = dx * ux + dy * uy
            off = abs(-dx * uy + dy * ux)
            if MIN_RANGE_M <= ahead <= MAX_RANGE_M and off <= NEAR_M:
                found[i].append({
                    "region": r["id"], "record": r["entity_id"], "at": r["observed_at"],
                    "clock": time.strftime("%H:%M:%S", time.localtime(r["observed_at"])),
                    "frame": r["frame_id"], "bbox": box, "pose_from": source,
                    "off_m": round(off, 2), "ahead_m": round(ahead, 2),
                    "bad_pose": any(lo <= r["observed_at"] <= hi for lo, hi in bad),
                    "note": (r["note"] or "")[:160]})

    os.makedirs(a.out, exist_ok=True)
    out = []
    for i, (name, kind, px, py, want) in enumerate(spots):
        cands = sorted(found[i], key=lambda c: c["at"])
        slug = ("%s-%s" % (name, kind)).replace(" ", "_")
        out.append({"thing": name, "kind": kind, "at": [px, py], "phase": want,
                    "sheet": slug + ".png", "candidates": cands})
        sheet(cands, a.frames, os.path.join(a.out, slug + ".png"), "%s, %s (%.2f, %.2f)" % (name, kind, px, py))
        print("%-32s %-8s %3d candidates, %3d from the recording's pose, %3d in a bad window, records %s"
              % (name[:32], kind, len(cands), sum(c["pose_from"] == "recording" for c in cands),
                 sum(c["bad_pose"] for c in cands),
                 sorted({c["record"] or "-" for c in cands})[:8]))
    json.dump(out, open(os.path.join(a.out, "candidates.json"), "w"), indent=1)
    json.dump({"things": {s[0]: [] for s in spots}, "left_out": [],
               "note": "region ids per physical thing, from the sheets; left_out for small, mixed or unclear"},
              open(os.path.join(a.out, "labels-template.json"), "w"), indent=1)


def sheet(cands, frames, path, title):
    """Crops in time order, each captioned region, record, clock, offset."""
    cols = 8
    rows = max(1, (len(cands) + cols - 1) // cols)
    img = Image.new("RGB", (cols * CROP_PX, rows * (CROP_PX + 28) + 20), "white")
    d = ImageDraw.Draw(img)
    d.text((4, 4), title, fill=(0, 0, 0))
    for k, c in enumerate(cands):
        u, v = (k % cols) * CROP_PX, 20 + (k // cols) * (CROP_PX + 28)
        f = os.path.join(frames, c["frame"] + ".jpg")
        try:
            im = Image.open(f)
            W, H = im.size
            x0, y0, x1, y1 = c["bbox"]
            crop = im.crop((int(x0 * W), int(y0 * H), int(x1 * W), int(y1 * H)))
            crop.thumbnail((CROP_PX - 4, CROP_PX - 4))
            img.paste(crop, (u + 2, v + 2))
        except (OSError, ValueError):
            d.text((u + 4, v + 40), "no frame", fill=(200, 0, 0))
        colour = (200, 0, 0) if c["bad_pose"] else ((0, 0, 200) if c["pose_from"] == "recording" else (0, 0, 0))
        d.text((u + 2, v + CROP_PX), "%d %s" % (c["region"], (c["record"] or "-").replace("object:", "o")), fill=colour)
        d.text((u + 2, v + CROP_PX + 12), "%s %.1fm" % (c["clock"], c["off_m"]), fill=colour)
    img.save(path)


if __name__ == "__main__":
    main()
