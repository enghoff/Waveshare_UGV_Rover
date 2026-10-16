"""Contact sheets for labelling which regions show an object.

    python sheets.py candidates LOOKS.npz SPEC.json OUT.json     (here)
    python3 sheets.py draw OUT.json DIR                          (on the rover)

SPEC is {object: {"at": [x, y], "near_m": m, "seeds": [region ids],
"windows": [[t0, t1], ...]}}. A region in a window is a candidate when its
line of sight passes within `near_m` of the place, between 0.3 and 6 m
ahead; when the region has a measured place, that place is within
near_m + 0.6 m of it; and it looks somewhat like one of the seeds (masked
DINOv2 cosine LOOSE_COS or more). The seeds are pictures already known to show
the object; the floor is loose so that the labels do not just repeat what the
encoder already finds alike. Candidates never come from the store's filing.

`draw` reads the frames from the rover's store and writes one sheet per object
and session, each crop headed by its region id, time and cosine; the labeller
lists the ids that do NOT show the object.
"""
import json
import math
import os
import sys
import time

import numpy as np

LOOSE_COS = float(os.environ.get("LOOSE_COS", 0.45))
MIN_AHEAD_M, MAX_AHEAD_M = 0.3, 6.0
MIN_AREA = 0.003


def candidates(looks_path, spec_path, out_path):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import data
    import truth
    a = data.load(looks_path)
    spec = json.load(open(spec_path))
    index = {int(i): k for k, i in enumerate(a["id"])}
    ss = truth.sessions()
    posed = np.array([p is not None for p in a["pose_from"]])
    area = (a["bbox"][:, 2] - a["bbox"][:, 0]) * (a["bbox"][:, 3] - a["bbox"][:, 1])
    out = {}
    for obj, s in spec.items():
        x, y = s["at"]
        near = s.get("near_m", 0.6)
        seeds = np.stack([a["vec"][index[r]] for r in s["seeds"] if r in index])
        b = np.radians(a["bearing"])
        dx, dy = x - a["cx"], y - a["cy"]
        ahead = dx * np.cos(b) + dy * np.sin(b)
        across = np.abs(-dx * np.sin(b) + dy * np.cos(b))
        ok = posed & (ahead >= MIN_AHEAD_M) & (ahead <= MAX_AHEAD_M) & (across <= near) \
            & (area >= MIN_AREA)
        measured = np.isfinite(a["px"])
        ok &= ~measured | (np.hypot(a["px"] - x, a["py"] - y) <= near + 0.6)
        inside = np.zeros(len(ok), bool)
        for t0, t1 in s["windows"]:
            inside |= (a["t"] >= t0) & (a["t"] <= t1)
        ok &= inside
        rows = np.flatnonzero(ok)
        cos = (a["vec"][rows] @ seeds.T).max(axis=1)
        keep = rows[cos >= LOOSE_COS]
        cos = cos[cos >= LOOSE_COS]
        out[obj] = [{"id": int(a["id"][i]), "frame": str(a["frame"][i]), "t": float(a["t"][i]),
                     "bbox": [float(v) for v in a["bbox"][i]], "cos": round(float(c), 2),
                     "session": truth.session_of(float(a["t"][i]), ss)}
                    for i, c in zip(keep, cos)]
        print("%-28s %4d candidates" % (obj, len(out[obj])))
    json.dump(out, open(out_path, "w"))


def draw(cands_path, out_dir):
    import sqlite3
    from PIL import Image, ImageDraw
    db = sqlite3.connect("file:" + os.path.expanduser("~/.ugv/world/world.db") + "?mode=ro", uri=True)
    cands = json.load(open(cands_path))
    os.makedirs(out_dir, exist_ok=True)
    W, H, COLS, ROWS = 200, 170, 8, 6
    for obj, rows in cands.items():
        by_session = {}
        for c in rows:
            by_session.setdefault(c["session"] or "other", []).append(c)
        for sess, cs in by_session.items():
            cs.sort(key=lambda c: c["t"])
            for page in range(0, len(cs), COLS * ROWS):
                part = cs[page:page + COLS * ROWS]
                sheet = Image.new("RGB", (COLS * W, ((len(part) + COLS - 1) // COLS) * H), "white")
                for k, c in enumerate(part):
                    row = db.execute("select path from frames where id = ?", (c["frame"],)).fetchone()
                    if not row or not os.path.exists(row[0]):
                        continue
                    im = Image.open(row[0]).convert("RGB")
                    w, h = im.size
                    x0, y0, x1, y1 = c["bbox"]
                    px, py = 0.35 * (x1 - x0), 0.35 * (y1 - y0)
                    box = (int(max(0, (x0 - px) * w)), int(max(0, (y0 - py) * h)),
                           int(min(w, (x1 + px) * w)), int(min(h, (y1 + py) * h)))
                    crop = im.crop(box)
                    d = ImageDraw.Draw(crop)
                    d.rectangle((int(x0 * w) - box[0], int(y0 * h) - box[1],
                                 int(x1 * w) - box[0], int(y1 * h) - box[1]), outline=(255, 0, 0), width=3)
                    crop.thumbnail((W - 4, H - 22))
                    cell = Image.new("RGB", (W, H), "white")
                    cell.paste(crop, (2, 20))
                    ImageDraw.Draw(cell).text((3, 3), "%d %s %.2f" % (
                        c["id"], time.strftime("%H:%M:%S", time.localtime(c["t"])), c["cos"]),
                        fill=(0, 0, 0))
                    sheet.paste(cell, ((k % COLS) * W, (k // COLS) * H))
                name = "%s--%s--%d.jpg" % (obj.replace(" ", "_"), sess.replace(" ", ""), page // (COLS * ROWS))
                sheet.save(os.path.join(out_dir, name), quality=85)
                print(name, len(part))


if __name__ == "__main__":
    if sys.argv[1] == "candidates":
        candidates(*sys.argv[2:5])
    else:
        draw(*sys.argv[2:4])
