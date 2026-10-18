"""Contact sheets of records, three crops each (newest, middle, oldest), for
labelling whether each is a thing. python3 record_sheets.py IDS.json OUT_DIR"""
import json
import os
import sqlite3
import sys

from PIL import Image, ImageDraw

DB = os.path.expanduser("~/.ugv/world/world.db")
S = os.path.expanduser("~/.ugv/world/visit_model")
ids = json.load(open(sys.argv[1]))
out_dir = sys.argv[2]
os.makedirs(out_dir, exist_ok=True)
recs = {t["id"]: t for t in json.load(open(os.path.join(S, "things.json")))["things"]}
db = sqlite3.connect("file:%s?mode=ro" % DB, uri=True)
paths = {}


def crop(frame, bbox):
    if frame not in paths:
        r = db.execute("select path from frames where id = ?", (frame,)).fetchone()
        paths[frame] = r[0] if r else None
    p = paths[frame]
    if not p or not os.path.exists(p):
        return None
    im = Image.open(p).convert("RGB")
    W, H = im.size
    x0, y0, x1, y1 = json.loads(bbox)
    pad = 0.04
    c = im.crop((max(0, int((x0 - pad) * W)), max(0, int((y0 - pad) * H)),
                 min(W, int((x1 + pad) * W)), min(H, int((y1 + pad) * H))))
    c.thumbnail((128, 128))
    return c


TILE_W, TILE_H, COLS, ROWS = 3 * 130 + 10, 152, 3, 8
PER = COLS * ROWS
for s in range(0, len(ids), PER):
    sheet = Image.new("RGB", (COLS * TILE_W, ROWS * TILE_H), (255, 255, 255))
    d = ImageDraw.Draw(sheet)
    for k, rid in enumerate(ids[s:s + PER]):
        r = recs[rid]
        obs = r["observations"]
        q = "select frame_id, bbox_json from observations where id in (%s) order by observed_at" % ",".join("?" * len(obs))
        rows = db.execute(q, obs).fetchall()
        pick, seen = [], set()
        for j in (len(rows) - 1, len(rows) // 2, 0, len(rows) - 2, 1):
            if 0 <= j < len(rows) and rows[j][0] not in seen:
                seen.add(rows[j][0])
                pick.append(rows[j])
            if len(pick) == 3:
                break
        x0, y0 = (k % COLS) * TILE_W, (k // COLS) * TILE_H
        d.text((x0 + 2, y0 + 1), "%d. %s  %d visits  z %.2f  n %d" % (
            s + k, rid, r["visits"], r["height_m"] if r.get("height_m") is not None else -9, len(rows)),
            fill=(0, 0, 0))
        for j, (frame, bbox) in enumerate(pick):
            c = crop(frame, bbox)
            if c is not None:
                sheet.paste(c, (x0 + j * 130, y0 + 16))
    sheet.save(os.path.join(out_dir, "records%02d.jpg" % (s // PER)), quality=85)
print("done")
