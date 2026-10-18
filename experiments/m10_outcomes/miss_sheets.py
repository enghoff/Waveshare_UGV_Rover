"""Contact sheets of the trial's missed looks: the trial picture (centre cross
= where it was aimed; boxes: green filed to the target, red pointing at it but
filed elsewhere, yellow pointing and unfiled, grey the rest) beside the
target record's three newest earlier crops and the taker's two.

    python3 miss_sheets.py POINTING.json OUT_DIR     (on the rover; pointing.py's output)
"""
import json
import os
import sqlite3
import sys

from PIL import Image, ImageDraw

DB = os.path.expanduser("~/.ugv/world/world.db")
S = os.path.expanduser("~/.ugv/world/visit_model")
OUT = sys.argv[2]
os.makedirs(OUT, exist_ok=True)
things = json.load(open(os.path.join(S, "things.json")))
recs = {t["id"]: t for t in things["things"]}
owner = {o: t["id"] for t in things["things"] for o in t["observations"]}
pts = json.load(open(sys.argv[1]))
db = sqlite3.connect("file:%s?mode=ro" % DB, uri=True)


def path_of(frame):
    r = db.execute("select path from frames where id = ?", (frame,)).fetchone()
    return r[0] if r else None


def crops(rid, exclude, n):
    t = recs.get(rid)
    if not t:
        return []
    ids = t["observations"]
    q = "select frame_id, bbox_json, observed_at from observations where id in (%s) order by observed_at desc" % \
        ",".join("?" * len(ids))
    out = []
    for frame, bbox, at in db.execute(q, ids):
        if frame in exclude:
            continue
        p = path_of(frame)
        if not p or not os.path.exists(p):
            continue
        im = Image.open(p).convert("RGB")
        W, H = im.size
        x0, y0, x1, y1 = json.loads(bbox)
        pad = 0.05
        box = (max(0, int((x0 - pad) * W)), max(0, int((y0 - pad) * H)),
               min(W, int((x1 + pad) * W)), min(H, int((y1 + pad) * H)))
        c = im.crop(box)
        c.thumbnail((150, 150))
        out.append(c)
        exclude = exclude | {frame}
        if len(out) >= n:
            break
    return out


rows = []
for l in pts:
    if l.get("landed"):
        continue
    p = path_of(l["frame"])
    if not p or not os.path.exists(p):
        continue
    im = Image.open(p).convert("RGB")
    W, H = im.size
    d = ImageDraw.Draw(im)
    pointing = {r["region"]: r for r in l.get("pointing", [])}
    taker = None
    for oid, bbox in db.execute("select id, bbox_json from observations where frame_id = ?", (l["frame"],)):
        x0, y0, x1, y1 = json.loads(bbox)
        r = pointing.get(oid)
        if r is None:
            col, w = (150, 150, 150), 1
        elif r["to"] == l["record"]:
            col, w = (0, 220, 0), 3
        elif r["to"] is None:
            col, w = (255, 220, 0), 3
        else:
            col, w = (255, 0, 0), 3
            if taker is None or r["cos_target"] > taker[1]:
                taker = (r["to"], r["cos_target"])
        d.rectangle([x0 * W, y0 * H, x1 * W, y1 * H], outline=col, width=w)
    d.line([W / 2 - 15, H / 2, W / 2 + 15, H / 2], fill=(0, 255, 255), width=2)
    d.line([W / 2, H / 2 - 15, W / 2, H / 2 + 15], fill=(0, 255, 255), width=2)
    im.thumbnail((360, 270))
    tgt = crops(l["record"], {l["frame"]}, 3)
    tk = crops(taker[0], {l["frame"]}, 2) if taker else []
    rows.append((l, im, tgt, tk, taker))

PER = 6
for s in range(0, len(rows), PER):
    part = rows[s:s + PER]
    sheet = Image.new("RGB", (360 + 5 * 155 + 20, len(part) * 300), (255, 255, 255))
    d = ImageDraw.Draw(sheet)
    for k, (l, im, tgt, tk, taker) in enumerate(part):
        y = k * 300
        d.text((4, y + 2), "#%d %s  target %s (%s visits, z %s, seen %.0f h ago)  %s%s" % (
            s + k, l.get("stop"), l["record"], l.get("target_visits"), l.get("target_z"), l.get("since_h") or -1,
            l["class"], "  taker %s" % taker[0] if taker else ""), fill=(0, 0, 0))
        sheet.paste(im, (0, y + 18))
        x = 370
        for c in tgt:
            sheet.paste(c, (x, y + 30))
            x += 155
        d.text((370, y + 185), "^ target %s, earlier" % l["record"], fill=(0, 0, 0))
        x = 370 + 3 * 155 + 10
        for c in tk:
            sheet.paste(c, (x, y + 30))
            x += 155
        if tk:
            d.text((370 + 3 * 155 + 10, y + 185), "^ taker %s" % taker[0], fill=(200, 0, 0))
    sheet.save(os.path.join(OUT, "sheet%02d.jpg" % (s // PER)), quality=85)
print(len(rows), "rows")
