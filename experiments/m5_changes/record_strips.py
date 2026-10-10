"""Photo strips of whole records, for labelling what each record is.

For each record id: up to SIX crops of looks filed to it, from different
pictures, spread over the record's history rather than only the latest, with a
little context round each region. Five records to a sheet, each row headed by
the record's id and its look count. Written to /tmp/strips/.

Run on the rover: python3 record_strips.py ids.json
"""
import json, os, sqlite3, sys

from PIL import Image, ImageDraw

SIX = 6
PER_SHEET = 5
H = 200
db = sqlite3.connect("file:" + os.path.expanduser("~/.ugv/world/world.db") + "?mode=ro", uri=True)
ids = json.load(open(sys.argv[1]))
OUT = "/tmp/strips"
os.makedirs(OUT, exist_ok=True)


def crops_of(eid):
    rows = db.execute("select id, frame_path, bbox_json from observations where entity_id=? "
                      "and bbox_json is not null order by observed_at", (eid,)).fetchall()
    by_frame = {}
    for oid, path, bb in rows:
        if path and os.path.exists(path) and path not in by_frame:
            by_frame[path] = (oid, json.loads(bb))
    frames = list(by_frame.items())
    if len(frames) > SIX:
        step = len(frames) / SIX
        frames = [frames[int(i * step)] for i in range(SIX)]
    out = []
    for path, (oid, (x0, y0, x1, y1)) in frames:
        im = Image.open(path).convert("RGB")
        w, h = im.size
        px, py = 0.25 * (x1 - x0), 0.25 * (y1 - y0)
        box = (int(max(0, (x0 - px) * w)), int(max(0, (y0 - py) * h)),
               int(min(w, (x1 + px) * w)), int(min(h, (y1 + py) * h)))
        c = im.crop(box)
        d = ImageDraw.Draw(c)
        d.rectangle([x0 * w - box[0], y0 * h - box[1], x1 * w - box[0], y1 * h - box[1]],
                    outline=(255, 0, 0), width=3)
        c.thumbnail((H * 2, H))
        out.append(c)
    return out, len(rows)


index = []
for s in range(0, len(ids), PER_SHEET):
    chunk = ids[s:s + PER_SHEET]
    rows = []
    for eid in chunk:
        crops, n = crops_of(eid)
        rows.append((eid, n, crops))
    width = max(sum(c.width + 4 for c in cr) for _, _, cr in rows) + 140
    sheet = Image.new("RGB", (max(width, 400), (H + 8) * len(rows)), "white")
    d = ImageDraw.Draw(sheet)
    for i, (eid, n, crops) in enumerate(rows):
        y = i * (H + 8)
        d.text((4, y + 4), eid, fill=(0, 0, 0))
        d.text((4, y + 20), f"{n} looks", fill=(80, 80, 80))
        x = 140
        for c in crops:
            sheet.paste(c, (x, y))
            x += c.width + 4
        d.line([(0, y + H + 4), (sheet.width, y + H + 4)], fill=(200, 200, 200))
    name = f"strip-{s // PER_SHEET:02d}.jpg"
    sheet.save(os.path.join(OUT, name), quality=78)
    index.append({"sheet": name, "records": chunk})
json.dump(index, open(os.path.join(OUT, "index.json"), "w"), indent=1)
print(len(index), "sheets")
