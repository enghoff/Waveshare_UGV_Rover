"""Contact sheets for empty aimed looks: the look, and what the thing looked like.

Each sheet: the aimed look's picture with every region outlined (grey) and the
region nearest the thing's bearing in red, then up to four earlier crops filed
to the thing, each from a different picture. Written to /tmp/sheets/.

Run on the rover after why_empty.py (it reuses /tmp/wscopy):
    python3 sheets.py --day 2026-10-10 [--max 16]
"""
import json, math, os, sqlite3, sys, time

from PIL import Image, ImageDraw

sys.path.insert(0, os.path.expanduser("~/ugv"))
from world_state import resolve, store as store_mod  # noqa: E402

world = store_mod.WorldStore("/tmp/wscopy")
wdb = sqlite3.connect("file:/tmp/wscopy/world.db?mode=ro", uri=True)
ep = sqlite3.connect("file:/home/jetson/.ugv/autonomy/episodes.db?mode=ro", uri=True)
OUT = "/tmp/sheets"
os.makedirs(OUT, exist_ok=True)
day = sys.argv[sys.argv.index("--day") + 1] if "--day" in sys.argv else None
ONLY = None
if "--only" in sys.argv:
    _f, _k = sys.argv[sys.argv.index("--only") + 1:sys.argv.index("--only") + 3]
    ONLY = {e for e, v in json.load(open(_f)).items() if isinstance(v, list) and v[0] == _k}
most = int(sys.argv[sys.argv.index("--max") + 1]) if "--max" in sys.argv else 16
snaps = {}


def snapshot(d):
    if d not in snaps:
        row = ep.execute("select body_json from snapshots where digest=?", (d,)).fetchone()
        snaps[d] = json.loads(row[0]) if row else None
    return snaps[d]


def placement_then(inputs, target):
    sit = snapshot(inputs) or {}
    body = snapshot(sit.get("world_and_map")) if isinstance(sit.get("world_and_map"), str) else None
    for e in (body or {}).get("entities") or []:
        if e.get("id") == target:
            return e.get("placement")
    return None


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def box(img, bb, colour, width):
    w, h = img.size
    x0, y0, x1, y1 = bb
    ImageDraw.Draw(img).rectangle([x0 * w, y0 * h, x1 * w, y1 * h], outline=colour, width=width)


made = 0
index = []
for eid, opened in ep.execute("select id, opened_at from episodes order by id"):
    if made >= most:
        break
    if day and time.strftime("%Y-%m-%d", time.localtime(opened)) != day:
        continue
    cands, chose, inputs, look = {}, None, None, None
    for kind, body in ep.execute("select kind, body_json from events where episode_id=? order by seq", (eid,)):
        b = json.loads(body)
        if kind == "candidate":
            cands[b.get("goal")] = b
        elif kind == "decision":
            chose, inputs = b.get("chose"), b.get("inputs")
        elif kind == "call" and b.get("call") == "world_inspect":
            look = b
    if not chose or not chose.startswith("improve_geometry") or look is None:
        continue
    cons = ((cands.get(chose) or {}).get("params") or {}).get("constraints") or {}
    res = look.get("result") or {}
    filing = res.get("aimed_filing") or {}
    if cons.get("in_depth_view") is not True or filing.get("filed") or "points at it" not in str(filing.get("why")):
        continue
    target = chose.split("@")[0].split(":", 1)[1]
    if ONLY is not None and target not in ONLY:
        continue
    p = placement_then(inputs, target)
    rows = world.observations(frame_id=res.get("frame_id"), limit=64, vectors=True)
    if not p or not rows:
        continue
    near, near_off, near_seen = None, None, None
    for o in rows:
        ray = resolve.ray_of(o, None)
        if not ray:
            continue
        off = wrap(float(ray["bearing_deg"]) - math.degrees(math.atan2(
            float(p["y_m"]) - float(ray["y_m"]), float(p["x_m"]) - float(ray["x_m"]))))
        if near is None or abs(off) < abs(near_off):
            near, near_off = o, off
            vec = o.get("dino_blob") or b""
            near_seen = resolve.appearance(world, target, vec) if vec else None
    if near is None:
        continue
    frame = Image.open(rows[0]["frame_path"]).convert("RGB")
    for o in rows:
        bb = json.loads(o["bbox_json"]) if isinstance(o.get("bbox_json"), str) else o.get("bbox")
        if bb:
            box(frame, bb, (160, 160, 160), 2)
    nb = json.loads(near["bbox_json"]) if isinstance(near.get("bbox_json"), str) else near.get("bbox")
    box(frame, nb, (255, 0, 0), 5)
    frame.thumbnail((800, 600))
    crops, seen_frames = [], set()
    for oid, fpath, bbj in wdb.execute(
            "select id, frame_path, bbox_json from observations where entity_id=? and observed_at < ? "
            "order by observed_at desc", (target, opened)):
        if fpath in seen_frames or not bbj or not fpath or not os.path.exists(fpath):
            continue
        seen_frames.add(fpath)
        im = Image.open(fpath).convert("RGB")
        w, h = im.size
        x0, y0, x1, y1 = json.loads(bbj)
        pad = 0.15
        cx0, cy0 = max(0, (x0 - pad * (x1 - x0)) * w), max(0, (y0 - pad * (y1 - y0)) * h)
        cx1, cy1 = min(w, (x1 + pad * (x1 - x0)) * w), min(h, (y1 + pad * (y1 - y0)) * h)
        c = im.crop((int(cx0), int(cy0), int(cx1), int(cy1)))
        c.thumbnail((290, 290))
        crops.append(c)
        if len(crops) == 4:
            break
    sheet = Image.new("RGB", (frame.width + 300, max(frame.height, 300 * max(1, len(crops)) // 1)), "white")
    sheet.paste(frame, (0, 0))
    y = 0
    for c in crops[:2]:
        sheet.paste(c, (frame.width + 5, y))
        y += c.height + 5
    if len(crops) > 2:
        sheet2 = Image.new("RGB", (sheet.width, sheet.height + 300), "white")
        sheet2.paste(sheet, (0, 0))
        x = 0
        for c in crops[2:]:
            sheet2.paste(c, (x, sheet.height + 5))
            x += c.width + 5
        sheet = sheet2
    name = f"{made:02d}-{target.replace(':', '')}.jpg"
    sheet.save(os.path.join(OUT, name), quality=80)
    index.append({"sheet": name, "at": time.strftime("%m-%d %H:%M:%S", time.localtime(opened)),
                  "target": target, "claim_m": p.get("stated_uncertainty_m"),
                  "near_off_deg": round(near_off, 1), "appearance": None if near_seen is None else round(near_seen, 2),
                  "crops": len(crops), "frame": res.get("frame_id")})
    made += 1
json.dump(index, open(os.path.join(OUT, "index.json"), "w"), indent=1)
print(json.dumps(index, indent=1))
