"""Was each staged thing photographed in loop A? Run during the loop's second
half, so the move list is ready when the rover reaches the bedroom.

    python3 session_check.py NAME PLANNED.json [--db DB] [--frames DIR] [--sessions DIR]

For each thing the plan sets out ("added"), the regions of the session's
loop A (from its first entry, or from the session's start if loop A has not
ended) whose sight line passes within NEAR_M of its place, nearest first, and
a contact sheet of them in ~/.ugv/sessions/NAME-check/ for the coding agent to
look at. A thing with fewer than MIN_PICTURES such regions is not to be moved
(identity across a move cannot be judged on less), and the list says so.
Written after M5 session 5, where this check was written while the rover
waited (2026-10-16); tried on that session's data before session 6.
"""
import json
import math
import os
import sqlite3
import sys

from PIL import Image, ImageDraw

NEAR_M = 0.6
NEAR_PER_M = 0.05
MAX_M = 5.0
MIN_PICTURES = 2
#: Staged things stand on the floor: a region whose lower edge is higher than
#: this above the horizontal is something raised behind the place (a painting
#: on the wall the sight line meets), and a measured range this far from the
#: place is something nearer or further along the same line.
FLOOR_EDGE_DEG = 5.0
RANGE_SLACK_M = 1.0


def arg(name, default):
    if name in sys.argv:
        return sys.argv[sys.argv.index(name) + 1]
    return default


def main():
    name, planned = sys.argv[1], sys.argv[2]
    sessions = os.path.expanduser(arg("--sessions", "~/.ugv/sessions"))
    db_path = os.path.expanduser(arg("--db", "~/.ugv/world/world.db"))
    frames = arg("--frames", None)
    rec = json.load(open(os.path.join(sessions, name + ".json")))["entries"]
    loop_a = [e["at"] for e in rec if e["step"] == "loop A" and e.get("begin")]
    begin = loop_a[0] if loop_a else rec[0]["at"]
    ends = [e["at"] for e in rec if e["step"] == "loop A" and e.get("end")]
    end = ends[0] if ends else 1e12
    plan = json.load(open(planned))
    db = sqlite3.connect("file:%s?mode=ro" % db_path, uri=True)
    rows = db.execute("SELECT id, observed_at, frame_path, bbox_json, observer_pose_json, bearing_deg,"
                      " elevation_deg, elevation_span_deg, range_m FROM observations WHERE observed_at"
                      " BETWEEN ? AND ? AND bearing_deg IS NOT NULL AND camera = 'gimbal'",
                      (begin, end)).fetchall()
    out_dir = os.path.join(sessions, name + "-check")
    os.makedirs(out_dir, exist_ok=True)
    moves = {m["what"] for m in plan.get("moves", [])}
    verdict = {}
    for thing in plan.get("added", []):
        px, py = thing["at"]
        hits = []
        for rid, t, path, bbox, pose, bearing, elev, span, rng in rows:
            if elev is not None and span is not None and elev - span / 2.0 > FLOOR_EDGE_DEG:
                continue
            p = json.loads(pose or "{}")
            if p.get("x_m") is None:
                continue
            dx, dy = px - p["x_m"], py - p["y_m"]
            if math.hypot(dx, dy) > MAX_M:
                continue
            b = math.radians(bearing)
            along = dx * math.cos(b) + dy * math.sin(b)
            across = abs(-dx * math.sin(b) + dy * math.cos(b))
            if rng is not None and abs(rng - along) > RANGE_SLACK_M:
                continue
            if along > 0.3 and across <= NEAR_M + NEAR_PER_M * along:
                hits.append((across, rid, t, path, bbox))
        hits.sort()
        tiles = []
        for across, rid, t, path, bbox in hits[:16]:
            f = path if frames is None else os.path.join(frames, os.path.basename(path))
            full = f.replace(".jpg", ".full.jpg")
            f = full if os.path.exists(full) else f
            if not os.path.exists(f):
                continue
            im = Image.open(f).convert("RGB")
            w, h = im.size
            x0, y0, x1, y1 = json.loads(bbox)
            mx, my = 0.3 * (x1 - x0), 0.3 * (y1 - y0)
            crop = im.crop((int(max(0, x0 - mx) * w), int(max(0, y0 - my) * h),
                            int(min(1, x1 + mx) * w), int(min(1, y1 + my) * h))).resize((220, 200))
            ImageDraw.Draw(crop).text((4, 4), "%d" % rid, fill=(255, 255, 0))
            tiles.append(crop)
        if tiles:
            sheet = Image.new("RGB", (220 * min(8, len(tiles)), 200 * ((len(tiles) + 7) // 8)))
            for k, tile in enumerate(tiles):
                sheet.paste(tile, ((k % 8) * 220, (k // 8) * 200))
            sheet.save(os.path.join(out_dir, thing["what"].replace(" ", "_") + ".jpg"), quality=85)
        frames_seen = len({h[3] for h in hits})
        ok = frames_seen >= MIN_PICTURES
        verdict[thing["what"]] = {"regions": len(hits), "pictures": frames_seen,
                                  "move": (thing["what"] in moves) and ok}
        print("%-16s %3d regions in %3d pictures  %s" % (
            thing["what"], len(hits), frames_seen,
            ("MOVE" if ok else "DO NOT MOVE (too few pictures)") if thing["what"] in moves else "stays"))
    json.dump(verdict, open(os.path.join(out_dir, "verdict.json"), "w"), indent=1)
    print("sheets in", out_dir, "-- look at them before telling the owner")


if __name__ == "__main__":
    main()
