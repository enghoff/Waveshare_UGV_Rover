"""Why an aimed look filed nothing: the region nearest the aim, and the gate it failed.

Run on the rover, from anywhere, against a session's aimed looks as
`captures/2026-10-08-m4-session-1/aimed-looks.json` lists them (copy it over):

    python3 aimed_misses.py aimed-looks.json > misses.jsonl

Read-only: it works on a copy of `~/.ugv/world/world.db` in a temporary
directory, with the deployed `world_state`. One JSON line per look that took a
picture and filed nothing: the target's bearing from where the picture was
taken, the region whose bearing is nearest it -- its offset, allowance used,
height and range agreement, appearance -- the gate that turned it away, and the
two regions of the picture that look most like the target. Placements are read
as they stand now, not as each goal saw them.
"""
import json, math, sqlite3, sys, tempfile
sys.path.insert(0, "/home/jetson/ugv")
from world_state import locate, resolve
from world_state.store import WorldStore

looks = [r for r in json.load(open(sys.argv[1]))
         if r.get("status") == "ok" and not (r.get("filing") or {}).get("filed")]
tmp = tempfile.mkdtemp(prefix="why-none-")
store = WorldStore(tmp)
with sqlite3.connect("file:/home/jetson/.ugv/world/world.db?mode=ro", uri=True) as src:
    src.backup(store.db)
store._create()
placed = {e["id"]: e for e in store.placed(map_session=store.map_session())}
out = []
for look in looks:
    target = look["target"]
    p = (placed.get(target) or {}).get("placement")
    rows = store.observations(frame_id=look["frame_id"], vectors=True) if look.get("frame_id") else []
    entry = {"episode": look["episode"], "target": target, "regions": len(rows),
             "why": (look.get("filing") or {}).get("why")}
    if not p:
        entry["gate"] = "target not placed now"
        out.append(entry); print(json.dumps(entry)); continue
    best = None
    for row in rows:
        ray = resolve.ray_of(row, None)
        if not ray:
            continue
        to = math.degrees(math.atan2(p["y_m"] - ray["y_m"], p["x_m"] - ray["x_m"]))
        off = (ray["bearing_deg"] - to + 180) % 360 - 180
        dist = math.hypot(p["x_m"] - ray["x_m"], p["y_m"] - ray["y_m"])
        if best is None or abs(off) < abs(best["off_deg"]):
            used = resolve._allowance_used(p, ray)
            app = resolve.appearance(store, target, row.get("dino_blob") or b"")
            best = {"obs": row["id"], "off_deg": round(off, 1),
                    "cross_m": round(dist * math.sin(math.radians(abs(off))), 2),
                    "dist_m": round(dist, 2), "range_m": row.get("range_m"),
                    "tol_m": round(p.get("uncertainty_m", 0), 2),
                    "used": None if used is None else round(used, 2),
                    "high": locate.stands_as_high(p, ray),
                    "range_ok": locate.stands_at_range(p, ray),
                    "appearance": None if app is None else round(app, 2)}
    entry["nearest"] = best
    looks_like = []
    for row in rows:
        ray = resolve.ray_of(row, None)
        app = resolve.appearance(store, target, row.get("dino_blob") or b"")
        if ray is None or app is None:
            continue
        to = math.degrees(math.atan2(p["y_m"] - ray["y_m"], p["x_m"] - ray["x_m"]))
        off = (ray["bearing_deg"] - to + 180) % 360 - 180
        looks_like.append((round(app, 2), round(off, 1), row.get("range_m"),
                           round(math.hypot(p["x_m"] - ray["x_m"], p["y_m"] - ray["y_m"]), 2)))
    looks_like.sort(reverse=True)
    entry["most_alike"] = looks_like[:2]
    if best is None:
        entry["gate"] = "no region with a bearing"
    elif best["used"] is None:
        entry["gate"] = "outside the allowance"
    elif not best["high"]:
        entry["gate"] = "height"
    elif not best["range_ok"]:
        entry["gate"] = "range"
    elif best["appearance"] is not None and best["appearance"] < resolve.DIFFERENT_THING:
        entry["gate"] = "appearance"
    else:
        entry["gate"] = "passes now (placement has moved since)"
    out.append(entry)
    print(json.dumps(entry))
store.close()
