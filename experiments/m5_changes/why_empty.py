"""Why an aimed look at a thing in the depth camera's view filed nothing.

For each improve_geometry look with the thing in view, rebuild the thing's
placement as the executive saw it (the world snapshot its decision names), take
the look's regions from a copy of the world store, and put each region through
the tests `aimed.choose` applies: bearing within the match tolerance, height,
range, appearance. Reports, per look, the region closest in bearing and which
test it failed, and a summary over all of them.

Run on the rover: python3 why_empty.py [--day 2026-10-10] [--all]
"""
import collections, json, math, os, sqlite3, sys, time

sys.path.insert(0, os.path.expanduser("~/ugv"))
from world_state import locate, resolve, store as store_mod  # noqa: E402

COPY = "/tmp/wscopy"
os.makedirs(COPY, exist_ok=True)
src = sqlite3.connect("file:" + os.path.expanduser("~/.ugv/world/world.db") + "?mode=ro", uri=True)
dst = sqlite3.connect(os.path.join(COPY, "world.db"))
src.backup(dst)
dst.close()
src.close()
world = store_mod.WorldStore(COPY)

ep = sqlite3.connect("file:/home/jetson/.ugv/autonomy/episodes.db?mode=ro", uri=True)
snap_cache = {}


def snapshot(digest):
    if digest not in snap_cache:
        row = ep.execute("select body_json from snapshots where digest=?", (digest,)).fetchone()
        snap_cache[digest] = json.loads(row[0]) if row else None
    return snap_cache[digest]


def placement_then(inputs, target):
    sit = snapshot(inputs)
    if not sit:
        return None
    wm = sit.get("world_and_map")
    body = snapshot(wm) if isinstance(wm, str) else None
    for e in (body or {}).get("entities") or []:
        if e.get("id") == target:
            return e.get("placement")
    return None


def bearing_to(ray, p):
    return math.degrees(math.atan2(float(p["y_m"]) - float(ray["y_m"]),
                                   float(p["x_m"]) - float(ray["x_m"])))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


day = sys.argv[sys.argv.index("--day") + 1] if "--day" in sys.argv else None
ONLY = None
if "--only" in sys.argv:
    _f, _k = sys.argv[sys.argv.index("--only") + 1:sys.argv.index("--only") + 3]
    ONLY = {e for e, v in json.load(open(_f)).items() if isinstance(v, list) and v[0] == _k}
looks = []
for eid, opened in ep.execute("select id, opened_at from episodes order by id"):
    if day and time.strftime("%Y-%m-%d", time.localtime(opened)) != day:
        continue
    cands, chose, inputs, look, measured = {}, None, None, None, None
    for kind, body in ep.execute(
            "select kind, body_json from events where episode_id=? order by seq", (eid,)):
        b = json.loads(body)
        if kind == "candidate":
            cands[b.get("goal")] = b
        elif kind == "decision":
            chose, inputs = b.get("chose"), b.get("inputs")
        elif kind == "call" and b.get("call") == "world_inspect":
            look = b
        elif kind == "measured" and b.get("what") == "the attempt":
            measured = b
    if not chose or not chose.startswith("improve_geometry") or look is None:
        continue
    cons = ((cands.get(chose) or {}).get("params") or {}).get("constraints") or {}
    if cons.get("in_depth_view") is not True:
        continue
    res = look.get("result") or {}
    filing = res.get("aimed_filing") or {}
    target = chose.split("@")[0].split(":", 1)[1]
    if ONLY is not None and target not in ONLY:
        continue
    looks.append({"at": opened, "target": target, "frame": res.get("frame_id"),
                  "filed": filing.get("filed"), "why": filing.get("why"),
                  "placement": placement_then(inputs, target)})

summary = collections.Counter()
misses = []
for lk in looks:
    if lk["filed"] and "--all" not in sys.argv:
        continue
    p = lk["placement"]
    if not p or "x_m" not in p or not lk["frame"]:
        summary["no placement or frame"] += 1
        continue
    rows = world.observations(frame_id=lk["frame"], limit=64, vectors=True)
    rays = [(r, o) for r, o in ((resolve.ray_of(o, None), o) for o in rows) if r]
    if not rays:
        summary["no region with a bearing"] += 1
        continue
    best = None
    for ray, obs in rays:
        off = wrap(float(ray["bearing_deg"]) - bearing_to(ray, p))
        tol = locate.match_tolerance(p, ray)
        rng = math.hypot(float(p["x_m"]) - float(ray["x_m"]), float(p["y_m"]) - float(ray["y_m"]))
        miss = locate.cross_track_of(float(p["x_m"]), float(p["y_m"]), ray)
        tests = {
            "bearing": miss <= tol,
            "height": locate.stands_as_high(p, ray),
            "range": locate.stands_at_range(p, ray),
            "agrees": locate.agrees(p, ray, tol),
        }
        vec = obs.get("dino_blob") or b""
        seen = resolve.appearance(world, lk["target"], vec) if vec else None
        tests["appearance"] = seen is None or seen >= resolve.DIFFERENT_THING
        row = {"off_deg": round(off, 1), "miss_m": round(miss, 2), "tol_m": round(tol, 2),
               "range_to_m": round(rng, 2), "measured_range_m": obs.get("range_m"),
               "appearance": None if seen is None else round(seen, 2),
               "label": (obs.get("label") or "")[:20], "tests": tests}
        if best is None or abs(off) < abs(best["off_deg"]):
            best = row
    failed = [k for k, v in best["tests"].items() if not v and k != "agrees"]
    key = "+".join(failed) if failed else ("agrees-only" if not best["tests"]["agrees"] else "passes now")
    summary["nearest region fails: " + key] += 1
    misses.append(abs(best["off_deg"]))
    if "-v" in sys.argv:
        print(time.strftime("%m-%d %H:%M", time.localtime(lk["at"])), lk["target"],
              "claim", p.get("stated_uncertainty_m"), json.dumps(best))

print("looks examined:", sum(summary.values()))
for k, n in summary.most_common():
    print(f"  {n:3d}  {k}")
if misses:
    misses.sort()
    q = lambda f: misses[min(len(misses) - 1, int(f * len(misses)))]
    print("bearing off, nearest region (deg): median %.1f, 75%% %.1f, 90%% %.1f" % (q(0.5), q(0.75), q(0.9)))
