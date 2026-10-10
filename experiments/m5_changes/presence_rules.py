"""How generous can "it is still there" be, on looks at real objects?

For every in-view aimed look at a record labelled one real object (labels JSON,
OBJ), the thing was there: nothing moved during these runs except the armchair,
which was not looked at in its absence. A presence rule that says "not there"
on one of these is a false alarm.

A rule: some region of the look points at the thing's placement (bearing within
`locate.match_tolerance`, no map reach), optionally agrees on height and range,
and looks like the record at least A (`resolve.appearance`). Each rule is also
run with a DECOY: the same look and placement, but appearance asked of a record
of a different kind of object -- what the rule would say if something else now
stood there. That is the rule's chance of missing a change.

Run on the rover after why_empty.py has made /tmp/wscopy:
    python3 presence_rules.py /tmp/labels.json
"""
import collections, json, math, os, random, sqlite3, sys, time

sys.path.insert(0, os.path.expanduser("~/ugv"))
from world_state import locate, resolve, store as store_mod  # noqa: E402

world = store_mod.WorldStore("/tmp/wscopy")
ep = sqlite3.connect("file:/home/jetson/.ugv/autonomy/episodes.db?mode=ro", uri=True)
labels = json.load(open(sys.argv[1]))
labels.pop("_about", None)

KINDS = ["painting", "cabinet", "armchair", "bed", "wardrobe", "lamp", "rug",
         "door", "tv", "bean"]


def kind_of(note):
    note = note.lower()
    for k in KINDS:
        if k in note:
            return k
    return "other"


obj = {e: kind_of(v[1]) for e, v in labels.items() if v[0] == "OBJ"}
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


looks = []
for eid, opened in ep.execute("select id, opened_at from episodes order by id"):
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
    if cons.get("in_depth_view") is not True:
        continue
    target = chose.split("@")[0].split(":", 1)[1]
    if target not in obj:
        continue
    res = look.get("result") or {}
    p = placement_then(inputs, target)
    if not p or "x_m" not in p or not res.get("frame_id"):
        continue
    rows = world.observations(frame_id=res["frame_id"], limit=64, vectors=True)
    rays = [(r, o) for r, o in ((resolve.ray_of(o, None), o) for o in rows) if r]
    looks.append({"at": opened, "target": target, "p": p, "rays": rays,
                  "filed": bool((res.get("aimed_filing") or {}).get("filed"))})

rng = random.Random(10)
others = collections.defaultdict(list)
for e, k in obj.items():
    others[k].append(e)


def decoy_for(target):
    k = obj[target]
    pool = [e for kk, es in others.items() if kk != k for e in es]
    return rng.choice(pool)


app_cache = {}


def app(entity, obs):
    key = (entity, obs["id"])
    if key not in app_cache:
        vec = obs.get("dino_blob") or b""
        app_cache[key] = resolve.appearance(world, entity, vec) if vec else None
    return app_cache[key]


def present(lk, entity, a, height, rng_):
    p = lk["p"]
    for ray, obs in lk["rays"]:
        if locate.cross_track_of(float(p["x_m"]), float(p["y_m"]), ray) > locate.match_tolerance(p, ray):
            continue
        if height and not locate.stands_as_high(p, ray):
            continue
        if rng_ and not locate.stands_at_range(p, ray):
            continue
        s = app(entity, obs)
        if s is not None and s >= a:
            return True
    return False


with_regions = [lk for lk in looks if lk["rays"]]
decoys = {id(lk): [decoy_for(lk["target"]) for _ in range(5)] for lk in with_regions}
print(f"looks at real objects in view: {len(looks)}, with regions: {len(with_regions)}, "
      f"filed by today's rule: {sum(lk['filed'] for lk in with_regions)}")
print("rule                         says present (true)   decoy says present")
for height, rng_ in ((False, False), (True, False), (True, True)):
    for a in (0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55):
        t = sum(present(lk, lk["target"], a, height, rng_) for lk in with_regions)
        d = sum(present(lk, z, a, height, rng_) for lk in with_regions for z in decoys[id(lk)])
        nd = 5 * len(with_regions)
        name = f"A>={a:.2f}" + (" +height" if height else "") + (" +range" if rng_ else "")
        print(f"  {name:26s} {t:3d}/{len(with_regions)} ({t / len(with_regions):4.0%})"
              f"        {d:3d}/{nd} ({d / nd:4.0%})")
