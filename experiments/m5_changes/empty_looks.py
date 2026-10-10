"""Aimed looks at things in the depth camera's view: filed, or seen empty.

Read-only over the executive's episode store. For every improve_geometry
episode whose look was aimed with the thing in the depth camera's view, record
whether the look filed anything to the thing. Prints per-day counts, and for
things looked at more than once, whether a second look repeated the first's
answer. Output JSON lines to stdout with --dump.
"""
import collections, json, sqlite3, sys, time

DB = "file:/home/jetson/.ugv/autonomy/episodes.db?mode=ro"
db = sqlite3.connect(DB, uri=True)

rows = []
for eid, ref, opened in db.execute("select id, ref, opened_at from episodes order by id"):
    cands, chose, look, measured = {}, None, None, None
    for kind, body in db.execute(
            "select kind, body_json from events where episode_id=? order by seq", (eid,)):
        b = json.loads(body)
        if kind == "candidate":
            cands[b.get("goal")] = b
        elif kind == "decision":
            chose = b.get("chose")
        elif kind == "call" and b.get("call") == "world_inspect":
            look = b
        elif kind == "measured" and b.get("what") == "the attempt":
            measured = b
    if not chose or not chose.startswith("improve_geometry") or look is None:
        continue
    cand = cands.get(chose) or {}
    cons = (cand.get("params") or {}).get("constraints") or {}
    res = look.get("result") or {}
    filing = res.get("aimed_filing") or {}
    rows.append({
        "at": opened, "day": time.strftime("%Y-%m-%d", time.localtime(opened)),
        "target": chose.split("@")[0].split(":", 1)[1],
        "in_depth_view": cons.get("in_depth_view"),
        "ok": bool(look.get("ok")),
        "filed": filing.get("filed"),
        "why": str(filing.get("why") or "")[:80],
        "seen_empty": bool((measured or {}).get("seen_empty")),
        "improved": float((measured or {}).get("placement_improved_m") or 0.0),
        "before_m": (measured or {}).get("placement_uncertainty_before_m"),
    })

if "--dump" in sys.argv:
    for r in rows:
        print(json.dumps(r))
    sys.exit()

print("aimed improve_geometry looks:", len(rows))
by_day = collections.defaultdict(collections.Counter)
for r in rows:
    c = by_day[r["day"]]
    c["looks"] += 1
    if r["in_depth_view"] is True:
        c["in_view"] += 1
        c["in_view_filed"] += bool(r["filed"])
        c["in_view_empty"] += r["seen_empty"]
print("day          looks in_view filed empty")
for day in sorted(by_day):
    c = by_day[day]
    print(f"{day}  {c['looks']:5d} {c['in_view']:7d} {c['in_view_filed']:5d} {c['in_view_empty']:5d}")

whys = collections.Counter(r["why"] for r in rows if r["in_depth_view"] is True and not r["filed"])
print("\nwhy an in-view look filed nothing:")
for why, n in whys.most_common(8):
    print(f"  {n:4d}  {why}")

# Second looks: consecutive in-view looks at the same thing, same day.
seq = collections.defaultdict(list)
for r in rows:
    if r["in_depth_view"] is True:
        seq[(r["day"], r["target"])].append(r)
pairs = collections.Counter()
for looks in seq.values():
    for a, b in zip(looks, looks[1:]):
        pairs[(a["seen_empty"], b["seen_empty"])] += 1
print("\nconsecutive in-view looks at one thing, same day (first, second):")
for (a, b), n in sorted(pairs.items()):
    print(f"  {'empty' if a else 'found':5s} -> {'empty' if b else 'found':5s}: {n}")
things = collections.Counter()
for (day, target), looks in seq.items():
    e = sum(l["seen_empty"] for l in looks)
    things["things"] += 1
    things["ever_empty"] += e > 0
    things["always_empty"] += e == len(looks)
    things["always_empty_2plus"] += (e == len(looks) and len(looks) >= 2)
    things["looked_2plus"] += len(looks) >= 2
print("\nthings looked at in view (per day):", dict(things))

print()
print("in-view looks by how well the thing was placed before the look:")
bands = [(0, 0.25), (0.25, 0.5), (0.5, 1.0), (1.0, 99)]
for lo, hi in bands:
    sel = [r for r in rows if r["in_depth_view"] is True and r["before_m"] is not None
           and lo <= float(r["before_m"]) < hi]
    e = sum(r["seen_empty"] for r in sel)
    f = sum(bool(r["filed"]) for r in sel)
    print(f"  placed to {lo:.2f}-{hi:.2f} m: {len(sel):3d} looks, filed {f:3d}, empty {e:3d}")
