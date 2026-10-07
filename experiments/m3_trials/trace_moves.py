"""Every move navigation made during an M3 session, matched to the episode that asked
for it; and what followed every failed action. Run on the rover from ~/ugv/autonomy
with PYTHONPATH=. Prints a summary and writes /tmp/trace_moves.json."""
import datetime, glob, json, os, re
import store

s = store.EpisodeStore()
db = s.db
since = datetime.datetime(2026, 10, 5).timestamp()

# Sessions: runs of episodes with no gap over 150 s.
eps = db.execute("select id, opened_at from episodes where opened_at > ? order by id",
                 (since,)).fetchall()
sessions, current = [], []
for eid, at in eps:
    if current and at - current[-1][1] > 150:
        sessions.append(current); current = []
    current.append((eid, at))
if current:
    sessions.append(current)

calls = {}      # episode -> list of (at, call body)
for eid, at, bj in db.execute("select episode_id, at, body_json from events where kind='call' "
                              "and episode_id >= ? order by episode_id, seq", (eps[0][0],)):
    calls.setdefault(eid, []).append((at, json.loads(bj)))
decision = {eid: json.loads(bj) for eid, bj in db.execute(
    "select episode_id, body_json from events where kind='decision' and episode_id >= ?", (eps[0][0],))}

moves = []
pat = re.compile(r"\[(\d+\.\d+)\] \[nav_bridge\]: (goto|turn|drive|explore): (\S+) -- (.*)")
for path in glob.glob(os.path.expanduser("~/.ros/log/**/*.log"), recursive=True) + \
        glob.glob(os.path.expanduser("~/.ros/log/*.log")):
    try:
        with open(path, errors="replace") as handle:
            for line in handle:
                m = pat.search(line)
                if m and float(m.group(1)) > since:
                    moves.append((float(m.group(1)), m.group(2), m.group(3), m.group(4)[:120]))
    except OSError:
        pass
moves = sorted(set(moves))

report = []
for n, sess in enumerate(sessions, 1):
    first, last = sess[0][1], max(at for eid, at in sess)
    ids = [eid for eid, _ in sess]
    end = max([at for eid in ids for at, _ in calls.get(eid, [])] + [last]) + 5
    drives = [(at, eid, c) for eid in ids for at, c in calls.get(eid, []) if c.get("call") == "drive_to"]
    inside = [m for m in moves if first - 2 <= m[0] <= end]
    matched, unmatched = [], []
    used = set()
    for m in inside:
        best = None
        for at, eid, c in drives:
            dt = abs(at - m[0])
            if dt <= 5 and (at, eid) not in used and (best is None or dt < best[0]):
                best = (dt, at, eid, c)
        if best:
            used.add((best[1], best[2])); matched.append((m, best[2], round(best[0], 2)))
        else:
            unmatched.append(m)
    not_navigated = [(eid, (c.get("result") or {}).get("detail") or c.get("error"))
                     for at, eid, c in drives if (at, eid) not in used]
    # What followed each failed action.
    after_failure = []
    for i, eid in enumerate(ids):
        failed = [c for _, c in calls.get(eid, []) if not c.get("ok")]
        if failed and i + 1 < len(ids):
            nxt = ids[i + 1]
            d = decision.get(nxt) or {}
            after_failure.append((eid, nxt, (d.get("chose") or "none")[:60],
                                  bool(d), str(d.get("why") or "")[:80]))
    report.append({"session": n, "episodes": [ids[0], ids[-1]],
                   "from": datetime.datetime.fromtimestamp(first).isoformat(timespec="seconds"),
                   "drive_calls": len(drives), "nav_moves": len(inside),
                   "matched": len(matched), "unmatched": unmatched,
                   "drives_navigation_never_ran": not_navigated,
                   "after_failure": after_failure,
                   "same_goal_again": sum(1 for f in after_failure
                                          if f[2] == ((decision.get(f[0]) or {}).get("chose") or "")[:60])})
for r in report:
    print(r["session"], r["from"], "episodes", r["episodes"], "| drive calls", r["drive_calls"],
          "| nav moves", r["nav_moves"], "matched", r["matched"], "| unmatched", len(r["unmatched"]),
          "| drives nav never ran", len(r["drives_navigation_never_ran"]),
          "| failures followed by", len(r["after_failure"]), "decisions, same goal again", r["same_goal_again"])
    for u in r["unmatched"]:
        print("    UNMATCHED", u)
json.dump(report, open("/tmp/trace_moves.json", "w"), indent=1, default=str)
