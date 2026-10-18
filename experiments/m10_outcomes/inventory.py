"""M10's first question: what the episode store holds to learn outcomes from.

    python3 inventory.py [EPISODES.db] [OUT.jsonl]

One row per goal an autonomous turn chose and set off on: when, which world it
was chosen from (the world generation: the old store's, or the world model's
since 2026-10-17), the goal's type, what it predicted (the gain's kind and
value, the gain and utility the scorer gave it, its predicted time and travel),
and what was measured after it (how the attempt closed, the placement's
uncertainty before and after, whether a change was still reported, time and
travel spent). Then a summary by goal type and world. Read-only; run on the
rover, where the store is, or on a copy.
"""
import collections
import json
import os
import sqlite3
import sys
import time

DB = os.path.expanduser("~/.ugv/autonomy/episodes.db")


def rows(db):
    eps = {i: dict(ref=r, at=a, generation=g, map_session=m)
           for i, r, a, g, m in db.execute(
               "select id, ref, opened_at, world_generation, map_session from episodes")}
    per = collections.defaultdict(list)
    for eid, seq, at, kind, body in db.execute(
            "select episode_id, seq, at, kind, body_json from events "
            "where kind in ('candidate', 'decision', 'call', 'measured', 'closed') "
            "order by episode_id, seq"):
        per[eid].append((at, kind, json.loads(body)))
    for eid, events in per.items():
        decision = next((b for _, k, b in events if k == "decision"), None)
        if not decision or not decision.get("chose"):
            continue
        chose = decision["chose"]
        cand = next((b for _, k, b in events if k == "candidate" and b.get("goal") == chose), None)
        if cand is None:
            continue
        params = cand.get("params") or {}
        score = params.get("score") or {}
        attempt = next((b for _, k, b in events if k == "measured"
                        and b.get("what") == "the attempt"), {})
        closed = next((b for _, k, b in events if k == "closed"), {})
        calls = [b for _, k, b in events if k == "call"]
        e = eps[eid]
        yield {
            "episode": eid, "at": round(e["at"], 1), "generation": e["generation"],
            "map_session": e["map_session"], "type": params.get("type"), "goal": chose,
            "gain_kind": params.get("gain_kind"), "gain_value": params.get("gain_value"),
            "gain": score.get("gain"), "utility": score.get("utility"),
            "weights": score.get("weights_version"),
            "time_s_predicted": params.get("time_s"), "travel_m_predicted": params.get("travel_m"),
            "candidates": sum(1 for _, k, _ in events if k == "candidate"),
            "outcome": closed.get("outcome"),
            "uncertainty_before_m": attempt.get("placement_uncertainty_before_m"),
            "uncertainty_after_m": attempt.get("placement_uncertainty_after_m"),
            "filed": attempt.get("filed"), "put_aside": attempt.get("put_aside"),
            "change_still_reported": attempt.get("change_still_reported"),
            "travelled_m": attempt.get("travelled_m"),
            "call_s": round(sum(c.get("duration_s") or 0 for c in calls), 1),
            "calls": [c.get("call") for c in calls],
        }


def main():
    db_path = sys.argv[1] if len(sys.argv) > 1 else DB
    out = sys.argv[2] if len(sys.argv) > 2 else None
    db = sqlite3.connect("file:%s?mode=ro" % db_path, uri=True)
    got = list(rows(db))
    if out:
        with open(out, "w") as f:
            for r in got:
                f.write(json.dumps(r) + "\n")
    model_since = time.mktime((2026, 10, 17, 0, 0, 0, 0, 0, -1))
    table = collections.defaultdict(collections.Counter)
    for r in got:
        world = "model" if r["at"] >= model_since else "store"
        key = (world, r["type"])
        table[key]["chosen"] += 1
        table[key]["outcome " + str(r["outcome"])] += 1
        if r["uncertainty_before_m"] is not None and r["uncertainty_after_m"] is not None:
            table[key]["placement measured"] += 1
            if r["uncertainty_after_m"] < r["uncertainty_before_m"] - 1e-6:
                table[key]["placement improved"] += 1
        if r["change_still_reported"] is not None:
            table[key]["change measured"] += 1
    days = collections.Counter(time.strftime("%m-%d", time.localtime(r["at"])) for r in got)
    print("%d goals chosen, %d days: %s" % (len(got), len(days), dict(sorted(days.items()))))
    for (world, kind), c in sorted(table.items(), key=lambda kv: (kv[0][0], str(kv[0][1]))):
        print("%-6s %-22s %s" % (world, kind, dict(c)))


if __name__ == "__main__":
    main()
