"""Does anything about a record say whether a look aimed at it will file?

A geometry goal is only worth its drive if the record it aims at is something
the camera will recognise where the record says it is. On 2026-10-08 most were
not (docs/progress/2026-10-08-why-aimed-looks-miss.md). This lists, for every
aimed look with a filing outcome, a few facts about its target and whether the
look filed, so that a predictor can be looked for before one is built:

    observations   how many looks the record holds
    viewpoints     separate places it was placed from
    days           distinct days it was seen on
    ranged_share   share of its observations with a depth range
    agreeing_share share of its ranged observations whose range point lies
                   within its stated uncertainty of the placement
    uncertainty_m  the placement's matching tolerance
    extent_m, above_floor_m

Read-only. Two sources:

    --replay <replay_aimed.py output> --store <the store it ran on>
    --looks <aimed-looks.json of a session> --store <a copy of the rover's store>

    python experiments/entity_association/target_features.py --replay ... --store ... \\
        --output <new-file>
"""
from __future__ import annotations

import argparse
import json
import math
import sqlite3
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import locate, resolve  # noqa: E402
from world_state.store import _readable  # noqa: E402


def features(db, entity_id):
    row = db.execute("SELECT placement_json, observation_count FROM entities WHERE id=?",
                     (entity_id,)).fetchone()
    if not row:
        return None
    p = json.loads(row[0] or "{}")
    if "x_m" not in p:
        return None
    obs = [_readable(dict(r)) for r in db.execute(
        "SELECT * FROM observations WHERE entity_id=?", (entity_id,))]
    ranged = agreeing = 0
    days = set()
    for o in obs:
        days.add(time.strftime("%Y-%m-%d", time.localtime(o["observed_at"])))
        if o.get("range_m") is None or o.get("bearing_deg") is None:
            continue
        ray = resolve.ray_of(o)
        if ray is None:
            continue
        ranged += 1
        flat = float(o["range_m"]) * math.cos(math.radians(locate.elevation_of(ray) or 0.0))
        b = math.radians(ray["bearing_deg"])
        px, py = ray["x_m"] + flat * math.cos(b), ray["y_m"] + flat * math.sin(b)
        if math.hypot(px - p["x_m"], py - p["y_m"]) <= max(float(p.get("uncertainty_m") or 0), 0.3):
            agreeing += 1
    return {"observations": len(obs), "viewpoints": p.get("viewpoints"), "days": len(days),
            "ranged_share": round(ranged / len(obs), 2) if obs else None,
            "agreeing_share": round(agreeing / ranged, 2) if ranged else None,
            "uncertainty_m": p.get("uncertainty_m"), "extent_m": p.get("extent_m"),
            "above_floor_m": p.get("height_above_floor_m")}


def main():
    a = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    a.add_argument("--replay", type=Path)
    a.add_argument("--looks", type=Path, action="append", default=[])
    a.add_argument("--store", type=Path, required=True)
    a.add_argument("--output", type=Path, required=True)
    args = a.parse_args()
    assert not args.output.exists(), "choose a new output file"
    db = sqlite3.connect(args.store.resolve().as_uri() + "?mode=ro", uri=True)
    db.row_factory = sqlite3.Row
    rows = []
    if args.replay:
        for look in json.loads(args.replay.read_text())["looks_detail"]:
            if "not placed" in (look.get("why") or ""):
                continue
            rows.append((look["target"], bool(look.get("filed")), "replay"))
    for path in args.looks:
        for look in json.loads(path.read_text()):
            if look.get("status") != "ok":
                continue
            rows.append((look["target"], bool((look.get("filing") or {}).get("filed")), path.parent.name))
    out = []
    for target, filed, source in rows:
        f = features(db, target)
        if f:
            out.append({"target": target, "filed": filed, "source": source, **f})
    args.output.write_text(json.dumps(out, indent=1) + "\n")
    print(len(out), "looks;", sum(r["filed"] for r in out), "filed")
    for key in ("observations", "viewpoints", "days", "ranged_share", "agreeing_share",
                "uncertainty_m", "extent_m", "above_floor_m"):
        values = {flag: [r[key] for r in out if r["filed"] == flag and r[key] is not None]
                  for flag in (True, False)}
        if values[True] and values[False]:
            print("%-15s filed median %6.2f (n %d)   not filed median %6.2f (n %d)" % (
                key, statistics.median(values[True]), len(values[True]),
                statistics.median(values[False]), len(values[False])))


if __name__ == "__main__":
    main()
