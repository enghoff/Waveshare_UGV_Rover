"""Is a claim from aimed depth honest when it rests on one range, or on ranges that disagree?

Found on the rover on 2026-10-08 (M4 session 1): object:367, a kitchen cabinet,
was filed two ranged aimed looks about a metre apart. Its claim went 0.24, 0.74,
0.62 and back to 0.20 m as its bearing placement moved, because a range that no
longer points at the bearing placement is dropped, and one range left alone
claims the 0.20 m floor. This scores the claim rule against the owner's tape on
the six targets of 2026-10-03 (the looks `placement_calibration.py` uses, every
ranged one treated as aimed), split by how many viewpoints ranged the thing:

    floor       the production rule: agreeing ranges only, never below 0.20 m
    p68         the same, with the floor for a single ranged viewpoint at the
                68th percentile of single ranges' misses on the tape
    disagree    the same as floor, but the spread is taken over every ranged
                point, agreeing or not, so a range that disagrees widens the
                claim rather than vanishing
    both        p68 and disagree together

Prints the single-range miss distribution, then for each rule how often the tape
lies inside the claim and the median claim, for one ranged viewpoint and for
more.

    python experiments/entity_association/claim_honesty.py --output <new-file>
"""
from __future__ import annotations

import argparse
import json
import math
import sqlite3
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "captures/2026-10-03-targets"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from world_state import aimed, locate, resolve  # noqa: E402
from world_state.store import _readable  # noqa: E402
from placement_calibration import STORE, TARGETS, current, moving  # noqa: E402


def points_of(rays_rows, base, every=False):
    """Each ranged look's point: (x, y, own error, viewpoint, agrees)."""
    out = []
    for ray, row in rays_rows:
        rng = row.get("range_m")
        if rng is None:
            continue
        agrees = resolve._allowance_used(base, ray) is not None
        if not agrees and not every:
            continue
        elevation = locate.elevation_of(ray) or 0.0
        flat = float(rng) * math.cos(math.radians(elevation))
        b = math.radians(ray["bearing_deg"])
        deg = aimed.MOVING_BEARING_DEG if moving(row) else aimed.STILL_BEARING_DEG
        own = math.hypot(float(row.get("range_sigma_m") or locate.RANGE_SIGMA_M),
                         flat * math.tan(math.radians(deg)))
        out.append((ray["x_m"] + flat * math.cos(b), ray["y_m"] + flat * math.sin(b), own,
                    (round(ray["x_m"] / locate.MIN_BASELINE_M),
                     round(ray["y_m"] / locate.MIN_BASELINE_M)), agrees))
    return out


def claim(rays_rows, single_floor, count_disagreeing):
    base = current(rays_rows)
    if base is None:
        return None
    every = points_of(rays_rows, base, every=True)
    agreeing = [p for p in every if p[4]]
    if not agreeing:
        return None
    x = statistics.median(p[0] for p in agreeing)
    y = statistics.median(p[1] for p in agreeing)
    over = every if count_disagreeing else agreeing
    spread = math.sqrt(sum((p[0] - x) ** 2 + (p[1] - y) ** 2 for p in over) / len(over))
    places = len({p[3] for p in agreeing})
    own = statistics.median(p[2] for p in agreeing) / math.sqrt(places)
    floor = single_floor if places == 1 else aimed.CLAIM_FLOOR_M
    return {"x_m": x, "y_m": y, "claim_m": max(spread, own, floor), "places": places}


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    assert not a.output.exists(), "choose a new output file"
    from frame import TRUTH, W2M
    accept = sqlite3.connect((ROOT / "captures/2026-10-03-targets/world-2026-10-03-acceptance.db")
                             .resolve().as_uri() + "?mode=ro", uri=True)
    store = sqlite3.connect(STORE.resolve().as_uri() + "?mode=ro", uri=True)
    store.row_factory = sqlite3.Row
    sequences, single_misses = {}, []
    for target, entities in TARGETS.items():
        ids = [r[0] for e in entities for r in accept.execute(
            "select id from observations where entity_id=?", (e,))]
        rows = [_readable(dict(r), vectors=True) for r in store.execute(
            "select * from observations where id in (%s) order by observed_at, id"
            % ",".join("?" * len(ids)), ids)]
        tape = W2M(*TRUTH[target][:2])
        seen = []
        for row in rows:
            ray = resolve.ray_of(row, None)
            if ray is not None:
                seen.append((ray, row))
                if row.get("range_m") is not None:
                    for px, py, *_ in points_of([(dict(ray), row)], {"x_m": 0, "y_m": 0}, every=True):
                        single_misses.append(math.hypot(px - tape[0], py - tape[1]))
        sequences[target] = (seen, tape)
    single_misses.sort()

    def pct(q):
        return single_misses[min(len(single_misses) - 1, int(q * len(single_misses)))]
    p68 = pct(0.68)
    print("single ranges against the tape: %d, median %.2f m, 68th %.2f, 90th %.2f"
          % (len(single_misses), pct(0.5), p68, pct(0.9)))
    rules = {"floor": (aimed.CLAIM_FLOOR_M, False), "p68": (p68, False),
             "disagree": (aimed.CLAIM_FLOOR_M, True), "both": (p68, True)}
    out = {"single_range_misses_m": [round(m, 3) for m in single_misses],
           "p68_m": round(p68, 3), "rules": {}}
    for name, (single_floor, disagreeing) in rules.items():
        states = []
        for target, (seen, tape) in sequences.items():
            for k in range(1, len(seen) + 1):
                got = claim([(dict(r), w) for r, w in seen[:k]], single_floor, disagreeing)
                if got:
                    off = math.hypot(got["x_m"] - tape[0], got["y_m"] - tape[1])
                    states.append((target, got["places"], off, got["claim_m"]))
        result = {}
        for label, keep in (("one ranged viewpoint", lambda s: s[1] == 1),
                            ("two or more", lambda s: s[1] >= 2), ("all", lambda s: True)):
            part = [s for s in states if keep(s)]
            if not part:
                continue
            result[label] = {"states": len(part),
                             "inside": round(sum(s[2] <= s[3] for s in part) / len(part), 3),
                             "median_off_m": round(statistics.median(s[2] for s in part), 3),
                             "median_claim_m": round(statistics.median(s[3] for s in part), 3)}
        out["rules"][name] = result
        print(name, json.dumps(result))
    a.output.write_text(json.dumps(out, indent=1) + "\n")


if __name__ == "__main__":
    main()
