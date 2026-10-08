"""Does a placement's stated uncertainty say how far it is from where the thing is?

The six targets the owner taped on 2026-10-03 (`captures/2026-10-03-targets/`) are
seen in 126 looks matched to them by eye. Their looks are fed in time order, and
after each one the placement is worked out two ways:

    current     `locate.best_fix` then `locate.refine`, as the resolver does. Its
                stated uncertainty is never less than its best pair's.
    pooled      a least-squares fit over every agreeing bearing (fixed 2026-10-08
                before scoring against the tape): each bearing's sideways error is
                its range times the tangent of 2.0 degrees for a still look or 4.5
                for a moving one -- the median and tail of the misses measured on
                the labelled looks of 2026-10-07, not on the tape -- plus its
                origin error; bearings from within `MIN_BASELINE_M` of each other
                share one viewpoint's weight; bearings more than two sigma off are
                down-weighted (Huber), refitted three times. The stated
                uncertainty is the one-sigma radius, sqrt of the covariance's
                trace, the meaning a pair crossing's has.

Scored against the tape: how far each placement is from the target, the
uncertainty it states, and how often the tape lies inside it.

    python experiments/entity_association/placement_calibration.py --output <file>
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
from world_state import locate, resolve  # noqa: E402
from world_state.store import _readable  # noqa: E402

STILL_DEG, MOVING_DEG, HUBER = 2.0, 4.5, 2.0
TARGETS = {"T2": ["object:5"], "T3": ["object:3"], "T4": ["object:26", "object:37", "object:30"],
           "T7": ["object:4", "object:16"], "T8": ["object:6", "object:10"], "T9": ["object:22"]}
STORE = ROOT / ("captures/visibility-independent-20261007-12/world/recordings/"
                "visibility-independent-20261007-12/after.db")


def moving(row) -> bool:
    return ((row.get("origin_sigma_m") or 0.0) > 0.0
            or (row.get("bearing_sigma_deg") or 0.0) > locate.BEARING_SIGMA_DEG)


def pooled(rays_rows):
    """The pooled fit above. Returns {x_m, y_m, uncertainty_m} or None."""
    rays = [(ray, row) for ray, row in rays_rows]
    if len({row["inference_id"] for _, row in rays}) < 2:
        return None
    # Viewpoints: bearings taken within MIN_BASELINE_M of each other share a weight.
    places = []
    for ray, _ in rays:
        for place in places:
            if math.hypot(ray["x_m"] - place[0], ray["y_m"] - place[1]) < locate.MIN_BASELINE_M:
                place[2] += 1
                ray["_place"] = place
                break
        else:
            place = [ray["x_m"], ray["y_m"], 1]
            places.append(place)
            ray["_place"] = place
    if len(places) < 2:
        return None
    start = locate.best_fix([ray for ray, _ in rays])
    if start is None:
        return None
    x, y = float(start["x_m"]), float(start["y_m"])
    robust = [1.0] * len(rays)
    for _ in range(4):
        a11 = a12 = a22 = b1 = b2 = 0.0
        sigmas = []
        for k, (ray, row) in enumerate(rays):
            b = math.radians(ray["bearing_deg"])
            nx, ny = -math.sin(b), math.cos(b)
            r = max(math.hypot(x - ray["x_m"], y - ray["y_m"]), locate.MIN_RANGE_M)
            deg = MOVING_DEG if moving(row) else STILL_DEG
            s = math.hypot(r * math.tan(math.radians(deg)),
                           float(ray.get("origin_sigma_m") or 0.0))
            sigmas.append(s)
            w = robust[k] / (s * s) / ray["_place"][2]
            d = nx * ray["x_m"] + ny * ray["y_m"]
            a11 += w * nx * nx; a12 += w * nx * ny; a22 += w * ny * ny
            b1 += w * nx * d; b2 += w * ny * d
        det = a11 * a22 - a12 * a12
        if det <= 1e-12:
            return None
        x, y = (a22 * b1 - a12 * b2) / det, (a11 * b2 - a12 * b1) / det
        robust = []
        for (ray, _), s in zip(rays, sigmas):
            off = locate.cross_track_of(x, y, ray) / s
            robust.append(1.0 if off <= HUBER else HUBER / off)
    trace = (a11 + a22) / det
    return {"x_m": x, "y_m": y, "uncertainty_m": math.sqrt(trace)}


def ranged(rays_rows):
    """Depth where it agrees, bearings where it does not (fixed 2026-10-08, after the
    taped drive's ranges were seen to land within 0.03-0.27 m of the paintings, and
    before this was scored): start from `current`; every look that ranged and points
    at that placement within its allowance gives a point, its range laid flat at its
    elevation along its bearing. Two or more ranged viewpoints, or one ranged look,
    replace the position with the median point. The uncertainty is the larger of the
    points' spread (root mean square about the median) and their median own error
    -- the range's sigma and the bearing's sideways error at that range -- over the
    root of how many separate viewpoints ranged it."""
    base = current(rays_rows)
    if base is None:
        return None
    points = []
    for ray, row in rays_rows:
        rng = row.get("range_m")
        if rng is None or resolve._allowance_used(base, ray) is None:
            continue
        elevation = locate.elevation_of(ray) or 0.0
        flat = float(rng) * math.cos(math.radians(elevation))
        b = math.radians(ray["bearing_deg"])
        deg = MOVING_DEG if moving(row) else STILL_DEG
        own = math.hypot(float(row.get("range_sigma_m") or locate.RANGE_SIGMA_M),
                         flat * math.tan(math.radians(deg)))
        points.append((ray["x_m"] + flat * math.cos(b), ray["y_m"] + flat * math.sin(b),
                       own, (round(ray["x_m"] / locate.MIN_BASELINE_M),
                             round(ray["y_m"] / locate.MIN_BASELINE_M))))
    if not points:
        return base
    x = statistics.median(p[0] for p in points)
    y = statistics.median(p[1] for p in points)
    spread = math.sqrt(sum((p[0] - x) ** 2 + (p[1] - y) ** 2 for p in points) / len(points))
    places = len({p[3] for p in points})
    own = statistics.median(p[2] for p in points) / math.sqrt(places)
    return {"x_m": x, "y_m": y, "uncertainty_m": max(spread, own)}


def current(rays_rows):
    rays = [ray for ray, _ in rays_rows]
    best = locate.best_fix(rays)
    return None if best is None else locate.refine(best, rays)


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
    out = {"method": {"still_deg": STILL_DEG, "moving_deg": MOVING_DEG, "huber": HUBER},
           "targets": {}}
    for target, entities in TARGETS.items():
        ids = [r[0] for e in entities for r in accept.execute(
            "select id from observations where entity_id=?", (e,))]
        rows = [_readable(dict(r), vectors=True) for r in store.execute(
            "select * from observations where id in (%s) order by observed_at, id"
            % ",".join("?" * len(ids)), ids)]
        tx, ty = W2M(*TRUTH[target][:2])
        steps, seen = [], []
        for row in rows:
            ray = resolve.ray_of(row, None)
            if ray is None:
                continue
            seen.append((ray, row))
            looks = len({r["inference_id"] for _, r in seen})
            entry = {"looks": looks}
            for name, method in (("current", current), ("pooled", pooled), ("ranged", ranged)):
                got = method([(dict(r), w) for r, w in seen])
                if got:
                    error = math.hypot(got["x_m"] - tx, got["y_m"] - ty)
                    entry[name] = [round(error, 3), round(float(got["uncertainty_m"]), 3)]
            steps.append(entry)
        out["targets"][target] = steps
    summary = {}
    for name in ("current", "pooled", "ranged"):
        pairs = [s[name] for steps in out["targets"].values() for s in steps if name in s]
        final = [steps[-1][name] for steps in out["targets"].values() if steps and name in steps[-1]]
        summary[name] = {
            "states": len(pairs),
            "inside_share": round(sum(e <= u for e, u in pairs) / max(len(pairs), 1), 3),
            "median_error_m": round(statistics.median(e for e, _ in pairs), 3),
            "median_stated_m": round(statistics.median(u for _, u in pairs), 3),
            "final": [[round(e, 2), round(u, 2)] for e, u in final]}
    out["summary"] = summary
    a.output.write_text(json.dumps(out, indent=1) + "\n")
    for name, s in summary.items():
        print("%-8s %d states: tape inside the stated uncertainty %.0f%%; median off the tape "
              "%.2f m, median stated %.2f m; final (off, stated): %s"
              % (name, s["states"], 100 * s["inside_share"], s["median_error_m"],
                 s["median_stated_m"], s["final"]))


if __name__ == "__main__":
    main()
