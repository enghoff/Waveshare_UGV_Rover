"""How likely a look at one of the world model's records is to land on it.

A look *lands* when the model files a region of its picture to the record it
was aimed at. Most do not: of 430 looks the rover took from 2026-10-05 to
10-18, judged on the model by replaying it over the history, 115 landed, and
what decided it most was how long ago the record was last seen -- 41% landed
under three hours, 2% after a day, because the model takes a picture into a
record unseen for three hours only on a move's likeness -- and then whether a
record at least as established stood within 0.8 m, which takes the picture
instead (experiments/m10_outcomes/README.md).

The planner used to assume half of them land. This is M10's predictor in its
place (docs/plans/autonomous-curiosity.md, M10): a logistic regression whose
numbers are in `landing.json` beside this file, fitted on every one of those
looks by experiments/m10_outcomes/predict.py and frozen there. **It only
re-ranks looks the planner would already offer**; it removes no veto and adds no
goal. Without its numbers -- the file missing, unreadable, or for other
features than these -- or without what a feature needs, the planner falls back
to the half it used before (`fallback`), which is M10's criterion 6.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

MODEL_PATH = Path(__file__).with_name("landing.json")
#: The features, in the order the numbers in `landing.json` are for. A file
#: fitted on any other list is not used.
FEATURES = ("log_visits", "spread_m", "rival", "crowd", "nearest_m", "range_m",
            "needs_movement", "log_since_h", "unseen_3h")
#: Another record within this is a neighbour: counted in `crowd`, and a rival
#: when it has been seen on at least as many visits.
RIVAL_M = 0.8
#: The nearest neighbour is counted no further away than this.
NEAREST_CAP_M = 2.0
#: The model's own line (world_state/visit_model/model.py, STALE_S).
STALE_H = 3.0

_cache: dict[str, Any] = {}


def model(path: Path = MODEL_PATH) -> dict[str, Any] | None:
    """The fitted numbers, or None when there are none fit to use."""
    key = str(path)
    if key in _cache:
        return _cache[key]
    found = None
    try:
        raw = json.loads(Path(path).read_text())
        n = len(FEATURES)
        if (tuple(raw.get("features") or ()) == FEATURES
                and all(len(raw.get(k) or ()) == n for k in ("mean", "scale", "coef"))
                and all(math.isfinite(float(v)) for k in ("mean", "scale", "coef")
                        for v in raw[k])
                and all(float(s) > 0.0 for s in raw["scale"])
                and math.isfinite(float(raw.get("intercept")))):
            found = dict(raw, digest=hashlib.sha256(Path(path).read_bytes()).hexdigest()[:12])
    except (OSError, ValueError, TypeError):
        found = None
    _cache[key] = found
    return found


def digest(path: Path = MODEL_PATH) -> str | None:
    """Which numbers a decision used: the first twelve hex digits of the
    file's SHA-256, or None when there are none fit to use."""
    m = model(path)
    return None if m is None else m["digest"]


def neighbours(entity: dict[str, Any], others: list[dict[str, Any]]) -> dict[str, Any]:
    """The records standing near this one: how many within RIVAL_M, whether one
    of them has been seen on at least as many visits, and the nearest."""
    place = entity.get("placement") or {}
    x, y = place.get("x_m"), place.get("y_m")
    visits = int(place.get("viewpoints") or 0)
    crowd, rival, nearest = 0, False, NEAREST_CAP_M
    if x is None or y is None:
        return {"crowd": 0, "rival": False, "nearest_m": NEAREST_CAP_M}
    me = entity.get("id")
    for other in others:
        if other is entity or other.get("id") == me or other.get("kind") != "record":
            continue
        op = other.get("placement") or {}
        if op.get("x_m") is None or op.get("y_m") is None:
            continue
        d = math.hypot(float(op["x_m"]) - float(x), float(op["y_m"]) - float(y))
        nearest = min(nearest, d)
        if d <= RIVAL_M:
            crowd += 1
            rival = rival or int(op.get("viewpoints") or 0) >= visits
    return {"crowd": crowd, "rival": rival, "nearest_m": nearest}


def lands(entity: dict[str, Any], near: dict[str, Any], now: float, range_m: float,
          needs_movement: bool, fallback: float,
          path: Path = MODEL_PATH) -> tuple[float, str]:
    """The chance a look from `range_m` lands on `entity`, and where the number
    came from: "learned", or "fallback" with the planner's old half."""
    m = model(path)
    place = entity.get("placement") or {}
    seen = entity.get("last_seen_at")
    visits = place.get("viewpoints")
    spread = place.get("uncertainty_m")
    if m is None or seen is None or visits is None or spread is None:
        return fallback, "fallback"
    since_h = max(0.0, (float(now) - float(seen)) / 3600.0)
    x = (math.log1p(int(visits)), float(spread), float(bool(near["rival"])),
         float(near["crowd"]), float(near["nearest_m"]), float(range_m),
         float(bool(needs_movement)), math.log1p(since_h), float(since_h >= STALE_H))
    z = float(m["intercept"]) + sum(
        float(c) * (v - float(mu)) / float(s)
        for c, v, mu, s in zip(m["coef"], x, m["mean"], m["scale"]))
    return 1.0 / (1.0 + math.exp(-z)), "learned"
