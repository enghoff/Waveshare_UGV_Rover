"""A look aimed at a thing gives that thing the region it was aimed at.

**Why a look needs to say what it was for.** The autonomy executive drives to a
viewpoint facing a thing and looks, and until 2026-10-08 the look's regions were
then filed like any other -- by `resolve`, which knows nothing of the aim. Of the
157 aimed looks kept on record from 2026-10-02 to 10-06, the region at the aim
reached the thing it was aimed at in 5. Where the thing had duplicates the
region was ambiguous among them and waited, or joined another record of the same
object; where the thing sat behind low furniture, the lidar's reach hid it. See
docs/progress/2026-10-08-aimed-looks.md.

**The rule**, measured on those 157 looks before it was written here: a region is
eligible for the target when it has a bearing, points at the target's placement
within the resolver's own allowance with the map's reach left out (the reach is
what hides a painting behind chairs; the look was aimed at the thing, so it is
in view), agrees on height and, where ranged, on range, and looks like the
target wherever appearance can be asked (`resolve.DIFFERENT_THING`). The eligible
region using the least of its allowance is chosen; a near tie (`CLOSE_ALLOWANCE`)
must be broken by appearance (`APPEARANCE_LEAD`) or nothing is chosen. Reviewed
from the photographs, 16 of the 19 judgeable picks showed the target, 11 of them
regions the resolver had filed elsewhere. The three wrong picks all had targets
whose own records were mixed.

**What filing by aim does and does not do.** The region is attached to the
target and recorded in `aimed_looks`, and the target's placement is worked out
again (`resolve._replace_placement`, which takes aimed ranges as the position
and the claim -- see `aimed_claim`). It is *not* added to the target's exemplars:
a wrong pick that became one would make the target accept the next wrong look
more readily, which is the ratchet `appearance.appearance` exists to avoid.

**Records the region also fits.** A region that points at, and looks very like
(`resolve.RECOGNISED`), another record besides the target is evidence that the
two records are one object -- the painting of 2026-10-07 had eight. Those are
kept with the filing (`same_object_suspects`) so that the executive can cool them
off together and a person can be offered the merge. Nothing is merged here.
"""
from __future__ import annotations

import math
import statistics
from typing import Any

from . import locate

CLOSE_ALLOWANCE = 0.25
APPEARANCE_LEAD = 0.05
#: The bearing error a range's sideways error is worked out from, still and
#: moving: the median and tail of the misses measured on labelled looks on
#: 2026-10-07, the same figures `placement_calibration.py` was fixed with.
STILL_BEARING_DEG = 2.0
MOVING_BEARING_DEG = 4.5
#: The least an aimed range may claim. Single ranges on the taped paintings of
#: 2026-10-03 landed a median 0.16 to 0.28 m from the tape, and put through the
#: recorded aimed looks a claim without it fell to 0.04 m. Behind the dining
#: chairs it is not enough: there a range can measure the chair in front, and
#: three of six such filings claimed less than they were off by, with the floor
#: or without (docs/progress/2026-10-08-aimed-filing.md).
CLAIM_FLOOR_M = 0.20


def choose(store, target_id: str, placement: dict[str, Any],
           rays: list[tuple[dict[str, Any], dict[str, Any]]]
           ) -> tuple[dict[str, Any] | None, str]:
    """The region of one look that is the target, and why; (None, why) if none.

    `rays` are `(ray, observation)` pairs for the look's regions, built without
    the map's reach."""
    from . import resolve
    eligible = []
    for ray, observation in rays:
        used = resolve._allowance_used(placement, ray)
        if used is None:
            continue
        if not locate.stands_as_high(placement, ray):
            continue
        if not locate.stands_at_range(placement, ray):
            continue
        vector = observation.get("dino_blob") or b""
        seen = resolve.appearance(store, target_id, vector) if vector else None
        if seen is not None and seen < resolve.DIFFERENT_THING:
            continue
        eligible.append((used, seen, observation))
    if not eligible:
        return None, "no region of the look points at it"
    eligible.sort(key=lambda one: one[0])
    best = eligible[0]
    close = [one for one in eligible[1:] if one[0] - best[0] <= CLOSE_ALLOWANCE]
    if close:
        rivals = [one for one in [best, *close] if one[1] is not None]
        if len(rivals) != 1 + len(close):
            return None, "two regions point at it and appearance cannot be asked"
        rivals.sort(key=lambda one: -one[1])
        if rivals[0][1] - rivals[1][1] < APPEARANCE_LEAD:
            return None, "two regions point at it and look alike"
        best = rivals[0]
    return best[2], (f"points at it using {best[0]:.0%} of its allowance"
                     + ("" if best[1] is None else f", appearance {best[1]:.2f}"))


def also_fits(store, target_id: str, observation: dict[str, Any],
              ray: dict[str, Any], entities: list[dict[str, Any]]) -> list[str]:
    """Other records this region points at and looks very like."""
    from . import resolve
    vector = observation.get("dino_blob") or b""
    if not vector:
        return []
    found = []
    for entity in entities:
        if entity["id"] == target_id:
            continue
        placement = entity.get("placement") or {}
        if "x_m" not in placement:
            continue
        if resolve._allowance_used(placement, ray) is None:
            continue
        if not locate.stands_as_high(placement, ray):
            continue
        seen = resolve.appearance(store, entity["id"], vector)
        if seen is not None and seen >= resolve.RECOGNISED:
            found.append(entity["id"])
    return found


def file_by_aim(store, target_id: str, frame_id: str, reach=None) -> dict[str, Any]:
    """Give `target_id` the region of look `frame_id` that is aimed at it.

    Never raises: an aimed look that cannot be filed is still a recorded look,
    and the answer says why. `reach` is used only to refit the placement
    afterwards, as the resolver would.
    """
    from . import resolve
    try:
        session = store.map_session()
        target = next((e for e in store.placed(map_session=session)
                       if e["id"] == target_id), None)
        if target is None:
            return {"filed": None, "why": f"{target_id} is not placed in this map"}
        rows = store.observations(frame_id=frame_id, limit=64, vectors=True)
        rays = [(ray, row) for ray, row in
                ((resolve.ray_of(row, None), row) for row in rows) if ray]
        if not rays:
            return {"filed": None, "why": "no region of the look has a bearing"}
        chosen, why = choose(store, target_id, target["placement"], rays)
        if chosen is None:
            return {"filed": None, "why": why}
        was = chosen.get("entity_id")
        ray = next(r for r, row in rays if row["id"] == chosen["id"])
        suspects = also_fits(store, target_id, chosen, ray,
                             store.placed(map_session=session))
        if was and was != target_id and was not in suspects:
            suspects.append(was)
        store.attach(target_id, [chosen["id"]],
                     f"filed by aim at {target_id}: {why}")
        store.record_aimed(chosen["id"], target_id, session, suspects, was)
        before = (target["placement"] or {}).get("stated_uncertainty_m")
        resolve._replace_placement(store, target_id, session, reach)
        after = next((e for e in store.placed(map_session=session)
                      if e["id"] == target_id), {}).get("placement") or {}
        return {"filed": chosen["id"], "why": why, "was": was,
                "ranged": chosen.get("range_m") is not None,
                "same_object_suspects": suspects,
                "claim_before_m": before,
                "claim_after_m": after.get("stated_uncertainty_m")}
    except Exception as error:      # a look must never fail for its filing
        return {"filed": None, "why": f"{type(error).__name__}: {error}"}


def aimed_claim(rays: list[dict[str, Any]], aimed_ids: set[int],
                placement: dict[str, Any]) -> dict[str, Any] | None:
    """Position and claim from the aimed looks that ranged, or None.

    Each aimed ranged look that still points at the bearing-only placement gives
    a point: its range laid flat at its elevation, along its bearing. The median
    point is the position; the claim is the larger of the points' spread and
    their median own error -- the range's sigma and the bearing's sideways error
    at that range -- over the root of how many viewpoints ranged it. On the six
    taped targets of 2026-10-03 this rule, over eye-matched looks, put
    placements a median 0.11 m from the tape against 0.36 m from bearings, with
    the tape inside the claim 68% of the time. Only aimed looks count, because
    over every look a mixed record's ranges dragged the position back off
    (docs/progress/2026-10-08-depth-placement.md).
    """
    from . import resolve
    points = []
    for ray in rays:
        if ray.get("observation_id") not in aimed_ids or ray.get("range_m") is None:
            continue
        if resolve._allowance_used(placement, ray) is None:
            continue
        elevation = locate.elevation_of(ray) or 0.0
        flat = float(ray["range_m"]) * math.cos(math.radians(elevation))
        bearing = math.radians(float(ray["bearing_deg"]))
        moving = ((ray.get("origin_sigma_m") or 0.0) > 0.0
                  or (ray.get("bearing_sigma_deg") or 0.0) > locate.BEARING_SIGMA_DEG)
        own = math.hypot(float(ray.get("range_sigma_m") or locate.RANGE_SIGMA_M),
                         flat * math.tan(math.radians(
                             MOVING_BEARING_DEG if moving else STILL_BEARING_DEG)))
        points.append((float(ray["x_m"]) + flat * math.cos(bearing),
                       float(ray["y_m"]) + flat * math.sin(bearing), own,
                       (round(float(ray["x_m"]) / locate.MIN_BASELINE_M),
                        round(float(ray["y_m"]) / locate.MIN_BASELINE_M))))
    if not points:
        return None
    x = statistics.median(p[0] for p in points)
    y = statistics.median(p[1] for p in points)
    spread = math.sqrt(sum((p[0] - x) ** 2 + (p[1] - y) ** 2 for p in points)
                       / len(points))
    own = statistics.median(p[2] for p in points) / math.sqrt(len({p[3] for p in points}))
    return {"x_m": round(x, 3), "y_m": round(y, 3),
            "aimed_uncertainty_m": round(max(spread, own, CLAIM_FLOOR_M), 3),
            "aimed_ranges": len(points)}
