"""Which uncertain things are worth going to check, and where to check them from.

M0a's goal type ([R-AUT-12](../docs/requirements/autonomy.md#r-aut-12)), and a
pure function of a `Situation` like everything in `goals.py`.

**Every placed thing is a hypothesis.** A placement is where some looks crossed,
and the resolver can be wrong about which looks belong together: on the labelled
drive of 2026-09-08, 17 of 76 things held looks at two different objects, and a
crossing made of two objects' bearings can land where neither of them is. So the
claim an inspection tests is the plainest one the placement makes -- *something
stands here, to within this much* -- and the question is whether a look from a
place chosen on the map, not on the claim, finds depth there.

**Identity is not the question, and that is deliberate.** One look cannot say
which thing it is looking at; `world_state/hypothesis_check.py` gives the
measurement. A supported claim says something is there and nothing about what,
and no goal here or anywhere in this component acts on which thing it was.

## What a request carries

The claim and its alternatives, the question, a viewpoint validated against the
map rather than the claim, finite travel, time and attempt limits, and the case
the attempt is spent against. The case is the *place*: two requests within
`CASE_RADIUS_M` of each other on the same map session are one case whatever the
resolver calls the thing, so a merge, a rename or a regenerated goal lands on
the attempts already spent. The source looks are added by the executive when it
plans, from the world state as it is then.

## What it will not propose

A claim placed looser than `LOOSEST_M`, because one look cannot test it. A thing
placed on another map session. A place already answered, or one whose attempts
are spent: those come back as candidates so that the refusal is in the record,
and `scoring.py` vetoes them. And nothing at all unless the M0a protocol is
running, which `scoring.py` also decides.
"""
from __future__ import annotations

import math
from typing import Any

import goals as goals_mod
import mapgrid
import refs
from situation import Situation

GOAL_TYPE = "inspect_hypothesis"

#: How many claims are worth considering in one pass, nearest first. A bound on
#: work, like `goals.ENTITY_LIMIT`.
CLAIM_LIMIT = 12

#: How far from the place the look is taken. Nearer than a metre and the patch
#: of depth that must clear the place's uncertainty fills most of the depth
#: camera's view. Further than 1.6 m and the range is too coarse to place a
#: point to the thing's half-width, which the check requires of a look before it
#: may confirm anything (`hypothesis_check.THING_M`): on still looks of
#: 2026-10-01 the range's error was 0.07 m in the middle under 1.5 m, 0.08 m
#: from 1.5 to 2.5 m and 0.13 m beyond, and twice it has to stay under 0.15 m.
NEAR_M = 1.0
FAR_M = 1.6

#: The loosest claim one look can test, the same figure the check refuses past
#: (`hypothesis_check.LOOSEST_M`).
LOOSEST_M = 0.5

#: The depth camera's field of view, from its own calibration of 2026-09-30
#: (fx 500.3, fy 500.2 on a 640 by 360 picture), and how far inside its edge a
#: patch must stay. At twenty degrees up it sees from level to forty up, and
#: nothing below the camera's own height -- which is why a look may be level.
DEPTH_HFOV_DEG = 65.2
DEPTH_VFOV_DEG = 39.6
EDGE_DEG = 3.0

#: The slack the check puts round a place, so that the viewpoint is chosen to
#: fit the patch the check will read (`hypothesis_check.PATCH_SLACK_M` and
#: `POINTING_DEG`), and what a claim with no height is given for one.
PATCH_SLACK_M = 0.15
POINTING_DEG = 3.0
UNKNOWN_HEIGHT_SIGMA_M = 0.5

#: The tilts a look may be taken at, rest first because rest is where the
#: rover's own looks are taken from: `permission.INSPECTION_TILTS`.
TILTS = (20.0, 0.0)

#: What one attempt may spend. One retry, and an attempt's travel is the walk
#: there with half again for a route that is longer than its estimate, plus a
#: metre; its time is twice the estimate plus half a minute for the look and the
#: check. Both are also held under the daemon's ceilings
#: (`permission.INSPECTION_MAX_*`), which refuse a request that asks for more.
ATTEMPTS = 2
TRAVEL_FACTOR = 1.5
TRAVEL_SLACK_M = 1.0
TIME_SLACK_S = 30.0
MAX_TRAVEL_M = 8.0
MAX_S = 180.0

#: Two claims are one case within this distance on the same map session,
#: whatever the resolver calls them (`permission.CASE_RADIUS_M`).
CASE_RADIUS_M = 0.5

#: How close to the place a viewpoint may be, on top of the claim's own
#: uncertainty: the rover's footprint and then some
#: (`permission.INSPECTION_STANDOFF_M`).
STANDOFF_M = 0.5

ANSWERED = ("supported", "contradicted")


def generate(situation: Situation) -> list[goals_mod.Candidate]:
    """One inspection per claim worth testing, from the cheapest place that fits."""
    reach, where = situation.reach, situation.where
    if reach is None or where is None:
        return []
    generation = (situation.world_generation
                  if situation.world_generation != refs.UNKNOWN else None)
    claims = []
    for entity in situation.entities:
        claim = _claim_of(entity, situation.map_session)
        if claim is None:
            continue
        gap = math.hypot(claim["x_m"] - where[0], claim["y_m"] - where[1])
        claims.append((round(gap, 3), str(entity.get("id") or ""), entity, claim))
    claims.sort(key=lambda one: (one[0], one[1]))
    out = []
    for _gap, entity_id, entity, claim in claims[:CLAIM_LIMIT]:
        found = _candidate(situation, reach, entity_id, entity, claim, generation)
        if found is not None:
            out.append(found)
    return out


def case_of(claim: dict[str, Any], map_session: int | None) -> str:
    """The name a case is recorded under. Matching is by place; this is a label."""
    return (f"case/{map_session}/{float(claim['x_m']):.2f},"
            f"{float(claim['y_m']):.2f}")


def history(situation: Situation, claim: dict[str, Any]) -> list[dict[str, Any]]:
    """The attempts already spent on this place, oldest first."""
    session = situation.map_session
    out = []
    for one in situation.inspections:
        target = one.get("target") or {}
        if target.get("x_m") is None or target.get("y_m") is None:
            continue
        if session is not None and one.get("map_session") not in (None, session):
            continue
        gap = math.hypot(float(target["x_m"]) - claim["x_m"],
                         float(target["y_m"]) - claim["y_m"])
        if gap <= CASE_RADIUS_M:
            out.append(one)
    out.sort(key=lambda one: float(one.get("at") or 0.0))
    return out


def _claim_of(entity: dict[str, Any], session: int | None) -> dict[str, Any] | None:
    placement = entity.get("placement") or {}
    if placement.get("x_m") is None or placement.get("y_m") is None:
        return None
    if session is not None and entity.get("placement_map_session") not in (None,
                                                                           session):
        return None
    # What the rover claims for the placement, which for a thing placed from a
    # single look is wider than the tolerance the resolver matches with (see
    # world_state/locate.py, STATED_SINGLE_LOOK_M). Older placements lack it.
    uncertainty = placement.get("stated_uncertainty_m")
    if uncertainty is None:
        uncertainty = placement.get("uncertainty_m")
    if uncertainty is None:
        uncertainty = entity.get("placement_uncertainty_m")
    claim = {"x_m": round(float(placement["x_m"]), 3),
             "y_m": round(float(placement["y_m"]), 3),
             "uncertainty_m": None if uncertainty is None
             else round(float(uncertainty), 3)}
    for name in ("height_m", "height_sigma_m"):
        if placement.get(name) is not None:
            claim[name] = round(float(placement[name]), 3)
    return claim


def _candidate(situation: Situation, reach: mapgrid.Reach, entity_id: str,
               entity: dict[str, Any], claim: dict[str, Any],
               generation: str | None) -> goals_mod.Candidate | None:
    sigma = claim.get("uncertainty_m")
    if sigma is None or sigma > LOOSEST_M:
        return None
    x, y = claim["x_m"], claim["y_m"]
    near = max(NEAR_M, sigma + STANDOFF_M)
    view = None
    for view_x, view_y, walk_m in reach.ring(x, y, near, FAR_M):
        if not reach.is_free(view_x, view_y):
            continue
        if not reach.clear_line(view_x, view_y, x, y):
            continue
        fits = _fits(claim, math.hypot(x - view_x, y - view_y))
        if fits is None:
            continue
        # Nearest walk first, so the first viewpoint the whole patch fits from
        # is the cheapest one; a viewpoint where only the middle of the place is
        # in view is kept in reserve for a claim no whole-patch view exists for.
        if fits["patch_fits"]:
            view = (view_x, view_y, walk_m, fits)
            break
        if view is None:
            view = (view_x, view_y, walk_m, fits)
    if view is None:
        return None
    view_x, view_y, walk_m, fits = view
    range_m = math.hypot(x - view_x, y - view_y)
    heading = math.degrees(math.atan2(y - view_y, x - view_x))

    spent = history(situation, claim)
    answered = next((one for one in reversed(spent)
                     if one.get("outcome") in ANSWERED), None)
    doubt = 1.0 if not spent else 0.5
    gain = doubt if fits["patch_fits"] else doubt * 0.5
    travel = min(MAX_TRAVEL_M, TRAVEL_FACTOR * walk_m + TRAVEL_SLACK_M)
    time_s = (goals_mod.GOAL_OVERHEAD_S + walk_m / goals_mod.SPEED_MS
              + goals_mod.LOOK_S)
    seconds = min(MAX_S, 2.0 * time_s + TIME_SLACK_S)
    case = case_of(claim, situation.map_session)
    looks = entity.get("observation_count")
    question = (f"seen from ({view_x:.2f}, {view_y:.2f}), {range_m:.1f} m away "
                f"at tilt {fits['tilt_deg']:.0f}, does depth find a surface "
                f"where {entity_id}'s looks crossed, with a region ranged on it?")
    return goals_mod.Candidate(
        id=f"{GOAL_TYPE}:{entity_id}@{view_x:.2f},{view_y:.2f}",
        type=GOAL_TYPE, target=entity_id,
        refs=[refs.world(generation, entity_id)] if entity_id else (),
        why=(f"{entity_id} is believed to stand at ({x:.2f}, {y:.2f}) to "
             f"{sigma:.2f} m" + (f" from {looks} looks" if looks else "")
             + ("; nothing has checked that anything is there" if not spent
                else f"; {len(spent)} attempt(s) so far, none answered")),
        expects=(f"a look from ({view_x:.2f}, {view_y:.2f}) facing "
                 f"{heading:.0f} degrees at tilt {fits['tilt_deg']:.0f}, "
                 f"{range_m:.1f} m from the place"),
        action=[{"call": "drive_to", "params": {"x_m": round(view_x, 3),
                                                "y_m": round(view_y, 3),
                                                "heading_deg": round(heading, 1)}},
                {"call": "world_inspect",
                 "params": {"tilt_deg": fits["tilt_deg"], "fresh": True,
                            "keep_depth": True}},
                {"call": "world_state_check", "params": {"claim": claim}}],
        travel_m=walk_m, time_s=time_s,
        energy_wh=time_s * goals_mod.NOMINAL_DRAW_W / 3600.0,
        risk="drives_on_mapped_floor",
        gain_kind="hypothesis_doubt", gain_value=gain,
        gain_detail={
            "claim": claim,
            "claimed_by": entity_id,
            "question": question,
            "alternatives": [
                "nothing stands there: the looks crossed by coincidence, "
                "possibly being looks at different things",
                "something stands nearer or further along the line the looks "
                "came from",
                "it is there, and this look cannot see it: occluded, outside "
                "the depth camera's view, or too dark for depth"],
            "evidence_needed": (
                "supported: a region on the line ranged where the place is; "
                "contradicted: depth measured past the place across its whole "
                "uncertainty; anything else is unresolved"),
            "case": case,
            "attempts_so_far": len(spent),
            "outcomes_so_far": [one.get("outcome") for one in spent],
            "doubt": doubt,
            **fits,
        },
        constraints={
            "reachable_m": walk_m,
            "on_free_floor": reach.is_free(view_x, view_y),
            "needs_movement": True,
            "needs_depth_camera": True,
            "range_m": round(range_m, 3),
            "in_certified_band": NEAR_M <= range_m <= FAR_M,
            "goal": {"x_m": round(view_x, 3), "y_m": round(view_y, 3),
                     "heading_deg": round(heading, 1)},
            "tilt_deg": fits["tilt_deg"],
            "case": case,
            "case_attempts": len(spent),
            "case_answered": None if answered is None else answered.get("outcome"),
            "inspection": {"case": case,
                           "target": {"x_m": x, "y_m": y},
                           "limits": {"travel_m": round(travel, 2),
                                      "seconds": round(seconds, 1),
                                      "attempts": ATTEMPTS}},
        })


def _fits(claim: dict[str, Any], range_m: float) -> dict[str, Any] | None:
    """Which tilt shows the place from this far away, and whether all of it.

    The whole patch the check will read has to be inside the depth camera's
    picture for a contradiction to be possible; the middle of it alone still
    lets a ranged region support the claim. None when not even the middle of the
    place would be in view at either tilt.
    """
    sigma = float(claim.get("uncertainty_m") or 0.0)
    height = claim.get("height_m")
    height_sigma = (UNKNOWN_HEIGHT_SIGMA_M if height is None
                    else float(claim.get("height_sigma_m") or 0.0))
    distance = math.hypot(range_m, float(height or 0.0))
    pointing = POINTING_DEG
    across = math.degrees(math.atan2(sigma + PATCH_SLACK_M, distance)) + pointing
    rise = math.degrees(math.atan2(height_sigma + PATCH_SLACK_M, distance)) + pointing
    elevation = math.degrees(math.atan2(float(height or 0.0), max(range_m, 0.05)))
    half_h = DEPTH_HFOV_DEG / 2.0 - EDGE_DEG
    half_v = DEPTH_VFOV_DEG / 2.0 - EDGE_DEG
    for tilt in TILTS:
        off = abs(elevation - tilt)
        if off + rise <= half_v and across <= half_h:
            return {"tilt_deg": tilt, "patch_fits": True,
                    "elevation_deg": round(elevation, 1)}
    # **When neither tilt fits the whole patch, the one nearer the place.** It
    # used to be the first in the list that had the middle in view, which is
    # twenty up: on 2026-10-02 that put a chair seat 0.19 m above the camera
    # 14 degrees below the middle of the picture with its patch off the bottom
    # edge, where level had it 6 degrees above, and three checks came back
    # "outside the depth camera's view".
    nearest = min(TILTS, key=lambda tilt: abs(elevation - tilt))
    if abs(elevation - nearest) > half_v:
        return None
    return {"tilt_deg": nearest, "patch_fits": False,
            "elevation_deg": round(elevation, 1)}
