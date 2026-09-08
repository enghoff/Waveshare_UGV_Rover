"""What the rover could usefully do next, and what each one would cost it.

**Everything here is a pure function of a `Situation`.** Nothing in this module
reads the rover, the clock or a file, so the same reading produces the same
candidates in the same order for ever -- which is what makes a decision built on
them replayable, and it is why the situation is read once and passed in rather
than fetched where it is needed.

Two kinds of goal, which is what Milestone M2 enables:

**`explore_frontier`** -- go and stand where the map stops, so the scanner sees
past it. The choosing is `frontier.py`'s, the same module `explore` ranks with,
reached through [`mapgrid.py`](mapgrid.py); what this file adds is the estimate
of how much new floor standing there would actually buy.

**`improve_geometry`** -- go and look at a thing the rover has already placed,
standing somewhere its position would come out better. A bearing pins it across
the line of sight and says nothing along it, so two looks from nearly the same
place leave the same long thin uncertainty however many times they are repeated:
the useful viewpoint is the one whose ray crosses the error ellipse's long axis.
That is the arithmetic in `_from_viewpoint`, and it is what makes this a goal
rather than a wish.

## What it will not propose

**A thing that has never been placed.** One bearing gives a direction and no
position, so there is nowhere to plan a viewpoint *around*; what that thing
needs is for the rover to be somewhere else entirely, which is what exploring
does. Placed-but-never-ranged is a different case and is covered: the position
exists, and the missing distance is exactly what a viewpoint inside the depth
camera's band would supply.

**Anything the rover would have to invent.** Every estimate below is built from
a number the rover measured or a constant this file declares out loud with the
measurement behind it. Where the measurement does not exist -- the energy a
drive costs, the tilt needed to see a thing whose height is uncertain -- the
estimate says so and the scorer charges for the ignorance rather than guessing
past it.
"""
from __future__ import annotations

import math
from typing import Any, Iterable

import mapgrid
import refs
from situation import Situation

#: How many frontiers and how many things are worth ranking in one pass. Both
#: are bounds on work rather than policy: the survey returns everywhere worth
#: driving to and the world state holds over a hundred things, and scoring all
#: of them would cost seconds of a rover that is deciding every minute. The
#: orderings they cut are `frontier.py`'s own cost and the plainest measure of
#: how badly a thing is placed, so what is dropped is the tail of both.
FRONTIER_LIMIT = 8
ENTITY_LIMIT = 12

#: How many places to stand are considered per thing, and how many become
#: candidates. The ring is walked in a fixed order and the two kept are the best
#: prediction and the cheapest one worth having -- so the scorer is given a real
#: choice between a good viewpoint far away and an adequate one nearby, which is
#: the trade it exists to make.
VIEWPOINTS_CONSIDERED = 60
VIEWPOINTS_KEPT = 2

#: The band the rover's geometry is certified in, from the acceptance run of
#: 2026-09-08: 0.5 to 2.5 m, where the targets can be detected and tape-measured
#: and where 61-71% of this rover's looks at placed things fall. A viewpoint
#: outside it is not refused here -- it is recorded with the fact, and
#: `scoring.py` vetoes it, so that the reason appears in the record instead of
#: the candidate quietly never existing.
BAND_NEAR_M = 0.5
BAND_FAR_M = 2.5

#: How far out places to stand are looked for, which is wider than the band on
#: purpose. **A thing with no viewpoint inside the certified band should say so
#: rather than vanish**: if the search stopped at the band, a thing the rover
#: can only get within three metres of would produce no candidate at all, and
#: the record would not distinguish that from a thing nobody wanted to look at.
#: So the search is wider, viewpoints inside the band are preferred whenever
#: there are any, and what comes back from outside it is refused by
#: `scoring.py` with the distance in the sentence.
SEARCH_NEAR_M = 0.3
SEARCH_FAR_M = 4.0

#: What one bearing is worth believing to, in degrees. The resolver is told 1.5
#: and R-WS-10 is `failing` because a driven recording put half of them outside
#: that; 2.3 is what the same bench measured for a look reached from the
#: descending side of the pan servo's backlash. The larger of the two is used
#: here deliberately: an estimate of what a new look would buy should be the
#: pessimistic one until the acceptance measurement is retaken.
BEARING_SIGMA_DEG = 2.3

#: Where a measured distance lands, from the acceptance run: 69% of ranges
#: within 0.5 m of the object. So a first range collapses the along-ray
#: uncertainty to about this and no further, and a thing already placed better
#: than this has nothing to gain from being ranged.
RANGE_LANDS_WITHIN_M = 0.5

#: What the rover drives at, and what a goal costs before the wheels turn.
#: 0.35 m/s is `nav_limits.DEFAULT_SPEED_MS`, the speed a goal is asked for; a
#: real route averages less, so a time estimate built on it is optimistic and is
#: charged as such. The overhead is the planning phase, the turn onto the route
#: and the stop at the end -- seconds rather than tenths, and enough of them
#: that a two-metre goal is mostly overhead.
SPEED_MS = 0.35
GOAL_OVERHEAD_S = 8.0

#: A look, once the rover is standing still: the gimbal settling and the capture
#: through the encoders. The capture is 0.45 s measured on the Orin; the settle
#: is what dominates, and this is the figure `centre_gimbal` waits.
LOOK_S = 2.0

#: What the rover draws while it does any of that, in watts. **Unmeasured.**
#: There is no current sense on this chassis -- the driver board reports pack
#: voltage and nothing else -- so this is a nominal figure for the host, the
#: lidar, the depth camera and two motors, and every energy estimate below is
#: therefore a restatement of time. `scoring.py` weights it at zero for exactly
#: that reason, and keeps the term so that a rover which can one day measure its
#: own current has somewhere to put the answer.
NOMINAL_DRAW_W = 15.0

#: What a thing with no measured distance is treated as being placed to, in
#: metres, when its own store carries no error figure. A declared stand-in
#: rather than a measurement, and it only ever makes a candidate look *less*
#: attractive than an entity with a real ellipse: the gain from ranging it is
#: capped by this and not by an optimistic guess.
UNKNOWN_PLACEMENT_M = 1.0

#: How well a thing can be placed at all, in metres, however good the crossing.
#: The arithmetic below knows only about bearings: it does not know that the
#: rover's own place on the map is uncertain, that the thing has width, or that
#: a box drawn round it moves between frames. All three are floors under the
#: answer, and without one the model cheerfully predicts a centimetre from a
#: viewpoint half a metre away. A tenth of a metre is what the acceptance run
#: actually achieved on its best pair across the floor -- 0.098 m and 0.144 m on
#' two tape-measured separations -- and nothing this rover has ever done is
#: better than that.
PLACEMENT_FLOOR_M = 0.10

#: How uncertain a thing's height may be before the tilt needed to look at it
#: cannot be predicted. Above it the candidate is marked `tilt_unknown` and the
#: scorer discounts it, because the validated envelope covers tilt zero and
#: tilt +20 and nothing says which one this look would need.
HEIGHT_SIGMA_LIMIT_M = 0.50


class Candidate:
    """One thing the rover could do, with everything a decision needs about it.

    The fields are the architecture's list for a candidate goal, and they are
    kept separate on purpose: what it is (`type`, `target`), what it would do
    (`action`, `expects`), what it would cost (`travel_m`, `time_s`,
    `energy_wh`), what it would buy (`gain_kind`, `gain_value`, `gain_detail`),
    what could refuse it (`constraints`) and how dangerous it is (`risk`). The
    score is not here at all -- `scoring.py` produces it from these, and keeping
    them apart is what stops a generator quietly deciding what should win.
    """

    __slots__ = ("id", "type", "target", "refs", "why", "expects", "action",
                 "travel_m", "time_s", "energy_wh", "risk", "gain_kind",
                 "gain_value", "gain_detail", "constraints")

    def __init__(self, *, id: str, type: str, why: str, expects: str,
                 action: list[dict[str, Any]], gain_kind: str,
                 gain_value: float, target: str = "",
                 refs: Iterable[str] = (), travel_m: float = 0.0,
                 time_s: float = 0.0, energy_wh: float = 0.0,
                 risk: str = "stationary",
                 gain_detail: dict[str, Any] | None = None,
                 constraints: dict[str, Any] | None = None) -> None:
        self.id = id
        self.type = type
        self.target = target
        self.refs = tuple(refs)
        self.why = why
        self.expects = expects
        self.action = list(action)
        self.travel_m = float(travel_m)
        self.time_s = float(time_s)
        self.energy_wh = float(energy_wh)
        self.risk = risk
        self.gain_kind = gain_kind
        self.gain_value = float(gain_value)
        self.gain_detail = dict(gain_detail or {})
        self.constraints = dict(constraints or {})

    def as_dict(self) -> dict[str, Any]:
        return {"id": self.id, "type": self.type, "target": self.target,
                "refs": list(self.refs), "why": self.why,
                "expects": self.expects, "action": self.action,
                "travel_m": round(self.travel_m, 2),
                "time_s": round(self.time_s, 1),
                "energy_wh": round(self.energy_wh, 4), "risk": self.risk,
                "gain_kind": self.gain_kind,
                "gain_value": round(self.gain_value, 4),
                "gain_detail": self.gain_detail,
                "constraints": self.constraints}

    def __repr__(self) -> str:                                 # pragma: no cover
        return f"<Candidate {self.id} {self.gain_kind}={self.gain_value:.3f}>"


def generate(situation: Situation) -> list[Candidate]:
    """Everything worth considering, in a fixed order.

    Order is by type and then by the generator's own ranking, and it is fixed so
    that two runs over one situation produce the same list -- the scorer sorts by
    score afterwards, and a stable input is what makes a tie break the same way
    twice.
    """
    return [*explore_frontier(situation), *improve_geometry(situation)]


# --- going where the map stops ----------------------------------------------

def explore_frontier(situation: Situation) -> list[Candidate]:
    """Places to stand where the scanner would see ground nobody has mapped."""
    reach, grid, where = situation.reach, situation.grid, situation.where
    if reach is None or grid is None or where is None:
        return []
    found, summary = mapgrid.frontiers(grid, where)
    out: list[Candidate] = []
    for one in found[:FRONTIER_LIMIT]:
        x, y = float(one["x"]), float(one["y"])
        # Two estimates of what standing there reveals, and the smaller wins.
        # Boundary times sensing depth is what the scanner sweeps through the
        # gap; the unknown ground actually within reach is what there is to
        # sweep. A one-cell frontier in front of half a house is the first
        # number; a wide frontier onto a cupboard is the second.
        swept = float(one["size_m"]) * mapgrid.SENSE_DEPTH_M
        there = reach.unknown_area_m2(x, y)
        area = min(swept, there)
        travel = float(one["distance_m"])
        time_s = GOAL_OVERHEAD_S + travel / SPEED_MS
        out.append(Candidate(
            id=f"explore_frontier@{x:.2f},{y:.2f}",
            type="explore_frontier",
            why=(f"{one['size_m']:.1f} m of the map's edge is "
                 f"{travel:.1f} m away, with about {area:.0f} m2 of unmapped "
                 f"floor behind it"),
            expects=(f"a scan from ({x:.2f}, {y:.2f}) facing "
                     f"{math.degrees(float(one['yaw'])):.0f} degrees, turning "
                     f"unknown ground into floor or wall"),
            action=[{"call": "drive_to", "params": {"x_m": round(x, 3),
                                                    "y_m": round(y, 3)}}],
            travel_m=travel, time_s=time_s,
            energy_wh=time_s * NOMINAL_DRAW_W / 3600.0,
            # The map's edge is the one place this rover cannot see what it is
            # driving into: the lidar's plane is 20 cm up and there is no drop
            # sensing, so a frontier at the top of a staircase looks exactly
            # like a frontier at a doorway. Named on every candidate of this
            # type rather than argued about later.
            risk="drives_onto_unmapped_ground",
            gain_kind="unknown_floor_m2", gain_value=area,
            gain_detail={"boundary_m": one["size_m"],
                         "swept_m2": round(swept, 1),
                         "unknown_within_reach_m2": round(there, 1),
                         "frontier_cells": one["cells"],
                         "unknown_share": mapgrid.unknown_share(summary),
                         "survey_cost": one["cost"]},
            constraints={"reachable_m": travel,
                         "on_free_floor": reach.is_free(x, y),
                         "needs_movement": True,
                         "goal": {"x_m": round(x, 3), "y_m": round(y, 3)}}))
    return out


# --- going where a thing would come out better ------------------------------

def improve_geometry(situation: Situation) -> list[Candidate]:
    """Viewpoints that would sharpen where a thing the rover knows actually is."""
    reach = situation.reach
    if reach is None:
        return []
    generation = (situation.world_generation
                  if situation.world_generation != refs.UNKNOWN else None)
    out: list[Candidate] = []
    for entity in _worth_looking_at(situation)[:ENTITY_LIMIT]:
        out.extend(_viewpoints(situation, reach, entity, generation))
    return out


def _worth_looking_at(situation: Situation) -> list[dict[str, Any]]:
    """The placed things with the most to gain, worst first.

    Placed, because a viewpoint has to be planned around a position and a thing
    seen once has none. In this map session, because a placement from a map that
    has since been replaced names coordinates that no longer mean anything --
    the world state keeps those things for recognition and not for driving to.
    """
    session = situation.map_session
    wanted: list[tuple[float, str, dict]] = []
    for entity in situation.entities:
        placement = entity.get("placement") or {}
        if placement.get("x_m") is None or placement.get("y_m") is None:
            continue
        if (session is not None
                and entity.get("placement_map_session") not in (None, session)):
            continue
        wanted.append((-_room_to_improve(entity), str(entity.get("id") or ""),
                       entity))
    wanted.sort(key=lambda one: (one[0], one[1]))
    return [one[2] for one in wanted]


def _room_to_improve(entity: dict[str, Any]) -> float:
    """How badly this thing is placed, in metres, whatever the reason.

    One number for two different faults so that they can be ranked against each
    other: a thing whose bearings cross badly carries its own error figure, and
    a thing nothing has ever measured the distance to is treated as no better
    than a declared stand-in until one does.
    """
    placement = entity.get("placement") or {}
    error = placement.get("error_major_m")
    if error is None:
        error = placement.get("uncertainty_m")
    error = UNKNOWN_PLACEMENT_M if error is None else float(error)
    ranging = entity.get("ranging") or {}
    if ranging.get("never_ranged"):
        error = max(error, RANGE_LANDS_WITHIN_M)
    return error


def _viewpoints(situation: Situation, reach: mapgrid.Reach,
                entity: dict[str, Any], generation: str | None
                ) -> list[Candidate]:
    """Up to two places to stand and look at one thing.

    The best prediction and the cheapest viewpoint worth having, which are
    usually not the same place: the ray that crosses the error ellipse squarely
    is often across the room, and the one at the rover's feet buys half as much
    for a tenth of the walk. Both are offered and the scorer decides -- that
    trade is exactly what it exists to make, and a generator that made it here
    would be a scorer nobody could see.
    """
    placement = entity.get("placement") or {}
    x, y = float(placement["x_m"]), float(placement["y_m"])
    entity_id = str(entity.get("id") or "")
    ranging = entity.get("ranging") or {}

    # **The band is applied before the bound on work, not after.** The ring
    # comes back nearest-walk first, so a thing across the room has its sixty
    # nearest places to stand all at the far end of the search -- three and four
    # metres away, outside the band -- and cutting the list first would leave
    # every viewpoint that would actually have worked on the floor. The
    # scenarios caught this; the ordering here is the fix.
    whole = reach.ring(x, y, SEARCH_NEAR_M, SEARCH_FAR_M)
    in_band = [one for one in whole if BAND_NEAR_M <= _gap(one, x, y) <= BAND_FAR_M]
    ring = (in_band or whole)[:VIEWPOINTS_CONSIDERED]
    predicted, outside = [], []
    usable = blocked = 0
    for view_x, view_y, walk_m in ring:
        if not reach.clear_line(view_x, view_y, x, y):
            blocked += 1
            continue
        usable += 1
        got = _from_viewpoint(placement, entity, view_x, view_y)
        if got["gain_m"] <= 0.0:
            continue
        if BAND_NEAR_M <= got["range_m"] <= BAND_FAR_M:
            predicted.append((got, view_x, view_y, walk_m))
        else:
            outside.append((got, view_x, view_y, walk_m))
    # Only when there is nowhere inside the band to stand, so that the refusal
    # appears in the record instead of the thing quietly not being considered.
    predicted = predicted or outside
    if not predicted:
        # Two different answers, and they must not read alike. Somewhere to
        # stand and nothing to gain is a thing that is already placed as well as
        # this rover can place it, and it should simply not be proposed.
        # Nowhere to stand at all is a thing worth looking at that cannot be
        # looked at, and that is worth saying out loud.
        if usable:
            return []
        return _nowhere_to_stand(entity, entity_id, generation, x, y,
                                 len(ring), blocked)

    best = max(predicted, key=lambda one: (round(one[0]["gain_m"], 4),
                                           -round(one[3], 3)))
    cheap = min(predicted, key=lambda one: (round(one[3], 3),
                                            -round(one[0]["gain_m"], 4)))
    keep = [best] if cheap is best else [best, cheap]
    # Anything the cheap one is barely worth: half the best prediction is the
    # line, and below it the near viewpoint is a look that leaves the thing as
    # badly placed as it found it.
    keep = [one for one in keep
            if one[0]["gain_m"] >= 0.5 * best[0]["gain_m"]][:VIEWPOINTS_KEPT]

    out = []
    for got, view_x, view_y, walk_m in keep:
        time_s = GOAL_OVERHEAD_S + walk_m / SPEED_MS + LOOK_S
        out.append(Candidate(
            id=f"improve_geometry:{entity_id}@{view_x:.2f},{view_y:.2f}",
            type="improve_geometry", target=entity_id,
            refs=[refs.world(generation, entity_id)] if entity_id else (),
            why=got["why"],
            expects=(f"a look from ({view_x:.2f}, {view_y:.2f}), "
                     f"{got['range_m']:.1f} m from the thing, crossing the "
                     f"present uncertainty at {got['crossing_deg']:.0f} degrees"),
            action=[{"call": "drive_to", "params": {"x_m": round(view_x, 3),
                                                    "y_m": round(view_y, 3)}},
                    {"call": "look_at", "params": {"entity": entity_id}}],
            travel_m=walk_m, time_s=time_s,
            energy_wh=time_s * NOMINAL_DRAW_W / 3600.0,
            # It drives to somewhere the map already calls free floor and looks:
            # the whole route is over ground the scanner has painted, which is
            # what makes this the milder of the two risk classes.
            risk="drives_on_mapped_floor",
            gain_kind="placement_uncertainty_m", gain_value=got["gain_m"],
            gain_detail=got,
            constraints={"reachable_m": walk_m,
                         "on_free_floor": reach.is_free(view_x, view_y),
                         "needs_movement": walk_m > 0.05,
                         "needs_depth_camera": bool(ranging.get("never_ranged")),
                         "range_m": got["range_m"],
                         "in_certified_band": (BAND_NEAR_M <= got["range_m"]
                                               <= BAND_FAR_M),
                         "tilt_unknown": got["tilt_unknown"],
                         "goal": {"x_m": round(view_x, 3),
                                  "y_m": round(view_y, 3)}}))
    return out


def _gap(place: tuple[float, float, float], x: float, y: float) -> float:
    """How far a place to stand is from the thing it would be looking at."""
    return math.hypot(place[0] - x, place[1] - y)


def _nowhere_to_stand(entity: dict[str, Any], entity_id: str,
                      generation: str | None, x: float, y: float,
                      offered: int, blocked: int) -> list[Candidate]:
    """A thing worth looking at that there is nowhere to look at it from.

    **Recorded as a refused candidate rather than left out**, and the difference
    is the whole point of writing candidates down: a thing placed inside a wall,
    or in a corner of the map the rover cannot walk to, would otherwise be
    invisible in the record -- indistinguishable from a thing nothing wanted to
    look at. It comes back with no route, so `scoring.py` vetoes it and the
    reason appears next to the goal it refused.
    """
    if not entity_id:
        return []                                              # pragma: no cover
    reason = ("every place within the certified band is either unreachable or "
              "has a wall in the way"
              if blocked else
              "there is nowhere within the certified band that the rover can "
              "walk to")
    return [Candidate(
        id=f"improve_geometry:{entity_id}@nowhere",
        type="improve_geometry", target=entity_id,
        refs=[refs.world(generation, entity_id)],
        why=(f"{entity_id} is placed to "
             f"{_room_to_improve(entity):.2f} m and could be better, but "
             f"{reason} ({offered} places tried, {blocked} of them blocked)"),
        expects="nothing: there is nowhere to take the look from",
        action=[], risk="drives_on_mapped_floor",
        gain_kind="placement_uncertainty_m",
        gain_value=_room_to_improve(entity),
        gain_detail={"nowhere_to_stand": True, "places_tried": offered,
                     "blocked_by_a_wall": blocked,
                     "thing_at": {"x_m": round(x, 3), "y_m": round(y, 3)}},
        constraints={"reachable_m": None, "on_free_floor": False,
                     "needs_movement": True,
                     "goal": {"x_m": round(x, 3), "y_m": round(y, 3)}})]


def _from_viewpoint(placement: dict[str, Any], entity: dict[str, Any],
                    view_x: float, view_y: float) -> dict[str, Any]:
    """What one more look from here would do to where the thing is believed to be.

    **A bearing constrains a position across the line of sight and not along
    it.** At range d a bearing good to sigma pins the thing to about `d*sigma`
    sideways and says nothing about how far away it is, so the uncertainty a
    crossing leaves behind is an ellipse whose long axis lies along the ray. Two
    looks from nearly the same place therefore leave the same long thin ellipse,
    however many times they are taken; a look from the side collapses it.

    So the useful quantity is how squarely the new ray crosses the present long
    axis. With the present long axis `a`, a new sideways constraint `w = d*sigma`
    and `phi` the angle between that constraint's direction and the long axis,
    combining the two as independent Gaussians gives

        1/a'^2 = 1/a^2 + cos(phi)^2 / w^2

    which is the whole model. A ray straight down the long axis has `cos(phi)`
    zero and buys nothing, which is the correct answer and the one a scorer
    guessing from "another look is another look" would get wrong.

    A thing whose distance has never been measured is the other case, and the
    reduction there is not from crossing but from a range landing on it: the
    long axis collapses to about where ranges land, which the acceptance run
    measured, and no further.
    """
    range_m = math.hypot(view_x - float(placement["x_m"]),
                         view_y - float(placement["y_m"]))
    sigma = math.radians(BEARING_SIGMA_DEG)
    across_m = max(range_m, 0.05) * sigma

    before = placement.get("error_major_m")
    if before is None:
        before = placement.get("uncertainty_m")
    before = UNKNOWN_PLACEMENT_M if before is None else float(before)

    major_deg = placement.get("error_major_deg")
    if major_deg is None:
        # No axis reported means no direction to cross, so the crossing angle
        # cannot be claimed. Treated as the worst case rather than the average
        # one: a candidate whose benefit cannot be predicted should not outrank
        # one whose benefit is known.
        crossing_deg, cos_phi = 0.0, 0.0
    else:
        ray_deg = math.degrees(math.atan2(float(placement["y_m"]) - view_y,
                                          float(placement["x_m"]) - view_x))
        # The new constraint lies across the ray; the angle that matters is
        # between that direction and the long axis being cut.
        across_deg = ray_deg + 90.0
        phi = math.radians(_fold(across_deg - float(major_deg)))
        cos_phi = abs(math.cos(phi))
        crossing_deg = 90.0 - abs(_fold(across_deg - float(major_deg)))
        crossing_deg = abs(crossing_deg)

    after = before
    if cos_phi > 0.0:
        after = 1.0 / math.sqrt(1.0 / (before * before)
                                + (cos_phi * cos_phi) / (across_m * across_m))

    ranging = entity.get("ranging") or {}
    first_range = bool(ranging.get("never_ranged"))
    if first_range:
        # A measured distance does what no second bearing can: it cuts the long
        # axis directly. It cannot do better than where ranges land, and it is
        # only on offer inside the band the depth camera was certified in.
        if BAND_NEAR_M <= range_m <= BAND_FAR_M:
            after = min(after, max(RANGE_LANDS_WITHIN_M,
                                   float(placement.get("error_minor_m") or 0.0)))

    # Nothing this rover does places a thing better than a tenth of a metre, and
    # the arithmetic above does not know that -- it knows about bearings and not
    # about the pose they were taken from or the width of the thing they were
    # taken of. Applied last, so it caps both roads to an answer.
    after = max(after, PLACEMENT_FLOOR_M)

    height_sigma = placement.get("height_sigma_m")
    tilt_unknown = (height_sigma is None
                    or float(height_sigma) > HEIGHT_SIGMA_LIMIT_M)

    return {"gain_m": max(0.0, round(before - after, 4)),
            "before_m": round(before, 4), "after_m": round(after, 4),
            "range_m": round(range_m, 3),
            "across_m": round(across_m, 4),
            "crossing_deg": round(crossing_deg, 1),
            "bearing_sigma_deg": BEARING_SIGMA_DEG,
            "first_range": first_range,
            "viewpoints_so_far": placement.get("viewpoints"),
            "parallax_so_far_deg": placement.get("parallax_deg"),
            "tilt_unknown": tilt_unknown,
            "why": _why(entity, before, after, range_m, crossing_deg,
                        first_range)}


def _why(entity: dict[str, Any], before: float, after: float, range_m: float,
         crossing_deg: float, first_range: bool) -> str:
    """One sentence a person could be told, with the numbers in it."""
    name = str(entity.get("id") or "a thing")
    looks = entity.get("observation_count")
    seen = f", seen {looks} times" if looks else ""
    if first_range:
        return (f"{name} has never had its distance measured{seen}; from "
                f"{range_m:.1f} m the depth camera could, which would place it "
                f"to about {after:.2f} m instead of {before:.2f} m")
    return (f"{name} is placed to {before:.2f} m{seen}; a look from "
            f"{range_m:.1f} m crossing at {crossing_deg:.0f} degrees would "
            f"bring that to about {after:.2f} m")


def _fold(degrees: float) -> float:
    """An angle folded into -90..90, which is all a line's direction means."""
    folded = (degrees + 90.0) % 180.0 - 90.0
    return folded
