"""What each candidate is worth, what refuses it outright, and which one wins.

**Curiosity is scored here, not prompted.** There is no model in this file and
no sentence asking one to be curious: a candidate's worth is arithmetic over the
estimates `goals.py` produced, with every term written down and every weight in
a configuration a person can read and change. That is the design's own rule --
weights are configuration, not hidden model behaviour -- and it is what makes a
choice arguable afterwards.

## The score

    utility = purpose_relevance * gain
              - w_time   * time_cost
              - w_travel * travel_cost
              - w_energy * energy_cost
              - switching_cost

Every term is dimensionless, and the conversions that make them so are declared
in `Weights` rather than buried: gain is the physical estimate over the scale at
which that kind of knowledge is worth having, time is seconds over a minute,
travel is metres over the long way across a room. **Idle is zero**, so a
candidate has to be worth more than doing nothing rather than merely be the best
of a bad list, and a candidate must clear `min_gain` before its cheapness can
recommend it at all.

## Refusals come first, and cannot be outscored

Two kinds, and the difference is not a technicality:

**The gate** is about the rover, not the goal -- a flat battery, an unhealthy
navigator, a stop somebody latched, or the standing fact of Phase 2 that nothing
here has any authority to move anything. When the gate is shut nothing is
chosen, whatever the candidates say.

**A veto** is about one candidate -- somewhere it cannot walk to, a viewpoint
outside the band the geometry is certified in, a goal outside a configured safe
area, hardware it needs and does not have. A vetoed candidate is removed before
scoring and can never be selected however the weights are set, which is a
property `test_scoring.py` checks by turning the gain weight up to a thousand.

The scored-but-refused ones are kept and reported. "Why did it not go and look
at the thing in the hall" is the question a shadow run exists to answer, and the
answer is worthless if the candidate quietly never existed.
"""
from __future__ import annotations

import json
import os
from typing import Any

import cooling
import goals as goals_mod
from situation import Situation

#: Where a person's own weights and purpose live, if they have written any. Off
#: the deploy tree with the rest of the rover's runtime state, so that deploying
#: this component cannot overwrite what somebody tuned.
CONFIG_NAME = "scoring.json"

#: How near the goal the last deliberation wanted a candidate has to be to count
#: as the same one. `frontier.HYSTERESIS_M`'s metre, and for the same reason it
#: is a metre there: more than the reordering noise between two candidates, less
#: than the distance to anything in another room.
HYSTERESIS_M = 1.0


class Weights:
    """The configuration a score is computed with, and its version.

    Recorded whole into every decision rather than referenced by name. A version
    string alone would answer "which weights were these" only for as long as
    somebody kept the file that went with it, and the file lives on the rover
    where nothing versions it.
    """

    __slots__ = ("version", "purpose", "w_time", "w_travel", "w_energy",
                 "switching_cost", "min_gain", "room_m2",
                 "useful_uncertainty_m", "time_scale_s", "travel_scale_m",
                 "battery_floor_v", "geofence", "source")

    def __init__(self, **fields: Any) -> None:
        for name, value in DEFAULTS.items():
            setattr(self, name, fields.get(name, value))
        self.source = fields.get("source", "built in")

    def relevance(self, goal_type: str) -> float:
        """How much this rover's owner cares about this kind of goal.

        **Everything is 1.0 until somebody says otherwise, and that is a stated
        position rather than a missing feature.** The design's purpose term is a
        user policy -- "learn where household objects tend to be", "understand
        every doorway and route" -- and inventing one on the owner's behalf
        would put a preference nobody holds into every decision the rover
        records. Equal weight is the honest default: it says the rover has been
        told nothing about what it is for.
        """
        return float(self.purpose.get(goal_type, self.purpose.get("default", 1.0)))

    def as_dict(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in
                (*DEFAULTS.keys(), "source")}

    @classmethod
    def load(cls, directory: str | None = None) -> "Weights":
        """The weights on this rover, or the built-in ones if it has none."""
        path = os.path.join(directory or _autonomy_dir(), CONFIG_NAME)
        try:
            with open(path, "r", encoding="utf-8") as handle:
                body = json.load(handle)
        except (OSError, ValueError):
            return cls()
        if not isinstance(body, dict):
            return cls()
        return cls(**{**body, "source": path})

    @classmethod
    def from_dict(cls, body: dict[str, Any] | None) -> "Weights":
        return cls(**dict(body or {}))


#: The built-in configuration. Each number is here with the reason it is that
#: number, because a weight without a reason is a weight nobody can argue with.
DEFAULTS: dict[str, Any] = {
    # Bumped whenever anything below changes, so that two decisions scored
    # differently can be told apart in the record without diffing them.
    "version": "1",

    # Per goal type, and 1.0 everywhere until the owner declares a purpose.
    "purpose": {"default": 1.0, "explore_frontier": 1.0,
                "improve_geometry": 1.0},

    # What a minute of the rover's time and ten metres of its driving are worth
    # against a whole unit of knowledge. Both at 0.3 so that a goal has to be
    # roughly a room's worth of new floor to justify crossing the house for it,
    # while a good goal two metres away is nearly free. They are equal because
    # nothing measured says which of time and travel this rover should mind
    # more; when something does, this is where it goes.
    "w_time": 0.30,
    "w_travel": 0.30,
    # Zero, and kept. There is no current sense on this chassis, so the energy
    # estimate is time in different units and charging for it would charge for
    # time twice. See `goals.NOMINAL_DRAW_W`.
    "w_energy": 0.0,
    # What it costs to turn the rover away from what it is already doing. A
    # sixth of a unit is more than the noise between two similar candidates and
    # less than any real difference between them, which is the whole job: it
    # stops a rover swapping goals every time the map redraws, and it does not
    # stop it abandoning a poor goal for a good one.
    "switching_cost": 0.15,
    # Being safe and reachable is not enough to move a rover. Below this the
    # candidate is not worth the disturbance whatever it costs. On the scale
    # below, 0.15 of a unit is about five centimetres off where a thing is, or
    # three and a half square metres of floor -- under either the rover has
    # better things to do than drive across the room.
    "min_gain": 0.15,

    # The scales that make the terms dimensionless. Each is the value worth
    # half a unit; see `_saturate`.
    #
    # 20 m2 is a small room: the floor a rover that drove to one frontier could
    # plausibly add to the map in one go. 0.30 m is not a guess at all -- it is
    # the separation tolerance the M0 acceptance run declared and passed, so
    # taking that much uncertainty out of a placement is the scale of the
    # knowledge this rover is judged on. A minute and ten metres are the scale
    # of one errand.
    "room_m2": 20.0,
    "useful_uncertainty_m": 0.30,
    "time_scale_s": 60.0,
    "travel_scale_m": 10.0,

    # 11.2 V is 3.73 V per cell on this three-cell pack, which the driver
    # board's own curve calls about a fifth left and reports as `low`. A rover
    # deciding to drive somewhere on the last fifth of its battery is a rover
    # that ends the day somewhere nobody wanted it.
    "battery_floor_v": 11.2,

    # No safe area is configured, which is a fact about this rover rather than
    # an oversight: nothing here may move, so nothing needs fencing yet. When
    # Phase 3 arrives it is `{"x_m", "y_m", "radius_m"}` or a `{"min_x_m", ...}`
    # box, and the veto below is already written against both.
    "geofence": None,
}

DEFAULT = Weights()


# --- what stops a decision from being acted on ------------------------------

def gate(situation: Situation, weights: Weights = DEFAULT, *,
         authority: bool = False) -> list[dict[str, str]]:
    """Everything about the rover that would refuse any goal at all.

    `authority` is the standing fact of this phase and defaults to false: there
    is no executive, and the component that records this cannot make a call that
    moves anything. It is an argument rather than a constant so that the day
    something does have authority, the test that says an ungated rover chooses
    is the same test as today's.
    """
    shut: list[dict[str, str]] = []
    if not authority:
        shut.append({"gate": "no movement authority",
                     "why": "nothing in this component can move the rover: "
                            "every call it may make is a read"})
    for name, sentence in sorted(situation.health().items()):
        shut.append({"gate": name, "why": sentence})
    volts = situation.battery_v
    if volts is None:
        shut.append({"gate": "battery unknown",
                     "why": "the driver board did not report a battery "
                            "voltage, so there is no telling what is left"})
    elif volts < float(weights.battery_floor_v):
        shut.append({"gate": "battery low",
                     "why": f"the pack reads {volts:.2f} V, under the "
                            f"{weights.battery_floor_v:.1f} V this rover keeps "
                            f"in reserve"})
    return shut


def vetoes(candidate: goals_mod.Candidate, situation: Situation,
           weights: Weights = DEFAULT) -> list[dict[str, str]]:
    """Everything about one candidate that refuses it outright.

    Ordered as written rather than by severity, because a candidate refused for
    three reasons should report all three: fixing the one a sorter happened to
    put first would leave the goal just as impossible.
    """
    out: list[dict[str, str]] = []
    facts = candidate.constraints

    if facts.get("needs_movement") and facts.get("reachable_m") is None:
        out.append({"veto": "unreachable",
                    "why": "there is no route to it over floor the map calls "
                           "free"})
    if facts.get("needs_movement") and not facts.get("on_free_floor", True):
        out.append({"veto": "not on known floor",
                    "why": "the place it would drive to is not ground the "
                           "mapper has confirmed is free, and this rover has no "
                           "way to see a step or a drop before it reaches one"})
    if facts.get("needs_depth_camera") and not _depth_camera(situation):
        out.append({"veto": "no depth camera",
                    "why": "the distance is the whole point of this goal and "
                           "the depth camera is not answering"})
    if "in_certified_band" in facts and not facts["in_certified_band"]:
        out.append({"veto": "outside the certified band",
                    "why": f"the look would be taken from "
                           f"{facts.get('range_m')} m, outside the "
                           f"{goals_mod.BAND_NEAR_M} to {goals_mod.BAND_FAR_M} m "
                           f"band this rover's geometry was accepted in"})
    outside = _outside_geofence(facts.get("goal"), weights.geofence)
    if outside:
        out.append({"veto": "outside the safe area", "why": outside})
    cool = cooling.cooling(situation.cooled, candidate.target, now=situation.at)
    if cool:
        out.append({"veto": "cooling off", "why": str(cool.get("why") or
                                                      "recently got nowhere")})
    return out


def _depth_camera(situation: Situation) -> bool:
    """Is the thing that measures distances answering at all?

    Read off whether the perception loop is running rather than asked directly:
    the depth camera is reached through it, and a stopped loop is a rover that
    will not produce a range whatever the camera is doing.
    """
    building = situation.building
    if not building:
        return False
    return building.get("building") is not False


def _outside_geofence(goal: dict[str, Any] | None,
                      fence: dict[str, Any] | None) -> str:
    """Whether a goal leaves the configured safe area, if one is configured."""
    if not fence or not goal:
        return ""
    x, y = goal.get("x_m"), goal.get("y_m")
    if x is None or y is None:
        return ""
    if fence.get("radius_m") is not None:
        gap = ((x - float(fence.get("x_m", 0.0))) ** 2
               + (y - float(fence.get("y_m", 0.0))) ** 2) ** 0.5
        if gap > float(fence["radius_m"]):
            return (f"it is {gap:.1f} m from the middle of the safe area, "
                    f"which reaches {float(fence['radius_m']):.1f} m")
        return ""
    for low, high, value, axis in ((fence.get("min_x_m"), fence.get("max_x_m"),
                                    x, "x"),
                                   (fence.get("min_y_m"), fence.get("max_y_m"),
                                    y, "y")):
        if low is not None and value < float(low):
            return f"its {axis} is outside the safe area"
        if high is not None and value > float(high):
            return f"its {axis} is outside the safe area"
    return ""


# --- what a candidate is worth ----------------------------------------------

def score(candidate: goals_mod.Candidate, situation: Situation,
          weights: Weights = DEFAULT) -> dict[str, Any]:
    """Every term of one candidate's utility, and the total.

    Returned as the decomposition rather than as a number, because a decision
    that records only its winner records nothing anybody can argue with. The
    keys are the terms of the formula at the top of this file, in that order.
    """
    gain, gain_note = _gain(candidate, weights)
    relevance = weights.relevance(candidate.type)
    time_cost = candidate.time_s / float(weights.time_scale_s)
    travel_cost = candidate.travel_m / float(weights.travel_scale_m)
    energy_cost = candidate.energy_wh
    switching = _switching(candidate, situation, weights)

    utility = (relevance * gain
               - float(weights.w_time) * time_cost
               - float(weights.w_travel) * travel_cost
               - float(weights.w_energy) * energy_cost
               - switching)
    return {
        "gain": round(gain, 4),
        "gain_note": gain_note,
        "purpose_relevance": relevance,
        "time_cost": round(time_cost, 4),
        "travel_cost": round(travel_cost, 4),
        "energy_cost": round(energy_cost, 5),
        "switching_cost": round(switching, 4),
        "utility": round(utility, 4),
        "weights_version": weights.version,
        "below_min_gain": gain < float(weights.min_gain),
    }


def _saturate(value: float, scale: float) -> float:
    """A physical estimate as a number between nothing and one unit.

    `value / (value + scale)`, which is half a unit at the declared scale and
    approaches one and never reaches it. **Diminishing returns rather than a
    cliff**, and the difference matters more than it looks: a hard cap at the
    scale would make every badly placed thing on this rover score exactly one --
    most of them are half a metre out -- and a scorer whose gains are all equal
    is a scorer that has quietly become a cost model. This keeps the ordering of
    two candidates informative all the way up, while still refusing to let one
    enormous frontier be worth ten rooms to a rover that can only be in one
    place.
    """
    value = max(0.0, float(value))
    return 0.0 if value <= 0.0 else value / (value + max(1e-9, scale))


def _switching(candidate: goals_mod.Candidate, situation: Situation,
               weights: Weights) -> float:
    """What it costs to change what the rover is doing to do this instead.

    **Charged for the change and not for the goal**, which is why the candidate
    the last deliberation preferred pays nothing: two frontiers a few hundredths
    apart will trade places every time the map redraws, and a rover that follows
    the top of that list turns round in the middle of the room and then turns
    round again. It is the same argument, and the same fix, as the hysteresis in
    `frontier.py` -- more than the reordering noise, less than any real
    difference.

    Interrupting a move already under way costs the same, because it is the same
    thing from the rover's point of view.
    """
    previous = situation.previous_goal
    if previous:
        return 0.0 if _the_same_goal(candidate, previous) else float(
            weights.switching_cost)
    return float(weights.switching_cost) if situation.busy() else 0.0


def _the_same_goal(candidate: goals_mod.Candidate,
                   previous: dict[str, Any]) -> bool:
    """Is this the goal the last deliberation wanted, in all but its name?

    **What makes two goals the same depends on what the goal is about.** A goal
    about a thing is the same goal wherever the rover ends up standing to look
    at it, and a different thing is a different goal however close the two
    viewpoints happen to be -- which is not hypothetical: on the real map a
    place to stand and look at one object is routinely within a metre of a place
    to stand and look at another. A goal about a place -- a frontier -- is the
    same goal a few centimetres along, because the cell the chooser picks for
    one doorway wanders every time the map is redrawn. The metre is
    `frontier.HYSTERESIS_M`'s and for its reason: more than the reordering
    noise, less than the distance to anything in another room.
    """
    if previous.get("id") and candidate.id == previous["id"]:
        return True
    if candidate.target or previous.get("target"):
        return bool(previous.get("target")) and candidate.target == previous["target"]
    was, goes = previous.get("goal"), candidate.constraints.get("goal")
    if isinstance(was, dict) and isinstance(goes, dict):
        if was.get("x_m") is not None and goes.get("x_m") is not None:
            gap = ((goes["x_m"] - was["x_m"]) ** 2
                   + (goes["y_m"] - was["y_m"]) ** 2) ** 0.5
            return gap <= HYSTERESIS_M
    return False


def _gain(candidate: goals_mod.Candidate, weights: Weights
          ) -> tuple[float, str]:
    """The physical estimate turned into the one currency, and how.

    Capped at one unit. A frontier onto the whole outdoors is not worth ten
    rooms to a rover that can only be in one place, and an uncapped gain is how
    a single enormous candidate comes to outrank every sensible one for ever.

    An estimate the generator could not make is charged for rather than
    ignored: a candidate whose benefit is unpredictable ranks below one whose
    benefit is known and equal, which is the conservative direction.
    """
    value = candidate.gain_value
    if candidate.gain_kind == "unknown_floor_m2":
        gain = _saturate(value, float(weights.room_m2))
        note = (f"{value:.0f} m2 of unmapped floor, counted against the "
                f"{float(weights.room_m2):.0f} m2 that is worth half a unit")
    elif candidate.gain_kind == "placement_uncertainty_m":
        gain = _saturate(value, float(weights.useful_uncertainty_m))
        note = (f"{value:.2f} m taken off where the thing is, counted against "
                f"the {float(weights.useful_uncertainty_m):.2f} m tolerance the "
                f"acceptance run declared")
    else:                                                      # pragma: no cover
        return 0.0, f"no scale is declared for {candidate.gain_kind}"

    if candidate.constraints.get("tilt_unknown"):
        # The validated envelope covers tilt zero and tilt +20, and nothing
        # says which this look would need, because the thing's height above the
        # floor is not known to better than the tilt it implies -- R-WS-11 is
        # open for exactly this. Half, and said out loud.
        gain *= 0.5
        note += ("; halved because the thing's height is too uncertain to say "
                 "which of the two validated tilts the look would need")
    return gain, note


# --- the decision -----------------------------------------------------------

def consider(situation: Situation, weights: Weights = DEFAULT, *,
             authority: bool = False,
             candidates: list[goals_mod.Candidate] | None = None
             ) -> dict[str, Any]:
    """Generate, refuse, score, and say what it would do.

    The whole of one deliberation, and the only function anything outside this
    module needs. What comes back is what gets recorded: the candidates with
    their scores and their refusals, what the rover would have chosen, and --
    when it would have chosen nothing -- the reason, which is a different fact
    from "there was nothing to choose".
    """
    found = list(goals_mod.generate(situation)
                 if candidates is None else candidates)
    shut = gate(situation, weights, authority=authority)

    ranked = []
    for candidate in found:
        refusals = vetoes(candidate, situation, weights)
        terms = score(candidate, situation, weights)
        ranked.append({"candidate": candidate.as_dict(),
                       "score": terms,
                       "vetoes": refusals})
    # Best first, and by identifier where two are equal, so that a tie breaks
    # the same way twice. A vetoed candidate keeps its place in the ordering
    # rather than being sorted to the bottom: what it would have scored is the
    # interesting half of "why did it not do that".
    ranked.sort(key=lambda one: (-one["score"]["utility"],
                                 one["candidate"]["id"]))

    # **Idle has zero utility**, so a candidate whose costs eat its gain is
    # worse than standing still and is not chosen. Without this the rover would
    # always do the least bad thing on the list, which on a finished map is a
    # long drive for nothing.
    allowed = [one for one in ranked
               if not one["vetoes"] and not one["score"]["below_min_gain"]
               and one["score"]["utility"] > 0.0]
    preferred = allowed[0] if allowed else None

    refused_above = []
    if preferred is not None:
        for one in ranked:
            if one is preferred:
                break
            refused_above.append({
                "id": one["candidate"]["id"],
                "utility": one["score"]["utility"],
                "why": _refusal(one, weights)})

    return {
        "at": situation.at,
        "weights": weights.as_dict(),
        "gate": shut,
        "authority": bool(authority),
        "considered": ranked,
        "preferred": preferred,
        "refused_above_it": refused_above,
        "chose": (preferred["candidate"]["id"]
                  if preferred is not None and authority and not shut else None),
        "why_nothing": _why_nothing(shut, ranked, preferred, weights,
                                    authority),
    }


def _refusal(one: dict[str, Any], weights: Weights) -> str:
    if one["vetoes"]:
        return "; ".join(f"{veto['veto']}: {veto['why']}"
                         for veto in one["vetoes"])
    if one["score"]["below_min_gain"]:
        return (f"it would gain {one['score']['gain']}, under the "
                f"{weights.min_gain} minimum worth disturbing the rover for")
    if one["score"]["utility"] <= 0.0:
        return (f"it would cost more than it is worth: {one['score']['utility']}"
                f" against nothing at all for staying put")
    return ""                                                  # pragma: no cover


def _why_nothing(shut: list[dict[str, str]], ranked: list[dict[str, Any]],
                 preferred: dict[str, Any] | None, weights: Weights,
                 authority: bool) -> str:
    """Why nothing is being acted on, in one sentence a person can act on.

    Three genuinely different states, and a rover that reported them alike
    would be hiding the one that matters: nothing to do, something to do and no
    permission, and something to do and a reason not to.
    """
    if not ranked:
        return "nothing was worth considering from here"
    if preferred is None:
        return ("nothing cleared its refusals: "
                + "; ".join(sorted({_refusal(one, weights) for one in ranked
                                    if _refusal(one, weights)}))[:400])
    if shut:
        return ("it would " + preferred["candidate"]["id"] + ", but "
                + "; ".join(one["why"] for one in shut))
    if not authority:                                          # pragma: no cover
        return "it would " + preferred["candidate"]["id"] + ", but may not act"
    return ""


def _autonomy_dir() -> str:
    import store as store_mod
    return store_mod.autonomy_dir()
