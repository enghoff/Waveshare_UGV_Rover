#!/usr/bin/env python3
"""Rooms drawn on paper, and what the rover ought to want to do in each.

    python autonomy/scenarios.py
    python autonomy/scenarios.py --show frontiers

**A scorer can only be judged against cases somebody decided the answer to
first.** Running it on the rover shows what it does; it cannot show whether that
was right, because the rover has no opinion. So the acceptance set is a library
of small situations with the expected outcome written down beside each -- and
the rule that goes with it is the one the plan states: when the code and the
expectation disagree, the disagreement is reviewed, and the expectation is
changed only with a written reason and never merely to make the number pass.

Each scenario is a room drawn as a picture, a few things placed in it, and what
should happen:

    ##################          the room, one character per cell:
    #................#            #  wall      .  floor
    #.......R........#            ?  unseen    R  the rover
    #................#
    ############.....#

That is the whole format. It is a picture rather than a list of coordinates
because the expectation has to be arguable by somebody who is not holding the
code in their head: you can see the doorway, and you can see that the thing in
the corner is behind a wall.

## Everything here is hypothetical, deliberately

The scenarios are scored **as if the rover had authority to act**, because that
is the only way to test the refusals that would stop it: a gate that is always
shut -- and today's is, since nothing in this component can move anything -- would
make every case come out the same. What is being checked is the choosing, and
the choosing is what Phase 3 will inherit.

## What this file is not

It is not a simulator. Nothing here drives anything, nothing steps time forward,
and no scenario says what the rover *did* -- only what it should have wanted.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from typing import Any

import scoring
from situation import Situation

#: One character per cell, and the first row drawn is the highest y -- so a room
#: drawn here looks like a room on a map rather than upside down.
WALL, FLOOR, UNSEEN, ROVER = "#", ".", "?", "R"

#: Where the curated set lives. One file per theme rather than one big one,
#: because a reviewer reads them by theme: what it explores, what it goes back
#: to look at, what it refuses, and how it chooses between them.
SCENARIO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "scenarios")

#: What the milestone asks of this set: at least forty cases, and the expected
#: ordering right in at least ninety-five per cent of them.
REQUIRED_CASES = 40
REQUIRED_RATE = 0.95


# --- the shapes a scenario is built from ------------------------------------

def occupancy(rows: list[str], *, resolution_m: float = 0.1,
              origin: tuple[float, float] = (0.0, 0.0)) -> dict[str, Any]:
    """A drawn room in the shape the daemon's `nav_grid` returns one.

    Compressed and base64'd like the real thing rather than handed over as a
    list of numbers, so that everything downstream -- the decode, the sign of an
    unknown cell, where the origin sits -- is exercised by every scenario. A
    fixture that skipped the encoding could not catch the one bug this path has
    ever had.
    """
    import base64
    import zlib

    height = len(rows)
    width = max(len(row) for row in rows)
    cells = bytearray()
    for row in reversed(rows):
        for char in row.ljust(width, UNSEEN):
            cells.append(100 if char == WALL else
                         0 if char in (FLOOR, ROVER) else 255)
    return {"ok": True, "width": width, "height": height,
            "resolution_m": resolution_m,
            "origin_x_m": origin[0], "origin_y_m": origin[1],
            "data": base64.b64encode(zlib.compress(bytes(cells), 6)
                                     ).decode("ascii")}


def rover_in(rows: list[str], *, resolution_m: float = 0.1,
             origin: tuple[float, float] = (0.0, 0.0)) -> tuple[float, float]:
    """Where the `R` in a drawn room is, in map metres."""
    for index, row in enumerate(reversed(rows)):
        if ROVER in row:
            return (origin[0] + (row.index(ROVER) + 0.5) * resolution_m,
                    origin[1] + (index + 0.5) * resolution_m)
    raise ValueError("no R in the room: nothing says where the rover is")


def thing(entity_id: str, x: float, y: float, *, uncertainty_m: float = 0.30,
          major_deg: float = 0.0, minor_m: float | None = None,
          looks: int = 6, ranged: int = 2, height_sigma_m: float = 0.05,
          viewpoints: int = 2, map_session: int = 7,
          placed: bool = True) -> dict[str, Any]:
    """One entity in the shape `world_state_entities` reports one.

    The defaults are a thing this rover would really hold: placed, seen a
    handful of times from two viewpoints, its distance measured on some of them.
    Every case a scenario cares about is one of those turned off.
    """
    placement = {"x_m": x, "y_m": y, "uncertainty_m": uncertainty_m,
                 "error_major_m": uncertainty_m,
                 "error_minor_m": (uncertainty_m / 3.0 if minor_m is None
                                   else minor_m),
                 "error_major_deg": major_deg,
                 "height_sigma_m": height_sigma_m,
                 "viewpoints": viewpoints}
    return {
        "id": entity_id, "kind": "object", "label": "",
        "observation_count": looks, "created_at": 1757320000.0,
        "last_seen_at": 1757320600.0,
        "placement": placement if placed else None,
        "placement_uncertainty_m": uncertainty_m if placed else None,
        "placement_map_session": map_session if placed else None,
        "last_map_session": map_session,
        "ranging": {"looks": looks, "ranged": ranged,
                    "outside_view": max(0, looks - ranged), "unmeasurable": 0,
                    "never_ranged": ranged == 0,
                    "only_outside_view": ranged == 0 and looks > 0},
        "exemplar_count": 3,
    }


def situation(rows: list[str], *, entities: list[dict] | None = None,
              at: float = 1757320800.0, battery_v: float = 12.1,
              driving: bool = False, exploring: bool = False,
              estop: bool = False, position_trusted: bool = True,
              map_settled: bool = True, building: bool = True,
              map_session: int = 7, generation: str = "9f2a1c04ffab3d21",
              world: dict | None = None, nav: dict | None = None,
              resolution_m: float = 0.1,
              origin: tuple[float, float] = (0.0, 0.0),
              **extra: Any) -> dict[str, Any]:
    """A whole situation, in the shape `situation.Situation` holds one."""
    x, y = rover_in(rows, resolution_m=resolution_m, origin=origin)
    body: dict[str, Any] = {
        "at": at,
        "world_generation": generation,
        "map_session": map_session,
        "world": {"entities": len(entities or []), "observations": 100,
                  "unmatched": 4, "inspections": 30,
                  "map_session": map_session, "world_generation": generation,
                  "last_at": at - 2.0, "last_status": "ok", **(world or {})},
        "entities": list(entities or []),
        "nav": {"driving": driving, "exploring": exploring, "estop": estop,
                "pose": {"x_m": x, "y_m": y, "heading_deg": 90.0},
                "map_id": "m1", "map_settled": map_settled, "map_kept": True,
                "position_trusted": position_trusted, "match_score": 0.8,
                **(nav or {})},
        "battery_v": battery_v,
        "building": {"building": building, "looks": 12, "every_s": 1.0},
        "map": occupancy(rows, resolution_m=resolution_m, origin=origin),
    }
    body.update(extra)
    return body


# --- reading a scenario file ------------------------------------------------

def load(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        body = json.load(handle)
    body["path"] = path
    for one in body.get("scenarios", []):
        one["file"] = os.path.basename(path)
    return body


def load_all(directory: str | None = None) -> list[dict[str, Any]]:
    """Every scenario file, in a fixed order so a run is repeatable."""
    where = directory or SCENARIO_DIR
    return [load(path) for path in sorted(glob.glob(os.path.join(where,
                                                                 "*.json")))]


def expand(rows: list[Any]) -> list[str]:
    """A drawn room, with runs of identical rows written as `[count, row]`.

    Twenty-six identical rows of question marks say less to a reader than the
    line that makes them, and a picture nobody reads is a picture nobody checks.
    """
    out: list[str] = []
    for row in rows:
        if isinstance(row, str):
            out.append(row)
        else:
            count, pattern = row
            out.extend([pattern] * int(count))
    return out


def build(scenario: dict[str, Any], maps: dict[str, Any]) -> Situation:
    """The situation one scenario describes."""
    room = maps[scenario["map"]]
    rows = expand(room["rows"] if isinstance(room, dict) else room)
    fields = dict(scenario.get("rover") or {})
    entities = [thing(one.pop("id"), *one.pop("at"), **one)
                for one in [dict(each) for each in
                            scenario.get("entities") or []]]
    body = situation(rows, entities=entities,
                     resolution_m=(room.get("resolution_m", 0.1)
                                   if isinstance(room, dict) else 0.1),
                     **fields)
    for name in ("cooled", "previous_goal"):
        if name in scenario:
            body[name] = scenario[name]
    return Situation(body)


# --- what a scenario expects ------------------------------------------------

def matches(matcher: Any, candidate: dict[str, Any]) -> bool:
    """Does this candidate answer to this description?

    A matcher is a small dictionary rather than an identifier, because an
    identifier carries the coordinates the generator happened to choose and an
    expectation should not have to know them: what a reviewer means is "the
    frontier one" or "the one about object:8".
    """
    if isinstance(matcher, str):
        return candidate["id"] == matcher
    if "id" in matcher and candidate["id"] != matcher["id"]:
        return False
    if "type" in matcher and candidate["type"] != matcher["type"]:
        return False
    if "target" in matcher and candidate["target"] != matcher["target"]:
        return False
    if "near" in matcher:
        x, y, tolerance = matcher["near"]
        goal = candidate["constraints"].get("goal") or {}
        if goal.get("x_m") is None:
            return False
        if abs(goal["x_m"] - x) > tolerance or abs(goal["y_m"] - y) > tolerance:
            return False
    return True


def judge(scenario: dict[str, Any], got: dict[str, Any]) -> list[str]:
    """Everything the run did that the scenario did not expect.

    An empty list is a pass. Each entry is a sentence, because a failing
    scenario is read by a person deciding which of the two is wrong.
    """
    expect = scenario.get("expect") or {}
    wrong: list[str] = []
    considered = got["considered"]
    chosen = got["preferred"]

    if "chose" in expect:
        wanted = expect["chose"]
        if wanted in (None, "nothing"):
            if chosen is not None:
                wrong.append(f"expected it to choose nothing, and it would "
                             f"{chosen['candidate']['id']}")
        elif chosen is None:
            wrong.append("expected a choice, and it would do nothing: "
                         + got["why_nothing"])
        elif not matches(wanted, chosen["candidate"]):
            wrong.append(f"expected {_name(wanted)}, and it would "
                         f"{chosen['candidate']['id']}")

    # An order is checked by where each matcher first appears in the ranking,
    # rather than by comparing scores: what a reviewer means by "the doorway
    # before the corner" is that one outranks the other, not by how much.
    order = expect.get("order") or []
    if order:
        places = []
        for matcher in order:
            where = [index for index, one in enumerate(considered)
                     if matches(matcher, one["candidate"])]
            if not where:
                wrong.append(f"nothing matched {_name(matcher)}, so the order "
                             f"could not be checked")
                places = []
                break
            places.append(min(where))
        if places and places != sorted(places):
            wrong.append("expected the order " +
                         " then ".join(_name(one) for one in order) +
                         f", and got positions {places}")

    for matcher in expect.get("vetoed") or []:
        found = [one for one in considered
                 if matches(matcher, one["candidate"])]
        if not found:
            wrong.append(f"expected {_name(matcher)} to be refused, and it was "
                         f"not offered at all")
            continue
        vetoes = [veto["veto"] for one in found for veto in one["vetoes"]]
        if not vetoes:
            wrong.append(f"expected {_name(matcher)} to be refused, and it was "
                         f"allowed")
        elif matcher.get("veto") and matcher["veto"] not in vetoes:
            wrong.append(f"expected {_name(matcher)} refused as "
                         f"{matcher['veto']!r}, and it was refused as "
                         f"{vetoes}")

    for matcher in expect.get("offered") or []:
        if not any(matches(matcher, one["candidate"]) for one in considered):
            wrong.append(f"expected {_name(matcher)} to be offered at all")

    for matcher in expect.get("not_offered") or []:
        if any(matches(matcher, one["candidate"]) for one in considered):
            wrong.append(f"expected nothing matching {_name(matcher)}")

    for name in expect.get("gate") or []:
        if not any(one["gate"] == name for one in got["gate"]):
            wrong.append(f"expected the gate shut for {name!r}, and it was "
                         f"{[one['gate'] for one in got['gate']]}")

    # One term of one candidate's score, for the cases where what is being
    # checked is not who won but what a particular cost did -- that switching
    # was charged, or that an unpredictable tilt halved the gain.
    for wanted in expect.get("terms") or []:
        found = [one for one in considered
                 if matches(wanted.get("match", {}), one["candidate"])]
        if not found:
            wrong.append(f"nothing matched {_name(wanted.get('match', {}))}, "
                         f"so its {wanted.get('field')} could not be checked")
            continue
        value = found[0]["score"].get(wanted["field"])
        if "at_least" in wanted and not value >= wanted["at_least"]:
            wrong.append(f"expected {wanted['field']} of at least "
                         f"{wanted['at_least']} and got {value}")
        if "at_most" in wanted and not value <= wanted["at_most"]:
            wrong.append(f"expected {wanted['field']} of at most "
                         f"{wanted['at_most']} and got {value}")

    if "candidates" in expect and len(considered) != expect["candidates"]:
        wrong.append(f"expected {expect['candidates']} candidates and got "
                     f"{len(considered)}")
    return wrong


def _name(matcher: Any) -> str:
    if isinstance(matcher, str):
        return matcher
    parts = [str(matcher.get("type") or "a candidate")]
    if matcher.get("target"):
        parts.append("about " + matcher["target"])
    if matcher.get("near"):
        parts.append("near (%.1f, %.1f)" % (matcher["near"][0],
                                            matcher["near"][1]))
    return " ".join(parts)


# --- running the set --------------------------------------------------------

def run(directory: str | None = None) -> dict[str, Any]:
    """Every scenario in the library, and how many came out as expected."""
    results = []
    for body in load_all(directory):
        maps = body.get("maps") or {}
        for scenario in body.get("scenarios") or []:
            here = build(scenario, maps)
            weights = scoring.Weights.from_dict(scenario.get("weights"))
            got = scoring.consider(
                here, weights,
                authority=bool(scenario.get("authority", True)))
            wrong = judge(scenario, got)
            results.append({"name": scenario["name"],
                            "file": scenario.get("file", ""),
                            "why": scenario.get("why", ""),
                            "passed": not wrong, "wrong": wrong,
                            "decision": got})
    passed = sum(1 for one in results if one["passed"])
    return {"results": results, "cases": len(results), "passed": passed,
            "rate": (passed / len(results)) if results else 0.0}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the curated scenarios against the scorer.")
    parser.add_argument("--dir", default=None, help="where the scenarios are")
    parser.add_argument("--show", default="",
                        help="print the full ranking for scenarios whose name "
                             "or file contains this")
    args = parser.parse_args(argv)

    got = run(args.dir)
    for one in got["results"]:
        mark = "ok  " if one["passed"] else "FAIL"
        print(f"  {mark} {one['file']}: {one['name']}")
        for sentence in one["wrong"]:
            print(f"       {sentence}")
        if args.show and (args.show in one["name"] or args.show in one["file"]):
            for ranked in one["decision"]["considered"]:
                veto = "; ".join(veto["veto"] for veto in ranked["vetoes"])
                print(f"       {ranked['score']['utility']:>7.3f} "
                      f"{ranked['candidate']['id']}"
                      f"{'  refused: ' + veto if veto else ''}")
    print(f"\n{got['passed']} of {got['cases']} scenarios as expected "
          f"({100 * got['rate']:.0f}%)")
    if got["cases"] < REQUIRED_CASES:
        print(f"  the acceptance set is meant to hold at least "
              f"{REQUIRED_CASES} cases")
    return 0 if (got["rate"] >= REQUIRED_RATE
                 and got["cases"] >= REQUIRED_CASES) else 1


if __name__ == "__main__":
    sys.exit(main())
