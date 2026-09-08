#!/usr/bin/env python3
"""One deliberation: look at the rover, work out what would be worth doing, and
write down what it would have chosen and why.

    ssh orin 'cd ~/ugv/autonomy && python3 decide.py'

**It chooses and does not act, and cannot.** The situation is read through
`client.ReadOnly`, which refuses every call that could move anything, so what
this produces is a decision with no way to carry it out -- which is exactly what
Milestone M2 asks for: the rover explaining what it would investigate next,
before anything is allowed to let it.

The episode it writes is the shape an executive will write later. The candidates
are recorded whether they won or lost, the decision names the snapshot it was
made from rather than the live world, and the whole thing closes `abandoned`,
which is not a failure: it is what a decision taken with no authority to act
closes with.

## What is recorded, and why it is recorded that way

The situation is snapshotted *whole* -- the things with their placements, the
occupancy map, the pose, the battery -- and the decision names its digest. So a
replay a month later reranks the same candidates from the same inputs, and any
disagreement between what it computes now and what the record says is a change
in this code rather than a change in the world. That is the only way "the
scoring is deterministic under replay" can be checked rather than asserted.

The weights go into the decision as well, in full. A version string alone would
answer "which weights were these" only while somebody kept the file that went
with it -- and that file lives on the rover, where nothing versions it.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any

import client as client_mod
import cooling
import events
import refs
import scoring
import situation as situation_mod
import store as store_mod
import summary as summary_mod

#: The trigger an episode of this kind opens with. A sentence rather than a
#: word, because it appears in every summary a person reads.
TRIGGER = "the rover considered what to do next"

#: How many candidates are written into the record. All of them, and the number
#: is here as a bound rather than a policy -- `goals.py` already limits how many
#: it generates, and this exists so that a future generator which returns
#: hundreds cannot quietly turn one deliberation into a thousand rows.
RECORD_LIMIT = 64

#: Where one deliberation leaves what the next one needs: the situation it was
#: made from, so that things which took looks and got no better can be noticed;
#: what it would have done, so that changing its mind costs something; and the
#: cooling that was in force. Marks rather than memory, so a recorder stopped
#: and started again carries on with what the record says instead of an empty
#: head.
SITUATION_MARK = "last_situation"
GOAL_MARK = "last_goal"
COOLED_MARK = "cooled"


def prepare(store: store_mod.EpisodeStore,
            here: situation_mod.Situation) -> situation_mod.Situation:
    """Fill in what this deliberation needs to know about the last one.

    Both additions go into the situation itself rather than being passed round
    it, because a decision has to be reproducible from the snapshot alone: a
    cooling list held in a variable somewhere would refuse a candidate on
    replay day for a reason that is nowhere in the record.
    """
    previous = store.snapshot_body(store.marked(SITUATION_MARK)) or None
    was_cooled = _loads(store.marked(COOLED_MARK)) or []
    here.body["cooled"] = cooling.update(previous, here.body, was_cooled,
                                         now=here.at)
    here.body["previous_goal"] = _loads(store.marked(GOAL_MARK)) or {}
    return here


def _goal_of(preferred: dict[str, Any] | None) -> dict[str, Any]:
    """What the last deliberation wanted, in the terms the next can recognise.

    Not the identifier alone: that carries the viewpoint the generator happened
    to pick, and the same goal over a map refined by a centimetre has a
    different one. What survives is the thing it was about and the place it was
    going -- see `scoring._the_same_goal`.
    """
    if preferred is None:
        return {}
    body = preferred["candidate"]
    return {"id": body["id"], "type": body["type"], "target": body["target"],
            "goal": body["constraints"].get("goal")}


def deliberate(store: store_mod.EpisodeStore,
               here: situation_mod.Situation,
               weights: scoring.Weights = scoring.DEFAULT, *,
               authority: bool = False,
               note: str = "") -> dict[str, Any]:
    """Consider, record, and hand back both the episode and what was decided."""
    began = time.time()
    prepare(store, here)
    got = scoring.consider(here, weights, authority=authority)
    took_s = time.time() - began

    generation = (here.world_generation
                  if here.world_generation != refs.UNKNOWN else None)
    episode = store.open_episode(
        TRIGGER, world_generation=generation, map_session=here.map_session,
        detail={"candidates": len(got["considered"]),
                "weights_version": weights.version,
                "authority": bool(authority)},
        note=note or ("a shadow decision: it chose what it would do and has no "
                      "way to do it"))

    inputs = store.snapshot("situation", here.as_dict())

    for one in got["considered"][:RECORD_LIMIT]:
        body = one["candidate"]
        store.append(episode, events.candidate(
            body["id"], _candidate_why(one),
            score=one["score"]["utility"],
            params={"type": body["type"], "action": body["action"],
                    "expects": body["expects"],
                    "travel_m": body["travel_m"], "time_s": body["time_s"],
                    "energy_wh": body["energy_wh"], "risk": body["risk"],
                    "gain_kind": body["gain_kind"],
                    "gain_value": body["gain_value"],
                    "gain_detail": body["gain_detail"],
                    "constraints": body["constraints"],
                    "score": one["score"],
                    "vetoes": one["vetoes"]},
            refs=body["refs"]))

    preferred = got["preferred"]
    chose = got["chose"] or "nothing"
    store.append(episode, events.decision(
        chose, _decision_why(got), inputs,
        rejected=[one["candidate"]["id"] for one in got["considered"]
                  if preferred is None or one is not preferred],
        refs=(preferred["candidate"]["refs"] if preferred else [])))

    store.append(episode, events.measured(
        "the deliberation",
        considered=len(got["considered"]),
        vetoed=sum(1 for one in got["considered"] if one["vetoes"]),
        deliberation_s=round(took_s, 3),
        # What the chosen goal would have cost, kept beside the decision so that
        # a reader is told the price of the thing that was wanted rather than
        # having to find it among the candidates.
        would_travel_m=(preferred["candidate"]["travel_m"] if preferred
                        else None),
        would_take_s=(preferred["candidate"]["time_s"] if preferred else None),
        would_gain=(preferred["score"]["gain"] if preferred else None),
        gate=[one["gate"] for one in got["gate"]] or None,
        battery_v=here.battery_v,
        pose=here.pose,
        world_at=inputs))

    store.close_episode(
        episode, "succeeded" if got["chose"] else "abandoned",
        detail=got["why_nothing"] or "acted on")

    # Left for the next deliberation, after the episode is closed rather than
    # before: a crash between opening and closing should leave the marks where
    # they were, so that the next one compares against a reading it actually
    # finished with.
    store.mark(SITUATION_MARK, inputs)
    store.mark(COOLED_MARK, _dumps(here.cooled))
    store.mark(GOAL_MARK, _dumps(_goal_of(preferred)))
    return {"episode": episode, "decision": got, "inputs": inputs,
            "deliberation_s": took_s}


def _candidate_why(one: dict[str, Any]) -> str:
    """One line about a candidate, with its refusal on it if it has one."""
    why = one["candidate"]["why"]
    if one["vetoes"]:
        return why + " -- refused: " + "; ".join(
            f"{veto['veto']}" for veto in one["vetoes"])
    if one["score"]["below_min_gain"]:
        return why + " -- under the minimum worth disturbing the rover for"
    return why


def _decision_why(got: dict[str, Any]) -> str:
    """The decision's own sentence: what it wanted, and what stopped it."""
    preferred = got["preferred"]
    if preferred is None:
        return got["why_nothing"]
    why = preferred["candidate"]["why"]
    if got["chose"]:
        return why
    # The gate's own words rather than the standalone sentence, which begins by
    # naming the goal again -- and this line has just named it.
    blocked = "; ".join(one["why"] for one in got["gate"]) or got["why_nothing"]
    return f"it would have chosen this -- {why} -- but {blocked}"


def render(got: dict[str, Any]) -> str:
    """The concise record: what it wanted, why, what it would cost, and why the
    better-looking options were refused.

    This is the answer to "the console or log can answer in one record", and it
    is deliberately short enough to read in a scrolling log.
    """
    decision = got["decision"]
    lines = []
    preferred = decision["preferred"]
    if preferred is None:
        lines.append("would do nothing: " + decision["why_nothing"])
    else:
        body, terms = preferred["candidate"], preferred["score"]
        lines.append(f"would {body['id']}: {body['why']}")
        lines.append(f"  expects {body['expects']}")
        lines.append(f"  costs {body['travel_m']} m and about "
                     f"{body['time_s']:.0f} s; gain {terms['gain']} "
                     f"({terms['gain_note']})")
        lines.append(f"  score {terms['utility']} = "
                     f"{terms['purpose_relevance']}*{terms['gain']} "
                     f"- time {terms['time_cost']} - travel "
                     f"{terms['travel_cost']} - switching "
                     f"{terms['switching_cost']} (weights "
                     f"{terms['weights_version']})")
    for one in decision["refused_above_it"]:
        lines.append(f"  {one['id']} scored higher at {one['utility']} and was "
                     f"refused -- {one['why']}")
    for one in decision["gate"]:
        lines.append(f"  cannot act: {one['gate']} -- {one['why']}")
    counts = (f"{len(decision['considered'])} candidates, "
              f"{sum(1 for one in decision['considered'] if one['vetoes'])} "
              f"vetoed, in {got['deliberation_s']:.2f} s")
    lines.append("  " + counts)
    return "\n".join(lines)


def _dumps(value: Any) -> str:
    return json.dumps(value, sort_keys=True, default=str)


def _loads(text: str) -> Any:
    try:
        return json.loads(text)
    except (ValueError, TypeError):
        return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Work out what the rover would do next. Decides, never acts.")
    parser.add_argument("--dir", default=None,
                        help="where to keep the record (default ~/.ugv/autonomy)")
    parser.add_argument("--no-record", action="store_true",
                        help="print the decision without writing an episode")
    parser.add_argument("--full", action="store_true",
                        help="list every candidate, not only the choice")
    args = parser.parse_args(argv)

    store = store_mod.EpisodeStore(args.dir)
    weights = scoring.Weights.load(args.dir)
    here = situation_mod.Situation.read(client_mod.ReadOnly())

    if args.no_record:
        began = time.time()
        decision = scoring.consider(here, weights)
        got = {"decision": decision, "deliberation_s": time.time() - began,
               "episode": ""}
    else:
        got = deliberate(store, here, weights)

    print(f"weights {weights.version} from {weights.source}")
    print(render(got))
    if args.full:
        print()
        for one in got["decision"]["considered"]:
            mark = "x" if one["vetoes"] else (
                "-" if one["score"]["below_min_gain"] else " ")
            print(f" {mark} {one['score']['utility']:>7.3f}  "
                  f"{one['candidate']['id']}")
            if one["vetoes"]:
                for veto in one["vetoes"]:
                    print(f"        {veto['veto']}: {veto['why']}")
    if got["episode"]:
        print()
        print(summary_mod.of(store, got["episode"],
                             live_world_generation=here.world_generation))
    store.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
