"""Rebuilding what happened, from the record and nothing else.

**This module cannot touch the rover, and that is structural rather than
careful.** It imports the store, the reference vocabulary and the standard
library; there is no client, no socket and no daemon call anywhere in the path,
so a replay that wanted to drive the rover would have nothing to drive it with.
`test_replay.py` reads the module and checks the imports, because "we were
careful" is the kind of guarantee that stops being true in six months.

The same property is what makes a reconstruction independent of time. Every
decision recorded a snapshot of what it was made from, so replaying it a month
later reads that snapshot and not today's world -- the entity it chose may since
have been merged, split, or had its identifier handed to something else by a
clear, and none of that changes what the episode says the rover did.

What a reader gets told, and what they are never told:

- What is missing is named, with which kind of missing it is. Evidence the owner
  deleted reads as deleted, with the reason they gave; evidence that has simply
  gone reads as absent. Neither is ever quietly replaced by a newer record that
  happens to share a local identifier, because a stored reference carries the
  generation of the store that minted it.
- A correction is shown next to what it corrects, never instead of it.
- An annotation added after the episode closed is marked as such, so that
  hindsight cannot be mistaken for what was known at the time.
"""
from __future__ import annotations

from typing import Any, Iterator

import events as events_mod
import refs as refs_mod
import store as store_mod


def reconstruct(store: store_mod.EpisodeStore, episode_ref: str, *,
                live_world_generation: str | None = None) -> dict[str, Any]:
    """Everything the record can say about one episode.

    `live_world_generation` is optional and changes nothing about the
    reconstruction; it decides only whether each world reference is reported as
    something that could still be looked up today.
    """
    episode = store.episode(episode_ref)
    steps = list(_steps(store, episode))
    decision = _decision(store, steps)
    missing = _missing(store, steps, decision)
    return {
        "ref": episode["ref"],
        "number": episode["number"],
        "opened_at": episode["opened_at"],
        "trigger": episode["trigger"],
        "trigger_detail": episode["trigger_detail"],
        "world_generation": episode["world_generation"],
        "map_session": episode["map_session"],
        "note": episode["note"],
        "steps": steps,
        "decision": decision,
        "calls": [step["body"] for step in steps if step["kind"] == "call"],
        "dispatches": [step["body"] for step in steps if step["kind"] == "dispatch"],
        "candidates": [step["body"] for step in steps
                       if step["kind"] == "candidate"],
        "models": [step["body"] for step in steps if step["kind"] == "model"],
        "outcome": _outcome(steps),
        "evidence": _evidence(store, steps),
        "references": _references(store, episode, steps, live_world_generation),
        "annotations": [step for step in steps if step["after_close"]],
        "missing": missing,
        "replayable": not missing,
    }


def play(store: store_mod.EpisodeStore,
         episode_ref: str) -> Iterator[dict[str, Any]]:
    """Walk the episode one step at a time, in the order it happened.

    Deterministic because it is a read: the same database yields the same steps
    in the same order however often it is played, and playing it does not write
    anything, so a replay cannot change what the next replay sees.
    """
    yield from _steps(store, store.episode(episode_ref))


def selected_action(store: store_mod.EpisodeStore,
                    episode_ref: str) -> dict[str, Any] | None:
    """The action the episode chose, its parameters and what came of it.

    The narrow question the acceptance criterion asks -- can the selected
    action, its parameters and its result be rebuilt from stored records alone --
    answered without making the caller read a whole reconstruction.
    """
    got = reconstruct(store, episode_ref)
    if got["decision"] is None:
        return None
    chose = got["decision"]["chose"]
    # The first call after the decision, found by where it sits rather than by
    # what it is called. The decision names a goal in the executive's own words
    # -- `look_at(object:8)` -- and the call names a daemon entry point, and
    # neither end promises those will go on matching. Tying the record together
    # by string would mean a rename somewhere else quietly emptying every
    # reconstruction.
    after = [step for step in got["steps"]
             if step["kind"] == "call" and step["seq"] > got["decision"]["seq"]
             and not step["after_close"]]
    if after:
        body = after[0]["body"]
        return {"action": chose, "called": True, "call": body.get("call"),
                "params": body.get("params"), "ok": body.get("ok"),
                "result": body.get("result"), "error": body.get("error", ""),
                "outcome": got["outcome"]}
    if got["dispatches"]:
        body = got["dispatches"][0]
        return {"action": chose, "called": None, "call": body["call"],
                "params": body["params"], "ok": None, "result": None,
                "error": "dispatch was prepared but its outcome was not recorded",
                "outcome": got["outcome"]}
    # Chosen and never called. A shadow run does this every time, and it is a
    # result rather than a gap: the rover decided what it would do and had no
    # authority to do it.
    return {"action": chose, "called": False, "call": None, "params": None,
            "ok": None, "result": None, "error": "", "outcome": got["outcome"]}


# --- the pieces --------------------------------------------------------------

def _steps(store: store_mod.EpisodeStore,
           episode: dict[str, Any]) -> Iterator[dict[str, Any]]:
    events = episode["events"]
    closed_at = next((one["seq"] for one in events
                      if one["kind"] == events_mod.CLOSED), None)
    corrections: dict[int, list[int]] = {}
    for one in events:
        if one["corrects"] is not None:
            corrections.setdefault(one["corrects"], []).append(one["seq"])
    for one in events:
        yield {**one,
               "corrected_by": corrections.get(one["seq"], []),
               "after_close": closed_at is not None and one["seq"] > closed_at}


def _decision(store: store_mod.EpisodeStore,
              steps: list[dict[str, Any]]) -> dict[str, Any] | None:
    """The last decision in the episode, with the world it was made from.

    The last rather than the first: an episode that decided twice changed its
    mind, and what it acted on is the second one. Both are still in `steps`.
    """
    chosen = [step for step in steps if step["kind"] == "decision"]
    if not chosen:
        return None
    body = chosen[-1]["body"]
    digest = body.get("inputs", "")
    return {"seq": chosen[-1]["seq"], "chose": body.get("chose"),
            "why": body.get("why"), "rejected": body.get("rejected", []),
            "inputs_digest": digest,
            "inputs": store.snapshot_body(digest)}


def _outcome(steps: list[dict[str, Any]]) -> dict[str, Any] | None:
    for step in steps:
        if step["kind"] == events_mod.CLOSED:
            return step["body"]
    return None


def _evidence(store: store_mod.EpisodeStore,
              steps: list[dict[str, Any]]) -> list[dict[str, Any]]:
    seen: dict[str, dict[str, Any]] = {}
    for step in steps:
        for digest in step["evidence"]:
            if digest not in seen:
                seen[digest] = {**store.evidence_state(digest),
                                "first_seen_at_step": step["seq"]}
    return list(seen.values())


def _references(store: store_mod.EpisodeStore, episode: dict[str, Any],
                steps: list[dict[str, Any]],
                live: str | None) -> list[dict[str, Any]]:
    """Every durable name the episode used, and what became of it.

    `resolvable` is the question worth asking and it is usually answered no. A
    reference minted before the world store could say which generation it was --
    which is every reference recorded until that lands -- can never be looked up,
    and a reference from a store that has since been cleared must not be, because
    its identifier has been handed to something else.
    """
    out: dict[str, dict[str, Any]] = {}
    for step in steps:
        for ref in step["refs"]:
            if ref in out:
                continue
            parsed = refs_mod.parse(ref)
            became = store.now_called(ref)
            out[ref] = {
                "ref": ref,
                "local": refs_mod.local_id(ref),
                "generation": None if parsed is None else parsed.generation,
                "readable": parsed is not None,
                "resolvable": refs_mod.resolvable(ref, live),
                "now_called": [] if became == [ref] else became,
                "why_not": _why_not(parsed, live),
            }
    return list(out.values())


def _why_not(parsed: refs_mod.Ref | None, live: str | None) -> str:
    if parsed is None:
        return "not a reference this build can read"
    if parsed.namespace != refs_mod.WORLD:
        return ""
    if not parsed.is_dated:
        return ("recorded before the world state could say which store it was, "
                "so the identifier cannot safely be looked up")
    if not live:
        return "the world state's generation is not known here"
    if parsed.generation != live:
        return ("belongs to a world state that has since been cleared; the "
                "identifier has been reissued")
    return ""


def _missing(store: store_mod.EpisodeStore, steps: list[dict[str, Any]],
             decision: dict[str, Any] | None) -> list[dict[str, Any]]:
    """What this episode needs and no longer has.

    An episode is replayable when this list is empty, and the list is the honest
    part: a deleted picture is reported with the reason it was deleted, and a
    replay does not go looking for something newer to stand in for it.
    """
    out = []
    if decision is not None and decision["inputs"] is None:
        out.append({"what": "snapshot", "digest": decision["inputs_digest"],
                    "state": store_mod.ABSENT,
                    "why": "the decision inputs are not in this database"})
    for held in _evidence(store, steps):
        if held["state"] != store_mod.HELD:
            out.append({"what": "evidence", "digest": held["digest"],
                        "state": held["state"],
                        "why": held.get("why", ""),
                        "at": held.get("at")})
    return out
