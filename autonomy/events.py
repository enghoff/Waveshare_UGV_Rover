"""What an episode is allowed to say, and the fields each kind has to carry.

An episode is only worth keeping if it can be read back by somebody who was not
there, so the shapes are fixed here rather than left to whatever the caller
happened to put in a dictionary. A malformed event raises where it is recorded,
which is the moment somebody can still fix it -- the alternative is finding out
at replay, a fortnight later, that the one episode worth understanding is the one
that recorded a goal without saying why.

**A call event carries the answer as well as the question.** That is what lets
`replay.py` reconstruct a run without touching the rover: the result is already
in the record, so there is nothing for a replay to go and ask.

The fields come from the episode requirements in the Phase 1 plan. Where a field
is optional it is because the rover genuinely may not have it -- there is no
battery reading when the driver board has not answered, and asking for one anyway
would mean recording a zero that reads like a flat battery.
"""
from __future__ import annotations

from typing import Any, Iterable, NamedTuple

#: Every kind of event, and the keys its body must have. Extra keys are allowed
#: and kept: a caller with more to say about a decision should say it, and a
#: schema that refused would just push the detail into a note nobody parses.
REQUIRED: dict[str, tuple[str, ...]] = {
    # A goal the rover considered, whether or not it chose it. Recorded even
    # when it loses, because "why did it not go and look at the thing in the
    # hall" is the question a shadow run exists to answer.
    "candidate": ("goal", "why"),
    # The choice, and what it was made from. `inputs` is the digest of the
    # snapshot in `snapshots`, never a live query -- see `store.snapshot`.
    "decision": ("chose", "why", "inputs"),
    # A call the executive asked the daemon for, and what came back. `ok` is the
    # daemon's own verdict; `result` is whatever it returned.
    "call": ("call", "params", "ok"),
    # A model call, so that a decision made by a model can be attributed to a
    # particular one. Versions change under us and the episode has to say which
    # one was answering.
    "model": ("provider", "model", "purpose"),
    # What the attempt did to the world state. The references are the durable
    # kind; the counts are what makes a change visible at a glance.
    "world_change": ("what",),
    # Timing, travel and battery, whichever of them existed.
    "measured": ("what",),
    # Free text, and the only kind a person is expected to write by hand. Also
    # what a correction is: set `corrects` and say what was wrong.
    "note": ("text",),
    # How the episode ended. Exactly one of these per episode, appended by
    # `store.close_episode`, and where the outcome is read back from.
    "closed": ("outcome",),
}

#: The kind that ends an episode. Named here because both the store and the
#: replay have to agree on it and neither should spell it out again.
CLOSED = "closed"

#: Outcomes an episode may close with. `abandoned` is not a failure: a shadow run
#: that decided not to act is the ordinary case in Phase 1 and must not be
#: counted as something going wrong.
OUTCOMES = ("succeeded", "failed", "abandoned", "interrupted")


class Event(NamedTuple):
    """One thing that happened, with the durable names of what it touched.

    `refs` and `evidence` are kept out of the body rather than buried in it so
    that a reader -- or a retention pass looking for what an episode still needs
    -- can find every reference in the database without knowing the shape of
    every body it might be inside.
    """

    kind: str
    body: dict[str, Any]
    refs: tuple[str, ...] = ()
    evidence: tuple[str, ...] = ()
    corrects: int | None = None


def make(kind: str, body: dict[str, Any], *, refs: Iterable[str] = (),
         evidence: Iterable[str] = (), corrects: int | None = None) -> Event:
    """Build an event, refusing one that could not be read back."""
    validate(kind, body)
    return Event(kind, dict(body), tuple(refs), tuple(evidence), corrects)


def validate(kind: str, body: dict[str, Any]) -> None:
    """Raise unless this body can stand as that kind of event."""
    if kind not in REQUIRED:
        raise ValueError(f"no such event kind: {kind!r}")
    if not isinstance(body, dict):
        raise ValueError(f"{kind} body must be a dict, not {type(body).__name__}")
    missing = [name for name in REQUIRED[kind] if name not in body]
    if missing:
        raise ValueError(f"{kind} event is missing {', '.join(missing)}")
    if kind == CLOSED and body["outcome"] not in OUTCOMES:
        raise ValueError(f"outcome must be one of {OUTCOMES}, not "
                         f"{body['outcome']!r}")


# --- the shapes, spelled out ------------------------------------------------
#
# Thin wrappers over `make`, and they earn their place by being the only list of
# what a caller is expected to record. An executive written against these cannot
# forget to record the losing candidates, because there is a function for them.

def candidate(goal: str, why: str, *, score: float | None = None,
              params: dict[str, Any] | None = None,
              refs: Iterable[str] = ()) -> Event:
    body: dict[str, Any] = {"goal": goal, "why": why}
    if score is not None:
        body["score"] = score
    if params:
        body["params"] = params
    return make("candidate", body, refs=refs)


def decision(chose: str, why: str, inputs: str, *,
             rejected: Iterable[str] = (), refs: Iterable[str] = ()) -> Event:
    """`inputs` is a snapshot digest. Passing a live dictionary here is the
    mistake this component exists to prevent, so it is typed as a string."""
    if not isinstance(inputs, str) or not inputs.startswith("sha256:"):
        raise ValueError("decision inputs must be a snapshot digest; snapshot "
                         "the world first with store.snapshot()")
    body: dict[str, Any] = {"chose": chose, "why": why, "inputs": inputs}
    if rejected:
        body["rejected"] = list(rejected)
    return make("decision", body, refs=refs)


def call(name: str, params: dict[str, Any], *, ok: bool,
         result: Any = None, error: str = "", duration_s: float | None = None,
         refs: Iterable[str] = ()) -> Event:
    body: dict[str, Any] = {"call": name, "params": params, "ok": bool(ok)}
    if result is not None:
        body["result"] = result
    if error:
        body["error"] = error
    if duration_s is not None:
        body["duration_s"] = duration_s
    return make("call", body, refs=refs)


def model(provider: str, name: str, purpose: str, *,
          duration_s: float | None = None,
          tokens: dict[str, Any] | None = None) -> Event:
    body: dict[str, Any] = {"provider": provider, "model": name,
                            "purpose": purpose}
    if duration_s is not None:
        body["duration_s"] = duration_s
    if tokens:
        body["tokens"] = tokens
    return make("model", body)


def world_change(what: str, *, created: Iterable[str] = (),
                 matched: Iterable[str] = (), refs: Iterable[str] = (),
                 evidence: Iterable[str] = ()) -> Event:
    body: dict[str, Any] = {"what": what}
    if created:
        body["created"] = list(created)
    if matched:
        body["matched"] = list(matched)
    return make("world_change", body, refs=refs, evidence=evidence)


def measured(what: str, **values: Any) -> Event:
    """Timing, travel, battery -- whichever of them the rover could answer.

    A value that is None is dropped rather than stored, because a null battery
    and a flat one must not read alike.
    """
    body: dict[str, Any] = {"what": what}
    body.update({k: v for k, v in values.items() if v is not None})
    return make("measured", body)


def note(text: str, *, corrects: int | None = None,
         refs: Iterable[str] = ()) -> Event:
    return make("note", {"text": text}, refs=refs, corrects=corrects)
