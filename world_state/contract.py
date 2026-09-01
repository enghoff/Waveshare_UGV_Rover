"""What the model is asked for, and what is done to its answer before any of it
is believed.

Two rules shape everything here. The model proposes and the application disposes:
nothing that arrives in this file may allocate an identifier, name an entity that
was not put in front of it, or state where anything is in metres. And a malformed
answer is an ordinary outcome rather than an exception -- a 2B model asked for JSON
will sometimes produce prose, and that has to end in a diagnostic line and no
change to the world, not in a traceback in the process that owns the gimbal.

`validate` is therefore total: it returns a `Result` for every input, including
"none of that was usable", and the reasons come back with it so the popup can say
what went wrong instead of showing an inspection that silently did nothing.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Any

#: Bumped whenever the wording below, or the schema the model is held to, changes
#: in a way that could change what it reports. Stamped on every observation,
#: because an experiment that ran across a prompt change and cannot tell which half
#: is which has measured nothing.
PROMPT_VERSION = "3"

#: The semantic vocabulary, kept loose and practical on purpose. `room_hint`,
#: `text` and `hazard` are deliberately absent: nothing consumes them, and a
#: `hazard` label that reads as a safety signal while driving nothing at all is
#: worse than no label. A stair is an ordinary entity whose description happens to
#: say it is a stair.
KINDS = ("object", "furniture", "opening", "person", "unknown")

#: At most this many observations from one picture. A model that decides to
#: inventory every book on a shelf is answering a different question than the one
#: it was asked, and the store should not grow an entity per book while it does.
MAX_OBSERVATIONS = 10

#: Labels that name nothing in particular. An observation carrying one of these is
#: kept as history -- it is what the model said -- but no entity is created for it,
#: because "object:7 -- thing" is a row that can never be recognised again.
VAGUE = {"", "object", "thing", "things", "item", "items", "stuff", "unknown",
         "something", "some object", "n/a", "none", "-"}

#: Keys that would be the model measuring the room from one photograph. Stripped
#: before storage and reported, rather than quietly ignored: a model that keeps
#: offering metres is worth knowing about, and the day this schema grows a real
#: metric field it must not inherit a habit of accepting guesses in it.
METRIC_KEYS = {"x", "y", "z", "x_m", "y_m", "z_m", "map_x", "map_y", "map_z",
               "distance", "distance_m", "range_m", "depth_m", "position",
               "coordinates", "world_x", "world_y", "pose", "heading_deg"}

#: The other scale a bounding box arrives on. Qwen3-VL, which this model is a
#: fine-tune of, is trained to place things on a grid a thousand units across, and
#: asking for fractions does not reliably talk it out of that.
GRID = 1000.0
#: Above this, a box is on the grid rather than being a sloppy fraction. Two
#: rather than one so that a box hanging off the edge of the picture is still read
#: as the fractions it obviously is; a box on the grid is in the hundreds, so
#: there is nothing between the two to get wrong.
FRACTION_CEILING = 2.0

MAX_LABEL = 60
MAX_DESCRIPTION = 240
MAX_HINT = 40
MAX_SCENE = 400


@dataclass
class Seen:
    """One observation the model proposed, after validation.

    `existing_entity` is either None or an identifier that was genuinely in the
    list the model was shown -- there is no third case by the time anything holds
    one of these. `concrete` is whether this is worth an entity of its own if it is
    new, which is a question about the label rather than about the model's
    confidence in it.
    """

    kind: str
    label: str
    description: str = ""
    location_hint: str = ""
    bbox: list[float] | None = None
    existing_entity: str | None = None
    raw: dict[str, Any] = field(default_factory=dict)

    @property
    def concrete(self) -> bool:
        return self.label.strip().lower() not in VAGUE and len(self.label) >= 3


@dataclass
class Result:
    """One validated model answer.

    `seen` is what may be stored. `rejected` and `stripped` are why the rest was
    not, in sentences, because they end up in the diagnostics line that tells the
    difference between "the model found nothing" and "the model answered and we
    threw it away".
    """

    scene: str = ""
    seen: list[Seen] = field(default_factory=list)
    rejected: list[str] = field(default_factory=list)
    stripped: list[str] = field(default_factory=list)
    #: How many boxes arrived on the thousand grid rather than as fractions. A
    #: count rather than a complaint: it is normal for this model, it is corrected
    #: rather than refused, and it is worth being able to see change if the model
    #: or the prompt does.
    rescaled: int = 0
    raw: dict[str, Any] = field(default_factory=dict)
    error: str = ""

    @property
    def ok(self) -> bool:
        return not self.error

    def detail(self) -> str:
        """The one sentence a reader gets in the popup's diagnostics row."""
        if self.error:
            return self.error
        parts = []
        if self.rejected:
            parts.append("; ".join(self.rejected[:4]))
        if self.stripped:
            parts.append("ignored " + ", ".join(sorted(set(self.stripped))[:6])
                         + " from the model")
        if self.rescaled:
            parts.append(f"{self.rescaled} box(es) came back on the 0-1000 grid "
                         f"and were divided down")
        return " -- ".join(parts)


#: The shape the model is constrained to when the backend can constrain it.
#: llama.cpp turns this into a grammar, which is what stops a 2B model from
#: answering in prose and is the cheapest of the three defences against runaway
#: generation (the others being a token cap and a wall clock).
#:
#: **The lengths are load-bearing, and that is a finding from the rover.** A
#: grammar built without them constrains the shape and not the size, and this
#: model used that freedom to write three and a half thousand characters of essay
#: into `scene` -- running out of tokens before it closed the object, so a perfectly
#: good look at a room was thrown away as truncated. llama.cpp turns `maxLength`
#: into part of the grammar, so the string simply ends and the object closes.
RESPONSE_SCHEMA = {
    "type": "object",
    "properties": {
        "scene": {"type": "string", "maxLength": 300},
        "observations": {
            "type": "array",
            "maxItems": MAX_OBSERVATIONS,
            "items": {
                "type": "object",
                "properties": {
                    "existing_entity": {"type": ["string", "null"]},
                    "kind": {"type": "string", "enum": list(KINDS)},
                    "label": {"type": "string", "maxLength": 40},
                    "description": {"type": "string", "maxLength": 160},
                    "location_hint": {"type": "string", "maxLength": 24},
                    "bbox_norm": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 4, "maxItems": 4,
                    },
                },
                "required": ["kind", "label", "description", "bbox_norm"],
            },
        },
    },
    "required": ["scene", "observations"],
}


def build_prompt(known: list[dict[str, Any]]) -> str:
    """What the model is asked, with the entities it has already been given names
    for.

    The known list is the whole of the identity experiment. The model is not asked
    "what is this?" but "is this one of these?", and it can only answer with a name
    from the list or with nothing -- which is what makes a duplicate entity a
    result rather than a bug in the prompt.
    """
    lines = [
        "You are the perception layer of a small indoor robot. You are looking at "
        "one photograph taken by the robot's camera.",
        "",
        "List the things in this picture that a person would still care about "
        "tomorrow: furniture, doorways, people, and objects that stay where they "
        "are put. Ignore floor texture, reflections, lighting and anything you "
        "cannot name.",
        "",
        "Answer with JSON only, in this shape:",
        json.dumps({"scene": "one sentence about the room",
                    "observations": [{"existing_entity": "object:12 or null",
                                      "kind": "|".join(KINDS),
                                      "label": "short name",
                                      "description": "one clause",
                                      "location_hint": "ahead-left",
                                      "bbox_norm": ["left", "top", "right",
                                                    "bottom"]}]},
                   indent=2),
        "",
        # Asked for on the model's own scale rather than on ours. Cosmos Reason 2
        # is a Qwen3-VL fine-tune and places things on a grid a thousand units
        # across; asked for fractions it answered on the thousand grid anyway
        # about half the time, so the prompt has been moved to meet it and
        # `_bbox` reads both. The four words rather than four numbers are for a
        # separate reason measured on the rover: given an example box of real
        # numbers, this model copies it verbatim into every observation.
        "bbox_norm is [left, top, right, bottom] in image coordinates on a grid "
        "1000 wide and 1000 high, so the whole picture is [0, 0, 1000, 1000]. "
        "Give a different box for each thing.",
        "",
        "Do not estimate distances, sizes in metres, or positions on a map: you "
        "cannot measure those from one photograph and any you give will be "
        "discarded.",
        "",
    ]
    if known:
        lines.append("The robot has already given names to these things. If "
                     "something in this picture is one of them, put its exact "
                     "name in existing_entity; otherwise use null. Never invent a "
                     "name that is not on this list.")
        lines.append("")
        for entity in known:
            described = entity.get("canonical_description") or entity.get("label")
            lines.append(f"- {entity['id']} -- {entity.get('label', '')}"
                         f"{': ' + described if described else ''}")
    else:
        lines.append("The robot has not named anything yet, so existing_entity is "
                     "null for everything in this picture.")
    lines.append("")
    lines.append(f"At most {MAX_OBSERVATIONS} observations. JSON only, no other "
                 f"text.")
    return "\n".join(lines)


def extract_json(text: str) -> tuple[dict[str, Any] | None, str]:
    """The model's answer as an object, whatever it wrapped it in.

    Three things get in the way of `json.loads` in practice and all three are
    ordinary: a fenced code block, a sentence of preamble, and -- for a reasoning
    model like this one -- a block of thinking before the answer. The thinking is
    dropped here and never reaches the store: this experiment records what the
    model said about the room, and a saved chain of thought is neither a fact about
    the room nor something anything downstream should learn to read.
    """
    if not isinstance(text, str) or not text.strip():
        return None, "the model answered with nothing"
    cleaned = re.sub(r"<think>.*?</think>", " ", text, flags=re.S | re.I)
    cleaned = re.sub(r"^.*?</think>", " ", cleaned, flags=re.S | re.I)
    fenced = re.search(r"```(?:json)?\s*(.+?)```", cleaned, flags=re.S)
    if fenced:
        cleaned = fenced.group(1)
    try:
        payload = json.loads(cleaned)
    except ValueError:
        payload = None
    if payload is None:
        start = cleaned.find("{")
        if start < 0:
            return None, "the model's answer contained no JSON object"
        depth, end, in_string, escaped = 0, -1, False, False
        for at in range(start, len(cleaned)):
            char = cleaned[at]
            if in_string:
                if escaped:
                    escaped = False
                elif char == "\\":
                    escaped = True
                elif char == '"':
                    in_string = False
                continue
            if char == '"':
                in_string = True
            elif char == "{":
                depth += 1
            elif char == "}":
                depth -= 1
                if depth == 0:
                    end = at + 1
                    break
        if end < 0:
            return None, ("the model's JSON was cut off before it closed, which is "
                          "what running out of tokens looks like")
        try:
            payload = json.loads(cleaned[start:end])
        except ValueError as error:
            return None, f"the model's answer was not JSON: {error}"
    if not isinstance(payload, dict):
        return None, "the model answered with JSON that was not an object"
    return payload, ""


def validate(payload: Any, known_ids) -> Result:
    """One model answer, reduced to what may be stored.

    Total by design: every input produces a `Result`, and one whose `seen` list is
    empty is a perfectly good answer -- "nothing salient in this picture" is a
    finding. What separates it from a failure is `error`, which is set only when
    the answer itself could not be read.
    """
    known = set(known_ids or ())
    if not isinstance(payload, dict):
        return Result(error="the model's answer was not an object")
    result = Result(raw=payload)
    scene = payload.get("scene") or payload.get("summary") or ""
    result.scene = _text(scene, MAX_SCENE)

    items = payload.get("observations")
    if items is None:
        result.error = "the model's answer had no observations list"
        return result
    if not isinstance(items, list):
        result.error = "the model's observations were not a list"
        return result
    if len(items) > MAX_OBSERVATIONS:
        result.rejected.append(
            f"the model returned {len(items)} observations; kept the first "
            f"{MAX_OBSERVATIONS}")
        items = items[:MAX_OBSERVATIONS]

    for index, item in enumerate(items):
        seen, why = _one(item, known, result)
        if seen is None:
            result.rejected.append(f"observation {index + 1}: {why}")
        else:
            result.seen.append(seen)
    return result


def _one(item: Any, known: set, result: Result) -> tuple[Seen | None, str]:
    if not isinstance(item, dict):
        return None, "not an object"
    for key in item:
        if key.lower() in METRIC_KEYS:
            result.stripped.append(key)

    label = _text(item.get("label"), MAX_LABEL)
    if not label:
        return None, "no label, so there is nothing to record"

    kind = str(item.get("kind") or "").strip().lower()
    if kind not in KINDS:
        # Coerced rather than refused. The kind is a filing convenience and the
        # label is the content; throwing away a perfectly good "grey sofa" because
        # the model said "seating" would lose the observation to protect a
        # vocabulary that exists only to group rows in a list.
        if kind:
            result.stripped.append(f"kind={kind}")
        kind = "unknown"

    reference = item.get("existing_entity")
    if reference is not None:
        if not isinstance(reference, str) or reference.strip() not in known:
            # Refused, not created. An identifier the model was never shown is
            # either invention or a stale memory of an earlier conversation, and
            # auto-creating an entity for it would let the model choose its own
            # names by the back door.
            return None, (f"named {reference!r}, which was not in the list it was "
                          f"shown")
        reference = reference.strip()

    return Seen(kind=kind, label=label,
                description=_text(item.get("description"), MAX_DESCRIPTION),
                location_hint=_text(item.get("location_hint"), MAX_HINT),
                bbox=_bbox(item.get("bbox_norm"), result),
                existing_entity=reference,
                raw=_clean(item)), ""


def _bbox(value: Any, result: Result) -> list[float] | None:
    """Four fractions of the picture, or nothing.

    **The scale has to be worked out rather than trusted, and that is a finding
    from the rover rather than a precaution.** Asked for fractions between zero and
    one, Cosmos Reason 2 answers on the 0-1000 grid its Qwen3-VL base was trained
    to use about as often as it answers in fractions. Both are readable and neither
    is ambiguous -- a picture is one unit across, so anything past one is not a
    fraction -- so a box whose numbers all fit the thousand grid is divided down
    and the rescaling is reported.

    Clamped after that rather than refused, because a box hanging a few percent off
    the edge of the frame is a model being approximate about something real, and
    the box is only ever used to draw a rectangle and to nudge a bearing. A box
    that is inside out or has no area is dropped and the observation kept: the
    label is the finding, and the rectangle is how you check it.
    """
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        result.rejected.append("a bounding box that was not four numbers was dropped")
        return None
    try:
        numbers = [float(number) for number in value]
    except (TypeError, ValueError):
        result.rejected.append("a bounding box that was not numeric was dropped")
        return None
    if any(number != number for number in numbers):  # NaN is never equal to itself
        result.rejected.append("a bounding box containing NaN was dropped")
        return None
    if max(numbers) > FRACTION_CEILING:
        if min(numbers) < -GRID * 0.05 or max(numbers) > GRID * 1.05:
            result.rejected.append(
                "a bounding box on no scale this understands was dropped")
            return None
        numbers = [number / GRID for number in numbers]
        result.rescaled += 1
    clamped = [min(1.0, max(0.0, number)) for number in numbers]
    left, top, right, bottom = clamped
    if right <= left or bottom <= top:
        result.rejected.append("a bounding box with no area was dropped")
        return None
    return [round(number, 4) for number in clamped]


def _clean(item: dict[str, Any]) -> dict[str, Any]:
    """The model's own words for this observation, minus anything it should not
    have measured. Kept verbatim otherwise, because "what did Cosmos actually say"
    is a question the popup has to be able to answer."""
    return {key: value for key, value in item.items()
            if key.lower() not in METRIC_KEYS}


def _text(value: Any, limit: int) -> str:
    if value is None:
        return ""
    text = str(value).strip()
    text = re.sub(r"\s+", " ", text)
    return text[:limit]
