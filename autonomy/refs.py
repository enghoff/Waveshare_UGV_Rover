"""How an episode names a thing so that the name still means it next month.

An episode is a record of the rover deciding something, and every interesting
thing it can say points outwards: *this* entity was the goal, *that* picture is
why. The world state those names come from is not stable underneath them. Four
things happen to it, and only the last one is obvious:

1. **Clearing the semantic world resets the identifier counters.** `WorldStore.
   allocate` counts in a table and `clear` empties that table with everything
   else, so the next `object:8` is a different object from the last one. A stored
   reference does not dangle -- it quietly points at a stranger, which is worse.
2. **A merge deletes the losing entity.** `WorldStore.merge` moves the
   observations across and removes the row, so a reference to the loser resolves
   to nothing at all.
3. **A thing that was two objects gets taken apart.** The identity fault under
   investigation on 2026-09-08 means this is a live event and not a hypothetical:
   a reference then names part of what it used to name.
4. **The owner deletes evidence, or retention expires it.** The picture is gone
   and that has to read as gone.

The answer here has three parts, and the first is the only one that needs
anything from another component.

**Names carry the generation of the store that minted them.** A world reference
is `ws/<generation>/object:8`, where the generation is an opaque token the world
store mints when its database is created and mints again when it is cleared. A
reference whose generation is not the live one cannot resolve to a live row *at
all*, which is the point: it fails closed rather than pointing at a stranger.

That token does not exist yet. It is about ten lines in `world_state/store.py` --
one more `meta` row, written by `_create` and rewritten by `clear` -- and that
file belongs to the identity work going on beside this. Until it lands, an
episode records `UNKNOWN` and every reference minted against it is permanently
unresolvable, which is the honest answer rather than a convenient one.

**Evidence is named by what it contains.** A frame is `sha256:<hex>` of its own
bytes, never a filename or a row id. That is globally unique with no namespace to
negotiate, it survives every clear, and twenty episodes that looked at the same
picture hold one copy of it.

**Nothing here is used at replay time.** Replay reads the snapshot the episode
stored, never the live world; see `replay.py`. These references exist so that a
person, or a later query, can ask whether the thing an episode acted on is still
around -- and get "no" when it is not.
"""
from __future__ import annotations

import hashlib
import re
from typing import Any, NamedTuple

#: The world state's namespace, and this component's own.
WORLD = "ws"
AUTONOMY = "au"

#: Content addresses use their algorithm as the namespace, so a reference always
#: says how to check it.
DIGEST = "sha256"

#: What an episode records when the world state could not tell it which store it
#: was talking to. Deliberately not hex, so it can never collide with a real
#: generation, and deliberately not empty, so a reference that carries it still
#: parses and can still be reported.
UNKNOWN = "unknown"

#: A generation is sixteen hex characters from the operating system's entropy.
#: Long enough that two stores never collide, short enough to read out loud when
#: somebody is comparing a recording against a rover.
GENERATION = re.compile(r"^[0-9a-f]{16}$")

#: A local identifier is `kind:number`, which is what `WorldStore.allocate`
#: returns. The one character it must not contain is the separator.
LOCAL = re.compile(r"^[A-Za-z][A-Za-z0-9_]*:[0-9]+$")

_HEX = re.compile(r"^[0-9a-f]{64}$")


class Ref(NamedTuple):
    """A parsed reference: which store, which generation of it, which row.

    `generation` is None for a content address, because the bytes are their own
    namespace and there is nothing for a generation to disambiguate.
    """

    namespace: str
    generation: str | None
    local: str

    def __str__(self) -> str:
        if self.generation is None:
            return f"{self.namespace}:{self.local}"
        return f"{self.namespace}/{self.generation}/{self.local}"

    @property
    def is_digest(self) -> bool:
        return self.namespace == DIGEST

    @property
    def is_dated(self) -> bool:
        """True when this names a store generation that may since have gone."""
        return self.generation is not None and self.generation != UNKNOWN


def new_generation(entropy: bytes | None = None) -> str:
    """Mint a generation token. The world store's will be minted the same way."""
    import os

    return (entropy or os.urandom(8)).hex()[:16]


def world(generation: str | None, local: str) -> str:
    """Name a row in the semantic world state.

    A missing generation becomes `UNKNOWN` rather than an error, because the
    caller that has none is the ordinary case today and refusing to record the
    episode at all would lose more than it protects.
    """
    return str(Ref(WORLD, _generation(generation), _local(local)))


def episode(generation: str, number: int) -> str:
    """Name an episode in this component's own store.

    Episodes carry a generation for the same reason world rows do, and it is not
    the same one: this database is never cleared, but it can be deleted and
    started again, and an episode reference that outlived its database in
    somebody's notes should not come back pointing at a different episode.
    """
    return str(Ref(AUTONOMY, _generation(generation), _local(f"episode:{number}")))


def digest(data: bytes) -> str:
    """Name a picture, a depth map or a snapshot by what is in it."""
    return str(Ref(DIGEST, None, hashlib.sha256(data).hexdigest()))


def parse(ref: str) -> Ref | None:
    """The reference, or None if this is not one. Never raises.

    Never raising is deliberate. These strings come out of a database that is
    meant to outlive several versions of the code that wrote it, and a row that
    cannot be parsed should be reported as unreadable next to the rows that can,
    rather than take the whole reading down.
    """
    if not isinstance(ref, str) or not ref:
        return None
    if ref.startswith(DIGEST + ":"):
        rest = ref[len(DIGEST) + 1:]
        return Ref(DIGEST, None, rest) if _HEX.match(rest) else None
    parts = ref.split("/")
    if len(parts) != 3:
        return None
    namespace, generation, local = parts
    if namespace not in (WORLD, AUTONOMY):
        return None
    if generation != UNKNOWN and not GENERATION.match(generation):
        return None
    if not LOCAL.match(local):
        return None
    return Ref(namespace, generation, local)


def resolvable(ref: str, live_generation: str | None) -> bool:
    """Whether this reference may be looked up in the store that is live now.

    **The false answers are the useful ones.** A reference from a cleared store,
    or one that never knew which store it came from, is not a lookup that returns
    nothing -- it is a lookup that must not be attempted, because the identifier
    it holds has since been handed to something else.
    """
    parsed = parse(ref)
    if parsed is None or parsed.namespace != WORLD:
        return False
    if not parsed.is_dated or not live_generation:
        return False
    return parsed.generation == live_generation


def local_id(ref: str) -> str | None:
    """The bare `object:8` inside a reference, for showing a person.

    Callers that mean to *look something up* want `resolvable` first. This is for
    printing, where "object:8, from a world state that has since been cleared" is
    the sentence being built.
    """
    parsed = parse(ref)
    return None if parsed is None or parsed.is_digest else parsed.local


def generation_of(summary: dict[str, Any] | None) -> str:
    """Read the world store's generation off whatever it last told us.

    Written to look in more than one place because the field does not exist yet
    and its name is not settled; when it lands, this is the only thing that has
    to agree with it. An answer that is not a generation token is treated as no
    answer, so a world store that grows a differently-shaped field cannot get an
    episode stamped with something meaningless.
    """
    if not isinstance(summary, dict):
        return UNKNOWN
    for key in ("world_generation", "generation", "store_generation"):
        got = summary.get(key)
        if isinstance(got, str) and GENERATION.match(got):
            return got
    return UNKNOWN


def _generation(generation: str | None) -> str:
    if not generation or generation == UNKNOWN:
        return UNKNOWN
    if not GENERATION.match(generation):
        raise ValueError(f"not a generation token: {generation!r}")
    return generation


def _local(local: str) -> str:
    if not LOCAL.match(local or ""):
        raise ValueError(f"not a local identifier: {local!r}")
    return local
