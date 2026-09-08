"""The names: that a stale one fails closed rather than pointing at a stranger.

The fault being defended against is not a dangling reference, which is loud and
harmless. It is the quiet one: the world state's identifier counters restart when
the store is cleared, so `object:8` recorded last week and `object:8` today are
different objects with the same name, and an episode that resolved the old name
against the new store would produce a confident, wrong account of what the rover
did. Every check here is about that.
"""
from __future__ import annotations

from test_harness import check
import refs


A = "9f2a1c04ffab3d21"
B = "0011223344556677"


def test_a_name_carries_the_store_that_minted_it() -> None:
    """Which is the whole mechanism; everything else follows from it."""
    check("a world reference names its generation",
          refs.world(A, "object:8"), f"ws/{A}/object:8")
    check("...and parses back", refs.parse(f"ws/{A}/object:8"),
          refs.Ref("ws", A, "object:8"))
    check("an episode reference is shaped the same way",
          refs.episode(A, 12), f"au/{A}/episode:12")


def test_the_same_local_name_in_two_stores_is_two_names() -> None:
    """The clear of 07:20 on 2026-09-08 is what this is about.

    That morning's acceptance drive ran in a store cleared at 07:20, so its
    `object:8` is a different object from the `object:8` in the previous day's
    recording. Two episodes holding the plain local name would be
    indistinguishable; holding these, they cannot be confused.
    """
    yesterday, today = refs.world(A, "object:8"), refs.world(B, "object:8")
    check("two stores, two names", yesterday == today, False)
    check("today's name resolves against today's store",
          refs.resolvable(today, B), True)
    check("yesterday's does not, although it reads the same",
          refs.resolvable(yesterday, B), False)
    check("...and both still print as object:8 for a person",
          (refs.local_id(yesterday), refs.local_id(today)),
          ("object:8", "object:8"))


def test_a_name_from_an_unknown_store_can_never_be_looked_up() -> None:
    """The state every reference is in until `world_state` mints a generation.

    Permanently unresolvable rather than provisionally: nothing later can tell us
    which store an episode recorded before the store could say, so filling it in
    afterwards would be a guess dressed as a record.
    """
    nameless = refs.world(None, "object:8")
    check("an unknown generation is recorded, not refused",
          nameless, "ws/unknown/object:8")
    check("...and never resolves, whatever store is live",
          refs.resolvable(nameless, A), False)
    check("...not even against a store that calls itself unknown",
          refs.resolvable(nameless, refs.UNKNOWN), False)
    check("...and it is visibly not dated",
          refs.parse(nameless).is_dated, False)


def test_nothing_resolves_when_the_live_store_is_unknown() -> None:
    """A rover that cannot say which world it is running is not a rover to
    resolve anything against."""
    check("a good name against no live store",
          refs.resolvable(refs.world(A, "object:8"), None), False)
    check("...or against an empty one",
          refs.resolvable(refs.world(A, "object:8"), ""), False)


def test_evidence_is_named_by_what_is_in_it() -> None:
    check("the same bytes give the same name",
          refs.digest(b"a picture") == refs.digest(b"a picture"), True)
    check("different bytes do not",
          refs.digest(b"a picture") == refs.digest(b"another"), False)
    check("and it says how to check it",
          refs.digest(b"a picture").startswith("sha256:"), True)
    check("a digest has no generation, because bytes need none",
          refs.parse(refs.digest(b"x")).generation, None)


def test_reading_a_reference_never_raises() -> None:
    """These strings come out of a database meant to outlive the code that wrote
    it, so an unreadable one is reported beside the readable ones."""
    for rubbish in ("", "object:8", "ws/object:8", "ws//object:8", "a/b/c",
                    "ws/NOTHEX0123456789/object:8", "ws/" + A + "/object",
                    "ws/" + A + "/ob ject:8", "sha256:zz", None, 7,
                    "ws/" + A + "/object:8/extra"):
        check(f"{rubbish!r} is not a reference", refs.parse(rubbish), None)


def test_a_generation_is_read_off_what_the_world_state_reported() -> None:
    """And an answer that is not a generation is treated as no answer, so that a
    world state which grows a differently shaped field cannot stamp an episode
    with something meaningless."""
    check("the field, when it is there",
          refs.generation_of({"world_generation": A}), A)
    check("an older name for it", refs.generation_of({"generation": A}), A)
    check("no summary at all", refs.generation_of(None), refs.UNKNOWN)
    check("a summary without one", refs.generation_of({"entities": 4}),
          refs.UNKNOWN)
    check("something that is not a token",
          refs.generation_of({"world_generation": 7}), refs.UNKNOWN)
    check("...or is the wrong shape",
          refs.generation_of({"world_generation": "session-3"}), refs.UNKNOWN)


def test_a_generation_is_minted_from_the_operating_system() -> None:
    check("sixteen hex characters",
          bool(refs.GENERATION.match(refs.new_generation())), True)
    check("and two of them differ",
          refs.new_generation() == refs.new_generation(), False)


def test_a_bad_name_is_refused_where_it_is_minted() -> None:
    """Loudly, at the moment somebody can still fix it."""
    for bad in ("object", "object:", ":8", "8", "object:eight"):
        try:
            refs.world(A, bad)
            check(f"minting {bad!r} is refused", "allowed", "refused")
        except ValueError:
            check(f"minting {bad!r} is refused", "refused", "refused")
    try:
        refs.world("not-a-generation", "object:8")
        check("minting against a bad generation is refused", "allowed", "refused")
    except ValueError:
        check("minting against a bad generation is refused", "refused", "refused")


TESTS = (
    test_a_name_carries_the_store_that_minted_it,
    test_the_same_local_name_in_two_stores_is_two_names,
    test_a_name_from_an_unknown_store_can_never_be_looked_up,
    test_nothing_resolves_when_the_live_store_is_unknown,
    test_evidence_is_named_by_what_is_in_it,
    test_reading_a_reference_never_raises,
    test_a_generation_is_read_off_what_the_world_state_reported,
    test_a_generation_is_minted_from_the_operating_system,
    test_a_bad_name_is_refused_where_it_is_minted,
)
