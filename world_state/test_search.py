"""Search: comparing a description against what the rover actually saw.

A search that answers confidently from too little evidence is worse than one that
says it has barely looked, so what is checked is that a flat field decides
nothing, and that rows from the other backend are counted out rather than ranked.
"""
from __future__ import annotations

import tempfile

from test_harness import check
from test_fakes import a_store, observe
from world_state import search


# --- finding a thing by describing it ----------------------------------------


def _packed(*values):
    import struct
    return struct.pack(f"<{len(values)}f", *values)


def test_a_query_that_matches_nothing_says_so() -> None:
    """The part that matters, and the part a ranking alone cannot do.

    A list of scores always has a top, so the question is whether that top means
    anything. Measured on the rover it is the raw score that answers this and not
    the separation, so a field of near misses is not a match however flat it is.
    """
    query = _packed(1.0, 0.0, 0.0)
    # Everything here scores about 0.05 against the query: the shape a room full
    # of things that are not what was asked for produces.
    near = [{"id": n, "siglip_blob": _packed(0.05 + n * 0.0002, 1.0, 0.0)}
            for n in range(40)]
    answer = search.rank(query, near)
    check("a field of near misses is ranked", len(answer["matches"]), 10)
    check("...but not believed", answer["confident"], False)
    check("...and the reason is in words a person can read",
          "nothing here matches" in answer["detail"], True)
    check("...which quotes the score and the bar it missed",
          f"{search.MATCHES:.2f}" in answer["detail"], True)

    real = near + [{"id": 99, "siglip_blob": _packed(1.0, 0.02, 0.0)}]
    answer = search.rank(query, real)
    check("a match that scores well is believed", answer["confident"], True)
    check("...and it is the right one", answer["matches"][0]["observation_id"], 99)
    check("...with the score behind the verdict shown",
          answer["best"] >= search.MATCHES, True)


def test_a_search_says_which_part_of_the_frame_it_found() -> None:
    """A picture of a room is not an answer.

    A stored frame holds a dozen things and the match is one of them, so without
    the box the person is left to guess which. It travels from the store already
    decoded, under the same name the rest of the codebase uses.
    """
    query = _packed(1.0, 0.0, 0.0)
    rows = [{"id": 1, "siglip_blob": _packed(1.0, 0.0, 0.0),
             "frame_id": "f1", "bbox": [0.1, 0.2, 0.3, 0.4]},
            {"id": 2, "siglip_blob": _packed(0.0, 1.0, 0.0),
             "frame_id": "f1", "bbox": None}]
    answer = search.rank(query, rows)
    check("the match carries the box it was found in",
          answer["matches"][0]["bbox"], [0.1, 0.2, 0.3, 0.4])
    check("...and an observation without one says so rather than failing",
          answer["matches"][1]["bbox"], None)

    # And the store hands it over decoded rather than as the JSON it is stored as.
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            observe(store, 0.0, 0.0, 45.0)
            rows = store.searchable()
            check("the store decodes the box for a search", len(rows), 1)
            check("...as four numbers, not a string",
                  isinstance(rows[0]["bbox"], list), True)
            check("...and does not leak the column name",
                  "bbox_json" in rows[0], False)
        finally:
            store.close()


def test_a_flat_field_is_not_what_decides_a_match() -> None:
    """The rule this replaced, kept as a test so it cannot come back by accident.

    Two searches with the same separation and different scores must get different
    answers, and two with the same score and different separations the same one.
    Measured on the rover the separation told present from absent no better than
    a coin, so it must not be able to overturn the score.
    """
    query = _packed(1.0, 0.0, 0.0)
    crowd = [{"id": n, "siglip_blob": _packed(0.02, 1.0, n * 0.01)}
             for n in range(40)]

    # A real match with nothing else near it, and the same match in a room where
    # several things score almost as well. The separation differs greatly.
    alone = search.rank(query, crowd + [
        {"id": 1, "siglip_blob": _packed(1.0, 0.05, 0.0)}])
    among = search.rank(query, crowd + [
        {"id": 1, "siglip_blob": _packed(1.0, 0.05, 0.0)},
        {"id": 2, "siglip_blob": _packed(1.0, 0.07, 0.0)},
        {"id": 3, "siglip_blob": _packed(1.0, 0.09, 0.0)}])
    check("a thing seen once is found", alone["confident"], True)
    check("...and seeing it three times does not unfind it",
          among["confident"], True)
    check("...even though the separation has collapsed",
          among["stands_clear"] < alone["stands_clear"], True)


def test_too_little_seen_is_not_a_match_either() -> None:
    """Three stored regions cannot rule anything out, but they can still find
    something. What changes below a dozen is what the answer says about itself,
    not whether it is believed."""
    query = _packed(1.0, 0.0, 0.0)
    wrong = [{"id": n, "siglip_blob": _packed(0.04, 1.0, 0.0)}
             for n in range(3)]
    answer = search.rank(query, wrong)
    check("three things that are not it is not a match", answer["confident"], False)
    check("...and it says the rover has barely looked, rather than "
          "blaming the query", "not have looked at it yet" in answer["detail"],
          True)

    right = wrong + [{"id": 9, "siglip_blob": _packed(1.0, 0.0, 0.0)}]
    answer = search.rank(query, right)
    check("but a real match among four is still a match",
          answer["confident"], True)
    check("...and does not pretend to a separation worth reading",
          "spreads above" in answer["detail"], False)


def test_vectors_from_the_other_backend_are_not_ranked() -> None:
    """Comparing across backends would rank noise, so it is refused."""
    query = _packed(1.0, 0.0, 0.0)
    rows = [{"id": 1, "siglip_blob": _packed(1.0, 0.0, 0.0),
             "vectors_from": "onnxruntime"},
            {"id": 2, "siglip_blob": _packed(0.2, 1.0, 0.0),
             "vectors_from": "tensorrt"}]
    answer = search.rank(query, rows, backend="tensorrt")
    check("the row from the other backend is counted out",
          (answer["considered"], answer["skipped"]), (1, 1))
    check("...and the one that can be compared is ranked",
          answer["matches"][0]["observation_id"], 2)


def test_the_fast_path_and_the_plain_one_score_the_same() -> None:
    """A store of a thousand vectors is scored with one matrix multiply where
    numpy is there to do it, which on the rover is three milliseconds against
    the 0.29 s the Python loop was costing every search. The loop is still what
    runs on a host with only the standard library, so the two have to agree:
    a rover that found different things depending on what was installed would
    be worse than a slow one.
    """
    import random

    random.seed(7)

    def vector():
        return _packed(*[random.uniform(-1.0, 1.0) for _ in range(32)])

    rows = [{"id": n, "siglip_blob": vector()} for n in range(50)]
    # A vector of no length, which must score nothing rather than divide by
    # nought, and a row from the other backend, which must be counted out.
    rows.append({"id": 98, "siglip_blob": _packed(*([0.0] * 32))})
    rows.append({"id": 99, "siglip_blob": vector(), "vectors_from": "onnxruntime"})
    query = vector()

    fast = search.rank(query, rows, limit=60, backend="tensorrt")
    plain_numpy = search._numpy
    search._numpy = lambda: None
    try:
        plain = search.rank(query, rows, limit=60, backend="tensorrt")
    finally:
        search._numpy = plain_numpy

    check("both paths rank the same things in the same order",
          [one["observation_id"] for one in fast["matches"]],
          [one["observation_id"] for one in plain["matches"]])
    check("...to the same scores",
          [one["score"] for one in fast["matches"]],
          [one["score"] for one in plain["matches"]])
    check("...count out the same rows",
          (fast["considered"], fast["skipped"]),
          (plain["considered"], plain["skipped"]))
    check("...and reach the same verdict",
          (fast["confident"], fast["best"], fast["stands_clear"]),
          (plain["confident"], plain["best"], plain["stands_clear"]))
    check("a vector of no length scores nothing rather than failing",
          [one["score"] for one in fast["matches"]
           if one["observation_id"] == 98], [0.0])


TESTS = (
    test_a_query_that_matches_nothing_says_so,
    test_a_search_says_which_part_of_the_frame_it_found,
    test_a_flat_field_is_not_what_decides_a_match,
    test_too_little_seen_is_not_a_match_either,
    test_vectors_from_the_other_backend_are_not_ranked,
    test_the_fast_path_and_the_plain_one_score_the_same,
)
