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
          f"{search.MATCHES:.3f}" in answer["detail"], True)

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


def test_a_sliver_at_the_edge_of_the_frame_is_not_ranked() -> None:
    """A strip the edge of the frame has cut a region down to matches anything.

    Measured on the rover: a few pixels of window frame in the corner of a
    picture came back as "a traffic light". So a sliver is counted out, and
    counted, while a small thing seen whole in the middle of the frame and a wide
    region against its edge are both still ranked -- it takes both halves.
    """
    query = _packed(1.0, 0.0, 0.0)
    field = [{"id": n, "siglip_blob": _packed(0.02, 1.0, n * 0.01),
              "bbox": [0.3, 0.3, 0.6, 0.6]} for n in range(20)]
    strip = {"id": 50, "siglip_blob": _packed(1.0, 0.0, 0.0),
             "bbox": [0.0, 0.0806, 0.0437, 0.2156]}
    answer = search.rank(query, field + [strip])
    check("a sliver that would score best is not ranked",
          [one["observation_id"] for one in answer["matches"]].count(50), 0)
    check("...so it cannot make the search believe it", answer["confident"], False)
    check("...and it is counted as left aside", answer["slivers"], 1)

    small = {"id": 51, "siglip_blob": _packed(1.0, 0.0, 0.0),
             "bbox": [0.45, 0.45, 0.48, 0.48]}
    wide = {"id": 52, "siglip_blob": _packed(1.0, 0.01, 0.0),
            "bbox": [0.0271, 0.6449, 0.8401, 0.7841]}
    edge = {"id": 53, "siglip_blob": _packed(1.0, 0.02, 0.0),
            "bbox": [0.8613, 0.0, 0.9999, 0.8294]}
    answer = search.rank(query, field + [small, wide, edge])
    ranked = [one["observation_id"] for one in answer["matches"]]
    check("a small thing in the middle of the frame is still ranked",
          51 in ranked, True)
    check("...and so are wide regions against the edge",
          (52 in ranked, 53 in ranked), (True, True))
    check("...with nothing left aside", answer["slivers"], 0)

    answer = search.rank(query, [strip])
    check("a store of nothing but slivers says it could compare nothing",
          (answer["confident"], answer["detail"]),
          (False, "nothing stored can be compared with this query"))


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


# --- finding a thing, rather than a look -------------------------------------


def _remembered(store, entity_id, vector, backend="tensorrt",
                box=(0.4, 0.4, 0.6, 0.6)):
    """One look with this semantic vector, recorded and given to a thing."""
    from world_state.perception_client import Sighting

    store.record([Sighting(bbox=list(box), dino=b"", siglip=vector)],
                 capture={"frame_id": "f", "pan": 0.0, "tilt": 0.0,
                          "pose": {"x_m": 1.0, "y_m": 1.0, "heading_deg": 0.0}},
                 source="perception", fov_deg=66.0, vectors_from=backend)
    look = store.observations(limit=1)[0]["id"]
    if entity_id:
        store.attach(entity_id, [look], why="the test")
    return look


def test_things_are_ranked_by_their_average_look() -> None:
    """`find_thing`'s ranking since 2026-10-13.

    The look search answers "which picture is most like this", and one lucky
    look can carry a thing that mostly looks like something else. An average over
    all of a thing's looks asks what the thing looks like, which is the question.
    """
    query = _packed(1.0, 0.0, 0.0)
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            lamp, lucky = store.create_entity(), store.create_entity()
            for _ in range(3):
                _remembered(store, lamp, _packed(0.9, 0.3, 0.0))
            # One look that matches perfectly, among two that are something else.
            for vector in (_packed(1.0, 0.0, 0.0), _packed(0.0, 1.0, 0.0),
                           _packed(0.0, 0.0, 1.0)):
                _remembered(store, lucky, vector)
            check("the look search would answer with the lucky look",
                  search.rank(query, store.searchable())["matches"][0]["score"], 1.0)

            index = search.ThingIndex(store)
            answer = search.rank_things(query, index)
            check("the thing search answers with the thing that looks like it",
                  answer["matches"][0]["entity_id"], lamp)
            check("...and believes it", answer["confident"], True)
            check("...saying how many looks it is the average of",
                  answer["matches"][0]["looks"], 3)
            check("...and the floor it was judged against is the thing floor",
                  answer["floor"], search.THING_MATCHES)

            # A look that belongs to no thing has nowhere to be and nothing to
            # drive to, so it is not something this search can answer with.
            _remembered(store, None, _packed(1.0, 0.0, 0.0))
            again = search.rank_things(query, index)
            check("a look that belongs to nothing is not ranked",
                  [one["entity_id"] for one in again["matches"]], [lamp, lucky])

            nothing = search.rank_things(_packed(0.0, 0.0, -1.0), index)
            check("a phrase like none of the things is not found",
                  nothing["confident"], False)
            check("...and the reason quotes the thing floor",
                  f"{search.THING_MATCHES:.3f}" in nothing["detail"], True)
        finally:
            store.close()


def test_a_thing_average_is_built_again_only_when_its_looks_change() -> None:
    """Building every average reads every look the store holds, so it is kept, and
    a search only pays for the things whose looks have changed since the last."""
    query = _packed(1.0, 0.0, 0.0)
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            lamp, chair = store.create_entity(), store.create_entity()
            _remembered(store, lamp, _packed(1.0, 0.0, 0.0))
            _remembered(store, chair, _packed(0.0, 1.0, 0.0))
            index = search.ThingIndex(store)
            check("the first search builds every thing", index.refresh(), 2)
            check("...and the next, with nothing new, builds none", index.refresh(), 0)

            # The chair is seen again, and looks more like the query this time.
            _remembered(store, chair, _packed(1.0, 0.0, 0.0))
            check("a thing seen again is the only one built again",
                  index.refresh(), 1)
            scores = {one["entity_id"]: one["score"]
                      for one in search.rank_things(query, index)["matches"]}
            check("...and its average has moved",
                  round(scores[chair], 3), round(1.0 / 2 ** 0.5, 3))

            store.clear()
            check("a cleared store leaves nothing to rank",
                  search.rank_things(query, index)["matches"], [])
        finally:
            store.close()


def test_a_thing_average_keeps_to_one_backend_and_leaves_out_slivers() -> None:
    """R-WS-4 for averages: a GPU and a CPU vector are not comparable, so they are
    never averaged together, and only the backend that took the newest look --
    the one the sidecar is running, which embedded the phrase -- is searched.
    And a sliver, which matches whatever is asked, counts for nothing in an
    average either."""
    query = _packed(1.0, 0.0, 0.0)
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            old, edge, lamp = (store.create_entity(), store.create_entity(),
                               store.create_entity())
            _remembered(store, old, _packed(1.0, 0.0, 0.0), backend="onnx-cpu")
            _remembered(store, edge, _packed(1.0, 0.0, 0.0),
                        box=(0.0, 0.2, 0.03, 0.8))
            _remembered(store, lamp, _packed(0.8, 0.6, 0.0))
            ranked = search.rank_things(query, search.ThingIndex(store))
            check("only the thing seen by the running backend, whole, is ranked",
                  [one["entity_id"] for one in ranked["matches"]], [lamp])
        finally:
            store.close()


TESTS = (
    test_things_are_ranked_by_their_average_look,
    test_a_thing_average_is_built_again_only_when_its_looks_change,
    test_a_thing_average_keeps_to_one_backend_and_leaves_out_slivers,
    test_a_query_that_matches_nothing_says_so,
    test_a_search_says_which_part_of_the_frame_it_found,
    test_a_flat_field_is_not_what_decides_a_match,
    test_too_little_seen_is_not_a_match_either,
    test_a_sliver_at_the_edge_of_the_frame_is_not_ranked,
    test_vectors_from_the_other_backend_are_not_ranked,
    test_the_fast_path_and_the_plain_one_score_the_same,
)
