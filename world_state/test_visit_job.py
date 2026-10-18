"""The world model's job follows the store it files from (visit_model/job.py).

Its records are places on one map, filed from one filling of the store. When
the store is cleared, or put on another map, the job keeps its engine and
everything of that map aside and files what the store holds then, from nothing.
The records it writes say which map and which filling they are of, which is
what the daemon checks before it lets them answer (visit_things.py,
rover_daemon/test_recall.py).
"""
from __future__ import annotations

import base64
import json
import os
import sys
import tempfile
import zlib

from test_fakes import a_sighting, a_store
from test_harness import SKIP, check

HERE = os.path.dirname(os.path.abspath(__file__))


def _a_grid():
    """Navigation's grid as `nav_grid` sends it: four cells, one a wall."""
    import numpy as np

    cells = np.zeros((2, 2), np.int8)
    cells[1, 1] = 100
    return {"ok": True, "width": 2, "height": 2, "resolution_m": 0.05,
            "origin_x_m": -0.05, "origin_y_m": -0.05,
            "data": base64.b64encode(zlib.compress(cells.tobytes())).decode()}


def test_the_world_models_job_follows_the_store_it_files_from() -> None:
    try:
        import numpy  # noqa: F401
        import PIL  # noqa: F401
        sys.path.insert(0, os.path.join(HERE, "visit_model"))
        import engine
        import job
    except ImportError as error:
        SKIP.append(f"the world model's job following the store ({error})")
        return

    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        state = os.path.join(directory, "visit_model")
        os.makedirs(state)
        path = os.path.join(state, "engine.pkl")
        said: list[str] = []
        grids = {"0000000000bb": _a_grid()}

        def daemon(call, arguments=None, timeout=10.0):
            if call == "nav_status":
                return {"ok": True, "map_id": store.map_id()}
            return grids.get(store.map_id(), {"ok": False})

        was = job.DB, job.daemon
        job.DB, job.daemon = store.path, daemon
        try:
            store.follow_map("0000000000aa")
            e = job.load(path, state, said.append)
            check("with no engine the job files the store from nothing, noting which filling on which map",
                  e.filling, {"store_generation": store.generation(), "map_id": "0000000000aa",
                              "map_session": store.map_session()})
            check("...with no walls, navigation having sent no grid of that map",
                  any("no grid of map 0000000000aa" in line for line in said), True)
            job.once(e, state, False, said.append)
            with open(os.path.join(state, "things.json")) as f:
                written = json.load(f)
            check("the records it writes say which",
                  (written["map_id"], written["store_generation"], written["map_session"]),
                  ("0000000000aa", store.generation(), store.map_session()))
            check("while the store holds the same, the engine carries on",
                  job.follow(e, state, said.append) is e, True)

            session = store.map_session()
            store.record([a_sighting()], capture={"frame_id": "f1", "pan": 0.0,
                                                  "pose": {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0}})
            store.follow_map("0000000000bb")
            moved = job.follow(e, state, said.append)
            check("put on another map, rows kept, the store files a new session",
                  store.map_session(), session + 1)
            check("...and the job a new engine, its records named afresh",
                  (moved is e, moved.generation != e.generation), (False, True))
            check("...of that map", (moved.filling["map_id"], moved.filling["map_session"]),
                  ("0000000000bb", session + 1))
            aside = os.listdir(os.path.join(state, "set-aside"))
            check("...the old engine and its records kept aside, named for why and for them",
                  (aside, sorted(os.listdir(os.path.join(state, "set-aside", aside[0])))),
                  ([aside[0][:15] + "-map-" + e.generation],
                   ["decisions.jsonl", "engine.pkl", "things.json"]))
            check("...and raised things placed on the new map's own grid",
                  int(job.walls_for(moved, state, said.append)["stable"].sum()), 1)

            store.clear()
            cleared = job.follow(moved, state, said.append)
            check("cleared with the map kept, the job starts again too",
                  (cleared.generation != moved.generation, cleared.filling["store_generation"]),
                  (True, store.generation()))
            check("...saying so", any("the world state was cleared" in line for line in said), True)
            aside = [n for n in os.listdir(os.path.join(state, "set-aside"))
                     if n.endswith("-cleared-" + moved.generation)]
            check("...with that engine and the new map's walls kept aside as well",
                  (len(aside), "walls.npz" in os.listdir(os.path.join(state, "set-aside", *aside[:1]))),
                  (1, True))

            # Saved before the job wrote down what it files from: on map
            # session 67 it is the engine of the store as it stands, and on any
            # other it is not.
            old = engine.Engine()
            old.generation, old.filed_as = "00000000000000ab", job.FILED_AS
            del old.filling
            old.save(path)
            check("an engine saved before is not taken as this store's on another session",
                  job.load(path, state, said.append).filling, None)
            with store.db:
                store.db.execute("REPLACE INTO meta(key, value) VALUES('map_session', '67')")
            check("...and is on session 67, the one it filed",
                  job.load(path, state, said.append).filling["map_session"], 67)
        finally:
            job.DB, job.daemon = was
            store.close()


def test_records_of_people_are_marked_for_the_planner() -> None:
    """2026-10-18: of the 149 records a run could aim a look at, 8 were people,
    who are gone by the time the look is taken. The job marks the records a
    frozen model takes for a person (visit_model/people.py), reading only the
    regions filed since its last pass; the planner leaves them alone."""
    try:
        import numpy  # noqa: F401
        sys.path.insert(0, os.path.join(HERE, "visit_model"))
        import job
        import people
    except ImportError as error:
        SKIP.append(f"records of people marked ({error})")
        return
    from world_state.perception_client import Sighting
    from test_fakes import a_vector

    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        model = os.path.join(directory, "person.json")
        # A person, to this model, is a region whose first DINOv2 component is large.
        with open(model, "w") as f:
            json.dump({"dims": {"dino": 8, "siglip": 8}, "coef": [12.0] + [0.0] * 15,
                       "intercept": -6.0, "threshold": 0.75}, f)

        def seen(first):
            return Sighting(bbox=[0.1, 0.3, 0.5, 0.9], region_score=0.8, area=0.2,
                            dino=a_vector(1.0), siglip=a_vector(0.0, 1.0),
                            dino_alone=a_vector(first, 1.0 - first), mask_share=0.5)

        def regions(*firsts):
            ids = []
            for first in firsts:
                store.record([seen(first)], capture={
                    "frame_id": "f%d" % len(store.observations(limit=1000)), "pan": 0.0,
                    "pose": {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0}})
                ids.append(int(store.db.execute("SELECT MAX(id) FROM observations").fetchone()[0]))
            return ids

        try:
            person, chair = regions(1.0, 1.0), regions(0.0, 0.1)
            records = [{"id": "v1", "observations": person}, {"id": "v2", "observations": chair}]
            scorer = people.People(store.path, model)
            got = scorer.score(records, "g1")
            check("a record of a person is taken for one, and one of a chair is not",
                  (scorer.flag(got["v1"]), scorer.flag(got["v2"])), (True, False))
            more = regions(0.0, 0.0, 0.0, 0.0)
            records[0]["observations"] = person + more
            again = scorer.score(records, "g1")
            fresh = people.People(store.path, model).score(records, "g1")
            check("...regions filed to it since are added, as summing it afresh would",
                  (round(again["v1"], 9) == round(fresh["v1"], 9), scorer.flag(again["v1"])), (True, False))
            records[0]["observations"] = person
            check("...and one that has lost regions to a merge is summed again",
                  scorer.flag(scorer.score(records, "g1")["v1"]), True)
            check("with no model there is nothing to say",
                  people.People(store.path, os.path.join(directory, "none.json")).score(records),
                  {"v1": None, "v2": None})

            said = []
            was = job.DB, job._PEOPLE
            job.DB, job._PEOPLE = store.path, {store.path: people.People(store.path, model)}
            try:
                marked = [dict(r) for r in records]
                digest = job.mark_people(marked, "g1", said.append)
                check("the job writes each record's chance and whether it is taken for a person",
                      [(r["person"], r["person_p"] > 0.9) for r in marked] + [bool(digest)],
                      [(True, True), (False, False), True])
                job._PEOPLE = {store.path: None}
                job._PEOPLE[store.path] = type("Broken", (), {"score": lambda *a: 1 / 0})()
                marked = [dict(r) for r in records]
                check("...and when scoring fails it says so and still writes them, unmarked",
                      (job.mark_people(marked, "g1", said.append), [r["person"] for r in marked],
                       any("people not scored" in line for line in said)),
                      (None, [None, None], True))
            finally:
                job.DB, job._PEOPLE = was
        finally:
            store.close()


TESTS = (
    test_the_world_models_job_follows_the_store_it_files_from,
    test_records_of_people_are_marked_for_the_planner,
)
