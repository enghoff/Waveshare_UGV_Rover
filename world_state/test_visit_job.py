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


TESTS = (
    test_the_world_models_job_follows_the_store_it_files_from,
)
