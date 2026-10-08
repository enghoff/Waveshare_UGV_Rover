"""Work out which records are one object, in a process of its own, and write it down.

    python3 -m world_state.groups_job <world.db> <groups.json>

The daemon starts this, at low priority, when the groups it holds are older than
it allows; an autonomous run reads them to set a group's records aside together
(`rover_world._tool_world_state_entities`, `autonomy/cooling.py`). A process of
its own because co-fit is mostly Python loops, and in the daemon's process it
would hold the interpreter from the threads that answer STOP.

The store is opened read-only and copied; the grouping runs on the copy, exactly
as the console's preview does (`reader_groups.group`), and nothing is written to
the store. The answer goes to a temporary file first and is renamed into place,
so a reader never sees half of it.
"""
from __future__ import annotations

import json
import os
import sqlite3
import sys
import tempfile
import time
from pathlib import Path


def run(database: Path, output: Path) -> dict:
    from .reader_groups import group
    from .store import WorldStore

    began = time.time()
    with tempfile.TemporaryDirectory(prefix="ugv-groups-job-") as directory:
        clone = WorldStore(directory)
        try:
            with sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True) as source:
                source.backup(clone.db)
            clone._create()
            session = clone.map_session()
            generation = clone.generation()
            grouped = group(clone)
        finally:
            clone.close()
    if not grouped.get("ok"):
        answer = {"ok": False, "error": grouped.get("error"), "computed_at": began}
    else:
        groups = [sorted(members) for members in grouped["members"].values()
                  if len(members) > 1]
        answer = {"ok": True, "computed_at": began, "map_session": session,
                  "world_generation": generation, "converged": grouped["converged"],
                  "groups": sorted(groups), "seconds": round(time.time() - began, 1)}
    partial = output.with_suffix(output.suffix + ".partial")
    partial.write_text(json.dumps(answer) + "\n")
    os.replace(partial, output)
    return answer


if __name__ == "__main__":
    got = run(Path(sys.argv[1]), Path(sys.argv[2]))
    print(json.dumps({k: v for k, v in got.items() if k != "groups"}
                     | {"groups": len(got.get("groups") or [])}))
