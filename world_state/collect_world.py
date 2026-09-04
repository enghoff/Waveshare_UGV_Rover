#!/usr/bin/env python3
"""Pull the rover's live map, pose and placed things off it, read-only.

    ssh orin 'python3 ~/ugv/world_state/collect_world.py'
    scp orin:/tmp/world_replay.json /tmp/world_replay.json
    python3 world_state/bench_approach.py /tmp/run.db --grid /tmp/world_replay.json

Runs **on** the rover, because both things it asks for are on loopback: the
daemon's control port and the navigation bridge. Everything it calls is something
the drive console already calls every couple of seconds, plus one `map`; nothing
is written, nothing is recorded and the rover is not moved.

**Why it exists at all.** A recorded `world.db` carries poses and bearings but not
the occupancy grid they were measured against, so a replay of the viewpoint
chooser has no walls to test against unless the grid is fetched separately. The
map session comes with it for the check that matters: coordinates measured under
a map that has since been cleared name a place in the new one only by
coincidence, and a grid from the wrong session would test a recording's
placements against somebody else's walls.
"""
from __future__ import annotations

import json
import socket
import sys

DAEMON = ("127.0.0.1", 8769)
BRIDGE = ("127.0.0.1", 8773)
WHERE = "/tmp/world_replay.json"


def ask(where, request, timeout=15.0):
    """One request, one reply, skipping a move's running commentary.

    The bridge narrates while it works and the daemon does not, so this reads
    until something that is not a progress line arrives -- which is the reply for
    both of them.
    """
    sock = socket.create_connection(where, timeout)
    try:
        handle = sock.makefile("rwb")
        handle.write(json.dumps(request).encode() + b"\n")
        handle.flush()
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError("no answer to %r" % request)
            answer = json.loads(line)
            if answer.get("kind") != "progress":
                return answer
    finally:
        sock.close()


def main() -> int | str:
    grid = ask(BRIDGE, {"op": "map"})
    if not grid.get("ok"):
        return "no map yet: %s" % grid.get("error")
    status = ask(DAEMON, {"call": "nav_status", "arguments": {}})
    listing = ask(DAEMON, {"call": "world_state_entities", "arguments": {}})
    if not listing.get("ok"):
        return "no world state: %s" % listing.get("error")

    things = []
    for entity in listing.get("entities") or []:
        if not entity.get("placement"):
            continue
        # Asked for one at a time because that is the call that carries the rays,
        # and the rays are the directions the thing was seen from.
        detail = ask(DAEMON, {"call": "world_state_entity",
                              "arguments": {"id": entity["id"]}})
        if not detail.get("ok"):
            continue
        things.append({"id": entity["id"],
                       "placement": entity["placement"],
                       "placement_map_session":
                           entity.get("placement_map_session"),
                       "observation_count": entity.get("observation_count"),
                       "rays": detail.get("rays") or []})

    held = {"grid": grid,
            "pose": status.get("pose") if status.get("ok", True) else None,
            "map_session": (listing.get("summary") or {}).get("map_session"),
            "entities": things}
    with open(WHERE, "w") as handle:
        json.dump(held, handle)
    print("%s: %d placed things, %d of them with rays, map %sx%s at %.3f m, "
          "map session %s" % (WHERE, len(things),
                              sum(1 for one in things if one["rays"]),
                              grid.get("width"), grid.get("height"),
                              float(grid.get("resolution_m") or 0.0),
                              held["map_session"]))
    print("the rover is at %s" % (held["pose"],))
    return 0


if __name__ == "__main__":
    sys.exit(main())
