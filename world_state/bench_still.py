#!/usr/bin/env python3
"""What refusing to record the same picture twice catches, and what it costs.

    ssh orin 'cd ~/ugv/world_state && python3 bench_still.py --burst 40 --gap 1'
    python world_state/bench_still.py /tmp/run.db --frames /tmp/frames

Two halves, because the rule has two sides and neither answers for the other.
The **burst** half needs the camera and a rover that is standing still: it takes
pictures of an unchanging room and measures how much they differ anyway, which
is the floor `inspector.SAME_PICTURE_SHARE` has to clear. The **recording** half
runs at a desk and replays a real drive twice -- once as the rover recorded it,
once with the rule applied -- which is what says the rule costs a drive nothing.

## What it measured, parked on 2026-09-04

The camera was pointed at a wall, the corner of a sofa and a cable on the floor,
and nothing in the room moved for either burst.

| frames | apart | worst neighbours | worst pair in the burst | recorded of 40 |
|---|---|---:|---:|---:|
| 40 | 1 s | 0.000 | 0.004 | **1** |
| 40 | 15 s | 0.039 | 0.199 | **6** |

**A rover looking once a second at a room that is not changing records one look
instead of forty.** The ten-minute figure is not the room and is not the rule
failing: the room was the same picture throughout, and what moved was
auto-exposure hunting by several grey levels unevenly across the frame. It is
why the write-up on `SAME_PICTURE_SHARE` says this cannot judge two looks minutes
apart, and why `rover_world.LOOK_ANYWAY_S` still gets its look every five.

## And what it cost the drive of 2026-09-04

163 looks and 643 observations, of which **the rule discards 2 looks and 4
observations** -- both of them taken after the rover had parked, both scoring
0.000. Every entity, every attachment and both of `replay.py`'s scores come back
identical. On a rover that is driving this changes nothing, which is the point:
it is a test for a rover that is not.
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

if __package__ in (None, ""):                       # run as a script
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "world_state"

from . import inspector                             # noqa: E402
from . import replay as replay_module               # noqa: E402


def refused(groups: list[list[dict]], frames_dir: str, limit: float
            ) -> tuple[set, list[tuple]]:
    """Which recorded observations the rule would have thrown away.

    Walked in the order the rover took them and compared against the last frame
    that was *kept*, exactly as `Inspector` does -- a recording judged against
    the last frame merely seen would let an arbitrarily large drift through one
    imperceptible step at a time.
    """
    doomed, dropped, kept = set(), [], None
    for group in groups:
        name = group[0].get("frame_id")
        path = os.path.join(frames_dir, f"{name}.jpg") if name else ""
        if not path or not os.path.exists(path):
            kept = None                        # nothing to compare the next one to
            continue
        with open(path, "rb") as handle:
            seen = inspector.picture(handle.read())
        share = inspector.picture_changed(kept, seen)
        if share is not None and share < limit:
            doomed.update(row["id"] for row in group)
            dropped.append((group[0]["inference_id"], share, len(group)))
        else:
            kept = seen
    return doomed, dropped


def burst(count: int, gap: float, address: str) -> int:
    """Ask the daemon for pictures of a room nobody is touching, and score them.

    Through `camera_jpeg` rather than through the camera directly, because that
    is the path a look takes: the tracking loop's newest frame when the loop has
    the camera, and a bounded one-shot grab when it has not. A burst taken any
    other way would measure a camera this rover does not look through.
    """
    import json
    import socket

    def call(name: str) -> dict:
        sock = socket.create_connection(address_of(address), 5)
        sock.settimeout(30)
        handle = sock.makefile("rwb")
        handle.write(json.dumps({"call": name, "arguments": {}}).encode() + b"\n")
        handle.flush()
        answer = json.loads(handle.readline())
        handle.close()
        sock.close()
        return answer

    import base64

    pictures, lost = [], 0
    for _index in range(count):
        began = time.monotonic()
        try:
            answer = call("camera_jpeg")
        except Exception as error:                             # noqa: BLE001
            answer = {"ok": False, "error": f"{type(error).__name__}: {error}"}
        if answer.get("ok") and answer.get("jpeg_base64"):
            reduced = inspector.picture(base64.b64decode(answer["jpeg_base64"]))
            if reduced is not None:
                pictures.append(reduced)
        else:
            lost += 1
        time.sleep(max(0.0, gap - (time.monotonic() - began)))
    if len(pictures) < 4:
        print(f"the camera gave {len(pictures)} usable pictures and lost {lost}; "
              f"nothing can be said from that", file=sys.stderr)
        return 1

    neighbours = [inspector.picture_changed(pictures[index - 1], pictures[index])
                  for index in range(1, len(pictures))]
    every = [inspector.picture_changed(pictures[first], pictures[second])
             for second in range(len(pictures)) for first in range(second)]
    recorded, held = 0, None
    for one in pictures:
        share = inspector.picture_changed(held, one)
        if share is None or share >= inspector.SAME_PICTURE_SHARE:
            recorded += 1
            held = one
    span = (len(pictures) - 1) * gap
    print(f"{len(pictures)} pictures {gap:g} s apart, {span:.0f} s in all "
          f"({lost} lost)\n")
    print(f"  one against the one before it   median {statistics.median(neighbours):.3f}  "
          f"worst {max(neighbours):.3f}")
    print(f"  any two in the burst            median {statistics.median(every):.3f}  "
          f"worst {max(every):.3f}")
    print(f"\n  the limit is {inspector.SAME_PICTURE_SHARE:.2f}, so a rover "
          f"looking this often at this room records "
          f"{recorded} of {len(pictures)} looks")
    if max(neighbours) >= inspector.SAME_PICTURE_SHARE:
        print("  ITS OWN NEIGHBOURS CLEAR THE LIMIT -- nothing here was moving, so "
              "either the room was not as still as it looked or the limit is too "
              "low for this camera.")
    return 0


def address_of(text: str) -> tuple[str, int]:
    host, _, port = text.partition(":")
    return host or "127.0.0.1", int(port) if port else 8769


def recording(args) -> int:
    """Replay the drive twice, and mark both with `replay.py`'s own scores."""
    if not os.path.exists(args.database):
        print(f"no such recording: {args.database}", file=sys.stderr)
        return 2
    if not args.frames or not os.path.isdir(args.frames):
        print("--frames is what this compares; without the pictures there is "
              "nothing to compare", file=sys.stderr)
        return 2
    blank = replay_module.blank_ids(args.database, args.frames)
    reach = replay_module.reach_from(args.map) if args.map else None
    groups = replay_module.inspections(args.database)
    observations = sum(len(group) for group in groups)
    doomed, dropped = refused(groups, args.frames, inspector.SAME_PICTURE_SHARE)
    print(f"  {len(groups)} looks, {observations} observations; the limit is "
          f"{inspector.SAME_PICTURE_SHARE:.2f}")
    print(f"  {len(dropped)} looks and {len(doomed)} observations would not "
          f"have been recorded")
    if args.detail:
        for inference_id, share, held in dropped:
            print(f"    look {inference_id}: {share:.3f} of the picture "
                  f"differed, {held} regions")

    for name, skip in (("every look", blank),
                       ("changed looks only", blank | doomed)):
        if args.only and args.only != name.split()[0]:
            continue
        began = time.time()
        entities, kept = replay_module.replay(args.database, skip=skip,
                                              reach=reach, groups=groups)
        print(f"\n=== {name}   ({time.time() - began:.1f} s)")
        replay_module.score(entities, kept, detail=args.detail)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("database", nargs="?", default="",
                        help="a world.db copied off the rover")
    parser.add_argument("--frames", default="",
                        help="the directory its frames were copied to")
    parser.add_argument("--map", default="")
    parser.add_argument("--burst", type=int, default=0,
                        help="instead: take this many pictures through the "
                             "daemon and score them. Needs the rover.")
    parser.add_argument("--gap", type=float, default=1.0,
                        help="seconds between the pictures of a burst")
    parser.add_argument("--rover", default="127.0.0.1:8769")
    parser.add_argument("--only", default="",
                        help="run one arm by name: every or changed")
    parser.add_argument("--detail", action="store_true")
    args = parser.parse_args()

    if args.burst:
        return burst(args.burst, args.gap, args.rover)
    if not args.database:
        parser.error("give a recording, or --burst to measure the camera")
    return recording(args)


if __name__ == "__main__":
    sys.exit(main())
