#!/usr/bin/env python3
"""What one look costs, and what it finds, on whatever machine this is run on.

    ssh orin 'cd ~/ugv/world_state && python3 bench_perceive.py'
    ssh orin 'cd ~/ugv/world_state && python3 bench_perceive.py --threads 4'
    python world_state/bench_perceive.py path/to/frames

**The rover's own numbers are the only ones that decide anything here.** A
desktop runs this pipeline several times faster than the Jetson does, so a
desktop figure says the code works, not that it fits. What has to fit is one
look inside the time the rest of the rover can spare, with the lidar still
reporting no dropped scans while it runs.

With no arguments it reads the frames the world state has already stored, under
``~/.ugv/world/frames``, which on the rover are pictures of the actual room.
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from world_state.perceive import Perception          # noqa: E402
from world_state.store import world_dir              # noqa: E402


def frames(where: str | None) -> list[str]:
    directory = where or os.path.join(world_dir(), "frames")
    if os.path.isfile(directory):
        return [directory]
    if not os.path.isdir(directory):
        return []
    return [os.path.join(directory, name) for name in sorted(os.listdir(directory))
            if name.lower().endswith((".jpg", ".jpeg", ".png"))]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", nargs="?", help="a directory of pictures")
    parser.add_argument("--threads", type=int, default=0,
                        help="onnxruntime intra-op threads; 0 leaves its default")
    parser.add_argument("--repeat", type=int, default=1,
                        help="look at each frame this many times")
    parser.add_argument("--top", type=int, default=6,
                        help="how many regions to print per frame")
    arguments = parser.parse_args()

    pictures = frames(arguments.frames)
    if not pictures:
        print("no frames to look at -- give a directory, or run an inspection "
              "first so there is one stored", file=sys.stderr)
        return 1

    perception = Perception(threads=arguments.threads)
    ready, why = perception.available()
    if not ready:
        print(why, file=sys.stderr)
        return 1

    began = time.monotonic()
    perception.load()
    print(f"loaded three models and {len(perception.words)} vocabulary phrases "
          f"in {time.monotonic() - began:.1f} s"
          + (f" ({arguments.threads} threads)" if arguments.threads else ""))

    everything = []
    for path in pictures:
        with open(path, "rb") as handle:
            jpeg = handle.read()
        # The first look pays for the graph optimiser and the memory arena, which
        # is a real cost once per process and a misleading one to average in.
        looks = [perception.look(jpeg) for _ in range(arguments.repeat + 1)][1:] \
            if arguments.repeat > 0 else []
        if not looks:
            continue
        best = min(looks, key=lambda one: one["took_s"])
        everything.append(best)
        timings = best["timings"]
        print(f"\n=== {os.path.basename(path)}")
        print(f"  {best['found']} regions, {best['kept']} after filtering, "
              f"{len(best['regions'])} embedded | "
              f"fastsam {timings['regions_ms']} ms, dino {timings['dino_ms']} ms, "
              f"siglip {timings['siglip_ms']} ms, whole look {best['took_s']:.2f} s")
        for region in best["regions"][:arguments.top]:
            box = ", ".join(f"{value:.2f}" for value in region["bbox"])
            print(f"    {region['label']:<22} {region['label_score']:.3f}  "
                  f"area {region['area']:.3f}  box [{box}]")

    if everything:
        times = [one["took_s"] for one in everything]
        print(f"\n{len(times)} frames: median {statistics.median(times):.2f} s, "
              f"worst {max(times):.2f} s, "
              f"{statistics.median(len(one['regions']) for one in everything):.0f} "
              f"regions typical")
    return 0


if __name__ == "__main__":
    sys.exit(main())
