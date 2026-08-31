#!/usr/bin/env python3
"""Take whatever is on /map and write it down, with the numbers that make two maps
comparable.

    python3 map_score.py --out /path/to/name --label "kitchen-loop rtabmap"

Used by replay_bag.sh after a replayed drive, and usable on the live rover to
capture what the mapper currently believes:

    . ~/ugv/ros_nav/env.sh; . ~/ugv/ros_nav/dds.sh
    python3 ~/ugv/ros_nav/map_score.py --out /tmp/now --label live

## The numbers, and which one to argue about

A mapper that has lost track does not usually produce an empty map -- it produces
a *bigger* one, because the same wall gets drawn twice a few degrees apart and the
room grows. So:

  extent            how much ground the grid covers. A room does not change size,
                    so between two runs over one recording, smaller is better.
  occupied          cells the mapper calls a wall. Doubling a wall doubles this.
  walls per m2      occupied cells divided by the floor area the mapper found. The
                    single number worth quoting: it is independent of how much of
                    the room got explored, which the two above are not.

There is deliberately no score for "is this map correct", because nothing here
knows what the room looks like. These numbers compare two maps of *the same
recording*; they say nothing across different drives.

## Why this writes a plain greyscale dump and not the console's picture

`lidar_slam/mapimg.py` is this rover's map renderer and there should not be a
second one -- it draws the arrow, the trail, the camera's cone and the caption a
model reads, and a rival renderer would drift away from it. This is not that: it
is a diagnostic dump of one grid at one moment, in the same PGM convention the
ROS map_server uses, so that RTAB-Map's grid and slam_toolbox's can be laid side
by side. The console keeps its picture; this is a contact sheet.
"""

import argparse
import collections
import struct
import sys
import time
import zlib

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

#: The map_server PGM convention, so anything that reads one of those reads these.
OCCUPIED, FREE, UNKNOWN = 0, 254, 205


def png_bytes(width, height, pixels):
    """One channel, 8 bits, no filtering -- a greyscale PNG of `pixels`."""
    rows = b"".join(b"\x00" + bytes(pixels[y * width:(y + 1) * width])
                    for y in range(height))

    def chunk(tag, body):
        block = tag + body
        return struct.pack(">I", len(body)) + block + struct.pack(">I", zlib.crc32(block))

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(rows, 6))
            + chunk(b"IEND", b""))


class MapGrab(Node):
    """One latched map, or nothing."""

    def __init__(self):
        super().__init__("map_score")
        self.grid = None
        # Both mappers publish the grid reliable and transient-local, so a
        # subscriber that arrives long after the last update still gets it. That
        # is the whole reason this can be a one-shot script: without matching
        # durability it would wait for the next update, which on a finished
        # replay is never.
        self.create_subscription(
            OccupancyGrid, "map", self.on_map,
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))

    def on_map(self, msg):
        self.grid = msg


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True,
                    help="path prefix; .pgm, .png and .txt are written")
    ap.add_argument("--label", default="map", help="name for the report line")
    ap.add_argument("--wait", type=float, default=20.0,
                    help="seconds to wait for a map before giving up")
    args = ap.parse_args(argv)

    rclpy.init()
    node = MapGrab()
    # Wall time rather than the node's clock: under a replay this may be on
    # simulated time, which stops dead when the bag ends -- and a deadline on a
    # clock that has stopped is a wait that never finishes.
    stop_at = time.monotonic() + args.wait
    while node.grid is None and time.monotonic() < stop_at:
        rclpy.spin_once(node, timeout_sec=0.5)
    grid = node.grid
    rclpy.shutdown()

    if grid is None:
        print("%s: no map published in %.0f s" % (args.label, args.wait))
        return 1

    w, h = grid.info.width, grid.info.height
    res = grid.info.resolution
    counts = collections.Counter(grid.data)
    occupied = sum(v for k, v in counts.items() if k >= 65)
    free = sum(v for k, v in counts.items() if 0 <= k < 65)
    unknown = counts.get(-1, 0)
    floor_m2 = free * res * res
    per_m2 = occupied / floor_m2 if floor_m2 else float("nan")

    # ROS grids are row-major from the bottom-left; PGM images run top down, so
    # the rows come out reversed. Getting this wrong flips every map vertically,
    # which looks plausible and is wrong.
    pixels = bytearray(w * h)
    for y in range(h):
        src = (h - 1 - y) * w
        dst = y * w
        for x in range(w):
            v = grid.data[src + x]
            pixels[dst + x] = UNKNOWN if v < 0 else (OCCUPIED if v >= 65 else FREE)

    with open(args.out + ".pgm", "wb") as fh:
        fh.write(b"P5\n%d %d\n255\n" % (w, h))
        fh.write(bytes(pixels))
    with open(args.out + ".png", "wb") as fh:
        fh.write(png_bytes(w, h, pixels))

    report = ("%s: %.1f x %.1f m at %.0f cm, occupied %d, free %d, unknown %d, "
              "walls per m2 of floor %.1f"
              % (args.label, w * res, h * res, res * 100,
                 occupied, free, unknown, per_m2))
    with open(args.out + ".txt", "w", encoding="utf-8") as fh:
        fh.write(report + "\n")
    print(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
