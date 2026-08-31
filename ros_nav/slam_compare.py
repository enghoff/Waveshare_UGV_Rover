#!/usr/bin/env python3
"""Measure slam_toolbox and RTAB-Map against each other over one drive.

    ~/ugv/ros_nav/native.sh python3 ~/ugv/ros_nav/slam_compare.py --seconds 300
    ~/ugv/ros_nav/native.sh python3 ~/ugv/ros_nav/slam_compare.py --out /tmp/c.json

**Run it through native.sh, not from the conda environment.** It reads
`rtabmap_msgs`, and those message definitions exist only in the /opt/ros/jazzy
install that install-rtabmap.sh puts down -- RoboStack has no rtabmap package at
all. Everything else it reads is a standard message that both installs have.

## What this is for

The decision this rover has to make is whether RTAB-Map should take `map ->
odom` away from slam_toolbox, and that is not a decision to make by looking at
two pictures of a corridor. So both mappers are run over the same drive, from
the same `/scan` and the same wheels, and this records the numbers that would
justify a swap:

    corrections        how often each one changed its mind about where the
                       rover is, and by how much
    loop closures      how many times each decided it had come back somewhere
                       it had been
    graph size         what that cost in nodes
    return-to-start    with the rover driven back to its physical starting
                       point, how far each thinks it is from where it began
    grid agreement     how much of the two occupancy grids actually match
    update latency     what one update costs, and the worst case
    cpu and memory     what each process is spending

Nothing here publishes and nothing here drives. It is one more Python process on
a board that is already running two mappers, so keep the runs to the length of a
route rather than leaving it on.

## The one thing that makes the comparison valid

Both mappers call their frame `map`, and those are **two different frames with
the same name** -- each is the rover's pose at the moment that mapper started.
They coincide only if both were started together, which is what
`slam.launch.py rtabmap:=compare` does. Started at different times, the
return-to-start and grid-agreement numbers below are meaningless, so this refuses
to report them if the two graphs did not begin within a few seconds of each
other.

## The asymmetry that cannot be removed

RTAB-Map reports its own loop closures, on `/rtabmap/info`, per update.
slam_toolbox does not report anything equivalent -- there is no topic that says
"a closure was accepted". So closures are counted for both sides the only way
that is symmetric: **a step in `map -> odom`**. A pose graph that has bent to
fit a closure moves that correction, and a graph that has merely matched the
next scan barely does. RTAB-Map's own count is reported beside it, and the two
should roughly agree; where they do not, the honest reading is that the
step threshold is wrong and not that one mapper is lying.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from rtabmap_msgs.msg import Info, MapGraph

#: How often to sample `map -> odom` out of the transform tree. slam_toolbox
#: republishes it at 20 Hz whether or not it has changed, so this only has to be
#: fast enough not to miss a correction between two of its own updates.
SAMPLE_HZ = 10.0

#: What counts as the graph having changed its mind rather than merely tracking.
#: A scan match nudges `map -> odom` by millimetres; a loop closure moves it by
#: as much as the drift it just cancelled. 5 cm or 2 degrees is comfortably above
#: the first and far below the second.
STEP_M = 0.05
STEP_RAD = math.radians(2.0)

#: RTAB-Map link types that mean "this is a closure" rather than "these two nodes
#: were consecutive". From rtabmap_msgs/Link: 0 is a neighbour, 1 a global
#: closure, 2 a local-space closure (which is what proximity detection produces
#: on a lidar), 3 local-time, 4 user.
CLOSURE_LINK_TYPES = {1, 2, 3, 4}

#: Two graphs started more than this far apart do not share a map frame, so the
#: numbers that compare their frames cannot be reported.
COSTART_TOLERANCE_S = 20.0


def yaw_of(q):
    """Yaw from a quaternion, the flat-ground way base_node.py does it."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    """`a - b` wrapped to (-pi, pi]."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def pose_of(transform):
    """`(x, y, yaw)` from a geometry_msgs Transform."""
    t = transform.translation
    return t.x, t.y, yaw_of(transform.rotation)


class Series:
    """A sequence of `map -> odom` readings, and what changed between them."""

    def __init__(self, name):
        self.name = name
        self.last = None
        self.first = None
        self.first_at = None
        self.samples = 0
        self.steps = []          # (translation m, rotation rad) of each step

    def add(self, pose, at):
        if pose is None:
            return
        self.samples += 1
        if self.first is None:
            self.first, self.first_at = pose, at
        if self.last is not None:
            dx = pose[0] - self.last[0]
            dy = pose[1] - self.last[1]
            dt = math.hypot(dx, dy)
            dr = abs(angle_diff(pose[2], self.last[2]))
            if dt >= STEP_M or dr >= STEP_RAD:
                self.steps.append((dt, dr))
        self.last = pose

    def summary(self):
        moves = [s[0] for s in self.steps]
        turns = [math.degrees(s[1]) for s in self.steps]
        return {
            "samples": self.samples,
            "corrections": len(self.steps),
            "correction_m_mean": round(sum(moves) / len(moves), 4) if moves else 0.0,
            "correction_m_max": round(max(moves), 4) if moves else 0.0,
            "correction_deg_max": round(max(turns), 3) if turns else 0.0,
        }


class Compare(Node):

    def __init__(self, args):
        super().__init__("slam_compare")
        self.args = args
        self.started = time.time()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # slam_toolbox's correction comes off the transform tree, because that is
        # the answer Nav2 actually steers on. RTAB-Map's comes off mapGraph,
        # because in compare mode it is deliberately not allowed to publish a
        # transform at all -- run_rtabmap.sh explains why a second publisher of
        # `map -> odom` is not a second opinion but a corrupted one.
        self.st = Series("slam_toolbox")
        self.rt = Series("rtabmap")

        self.st_grid = None
        self.rt_grid = None
        self.st_nodes = 0
        self.rt_nodes = 0
        self.rt_links = 0
        self.rt_closure_links = 0
        self.rt_closure_events = 0
        self.rt_first_graph_at = None
        self.st_first_map_at = None
        self.update_ms = []
        self.tf_age_s = []

        # Latched: both mappers publish their grid transient-local so that a
        # subscriber arriving late still gets the current map rather than waiting
        # for the next rebuild.
        latched = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(OccupancyGrid, "/map", self.on_st_map, latched)
        self.create_subscription(OccupancyGrid, "/rtabmap/map", self.on_rt_map, latched)
        self.create_subscription(MapGraph, "/rtabmap/mapGraph", self.on_graph, 10)
        self.create_subscription(Info, "/rtabmap/info", self.on_info, 10)
        self.create_subscription(MarkerArray, "/slam_toolbox/graph_visualization",
                                 self.on_st_graph, 10)

        self.create_timer(1.0 / SAMPLE_HZ, self.sample)

        self.pids = self.find_pids()
        self.cpu0 = {name: self.read_cpu(pid) for name, pid in self.pids.items()}

    # --- what each mapper says ----------------------------------------------
    def on_st_map(self, msg):
        self.st_grid = msg
        if self.st_first_map_at is None:
            self.st_first_map_at = time.time()

    def on_rt_map(self, msg):
        self.rt_grid = msg

    def on_graph(self, msg):
        if self.rt_first_graph_at is None:
            self.rt_first_graph_at = time.time()
        self.rt_nodes = len(msg.poses_id)
        self.rt_links = len(msg.links)
        self.rt_closure_links = sum(1 for link in msg.links
                                    if link.type in CLOSURE_LINK_TYPES)
        self.rt.add(pose_of(msg.map_to_odom), time.time())

    def on_info(self, msg):
        # RTAB-Map's own answer to "did anything close", which slam_toolbox has
        # no equivalent of. Either field being non-zero is a closure: the first
        # is appearance-based and cannot fire without a camera, the second is
        # proximity detection, which is the whole loop-closure mechanism in a
        # lidar-only configuration.
        if msg.loop_closure_id > 0 or msg.proximity_detection_id > 0:
            self.rt_closure_events += 1
        stats = dict(zip(msg.stats_keys, msg.stats_values))
        total = stats.get("Timing/Total/ms")
        if total is not None:
            self.update_ms.append(float(total))

    def on_st_graph(self, msg):
        # slam_toolbox draws one marker per node in its pose graph. There is no
        # message that reports the edges, which is why closures are counted from
        # the correction rather than from here.
        self.st_nodes = len(msg.markers)

    def sample(self):
        now = time.time()
        self.st.add(self.lookup("map", "odom"), now)
        age = self.transform_age("map", "odom")
        if age is not None:
            self.tf_age_s.append(age)

    # --- the transform tree ---------------------------------------------------
    def lookup(self, parent, child):
        try:
            at = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1))
        except Exception:
            return None
        return pose_of(at.transform)

    def transform_age(self, parent, child):
        """How stale `map -> odom` is, in seconds.

        The failure this measures has bitten this rover before and has its own
        section in the README: when the mapper is busy the correction stops being
        republished, the controller refuses one older than its 0.3 s tolerance,
        and the goal comes back as code 102 with nothing recovering from it. If
        RTAB-Map is to take this job over, its worst case here is the number that
        decides whether it can.
        """
        try:
            at = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1))
        except Exception:
            return None
        stamp = at.header.stamp.sec + at.header.stamp.nanosec * 1e-9
        return max(0.0, self.get_clock().now().nanoseconds * 1e-9 - stamp)

    # --- what the processes cost ---------------------------------------------
    @staticmethod
    def find_pids():
        """The two mapper processes, by the path each was started from."""
        wanted = {"slam_toolbox": "async_slam_toolbox_node",
                  "rtabmap": "rtabmap_slam/rtabmap"}
        found = {}
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            try:
                with open(f"/proc/{entry}/cmdline", "rb") as handle:
                    cmd = handle.read().replace(b"\0", b" ").decode("utf-8", "replace")
            except OSError:
                continue
            for name, needle in wanted.items():
                if needle in cmd and name not in found:
                    found[name] = int(entry)
        return found

    @staticmethod
    def read_cpu(pid):
        """`(cpu_seconds, rss_bytes)` for one process, or None if it is gone."""
        try:
            with open(f"/proc/{pid}/stat", "r", encoding="utf-8") as handle:
                fields = handle.read().rsplit(") ", 1)[1].split()
            ticks = os.sysconf("SC_CLK_TCK")
            cpu = (int(fields[11]) + int(fields[12])) / ticks
            with open(f"/proc/{pid}/statm", "r", encoding="utf-8") as handle:
                rss = int(handle.read().split()[1]) * os.sysconf("SC_PAGE_SIZE")
            return cpu, rss
        except (OSError, IndexError, ValueError):
            return None

    def cost(self):
        elapsed = max(1e-6, time.time() - self.started)
        out = {}
        for name, pid in self.pids.items():
            now = self.read_cpu(pid)
            was = self.cpu0.get(name)
            if now is None or was is None:
                out[name] = {"pid": pid, "note": "process went away"}
                continue
            out[name] = {
                "pid": pid,
                "cpu_percent_of_one_core": round(100.0 * (now[0] - was[0]) / elapsed, 1),
                "rss_mb": round(now[1] / 1e6, 1),
            }
        return out

    # --- the grids ------------------------------------------------------------
    @staticmethod
    def grid_stats(grid):
        if grid is None:
            return None
        cells = grid.data
        occupied = sum(1 for c in cells if c >= 65)
        free = sum(1 for c in cells if 0 <= c <= 25)
        return {
            "width": grid.info.width,
            "height": grid.info.height,
            "resolution": round(grid.info.resolution, 4),
            "metres": [round(grid.info.width * grid.info.resolution, 2),
                       round(grid.info.height * grid.info.resolution, 2)],
            "occupied_cells": occupied,
            "free_cells": free,
            "known_fraction": round((occupied + free) / max(1, len(cells)), 4),
        }

    def grid_agreement(self):
        """How much of the two grids says the same thing about the same place.

        Compared in world coordinates rather than cell for cell: the two grids
        have the same 5 cm resolution but grow to fit what each mapper has seen,
        so their origins differ. Only cells both mappers call known are counted,
        because "one of them has not been there yet" is not a disagreement.
        """
        a, b = self.st_grid, self.rt_grid
        if a is None or b is None:
            return None
        if abs(a.info.resolution - b.info.resolution) > 1e-6:
            return {"note": "different resolutions, not comparable"}

        res = a.info.resolution
        ax, ay = a.info.origin.position.x, a.info.origin.position.y
        bx, by = b.info.origin.position.x, b.info.origin.position.y

        def classify(v):
            if v < 0:
                return None
            return 1 if v >= 65 else (0 if v <= 25 else None)

        both = agree = 0
        for row in range(a.info.height):
            wy = ay + (row + 0.5) * res
            brow = int((wy - by) / res)
            if brow < 0 or brow >= b.info.height:
                continue
            arow = row * a.info.width
            brow_off = brow * b.info.width
            for col in range(a.info.width):
                wx = ax + (col + 0.5) * res
                bcol = int((wx - bx) / res)
                if bcol < 0 or bcol >= b.info.width:
                    continue
                va = classify(a.data[arow + col])
                vb = classify(b.data[brow_off + bcol])
                if va is None or vb is None:
                    continue
                both += 1
                if va == vb:
                    agree += 1
        return {
            "cells_both_know": both,
            "agree": agree,
            "agreement": round(agree / both, 4) if both else None,
        }

    # --- where each thinks the rover ended up --------------------------------
    def where(self):
        """Each mapper's opinion of the rover's pose in its own map frame.

        slam_toolbox's is read straight off the tree. RTAB-Map's is composed by
        hand from its `map -> odom` and the tree's `odom -> base_link`, because
        in compare mode it publishes no transform for the tree to hold.
        """
        st = self.lookup("map", "base_link")
        odom = self.lookup("odom", "base_link")
        rt = None
        if self.rt.last is not None and odom is not None:
            mx, my, myaw = self.rt.last
            ox, oy, oyaw = odom
            cos, sin = math.cos(myaw), math.sin(myaw)
            rt = (mx + cos * ox - sin * oy,
                  my + sin * ox + cos * oy,
                  angle_diff(myaw + oyaw, 0.0))
        return st, rt, odom

    def report(self):
        st_now, rt_now, odom_now = self.where()

        costart = None
        if self.rt_first_graph_at and self.st_first_map_at:
            costart = abs(self.rt_first_graph_at - self.st_first_map_at)

        out = {
            "seconds": round(time.time() - self.started, 1),
            "co_started_within_s": round(costart, 1) if costart is not None else None,
            "frames_comparable": bool(costart is not None and costart <= COSTART_TOLERANCE_S),
            "slam_toolbox": dict(self.st.summary(), graph_nodes=self.st_nodes,
                                 grid=self.grid_stats(self.st_grid)),
            "rtabmap": dict(self.rt.summary(), graph_nodes=self.rt_nodes,
                            graph_links=self.rt_links,
                            closure_links=self.rt_closure_links,
                            closure_events_reported=self.rt_closure_events,
                            grid=self.grid_stats(self.rt_grid)),
            "grid_agreement": self.grid_agreement(),
            "cost": self.cost(),
        }

        if self.update_ms:
            ordered = sorted(self.update_ms)
            out["rtabmap"]["update_ms_mean"] = round(sum(ordered) / len(ordered), 1)
            out["rtabmap"]["update_ms_max"] = round(ordered[-1], 1)
            out["rtabmap"]["update_ms_p95"] = round(ordered[int(0.95 * (len(ordered) - 1))], 1)
        if self.tf_age_s:
            ordered = sorted(self.tf_age_s)
            out["slam_toolbox"]["map_odom_age_s_max"] = round(ordered[-1], 3)
            out["slam_toolbox"]["map_odom_age_s_p95"] = round(
                ordered[int(0.95 * (len(ordered) - 1))], 3)

        for name, pose in (("slam_toolbox", st_now), ("rtabmap", rt_now)):
            if pose is not None:
                out[name]["pose_now"] = [round(v, 3) for v in pose]
        if odom_now is not None:
            out["dead_reckoned_now"] = [round(v, 3) for v in odom_now]

        # Return-to-start only means anything if somebody drove a circuit and
        # said so, and only if both map frames are the same frame.
        if self.args.closed_loop:
            if not out["frames_comparable"]:
                out["return_to_start"] = {
                    "note": "the two mappers did not start together, so their "
                            "map frames are different frames with the same name"}
            else:
                out["return_to_start"] = {
                    name: round(math.hypot(pose[0], pose[1]), 3)
                    for name, pose in (("slam_toolbox", st_now), ("rtabmap", rt_now))
                    if pose is not None}
                if odom_now is not None:
                    out["return_to_start"]["dead_reckoning"] = round(
                        math.hypot(odom_now[0], odom_now[1]), 3)
        return out


def render(result):
    """The table a person reads, from the JSON a script reads."""
    lines = []
    add = lines.append
    add("")
    add(f"  over {result['seconds']} s"
        + ("" if result["frames_comparable"]
           else "   !! the two mappers did not start together"))
    add("")
    add(f"  {'':28} {'slam_toolbox':>16} {'rtabmap':>16}")
    st, rt = result["slam_toolbox"], result["rtabmap"]

    def row(label, key, fmt="{}"):
        a = st.get(key)
        b = rt.get(key)
        add(f"  {label:28} {fmt.format(a) if a is not None else '-':>16}"
            f" {fmt.format(b) if b is not None else '-':>16}")

    row("graph nodes", "graph_nodes")
    row("corrections", "corrections")
    row("largest correction (m)", "correction_m_max", "{:.3f}")
    row("largest correction (deg)", "correction_deg_max", "{:.2f}")
    add(f"  {'closures rtabmap reports':28} {'-':>16}"
        f" {rt.get('closure_events_reported', '-'):>16}")
    add(f"  {'closure links in graph':28} {'-':>16}"
        f" {rt.get('closure_links', '-'):>16}")

    for label, key in (("update (ms, mean)", "update_ms_mean"),
                       ("update (ms, worst)", "update_ms_max"),
                       ("map->odom age (s, worst)", "map_odom_age_s_max")):
        a, b = st.get(key), rt.get(key)
        if a is not None or b is not None:
            add(f"  {label:28} {a if a is not None else '-':>16}"
                f" {b if b is not None else '-':>16}")

    for label, key in (("grid, metres", "metres"), ("occupied cells", "occupied_cells")):
        a = (st.get("grid") or {}).get(key)
        b = (rt.get("grid") or {}).get(key)
        add(f"  {label:28} {str(a) if a is not None else '-':>16}"
            f" {str(b) if b is not None else '-':>16}")

    cost = result.get("cost", {})
    for label, key in (("cpu (% of one core)", "cpu_percent_of_one_core"),
                       ("memory (MB)", "rss_mb")):
        a = cost.get("slam_toolbox", {}).get(key)
        b = cost.get("rtabmap", {}).get(key)
        add(f"  {label:28} {a if a is not None else '-':>16}"
            f" {b if b is not None else '-':>16}")

    agreement = result.get("grid_agreement") or {}
    if agreement.get("agreement") is not None:
        add("")
        add(f"  the two grids agree about {agreement['agreement'] * 100:.1f}% of the"
            f" {agreement['cells_both_know']} cells both have seen")

    rts = result.get("return_to_start")
    if rts:
        add("")
        if "note" in rts:
            add(f"  return to start: {rts['note']}")
        else:
            add("  back at the starting point, each says it is this far from it:")
            for name in ("slam_toolbox", "rtabmap", "dead_reckoning"):
                if name in rts:
                    add(f"      {name:16} {rts[name]:.3f} m")
    add("")
    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--seconds", type=float, default=180.0,
                   help="how long to watch (default 180)")
    p.add_argument("--out", help="write the numbers here as JSON")
    p.add_argument("--closed-loop", action="store_true",
                   help="the rover was driven back to where it started, so "
                        "report how far each mapper thinks it is from it")
    args = p.parse_args()

    rclpy.init()
    node = Compare(args)
    if not node.pids.get("rtabmap"):
        print("slam_compare: no rtabmap process found -- start the stack with "
              "rtabmap:=compare", file=sys.stderr)
    deadline = time.time() + args.seconds
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass

    result = node.report()
    print(render(result))
    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            json.dump(result, handle, indent=2)
        print(f"  written to {args.out}\n")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
