#!/usr/bin/env python3
"""Ask the real lattice planner the same question again, and again, and again.

    python3 plan_bench.py --map recordings/trap-2026-08-25-spin.json --verify
    python3 plan_bench.py --map ... --start 1.0,0.5 --goal 6.0,3.0 --step 22.5
    python3 plan_bench.py --map ... --start ... --goal ... --step 360 --repeat 10
    python3 plan_bench.py --map ... --survey 20 --step 90
    python3 plan_bench.py --map ... --start ... --goal ... --set max_planning_time=3.0

**The rover does not move, and nothing this publishes reaches the live stack.**
It starts a *second* `planner_server` beside the live one, from the same
`config/nav2.yaml`, in its own namespace, with its own global costmap whose
static layer is fed a grid from a file rather than from slam_toolbox. The frames
are `benchmap` and `benchbase`, which exist nowhere else, so the live transform
tree is untouched. `--verify` checks the bench's costmap against the recorded
one cell by cell, and nothing measured here means anything until that passes.

**Why a second server rather than a model.** The fault this was written for is
inside `SmacPlannerLattice`, which is C++ in a shared library, so a Python
re-implementation of its search would be the thing under test rather than the
instrument. That is not a hypothetical: a model built alongside this one said
`rotation_penalty` was the fault and that lowering it would help, and this bench
showed the same query going from 9 plans in 16 to 5. The model counted
expansions to the first solution; cheap rotations widen the branching in the
heading dimension, and the real planner has to get through those states.

**What it found.** `--repeat` is the important flag, because the fault turned
out to be a stopwatch rather than a room. One query, one start heading, ten
runs: four plans and six "no valid path found", with all four successes landing
between 2.01 and 2.09 s against a `max_planning_time` of 2.0. A route of eight
to twelve metres across a mapped house costs this board about that much, so the
budget sat inside the spread and a large share of long goals were cut off
mid-search -- and Nav2 reports the clock running out as
`NoValidPathCouldBeFound`, which the console renders as "there is no route to
there". Anything that nudged the search decided which side of the line a query
landed on, the rover's start heading among them, which is why a goal behind the
rover looked like the cause. Raising the budget to 3 s planned all sixteen
headings, none of them needing more than 2.27 s.

**A warning worth heeding.** The board dropped off the network twice while this
bench was running long sweeps on it, both times needing a power cycle -- see
docs/rover-unresponsive.md. A second planner_server is a real load on four
cores that are already busy. Keep sweeps short, and do not leave one running
unattended.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import signal
import subprocess
import sys
import time

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from tf2_ros import StaticTransformBroadcaster

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "config", "nav2.yaml")
LATTICE = os.path.join(HERE, "config", "lattices", "diff_5cm_0.5m.json")

#: Everything the bench creates lives under this, and the frames below exist
#: nowhere else, so nothing here can be mistaken for the live stack.
NS = "planbench"
MAP_FRAME = "benchmap"
BASE_FRAME = "benchbase"
CHILD_LOG = "/tmp/plan_bench_planner.log"

#: costmap_2d's own values, which are not an OccupancyGrid's.
LETHAL = 254
INSCRIBED = 253
UNKNOWN = 255


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Grid:
    """A costmap snapshot, in costmap_2d's values rather than an OccupancyGrid's."""

    def __init__(self, width, height, resolution, ox, oy, cells):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.ox = ox
        self.oy = oy
        self.cells = cells

    @classmethod
    def load(cls, path):
        with open(path) as handle:
            whole = json.load(handle)
        snap = whole.get("global_costmap", whole)
        cells = base64.b64decode(snap["data"])
        return cls(snap["width"], snap["height"], snap["resolution"],
                   snap["origin"][0], snap["origin"][1], cells)

    def at(self, x, y):
        col = int((x - self.ox) / self.resolution)
        row = int((y - self.oy) / self.resolution)
        if not (0 <= col < self.width and 0 <= row < self.height):
            return UNKNOWN
        return self.cells[row * self.width + col]

    def xy(self, index):
        return (self.ox + (index % self.width + 0.5) * self.resolution,
                self.oy + (index // self.width + 0.5) * self.resolution)

    def occupancy(self):
        """The grid as slam_toolbox would have published it, before inflation.

        A recorded costmap has already been through an inflation layer, so
        feeding it back in as-is would inflate it twice and the bench would
        answer questions about a room 20 cm narrower than the recorded one.
        Only 254 is a real obstacle; 253 and everything under it is inflation,
        and the bench's own inflation layer puts it back.
        """
        out = []
        for value in self.cells:
            if value == UNKNOWN:
                out.append(-1)
            elif value >= LETHAL:
                out.append(100)
            else:
                out.append(0)
        return out


def bench_params(config, out):
    """The live planner's parameters, re-addressed to the bench's namespace."""
    with open(config) as handle:
        whole = yaml.safe_load(handle)
    planner = whole["planner_server"]["ros__parameters"]
    planner["GridBased"]["lattice_filepath"] = LATTICE
    # See dwb_bench.py: a bench that bonded would be a second heartbeat under
    # the name the live lifecycle manager is watching.
    planner["bond_heartbeat_period"] = 0.0
    costmap = whole["global_costmap"]["global_costmap"]["ros__parameters"]
    costmap["bond_heartbeat_period"] = 0.0
    costmap["global_frame"] = MAP_FRAME
    costmap["robot_base_frame"] = BASE_FRAME
    costmap["static_layer"]["map_topic"] = "/%s/map" % NS
    document = {NS: {"planner_server": {"ros__parameters": planner},
                     "global_costmap": {"global_costmap":
                                        {"ros__parameters": costmap}}}}
    with open(out, "w") as handle:
        yaml.safe_dump(document, handle, default_flow_style=False)
    return whole


class Bench(Node):
    def __init__(self, grid):
        super().__init__("plan_bench")
        self.grid = grid
        self.tf = StaticTransformBroadcaster(self)
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.map_pub = self.create_publisher(OccupancyGrid, "/%s/map" % NS,
                                             latched)
        self.planner = ActionClient(self, ComputePathToPose,
                                    "/%s/compute_path_to_pose" % NS)
        self.state = self.create_client(
            ChangeState, "/%s/planner_server/change_state" % NS)
        self.costmap = self.create_client(
            GetCostmap, "/%s/global_costmap/get_costmap" % NS)

    def place_robot(self, x=0.0, y=0.0):
        """A transform the bench costmap can find, in frames nothing shares.

        Costmap2DROS wants a robot pose every update cycle whatever the planner
        is doing with it, so one has to exist; where it is does not matter,
        because this costmap has no rolling window and no obstacle layer.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = MAP_FRAME
        t.child_frame_id = BASE_FRAME
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.grid.resolution
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = self.grid.ox
        msg.info.origin.position.y = self.grid.oy
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.occupancy()
        self.map_pub.publish(msg)

    def settle(self, future, limit=25.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=limit)
        return future.result()

    def transition(self, which, name):
        if not self.state.wait_for_service(timeout_sec=30.0):
            return "the bench planner never offered change_state"
        request = ChangeState.Request()
        request.transition.id = which
        answer = self.settle(self.state.call_async(request), 40.0)
        if answer is None:
            return "the bench planner did not answer a %s in time" % name
        if not answer.success:
            return "the bench planner refused to %s" % name
        return None

    def bench_costmap(self):
        if not self.costmap.wait_for_service(timeout_sec=15.0):
            return None
        answer = self.settle(self.costmap.call_async(GetCostmap.Request()), 20.0)
        if answer is None:
            return None
        m = answer.map
        return Grid(m.metadata.size_x, m.metadata.size_y, m.metadata.resolution,
                    m.metadata.origin.position.x, m.metadata.origin.position.y,
                    bytes(m.data))

    def plan(self, start, goal, limit=30.0):
        g = ComputePathToPose.Goal()
        g.planner_id = "GridBased"
        g.use_start = True
        for field, pose in (("start", start), ("goal", goal)):
            p = PoseStamped()
            p.header.frame_id = MAP_FRAME
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = float(pose[0])
            p.pose.position.y = float(pose[1])
            p.pose.orientation.z = math.sin(pose[2] / 2.0)
            p.pose.orientation.w = math.cos(pose[2] / 2.0)
            setattr(g, field, p)
        t0 = time.time()
        handle = self.settle(self.planner.send_goal_async(g), limit)
        if handle is None or not handle.accepted:
            return {"ok": False, "why": "the planner would not accept the goal",
                    "wall_s": time.time() - t0, "plan_s": 0.0}
        answer = self.settle(handle.get_result_async(), limit)
        wall = time.time() - t0
        if answer is None:
            return {"ok": False, "why": "the planner never answered",
                    "wall_s": wall, "plan_s": 0.0}
        res = answer.result
        poses = [(q.pose.position.x, q.pose.position.y,
                  yaw_of(q.pose.orientation)) for q in res.path.poses]
        pt = res.planning_time
        row = {"ok": bool(poses), "poses": len(poses),
               "plan_s": pt.sec + pt.nanosec * 1e-9, "wall_s": wall,
               "error_code": int(getattr(res, "error_code", 0)),
               "why": "" if poses else "no valid path found"}
        if poses:
            row["length_m"] = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                  for a, b in zip(poses, poses[1:]))
            row["pivot_deg"], row["pivot_poses"] = opening_pivot(poses)
            row["path"] = poses
        return row


def opening_pivot(poses, moved=0.03):
    """How much of a turn on the spot the route opens with, in degrees.

    A lattice path writes an in-place rotation as several poses at one point
    with the heading stepping round, so the measurement is: walk forward from
    the first pose while the rover has not translated, and total the heading
    change.
    """
    turn = 0.0
    count = 0
    for a, b in zip(poses, poses[1:]):
        if math.hypot(b[0] - a[0], b[1] - a[1]) > moved:
            break
        turn += abs(wrap(b[2] - a[2]))
        count += 1
    return math.degrees(turn), count


def verify(node, grid):
    """Does the bench costmap match the one that was recorded?

    The whole bench rests on this: a static layer plus an inflation layer fed
    the recorded obstacles has to reproduce the recorded costmap, or the planner
    here is searching a different room from the one the rover was in.
    """
    mine = node.bench_costmap()
    if mine is None:
        print("the bench costmap could not be read")
        return 1
    print("recorded %dx%d at %.3f origin %.3f,%.3f"
          % (grid.width, grid.height, grid.resolution, grid.ox, grid.oy))
    print("bench    %dx%d at %.3f origin %.3f,%.3f"
          % (mine.width, mine.height, mine.resolution, mine.ox, mine.oy))
    if (mine.width, mine.height) != (grid.width, grid.height):
        print("different shape, so nothing else here is comparable")
        return 1
    same = off = 0
    worst = {}
    for a, b in zip(grid.cells, mine.cells):
        if a == b:
            same += 1
        else:
            off += 1
            worst[(a, b)] = worst.get((a, b), 0) + 1
    total = len(grid.cells)
    print("%d of %d cells identical (%.2f%%)" % (same, total, 100.0 * same / total))
    if off:
        print("the ten commonest disagreements, recorded -> bench:")
        for (a, b), n in sorted(worst.items(), key=lambda kv: -kv[1])[:10]:
            print("   %3d -> %3d   %d cells" % (a, b, n))
    lethal_a = sum(1 for c in grid.cells if c == LETHAL)
    lethal_b = sum(1 for c in mine.cells if c == LETHAL)
    block_a = sum(1 for c in grid.cells if INSCRIBED <= c < UNKNOWN)
    block_b = sum(1 for c in mine.cells if INSCRIBED <= c < UNKNOWN)
    print("lethal cells   recorded %d, bench %d" % (lethal_a, lethal_b))
    print("inscribed ring recorded %d, bench %d" % (block_a, block_b))
    return 0 if same >= 0.99 * total else 1


def sweep(node, start_xy, goal_xy, step, repeat, out):
    sx, sy = start_xy
    gx, gy = goal_xy
    bearing = math.atan2(gy - sy, gx - sx)
    print("start (%.2f, %.2f) cost %d -> goal (%.2f, %.2f) cost %d: "
          "%.2f m straight, bearing %.0f deg"
          % (sx, sy, node.grid.at(sx, sy), gx, gy, node.grid.at(gx, gy),
             math.hypot(gx - sx, gy - sy), math.degrees(bearing)))
    print("%8s %8s %5s %7s %8s %8s %9s"
          % ("start", "off nose", "ok", "poses", "length", "plan_s", "opens"))
    rows = []
    n = max(1, int(round(360.0 / step)))
    for i in range(n):
        yaw = wrap(math.radians(i * step))
        off = math.degrees(wrap(yaw - bearing))
        for _ in range(repeat):
            r = node.plan((sx, sy, yaw), (gx, gy, bearing))
            r["yaw_deg"] = math.degrees(yaw)
            r["off_nose_deg"] = off
            rows.append(r)
            print("%8.1f %8.1f %5s %7s %8s %8.2f %9s  %s"
                  % (r["yaw_deg"], off, "yes" if r["ok"] else "NO",
                     r.get("poses", "-"),
                     ("%.2f" % r["length_m"]) if r["ok"] else "-",
                     r["plan_s"],
                     ("%.0f deg" % r["pivot_deg"]) if r["ok"] else "-",
                     r["why"]))
    good = [r for r in rows if r["ok"]]
    print("")
    print("%d of %d start headings planned" % (len(good), len(rows)))
    if good:
        print("slowest successful plan %.2f s against a max_planning_time of "
              "%.1f s" % (max(r["plan_s"] for r in good), 2.0))
    if out:
        with open(out, "w") as handle:
            json.dump({"start": [sx, sy], "goal": [gx, gy],
                       "rows": [{k: v for k, v in r.items() if k != "path"}
                                for r in rows]}, handle)
    return rows


def survey(node, count, step, seed, out):
    """Many routes across the map, each asked from every start heading.

    The question is not whether one route plans. It is whether the *same*
    route plans from every heading the rover might be standing at, because
    that difference is the whole fault: a rover that has just arrived
    somewhere is pointing the way it came, and the next goal may be behind it.
    """
    import random
    rng = random.Random(seed)
    grid = node.grid
    open_cells = [i for i, c in enumerate(grid.cells) if c < 128]
    print("%d cells the rover's centre may stand in" % len(open_cells))
    print("%6s %20s %20s %7s  %5s %5s  %8s %8s  %s"
          % ("#", "start", "goal", "line_m", "ok", "of", "median_s", "worst_s",
             "the headings that failed"))
    rows = []
    tries = 0
    made = 0
    while made < count and tries < count * 20:
        tries += 1
        a = grid.xy(rng.choice(open_cells))
        b = grid.xy(rng.choice(open_cells))
        line = math.hypot(b[0] - a[0], b[1] - a[1])
        if line < 3.0:
            continue
        made += 1
        bearing = math.atan2(b[1] - a[1], b[0] - a[0])
        outcomes = []
        n = max(1, int(round(360.0 / step)))
        for i in range(n):
            yaw = wrap(math.radians(i * step))
            r = node.plan((a[0], a[1], yaw), (b[0], b[1], bearing))
            r["yaw_deg"] = math.degrees(yaw)
            r["off_nose_deg"] = math.degrees(wrap(yaw - bearing))
            outcomes.append(r)
        good = [r for r in outcomes if r["ok"]]
        times = sorted(r["plan_s"] for r in outcomes)
        failed = [r["off_nose_deg"] for r in outcomes if not r["ok"]]
        print("%6d %20s %20s %7.2f  %5d %5d  %8.2f %8.2f  %s"
              % (made, "%.2f, %.2f" % a, "%.2f, %.2f" % b, line,
                 len(good), len(outcomes), times[len(times) // 2], times[-1],
                 ", ".join("%.0f" % f for f in failed) or "-"))
        rows.append({"start": list(a), "goal": list(b), "line_m": line,
                     "rows": [{k: v for k, v in r.items() if k != "path"}
                              for r in outcomes]})
    every = [r for row in rows for r in row["rows"]]
    hard = [row for row in rows if any(not r["ok"] for r in row["rows"])]
    print("")
    print("%d of %d queries planned; %d of %d routes failed from at least one "
          "start heading" % (sum(1 for r in every if r["ok"]), len(every),
                             len(hard), len(rows)))
    if hard:
        by_off = {}
        for row in hard:
            for r in row["rows"]:
                key = int(round(abs(r["off_nose_deg"]) / step) * step)
                seen = by_off.setdefault(key, [0, 0])
                seen[1] += 1
                if not r["ok"]:
                    seen[0] += 1
        print("failures by how far the goal was off the nose:")
        for key in sorted(by_off):
            bad, all_of = by_off[key]
            print("   %4d deg   %d of %d failed" % (key, bad, all_of))
    if out:
        with open(out, "w") as handle:
            json.dump(rows, handle)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--survey", type=int, default=0,
                    help="sample this many routes across the map instead")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--map", required=True,
                    help="a nav_record.py recording, or a bare costmap snapshot")
    ap.add_argument("--start", default="", help="x,y in the recorded map frame")
    ap.add_argument("--goal", default="", help="x,y in the recorded map frame")
    ap.add_argument("--step", type=float, default=22.5,
                    help="degrees between start headings")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--verify", action="store_true",
                    help="compare the bench costmap with the recorded one and stop")
    ap.add_argument("--out", default="")
    ap.add_argument("--set", action="append", default=[],
                    help="planner overrides, name=value, applied to GridBased")
    args = ap.parse_args()

    grid = Grid.load(args.map)
    params = "/tmp/plan_bench_params.yaml"
    bench_params(CONFIG, params)
    if args.set:
        with open(params) as handle:
            doc = yaml.safe_load(handle)
        block = doc[NS]["planner_server"]["ros__parameters"]["GridBased"]
        for pair in args.set:
            name, _, value = pair.partition("=")
            try:
                value = json.loads(value)
            except ValueError:
                pass
            block[name] = value
            print("override: GridBased.%s = %r" % (name, value))
        with open(params, "w") as handle:
            yaml.safe_dump(doc, handle, default_flow_style=False)

    running = subprocess.run(["pgrep", "-f", "__ns:=/%s" % NS],
                             stdout=subprocess.PIPE, text=True).stdout.split()
    running = [p for p in running if p != str(os.getpid())]
    if running:
        print("a bench planner is already running (pid %s); "
              "pkill -f '__ns:=/%s'" % (", ".join(running), NS))
        return 1

    log = open(CHILD_LOG, "w")
    child = subprocess.Popen(
        ["ros2", "run", "nav2_planner", "planner_server", "--ros-args",
         "-r", "__ns:=/%s" % NS, "--params-file", params],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Bench(grid)
    try:
        node.place_robot()
        node.publish_map()
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        for which, name in ((Transition.TRANSITION_CONFIGURE, "configure"),
                            (Transition.TRANSITION_ACTIVATE, "activate")):
            problem = node.transition(which, name)
            if problem:
                print(problem)
                time.sleep(2.0)
                print("its own log, %s, says:" % CHILD_LOG)
                with open(CHILD_LOG) as handle:
                    for line in handle.readlines()[-25:]:
                        print("  " + line.rstrip())
                return 1
        node.publish_map()
        deadline = time.time() + 5.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print("the bench planner is up on /%s, reading a map from %s"
              % (NS, os.path.basename(args.map)))

        if args.verify:
            return verify(node, grid)

        if not node.planner.wait_for_server(timeout_sec=20.0):
            print("the bench planner never offered compute_path_to_pose")
            return 1
        if args.survey:
            survey(node, args.survey, args.step, args.seed, args.out)
            return 0
        if not (args.start and args.goal):
            print("give --start and --goal, or --verify")
            return 1
        start = [float(v) for v in args.start.split(",")]
        goal = [float(v) for v in args.goal.split(",")]
        sweep(node, start, goal, args.step, args.repeat, args.out)
        return 0
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
        for send, wait in ((signal.SIGTERM, 12.0), (signal.SIGKILL, 5.0)):
            try:
                os.killpg(os.getpgid(child.pid), send)
            except (ProcessLookupError, PermissionError):
                pass
            end = time.monotonic() + wait
            while time.monotonic() < end:
                left = subprocess.run(["pgrep", "-f", "__ns:=/%s" % NS],
                                      stdout=subprocess.PIPE,
                                      text=True).stdout.split()
                if not left:
                    break
                time.sleep(0.5)
        log.close()


if __name__ == "__main__":
    sys.exit(main())
