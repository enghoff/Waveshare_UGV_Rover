#!/usr/bin/env python3
"""Record everything a failing drive is made of, so it can be replayed at a desk.

    python3 nav_record.py --seconds 180 --out /tmp/episode.json

**Why this exists.** Three attempts at this rover's "drives but goes nowhere"
fault have been built on invented geometry, and each one reproduced something
that turned out not to be what the rover was doing. A simulation is only worth
anything if it can be checked against what the real controller actually
commanded, tick by tick, from the same inputs. This records those inputs and
that output together:

    /plan                 the route the controller was handed, 1 Hz
    local_costmap         what it was avoiding, 2 Hz, in the odom frame
    odom -> base_link     where it thought it was, 10 Hz
    map -> base_link      and where that was on the map
    /cmd_vel_nav          what DWB decided, before the velocity smoother

`dwb_replay.py` reads the result and re-scores every tick offline. If its
choice matches the recorded `/cmd_vel_nav`, the model is a fair copy of the
controller and can be used to test a change; if it does not, the model is
wrong and nothing built on it means anything. That check is the whole point.

**It records the local costmap and not the global one.** The controller reads
the local, it is 60x60 rather than 283x276, and at 2 Hz for three minutes the
global would be three hundred megabytes of mostly unchanged cells. The global
is written once at the start, for context.

Nothing here publishes. It is one more Python node on a board with four cores
and no spare ones, so keep the recordings short.
"""

from __future__ import annotations

import argparse
import base64
import json
import math

from record_metrics import turning_of
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.srv import GetCostmap
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

POSE_HZ = 10.0
COSTMAP_HZ = 2.0

#: The controller settings a replay has to know to be a replay *of this drive*.
#:
#: Without these a recording is undated: `dwb_replay.py` would score it with
#: whatever is in `corridor_sim.py` today, which after a change is a different
#: controller from the one that produced the commands. That is not a small
#: error -- moving the align critics' look-ahead from 0.1 to 0.8 took the
#: model's agreement with this very recording from 82% to 1%, and the 1% was
#: the honest number, because by then the model was describing a controller the
#: rover had never run.
WANTED_PARAMS = [
    "FollowPath.PathAlign.forward_point_distance",
    "FollowPath.GoalAlign.forward_point_distance",
    "FollowPath.PathAlign.scale",
    "FollowPath.GoalAlign.scale",
    "FollowPath.PathDist.scale",
    "FollowPath.GoalDist.scale",
    # **The obstacle critic is named here, and the name changes with the
    # footprint.** `ObstacleFootprint` was in this list until the body went back
    # to a circle and `BaseObstacle` replaced it; asking for the dead name cost
    # every recording made afterwards its entire settings block, because
    # `rclcpp`'s parameter service answers a batch containing one undeclared
    # name with an *empty* reply rather than with the fifteen it does have. The
    # rover said so and nobody was reading: `[controller_server] [rclcpp]:
    # Failed to get parameters: FollowPath.ObstacleFootprint.scale`. `fetch_params`
    # asks one name at a time now, so the next critic swap costs one line of the
    # block instead of all of it.
    "FollowPath.BaseObstacle.scale",
    "FollowPath.PreferForward.scale",
    "FollowPath.sim_time",
    "FollowPath.vx_samples",
    "FollowPath.vtheta_samples",
    "FollowPath.max_vel_x",
    "FollowPath.max_vel_theta",
    "FollowPath.acc_lim_x",
    "FollowPath.acc_lim_theta",
    # **The two that decide which candidates exist at all.** Everything above
    # changes how a candidate scores; these change whether DWB ever offers it.
    # A drive recorded under one pair and replayed under another is not a
    # disagreement about scoring, it is the model being asked why the rover
    # picked a twist that was never on the list -- and it looks exactly like a
    # broken critic. Both recordings made before this line existed contain
    # standing turns at 0.052 rad/s, which today's floor drops: 62% of the
    # commands in one of them and 45% in the other.
    "FollowPath.min_speed_xy",
    "FollowPath.min_speed_theta",
]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)


class Recorder(Node):
    """Subscribes to everything and keeps it in memory until the end."""

    def __init__(self):
        super().__init__("nav_record")
        self.started = time.monotonic()
        self.plans = []
        self.costmaps = []
        self.poses = []
        self.commands = []
        self.global_costmap = None

        # The plan is published latched-ish by bt_navigator; take it reliable.
        self.create_subscription(Path, "plan", self.on_plan, 10)
        self.create_subscription(Twist, "cmd_vel_nav", self.on_cmd, 10)
        self.buffer = Buffer()
        TransformListener(self.buffer, self)
        self.costmap_client = self.create_client(
            GetCostmap, "/local_costmap/get_costmap")
        self.global_client = self.create_client(
            GetCostmap, "/global_costmap/get_costmap")
        self.param_client = self.create_client(
            GetParameters, "/controller_server/get_parameters")
        self.params = {}
        self.create_timer(1.0 / POSE_HZ, self.sample_pose)
        self.create_timer(1.0 / COSTMAP_HZ, self.sample_costmap)
        self.pending = None

    def stamp(self):
        return round(time.monotonic() - self.started, 3)

    def on_plan(self, msg):
        self.plans.append({
            "t": self.stamp(),
            "frame": msg.header.frame_id,
            "poses": [[round(p.pose.position.x, 4), round(p.pose.position.y, 4),
                       round(yaw_of(p.pose.orientation), 4)]
                      for p in msg.poses],
        })

    def on_cmd(self, msg):
        self.commands.append({"t": self.stamp(),
                              "vx": round(msg.linear.x, 4),
                              "wz": round(msg.angular.z, 4)})

    def sample_pose(self):
        row = {"t": self.stamp()}
        for frame in ("odom", "map"):
            try:
                t = self.buffer.lookup_transform(frame, "base_link",
                                                 rclpy.time.Time())
            except Exception:
                continue
            row[frame] = [round(t.transform.translation.x, 4),
                          round(t.transform.translation.y, 4),
                          round(yaw_of(t.transform.rotation), 4)]
        if len(row) > 1:
            self.poses.append(row)

    def sample_costmap(self):
        """One GetCostmap in flight at a time, so a slow answer cannot pile up."""
        if self.pending is not None:
            if not self.pending[1].done():
                return
            answer = self.pending[1].result()
            if answer is not None:
                self.costmaps.append(self.pack(self.pending[0], answer.map))
            self.pending = None
        if not self.costmap_client.service_is_ready():
            return
        self.pending = (self.stamp(),
                        self.costmap_client.call_async(GetCostmap.Request()))

    @staticmethod
    def pack(stamp, grid):
        return {
            "t": stamp,
            "width": grid.metadata.size_x,
            "height": grid.metadata.size_y,
            "resolution": round(grid.metadata.resolution, 4),
            "origin": [round(grid.metadata.origin.position.x, 4),
                       round(grid.metadata.origin.position.y, 4)],
            "data": base64.b64encode(bytes(bytearray(
                c & 0xFF for c in grid.data))).decode("ascii"),
        }

    def fetch_params(self):
        """Ask the controller what it is running, once, at the start.

        One name per request, which is slower and cannot fail silently. A batch
        is all-or-nothing: `rclcpp`'s parameter service catches the not-declared
        exception and returns with `values` empty, so a single stale name in the
        list -- a critic that has been renamed, a setting that moved -- takes the
        whole block down and leaves the recording undated. That is exactly what
        happened to every drive recorded between the footprint going back to a
        circle and this being noticed, and it is invisible from here: the reply
        arrives, it is simply empty.
        """
        # **Twenty seconds, not five, and the difference is the whole block.**
        # Under the loopback-only discovery `dds.sh` pins, a node that has just
        # started takes about five and a half seconds to see
        # `/controller_server/get_parameters` -- measured on the rover, where a
        # 5 s wait returned False and a 10 s wait returned True after 0.5 s more.
        # So a five-second limit sat exactly on the boundary and lost the
        # settings to a race, which looks identical to the dead-parameter-name
        # bug this method was written to fix: the recording simply arrives with
        # no settings and a warning nobody can act on.
        if not self.param_client.wait_for_service(timeout_sec=20.0):
            print("the controller's parameter service never appeared, so this "
                  "recording will not carry its settings", file=sys.stderr)
            return
        missing = []
        for name in WANTED_PARAMS:
            request = GetParameters.Request()
            request.names = [name]
            future = self.param_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            answer = future.result()
            if answer is None or not answer.values:
                missing.append(name)
                continue
            value = answer.values[0]
            if value.type == 3:
                self.params[name] = value.double_value
            elif value.type == 2:
                self.params[name] = value.integer_value
            elif value.type == 1:
                self.params[name] = value.bool_value
            else:
                missing.append(name)
        if missing:
            print("the controller does not have these settings, so this "
                  "recording will not carry them:", file=sys.stderr)
            for name in missing:
                print("   %s" % name, file=sys.stderr)

    def fetch_global(self):
        if not self.global_client.wait_for_service(timeout_sec=5.0):
            return
        future = self.global_client.call_async(GetCostmap.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None:
            self.global_costmap = self.pack(self.stamp(), future.result().map)

    def episode(self):
        return {
            "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "note": "local costmap is in the odom frame; the plan is in map",
            "params": self.params,
            "plans": self.plans,
            "costmaps": self.costmaps,
            "poses": self.poses,
            "commands": self.commands,
            "global_costmap": self.global_costmap,
        }




def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--seconds", type=float, default=180.0)
    parser.add_argument("--out", default="/tmp/episode.json")
    args = parser.parse_args()

    # `rclpy.spin_once(..., timeout_sec=0.05)` does not return when CycloneDDS
    # is wedged, so the wall-clock loop below never sees `end`. A --seconds 60
    # recording then lives for tens of minutes as a second participant and
    # takes the graph with it. This timer is independent of DDS.
    def _abort():
        print("nav_record: spin blocked past the recording window; "
              "DDS is wedged", file=sys.stderr, flush=True)
        os._exit(2)

    watchdog = threading.Timer(args.seconds + 60.0, _abort)
    watchdog.daemon = True
    watchdog.start()

    rclpy.init()
    node = Recorder()
    node.fetch_params()
    node.fetch_global()
    if node.params:
        print("controller settings at the time of this drive:")
        for name in sorted(node.params):
            print("   %-52s %s" % (name, node.params[name]))
    else:
        print("WARNING: could not read the controller's parameters, so this "
              "recording will not know what settings produced it")
    print("recording %.0f s to %s -- drive the rover now" % (args.seconds,
                                                             args.out),
          flush=True)
    end = time.monotonic() + args.seconds
    last = 0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        done = int(time.monotonic() - node.started)
        if done != last and done % 30 == 0:
            print("  %3ds  %d plans, %d costmaps, %d poses, %d commands"
                  % (done, len(node.plans), len(node.costmaps),
                     len(node.poses), len(node.commands)), flush=True)
            last = done

    episode = node.episode()
    with open(args.out, "w") as handle:
        json.dump(episode, handle)
    size = os.path.getsize(args.out) / 1e6
    print("wrote %s, %.1f MB: %d plans, %d costmaps, %d poses, %d commands"
          % (args.out, size, len(episode["plans"]), len(episode["costmaps"]),
             len(episode["poses"]), len(episode["commands"])))
    moved = 0.0
    poses = [p["odom"] for p in episode["poses"] if "odom" in p]
    for a, b in zip(poses, poses[1:]):
        moved += math.hypot(b[0] - a[0], b[1] - a[1])
    if poses:
        net = math.hypot(poses[-1][0] - poses[0][0], poses[-1][1] - poses[0][1])
        turned = turning_of(poses)
        print("the rover drove %.2f m of path, turned %.0f deg, and finished "
              "%.2f m from where it started" % (moved, math.degrees(turned),
                                                net))
        if moved > 0.3 and net < 0.3:
            print("that is the fault: it moved and did not go anywhere")
    node.destroy_node()
    rclpy.shutdown()
    watchdog.cancel()
    return 0


if __name__ == "__main__":
    sys.exit(main())
