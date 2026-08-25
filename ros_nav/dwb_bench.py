#!/usr/bin/env python3
"""Ask DWB, in as many words, why it will not set off.

    python3 dwb_bench.py                          # a path 3 m straight ahead
    python3 dwb_bench.py --bearing 0,20,40,60     # and swept off the nose
    python3 dwb_bench.py --bearing 40 --every-tick

**The rover does not move.** This starts a *second* `controller_server` beside
the live one, from the same `config/nav2.yaml`, subscribed to the same live
`/scan` and reading the same live transform tree -- and publishes its `/cmd_vel`
inside a namespace nothing is listening to. So the real DWB, with the real
critics, scores the real room at the pose the rover is actually standing in, and
the wheels never turn. It refuses to start if anything has subscribed to that
topic, or if another bench is already running.

**Why this had to exist.** `/evaluation` is the only place that says why a rover
that could move is standing still -- three of the faults in this stack were found
in it and nowhere else -- and reading it used to mean driving the rover into the
fault first, in a room, with somebody watching. The fault this was written for is
a rover that has to turn before it can drive: it turns, and goes on turning, and
fifteen seconds later `SimpleProgressChecker` calls it stuck and Nav2 starts
clearing costmaps that were never the problem. All of that follows from one
comparison made ten times a second, so the comparison is what to look at.

**What to read.** DWB's sample set is the product of two velocity iterators, so
every candidate it scores is one of two manoeuvres: a turn on the spot
(`vx = 0`), or an arc at `max_vel_x`. The number that decides whether the rover
goes anywhere is the *margin* between the best of each. A margin of a few per
cent is a controller that has chosen; a margin of a fraction of a per cent is a
controller whose choice is being made by whatever happens to be nearest in the
room, and that is a rover that sets off on one run and dithers on the next from
the same spot.

**Three bench-only parameter changes.** `short_circuit_trajectory_evaluation`
goes off so every candidate is scored all the way through rather than abandoned
once it is losing -- the winner is identical either way, but the losers'
breakdowns are otherwise truncated to whichever critic ruled them out. And
`debug_trajectory_details` goes on. Both cost CPU and buy a complete table. The
third is not about measurement at all and is in `bench_params` below: the bench
must not publish a lifecycle bond.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time

import yaml

import route_cost

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, FollowPath
from dwb_msgs.msg import LocalPlanEvaluation
import tf2_ros

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "config", "nav2.yaml")

#: Everything the bench creates lives under this, so nothing it publishes can be
#: mistaken by the live stack for an instruction.
NS = "bench"

#: Where the bench controller's own log goes. Read it when the bench will not
#: start: a parameter it does not like is reported there and nowhere else.
CHILD_LOG = "/tmp/dwb_bench_controller.log"

#: 2 forward samples x 16 rotation samples, plus the zero the iterator injects,
#: minus the four standing turns slower than the mixer floor. Fewer than this
#: on a tick means `ObstacleFootprint` vetoed the difference.
CANDIDATES = 29


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def bench_params(path, out):
    """The live controller's parameters, re-addressed to the bench's namespace.

    Read from the same file the rover runs rather than restated here: a bench
    carrying its own copy of the numbers is a bench that answers questions about
    a controller nobody is running.
    """
    with open(path) as handle:
        whole = yaml.safe_load(handle)
    controller = whole["controller_server"]["ros__parameters"]
    controller["short_circuit_trajectory_evaluation"] = False
    controller["FollowPath"]["short_circuit_trajectory_evaluation"] = False
    controller["FollowPath"]["debug_trajectory_details"] = True
    # **No bond, and this one is about not breaking the live stack.** Every Nav2
    # lifecycle node announces itself on `/bond` under an id taken from its node
    # *name*, and the bench's name is `controller_server` too. A bench that
    # bonded would be a second heartbeat under the name the live lifecycle
    # manager is watching, and the bench exiting would read to the manager as the
    # real controller dying -- so it would restart the navigation stack
    # underneath a rover that might be driving at the time.
    controller["bond_heartbeat_period"] = 0.0
    whole["local_costmap"]["local_costmap"]["ros__parameters"][
        "bond_heartbeat_period"] = 0.0
    document = {NS: {"controller_server": {"ros__parameters": controller},
                     "local_costmap": whole["local_costmap"]}}
    with open(out, "w") as handle:
        yaml.safe_dump(document, handle, default_flow_style=False)
    return whole


class Bench(Node):
    def __init__(self):
        super().__init__("dwb_bench")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.ticks = []
        self.commands = []
        # `lambda` rather than `self.ticks.append`, and the difference is not
        # style: a bound append captures the list object, so the sweep clearing
        # its results between bearings would leave the callback filling an
        # orphan. Every run after the first then reported no ticks at all.
        self.create_subscription(LocalPlanEvaluation, "/%s/evaluation" % NS,
                                 lambda m: self.ticks.append(m), 10)
        self.follow = ActionClient(self, FollowPath, "/%s/follow_path" % NS)
        # The *live* planner, which is the point: a bench fed a straight line is
        # answering a question about a path Nav2 would never have produced. NavFn
        # searches a 5 cm grid, so leaving a spot near a wall it can set off tens
        # of degrees away from the straight-line bearing, and that angle is the
        # whole input to the choice being measured here.
        self.planner = ActionClient(self, ComputePathToPose,
                                    "/compute_path_to_pose")
        self.state = self.create_client(
            ChangeState, "/%s/controller_server/change_state" % NS)

    def settle(self, future, limit=10.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=limit)
        return future.result()

    def pose(self, frame):
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.buf.lookup_transform(frame, "base_link",
                                              rclpy.time.Time())
            except Exception:
                continue
            return (t.transform.translation.x, t.transform.translation.y,
                    yaw_of(t.transform.rotation))
        return None

    def transition(self, which, name):
        if not self.state.wait_for_service(timeout_sec=25.0):
            return "the bench controller never offered change_state"
        request = ChangeState.Request()
        request.transition.id = which
        answer = self.settle(self.state.call_async(request), 40.0)
        if answer is None:
            return "the bench controller did not answer a %s in time" % name
        if not answer.success:
            return "the bench controller refused to %s" % name
        return None

    def listeners_on_cmd_vel(self):
        """Anything at all subscribed to where the bench publishes its commands.

        Asked before this node subscribes itself, so the only safe answer is
        zero: counting its own recorder and allowing one would let a real
        subscriber through on any tick the recorder was slow to appear.
        """
        return self.count_subscribers("/%s/cmd_vel" % NS)

    def watch_commands(self):
        """Record the commands, now that nothing else is hearing them."""
        self.create_subscription(
            Twist, "/%s/cmd_vel" % NS,
            lambda m: self.commands.append((m.linear.x, m.angular.z)), 10)


def planned_path(node, goal_xy, frame):
    """The route the live planner really returns to a goal, in map coordinates.

    Read-only as far as the rover is concerned: `ComputePathToPose` plans and
    nothing else. Returns the path and a sentence about it, or None and why not.
    """
    if not node.planner.wait_for_server(timeout_sec=10.0):
        return None, "the live planner is not offering compute_path_to_pose"
    goal = ComputePathToPose.Goal()
    goal.goal = PoseStamped()
    goal.goal.header.frame_id = frame
    goal.goal.header.stamp = node.get_clock().now().to_msg()
    goal.goal.pose.position.x = float(goal_xy[0])
    goal.goal.pose.position.y = float(goal_xy[1])
    goal.goal.pose.orientation.w = 1.0
    goal.planner_id = "GridBased"
    handle = node.settle(node.planner.send_goal_async(goal), 10.0)
    if handle is None or not handle.accepted:
        return None, "the planner would not accept the goal"
    answer = node.settle(handle.get_result_async(), 30.0)
    if answer is None:
        return None, "the planner never answered"
    path = answer.result.path
    if not path.poses:
        return None, ("the planner returned no route at all (error_code %s) -- "
                      "with a goal this simple that is almost always the rover "
                      "standing in a cell the costmap calls occupied"
                      % answer.result.error_code)
    return path, "%d poses" % len(path.poses)


def sets_off_at(path, here, ahead=0.5):
    """The heading the route leaves on, and how far off the nose that is.

    Measured `ahead` metres along the route rather than at its first pose, because
    a 5 cm grid path's first step is one cell and its direction is quantised to
    eight compass points -- so the first pose says 45 degrees where the route
    means 20.
    """
    poses = [(q.pose.position.x, q.pose.position.y) for q in path.poses]
    run = 0.0
    for a, b in zip(poses, poses[1:]):
        run += math.hypot(b[0] - a[0], b[1] - a[1])
        if run >= ahead:
            bearing = math.atan2(b[1] - here[1], b[0] - here[0])
            break
    else:
        bearing = math.atan2(poses[-1][1] - here[1], poses[-1][0] - here[0])
    off = math.atan2(math.sin(bearing - here[2]), math.cos(bearing - here[2]))
    return bearing, off


def straight_path(node, start, bearing, distance, frame, spacing=0.05):
    """A path from where the rover stands, out on a bearing, one cell at a time.

    Written in the costmap's own frame, so nothing measured here depends on
    `map -> odom` being fresh. That is a real fault and it has its own simulation
    in tf_stall_sim.py; mixing the two would make each unreadable.
    """
    path = Path()
    path.header.frame_id = frame
    path.header.stamp = node.get_clock().now().to_msg()
    steps = max(2, int(distance / spacing))
    for k in range(steps + 1):
        d = k * distance / steps
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = start[0] + d * math.cos(bearing)
        pose.pose.position.y = start[1] + d * math.sin(bearing)
        pose.pose.orientation.z = math.sin(bearing / 2.0)
        pose.pose.orientation.w = math.cos(bearing / 2.0)
        path.poses.append(pose)
    return path


def families(message):
    """The best candidate among the pivots and among the forward arcs.

    The two totals are directly comparable -- the lowest total in the whole set
    is what gets driven -- so the difference between them is the margin the rover
    sets off on.
    """
    pivot = forward = None
    for score in message.twists:
        if abs(score.traj.velocity.x) < 1e-6:
            if pivot is None or score.total < pivot.total:
                pivot = score
        else:
            if forward is None or score.total < forward.total:
                forward = score
    return pivot, forward


def table(pivot, forward):
    """Critic by critic, what each family was charged.

    Both the raw score and the scale are printed, because the two are not
    comparable across critics: DWB normalises the four `MapGridCritic` scales by
    the costmap resolution and leaves the others alone, so a scale of 32 in the
    config file arrives as 0.8 for `PathAlign` and as 32 for `RotateToGoal`.
    """
    lines = ["    %-18s %20s %20s" % ("critic", "best pivot", "best forward")]
    names = []
    for score in (pivot, forward):
        for s in (score.scores if score else []):
            if s.name not in names:
                names.append(s.name)
    a = {s.name: s for s in (pivot.scores if pivot else [])}
    b = {s.name: s for s in (forward.scores if forward else [])}
    for name in names:
        cell = []
        for side in (a, b):
            s = side.get(name)
            cell.append("-" if s is None
                        else "%8.2f x%-7.4f" % (s.raw_score, s.scale))
        lines.append("    %-18s %20s %20s" % (name, cell[0], cell[1]))
    lines.append("    %-18s %20s %20s"
                 % ("TOTAL (lowest wins)",
                    "-" if pivot is None else "%8.2f" % pivot.total,
                    "-" if forward is None else "%8.2f" % forward.total))
    return lines


def tail(path, lines):
    try:
        with open(path) as handle:
            return [line.rstrip() for line in handle.readlines()[-lines:]]
    except OSError as exc:
        return ["could not be read: %s" % exc]


def already_running():
    """Any bench controller still about, by process rather than by node graph.

    The node graph is the wrong instrument: two nodes may share a name, so
    `ros2 node list` shows the duplicate and cannot say which is which, while
    `ros2 lifecycle` silently answers for whichever it found first. The command
    line is unambiguous.
    """
    found = subprocess.run(["pgrep", "-f", "__ns:=/%s" % NS],
                           stdout=subprocess.PIPE, text=True)
    mine = str(os.getpid())
    return [pid for pid in found.stdout.split() if pid and pid != mine]


def stop(child):
    """Take down the whole process group, then make sure.

    `ros2 run` is a launcher and the controller is its child, so terminating the
    `ros2` process on its own leaves an orphan holding the node name. Two of
    those is what made this tool's own first runs unreadable.
    """
    for send, wait in ((signal.SIGTERM, 12.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(os.getpgid(child.pid), send)
        except (ProcessLookupError, PermissionError):
            pass
        # Waited out against the *process list*, not against the launcher's own
        # exit: `ros2 run` returns as soon as it has passed the signal on, and
        # the controller behind it takes several seconds to unwind its costmap.
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if not already_running():
                try:
                    child.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass
                return
            time.sleep(0.5)


def report(node, args):
    """Print what one tick stream says, and hand the sweep its two numbers."""
    ticks = node.ticks
    print("  %d control ticks, %d commands published (into nothing)"
          % (len(ticks), len(node.commands)))
    if not ticks:
        print("  nothing on /%s/evaluation: either the goal was refused before "
              "the first tick, or DWB found no legal trajectory at all -- which "
              "it reports as an abort rather than as a score." % NS)
        return None

    chose_pivot = chose_forward = 0
    for message in ticks:
        best = message.twists[message.best_index]
        if abs(best.traj.velocity.x) < 1e-6:
            chose_pivot += 1
        else:
            chose_forward += 1
    counts = [len(m.twists) for m in ticks]
    print("  candidates scored per tick: %d..%d (%d is nothing vetoed)"
          % (min(counts), max(counts), CANDIDATES))
    print("  it chose to turn on the spot on %d ticks and to drive on %d"
          % (chose_pivot, chose_forward))

    if args.every_tick:
        print("  tick   chosen vx   chosen vtheta    best pivot   best forward")
        for k, message in enumerate(ticks):
            best = message.twists[message.best_index]
            pivot, forward = families(message)
            print("  %4d   %9.2f   %11.1f    %10s   %12s"
                  % (k, best.traj.velocity.x,
                     math.degrees(best.traj.velocity.theta),
                     "-" if pivot is None else "%.2f" % pivot.total,
                     "-" if forward is None else "%.2f" % forward.total))

    for message in ticks:
        pivot, forward = families(message)
        if pivot is None or forward is None:
            continue
        print("  the choice, critic by critic, on the first tick where both "
              "manoeuvres had a candidate:")
        print("    the pivot it liked best turns at %.1f deg/s; the forward one "
              "runs %.2f m/s and turns at %.1f deg/s"
              % (math.degrees(pivot.traj.velocity.theta),
                 forward.traj.velocity.x,
                 math.degrees(forward.traj.velocity.theta)))
        for line in table(pivot, forward):
            print(line)
        margin = pivot.total - forward.total
        share = 100.0 * margin / max(1e-9, max(pivot.total, forward.total))
        if margin > 0:
            print("    so the forward arc wins by %.2f (%.2f%%)"
                  % (margin, share))
        else:
            print("    so the pivot wins by %.2f (%.2f%%), and the rover turns "
                  "rather than setting off" % (-margin, -share))
        return (chose_pivot, chose_forward, margin, share)

    print("  no tick had a candidate in both families. If every forward arc is "
          "missing then ObstacleFootprint vetoed all of them, and the rover is "
          "boxed in rather than dithering.")
    return (chose_pivot, chose_forward, None, None)


def budget(path, here, goal_xy):
    """Would this goal have run out of time before it was driven?

    The other half of the fault the bench was written for, and it needs no
    controller at all -- just the route and the two constants the bridge budgets
    on. Printed here rather than left to be inferred because the two halves
    present identically from outside: a rover that turns for fifty seconds and is
    then cancelled looks the same whether it was stuck or merely on a long way
    round.

    Deliberately imports the bridge's own arithmetic. A bench that agreed with
    the bridge only by coincidence would be worse than no bench.
    """
    metres, turning = route_cost.from_path(path)
    straight = math.hypot(goal_xy[0] - here[0], goal_xy[1] - here[1])
    was = max(30.0, 6.0 * straight / 0.35)
    now = route_cost.seconds_for(metres, turning, 0.35, 27.0, slack=3.0,
                                 floor=45.0)
    need = metres / 0.40 + turning / 27.0
    print("the route is %.2f m with %.0f deg of turning in it, against a straight "
          "line of %.2f m -- a detour of %.1fx"
          % (metres, turning, straight, metres / max(0.01, straight)))
    print("driving it perfectly, with no replan and no recovery, takes about "
          "%.0f s" % need)
    print("  the allowance this used to get, from the straight line:  %5.1f s%s"
          % (was, "   <-- cancelled mid-route" if was < need else ""))
    print("  the allowance it gets now, from the route:               %5.1f s"
          % now)


def summary(runs, distance):
    """The sweep in one table. The margin is the number to read."""
    print("")
    print("=== %.1f m of path, swept across the heading it starts at ==="
          % distance)
    print("  off the nose   turned  drove   forward wins by   as a share")
    for offset, outcome in runs:
        if outcome is None:
            print("  %8.0f deg   %-6s  %-6s %17s %12s"
                  % (offset, "-", "-", "no ticks", "-"))
            continue
        pivots, forwards, margin, share = outcome
        if margin is None:
            print("  %8.0f deg   %-6d  %-6d %17s %12s"
                  % (offset, pivots, forwards, "one family only", "-"))
            continue
        print("  %8.0f deg   %-6d  %-6d %17.2f %11.2f%%"
              % (offset, pivots, forwards, margin, share))
    print("")
    print("  A negative margin is the pivot winning, which is a rover that turns")
    print("  instead of setting off. Anything inside about a per cent either way")
    print("  is a coin toss settled by the furniture rather than by the route.")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--distance", type=float, default=3.0,
                   help="how long a path to lay down, in metres")
    p.add_argument("--bearing", default="0",
                   help="degrees the path sets off at, relative to the nose; a "
                        "comma-separated list sweeps them through one bench "
                        "controller. The run this was written for had about 39")
    p.add_argument("--seconds", type=float, default=8.0,
                   help="how long to let the bench controller think per bearing")
    p.add_argument("--every-tick", action="store_true",
                   help="a line per control tick rather than a summary")
    p.add_argument("--goal", default="",
                   help="x,y in the map frame. Asks the live planner for the real "
                        "route there and scores that instead of a straight line, "
                        "which is the only way to reproduce a particular run")
    p.add_argument("--frame", default="",
                   help="the costmap frame; taken from the config when empty")
    args = p.parse_args()

    params = "/tmp/dwb_bench_params.yaml"
    whole = bench_params(CONFIG, params)
    frame = args.frame or whole["local_costmap"]["local_costmap"][
        "ros__parameters"]["global_frame"]

    running = already_running()
    if running:
        print("a bench controller is already running (pid %s). Two nodes of one "
              "name make `ros2 lifecycle` answer for whichever it found first, "
              "so stop that one first with  pkill -f '__ns:=/%s'"
              % (", ".join(running), NS))
        return 1

    # The child's own output goes to a file rather than to the terminal, where it
    # would bury the table this exists to print -- but it is the first place to
    # look when the bench will not come up, so a failure prints its tail. A
    # session of its own so the whole group can be taken down; see stop().
    log = open(CHILD_LOG, "w")
    child = subprocess.Popen(
        ["ros2", "run", "nav2_controller", "controller_server", "--ros-args",
         "-r", "__ns:=/%s" % NS, "--params-file", params],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Bench()
    try:
        for which, name in ((Transition.TRANSITION_CONFIGURE, "configure"),
                            (Transition.TRANSITION_ACTIVATE, "activate")):
            problem = node.transition(which, name)
            if problem:
                print(problem)
                time.sleep(2.0)      # its buffer has to reach the file first
                print("its own log, %s, says:" % CHILD_LOG)
                for line in tail(CHILD_LOG, 25):
                    print("  " + line)
                return 1

        listeners = node.listeners_on_cmd_vel()
        if listeners:
            print("something is subscribed to /%s/cmd_vel (%d of them), so this "
                  "bench could drive the rover. Refusing." % (NS, listeners))
            return 1
        print("the bench controller is up, and nothing is listening to "
              "/%s/cmd_vel" % NS)
        node.watch_commands()

        here = node.pose(frame)
        if here is None:
            print("no %s -> base_link, so there is no pose to plan from" % frame)
            return 1
        print("the rover stands at (%.2f, %.2f) in %s, facing %.0f deg"
              % (here[0], here[1], frame, math.degrees(here[2])))

        if not node.follow.wait_for_server(timeout_sec=20.0):
            print("the bench controller never offered follow_path")
            return 1

        legs = []
        if args.goal:
            x, y = [float(v) for v in args.goal.split(",")]
            map_frame = whole["global_costmap"]["global_costmap"][
                "ros__parameters"]["global_frame"]
            in_map = node.pose(map_frame)
            path, note = planned_path(node, (x, y), map_frame)
            if path is None:
                print("no route to score: %s" % note)
                return 1
            bearing, off = sets_off_at(path, in_map)
            print("in %s the rover is at (%.2f, %.2f) facing %.0f deg, and the "
                  "planner's route to (%.2f, %.2f) is %s"
                  % (map_frame, in_map[0], in_map[1], math.degrees(in_map[2]),
                     x, y, note))
            print("it sets off on a bearing of %.0f deg, which is %.0f deg off "
                  "the nose" % (math.degrees(bearing), math.degrees(off)))
            budget(path, in_map, (x, y))
            legs.append((math.degrees(off), path))
        else:
            for offset in [float(v) for v in
                           str(args.bearing).replace(" ", "").split(",") if v]:
                legs.append((offset, None))

        runs = []
        for offset, ready in legs:
            print("")
            if ready is not None:
                print("--- the planner's own route, %.0f deg off the nose ---"
                      % offset)
            else:
                print("--- %.1f m of path, %.0f deg off the nose ---"
                      % (args.distance, offset))
            node.ticks = []
            node.commands = []
            goal = FollowPath.Goal()
            goal.path = ready if ready is not None else straight_path(
                node, here, here[2] + math.radians(offset), args.distance, frame)
            goal.controller_id = "FollowPath"
            goal.goal_checker_id = "goal_checker"
            goal.progress_checker_id = "progress_checker"
            handle = node.settle(node.follow.send_goal_async(goal), 15.0)
            if handle is None or not handle.accepted:
                print("  the bench controller would not accept the path")
                runs.append((offset, None))
                continue
            result = handle.get_result_async()
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline and not result.done():
                rclpy.spin_once(node, timeout_sec=0.1)
            if not result.done():
                handle.cancel_goal_async()
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.1)
            runs.append((offset, report(node, args)))

        if len(runs) > 1:
            summary(runs, args.distance)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
        stop(child)
        log.close()
        left = already_running()
        if left:
            print("warning: a bench controller is still running (pid %s); "
                  "pkill -f '__ns:=/%s'" % (", ".join(left), NS))
    return 0


if __name__ == "__main__":
    sys.exit(main())
