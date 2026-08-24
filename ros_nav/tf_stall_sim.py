#!/usr/bin/env python3
"""Why a long route ends in "lost -- Nav2 gave up without saying why (code 102)".

    python3 tf_stall_sim.py                 # the budget, and the margin it leaves
    python3 tf_stall_sim.py --sweep         # how long a stall has to be to abort
    python3 tf_stall_sim.py --stall 1.0     # one loop closure's worth of stall
    python3 tf_stall_sim.py --rebuild 0.45  # a map big enough that the rebuild bites
    python3 tf_stall_sim.py --board-stall 0.6

Code 102 is the controller's `TF_ERROR`, and nav_codes.py turns it into "lost".
It is thrown by DWB in one place -- `transformGlobalPlan`, when it cannot put the
rover's pose into the frame the plan is written in, which is `map`. So the failure
is not the lidar, not the planner and not the floor: it is `map -> odom` having
gone stale relative to `odom -> base_link`, and the two are published by different
processes at different rates for different reasons.

## The four clocks, and the 0.4 s between them

This simulates the timing of the whole chain, and nothing else -- no geometry, no
costmap, no control law. Every number in it is either read out of the two config
files beside this one or taken from the source of the thing being modelled:

  `lidar_node.py`      a scan every 100 ms, stamped at the *start* of the
                       revolution, so 100 ms in the past. Its own comment says
                       why, and it is right to do it.
  `base_node.py`       `odom -> base_link` at the driver board's ~17 Hz, stamped
                       now.
  `slam_toolbox`       async mode. `laserCallback` sets `scan_header` from the
                       scan it has just taken *and then processes it in the same
                       callback*, so nothing else is taken while it works. A
                       separate thread republishes `map -> odom` every 50 ms
                       stamped `scan_header + transform_timeout` -- **the stamp
                       of the last scan the callback picked up, not the time it
                       was published**. `restamp_tf` would make it `now`; it
                       defaults to false and this rover does not set it.
  `controller_server`  10 Hz. Its pose comes back stamped at the newest
                       `odom -> base_link`, i.e. about now, and DWB then asks
                       tf2 for `map` at that stamp. `nav_2d_utils::transformPose`
                       catches the extrapolation, falls back to the newest
                       `map -> odom` there is, and **fails outright if that one
                       is older than `transform_tolerance`**.

Put the four together and the whole budget is one subtraction:

    scan published at T, stamped T - 0.1
    map -> odom therefore stamped  T - 0.1 + transform_timeout(0.2) = T + 0.1
    controller asks at             now, needs  now - transform_tolerance(0.3)
    so it fails once               now > T + 0.4

**0.4 seconds.** That is how long `laserCallback` may go without picking up a
scan before the next control tick aborts the goal. Not how long a scan may take
to *process* -- how long the callback may be unavailable, which is the same thing
in async mode because processing happens inside it.

## What eats it, and why it is long routes

Three things, and all three grow with the route rather than with the goal:

- **The map rebuild.** `updateMap()` takes `smapper_mutex_`, which `addScan()`
  also needs, and rebuilds the whole occupancy grid from every scan in the graph.
  It runs every `map_update_interval` -- 2 s here -- but only if something
  subscribes to `/map`, and with Nav2 up the global costmap's static layer always
  does. Its cost grows with the number of nodes in the graph, which is one per
  20 cm driven.
- **Loop closure.** The comment in slam_toolbox.yaml estimates the 8 m search at
  "near a second" on this board. Closures fire when the rover comes back within
  3 m of somewhere it has been, which is a thing short goals in one room never do.
- **The board going quiet.** slam_toolbox's scan subscription is a
  `tf2_ros::MessageFilter` on `odom`, so a scan is only handed to the callback
  once `odom -> base_link` covers its stamp. A gap in the driver board's
  telemetry therefore stops `scan_header` advancing just as effectively as a busy
  mapper -- and this is the failure `--board-stall` models.

## What this cannot tell you, and must not be read as telling you

**It does not predict when a stall happens.** The cost of a rebuild on a
particular map and the cost of a closure on a particular graph are measurements
this has not got; they are arguments, and their defaults here are zero. What it
answers is the other half: *given* a stall of some length, does navigation
survive it. That half is arithmetic over published stamps and is worth trusting.

So a run with no stall injected reports no aborts and 0.4 s of slack, which is the
point -- see "The simulation that could not fail" in the README for what happens
when a simulation cannot report that its subject is fine. The prediction to check
on the rover is the threshold, and it is checkable in one line with the stack
running and a goal in flight:

    ssh bpi-m4zero 'P=$(pgrep -f async_slam_toolbox_node); kill -STOP $P; sleep 0.6; kill -CONT $P'
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# The board's telemetry rate, measured: the ESP32 speaks at about 17 Hz and
# base_node publishes one transform per record it has not seen before. See the
# note in base_node.tick, which is where that gating lives.
BOARD_HZ = 17.5
# The lidar's real rate rather than its nominal one. README: 9.9 Hz.
SCAN_HZ = 9.9
# tf2_ros::MessageFilter's queue in slam_toolbox. Full, it drops the oldest --
# which is the "Message Filter dropping message ... queue is full" line that
# CLAUDE.md warns reads as a scan problem and is not one.
FILTER_QUEUE = 5
# What the default behaviour tree will take before it abandons a NavigateToPose:
# one retry inside the FollowPath RecoveryNode, six around the whole navigation.
# A model of the tree rather than a reading of it, and only used to say when the
# console would have printed the message this file is about.
BT_FOLLOW_FAILURES = 7
# How long the tree spends clearing a costmap and re-sending before the
# controller is ticking again. Nothing depends on it being exact.
BT_RETRY_S = 0.3

STEP = 0.001


def setting(text, key, after=None, default=None):
    """One `key: number` out of a YAML file, without a YAML parser.

    Crude in the same way selftest.py's config reader is crude, and for the same
    reason: this runs on machines that have no `yaml`. `after` names the line to
    start looking from, which is how `transform_tolerance` under `FollowPath` is
    told from the behaviour server's copy of the same name.
    """
    lines = [line for line in text.splitlines() if not line.lstrip().startswith("#")]
    start = 0
    if after is not None:
        for i, line in enumerate(lines):
            if line.strip() == after:
                start = i + 1
                break
        else:
            return default
    for line in lines[start:]:
        stripped = line.strip()
        if stripped.startswith(key + ":"):
            try:
                return float(stripped.split(":", 1)[1].split("#")[0].strip())
            except ValueError:
                return default
    return default


def config():
    """The numbers that set the budget, read from the files that are deployed.

    Read rather than restated, because the whole finding is a subtraction between
    four of them and a copy that drifted would describe a rover that does not
    exist. Defaults are the stock values, for running this outside a checkout.
    """
    out = {"transform_timeout": 0.5, "minimum_time_interval": 0.5,
           "map_update_interval": 10.0, "transform_tolerance": 0.3,
           "controller_frequency": 20.0, "scan_stamp_offset": 1.0 / 10.0}
    slam = os.path.join(HERE, "config", "slam_toolbox.yaml")
    nav = os.path.join(HERE, "config", "nav2.yaml")
    if os.path.exists(slam):
        text = open(slam).read()
        for key in ("transform_timeout", "minimum_time_interval",
                    "map_update_interval", "transform_publish_period"):
            got = setting(text, key)
            if got is not None:
                out[key] = got
    if os.path.exists(nav):
        text = open(nav).read()
        got = setting(text, "transform_tolerance", after="FollowPath:")
        if got is not None:
            out["transform_tolerance"] = got
        got = setting(text, "controller_frequency")
        if got is not None:
            out["controller_frequency"] = got
    out.setdefault("transform_publish_period", 0.05)
    return out


def run(cfg, seconds=20.0, stall=0.0, stall_at=5.0, board_stall=0.0,
        rebuild=0.0, match=0.02, closure=0.0, closure_at=None):
    """Play the four clocks forward and count what the controller sees.

    `stall` is one injected occupancy of the laser callback -- a loop closure, a
    long scan match, a rebuild that was already running. `board_stall` is the
    driver board going quiet instead, which stops the message filter releasing
    scans. `rebuild` is what one `updateMap()` costs, every map_update_interval,
    for as long as the run lasts.
    """
    scan_period = 1.0 / SCAN_HZ
    board_period = 1.0 / BOARD_HZ
    control_period = 1.0 / cfg["controller_frequency"]

    t = 0.0
    next_scan = 0.0
    next_board = 0.0
    next_publish = 0.0
    next_control = 0.0
    next_rebuild = cfg["map_update_interval"]

    queue = []                 # scan stamps waiting on the message filter
    dropped = 0
    scan_header = None         # what slam_toolbox last picked up
    last_processed = -1e9
    odom_stamp = None
    map_odom = None            # the newest map -> odom stamp there is
    cb_free = 0.0              # when laserCallback is next able to take a scan
    mapper_free = 0.0          # smapper_mutex_
    rebuild_due = False
    injected = False
    closure_done = closure_at is None

    aborts = []
    slack_min = None
    slack_at = None
    controller_back = 0.0
    lost_at = None

    while t < seconds:
        # --- lidar_node: a revolution, stamped where it started ---------------
        if t >= next_scan:
            queue.append(t - cfg["scan_stamp_offset"])
            if len(queue) > FILTER_QUEUE:
                del queue[0]
                dropped += 1
            next_scan += scan_period

        # --- base_node: one transform per board record ------------------------
        if t >= next_board:
            if not (board_stall and stall_at <= t < stall_at + board_stall):
                odom_stamp = t
            next_board += board_period

        # --- the map rebuild thread, which wants the mapper -------------------
        if t >= next_rebuild:
            rebuild_due = True
            next_rebuild += cfg["map_update_interval"]
        if rebuild_due and rebuild > 0.0 and t >= mapper_free:
            mapper_free = t + rebuild
            rebuild_due = False

        # --- slam_toolbox: laserCallback, which is also the processing --------
        if t >= cb_free and t >= mapper_free and queue:
            # The message filter only releases a scan once odom covers its stamp.
            if odom_stamp is not None and odom_stamp >= queue[0]:
                stamp = queue.pop(0)
                scan_header = stamp            # the first line of laserCallback
                cost = 0.001
                if stamp - last_processed >= cfg["minimum_time_interval"]:
                    last_processed = stamp
                    cost = match
                    if not closure_done and t >= closure_at:
                        cost += closure
                        closure_done = True
                    if stall and not injected and t >= stall_at:
                        cost += stall
                        injected = True
                    mapper_free = t + cost     # Process() holds smapper_mutex_
                cb_free = t + cost

        # --- the transform thread: the same stamp, republished ----------------
        if t >= next_publish:
            if scan_header is not None:
                map_odom = scan_header + cfg["transform_timeout"]
            next_publish += cfg["transform_publish_period"]

        # --- controller_server: one tick of DWB -------------------------------
        if t >= next_control:
            next_control += control_period
            if odom_stamp is not None and map_odom is not None and t >= controller_back:
                # nav_2d_utils::transformPose, with the extrapolation fallback.
                slack = map_odom - (odom_stamp - cfg["transform_tolerance"])
                if slack_min is None or slack < slack_min:
                    slack_min, slack_at = slack, t
                if slack < 0.0:
                    aborts.append(round(t, 3))
                    controller_back = t + BT_RETRY_S
                    if len(aborts) >= BT_FOLLOW_FAILURES and lost_at is None:
                        lost_at = t
        t += STEP

    return {"aborts": aborts, "slack_min": slack_min, "slack_at": slack_at,
            "dropped": dropped, "lost_at": lost_at}


def describe(cfg):
    budget = (cfg["transform_timeout"] + cfg["transform_tolerance"]
              - cfg["scan_stamp_offset"])
    print("the budget, from the deployed configuration")
    print("  scan stamped %.0f ms before it is published   (lidar_node.py)"
          % (cfg["scan_stamp_offset"] * 1000))
    print("  transform_timeout      %.2f s                 (slam_toolbox.yaml)"
          % cfg["transform_timeout"])
    print("  transform_tolerance    %.2f s                 (nav2.yaml, FollowPath)"
          % cfg["transform_tolerance"])
    print("  controller             %.0f Hz" % cfg["controller_frequency"])
    print("  map rebuild every      %.1f s, holding the mapper"
          % cfg["map_update_interval"])
    print("  ->  laserCallback may go %.2f s without taking a scan. Past that,"
          % budget)
    print("      the next control tick throws ControllerTFError and the goal")
    print("      comes back as code 102, which nav_codes.py reads as \"lost\".")
    return budget


def sweep(cfg, args):
    print("\nhow long a stall has to be, injected once into laserCallback")
    print("  stall    aborts   first at   worst slack")
    threshold = None
    step = 0.05
    stall = step
    while stall <= 1.6001:
        got = run(cfg, seconds=12.0, stall=stall, stall_at=4.0,
                  rebuild=args.rebuild, match=args.match)
        first = got["aborts"][0] - 4.0 if got["aborts"] else None
        print("  %5.2f s  %6d   %8s   %+.3f s"
              % (stall, len(got["aborts"]),
                 "--" if first is None else "%.2f s" % first,
                 got["slack_min"]))
        if got["aborts"] and threshold is None:
            threshold = stall
        stall += step
    if threshold is not None:
        print("  -> the first stall length that aborts a goal is %.2f s"
              % threshold)
    return threshold


def main():
    cfg = config()
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--seconds", type=float, default=20.0,
                   help="how long a run is")
    p.add_argument("--stall", type=float, default=0.0,
                   help="one injected occupancy of laserCallback, seconds")
    p.add_argument("--board-stall", type=float, default=0.0,
                   help="the driver board going quiet instead, seconds")
    p.add_argument("--at", type=float, default=5.0, dest="stall_at",
                   help="when to inject it")
    p.add_argument("--rebuild", type=float, default=0.0,
                   help="what one updateMap() costs on this map. MEASURE IT; "
                        "the default is zero because this file does not know")
    p.add_argument("--closure", type=float, default=0.0,
                   help="what one loop closure costs. Same warning")
    p.add_argument("--closure-at", type=float, default=None,
                   help="when the closure fires")
    p.add_argument("--match", type=float, default=0.02,
                   help="an ordinary scan match, seconds")
    p.add_argument("--sweep", action="store_true",
                   help="find the stall length that first aborts a goal")
    args = p.parse_args()

    budget = describe(cfg)
    if args.sweep:
        threshold = sweep(cfg, args)
        return 0 if threshold and abs(threshold - budget) < 0.1 else 1

    got = run(cfg, seconds=args.seconds, stall=args.stall,
              stall_at=args.stall_at, board_stall=args.board_stall,
              rebuild=args.rebuild, match=args.match, closure=args.closure,
              closure_at=args.closure_at)
    print("\n%.0f s of driving, %s"
          % (args.seconds,
             ", ".join(filter(None, [
                 "a %.2f s stall at %.0f s" % (args.stall, args.stall_at)
                 if args.stall else None,
                 "the board quiet for %.2f s at %.0f s"
                 % (args.board_stall, args.stall_at) if args.board_stall else None,
                 "a %.2f s rebuild every %.1f s"
                 % (args.rebuild, cfg["map_update_interval"]) if args.rebuild else None,
                 "a %.2f s closure at %.0f s" % (args.closure, args.closure_at or 0)
                 if args.closure else None])) or "nothing injected"))
    print("  worst slack        %+.3f s at %.1f s"
          % (got["slack_min"], got["slack_at"]))
    print("  controller aborts  %d%s"
          % (len(got["aborts"]),
             "" if not got["aborts"] else "  (first at %.2f s)" % got["aborts"][0]))
    print("  scans dropped by the message filter  %d" % got["dropped"])
    if got["lost_at"] is not None:
        print("  -> the tree ran out of retries at %.1f s: the console says"
              % got["lost_at"])
        print("     \"lost -- Nav2 gave up without saying why (code 102)\"")
    elif got["aborts"]:
        print("  -> FollowPath aborted and the tree recovered; the rover stops,")
        print("     clears a costmap and carries on. A person sees a stutter.")
    else:
        print("  -> nothing aborted. This is what a healthy stack looks like,")
        print("     and it is the run this simulation has to be able to report.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
