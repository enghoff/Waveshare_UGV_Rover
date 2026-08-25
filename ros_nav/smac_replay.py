#!/usr/bin/env python3
"""Whether a state-lattice planner would have drawn the doorway corners.

    python3 smac_replay.py                  # the synthetic metre-wide 55 deg bend
    python3 smac_replay.py episode.json     # the rover's own recorded plans
    python3 smac_replay.py episode.json --dwb

**Why this exists.** docs/doorway-pivot.md ends on an open question: NavFn
draws corners this chassis cannot follow while driving, SimpleSmoother cannot
see curvature, and SmacPlannerLattice would. That last claim is not allowed to
ship on the strength of the plugin's name. This scores both searches on the
same costmap -- a synthetic doorway matching the recorded 44-67 deg in 1.2 m,
or a recording's own global costmap and plans -- and reports whether the
lattice path stays inside one DWB rollout.

It does not close a loop on a frozen map. That test condemned whatever
look-ahead the rover happened to be running, and the first doorway fix
shipped on it. The number here is the path's own tightest heading change
over 0.32 m of *forward* travel, which is how long DWB commits to an arc.
In-place rotations in the control set are a different manoeuvre and are
cut out of that window on purpose.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import goal_fit
import hybrid_astar as geo
import lattice


def load(path):
    with open(path) as handle:
        return json.load(handle)


def grid_of(snapshot):
    raw = base64.b64decode(snapshot["data"])
    return goal_fit.CostGrid(snapshot["width"], snapshot["height"],
                             snapshot["resolution"], snapshot["origin"][0],
                             snapshot["origin"][1], list(raw))


def start_goal_of(plan):
    """A lattice query from a recorded NavFn plan: first pose to last pose."""
    poses = plan.get("poses") or []
    if len(poses) < 2:
        return None
    x0, y0 = poses[0][0], poses[0][1]
    x1, y1 = poses[1][0], poses[1][1]
    xn, yn = poses[-1][0], poses[-1][1]
    start_yaw = math.atan2(y1 - y0, x1 - x0)
    if len(poses[-1]) > 2:
        goal_yaw = poses[-1][2]
    elif len(poses) >= 2:
        a, b = poses[-2], poses[-1]
        goal_yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    else:
        goal_yaw = start_yaw
    if len(poses[0]) > 2:
        start_yaw = poses[0][2]
    return (x0, y0, start_yaw), (xn, yn, goal_yaw)


def sharpest_plans(episode, count=4):
    """The recorded plans with the tightest 0.32 m window, unique by shape."""
    ranked = []
    seen = set()
    for plan in episode.get("plans") or []:
        pts = [(p[0], p[1]) for p in plan.get("poses") or []]
        if len(pts) < 3:
            continue
        key = tuple((round(x, 2), round(y, 2)) for x, y in (pts[0], pts[len(pts)//2], pts[-1]))
        if key in seen:
            continue
        seen.add(key)
        _at, bend = geo.tightest_window(pts)
        ranked.append((bend, plan, pts))
    ranked.sort(reverse=True, key=lambda item: item[0])
    return ranked[:count]


def report_path(info, indent="   "):
    extra = ""
    if "pivots" in info:
        extra = ", %d in-place turn%s" % (info["pivots"],
                                          "" if info["pivots"] == 1 else "s")
    print("%s%s: %.2f m, first 1.2 m bends %.0f deg, tightest %.1f deg "
          "over 0.32 m at s=%.2f%s  %s"
          % (indent, info["label"], info["length_m"], info["first_bend_deg"],
             info["tightest_deg"], info["tightest_at_m"], extra,
             "followable" if info["followable"] else "TOO TIGHT"))


def synthetic(dwb=False):
    print("synthetic metre-wide passage, 55 deg bend")
    print("   DWB's forward envelope is %.2f m radius, %.1f deg in one 0.32 m "
          "rollout; the lattice control set is %.2f m"
          % (geo.MIN_TURNING_RADIUS, math.degrees(geo.ROLLOUT_RAD),
             lattice.load_lattice()[0]["turning_radius"]))
    result = lattice.doorway_reproduction(dwb=dwb)
    if result.get("navfn") is None:
        print("   NavFn-like search found no route")
        return 1
    if result.get("lattice") is None:
        print("   lattice search found no route")
        return 1
    report_path(result["navfn"])
    report_path(result["lattice"])
    if dwb and result.get("navfn_mid") and result.get("lattice_mid"):
        print("   DWB along each path, arrival circle excluded:")
        for name in ("navfn_mid", "lattice_mid"):
            row = result[name]
            print("      %-11s %d ticks, %d forward, %d pivot, %d no-forward, "
                  "longest stall %.2f m"
                  % (name.replace("_mid", ""), row["ticks"], row["forward"],
                     row["pivot"], row["no_forward"], row["longest_stall_m"]))
    if not result["navfn"]["followable"] and result["lattice"]["followable"]:
        print("   the lattice draws a corner DWB can follow; the grid search does not.")
        return 0
    print("   the reproduction did not separate the two planners")
    return 1


def recorded(path, dwb=False):
    episode = load(path)
    grid = None
    if episode.get("global_costmap"):
        grid = grid_of(episode["global_costmap"])
        print("recorded global costmap %dx%d at %.2f m"
              % (grid.width, grid.height, grid.resolution))
    else:
        print("no global costmap in the recording, so only the NavFn plans "
              "themselves can be measured")
    plans = sharpest_plans(episode)
    if not plans:
        print("no recorded plans long enough to have a corner")
        return 1
    failed = 0
    for bend, plan, pts in plans:
        print("")
        print("recorded plan at t=%.1fs, %d poses"
              % (plan.get("t", 0.0), len(pts)))
        info = geo.describe_path(pts, "navfn (recorded)")
        report_path(info)
        if grid is None:
            continue
        sg = start_goal_of(plan)
        if sg is None:
            continue
        start, goal = sg
        found = lattice.lattice_astar(grid, start, goal)
        if found is None:
            print("   lattice search found no route from this plan's start to its end")
            failed += 1
            continue
        hinfo = lattice.describe_path(found, "lattice")
        report_path(hinfo)
        if not hinfo["followable"]:
            failed += 1
        if dwb:
            nmid = geo.midcourse_stats(grid, pts)
            hmid = geo.midcourse_stats(grid, found)
            print("   DWB along each path, arrival circle excluded:")
            print("      recorded   %d forward, %d pivot, %d no-forward, "
                  "stall %.2f m" % (nmid["forward"], nmid["pivot"],
                                    nmid["no_forward"], nmid["longest_stall_m"]))
            print("      lattice    %d forward, %d pivot, %d no-forward, "
                  "stall %.2f m" % (hmid["forward"], hmid["pivot"],
                                    hmid["no_forward"], hmid["longest_stall_m"]))
    if failed:
        print("")
        print("lattice search did not produce a followable path for every recorded corner")
        return 1
    print("")
    print("every recorded corner the lattice re-planned stayed inside one DWB rollout")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("episode", nargs="?", default="",
                        help="a nav_record.py JSON; omitted, the synthetic doorway")
    parser.add_argument("--dwb", action="store_true",
                        help="also score DWB along each path (slow, and not the "
                             "test -- the path geometry is)")
    args = parser.parse_args()
    if args.episode:
        return recorded(args.episode, args.dwb)
    return synthetic(args.dwb)


if __name__ == "__main__":
    sys.exit(main())
