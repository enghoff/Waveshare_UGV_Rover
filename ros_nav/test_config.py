"""The configuration the rover actually runs, and the codes it reports back.

Nav2 is configured in YAML that no test can typecheck, so what is checked is
that the settings agree with each other and with the chassis limits, and that
every error code the daemon can be handed has a phrase for a person.
"""
import json
import math
import os

from test_harness import HERE, check, section
from nav_types import MAX_SPEED_MS, MAX_TURN_DPS
from nav_codes import PHRASES, REASONS, phrase_for, reason_for


# --- the configuration files --------------------------------------------------
def _costmap_sections(text):
    """nav2.yaml split into the global costmap's block and the local one's.

    Crude, and deliberately so: this file cannot import yaml on every machine it
    runs on, and what the checks below ask is only whether a plugin name appears
    on one side of the file or the other. Returns (global, local) as text.
    """
    lines = text.splitlines(True)
    where, out = None, {"global_costmap:": [], "local_costmap:": []}
    for line in lines:
        if line.rstrip() in out:
            where = line.rstrip()
        elif line and not line[0].isspace() and not line.startswith("#"):
            where = None
        if where:
            out[where].append(line)
    return "".join(out["global_costmap:"]), "".join(out["local_costmap:"])


def settings_of(text):
    """Just the settings out of a YAML file, with the comments dropped.

    Everything below is checked by looking for a string, and both of the names
    that matter -- the critic that was replaced and the shim that was removed --
    go on appearing in the comments that explain why they are not there. A search
    over the whole file finds the explanation and calls it the setting.
    """
    keep = [line for line in text.splitlines()
            if line.strip() and not line.lstrip().startswith("#")]
    return chr(10).join(keep)


def test_configs_agree():
    """The three places a speed limit is written must say the same thing.

    Nav2's YAML cannot import from nav_types.py, so the numbers are copied -- and
    a copy that silently diverges is a rover whose controller commands more than
    the base will deliver, which shows up as a path it never quite follows.
    """
    section("configuration agrees with the measured chassis")
    path = os.path.join(HERE, "config", "nav2.yaml")
    if not os.path.exists(path):
        print("  .... skipped, no config/nav2.yaml")
        return
    with open(path) as fh:
        text = fh.read()
    # Deliberately *not* MAX_SPEED_MS. That constant is 0.35 m/s and this chassis
    # was measured at 0.33 at its slowest usable PWM, so putting it here pins
    # every command to the bottom of the range. What must hold is that Nav2's
    # limit lies inside the speeds actually measured, which is checked against
    # the store below where the store exists.
    check("Nav2's top speed is not the stale MAX_SPEED_MS",
          ("max_vel_x: %.2f" % MAX_SPEED_MS) in text, False)
    check("...and is a speed this chassis can actually reach",
          "max_vel_x: 0.40" in text, True)
    # A heading change tighter than an arc can absorb has to be a turn on the
    # spot, and a bare velocity controller will only ever approximate one.
    # The shim was tried and taken out again -- it cannot transform on a rover
    # whose transform tree runs at the driver board's 17 Hz, and it cost the
    # control loop a third of its rate. The comment in nav2.yaml has the whole
    # story; this keeps somebody from re-adding it without reading it.
    check("no rotation shim, which this rover's transform rate cannot support",
          "RotationShimController" in settings_of(text), False)
    # The footprint and the critic that reads it have to change together. A
    # rectangle with a 0.14 m inscribed radius under a critic that only tests the
    # centre cell is a rover free to swing its corners into the furniture.
    # Settings only. Both names go on appearing in the comments beside them,
    # which is where the reasoning lives, so a search over the whole file finds
    # the explanation and calls it the setting.
    settings = settings_of(text)
    # **A circle, because this rover pivots.** nav2 clears the robot at the
    # inscribed radius and the robot sweeps the circumscribed one, so any
    # non-circular body has a band round every wall it may legally stand in and
    # not turn out of. The rectangle that was here had a 0.14 m ring and 0.21 m
    # corners, and the rover was found wedged in exactly those ten centimetres.
    # A radius makes the two one number. 0.20 m is the chassis measured with a
    # tape; the old footprint was `slam2d.c`'s lidar self-return mask plus 5 cm
    # of margin, with an unmeasured guess forwards. See config/nav2.yaml.
    check("the body is a circle, so standing somewhere implies turning there",
          "robot_radius: 0.200" in settings, True)
    check("...in both costmaps, because two shapes is a planner that routes "
          "through gaps the controller will not drive",
          settings.count("robot_radius: 0.200") == 2, True)
    check("...and the rectangle that could not turn where it stood is gone",
          'footprint: "[[0.20' in settings, False)
    # The point test is a collision test again, and only because of the above.
    check("the obstacle critic is the point test a circular body takes",
          "BaseObstacle" in settings and "ObstacleFootprint" not in settings, True)
    # The arrival circle has to be bigger than the smallest move the rover has.
    # One forward sample at 0.40 m/s over a 0.8 s rollout is 32 cm, so a 15 cm
    # circle was a target DWB could not aim at: it sat 23 cm from a goal for
    # 25 s and timed out, because everything that closed the gap overshot it.
    # The two copies are the goal checker's and the controller's, and they have
    # to be the same number or RotateToGoal switches at a different radius from
    # the one that ends the goal.
    check("the arrival circle clears the chassis's 32 cm minimum move",
          settings.count("xy_goal_tolerance: 0.22") == 2, True)
    # The planner has to know the turning radius DWB can follow while driving.
    # NavFn does not, and that is the doorway lock-up: a 45 deg kink in one
    # rollout, every forward sample leaves the line, the rover pivots. The
    # plugin name GridBased is unchanged so the behaviour tree does not have
    # to move; the class behind it does. Lattice, not Hybrid: this chassis
    # can pivot, and Dubins Hybrid-A* cannot write that into a path.
    check("the planner is the state lattice, which is the one that knows a turning radius",
          "nav2_smac_planner::SmacPlannerLattice" in settings, True)
    check("NavFn is not the configured planner",
          "nav2_navfn_planner::NavfnPlanner" in settings, False)
    check("Hybrid-A* is not the configured planner either",
          "nav2_smac_planner::SmacPlannerHybrid" in settings, False)
    # **The align look-ahead, which is a controller setting but belongs next to
    # the planner ones because it decides whether the plan gets followed.**
    # 0.325 is nav2's own default and these two checks only hold it there.
    #
    # **The reason recorded for it has since been withdrawn**, so read them as
    # "this is the default and nothing has argued it away" rather than as a
    # measured choice. The argument was that an align critic charges
    # `unreachable_score_` -- 2881 points once scaled -- for a nose point the
    # flood could not reach, and that at 0.8 m this landed on a median 26% of
    # the candidates in a tick. The flood in the installed `libdwb_critics.so`
    # is not stopped by walls at all, so no candidate is ever charged it: 0 of
    # 8687 driving candidates and 0 of 6132 pivots over
    # recordings/trap-2026-08-25-spin.json. See corridor_sim.flood and
    # trap_sim.py --bias, and make a fresh case from a drive before moving it.
    #
    # `PreferForward` is the only critic in the set that prices turning as
    # such, and it was added on the same withdrawn measurement. It is left in
    # because the rover does still choose to turn far more often than to drive
    # -- on the recorded trap, on 481 of 496 ticks the model could score.
    check("something in the critic set prices turning, or the rover will spin",
          "PreferForward" in settings, True)
    check("the align look-ahead is back at nav2's default, not the 0.8 that trapped it",
          "PathAlign.forward_point_distance: 0.325" in settings
          and "GoalAlign.forward_point_distance: 0.325" in settings, True)
    check("...and neither align critic is left at 0.8",
          "forward_point_distance: 0.8" in settings, False)
    check("in-place turns are expensive, so a doorway that takes an arc gets one",
          "rotation_penalty: 5.0" in settings, True)
    # **The budget, and it is the one that was failing long goals.** A route
    # of eight to twelve metres across a mapped house costs this board one to
    # two and a half seconds, so a 2 s budget cut a large share of them off
    # mid-search -- and Nav2 reports that as `NoValidPathCouldBeFound`, which
    # reaches the operator as "there is no route to there". Measured with
    # plan_bench.py: one query at one start heading, ten times, planned 4 and
    # refused 6 at 2 s with every success landing at 2.01-2.09 s; at 3 s the
    # whole sixteen-heading sweep planned, none of it needing over 2.27 s.
    check("the planner has 4 s, because a house-sized route costs this board 2 to 3",
          "max_planning_time: 4.0" in settings, True)
    check("...and reverse expansion is off, because the lidar looks forwards",
          "allow_reverse_expansion: false" in settings, True)
    check("...and the lattice may enter unknown, because this rover maps as it drives",
          "allow_unknown: true" in settings, True)
    lattice_json = os.path.join(HERE, "config", "lattices", "diff_5cm_0.5m.json")
    check("the differential control set is in the tree, not left on a share path",
          os.path.isfile(lattice_json), True)
    if os.path.isfile(lattice_json):
        with open(lattice_json) as handle:
            meta = json.load(handle)["lattice_metadata"]
        check("...and is the 0.5 m differential sample, DWB's envelope to a centimetre",
              meta.get("motion_model") == "diff" and abs(meta.get("turning_radius", 0) - 0.5) < 1e-9,
              True)
    with open(os.path.join(HERE, "nav.launch.py")) as handle:
        launch = handle.read()
    check("launch injects an absolute lattice path; yaml cannot resolve a relative one",
          "lattice_filepath" in launch and "diff_5cm_0.5m.json" in launch, True)
    with open(os.path.join(HERE, "slam.launch.py")) as handle:
        slam_launch = handle.read()
    check("the python nodes are started by the interpreter, so a 644 checkout still runs",
          "sys.executable" in slam_launch
          and 'os.path.join(HERE, "lidar_node.py")' in slam_launch, True)

    # The lidar looks forwards, so a reverse leg is driven blind. DWB is left
    # with no reverse sample at all; backing out of a corner is the behaviour
    # server's `backup`, which the behaviour tree bounds to 30 cm.
    check("the controller has no reverse, since the rover cannot see behind it",
          "min_vel_x: 0.0" in settings and "min_vel_x: -0.40" not in settings,
          True)
    check("...but the smoother still passes one, or the recovery cannot back up",
          "min_velocity: [-0.40, 0.0, -0.78]" in settings, True)
    turn = math.radians(MAX_TURN_DPS)
    check("Nav2's turn limit matches MAX_TURN_DPS (%.2f rad/s)" % turn,
          abs(turn - 0.78) < 0.01, True)
    check("...and that is what the file says", "max_vel_theta: 0.78" in text, True)
    # Both floors have to move. Nav2's isValidSpeed is an AND: a theta floor
    # with min_speed_xy left at 0 never drops a sample. 0.21 rad/s is the
    # mixer's 12 deg/s; 0.1 m/s is below the only forward sample, so driving
    # is untouched.
    check("DWB will not sample a standing turn slower than the mixer can hold",
          "min_speed_theta: 0.21" in settings, True)
    check("...and min_speed_xy is not zero, or that theta floor is a no-op",
          "min_speed_xy: 0.1" in settings, True)
    check("...and the old zero floors are gone",
          "min_speed_xy: 0.0" in settings or "min_speed_theta: 0.0" in settings,
          False)

    # The two costmap rules that a rover running SLAM cannot break. Both were
    # broken at once, and between them they closed 61% of the mapped floor to the
    # planner, which is what a route four times longer than it needed to be is
    # made of. See the comments beside each of them in config/nav2.yaml.
    globals_, locals_ = _costmap_sections(text)
    check("the global costmap has no obstacle layer, which SLAM would ghost",
          "obstacle_layer" in globals_, False)
    check("...and still has the static layer, or it has nothing to plan on",
          "static_layer" in globals_, True)
    check("the local costmap does have one, since something must see a chair",
          "obstacle_layer" in locals_, True)
    check("...and clears the bearings that got nothing back",
          "inf_is_valid: true" in locals_, True)

    slam = os.path.join(HERE, "config", "slam_toolbox.yaml")
    if os.path.exists(slam):
        with open(slam) as fh:
            slam_text = fh.read()
        check("slam_toolbox and Nav2 agree the map resolution is 5 cm",
              "resolution: 0.05" in slam_text and "resolution: 0.05" in text, True)
        check("slam_toolbox is told the lidar's real reach",
              "max_laser_range: 8.0" in slam_text, True)
        check("mapping is on, or there is no map to navigate on",
              "mode: mapping" in slam_text, True)
        check("loop closing is on, which is the whole reason for this stack",
              "do_loop_closing: true" in slam_text, True)


def test_nav2_error_codes():
    """Nav2's result codes, as the words the daemon's `Outcome` already uses.

    The whole reason `nav_codes.py` lists every code instead of doing arithmetic
    on it: the numbers look systematic and are not. BackUp's 713 is invalid input
    and its 714 is a collision; DriveOnHeading's 723 is a collision and its 724 is
    invalid input -- the same two meanings, swapped, in adjacent blocks. A version
    of this that matched on the last digit passed every test written for it and
    reported a rover stopped by a wall as one that had timed out.
    """
    section("Nav2 result codes read as English")
    check("zero is an arrival", reason_for(0), "arrived")
    check("701, Spin timing out", reason_for(701), "timed out")
    check("703, Spin into something", reason_for(703), "blocked")
    check("713, BackUp given a nonsense distance", reason_for(713), "refused")
    check("714, BackUp into something", reason_for(714), "blocked")
    check("723, DriveOnHeading into something -- note it is not 724",
          reason_for(723), "blocked")
    check("724, DriveOnHeading given a nonsense distance",
          reason_for(724), "refused")
    check("702, a transform failure, which is being lost",
          reason_for(702), "lost")
    check("208, the planner finding no route, is being blocked",
          reason_for(208), "blocked")
    check("206, a goal with something in it, is a refusal",
          reason_for(206), "refused")
    check("105, the controller stuck, is being blocked",
          reason_for(105), "blocked")
    # 700 is Spin's UNKNOWN, and it caught the last-digit version red-handed:
    # 700 % 10 is 0, so a behaviour that failed for a reason it could not name was
    # reported as having arrived. The rover would have said it had turned.
    check("700 -- plain unknown -- is a failure and not an arrival",
          reason_for(700), "failed")
    check("a code nobody has heard of falls back rather than raising",
          reason_for(795), "failed")
    check("...and specifically not to an arrival, so a Nav2 upgrade that adds a "
          "failure does not have it read as a success",
          reason_for(795) == "arrived", False)

    # Every reason has to be one the daemon's callers understand, because
    # `_tool_drive` decides `ok` by testing the word.
    known = {"arrived", "blocked", "timed out", "lost", "refused", "failed"}
    check("every code maps to a word Outcome's readers know",
          sorted(set(REASONS.values()) - known), [])
    check("every phrase belongs to a code that exists",
          sorted(set(PHRASES) - set(REASONS)), [])
    check("Nav2's own words win over ours when it gives any",
          phrase_for(723, "the local costmap says no"),
          "the local costmap says no")
    check("...and ours are there for when it does not",
          phrase_for(723, "  ") != "", True)
    check("a code with neither says nothing rather than something made up",
          phrase_for(700, ""), "")


def test_nav2_error_codes_match_the_installed_nav2():
    """On the rover, check the numbers against the .action files themselves.

    The table was copied by hand out of `share/nav2_msgs/action/`, and a Nav2
    upgrade that renumbered anything would leave it quietly describing the
    previous version. Skipped where there is no ROS, which is most machines.
    """
    section("the code table matches the Nav2 that is installed")
    import glob
    import re as _re

    roots = glob.glob(os.path.expanduser(
        "~/miniforge3/envs/*/share/nav2_msgs/action"))
    if not roots:
        print("  .... skipped, no nav2_msgs on this machine")
        return
    wanted = {"UNKNOWN": "failed", "TIMEOUT": "timed out", "TF_ERROR": "lost",
              "COLLISION_AHEAD": "blocked", "INVALID_INPUT": "refused",
              "NO_VALID_PATH": "blocked", "GOAL_OCCUPIED": "refused",
              "START_OCCUPIED": "blocked", "GOAL_OUTSIDE_MAP": "refused",
              "START_OUTSIDE_MAP": "lost", "FAILED_TO_MAKE_PROGRESS": "blocked",
              "NO_VALID_CONTROL": "blocked", "PATIENCE_EXCEEDED": "blocked",
              "CONTROLLER_TIMED_OUT": "timed out",
              "INVALID_CONTROLLER": "refused", "INVALID_PLANNER": "refused",
              "INVALID_PATH": "refused"}
    interesting = ("Spin", "BackUp", "DriveOnHeading", "FollowPath",
                   "ComputePathToPose")
    for name in interesting:
        path = os.path.join(roots[0], "%s.action" % name)
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8", errors="replace") as fh:
            declared = _re.findall(r"^uint16 ([A-Z_]+)=(\d+)",
                                   fh.read(), _re.MULTILINE)
        for label, number in declared:
            if label == "NONE":
                continue
            code = int(number)
            if label not in wanted:
                check("%s.%s (%d) is a meaning this table has an opinion about"
                      % (name, label, code), label, "one of %s" % sorted(wanted))
                continue
            check("%s.%s is %d and reads as '%s'" % (name, label, code,
                                                     wanted[label]),
                  reason_for(code), wanted[label])


TESTS = (
    test_configs_agree,
    test_nav2_error_codes,
    test_nav2_error_codes_match_the_installed_nav2,
)
