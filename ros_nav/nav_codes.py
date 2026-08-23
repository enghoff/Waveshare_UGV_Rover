#!/usr/bin/env python3
"""Nav2's result codes, as the words this rover's tools already use.

Its own module, with nothing imported into it, for two reasons. The bridge needs
it and cannot be tested without rclpy, and `selftest.py` needs it and runs on a
workstation with no ROS at all -- so a copy in the test would be a copy that
drifts, and a table that has drifted is the quietest kind of wrong: a rover that
stopped because a wall was in front of it reports "timed out", and whoever reads
that goes looking for a slow planner.

**The codes are not systematic and must not be treated as though they were.**
They look like per-action blocks around one set of meanings, and the first version
of this exploited that by matching on the last digit. It is not true:

    BackUp          713 INVALID_INPUT     714 COLLISION_AHEAD
    DriveOnHeading  723 COLLISION_AHEAD   724 INVALID_INPUT

-- the same two meanings, in the opposite order, in two adjacent blocks. So every
code is listed.

Three families reach the daemon. The 700s are the behaviour server, which is what
`drive` and `turn_in_place` are. The 100s are the controller and the 200s the
planner, and those arrive through `NavigateToPose`: its own action declares only
`NONE`, because `bt_navigator` passes on whichever underlying server failed.
Copied from `share/nav2_msgs/action/*.action` in the installed ROS 2 Jazzy, and
the selftest checks the numbers against that file when it is run on the rover.
"""

# The words are `Outcome`'s vocabulary, which the drive console and the voice
# console both read by name:
#
#   arrived    it did the thing
#   blocked    something is in the way, or there is no way through
#   timed out  it ran out of the time it was given
#   lost       the transform tree could not say where the rover is
#   refused    the request itself was not acceptable
#   failed     something went wrong that Nav2 did not name
#
# `arrived` and `timed out` are the two the daemon reports as `ok`, because both
# mean the rover moved and neither means it hit anything.
REASONS = {
    0: "arrived",

    # --- the controller, through NavigateToPose
    100: "failed",        # UNKNOWN
    101: "refused",       # INVALID_CONTROLLER
    102: "lost",          # TF_ERROR
    103: "refused",       # INVALID_PATH
    104: "blocked",       # PATIENCE_EXCEEDED -- it waited for something to move
    105: "blocked",       # FAILED_TO_MAKE_PROGRESS
    106: "blocked",       # NO_VALID_CONTROL
    107: "timed out",     # CONTROLLER_TIMED_OUT

    # --- the planner, through NavigateToPose
    200: "failed",        # UNKNOWN
    201: "refused",       # INVALID_PLANNER
    202: "lost",          # TF_ERROR
    203: "lost",          # START_OUTSIDE_MAP -- we are somewhere unmapped
    204: "refused",       # GOAL_OUTSIDE_MAP
    205: "blocked",       # START_OCCUPIED
    206: "refused",       # GOAL_OCCUPIED
    207: "timed out",     # TIMEOUT
    208: "blocked",       # NO_VALID_PATH

    # --- Spin, which is turn_in_place
    700: "failed",        # UNKNOWN
    701: "timed out",     # TIMEOUT
    702: "lost",          # TF_ERROR
    703: "blocked",       # COLLISION_AHEAD

    # --- BackUp, which is drive with a negative distance
    710: "failed",        # UNKNOWN
    711: "timed out",     # TIMEOUT
    712: "lost",          # TF_ERROR
    713: "refused",       # INVALID_INPUT
    714: "blocked",       # COLLISION_AHEAD

    # --- DriveOnHeading, which is drive
    720: "failed",        # UNKNOWN
    721: "timed out",     # TIMEOUT
    722: "lost",          # TF_ERROR
    723: "blocked",       # COLLISION_AHEAD
    724: "refused",       # INVALID_INPUT
}

# What to say when Nav2 does not. Its `error_msg` field is often empty, and a
# reason with no explanation behind it is the difference between a console that
# tells somebody what to do next and one that says "blocked" and stops.
PHRASES = {
    104: "the controller waited for something in the way to move and it did not",
    105: "the rover stopped making progress along the route",
    106: "there was no safe way to keep following the route",
    203: "the rover is standing somewhere the map does not cover yet",
    204: "that place is off the edge of the map",
    205: "the rover is standing inside something the costmap believes in -- turn "
         "on the spot or back up, then try again",
    206: "the place asked for has something in it",
    208: "there is no route to there that the rover fits through",
    703: "turning that way would sweep through something",
    714: "there is something behind the rover",
    723: "there is something in the way",
    713: "that is not a distance this can drive",
    724: "that is not a distance this can drive",
}


def reason_for(code, default="failed"):
    """One of `Outcome`'s reasons, for a Nav2 result code.

    An unknown code falls back rather than raising, and deliberately not to
    "arrived": a version of Nav2 that has added a failure this table has not heard
    of must not have that failure read as a success.
    """
    if not code:
        return "arrived"
    return REASONS.get(int(code), default)


def phrase_for(code, message=""):
    """What to tell the caller. Nav2's own words first, then ours, then neither."""
    message = (message or "").strip()
    if message:
        return message
    return PHRASES.get(int(code or 0), "")
