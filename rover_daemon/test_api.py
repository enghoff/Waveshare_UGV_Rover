"""Tool-surface checks: every schema the model is shown must be runnable.

A tool whose name does not match its handler fails as "no such tool" out loud,
in the middle of a conversation, which is a poor place to find out. So the
schemas are checked against the handlers, and the handlers against a rover with
no hardware at all.
"""
from __future__ import annotations

import json
import os

from test_fakes import FakeLink, HERE
from test_harness import check

def test_schemas():
    import rover_daemon

    # Every schema this rover could ever offer, whatever it is configured with.
    # `look` is conditional -- see test_look -- and so are the driving tools and the
    # map, which need a lidar; but a conditional schema still has to have a handler,
    # and the point of this test is that none of them lie.
    every = (rover_daemon.TOOLS + [rover_daemon.LOOK_TOOL]
             + rover_daemon.NAV_TOOLS
             + [rover_daemon.MAP_TOOL, rover_daemon.MAP_POINT_TOOL]
             + [rover_daemon.SCRIPT_TOOL, rover_daemon.START_SCRIPT_TOOL,
                rover_daemon.STOP_SCRIPT_TOOL])
    # The schemas cross a network and go into a prompt, so they have to be JSON.
    json.dumps(every)
    names = [t["function"]["name"] for t in every]
    check("every schema is a function", {t["type"] for t in every}, {"function"})
    check("no tool is listed twice", len(set(names)), len(names))
    # The check this file exists for: a schema whose handler is missing fails as
    # "no such tool" mid-conversation rather than here.
    missing = [n for n in names if not hasattr(rover_daemon.Rover, f"_tool_{n}")]
    check("every schema has a handler", missing, [])
    # ...and the reverse, so a tool that works but is never offered is noticed.
    #
    # Minus the control calls, which are dispatched like tools because that is
    # the only protocol this daemon speaks, and are deliberately kept out of
    # `list_tools` so that no model is ever shown one. They are named here rather
    # than detected, so that adding a handler still has to be a deliberate
    # decision about which of the two kinds it is.
    handlers = sorted(m[len("_tool_"):] for m in dir(rover_daemon.Rover)
                      if m.startswith("_tool_"))
    control = ["set_vision", "nav_status", "map_png", "camera_jpeg", "clear_map",
               "detect_in",
               # Replugging the lidar in software, which is a control call for the
               # same reason wifi_join is: it is the right thing for a person
               # watching a stale map and the wrong thing for a model, since it
               # takes the camera down with it for a few seconds.
               "reset_lidar",
               # Two of the five scripting calls. The other three are model
               # tools now, offered to a client on loopback -- which since the
               # rover started holding its own conversation includes the model.
               # `start_script` and `script_stop` joined `run_script` there when
               # a behaviour stopped having a time limit: what ends one is being
               # stopped, so a model able to start one has to be able to stop
               # it. See LOCAL_ONLY in rover_daemon.py, and the script tool test
               # below for the gate.
               "script_status", "list_api",
               # And the network, which is a control call for a reason of its own:
               # a model that moved the rover onto another access point would be
               # cutting the wire its own conversation arrives on, and no wording
               # of a description makes that a good idea.
               "wifi_status", "wifi_join",
               # The semantic world state, all of it. This slice exists to find
               # out whether that world is worth trusting, and handing a model the
               # authority to write to it -- or to throw it away -- before that
               # question has an answer would be the wrong order. See
               # docs/task-semantic-world-state.md, "Authority boundaries".
               "world_building", "world_inspect", "world_map_session",
               "world_state_clear",
               "world_state_entities", "world_state_entity", "world_state_frame",
               "world_state_observations", "world_state_search",
               "world_state_summary"]
    for name in control:
        check(f"{name} is a control call, not a tool", name in handlers, True)
        check(f"...and is not offered to any model", name in names, False)
    check("every other handler is offered",
          sorted(names), [h for h in handlers if h not in control])
    for tool in every:
        function = tool["function"]
        check(f"{function['name']} describes itself", bool(function.get("description")), True)
        check(f"{function['name']} has a parameter object",
              function["parameters"]["type"], "object")

    # The model's map knobs have to match what the renderer will actually draw.
    # A schema that offers 50 m across is a tool that then silently clamps to 24,
    # and a 4B model will not notice.
    props = rover_daemon.MAP_TOOL["function"]["parameters"]["properties"]
    check("the model can ask how many metres across", "across_m" in props, True)
    check("...and how big a picture", "pixels" in props, True)
    check("...but is not shown the half-extent map_png takes",
          "half_extent_m" in props, False)
    check("...nor pixels-per-cell", "scale" in props, False)
    check("neither map knob is required, so 'show me the map' still works",
          rover_daemon.MAP_TOOL["function"]["parameters"].get("required"), [])
    check("the across ceiling is twice the drawing half-extent",
          props["across_m"]["maximum"], 2 * rover_daemon.MAP_MAX_HALF_EXTENT_M)
    check("the across floor is twice the drawing floor",
          props["across_m"]["minimum"], 1.0)
    check("the picture floor is the drawing floor",
          props["pixels"]["minimum"], rover_daemon.MAP_MIN_PIXELS)
    check("the picture ceiling is the drawing ceiling",
          props["pixels"]["maximum"], rover_daemon.MAP_MAX_PIXELS)


def test_control_calls_without_hardware():
    """The two newest control calls refuse rather than raise when the hardware they
    need is not fitted.

    Worth its own check because both are reached from a window with live buttons
    rather than from a model that would read an explanation: a daemon started
    without `--ros-nav` still shows a `clear map` button, and pressing it has to
    come back as a sentence rather than as a traceback that closes the connection.
    """
    import rover_daemon

    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    blind = rover.call("clear_map", {})
    check("clear_map with nothing to drive is refused", blind["ok"], False)
    # Deliberately not "lidar". The sensor belongs to the ROS stack, so a daemon
    # started without --ros-nav is one whose lidar is spinning perfectly well, and
    # a refusal that named it would send somebody to check a cable that is fine.
    check("...and says why", "--ros-nav" in blind["error"], True)
    # camera_jpeg needs a camera and *not* a vision host, which is the whole
    # difference between it and `look`: a daemon with nowhere to post a picture can
    # still hand one back in the reply.
    check("camera_jpeg is not gated on a vision host",
          "vision" in rover.call("camera_jpeg", {})["error"], False)
    # The script runner is attached by main() once the port is known, so a Rover
    # built any other way -- here, or by anything embedding it -- has none. The
    # four that need one have to say so rather than raise on a None.
    for name in ("run_script", "start_script", "script_status", "script_stop"):
        refused = rover.call(name, {"source": "print(1)"})
        check(f"{name} without a runner is refused", refused["ok"], False)
        check(f"...and says why", "not running scripts" in refused["error"], True)
    # `list_api` is the exception and needs nothing: it describes the primitives
    # by looking at the module, which is a question worth answering on a daemon
    # that is not currently in a position to run anything.
    reference = rover.call("list_api", {})
    check("list_api answers without a runner", reference["ok"], True)
    check("...with the primitives in it",
          all(word in reference["reference"] for word in ("gimbal.look_at", "every")),
          True)


def test_the_api_only_calls_tools_that_exist():
    """Every daemon call `rover_api.py` makes has a handler behind it.

    The same check `test_schemas` makes for the model's schemas, for the other
    surface: a script's primitive that names a tool which has since been renamed
    fails as "no such tool" in the middle of a behaviour, several minutes into a
    run, which is a poor place to find out. Read out of the source rather than
    from a list kept beside it, because a list kept beside it is a list that
    stops being true.
    """
    import re

    import rover_daemon

    with open(os.path.join(HERE, "rover_api.py"), "r", encoding="utf-8") as handle:
        source = handle.read()
    called = sorted(set(re.findall(r'_call\("([a-z_]+)"', source)))
    check("rover_api calls something", len(called) > 8, True)
    missing = [n for n in called if not hasattr(rover_daemon.Rover, f"_tool_{n}")]
    check("every primitive names a tool that exists", missing, [])
    # And a script must not be able to start a script: one slot, and a behaviour
    # that can spawn behaviours is a slot that means nothing.
    check("no primitive starts another script",
          [n for n in called if n.endswith("_script")], [])


def test_where():
    import rover_daemon

    # Positions are described in words because the model reads them out loud;
    # asking a 4B model to do arithmetic on pixel coordinates goes badly.
    wide = 640
    left = rover_daemon._where([10, 100, 80, 80, 0.9], wide, 480)
    middle = rover_daemon._where([280, 100, 80, 80, 0.9], wide, 480)
    right = rover_daemon._where([550, 100, 80, 80, 0.9], wide, 480)
    check("a face on the left is called left", left["where"], "left")
    check("a face in the middle is called centre", middle["where"], "centre")
    check("a face on the right is called right", right["where"], "right")
    near = rover_daemon._where([100, 100, 200, 200, 0.9], wide, 480)
    far = rover_daemon._where([100, 100, 40, 40, 0.9], wide, 480)
    check("a big face is near", near["distance"], "near")
    check("a small face is far", far["distance"], "far")


def test_flags():
    import rover_daemon

    # A small quantised model writes booleans loosely, and refusing those means
    # refusing the tool -- the same reasoning as _level.
    for value in (True, 1, "true", "True", " yes ", "on", "1"):
        check(f"{value!r} means yes", rover_daemon._flag(value, "f"), True)
    for value in (False, 0, "false", "no", "off", "0", ""):
        check(f"{value!r} means no", rover_daemon._flag(value, "f"), False)
    # But a word nobody can read is an error, not a silent False: quietly turning a
    # typo into "no" would hand back a picture facing the wrong way and looking fine.
    for value in ("maybe", "upwards", None, [], {}):
        try:
            rover_daemon._flag(value, "rover_up")
            check(f"{value!r} is refused", "accepted", "ValueError")
        except ValueError as error:
            check(f"{value!r} is refused, saying which argument",
                  "rover_up" in str(error), True)


TESTS = (
    test_schemas,
    test_control_calls_without_hardware,
    test_the_api_only_calls_tools_that_exist,
    test_where,
    test_flags,
)
