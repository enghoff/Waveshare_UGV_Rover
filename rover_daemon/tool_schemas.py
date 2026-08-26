"""Model-facing tool schemas. Literals, so prompts.py can read them with ast."""
from __future__ import annotations

from typing import Any

LIGHT_MAX = 255

# Wordier than they look because a 4B model at int4 reads these descriptions and
# nothing else. Anything left implicit is invented.

TOOLS: list[dict[str, Any]] = [
    {
        "type": "function",
        "function": {
            "name": "set_lights",
            "description": (
                "Switch or dim the rover's white headlights. The level is 0 for "
                "off, 255 for full brightness, and anything between for dimmer. "
                "Use 255 when asked to turn the lights on and 0 to turn them off."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "level": {"type": "integer", "minimum": 0, "maximum": LIGHT_MAX,
                              "description": "Brightness from 0 (off) to 255 (full)."},
                },
                "required": ["level"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_lights",
            "description": (
                "Report the headlight brightness as a level from 0 to 255 and "
                "whether they are on. The board cannot be read back, so this is "
                "the last level that was set."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "battery",
            "description": (
                "Read how much charge is left in the rover's battery. Use this "
                "whenever you are asked about the battery, the charge, the power, "
                "or how much longer the rover can keep going. It answers with a "
                "percentage, the pack voltage, and one word for the condition: "
                "full, ok, low, critical, or absent when no battery is fitted at "
                "all. Say the percentage rather than the voltage unless volts were "
                "asked for, and say plainly when it is low."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "look_at",
            "description": (
                "Point the rover's camera. pan is degrees left or right of "
                "straight ahead, negative for left and positive for right; tilt "
                "is degrees up or down, negative for down and positive for up. "
                "This stops face tracking if it is running, since both cannot "
                "aim the camera at once."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "pan": {"type": "number", "description": "Degrees; negative left."},
                    "tilt": {"type": "number", "description": "Degrees; negative down."},
                },
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "center_camera",
            "description": "Point the camera straight ahead and level. Stops face tracking.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "count_faces",
            "description": (
                # Reworded 2026-08-16 and measured: the old wording opened "Look "
                # through the camera once", and beside a tool that actually looks
                # it stopped being called at all -- "how many people can you see"
                # called nothing, 0/6. Naming what it is *not* for is what fixed
                # it. See voice_chat/README.md.
                "Count the people in front of the rover and say roughly where "
                "each one is: left, centre or right, and near or far. Use this "
                "only for counting people, not for seeing what something is. "
                "Does not move the camera."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "start_tracking",
            "description": (
                "Start following a face with the camera. The rover keeps whoever "
                "it finds centred in view, and sweeps to look for somebody if "
                "there is nobody about. It picks the largest face it can see, "
                "which is normally the nearest person."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop_tracking",
            "description": "Stop following a face and return the camera to centre.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "track_next",
            "description": (
                "Let go of the person being followed and look for a different "
                "one. The rover cannot tell people apart, so this ignores "
                "whoever is currently being followed for a few seconds and takes "
                "the next face it finds -- which may be the same person again if "
                "nobody else is there. Starts tracking if it was not running."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "tracking_status",
            "description": (
                "Report whether face tracking is running, whether a face is "
                "currently being followed, how many are in view, and where the "
                "camera is pointing."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
]


# Offered only when --vision is given, since without somewhere to send the
# picture this can do nothing but fail. Worded for a model that will otherwise
# answer from its imagination: the failure being designed against is a rover
# that describes a room it has never looked at, which sounds exactly like one
# that has.
#
# Every word of this description was measured, because the first one was not
# called at all -- 0/6 on "what can you see right now" beside the other nine
# schemas, while the same tool alone scored 6/6. A tool is not read on its own:
# it is read against its neighbours, and "take a photograph and look at it" lost
# to a list already full of looking. Naming it as the *only* way to see, and
# pointing the counting question at the tool that counts, took it to 6/6. The
# table is in voice_chat/README.md; change this wording only with numbers.
#
# The opening sentence was measured too, and so was its position. Without it the
# model answers the plainest questions there are -- "what can you see", "check
# your camera", "can you describe what is in front of you" -- with "I'll take a
# picture to see what's in front of me" and takes none: 0/6 each. Naming those
# questions takes them to 6/6, and naming them *first* is worth the last of it
# ("check your camera" is 0/6 with the same sentence at the end). Renaming the
# tool to take_picture, which is the model's own phrase for it, was tried and is
# much worse -- it collides with look_at, so "look around" aims the camera
# instead of photographing, and "what do you see now" falls 6/6 -> 0/6.
#
# The second sentence is about the questions that come *after* a picture, and it
# is here because the picture stopped outliving its turn. Once the looking
# exchange is dropped, "what else is on the table?" is a fresh question with no
# view behind it -- and it was answered "I can't see what's on the table without
# taking a picture" by a rover that could have taken one, 0/6. Naming those
# questions too, at 6 samples a cell:
#
#   "What else is on the table?"   0/6 -> 3/6
#   "What else is there?"          0/6 -> 3/6
#   "Is there anything else?"      0/6 -> 5/6
#   "How many people can you see?" 4/6 -> 6/6
#
# Position again, and again not the obvious one: in *front* of the opening
# sentence it totals higher still but takes "check your camera" 6/6 -> 3/6 and
# "what colour is the box" 6/6 -> 3/6, because it displaces the list that was
# put first for exactly that reason. Second is the only placement measured that
# costs no cell. Folding both lists into one sentence is worse than either
# (55/72 against 65/72) -- the follow-ups need their own sentence, not a longer
# list. Still only a partial fix: two of those cells are 3/6, not 6/6.
LOOK_TOOL: dict[str, Any] = {
    "type": "function",
    "function": {
        "name": "look",
        "description": (
            "Call it when you are asked what you can see, what is in front of "
            "you, to check your camera, or to describe or read anything. "
            "Call it again for a follow-up about the same view -- what else is "
            "there, what else is on something, whether there is anything else, "
            "or what colour or shape something is. You keep no picture between "
            "questions, so answering one of those means taking a new one. "
            "See what is in front of the rover. This is the only way you can see "
            "anything at all: it takes a photograph through the camera and shows "
            "it to you. Use it for every question about what is there, what "
            "something is, what it looks like, what it says, or what the rover "
            "can see. To count how many people are there, use the counting tool "
            "instead. It does not move the camera."
        ),
        "parameters": {"type": "object", "properties": {}},
    },
}


# Offered only when the daemon has a lidar, for the same reason `look` is offered
# only when there is somewhere to send a picture: a tool that cannot reach its
# hardware is worse than a missing one, because the model reports success and
# nothing happens.
NAV_TOOLS: list[dict[str, Any]] = [
    {
        "type": "function",
        "function": {
            "name": "drive",
            "description": (
                "Drive the rover straight forward. It watches its lidar the "
                "whole way and stops itself rather than hitting anything, but it "
                "does not steer around obstacles -- use drive_to for that, which "
                "plans a route. Always says how far it actually got and why it "
                "stopped, which will often be less than asked for. Pauses face "
                "tracking while it moves and resumes it afterwards. It cannot see "
                "steps, drops, or anything above or below the height of its lidar, "
                "so do not drive it near a stair or a table edge on the strength of "
                "this. To change heading, turn on the spot first."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "distance_m": {
                        # Forward only, even though the backend can reverse --
                        # Nav2's BackUp behaviour is what `drive` uses for a
                        # negative distance and it is wired up. This was inherited
                        # from the daemon's own planner, whose drive loop was
                        # forward-only, and it is a choice now rather than a
                        # constraint: `drive` is the reflex call a model reaches
                        # for, and a model that can ask for reverse in one word
                        # will. `drive_to` with a negative `ahead_m` backs up, and
                        # takes a planner and a costmap with it.
                        "type": "number", "minimum": 0.05, "maximum": 3.0,
                        "description": "How far to go, in metres.",
                    },
                    "speed_ms": {
                        # 0.5 rather than 0.35, and the old ceiling was not a
                        # ceiling at all. Measured on this chassis by
                        # ros_nav/calibrate_chassis.py, the slowest PWM the motors
                        # will turn at already does 0.33 m/s and PWM 140 does
                        # 0.68 -- so 0.35 was very nearly this rover's *minimum*,
                        # and every request was being pinned to the bottom of the
                        # range with no speed control left over.
                        "type": "number", "minimum": 0.05, "maximum": 0.5,
                        "description": "Metres per second. Leave it out for a "
                                       "sensible walking pace. This chassis will "
                                       "not move below about a third of a metre a "
                                       "second, so anything smaller is treated as "
                                       "that.",
                    },
                },
                "required": ["distance_m"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "drive_to",
            "description": (
                "Drive to a place a given distance ahead and to the left of the "
                "rover, in metres, going around obstacles. It plans a route of "
                "straight segments and turns, follows it without needing to hit "
                "the line exactly, and plans again if something gets in the way "
                "or the room has changed. Distances are from where the rover is "
                "now, not from where it started: positive ahead is forward, "
                "positive left is to its left, negatives are behind and right. "
                "Always says how far it actually got and why it stopped. Prefer "
                "this over a series of drive and turn calls when you know where "
                "you want to end up. It cannot see steps, drops, or table tops. "
                "This can take minutes over a long route."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "ahead_m": {
                        "type": "number", "minimum": -15.0, "maximum": 15.0,
                        "description": "Metres forward of the rover; negative is "
                                       "behind.",
                    },
                    "left_m": {
                        "type": "number", "minimum": -15.0, "maximum": 15.0,
                        "description": "Metres to the rover's left; negative is "
                                       "right.",
                    },
                    "speed_ms": {
                        # 0.5 rather than 0.35, and the old ceiling was not a
                        # ceiling at all. Measured on this chassis by
                        # ros_nav/calibrate_chassis.py, the slowest PWM the motors
                        # will turn at already does 0.33 m/s and PWM 140 does
                        # 0.68 -- so 0.35 was very nearly this rover's *minimum*,
                        # and every request was being pinned to the bottom of the
                        # range with no speed control left over.
                        "type": "number", "minimum": 0.05, "maximum": 0.5,
                        "description": "Metres per second. Leave it out for a "
                                       "sensible walking pace. This chassis will "
                                       "not move below about a third of a metre a "
                                       "second, so anything smaller is treated as "
                                       "that.",
                    },
                },
                "required": ["ahead_m", "left_m"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "turn_in_place",
            "description": (
                "Turn the rover on the spot without going anywhere, by a number of "
                "degrees: positive turns left, negative turns right. Use this to "
                "face something before driving to it, and use it to get out of a "
                "tight spot: turning is never refused, because rotating is how a "
                "rover that has got too close to something gets away from it. It "
                "turns more slowly when something is within about 25 cm, and says so."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "angle_deg": {
                        "type": "number", "minimum": -180, "maximum": 180,
                        "description": "Degrees to turn; positive is left.",
                    },
                },
                "required": ["angle_deg"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop_driving",
            "description": (
                "Stop the rover moving immediately. Use this the moment anyone asks "
                "it to stop, or if something sounds wrong."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "describe_surroundings",
            "description": (
                # Named for what it answers rather than for the sensor, on the same
                # reasoning as count_faces: a tool called "read the lidar" does not
                # get called when somebody asks what is around the rover.
                "Say what is around the rover and how much room it has, measured "
                "with its lidar rather than seen with its camera. Gives the walls, "
                "any free-standing objects, the gaps between them and how far it "
                "can go forward. Use this to answer questions about space, room, "
                "distance and what is in the way, and before driving somewhere. It "
                "does not use the camera and cannot tell you what anything is -- the "
                "lidar measures one flat slice at its own height, so a table appears "
                "only as its legs."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
]

# Bounds the model is shown for `show_map`. They live here, next to the schema,
# because prompts.py reads this file with ast and cannot see rover_nav. Across is
# twice the drawing half-extent (so 24 m is MAP_MAX_HALF_EXTENT_M); the pixel
# bounds are MAP_MIN_PIXELS and MAP_MAX_PIXELS. selftest.py refuses a drift.
MAP_ACROSS_MIN_M = 1.0
MAP_ACROSS_MAX_M = 24.0
MAP_PIXELS_MIN = 200
MAP_PIXELS_MAX = 1200

MAP_TOOL: dict[str, Any] = {
    "type": "function",
    "function": {
        "name": "show_map",
        "description": (
            "Take a top-down map of the space around the rover, built up from "
            "its lidar as it has driven, and look at it. Use this for questions "
            "about the shape of the space or about getting from one place to "
            "another. For a plain question about what is nearby, "
            "describe_surroundings is quicker and more precise. Leave the "
            "arguments out for about six metres across, which is a room. Pass "
            "across_m to see more or less of the floor -- a few metres to judge "
            "what the rover is about to drive into, up to twenty-four metres for "
            "the shape of a whole floor. Distances on a wide view drift, so do "
            "not plan a route home off one. Pass pixels only when you need a "
            "larger picture to read detail; a bigger picture takes the rover "
            "longer to draw."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "across_m": {
                    "type": "number",
                    "minimum": MAP_ACROSS_MIN_M,
                    "maximum": MAP_ACROSS_MAX_M,
                    "description": (
                        "How many metres of room the picture should cover. Leave "
                        "it out for about six, a room. Use a few metres for what "
                        "is immediately around the rover, and more for the shape "
                        "of a floor."
                    ),
                },
                "pixels": {
                    "type": "integer",
                    "minimum": MAP_PIXELS_MIN,
                    "maximum": MAP_PIXELS_MAX,
                    "description": (
                        "How big a picture to look at. Leave it out for a normal "
                        "one. 320 is a quick look; 640 shows more detail and "
                        "costs the rover more time to draw."
                    ),
                },
            },
            "required": [],
        },
    },
}



# The point a model names on the map picture, as a fraction of the picture's own
# width and height. The bounds live here beside the schema for the reason the map
# bounds above do: prompts.py reads this file with ast and cannot import rover_nav.
MAP_POINT_MIN = 0.0
MAP_POINT_MAX = 1.0

# **Why a fraction of a picture and not a pair of metres.** The daemon's `drive_to`
# has taken a point in the map's own frame ever since the drive console learned to
# send taps, and those two arguments are deliberately withheld from every model --
# the argument is written out in `_tool_drive_to` in rover_nav.py. It comes to this:
# nothing a model can see says where the rover is in that frame, because the room
# reaches it as bearings and the map as a picture centred on itself, so a model
# handed metres could only invent them, and an invented pair is a fifteen-metre
# drive to a place nobody chose.
#
# A fraction of the picture is the same destination named in the one frame a model
# genuinely has in front of it. It has just looked at the map; every number it
# needs to point at a doorway is a property of the image on its screen rather than
# of a coordinate system it has never been told. The daemon holds the pose the
# picture was drawn at and does the conversion, which is the same arithmetic and
# the same function the console uses for a mouse click.
MAP_POINT_TOOL: dict[str, Any] = {
    "type": "function",
    "function": {
        "name": "drive_to_map_point",
        "description": (
            "Drive to a place you can see on the map picture, by saying "
            "whereabouts on that picture it is. Look at the map with show_map "
            "first, then point at the spot on it. across is how far along the "
            "picture from the left: 0 at the left edge, 0.5 in the middle, 1 at "
            "the right edge. down is how far from the top: 0 at the top edge, "
            "0.5 in the middle, 1 at the bottom. The rover is the red triangle "
            "in the very middle, at 0.5 across and 0.5 down. Which part of the "
            "picture is in front of the rover depends on how the page is turned, "
            "and the map's own caption says which way that is, so read it rather "
            "than assuming the top of the picture is ahead. Point at green, "
            "which is floor the rover can reach: black is something solid and "
            "grey is floor it has never seen, and both are refused. Use this for "
            "somewhere you can see on the map, and drive_to instead when you "
            "know how far away the place is in metres. It plans a route around "
            "whatever is in the way and can take minutes over a long one."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "across": {
                    "type": "number",
                    "minimum": MAP_POINT_MIN, "maximum": MAP_POINT_MAX,
                    "description": "How far across the picture, from 0 at the "
                                   "left edge to 1 at the right.",
                },
                "down": {
                    "type": "number",
                    "minimum": MAP_POINT_MIN, "maximum": MAP_POINT_MAX,
                    "description": "How far down the picture, from 0 at the top "
                                   "edge to 1 at the bottom.",
                },
                "speed_ms": {
                    "type": "number", "minimum": 0.05, "maximum": 0.5,
                    "description": "Metres per second. Leave it out for a "
                                   "sensible walking pace. This chassis will not "
                                   "move below about a third of a metre a "
                                   "second, so anything smaller is treated as "
                                   "that.",
                },
            },
            "required": ["across", "down"],
        },
    },
}

# Offered only to a client on the loopback interface, which is the condition the
# three tools below share and the only one here that is about the caller rather
# than about the hardware. The rest of that argument is in `LOCAL_ONLY` in
# rover_daemon.py: these are the calls that run code rather than performing an
# act, so a stranger on the LAN is shown none of them and would be refused them.
# The model is inside that gate because the conversation moved onto the rover,
# not because the gate was opened.
#
# **This one's description is finished at runtime, not here.** `{api}` is filled
# with `rover_api.signatures()` and `{limit_s}` with the runner's own ceiling,
# both by `Rover.script_tools`, so a primitive that is renamed or a limit that is
# retuned cannot go on being advertised the way it used to be. What stays here is
# a literal, because prompts.py reads this file with `ast` and cannot run it.
SCRIPT_TOOL: dict[str, Any] = {
    "type": "function",
    "function": {
        "name": "run_script",
        "description": (
            "Write a short Python program and run it on the rover. Use this when "
            "what has been asked for is not one of your other tools but a "
            "sequence of them: something done a number of times over, something "
            "that keeps going while something else is true, something that has to "
            "look at a value before it acts, or several acts that have to happen "
            "in a particular order. Do not use it for anything one of your other "
            "tools already does on its own -- flashing the headlights three times "
            "is a program, turning them on is not. "
            "You cannot talk while it runs and it is stopped after {limit_s:.0f} "
            "seconds, so keep it to a few seconds' work; anything longer is "
            "something to say you cannot do yet. What comes back to you is "
            "whatever the program printed, so print what you want to be able to "
            "say afterwards. If it fails you are told which line failed and may "
            "fix it and run it again. Say what you are about to do before calling "
            "this, and never read the program itself out loud. The program cannot "
            "see: to look at something, use your own looking tool rather than "
            "writing a program about it. It is written against these primitives "
            "and can reach nothing else on the rover:\n"
            "from rover_api import lights, gimbal, camera, tracking, drive, "
            "power, every, wait\n"
            "{api}"
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "source": {
                    "type": "string",
                    "description": (
                        "The whole program, as Python source. Real newlines "
                        "between the lines and four spaces for an indent."
                    ),
                },
            },
            "required": ["source"],
        },
    },
}
# The other two thirds of the same idea, and the reason they are model tools at
# all is that a behaviour has no time limit any more (see the docstring in
# [scripting.py](scripting.py)): something that runs until it is told to stop
# needs somebody able to tell it. `start_script` hands back a handle instead of a
# result and `script_stop` ends whatever is holding the one slot, and both are
# loopback-only for exactly the reason `run_script` is.
#
# **Neither repeats the primitive list, deliberately.** All three of these are in
# front of the model in one list, so the surface written into `run_script`'s
# description above is already there to be read, and a second copy of it is nine
# hundred characters paid again on every turn of a realtime conversation to say
# something the model has just been told. So these two say "the same primitives
# as run_script" and nothing filled in at runtime, which is also why they are
# plain literals rather than templates.
START_SCRIPT_TOOL: dict[str, Any] = {
    "type": "function",
    "function": {
        "name": "start_script",
        "description": (
            "Start a program on the rover and carry on talking while it runs. "
            "Use this instead of run_script when what has been asked for has no "
            "end written into it -- something to keep doing, something to do "
            "until you are told to stop, something to watch for a while -- or "
            "when it would plainly take more than a few seconds. It is written "
            "against exactly the same primitives as run_script, listed in that "
            "tool's description. Nothing comes back but the fact that it "
            "started, so what the program prints is not something you will get "
            "to say: say what it is going to do before you call this, and do not "
            "claim afterwards that it has finished. It runs until it ends on its "
            "own or somebody stops it -- a loop with no end in it runs until "
            "then -- and script_stop is what ends it. Only one program runs at a "
            "time, so if one is already going this is refused and tells you "
            "which; stop that one first."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "source": {
                    "type": "string",
                    "description": (
                        "The whole program, as Python source. Real newlines "
                        "between the lines and four spaces for an indent."
                    ),
                },
            },
            "required": ["source"],
        },
    },
}

STOP_SCRIPT_TOOL: dict[str, Any] = {
    "type": "function",
    "function": {
        "name": "script_stop",
        "description": (
            "Stop the program that is running. Call it the moment you are asked "
            "to stop, or told that what it is doing is done with, or asked for "
            "something the program is in the way of. It is never a mistake to "
            "call: with nothing running it does nothing and says so. The program "
            "is asked politely first and has a couple of seconds to tidy up, so "
            "it may put its lights out or straighten up on the way. What comes "
            "back says whether anything was stopped, how long it had been going "
            "and whatever it printed. This stops a program and not the rover: to "
            "halt the wheels on their own, use stop_driving."
        ),
        "parameters": {"type": "object", "properties": {}},
    },
}
