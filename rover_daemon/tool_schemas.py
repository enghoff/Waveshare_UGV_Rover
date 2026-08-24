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

MAP_TOOL: dict[str, Any] = {
    "type": "function",
    "function": {
        "name": "show_map",
        "description": (
            "Take a top-down map of the few metres around the rover, built up from "
            "its lidar as it has driven, and look at it. Use this for questions "
            "about the shape of the space or about getting from one place to "
            "another. For a plain question about what is nearby, "
            "describe_surroundings is quicker and more precise."
        ),
        "parameters": {"type": "object", "properties": {}},
    },
}
