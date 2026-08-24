"""The rover's control plane: one process owning the board, the camera and the loop.

Everything that touches the rover's hardware goes through here. That is not
tidiness, it is the only arrangement that works: the ESP32 hangs off a single
UART and the camera can be opened by one process at a time, so two programs that
both want to command servos or look through the lens are two programs corrupting
each other. `drive_gamepad_pi.py` takes the UART for the wheels and the lights,
and `track_face_pi.py` takes it for the gimbal; running both means interleaved
JSON on one wire, and nothing at all could then also want the camera.

    python3 rover_daemon.py                    # the board's UART, camera, YuNet here
    python3 rover_daemon.py --host 192.168.1.22    # board over WiFi instead
    python3 rover_daemon.py --no-camera        # lights and gimbal only
    python3 rover_daemon.py --vision 192.168.1.3:8767   # ...and it can be looked through

`--vision` adds one tool, `look`, which takes a picture and posts it to the
voice service's `/frame` so that a vision-language model can be asked about it.
The frame goes straight from here to that card -- the same road the detector's
frames already take -- and never through the client holding the conversation.
It is a flag rather than a default because it only works against a service
running a model that can take an image: without one, `look` is a tool that can
only fail, and offering it is worse than not having it. Dropping the flag is
therefore the whole rollback on this side, since clients ask `list_tools` afresh
every time they connect.

Clients speak newline-delimited JSON over TCP -- one request, one reply:

    -> {"call": "set_lights", "arguments": {"level": 255}}
    <- {"ok": true, "level": 255, "on": true}
    -> {"call": "list_tools"}
    <- {"ok": true, "tools": [ ...JSON schemas... ]}

Four calls in that protocol are for the client rather than for the model, and none
of them appears in `list_tools`, so no model is ever shown them. `list_tools`
itself is one. `set_vision` says where `look` should post its pictures:

    -> {"call": "set_vision", "arguments": {"address": "192.168.1.7:8767"}}
    <- {"ok": true, "vision": "http://192.168.1.7:8767/frame", "tools": [...]}

The last two are for driving the rover by hand rather than by conversation, and
exist for [drive_web/drive_web.py](../drive_web/drive_web.py):
`nav_status` returns every number the driving loop has, and `map_png` returns the
map as base64 in the reply instead of posting it away. Both are things a person
watching a move needs and a model asked to narrate one does not.

`list_tools` is why the clients carry no schemas of their own. The daemon is the
only thing that knows what this rover can do, so it is the only thing that should
be describing it -- [voice_chat/session.py](../voice_chat/session.py) asks, and
hands the answer straight to the model. Adding a tool is a change to
[tool_schemas.py](tool_schemas.py) and the handler on Rover, with nothing to
redeploy anywhere else.

**Why the tracking loop lives here rather than staying a separate script.**
`track_face_pi.py` is still the right thing to run when face tracking is all you
want; it is standalone, it prints a status line, and it is where the loop was
worked out. This runs the same loop -- importing the same `aiming.py`, so the two
cannot become different robots -- but under a switch, sharing the board with
everything else, so that a conversation can start and stop it.

**The client used to be somewhere else, and one gate here depends on where it
is.** Speech used to run on a desk with a microphone, reaching this over the LAN,
which is why this binds an address rather than a Unix socket. The usual
arrangement now is the rover holding its own conversation -- the browser has the
microphone and [drive_web/omni_bridge.py](../drive_web/omni_bridge.py) has the
session -- so the model's tool calls arrive on loopback. `LOCAL_ONLY` below is
the one place that matters, because a rule written as "loopback only" was doing
duty as "no model, ever", and those two stopped meaning the same thing.
"""

from __future__ import annotations

import argparse
import json
import socket
import socketserver
import sys
import threading
import time
from typing import Any

import scripting

from board_link import (
    BAUD, CMD_LIGHTS, CMD_PROBE, DEFAULT_SERIAL, HttpLink, SerialLink,
    open_link, _battery_percent, _battery_state, _battery_summary,
    _field_number, _newest_telemetry,
)
from rover import Rover
from rover_camera import VisionLink, _where, default_camera
from rover_util import _flag, _level, _number
from rover_nav import (
    CAMERA_FOV_DEG, MAP_MAX_HALF_EXTENT_M, MAP_MAX_PIXELS,
    _map_cells, _map_view,
)
from rover_wifi import _terse_fields, _wifi_networks
from tool_schemas import (
    LIGHT_MAX, LOOK_TOOL, MAP_TOOL, NAV_TOOLS, SCRIPT_TOOL, TOOLS,
)

DEFAULT_BOARD_HOST = "192.168.1.22"
DEFAULT_SERVICE = "local"
DEFAULT_VISION = "192.168.1.3:8767"
DEFAULT_DEVICE = default_camera()
HOST = "0.0.0.0"
PORT = 8769
# Where the driver board is lent to the ROS 2 stack, on loopback only. 8769 is
# this daemon, 8770 the depth camera and 8771 the drive console, so the next one
# along. See board_bridge.py for why it is not simply another tool here.
BRIDGE_PORT = 8772
# And the one after that, going the other way: where the ROS 2 stack lends its
# navigation back. The pair is the whole interface between the two halves of this
# rover -- 8772 is hardware going out, 8773 is goals, the map and the pose coming
# in. See ros_navigator.py and ros_nav/nav_bridge.py.
ROS_NAV_PORT = 8773

# Calls that run code rather than perform an act, and are therefore refused from
# anywhere but this machine. Nothing on this port authenticates -- the same trade
# `face-detect` makes and the same home LAN -- so the difference between the rest
# of the protocol and these is the difference between a stranger flashing the
# headlights and a stranger with a shell on the rover. Bound to loopback they grant
# what an ssh session here already grants, and are reached the same way.
#
# **The conversation is inside this gate now, and that is the whole change.** When
# this was written, the client holding the conversation was on whichever desk had
# the microphone, so "loopback only" and "no model, ever" were the same sentence.
# The rover holds its own session with Alibaba's model today -- see
# [drive_web/omni_bridge.py](../drive_web/omni_bridge.py) -- so those tool calls
# arrive here from 127.0.0.1 like any other local client, and `run_script` is
# offered to the model in `list_tools` deliberately rather than by oversight. What
# the gate still refuses is a stranger on the LAN, which is what it was for; what
# it no longer implies is that nothing conversational can reach these.
#
# The other four stay unadvertised. A model that can run a program for fifteen
# seconds and be told what it printed does not also need to start one that
# outlives the question, and `list_api` is a catalogue whose contents are now
# written into `run_script`'s own description anyway.
#
# `script_status` is deliberately not among them. Watching a behaviour run is
# what a console on a desk wants, it changes nothing, and everything else this
# port hands out about the rover's state is already served on the LAN.
LOCAL_ONLY = ("run_script", "start_script", "script_stop", "list_api")
LOOPBACK = ("127.0.0.1", "::1", "::ffff:127.0.0.1")


class Handler(socketserver.StreamRequestHandler):
    """One client connection: newline-delimited JSON, one reply per request."""

    def handle(self) -> None:
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        rover: Rover = self.server.rover
        local = self.client_address[0] in LOOPBACK
        for raw in self.rfile:
            raw = raw.strip()
            if not raw:
                continue
            try:
                request = json.loads(raw)
            except ValueError:
                reply = {"ok": False, "error": "not JSON"}
            else:
                name = request.get("call")
                if name == "list_tools":
                    # Where the client is decides whether it is shown the one
                    # tool it would be refused. See `Rover.tools`.
                    reply = {"ok": True, "tools": rover.tools(local=local)}
                elif not isinstance(name, str):
                    reply = {"ok": False, "error": "every request needs a 'call'"}
                elif name in LOCAL_ONLY and not local:
                    reply = {"ok": False,
                             "error": f"{name} is only served on the rover itself; "
                                      f"reach it through an ssh tunnel"}
                else:
                    reply = rover.call(name, request.get("arguments") or {})
                    # `set_vision` answers with the tool names as they now
                    # stand, and one of those names depends on who is asking --
                    # which is knowledge this end has and a tool handler does
                    # not. Corrected here rather than passed down, so that
                    # "where the client is" stays in the one place that knows.
                    if local and isinstance(reply.get("tools"), list):
                        reply["tools"] = [t["function"]["name"]
                                          for t in rover.tools(local=True)]
            try:
                self.wfile.write(json.dumps(reply).encode() + b"\n")
            except OSError:
                return  # the client went away mid-reply; nothing to say about it


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> int | str:
    """Returns 0, or a message -- sys.exit prints a string and exits non-zero."""
    # `kill -USR1 <pid>` makes the daemon write a stack for every thread it has to
    # the log. Worth the four lines: this rover has four cores and a dozen threads,
    # and the question that keeps coming up is not what the code does but which
    # thread is holding a core right now. From outside, a thread that is spinning
    # and a thread that is blocked are both just a number in `top`.
    try:
        import faulthandler
        import signal

        faulthandler.register(signal.SIGUSR1, all_threads=True)
    except (AttributeError, ValueError, OSError):
        pass                     # no SIGUSR1 off Linux; the daemon still runs
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--serial", default=DEFAULT_SERIAL, metavar="PORT",
                        help=f"the ESP32's serial port (default {DEFAULT_SERIAL})")
    parser.add_argument("--host", default=None, metavar="ADDRESS",
                        help=f"command the board over WiFi at this address instead "
                             f"(the ESP32 is {DEFAULT_BOARD_HOST})")
    parser.add_argument("--service", default=DEFAULT_SERVICE, metavar="HOST[:PORT]",
                        help=f"the face detector: 'local' for YuNet in this "
                             f"process, or host:port for one over HTTP "
                             f"(default {DEFAULT_SERVICE})")
    parser.add_argument("--device", default=DEFAULT_DEVICE, metavar="PATH",
                        help=f"the camera (default {DEFAULT_DEVICE})")
    parser.add_argument("--vision", nargs="?", default=None, const=DEFAULT_VISION,
                        metavar="HOST[:PORT]",
                        help="offer the 'look' tool, posting frames to this vision "
                             f"service (bare --vision means {DEFAULT_VISION})")
    parser.add_argument("--no-camera", dest="camera", action="store_false",
                        help="lights and gimbal only, for a rover with no camera fitted")
    parser.add_argument("--camera-fov", type=float, default=CAMERA_FOV_DEG,
                        metavar="DEGREES",
                        help="how wide a slice of the room the camera sees across "
                             "the picture, drawn on the map as the gimbal's cone. "
                             "The default is a guess -- measure it by panning until "
                             "a known object just leaves the frame.")
    parser.add_argument("--ros-nav", nargs="?", default=None, const=ROS_NAV_PORT,
                        type=int, metavar="PORT",
                        help="offer the driving and mapping tools, backed by the "
                             "ROS 2 stack reached on this loopback port (bare flag "
                             f"means {ROS_NAV_PORT}). Without it the rover will not "
                             "move itself. Goes together with --board-bridge, which "
                             "is how the ROS side reaches the wheels.")
    parser.add_argument("--board-bridge", nargs="?", default=None, const=BRIDGE_PORT,
                        type=int, metavar="PORT",
                        help="lend the driver board to the ROS 2 stack on this "
                             f"loopback port (bare flag means {BRIDGE_PORT}). It is "
                             "the board's encoders and gyro on the way out and its "
                             "motor commands on the way in; see board_bridge.py.")
    parser.add_argument("--bind", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    try:
        link = open_link(args.serial, args.host)
    except Exception as error:
        return f"Cannot reach the driver board: {error}"

    rover = Rover(link, args.service, args.device if args.camera else None,
                  vision=args.vision, camera_fov_deg=args.camera_fov)
    if not rover.probe():
        link.close()
        return f"No answer from the driver board on {link.describe()}. Is it powered?"
    # The one thing that moves before anybody asks: the gimbal angles are a model
    # kept true by putting the camera where this thinks it is, since it cannot ask.
    rover.centre_gimbal()

    if args.ros_nav:
        # Nothing is opened and nothing is waited for. The ROS stack and this
        # daemon are started by the same crontab and the stack takes the best part
        # of a minute, so anything that insisted on it being up would mean every
        # reboot came back without the driving tools. Each tool connects when it is
        # called, and says so plainly when there is nothing to connect to.
        try:
            from ros_navigator import RosNavigator

            rover.nav = RosNavigator(port=args.ros_nav,
                                     on_drive_start=rover.park_tracking,
                                     on_drive_end=rover.unpark_tracking)
            rover.nav.start()
            up = "answering" if rover.nav.reachable else "not up yet"
            print(f"[rover] driving through ROS 2 on 127.0.0.1:{args.ros_nav} "
                  f"({up})", flush=True)
        except Exception as error:
            rover.nav = None
            print(f"[rover] no driving or mapping: {error}", file=sys.stderr,
                  flush=True)

    # The board, lent out. Started before the tool server rather than after, so
    # that a ROS stack brought up by the same crontab does not have to race it --
    # and failing to start it is not fatal for the same reason a missing lidar is
    # not: a rover that will not share its board is still a rover.
    bridge = None
    if args.board_bridge:
        try:
            import board_bridge
            bridge = board_bridge.BoardBridge(link, port=args.board_bridge)
            bridge.start()
            print(f"[rover] driver board shared on {bridge.describe()}", flush=True)
        except Exception as error:
            bridge = None
            print(f"[rover] board not shared: {error}", file=sys.stderr, flush=True)

    # A script reaches the rover by connecting back to this daemon on loopback,
    # like any other client -- so it can be told where that is only once the port
    # is settled. Starting one stops face tracking, for the reason `look_at` does:
    # two things aiming one gimbal is two robots. Every run ends with the wheels
    # stopped, which is a no-op unless it was killed in the middle of a move.
    rover.scripts = scripting.Runner(
        f"127.0.0.1:{args.port}",
        on_start=rover.stop_tracking,
        on_finish=lambda: rover.call("stop_driving", {}))

    server = Server((args.bind, args.port), Handler)
    server.rover = rover
    driving = ("off" if rover.nav is None
               else f"ros2 on 127.0.0.1:{args.ros_nav}")
    print(f"rover daemon on {args.bind}:{args.port} -- board {rover.describe()}, "
          f"camera {args.device if args.camera else 'none'}, detector {args.service}, "
          f"vision {rover.vision.describe() if rover.vision else 'off'}, "
          f"driving {driving}, "
          f"board shared {bridge.describe() if bridge else 'no'} "
          # Both counts, because there are honestly two: `run_script` is offered
          # to a client on loopback and refused to one on the LAN, so a single
          # number here would be wrong for one of the two readers of this line.
          f"({len(rover.tools())} tools, {len(rover.tools(local=True))} on loopback)",
          flush=True)

    def release_idle_camera() -> None:
        while True:
            time.sleep(5.0)
            rover.idle_tick()

    threading.Thread(target=release_idle_camera, daemon=True).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nstopping")
        server.server_close()
        rover.close()
        print("camera released, gimbal centred")
    return 0


if __name__ == "__main__":
    sys.exit(main())
