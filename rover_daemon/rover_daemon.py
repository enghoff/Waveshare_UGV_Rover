"""The rover's control plane: one process owning the board, the camera and the loop.

Everything that touches the rover's hardware goes through here. That is not
tidiness, it is the only arrangement that works: the ESP32 hangs off a single
UART and the camera can be opened by one process at a time, so two programs that
both want to command servos or look through the lens are two programs corrupting
each other. `drive_gamepad_pi.py` takes the UART for the wheels and the lights,
and `track_face_pi.py` takes it for the gimbal; running both means interleaved
JSON on one wire, and nothing at all could then also want the camera.

    python3 rover_daemon.py                    # ttyAMA0, camera, detector on the OAK
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
exist for [voice_chat/drive_web.py](../voice_chat/drive_web.py):
`nav_status` returns every number the driving loop has, and `map_png` returns the
map as base64 in the reply instead of posting it away. Both are things a person
watching a move needs and a model asked to narrate one does not.

`list_tools` is why the clients carry no schemas of their own. The daemon is the
only thing that knows what this rover can do, so it is the only thing that should
be describing it -- [voice_chat/talk.py](../voice_chat/talk.py) asks, and
hands the answer straight to the model. Adding a tool is a change to
[tool_schemas.py](tool_schemas.py) and the handler on Rover, with nothing to
redeploy anywhere else.

**Why the tracking loop lives here rather than staying a separate script.**
`track_face_pi.py` is still the right thing to run when face tracking is all you
want; it is standalone, it prints a status line, and it is where the loop was
worked out. This runs the same loop -- importing the same `aiming.py`, so the two
cannot become different robots -- but under a switch, sharing the board with
everything else, so that a conversation can start and stop it.

The client is not on this machine. Speech runs on whatever desk has a
microphone, and [voice_chat/talk.py](../voice_chat/talk.py) reaches this over the
LAN like any other client. That is why this binds an address rather than a Unix
socket, and it is why forwarding frames at 30% of the Pi's core is simply the
cost of tracking rather than something that has to be budgeted against anything
else running here.
"""

from __future__ import annotations

import argparse
import json
import socket
import socketserver
import sys
import threading
import time
from pathlib import Path
from typing import Any

import scripting

from board_link import (
    BAUD, CMD_LIGHTS, CMD_PROBE, DEFAULT_SERIAL, HttpLink, SerialLink,
    open_link, _battery_percent, _battery_state, _battery_summary,
    _field_number, _newest_telemetry,
)
from rover import Rover
from rover_camera import VisionLink, _where
from rover_util import _flag, _level, _number
from rover_nav import (
    CAMERA_FOV_DEG, DEFAULT_LIDAR, MAP_MAX_HALF_EXTENT_M, MAP_MAX_PIXELS,
    _map_cells, _map_view,
)
from rover_wifi import _terse_fields, _wifi_networks
from tool_schemas import LIGHT_MAX, LOOK_TOOL, MAP_TOOL, NAV_TOOLS, TOOLS

DEFAULT_BOARD_HOST = "192.168.1.22"
DEFAULT_SERVICE = "local"
DEFAULT_VISION = "192.168.1.3:8767"
DEFAULT_DEVICE = "/dev/video0"
HOST = "0.0.0.0"
PORT = 8769

# Calls that run code rather than perform an act, and are therefore refused from
# anywhere but this machine. Nothing on this port authenticates -- the same trade
# `face-detect` makes and the same home LAN -- so the difference between the rest
# of the protocol and these is the difference between a stranger flashing the
# headlights and a stranger with a shell on the Pi. Bound to loopback they grant
# what an ssh session here already grants, and are reached the same way.
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
                    reply = {"ok": True, "tools": rover.tools()}
                elif not isinstance(name, str):
                    reply = {"ok": False, "error": "every request needs a 'call'"}
                elif name in LOCAL_ONLY and not local:
                    reply = {"ok": False,
                             "error": f"{name} is only served on the rover itself; "
                                      f"reach it through an ssh tunnel"}
                else:
                    reply = rover.call(name, request.get("arguments") or {})
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
    # the log. Worth the four lines: this rover has one core and a dozen threads,
    # and the question that keeps coming up is not what the code does but which
    # thread is holding the core right now. From outside, a thread that is spinning
    # and a thread that is blocked are both just a number in `top`, and there is no
    # py-spy for armv6 to tell them apart.
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
                        help=f"the face detector: 'local' for the OAK in this "
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
    parser.add_argument("--lidar", nargs="?", default=None, const=DEFAULT_LIDAR,
                        metavar="PORT",
                        help="offer the driving and mapping tools, using the lidar on "
                             "this port; bare --lidar finds it by its stable "
                             "/dev/serial/by-id name. Without this the rover will "
                             "not move itself.")
    parser.add_argument("--camera-fov", type=float, default=CAMERA_FOV_DEG,
                        metavar="DEGREES",
                        help="how wide a slice of the room the camera sees across "
                             "the picture, drawn on the map as the gimbal's cone. "
                             "The default is a guess -- measure it by panning until "
                             "a known object just leaves the frame.")
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

    if args.lidar:
        # Two layouts to satisfy: in the repository this file is in rover_daemon/ and
        # lidar_slam/ is its sibling, while the Pi's ~/ugv is flat with lidar_slam/
        # inside it. Checking for the directory rather than assuming either means a
        # deployment that moves does not silently lose the driving tools.
        here = Path(__file__).resolve().parent
        for candidate in (here.parent / "lidar_slam", here / "lidar_slam"):
            if candidate.is_dir():
                sys.path.insert(0, str(candidate))
                break
        try:
            from navigator import Navigator
            rover.nav = Navigator(link,
                                  None if args.lidar == "auto" else args.lidar,
                                  on_drive_start=rover.park_tracking,
                                  on_drive_end=rover.unpark_tracking)
            # The port is opened by its loop, not here, and retried until it turns
            # up: on this Pi the lidar enumerates 93 s after the kernel starts, long
            # after cron has run this, so insisting on it now would mean every
            # reboot came up without the driving tools.
            rover.nav.start()
            # Said out loud because it is the one property of the map that decides
            # where the rover can still be driven and nothing else reports it. The
            # rover starts at the centre, so half of this is the reach in any
            # direction from wherever it was switched on -- past that a room is
            # driven through and not written down, and the map shows a straight edge
            # with nothing beyond it.
            cfg = rover.nav.slam.config
            print(f"[rover] mapping {cfg.grid_cells}x{cfg.grid_cells} cells at "
                  f"{cfg.resolution_m * 100:.0f} cm, "
                  f"{cfg.grid_cells * cfg.resolution_m:.0f} m across, "
                  f"{cfg.grid_cells * cfg.resolution_m / 2:.0f} m of reach from "
                  f"where it started", flush=True)
        except Exception as error:
            # Not fatal. A rover that cannot drive itself is still a rover that can
            # light up, aim its camera and hold a conversation, and the driving tools
            # simply will not be offered.
            rover.nav = None
            print(f"[rover] no driving or mapping: {error}", file=sys.stderr,
                  flush=True)

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
    print(f"rover daemon on {args.bind}:{args.port} -- board {rover.describe()}, "
          f"camera {args.device if args.camera else 'none'}, detector {args.service}, "
          f"vision {rover.vision.describe() if rover.vision else 'off'}, "
          f"lidar {(rover.nav.lidar_path or 'waiting for it') if rover.nav else 'off'} "
          f"({len(rover.tools())} tools)",
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
