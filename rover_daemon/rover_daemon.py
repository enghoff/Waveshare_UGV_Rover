"""The rover's control plane: one process owning the board, camera and tracking loop.

Everything that touches the rover hardware goes through here. The ESP32 hangs off
one UART and the camera can be opened by one process at a time, so separate
programs independently controlling the same devices are not a supported runtime
arrangement.

The current deployed path is:

    python3 rover_daemon.py --vision --board-bridge --ros-nav

Face detection is local YuNet on the rover's own host. `--vision` keeps the current
supervisor interface but no longer names a model host: the active Alibaba client
registers its loopback frame server with `set_vision` before it asks for tools.
An explicit `--vision HOST[:PORT]` remains useful for diagnostics.

Clients speak newline-delimited JSON over TCP -- one request, one reply. The
daemon is the source of truth for tool schemas; clients call `list_tools` rather
than carrying copies.
"""

from __future__ import annotations

import argparse
import json
import socket
import socketserver
import sys
import threading
import time

import scripting

from board_link import (
    DEFAULT_SERIAL, SerialLink, open_link, _battery_percent, _battery_state,
    _field_number, _newest_telemetry,
)
from rover import Rover
from rover_camera import _where, default_camera
from rover_util import _flag, _level
from rover_nav import (
    CAMERA_FOV_DEG, MAP_HALF_EXTENT_M, MAP_MAX_HALF_EXTENT_M, MAP_MAX_PIXELS,
    MAP_MIN_PIXELS, MAP_POINT_MAX_AGE_S, _map_cells, _map_view, _model_map_view,
)
from rover_wifi import _terse_fields, _wifi_networks
from tool_schemas import (
    LOOK_TOOL, MAP_POINT_TOOL, MAP_TOOL, NAV_TOOLS, SCRIPT_TOOL, START_SCRIPT_TOOL,
    STOP_SCRIPT_TOOL, TOOLS,
)

DEFAULT_BOARD_HOST = "192.168.1.22"
DEFAULT_DEVICE = default_camera()
# Bare --vision is retained for the supervised command line, but the current
# client supplies the real loopback destination with set_vision. An empty value
# therefore means "vision-capable, waiting for a client", not an old host.
DEFAULT_VISION = ""
HOST = "0.0.0.0"
PORT = 8769
BRIDGE_PORT = 8772
ROS_NAV_PORT = 8773

# Running rover-side scripts is allowed only to clients on this machine. The
# Alibaba conversation is rover-side now, so loopback is a legitimate model
# client; LAN callers are still refused these code-execution calls.
LOCAL_ONLY = ("run_script", "start_script", "script_stop", "list_api")
LOOPBACK = ("127.0.0.1", "::1", "::ffff:127.0.0.1")


def _requests(rfile):
    """The client's lines, ending quietly when the client goes away.

    A script's process can end with one of its connections still mid-request --
    it is killed, or its last line runs while a job it started is waiting on an
    answer -- and a connection dropped without a shutdown arrives here as a reset
    rather than as end of file. That is a client leaving, which is how every
    conversation this daemon has ends, and not something to print a traceback
    about into the log of a rover that is working perfectly.
    """
    try:
        for raw in rfile:
            yield raw
    except OSError:
        return


class Handler(socketserver.StreamRequestHandler):
    """One client connection: newline-delimited JSON, one reply per request."""

    def handle(self) -> None:
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        rover: Rover = self.server.rover
        local = self.client_address[0] in LOOPBACK
        for raw in _requests(self.rfile):
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
                    reply = {"ok": True, "tools": rover.tools(local=local)}
                elif not isinstance(name, str):
                    reply = {"ok": False, "error": "every request needs a 'call'"}
                elif name in LOCAL_ONLY and not local:
                    reply = {"ok": False,
                             "error": f"{name} is only served on the rover itself; "
                                      f"reach it through an ssh tunnel"}
                else:
                    reply = rover.call(name, request.get("arguments") or {})
                    # set_vision answers with names rather than schemas. The
                    # caller's locality can add script tools, which only this
                    # handler knows, so correct that list here.
                    if local and isinstance(reply.get("tools"), list):
                        reply["tools"] = [t["function"]["name"]
                                          for t in rover.tools(local=True)]
            try:
                self.wfile.write(json.dumps(reply).encode() + b"\n")
            except OSError:
                return


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> int | str:
    """Return 0, or a sentence that `sys.exit` prints with a non-zero status."""
    try:
        import faulthandler
        import signal

        faulthandler.register(signal.SIGUSR1, all_threads=True)
    except (AttributeError, ValueError, OSError):
        pass

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--serial", default=DEFAULT_SERIAL, metavar="PORT",
                        help=f"the ESP32's serial port (default {DEFAULT_SERIAL})")
    parser.add_argument("--host", default=None, metavar="ADDRESS",
                        help=f"command the board over WiFi at this address instead "
                             f"(the ESP32 is {DEFAULT_BOARD_HOST})")
    parser.add_argument("--device", default=DEFAULT_DEVICE, metavar="PATH",
                        help=f"the camera (default {DEFAULT_DEVICE})")
    parser.add_argument("--vision", nargs="?", default=None, const=DEFAULT_VISION,
                        metavar="HOST[:PORT]",
                        help="keep the look/vision path available; the current "
                             "client registers its frame server with set_vision. "
                             "An explicit host:port preconfigures a destination")
    parser.add_argument("--no-camera", dest="camera", action="store_false",
                        help="lights and gimbal only, for a rover with no camera fitted")
    parser.add_argument("--camera-fov", type=float, default=CAMERA_FOV_DEG,
                        metavar="DEGREES",
                        help="how wide a slice of the room the camera sees across "
                             "the picture, drawn on the map as the gimbal's cone")
    parser.add_argument("--ros-nav", nargs="?", default=None, const=ROS_NAV_PORT,
                        type=int, metavar="PORT",
                        help="offer driving/mapping tools backed by ROS 2 on this "
                             f"loopback port (bare flag means {ROS_NAV_PORT})")
    parser.add_argument("--board-bridge", nargs="?", default=None, const=BRIDGE_PORT,
                        type=int, metavar="PORT",
                        help="lend the driver board to ROS 2 on this loopback port "
                             f"(bare flag means {BRIDGE_PORT})")
    parser.add_argument("--bind", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    try:
        link = open_link(args.serial, args.host)
    except Exception as error:
        return f"Cannot reach the driver board: {error}"

    # Local YuNet is the only detector implementation in the current project.
    rover = Rover(link, "local", args.device if args.camera else None,
                  vision=args.vision, camera_fov_deg=args.camera_fov)
    if not rover.probe():
        link.close()
        return f"No answer from the driver board on {link.describe()}. Is it powered?"

    # The gimbal angles are a model. Put the hardware where that model starts.
    rover.centre_gimbal()

    # And the rover starts recording what it sees. Here rather than in Rover's
    # constructor: building the world state is something a *daemon* does, and a
    # bench script that makes a Rover to read the battery should not quietly begin
    # opening a database and taking pictures. The navigator is attached below and
    # the loop reads it through `getattr`, so starting first costs nothing but a
    # look or two without a pose, which are refused rather than stored. It also
    # empties the world state when the host has rebooted since it was recorded,
    # because the map those positions were measured in did not survive the
    # reboot, and it says so here when it did.
    note = rover.start_world_building()
    if note:
        print(note, flush=True)

    if args.ros_nav:
        try:
            from ros_navigator import RosNavigator

            rover.nav = RosNavigator(port=args.ros_nav)
            rover.nav.start()
            up = "answering" if rover.nav.reachable else "not up yet"
            print(f"[rover] driving through ROS 2 on 127.0.0.1:{args.ros_nav} "
                  f"({up})", flush=True)
        except Exception as error:
            rover.nav = None
            print(f"[rover] no driving or mapping: {error}", file=sys.stderr,
                  flush=True)

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

    rover.scripts = scripting.Runner(
        f"127.0.0.1:{args.port}",
        on_start=rover.stop_tracking,
        on_finish=lambda: rover.call("stop_driving", {}))

    server = Server((args.bind, args.port), Handler)
    server.rover = rover
    driving = ("off" if rover.nav is None
               else f"ros2 on 127.0.0.1:{args.ros_nav}")
    print(f"rover daemon on {args.bind}:{args.port} -- board {rover.describe()}, "
          f"camera {args.device if args.camera else 'none'}, detector local YuNet, "
          f"vision {rover.vision.describe() if rover.vision else 'waiting for client'}, "
          f"driving {driving}, "
          f"board shared {bridge.describe() if bridge else 'no'} "
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
