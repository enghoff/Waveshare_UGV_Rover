"""Checks for the omni session, rover_tools, and the prompt reader."""
from __future__ import annotations

from test_harness import FAIL, PASS, SKIP, check


def test_rover_client() -> None:
    """The line to the rover daemon. What the daemon does with a call is its own
    selftest's business -- rover_daemon/selftest.py, which runs on the rover."""
    import json as _json
    import socket
    import socketserver
    import threading

    try:
        import rover_tools
    except ImportError as exc:
        SKIP.append(f"rover client ({type(exc).__name__})")
        return

    seen = []

    class Fake(socketserver.StreamRequestHandler):
        def handle(self):
            for raw in self.rfile:
                request = _json.loads(raw)
                seen.append(request)
                if request.get("call") == "list_tools":
                    reply = {"ok": True, "tools": [{"type": "function",
                                                    "function": {"name": "set_lights"}}]}
                elif request.get("call") == "hang_up":
                    return  # close mid-conversation, as a restarted daemon would
                else:
                    reply = {"ok": True, "echo": request}
                self.wfile.write(_json.dumps(reply).encode() + b"\n")

    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True

    server = Server(("127.0.0.1", 0), Fake)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    host, port = server.server_address
    client = rover_tools.RoverClient(f"{host}:{port}")
    try:
        check("the daemon is found", client.probe(), True)
        check("tools come from the daemon, not from here",
              [t["function"]["name"] for t in client.tools()], ["set_lights"])
        check("a call reaches the daemon whole",
              client.call("set_lights", {"level": 255})["echo"],
              {"call": "set_lights", "arguments": {"level": 255}})

        # `run_script` is a program running on the rover rather than a line down
        # a UART, and the daemon may sit on it for half a minute before answering
        # about one it had to kill. Waited out on purpose: a timeout here is
        # reported to the model as "no answer from the rover daemon", so the
        # short patience would have it announce a dead rover over a script that
        # merely hit its limit.
        client.call("run_script", {"source": "print(1)"})
        check("a script is given the patience a job needs",
              client._sock.gettimeout(), rover_tools.RUN_SCRIPT_TIMEOUT_S)
        # And the next call gets it back, because the connection is kept open
        # across calls: a daemon that has genuinely gone should be noticed in
        # twelve seconds, not in thirty.
        client.call("ping", {})
        check("...and the call after it is back to a tool's patience",
              client._sock.gettimeout(), rover_tools.TIMEOUT_S)

        # A daemon that was restarted between two questions closes the
        # connection this client was keeping open. That must cost a reconnect,
        # not a tool call -- the failure it replaces is a conversation that
        # cannot touch the rover again until it is restarted too.
        client.call("hang_up", {})
        check("a dropped connection is remade", client.call("ping", {})["ok"], True)

        # And remaking it must not send the client back to the name. `bpi-m4zero.local`
        # is answered by mDNS -- multicast UDP, with nothing retransmitting it --
        # so on a rover whose wifi has gone weak the lookup is what fails first,
        # while the connection it was wanted for would have worked. Re-resolving
        # on every reconnect is what made a merely weak link read as an absent
        # rover on all six panels of the console at once.
        real_lookup = socket.getaddrinfo
        lookups = []

        def counted(*args, **kwargs):
            lookups.append(args[0])
            return real_lookup(*args, **kwargs)

        socket.getaddrinfo = counted
        try:
            client.call("hang_up", {})
            remade = client.call("ping", {})
        finally:
            socket.getaddrinfo = real_lookup
        check("a dropped connection is remade on the address already known",
              remade["ok"], True)
        check("...without asking for the name a second time", lookups, [])
    finally:
        client.close()
        server.shutdown()
        server.server_close()

    # A remembered address is not a hardcoded one. The wifi address can move,
    # so an address that stops answering is exactly how a client finds out it
    # has moved, and it has to ask the name again rather than go on dialling
    # where the rover used to be. That is the bug docs/hosts.md is about;
    # remembering an address without this would be a fresh way of writing it.
    first = Server(("127.0.0.1", 0), Fake)
    threading.Thread(target=first.serve_forever, daemon=True).start()
    second = Server(("127.0.0.1", 0), Fake)
    threading.Thread(target=second.serve_forever, daemon=True).start()
    now_at = ["127.0.0.1", first.server_address[1]]
    real_lookup = socket.getaddrinfo

    def mdns(host, port, *args, **kwargs):
        # Stands in for mDNS: one name, answered with wherever the rover is now.
        if host == "rover.invalid":
            host, port = now_at
        return real_lookup(host, port, *args, **kwargs)

    socket.getaddrinfo = mdns
    client = rover_tools.RoverClient(f"rover.invalid:{first.server_address[1]}")
    try:
        check("the rover is reached by name", client.probe(), True)
        client.call("hang_up", {})       # so the next call has to open a new one
        first.shutdown()
        first.server_close()
        now_at[1] = second.server_address[1]
        check("...and followed once the address it remembered stops answering",
              client.call("ping", {})["ok"], True)
    finally:
        socket.getaddrinfo = real_lookup
        client.close()
        second.shutdown()
        second.server_close()

    # Where this machine is, as the rover sees it. Taken off the socket rather
    # than guessed, because a desk has several addresses and only one of them is
    # on the way to the rover -- and which one that is changes when the rover
    # drives off its dock. It is what the client tells the daemon to post
    # pictures to, so a wrong answer here is a `look` that fails with a routing
    # error much later.
    server = Server(("127.0.0.1", 0), Fake)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    client = rover_tools.RoverClient(f"127.0.0.1:{server.server_address[1]}")
    try:
        check("the client knows which address the rover reaches it on",
              client.local_address(), "127.0.0.1")
    finally:
        client.close()
        server.shutdown()
        server.server_close()

    # And a daemon that is simply not there answers as a failure the model can
    # read out, rather than raising into the middle of a turn.
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        dead_port = probe.getsockname()[1]
    gone = rover_tools.RoverClient(f"127.0.0.1:{dead_port}")
    check("an absent daemon is not found", gone.probe(), False)
    result = gone.call("set_lights", {"level": 255})
    check("...and a call to it fails as a result", result["ok"], False)
    check("...saying where it was looking", "rover daemon" in result["error"], True)

    # Discovery, which is where the real bug was: a client that knows only one
    # of the rover's addresses reports no rover while the daemon is up and
    # serving. A dead candidate must be stepped over rather than concluded from.
    server = Server(("127.0.0.1", 0), Fake)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    live = f"127.0.0.1:{server.server_address[1]}"
    try:
        found = rover_tools.discover((f"127.0.0.1:{dead_port}", live))
        check("discovery steps over a dead address", found is not None, True)
        if found is not None:
            check("...and settles on the live one", found.describe(), live)
            # The short probe timeout must not stay in force afterwards, or the
            # first slow tool call would be cut off at a second and a half.
            check("...with the working timeout restored",
                  found._connect_timeout, rover_tools.CONNECT_TIMEOUT_S)
            found.close()
        check("discovery with nothing there gives None",
              rover_tools.discover((f"127.0.0.1:{dead_port}",)), None)
    finally:
        server.shutdown()
        server.server_close()

    # The name has to come first: it is the only candidate that stays right if
    # the wifi address moves, and a failed name lookup is slow enough that
    # paying for one before an address that would have worked is a real cost.
    check("the rover is looked for by name first",
          rover_tools.DEFAULT_CANDIDATES[0], "bpi-m4zero.local")

def test_connect_errors() -> None:
    """What the client says when the hosted service is not there.

    Each way the connection can fail must arrive as one sentence about the
    right cause -- a traceback out of `websockets` names asyncio internals and
    not which host, or which key, was refused.
    """
    import asyncio
    import socket
    import threading

    try:
        import session as omni
    except ImportError as exc:
        SKIP.append(f"connect errors ({type(exc).__name__}: needs websockets)")
        return

    def why(url: str) -> str:
        try:
            asyncio.run(omni._open(url, "sk-test", "qwen3.5-omni-plus-realtime-2026-03-15"))
            return "connected"
        except SystemExit as error:
            return str(error)
        except Exception as error:  # the failure this whole thing exists to prevent
            return f"raw {type(error).__name__}: {error}"

    check("a name that does not resolve says so",
          "cannot reach" in why("wss://nx.invalid.example/api-ws/v1/realtime"), True)

    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        dead = probe.getsockname()[1]
    refused = why(f"ws://127.0.0.1:{dead}/api-ws/v1/realtime")
    check("a refused port is explained, not raised", "cannot reach" in refused, True)
    check("...and names the port", f"127.0.0.1:{dead}" in refused, True)

    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    silent = listener.getsockname()[1]
    original = omni.OPEN_TIMEOUT_S
    try:
        omni.OPEN_TIMEOUT_S = 0.3
        check("a silent port times out with an explanation",
              "did not answer" in why(f"ws://127.0.0.1:{silent}/api-ws/v1/realtime"), True)
    finally:
        omni.OPEN_TIMEOUT_S = original
        listener.close()

    import http.server

    class Plain(http.server.BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_GET(self):
            self.send_error(404)

        def log_message(self, *args):
            pass

    http_server = http.server.HTTPServer(("127.0.0.1", 0), Plain)
    threading.Thread(target=http_server.serve_forever, daemon=True).start()
    try:
        wrong = why(f"ws://127.0.0.1:{http_server.server_address[1]}/")
        check("a plain HTTP answer is explained", "rather than upgrading" in wrong, True)
        check("...and quotes what it answered", "404" in wrong, True)
    finally:
        http_server.shutdown()
        http_server.server_close()

def test_prompts() -> None:
    """The prompt and the schemas are read from the source, not copied into it."""
    try:
        import prompts
    except ImportError as exc:
        SKIP.append(f"prompt reader ({type(exc).__name__})")
        return

    schemas = prompts.tools()
    check("every tool the daemon offers is found",
          prompts.names(schemas),
          ["set_lights", "get_lights", "battery", "look_at", "center_camera",
           "count_faces", "start_tracking", "stop_tracking", "track_next",
           "tracking_status", "look"])
    check("look is last, where the daemon appends it",
          prompts.names(schemas)[-1], "look")
    check("without vision there is no look",
          "look" in prompts.names(prompts.tools(vision=False)), False)
    # The reason this module exists rather than a literal: the ceiling is written
    # as a name in the daemon and has to survive being read out.
    lights = next(t for t in schemas if t["function"]["name"] == "set_lights")
    check("a schema's named constants are resolved",
          lights["function"]["parameters"]["properties"]["level"]["maximum"], 255)

    prompt = prompts.system_prompt()
    check("the prompt is unwrapped from its environment default",
          prompt.startswith("You are the voice of a small tracked rover."), True)
    # The sentence whose position was worth nine points out of ninety. It goes
    # last, and a client that reassembled the prompt in a different order would
    # be running a different experiment than the one that was measured.
    check("the tool prompt is in it", "never say you have switched" in prompt, True)
    check("...and the sentence about 'I will' is last",
          prompt.rstrip().endswith("Describe only what is actually in the picture."),
          True)
    check("vision can be left out",
          "take a picture first" in prompts.system_prompt(vision=False), False)

def test_frames() -> None:
    """The /frame contract the daemon posts to, served by the client instead."""
    try:
        import talk_frames
    except ImportError as exc:
        SKIP.append(f"frame server ({type(exc).__name__})")
        return

    import http.client
    import json as _json

    frames = talk_frames.Frames(0, host="127.0.0.1")
    frames.serve_in_background()
    port = frames.server_address[1]

    def post(body: bytes, path: str = "/frame"):
        connection = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
        connection.request("POST", path, body=body,
                           headers={"Content-Length": str(len(body))})
        response = connection.getresponse()
        payload = _json.loads(response.read())
        connection.close()
        return response.status, payload

    try:
        # A JPEG with a real frame header, so the size can be read back out of it
        # without decoding anything.
        jpeg = (b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00"
                b"\xff\xc0\x00\x11\x08\x01\xe0\x02\x80\x03\x01\x22\x00\x02\x11\x01"
                b"\x03\x11\x01" + b"\x00" * 64 + b"\xff\xd9")
        status, payload = post(jpeg)
        check("a posted frame is accepted", (status, payload["ok"]), (200, True))
        check("...and named", payload["image"], "frame-1")
        check("...and measured without decoding it",
              (payload["w"], payload["h"]), (640, 480))

        held = frames.take("frame-1")
        check("the frame is held for the turn that asked", held, jpeg)
        # One picture answers one question. The camera is on a gimbal that sweeps
        # while tracking runs, so a frame kept past its turn is a picture of
        # somewhere the rover is no longer pointing.
        check("...and only once", frames.take("frame-1"), None)

        status, payload = post(b"this is not a picture")
        check("something that is not a JPEG is refused",
              (status, payload["ok"]), (400, False))
        status, payload = post(b"\xff\xd8" + b"\x00" * talk_frames.MAX_FRAME_BYTES)
        check("...and so is one too big for the model",
              (status, payload["ok"]), (413, False))
        check("...saying what the limit was",
              str(talk_frames.MAX_FRAME_BYTES) in payload["error"], True)

        # Older frames are dropped rather than accumulating, since a client that
        # runs for hours would otherwise hold every picture it ever took.
        for _ in range(talk_frames.MAX_FRAMES + 2):
            post(jpeg)
        check("only a few frames are kept", len(frames._frames), talk_frames.MAX_FRAMES)
    finally:
        frames.shutdown()
        frames.server_close()

def test_move_commentary() -> None:
    """What the console makes of a move the rover is still in the middle of.

    `drive_to` answers once, at the end, so everything a person watching a click
    on the map wants to know arrives through nav_status while the move runs. This
    covers both halves of that: the English, and the rule that decides which of
    those sentences is worth a line in the transcript.
    """
    try:
        import console_model
    except ImportError as exc:
        SKIP.append(f"move commentary ({type(exc).__name__})")
        return

    say = console_model.move_sentence

    # A rover that has not been asked for anything, and one too old to publish
    # this at all. Neither may invent a commentary.
    check("an idle rover says nothing", say({"phase": "idle", "seq": 0}), "")
    check("and a rover with no move field says nothing", say({}), "")

    click = {"seq": 1, "kind": "drive_to", "phase": "planning",
             "asked": {"ahead_m": 1.2, "left_m": -0.4}}
    check("a click is acknowledged in the units it was made in", say(click),
          "planning a route to ahead +1.20 m, left -0.40 m")

    accepted = dict(click, seq=2, phase="driving", route_m=1.86, waypoints=4,
                    replans=0)
    check("an accepted route says how far and how many corners", say(accepted),
          "route accepted: 1.86 m through 4 waypoints")
    check("...and one corner is not one corners",
          say(dict(accepted, waypoints=1)),
          "route accepted: 1.86 m through 1 waypoint")

    # The rejection, which is the case this was asked for: a reason, not a silence
    # followed by a rover that never moved.
    refused = dict(click, seq=2, phase="ended", reason="blocked",
                   why="that place is solid")
    check("a refusal carries the planner's reason", say(refused),
          "blocked -- that place is solid")

    # Mid-route. The reason belongs to the replan and must not survive into the
    # route that comes back from it.
    again = dict(accepted, seq=3, phase="replanning", replans=1,
                 route_m=None, waypoints=None,
                 why="drifted 0.61 m off the route, so planning again from here")
    check("a replan says what provoked it", say(again),
          "replanning (#1) -- drifted 0.61 m off the route, so planning "
          "again from here")
    check("and its conclusion is the next route, with no reason attached",
          say(dict(again, seq=4, phase="driving", route_m=1.2, waypoints=3, why="")),
          "route accepted: 1.20 m through 3 waypoints")
    check("an ending counts the replans it took",
          say(dict(again, seq=5, phase="ended", reason="arrived", why="",
                   replans=2)),
          "arrived, after 2 replans")

    check("a turn is reported in degrees",
          say({"seq": 1, "kind": "turn_in_place", "phase": "turning",
               "asked": {"angle_deg": -90.0}}),
          "turning -90 deg")
    check("a straight drive in metres",
          say({"seq": 1, "kind": "drive", "phase": "driving",
               "asked": {"distance_m": 0.5}}),
          "driving 0.50 m")


def test_talk_session() -> None:
    """The protocol, against a service that only writes down what it was told."""
    try:
        import session as omni
        import talk_frames
        import mock_rover
        import rover_tools
    except ImportError as exc:
        SKIP.append(f"omni session ({type(exc).__name__})")
        return

    import asyncio
    import base64
    import json as _json

    class _Notes:
        def set(self, state):
            pass
        def clear(self):
            pass
        def say(self, text, err=False):
            pass

    class Recorder:
        """A WebSocket that goes nowhere."""

        def __init__(self):
            self.sent = []

        async def send(self, raw):
            self.sent.append(_json.loads(raw))

        def types(self):
            return [event["type"] for event in self.sent]

    frames = talk_frames.Frames(0, host="127.0.0.1")
    frames.serve_in_background()
    picture = mock_rover._test_card()
    if picture is None:
        SKIP.append("omni session (no OpenCV to draw a test frame)")
        frames.shutdown()
        frames.server_close()
        return

    rover = mock_rover.Rover(f"127.0.0.1:{frames.server_address[1]}", picture)
    server = mock_rover.serve(rover, "127.0.0.1", 0, quiet=True)
    client = rover_tools.RoverClient(f"127.0.0.1:{server.server_address[1]}")

    async def exercise():
        ws = Recorder()
        session = omni.Session(ws, client, frames, None, _Notes(),
                                   duplex=False, model="test", quiet=True)
        await session.configure(client.tools(), vision=True)
        sent = ws.sent[0]["session"]
        check("the session carries the daemon's schemas untouched",
              [t["function"]["name"] for t in sent["tools"]][:2],
              ["set_lights", "get_lights"])
        check("...and the deployed prompt",
              sent["instructions"].startswith("You are the voice of a small"), True)
        check("...and no turn detection when this client is doing the turns",
              sent["turn_detection"], None)

        # A tool call arriving as the service sends one.
        await session.handle({
            "type": "response.function_call_arguments.done",
            "call_id": "call_1", "name": "set_lights",
            "arguments": ' {"level": 255}'})  # the service pads with a space
        await session.handle({"type": "response.done", "response": {}})
        await session.drain()
        check("the call reached the rover", rover.lights, 255)
        result = next(e for e in ws.sent if e["type"] == "conversation.item.create")
        check("...and the result went back under its own call id",
              result["item"]["call_id"], "call_1")
        check("...as the daemon's answer, verbatim",
              _json.loads(result["item"]["output"]), {"ok": True, "level": 255})
        check("...and a reply was asked for", ws.types()[-1], "response.create")
        await session.handle({"type": "response.created", "response": {}})
        await session.handle({"type": "response.done", "response": {}})

        # And a call that produces a picture. The frame is not in the tool
        # result -- it arrives at this machine by the other road -- so what has
        # to happen is a lookup and a turn of its own.
        ws.sent.clear()

        async def acknowledge():
            """Stand in for the service confirming the picture's turn landed."""
            while True:
                if any(e["type"] == "input_audio_buffer.commit" for e in ws.sent):
                    session._landed.set()
                    return
                await asyncio.sleep(0.005)

        watcher = asyncio.create_task(acknowledge())
        await session.handle({
            "type": "response.function_call_arguments.done",
            "call_id": "call_2", "name": "look", "arguments": "{}"})
        await session.handle({"type": "response.done", "response": {}})
        await session.drain()
        watcher.cancel()
        check("a picture travels as audio, then image, then a commit",
              ws.types(),
              ["conversation.item.create", "input_audio_buffer.append",
               "input_image_buffer.append", "input_audio_buffer.commit",
               "response.create"])
        image = next(e for e in ws.sent if e["type"] == "input_image_buffer.append")
        check("...and it is the frame the rover posted",
              base64.b64decode(image["image"]), picture)

        # A frame this client is not holding. It happens for a dull reason --
        # two clients can hold the same port on Windows, so the rover's picture
        # goes to the other one -- and the consequence is not dull at all: told
        # the photograph succeeded and shown no photograph, the model describes
        # the room anyway, in confident detail, and none of it was ever there.
        # So the result the model sees has to stop saying it worked.
        ws.sent.clear()
        jpeg, rewritten = session._picture({"ok": True, "image": "frame-does-not-exist"})
        check("a missing frame yields no picture", jpeg, None)
        check("...and the result no longer claims to have worked",
              rewritten["ok"], False)
        check("...and says so in words the model can repeat",
              "never arrived" in rewritten["error"], True)
        check("...without leaving a name behind to describe", rewritten["image"], None)

        # A result that names nothing is left exactly as the rover wrote it.
        plain = {"ok": True, "level": 255, "on": True}
        check("a result with no picture in it is untouched",
              session._picture(plain), (None, plain))

        # Nothing is idle until the reply that was asked for has begun.
        check("a reply that was asked for is not idle", session.idle, False)
        await session.handle({"type": "response.created", "response": {}})
        await session.handle({"type": "response.done", "response": {}})
        check("...and is once it has been and gone", session.idle, True)

    try:
        asyncio.run(exercise())
    finally:
        client.close()
        server.shutdown()
        server.server_close()
        frames.shutdown()
        frames.server_close()
