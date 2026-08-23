"""Desk-side checks for talk.py, rover_tools, and endpointing."""
from __future__ import annotations

import io
import sys

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
        import talk
    except ImportError as exc:
        SKIP.append(f"connect errors ({type(exc).__name__}: needs the client venv)")
        return

    def why(url: str) -> str:
        try:
            asyncio.run(talk._open(url, "sk-test", "qwen3.5-omni-plus-realtime-2026-03-15"))
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
    original = talk.OPEN_TIMEOUT_S
    try:
        talk.OPEN_TIMEOUT_S = 0.3
        check("a silent port times out with an explanation",
              "did not answer" in why(f"ws://127.0.0.1:{silent}/api-ws/v1/realtime"), True)
    finally:
        talk.OPEN_TIMEOUT_S = original
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


def test_indicator() -> None:
    """The line that says whether the microphone is open.

    Worth checking because its failure mode is cosmetic and permanent: a status
    that is not rubbed out before a transcript line leaves "listening" welded to
    the front of what was heard, and a redirected run that emits carriage
    returns fills a log with them.
    """
    import io

    try:
        import talk_audio
    except ImportError as exc:
        SKIP.append(f"indicator ({type(exc).__name__}: needs the client venv)")
        return

    class Tty(io.StringIO):
        def isatty(self):
            return True

    def run(stream, script):
        real, sys.stdout = sys.stdout, stream
        try:
            with talk_audio.Indicator() as indicator:
                for step in script:
                    step(indicator)
        finally:
            sys.stdout = real
        return stream.getvalue()

    blank = "\r" + " " * talk_audio.STATUS_WIDTH + "\r"

    written = run(Tty(), [
        lambda i: i.set("listening"),
        lambda i: i.set("listening"),  # the same state again must not redraw
        lambda i: i.set("hearing"),
        lambda i: i.say("you: hello there"),
        lambda i: i.set("listening"),
    ])
    check("an unchanged state is not redrawn", written.count(talk_audio.STATUS["listening"]), 2)
    check("the status is rubbed out before a transcript line",
          blank + "you: hello there\n" in written, True)
    check("...and drawn again afterwards",
          written.split("you: hello there\n")[-1].startswith("\r"), True)
    # Whatever ends the conversation, the terminal is not left holding a
    # "listening" that stopped being true when the process did.
    check("the last thing written is an empty line", written.endswith(blank), True)

    # Redirected to a file, the whole thing goes quiet: a status that changes
    # fifty times a second would otherwise be most of the log.
    piped = run(io.StringIO(), [
        lambda i: i.set("listening"),
        lambda i: i.say("you: hello there"),
        lambda i: i.set("speaking"),
    ])
    check("a redirected run writes no status at all", piped, "you: hello there\n")


def test_endpointer() -> None:
    """The VAD decides when a turn is over -- the client's only real logic."""
    try:
        import numpy as np

        from endpointing import BLOCK, Endpointer
    except ImportError as exc:
        SKIP.append(f"endpointer ({type(exc).__name__}: needs numpy)")
        return

    rng = np.random.default_rng(0)
    quiet = lambda: rng.normal(0, 0.001, BLOCK).astype(np.float32)
    loud = lambda: rng.normal(0, 0.20, BLOCK).astype(np.float32)

    def run(script):
        """script: list of (kind, n_blocks). Returns utterances emitted."""
        ep = Endpointer()
        out = []
        for kind, n in script:
            for _ in range(n):
                got = ep.push(quiet() if kind == "q" else loud())
                if got is not None:
                    out.append(got)
        return out

    # 100 blocks of room tone must not trip anything.
    check("silence emits nothing", len(run([("q", 100)])), 0)

    # 50 blocks (1s) of speech, then 40 blocks (800ms) of silence -- past the
    # 700ms hangover, so exactly one utterance.
    got = run([("q", 30), ("l", 50), ("q", 40)])
    check("one utterance from speech+silence", len(got), 1)

    # ...and it carries the preroll, so the utterance is longer than the speech
    # alone. Without preroll the first consonant is clipped.
    if got:
        check("utterance includes preroll", len(got[0]) > 50 * BLOCK, True)

    # A 4-block (80ms) tick is under the 250ms minimum: a keyboard, not a word.
    check("rejects a too-short burst", len(run([("q", 30), ("l", 4), ("q", 40)])), 0)

    # A pause shorter than the hangover stays inside one turn rather than
    # splitting it -- people pause mid-sentence.
    check(
        "brief pause does not split a turn",
        len(run([("q", 30), ("l", 30), ("q", 20), ("l", 30), ("q", 40)])),
        1,
    )

    # Two turns separated by a real gap are two utterances.
    check(
        "two turns separated by a gap",
        len(run([("q", 30), ("l", 30), ("q", 45), ("l", 30), ("q", 45)])),
        2,
    )

    # Loud room tone must not permanently deafen it: the floor adapts on silence
    # blocks, so speech well above the new floor is still heard.
    ep = Endpointer()
    for _ in range(200):
        ep.push(rng.normal(0, 0.02, BLOCK).astype(np.float32))
    heard = any(ep.push(loud()) is not None or ep.speaking for _ in range(10))
    check("adapts to a noisy room", heard, True)


def test_speculation() -> None:
    """Handing the utterance over early, while the hang window still runs.

    The property that makes this safe is the last check here: what is sent
    early must be a *prefix* of what is confirmed. If that holds, the transcript
    of the early clip is a transcript of the real utterance and reusing it is
    not a guess.
    """
    try:
        import numpy as np

        from endpointing import BLOCK, Endpointer
    except ImportError as exc:
        SKIP.append(f"speculation ({type(exc).__name__}: needs numpy)")
        return

    rng = np.random.default_rng(1)
    quiet = lambda: rng.normal(0, 0.001, BLOCK).astype(np.float32)
    loud = lambda: rng.normal(0, 0.20, BLOCK).astype(np.float32)

    def run(script):
        """Drive the endpointer exactly as talk.py does; report what it emitted."""
        ep = Endpointer()
        utts, guesses, voids, early = [], [], [], []
        for kind, n in script:
            for _ in range(n):
                got = ep.push(quiet() if kind == "q" else loud())
                if got is not None:
                    utts.append(got)
                    early.append(ep.spoke_early)
                    continue
                if ep.take_void():
                    voids.append(True)
                elif (guess := ep.pending()) is not None:
                    guesses.append(guess)
        return utts, guesses, voids, early

    utts, guesses, voids, early = run([("q", 30), ("l", 50), ("q", 40)])
    check("speaks once per utterance", (len(utts), len(guesses)), (1, 1))
    check("nothing voided on a clean turn", voids, [])
    check("confirmed utterance is marked early", early, [True])
    # The whole point: the early clip is sent before the hang window is out, so
    # it is shorter than the utterance that follows it.
    check("early clip is shorter than the confirmed one",
          len(guesses[0]) < len(utts[0]), True)
    check("early clip is a prefix of the confirmed one",
          bool(np.array_equal(guesses[0], utts[0][:len(guesses[0])])), True)

    # A pause mid-sentence. The first speculation is taken back when the speaker
    # carries on -- and then, once they stop for real, a second one goes out
    # covering the whole utterance and *that* is the one confirmed. So a pause
    # costs one wasted transcription, not the benefit: the turn is still early.
    utts, guesses, voids, early = run(
        [("q", 30), ("l", 30), ("q", 10), ("l", 30), ("q", 40)])
    check("a resumed sentence voids the first speculation",
          (len(utts), len(guesses), voids, early), (1, 2, [True], [True]))
    check("the second speculation covers the whole utterance",
          len(guesses[1]) > len(guesses[0]), True)
    check("...and is still a prefix of what was confirmed",
          bool(np.array_equal(guesses[1], utts[0][:len(guesses[1])])), True)

    # Below the speech minimum nothing is sent early either -- a keyboard should
    # not cost a transcription.
    _utts, guesses, _voids, _early = run([("q", 30), ("l", 4), ("q", 40)])
    check("a too-short burst is not sent early", len(guesses), 0)


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
        import talk
    except ImportError as exc:
        SKIP.append(f"frame server ({type(exc).__name__}: needs sounddevice)")
        return

    import http.client
    import json as _json

    frames = talk.Frames(0, host="127.0.0.1")
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
        status, payload = post(b"\xff\xd8" + b"\x00" * talk.MAX_FRAME_BYTES)
        check("...and so is one too big for the model",
              (status, payload["ok"]), (413, False))
        check("...saying what the limit was",
              str(talk.MAX_FRAME_BYTES) in payload["error"], True)

        # Older frames are dropped rather than accumulating, since a client that
        # runs for hours would otherwise hold every picture it ever took.
        for _ in range(talk.MAX_FRAMES + 2):
            post(jpeg)
        check("only a few frames are kept", len(frames._frames), talk.MAX_FRAMES)
    finally:
        frames.shutdown()
        frames.server_close()


def test_speaker() -> None:
    """Playback bookkeeping: what was heard, and what was thrown away."""
    try:
        import talk
    except ImportError as exc:
        SKIP.append(f"speaker ({type(exc).__name__}: needs sounddevice)")
        return

    import numpy as np

    speaker = talk.Speaker(rate=24000)  # no card is opened until start()
    speaker.begin()
    speaker.write(np.ones(24000, dtype=np.float32) * 0.1)  # one second of reply
    check("nothing has been heard yet", speaker.played_ms(), 0)
    check("...and the speaker is busy", speaker.busy, True)

    # Pretend the card asked for a quarter of a second.
    out = np.zeros((6000, 1), dtype=np.float32)
    speaker._fill(out, 6000, None, None)
    check("a quarter second played", speaker.played_ms(), 250)

    dropped = speaker.flush()
    check("the rest is thrown away", round(dropped, 3), 0.75)
    check("...and the speaker falls silent", speaker.busy, False)
    # The number that matters after a barge-in: what the model must be told it
    # actually said, which is what it played and not what it sent.
    check("what was heard is remembered", speaker.played_ms(), 250)


def test_echo_guard() -> None:
    """The suppressor that keeps the rover from interrupting itself."""
    try:
        import talk
    except ImportError as exc:
        SKIP.append(f"echo guard ({type(exc).__name__}: needs sounddevice)")
        return

    speaker = talk.Speaker(rate=24000)
    ears = talk.Ears(speaker, factor=2.5, on=True)
    check("a silent speaker hears everything", ears.hears(0.001), True)

    speaker._level = 0.1  # the rover is talking
    check("its own voice does not get through", ears.hears(0.1), False)
    check("...nor a quiet room over the top of it", ears.hears(0.2), False)
    check("...but somebody talking over it does", ears.hears(0.4), True)

    off = talk.Ears(speaker, factor=2.5, on=False)
    check("switched off, everything gets through", off.hears(0.001), True)


def test_pointing_the_camera() -> None:
    """The client tells the rover where to post pictures, on every connection."""
    try:
        import talk
        import mock_rover
        import rover_tools
    except ImportError as exc:
        SKIP.append(f"camera pointing ({type(exc).__name__}: needs sounddevice)")
        return

    frames = talk.Frames(0, host="127.0.0.1")
    frames.serve_in_background()
    port = frames.server_address[1]

    # A rover started pointing at a host that is not there, which is the state
    # this whole mechanism exists for: the address was a constant, the model
    # moved off that host, and `look` kept posting into the void.
    rover = mock_rover.Rover("192.0.2.1:8767", None)
    server = mock_rover.serve(rover, "127.0.0.1", 0, quiet=True)
    client = rover_tools.RoverClient(f"127.0.0.1:{server.server_address[1]}")
    try:
        client.probe()
        talk.point_camera_here(client, frames)
        check("the rover is told where this client is listening",
              rover.vision, f"127.0.0.1:{port}")
        # And `look` now works, which is the only thing any of it was for.
        result = client.call("look", {})
        check("...so a picture can be taken", result.get("ok"), True)
        check("...and this client is holding it",
              frames.take(result.get("image", "")) is not None, True)

        # No frame server means no picture path, and a tool that cannot reach
        # the model's host is worse than a missing one.
        talk.point_camera_here(client, None)
        check("with nowhere to post, look is withdrawn",
              "look" in [t["function"]["name"] for t in client.tools()], False)
    finally:
        client.close()
        server.shutdown()
        server.server_close()
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

    # Which of those the transcript gets, as opposed to the panel, which gets all
    # of them. The rule is whether it says anything the request line above it did
    # not -- so the planner's verdict does and a turn restating the angle it was
    # given does not.
    logged = console_model.worth_logging
    check("the transcript takes the planning", logged(click), True)
    check("...and the route that came of it", logged(accepted), True)
    check("...and the replan", logged(again), True)
    check("...but not a turn saying it is turning",
          logged({"phase": "turning", "kind": "turn_in_place"}), False)
    check("...nor a drive saying it is driving",
          logged({"phase": "driving", "kind": "drive"}), False)
    check("...nor the ending, which the move's own reply is bringing",
          logged(refused), False)


def test_talk_session() -> None:
    """The protocol, against a service that only writes down what it was told."""
    try:
        import talk
        import mock_rover
        import rover_tools
    except ImportError as exc:
        SKIP.append(f"talk session ({type(exc).__name__}: needs sounddevice)")
        return

    import asyncio
    import base64
    import json as _json

    class Recorder:
        """A WebSocket that goes nowhere."""

        def __init__(self):
            self.sent = []

        async def send(self, raw):
            self.sent.append(_json.loads(raw))

        def types(self):
            return [event["type"] for event in self.sent]

    frames = talk.Frames(0, host="127.0.0.1")
    frames.serve_in_background()
    picture = mock_rover._test_card()
    if picture is None:
        SKIP.append("talk session (no OpenCV to draw a test frame)")
        frames.shutdown()
        frames.server_close()
        return

    rover = mock_rover.Rover(f"127.0.0.1:{frames.server_address[1]}", picture)
    server = mock_rover.serve(rover, "127.0.0.1", 0, quiet=True)
    client = rover_tools.RoverClient(f"127.0.0.1:{server.server_address[1]}")

    async def exercise():
        ws = Recorder()
        session = talk.Session(ws, client, frames, None, talk.Indicator(),
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

