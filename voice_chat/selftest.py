"""Offline checks for the parts of the voice stack that need no GPU and no mic.

Two halves, deliberately runnable independently, because they have different
dependencies and live on different machines:

    python voice_chat/selftest.py          # client half, needs only numpy
    ssh root@media /opt/voice_chat/.venv/bin/python /opt/voice_chat/selftest.py

Whichever half cannot import its dependencies is reported as skipped rather than
failing, so each machine runs the part that is actually deployed on it.

What is covered is the logic that decides *when* to speak and *what* to hand the
synthesiser -- the places where a bug is silent rather than loud. Tool calls are
covered for the same reason from both ends: the sniffer that keeps a call from
being read out loud, and the dispatch that turns one into a command to the
board. The models themselves are not covered here; they need the card.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PASS, FAIL, SKIP = [], [], []


def check(name: str, got, want) -> None:
    if got == want:
        PASS.append(name)
    else:
        FAIL.append(f"{name}\n     got:  {got!r}\n     want: {want!r}")


def test_sentences() -> None:
    """The splitter decides what Kokoro is handed, one clause at a time."""
    try:
        from server import _sentences
    except Exception as exc:
        SKIP.append(f"sentence splitter ({type(exc).__name__}: needs the server venv)")
        return

    def split(*pieces):
        return list(_sentences(iter(pieces)))

    check(
        "splits on sentence end",
        split("Hello there. ", "How are you?"),
        ["Hello there.", "How are you?"],
    )
    # The regression that motivates the lookahead: a decimal point is not a
    # sentence end, and neither is a dotted quad.
    check(
        "keeps decimals intact",
        split("The rover is 3.5 metres away."),
        ["The rover is 3.5 metres away."],
    )
    check(
        "keeps IP addresses intact",
        split("Connect to 192.168.1.4 first."),
        ["Connect to 192.168.1.4 first."],
    )
    # Short interjections get glued forward rather than spoken alone, where the
    # falling intonation of a one-word "sentence" sounds wrong.
    check(
        "glues short fragments forward",
        split("Yes. The battery is at forty percent."),
        ["Yes. The battery is at forty percent."],
    )
    check(
        "flushes the tail without punctuation",
        split("no trailing period here"),
        ["no trailing period here"],
    )
    check("empty stream yields nothing", split(""), [])
    # Streaming must not change the answer: the model emits sub-word pieces, and
    # a boundary landing mid-chunk should give the same split as one blob.
    blob = "First one here. Second one here. Third one here."
    check(
        "token-by-token matches whole-string",
        split(*list(blob)),
        split(blob),
    )
    check("splits on newlines too", split("One line here.\nAnother line here."),
          ["One line here.", "Another line here."])

    # The first chunk may break at a clause, because it is the only one that
    # gates first-audio. Here there is no sentence end to be had yet, and
    # waiting for one would hold the speaker silent for the rest of the reply.
    check(
        "first chunk breaks at a clause",
        split("I can see a desk and a lamp, ", "and a chair behind them."),
        ["I can see a desk and a lamp,", "and a chair behind them."],
    )
    # ...and only the first. Later chunks are already waiting on the speaker
    # rather than the card, so a clause break there costs prosody for nothing.
    check(
        "later chunks do not break at a clause",
        split("One thing here. Then, later, more text follows here."),
        ["One thing here.", "Then, later, more text follows here."],
    )
    # Too short to speak alone, clause or not.
    check(
        "short first clause waits",
        split("Yes, ", "it is on."),
        ["Yes, it is on."],
    )
    # The same trick _SENTENCE_END uses, for the same reason: a comma inside a
    # number has no space after it, so it is not a clause boundary.
    check(
        "keeps thousands separators intact",
        split("It is 1,234 metres away"),
        ["It is 1,234 metres away"],
    )
    check(
        "keeps clock times intact",
        split("Set the alarm for 3:30 and wait"),
        ["Set the alarm for 3:30 and wait"],
    )


def test_tool_sniffer() -> None:
    """What keeps a tool call from being read out loud, brace by brace."""
    try:
        from server import _parse_tool_call, _ToolSniffer
    except Exception as exc:
        SKIP.append(f"tool sniffer ({type(exc).__name__}: needs the server venv)")
        return

    def run(*pieces):
        """Feed the stream in chunks. Returns (prose, parsed call)."""
        sniffer = _ToolSniffer()
        prose = "".join(sniffer.feed(piece) for piece in pieces) + sniffer.flush()
        return prose, (_parse_tool_call(sniffer.tail) if sniffer.tail else None)

    call = '{"name": "set_lights", "arguments": {"level": 255}}'
    want = {"name": "set_lights", "arguments": {"level": 255}}

    # The two shapes a call arrives in, decided by the tokenizer rather than the
    # model: with its markers, or -- if they were skipped as special tokens --
    # as a bare object. Neither may leave a single character of prose behind.
    check("marked call is not spoken", run(f"<tool_call>\n{call}\n</tool_call>"), ("", want))
    check("bare call is not spoken", run(call), ("", want))
    check("leading newline before a bare call", run(f"\n{call}"), ("", want))

    # The marker arrives in sub-word pieces, so it has to be recognised across
    # chunk boundaries -- the regression that would otherwise speak "<tool" and
    # swallow the rest.
    check("marker split across chunks", run("<tool", "_call>", call), ("", want))
    check("call split across chunks", run("<tool_call>", '{"name": "get_', 'lights"}'),
          ("", {"name": "get_lights", "arguments": {}}))

    # Prose in front of a call is still spoken; prose is never held back waiting
    # for a marker that is not coming.
    check("prose before a call survives", run(f"Switching them on. <tool_call>{call}"),
          ("Switching them on. ", want))
    check("plain reply passes through", run("The lights are on."), ("The lights are on.", None))
    check("a lone angle bracket is not a marker", run("Under <3 volts."),
          ("Under <3 volts.", None))

    # Arguments as a JSON string rather than an object: some templates do this,
    # and a call that will not parse must come back as None so the turn falls
    # back to speaking rather than silently doing nothing.
    check("arguments as a string", _parse_tool_call('{"name": "set_lights", '
                                                    '"arguments": "{\\"level\\": 0}"}'),
          {"name": "set_lights", "arguments": {"level": 0}})
    check("unparseable call is refused", _parse_tool_call("<tool_call>{not json"), None)
    check("a call with no name is refused", _parse_tool_call('{"arguments": {}}'), None)

    # The contract _run_turn depends on: sentences arrive first and are spoken
    # as they land, the call arrives last and is never spoken. Standing the
    # model in for itself, since the ordering is the thing under test.
    import server

    original = server._generate
    try:
        def fake(pieces):
            # The stub takes /chat's overrides too. _reply_stream passes them
            # through whether or not they were given, so a stub that predates
            # them fails as a TypeError inside the thing under test.
            server._generate = lambda history, tools=(), system=None, temperature=None: \
                iter(pieces)
            return list(server._reply_stream([], []))

        check(
            "a call is yielded after the prose that preceded it",
            fake(["Switching them on. ", "<tool_call>", call]),
            [("sentence", "Switching them on."), ("tool", want)],
        )
        check(
            "a call on its own yields no speech at all",
            fake([f"<tool_call>{call}</tool_call>"]),
            [("tool", want)],
        )
        check(
            "an ordinary reply yields only sentences",
            fake(["The lights are on. ", "Anything else?"]),
            [("sentence", "The lights are on."), ("sentence", "Anything else?")],
        )
        # Something that opened like a call and was not one gets spoken rather
        # than swallowed: a silent turn is a worse failure than an odd one.
        check(
            "a malformed call is spoken rather than lost",
            fake(['{"name": "set_ligh']),
            [("sentence", '{"name": "set_ligh')],
        )
    finally:
        server._generate = original


def test_trim() -> None:
    """History trimming, which was inert for the whole life of this service.

    `apply_chat_template(tokenize=True)` returns a BatchEncoding on transformers
    5, so the old `len(...)` read 2 -- the number of keys -- and every history
    fitted the cache no matter how long it was. Nothing failed; turns just fell
    through to the dynamic cache and got slower. Hence a check on the token
    count itself, not only on the trimming built over it.
    """
    try:
        import server
        from transformers import AutoTokenizer

        server._tokenizer = AutoTokenizer.from_pretrained(server.LLM_MODEL)
    except Exception as exc:
        SKIP.append(f"history trimming ({type(exc).__name__}: needs the server venv)")
        return

    # The regression itself. Any small constant here means a mapping was
    # measured instead of the tokens inside it.
    check("a prompt is counted in tokens, not dict keys",
          server._prompt_len([{"role": "user", "content": "hello"}]) > 8, True)

    budget = server.CACHE_LEN - server.MAX_NEW_TOKENS - 32
    history = []
    for i in range(200):
        history.append({"role": "user", "content": f"Question number {i} about the rover."})
        history.append({"role": "assistant", "content": f"Answer number {i}, at some length."})

    check("an overlong history starts over budget", server._prompt_len(history) > budget, True)
    trimmed = server._trim(history)
    check("...and is trimmed to fit the cache", server._prompt_len(trimmed) <= budget, True)
    check("...keeping the most recent turn", trimmed[-1], history[-1])
    check("...and opening on a user turn", trimmed[0]["role"], "user")

    # A picture is one token in the rendered prompt and several hundred once the
    # processor has expanded it. If that is not added on, a history holding a
    # frame measures nearly empty, nothing is ever trimmed for it, and the
    # prompt overruns the static cache -- which does not fail, it just quietly
    # gets slower. The same shape of bug as the BatchEncoding one above.
    with_picture = [{"role": "user",
                     "content": [{"type": "image", "image": object()},
                                 {"type": "text", "text": "what is this"}]}]
    without = [{"role": "user", "content": "what is this"}]
    check("a picture is counted at what it costs",
          server._prompt_len(with_picture) - server._prompt_len(without),
          server._image_tokens)

    # A turn that called a tool is four messages, not two. Cutting a fixed pair
    # strands a call without its result, or a result with no call, and a model
    # shown either starts narrating tool plumbing out loud.
    exchange = [
        {"role": "user", "content": "put the lights on"},
        {"role": "assistant", "content": "",
         "tool_calls": [{"type": "function",
                         "function": {"name": "set_lights", "arguments": {"level": 255}}}]},
        {"role": "tool", "content": '{"ok": true, "level": 255, "on": true}'},
        {"role": "assistant", "content": "The lights are on."},
    ]
    original = server.CACHE_LEN
    try:
        # A window with room for roughly one exchange, so trimming is forced.
        server.CACHE_LEN = server.MAX_NEW_TOKENS + 32 + 120
        cut = server._trim(exchange * 6)
        check("a trimmed history opens on a user turn", cut[0]["role"], "user")
        check(
            "no tool result is left stranded without its call",
            [m["role"] for m in cut].count("tool")
            <= sum(1 for m in cut if m.get("tool_calls")),
            True,
        )
    finally:
        server.CACHE_LEN = original


def test_vision() -> None:
    """The picture path on this side: what is held, counted and forgotten.

    None of it needs the card or a vision model -- the images here are ordinary
    objects standing in for decoded frames, since nothing under test looks
    inside one. What is under test is the bookkeeping, which is where this can
    go wrong quietly: a picture that is counted as one token overruns the cache
    window, and one that is never forgotten fills it.
    """
    try:
        import server
    except Exception as exc:
        SKIP.append(f"vision plumbing ({type(exc).__name__}: needs the server venv)")
        return

    picture, older = object(), object()

    def took(image):
        """The three messages a look leaves behind, in the order _run_turn adds them."""
        return [
            {"role": "assistant", "content": "",
             "tool_calls": [{"type": "function", "function": {"name": "look", "arguments": {}}}]},
            {"role": "tool", "content": '{"ok": true}'},
            {"role": "user", "content": [{"type": "image", "image": image},
                                         {"type": "text", "text": "the picture"}]},
        ]

    history = ([{"role": "user", "content": "what can you see"}]
               + took(older)
               + [{"role": "assistant", "content": "A desk."},
                  {"role": "user", "content": "and now"}]
               + took(picture))

    check("images are found in the order the template wants them",
          server._images(history), [older, picture])
    check("a conversation with no pictures has none", server._images(history[:2]), [])
    # For counting and trimming, an image reduces to the text beside it rather
    # than being rendered -- neither of those should pay for a picture.
    check("counting sees text, not pictures",
          [m["content"] for m, original in zip(server._textual(history), history)
           if isinstance(original["content"], list)], ["the picture", "the picture"])

    # A picture is a user message, so it must not be read as the start of a new
    # exchange -- it belongs to the question that took it.
    check("an exchange is a spoken turn and everything answering it",
          [len(group) for group in server._exchanges(history)], [5, 4])

    # Only the newest exchange that looked survives, and it survives whole.
    kept = list(history)
    server._forget_pictures(kept)
    check("only the newest picture is kept", server._images(kept), [picture])
    check("...and the older one's call goes with it",
          sum(1 for m in kept if m.get("tool_calls")), 1)
    check("...along with the answer spoken from it",
          [m["content"] for m in kept if isinstance(m.get("content"), str) and m["content"]],
          ["and now", '{"ok": true}'])
    # ...and at the start of a turn, none of them survive: the camera has moved
    # since, so the newest is a picture of somewhere else, and a model holding
    # one does not take another.
    gone = list(history)
    server._forget_pictures(gone, keep_newest=False)
    check("a new turn starts with no picture at all", server._images(gone), [])
    # The call that fetched it goes too, and so does the reply spoken from it:
    # the model answers the next four questions about what is in front of it out
    # of its own last answer, word for word, with no call made. 0/6 measured on
    # every phrasing tried, and the same 0/6 with a note in the answer's place.
    check("...and nothing derived from it is left to copy", gone, [])
    # A turn that used some other tool is not a turn that looked, and keeps
    # everything: this cut is about the camera, not about tools.
    spoke = [{"role": "user", "content": "lights on"},
             {"role": "assistant", "content": "",
              "tool_calls": [{"type": "function",
                              "function": {"name": "set_lights", "arguments": {}}}]},
             {"role": "tool", "content": '{"ok": true}'},
             {"role": "assistant", "content": "They are on."}]
    mixed = spoke + history
    server._forget_pictures(mixed, keep_newest=False)
    check("an exchange that did not look is left alone", mixed, spoke)
    untouched = [{"role": "user", "content": "hello"}]
    server._forget_pictures(untouched, keep_newest=False)
    check("a conversation with no pictures is left alone",
          untouched, [{"role": "user", "content": "hello"}])

    # A refusal to see is the other thing the model reads back to itself instead
    # of looking, so it goes the same way. Real replies, taken off the wire: the
    # first four are what poisoned every question after them, and the rest are
    # ordinary answers that must survive -- a false positive here eats a turn of
    # somebody's conversation.
    refusals = [
        "I can't describe the person because I don't have the ability to read or "
        "interpret what they look like. I can only tell you where they are.",
        "I can't tell the color of the shirt because I can't see it.",
        "I checked my camera. I can't see anything right now.",
        "I don't have eyes, so I can't say what it looks like.",
    ]
    ordinary = [
        "I am the rover. I don't have a name.",
        "They are on.",
        "I can't turn the lights on because they are already on.",
        "I see a man in a red shirt sitting at a desk.",
        "I'm following one person now, who's to the right and slightly up.",
        "I can't reach that far, the camera only turns so far.",
    ]
    check("a refusal to see is recognised",
          [server._blind_refusal(r) for r in refusals], [True] * len(refusals))
    check("...and an ordinary answer is not",
          [r for r in ordinary if server._blind_refusal(r)], [])
    said_no = ([{"role": "user", "content": "describe the person"},
                {"role": "assistant", "content": refusals[0]}]
               + spoke
               + [{"role": "user", "content": "what is your name"},
                  {"role": "assistant", "content": ordinary[0]}])
    server._forget_refusals(said_no)
    check("the exchange that refused is dropped whole",
          [m["content"] for m in said_no if isinstance(m.get("content"), str)],
          ["lights on", "", '{"ok": true}', "They are on.", "what is your name",
           ordinary[0]])
    # A turn that acted is kept whatever it then said: a refusal after a `look`
    # is about a picture the rule above has already taken away.
    acted = list(spoke)
    acted[-1] = {"role": "assistant", "content": refusals[1]}
    kept_acted = list(acted)
    server._forget_refusals(kept_acted)
    check("an exchange that called a tool is left alone", kept_acted, acted)

    # The stash. Bounded two ways, because it is fed by whatever holds a camera
    # and a frame nobody claims must not become a leak.
    server._frames.clear()
    tokens = [server._stash(object()) for _ in range(server.MAX_FRAMES + 2)]
    check("the stash is bounded", len(server._frames), server.MAX_FRAMES)
    check("...dropping the oldest first", tokens[0] in server._frames, False)
    check("...and keeping the newest", tokens[-1] in server._frames, True)
    check("every frame gets its own name", len(set(tokens)), len(tokens))
    # A frame nobody claimed inside the window is gone, whether or not the stash
    # is full: the picture it holds stopped being true minutes ago.
    stale = server._stash(object())
    server._frames[stale] = (server._frames[stale][0],
                             server._frames[stale][1] - server.FRAME_TTL_S - 1)
    server._stash(object())
    check("a frame nobody claimed expires", stale in server._frames, False)
    server._frames.clear()

    try:
        import io

        from PIL import Image
    except Exception as exc:
        SKIP.append(f"frame decoding ({type(exc).__name__}: needs pillow)")
        return

    buf = io.BytesIO()
    Image.new("RGB", (1920, 1080), (20, 40, 60)).save(buf, format="JPEG")
    decoded = server._decode_frame(buf.getvalue())
    check("an oversized frame is brought down to the ceiling",
          max(decoded.size), server.VISION_MAX_SIDE)
    check("...keeping its shape", round(decoded.width / decoded.height, 2), 1.78)
    # A truncated frame is what a camera that has only just been opened gives.
    # It has to fail where it can be answered -- at the POST, which the rover can
    # retry -- rather than in the middle of somebody's sentence.
    try:
        server._decode_frame(buf.getvalue()[: len(buf.getvalue()) // 3])
        FAIL.append("a truncated frame should not decode")
    except Exception:
        PASS.append("a truncated frame is refused where the rover can hear about it")


def test_rover_client() -> None:
    """The line to the rover daemon. What the daemon does with a call is its own
    selftest's business -- rover_daemon/selftest.py, which runs on the rover."""
    import json as _json
    import socket
    import socketserver
    import threading

    try:
        import rover_tools
    except Exception as exc:
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

    # Discovery, which is where the real bug was: the rover answers on wlan0 or
    # eth0 depending on whether it is plugged in, and a client that knows only
    # one of them reports no rover while the daemon is up and serving. A dead
    # candidate must be stepped over rather than concluded from.
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

    # The name has to come first: it is the only candidate that is right whether
    # or not the rover is plugged in, and a failed name lookup is slow enough
    # that paying for one before an address that would have worked is a real cost.
    check("the rover is looked for by name first",
          rover_tools.DEFAULT_CANDIDATES[0], "rpi.local")


def test_connect_errors() -> None:
    """What the client says when the service is not there.

    This is the most likely thing to go wrong -- the card is shared, so the
    service is off more often than on -- and it used to be a fifty-line asyncio
    traceback that named neither the host nor the way to start it. Each way the
    connection can fail must arrive as one sentence about the right cause.
    """
    import asyncio
    import socket
    import threading

    try:
        import talk
    except Exception as exc:
        SKIP.append(f"connect errors ({type(exc).__name__}: needs the client venv)")
        return

    def why(url: str) -> str:
        try:
            asyncio.run(talk._open(url))
            return "connected"
        except talk.ServiceUnreachable as error:
            return str(error)
        except Exception as error:  # the failure this whole thing exists to prevent
            return f"raw {type(error).__name__}: {error}"

    check("a bad URL is explained, not raised",
          "is not a WebSocket URL" in why("http://127.0.0.1:8767/ws"), True)
    check("a name that does not resolve says so",
          "does not resolve" in why("ws://nx.invalid:8767/ws"), True)

    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        dead = probe.getsockname()[1]
    refused = why(f"ws://127.0.0.1:{dead}/ws")
    check("a refused port names the port", f"127.0.0.1:{dead}" in refused, True)
    # A service that is still loading its weights refuses exactly like one that
    # was never started, so the message has to cover both or it sends somebody
    # to restart a service that was seconds from being ready.
    check("...and allows for a service still warming up", "not bind its port" in refused, True)
    check("...and says how to start it", "switch_service.sh voice" in refused, True)

    # A listening socket nobody accepts from: the TCP handshake completes and the
    # HTTP one never does. This is the shape the failure took in practice -- a
    # host that is up with the port filtered looks the same from here.
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    silent = listener.getsockname()[1]
    original = talk.CONNECT_TIMEOUT_S
    try:
        talk.CONNECT_TIMEOUT_S = 0.3
        check("a silent port times out with an explanation",
              "did not answer" in why(f"ws://127.0.0.1:{silent}/ws"), True)
    finally:
        talk.CONNECT_TIMEOUT_S = original
        listener.close()

    # Something answering HTTP on the port is nearly always the wrong path: the
    # service serves /health and /chat beside /ws, and only /ws is a socket.
    import http.server

    class Plain(http.server.BaseHTTPRequestHandler):
        # HTTP/1.1 on purpose: websockets refuses a 1.0 response before it ever
        # looks at the status, and this is meant to exercise the status branch.
        protocol_version = "HTTP/1.1"

        def do_GET(self):
            self.send_error(404)

        def log_message(self, *args):
            pass

    http_server = http.server.HTTPServer(("127.0.0.1", 0), Plain)
    threading.Thread(target=http_server.serve_forever, daemon=True).start()
    try:
        wrong = why(f"ws://127.0.0.1:{http_server.server_address[1]}/")
        check("a plain HTTP answer points at the path", "/ws" in wrong, True)
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
        import talk
    except Exception as exc:
        SKIP.append(f"indicator ({type(exc).__name__}: needs the client venv)")
        return

    class Tty(io.StringIO):
        def isatty(self):
            return True

    def run(stream, script):
        real, sys.stdout = sys.stdout, stream
        try:
            with talk.Indicator() as indicator:
                for step in script:
                    step(indicator)
        finally:
            sys.stdout = real
        return stream.getvalue()

    blank = "\r" + " " * talk.STATUS_WIDTH + "\r"

    written = run(Tty(), [
        lambda i: i.set("listening"),
        lambda i: i.set("listening"),  # the same state again must not redraw
        lambda i: i.set("hearing"),
        lambda i: i.say("you: hello there"),
        lambda i: i.set("listening"),
    ])
    check("an unchanged state is not redrawn", written.count(talk.STATUS["listening"]), 2)
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
    except Exception as exc:
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
    except Exception as exc:
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


def main() -> int:
    test_sentences()
    test_tool_sniffer()
    test_trim()
    test_vision()
    test_rover_client()
    test_connect_errors()
    test_indicator()
    test_endpointer()
    test_speculation()

    for name in PASS:
        print(f"  ok   {name}")
    for name in SKIP:
        print(f"  skip {name}")
    for name in FAIL:
        print(f"  FAIL {name}")
    print(f"\n{len(PASS)} passed, {len(FAIL)} failed, {len(SKIP)} skipped")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
