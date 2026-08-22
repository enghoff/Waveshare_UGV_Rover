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

import io
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
        # A picture is a user message, so cutting at every role=user splits a
        # look in half: the question is dropped and the model answers the
        # caption. Same grouping _exchanges already uses, for the same reason.
        picture = object()
        look_turn = [
            {"role": "user", "content": "what's in the room"},
            {"role": "assistant", "content": "",
             "tool_calls": [{"type": "function",
                             "function": {"name": "look", "arguments": {}}}]},
            {"role": "tool", "content": '{"ok": true}'},
            {"role": "user", "content": [{"type": "image", "image": picture},
                                         {"type": "text", "text": "what's in the room"}]},
        ]
        cut = server._trim(exchange * 6 + look_turn)
        check("a look is not split off from the question that took it",
              any(m.get("content") == "what's in the room" for m in cut
                  if isinstance(m.get("content"), str)),
              True)
        check("...and the picture stays with that question",
              server._images(cut), [picture])
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

    check("the question sits on the picture",
          server._image_message(picture, "what's in the room")["content"][1]["text"],
          "what's in the room")

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

    # A promise is the same law a third time. All four of these were said by the
    # rover with no call behind them, and one of them was enough to take a 6/6
    # request to 0/6 for the rest of the conversation.
    promises = [
        "I will turn the lights on.",
        "I'll start tracking you as you move. I'll keep you centered in my view.",
        "I am going to switch the lights off for you.",
        "I am starting to follow the person in front of me.",
        "I will check the lights status.",
    ]
    innocent = [
        "I am the rover. I don't have a name.",
        "I am not sure what you mean by late time. Please say that again.",
        "I don't have a sense of time. I can't tell if it's late or not.",
        # A promise about nothing a tool does is a promise this must not eat.
        "I'll be here when you get back.",
        # Said *after* a call, which is the case the exchange rule protects.
        "I turned the lights on.",
        "The lights are on.",
    ]
    check("a promise to act is recognised",
          [server._promised(p) for p in promises], [True] * len(promises))
    check("...and an ordinary answer is not",
          [r for r in innocent if server._promised(r)], [])
    promised = ([{"role": "user", "content": "can each other lights on"},
                 {"role": "assistant", "content": promises[0]}]
                + spoke
                + [{"role": "user", "content": "are the lights on"},
                   {"role": "assistant", "content": innocent[-1]}])
    server._forget_promises(promised)
    check("the exchange that promised is dropped whole",
          [m["content"] for m in promised if isinstance(m.get("content"), str)],
          ["lights on", "", '{"ok": true}', "They are on.", "are the lights on",
           innocent[-1]])
    # "I'll keep following him" is honest when the call is in the same exchange.
    kept_promise = list(spoke)
    kept_promise[-1] = {"role": "assistant", "content": promises[1]}
    unchanged = list(kept_promise)
    server._forget_promises(kept_promise)
    check("a promise beside its own tool call is left alone",
          kept_promise, unchanged)

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

        # And remaking it must not send the client back to the name. `rpi.local`
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

    # A remembered address is not a hardcoded one. The rover answers on eth0 while
    # it is docked and on wlan0 once it has driven off, so an address that stops
    # answering is exactly how a client finds out it has moved, and it has to ask
    # the name again rather than go on dialling where the rover used to be. That is
    # the bug docs/hosts.md is about; remembering an address without this would be
    # a fresh way of writing it.
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
    except Exception as exc:
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


def test_prompts() -> None:
    """The prompt and the schemas are read from the source, not copied into it."""
    try:
        import prompts
    except Exception as exc:
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
    except Exception as exc:
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
    except Exception as exc:
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
    except Exception as exc:
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
    except Exception as exc:
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
    except Exception as exc:
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
    except Exception as exc:
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


def test_choosing_a_network() -> None:
    """Which access point the rover is on, and moving it to another.

    Over a real socket rather than against the class, because the shape of these
    two answers is the whole contract between the daemon and the console's network
    panel, and the mock is the only place that shape can be checked without a Pi
    and three routers.

    The unscanned case is the one worth pinning down. Nothing polls for a scan --
    it takes the dongle off channel for seconds, on a bus it shares with the camera
    -- so what a console normally sees is a list of one, and a panel that treated
    that as "there is nothing else out there" would be wrong in the ordinary case
    rather than the rare one.
    """
    try:
        import mock_rover
        import rover_tools
    except Exception as exc:
        SKIP.append(f"choosing a network ({type(exc).__name__})")
        return

    rover = mock_rover.Rover(None, None)
    server = mock_rover.serve(rover, "127.0.0.1", 0, quiet=True)
    client = rover_tools.RoverClient(f"127.0.0.1:{server.server_address[1]}")
    try:
        quiet = client.call("wifi_status", {})
        check("it says which network it is on",
              quiet.get("connected"), "TheGreatLord")
        check("...with a signal from the driver, in dBm",
              -90 <= quiet.get("level_dbm", 0) <= -20, True)
        check("...and an address, since being associated is not being online",
              quiet.get("address"), "192.168.1.47")
        check("without a scan the list is only what was last heard",
              [n["ssid"] for n in quiet["networks"]], ["TheGreatLord"])
        check("...and that row is marked as the one in use",
              quiet["networks"][0]["in_use"], True)

        looked = client.call("wifi_status", {"scan": True})
        check("a scan finds the neighbours", len(looked["networks"]) > 1, True)
        check("...and says which of them this rover has a passphrase for",
              [n["ssid"] for n in looked["networks"] if n["configured"]],
              ["TheGreatLord", "TheMaharaja", "TheGreatViking"])

        refused = client.call("wifi_join", {"ssid": "Alister"})
        check("a network with no passphrase is refused", refused.get("ok"), False)
        check("...by name, so the panel can say why",
              "no passphrase for Alister" in refused.get("error", ""), True)
        check("and so is a join with no network at all",
              client.call("wifi_join", {}).get("ok"), False)

        moved = client.call("wifi_join", {"ssid": "TheMaharaja"})
        check("a configured network is accepted", moved.get("joining"), "TheMaharaja")
        check("...and the answer warns that the link is about to go",
              "drop" in moved.get("note", ""), True)
        after = client.call("wifi_status", {})
        check("...and afterwards it is on it", after.get("connected"), "TheMaharaja")
        check("...with the outcome kept for whoever reconnects",
              after.get("last_join", {}).get("ok"), True)
    finally:
        client.close()
        server.shutdown()
        server.server_close()


def test_signal_verdict() -> None:
    """One word for a dBm reading, which is what gets the colour in the panel."""
    try:
        import console_model
    except Exception as exc:
        SKIP.append(f"signal verdict ({type(exc).__name__})")
        return

    verdict = console_model.wifi_verdict
    check("a strong link", verdict(-41), "good")
    check("a fading one", verdict(-68), "fair")
    check("one the wifi keeper is about to act on", verdict(-77), "poor")
    # No reading at all is the interface not reporting a signal, which is not good
    # news and must not be coloured as though it were.
    check("and no reading at all", verdict(None), "poor")


def test_map_size_for_a_panel() -> None:
    """Which map to ask the rover for once the browser has said how wide its panel
    turned out to be.

    Rounded *down* the ladder, and that is not a detail: the picture costs the Pi
    roughly its own area to draw, so a panel a few pixels over a rung must not buy
    the rung above it. Everything the browser gains by asking for more it throws
    away again scaling the picture back into the panel.
    """
    try:
        from console_model import size_for_panel
    except Exception as exc:
        SKIP.append(f"map size for a panel ({type(exc).__name__})")
        return

    check("a panel exactly on a rung takes that rung", size_for_panel(480), 480)
    check("...and one just over it does not take the next", size_for_panel(639), 480)
    check("...until it reaches it", size_for_panel(640), 640)
    # A phone in one column, or a window dragged narrow. There is no rung below the
    # smallest, and asking for nothing is not an option.
    check("a panel narrower than any rung takes the smallest", size_for_panel(210), 320)
    check("and a very wide one stops at the largest", size_for_panel(4000), 800)


def test_web_console() -> None:
    """The browser console's model, with no browser and no rover.

    Everything the page draws comes out of `Session`, so these are the panels
    themselves: the alarms that make a silent lidar unmissable, which networks are
    offered a join button, and how the map's two ladders answer a resized window.
    None of it needs a socket -- `Session` connects when its pump runs, and the pump
    is not started here.
    """
    try:
        import drive_web
    except Exception as exc:
        SKIP.append(f"web console ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)

    # The status panel. The formatting lives in console_model.py; what is tested
    # here is that the alarm flag reaches the page, because a lidar that has
    # gone silent under motor load makes every other number on that panel a lie.
    session.show_status({"ok": True, "lidar_live": False, "lidar_ok": False,
                         "estop": False, "position_trusted": True, "speed_ms": 0.0,
                         "pose": {"x_m": 1.0, "y_m": -0.5, "heading_deg": 90.0}})
    rows = dict((row[0], row) for row in session.status_rows)
    check("a silent lidar says so in capitals", rows["lidar"][1], "SILENT")
    check("...and is flagged so the page can colour it", rows["lidar"][2], True)
    check("a stale match is not an alarm on its own", rows["matched"][2], False)
    check("and the pose reads as a place",
          session.pose_text, "x +1.00  y -0.50  +90.0 deg")

    # A status the rover could not answer must blank the numbers rather than leave
    # the last good ones on screen looking current.
    session.show_status({"ok": False, "error": "no navigator"})
    check("a refused status blanks the rows",
          set(row[1] for row in session.status_rows), {"-"})
    check("...and says why", session.status_error, "no navigator")

    # The network list. Joinable means configured and not the one already in use --
    # a network the rover holds no passphrase for is worth seeing in the list and is
    # not worth a button.
    session.show_wifi({"ok": True, "connected": "Sonic", "level_dbm": -42,
                       "address": "192.168.1.47", "networks": [
                           {"ssid": "Sonic", "signal": 80, "in_use": True,
                            "configured": True},
                           {"ssid": "Sonic5", "signal": 61, "in_use": False,
                            "configured": True},
                           {"ssid": "next door", "signal": 44, "in_use": False,
                            "configured": False}]})
    offered = [n["ssid"] for n in session.wifi["networks"] if n["joinable"]]
    check("only a network it has a passphrase for is offered", offered, ["Sonic5"])
    check("the one it is on is named as such",
          session.wifi["networks"][0]["note"], "on it")
    check("and the strong link is coloured as one", session.wifi["verdict"], "good")

    # An older daemon has none of these calls. Say so once and stop asking, rather
    # than painting the panel red every five seconds for the rest of the session.
    quiet = drive_web.Session(None, 3.0, 480)
    quiet.show_wifi({"ok": False, "error": "no such tool: wifi_status"})
    check("a daemon too old for the network calls is asked once",
          quiet.wifi_ok, False)

    # Stepping the size by hand has to turn "fit the panel" off, or the next window
    # resize would silently undo the press.
    session.map_settings({"fit": True})
    session.panel_px = 700.0
    session.fit_map()
    check("fitting the panel picks the rung below its width", session.map_size, 640)
    session.map_settings({"size": -1})
    check("...and pressing smaller steps down from there", session.map_size, 480)
    check("...and stops the panel choosing", session.map_fit, False)

    # The map is square and the daemon says how big it came out, but a mock or an
    # older daemon may not -- and the page sets the panel's aspect ratio from this
    # number, so a wrong one puts a click somewhere else in the room.
    header = b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR" + (484).to_bytes(4, "big")
    check("the size can always be read off the picture itself",
          drive_web._png_width(header), 484)


def test_stopping_an_unwatched_rover() -> None:
    """The browser console's answer to a tab being closed mid-move.

    A desktop window sends a stop from its close handler. A browser tab that goes
    away says nothing at all and the server outlives it, so the promise is kept from
    the server's side instead: the event stream is the browser being present, and
    losing the last one while a move is running is treated as closing the window.

    The grace is the part worth testing. A reload tears the stream down and puts it
    back inside a few hundred milliseconds, and a console that stopped the rover for
    that would be unreloadable during the only minute it is interesting.
    """
    try:
        import drive_web
    except Exception as exc:
        SKIP.append(f"stopping an unwatched rover ({type(exc).__name__})")
        return

    class Fake:
        """A channel that records rather than connects."""

        def __init__(self):
            self.sent = []

        def submit(self, name, arguments=None):
            self.sent.append(name)

    session = drive_web.Session(None, 3.0, 480)
    session.halt = Fake()
    session.busy_since = 100.0
    session.busy_name = "drive"

    # Somebody is looking, so nothing happens however long the move runs.
    session.listeners = 1
    session.mind_the_watchers(100.0)
    session.mind_the_watchers(200.0)
    check("a watched move is left alone", session.halt.sent, [])

    # The stream goes. Inside the grace this is a reload, not a departure.
    session.listeners = 0
    session.mind_the_watchers(200.0)
    session.mind_the_watchers(200.0 + drive_web.ORPHAN_GRACE_S / 2)
    check("a reload does not stop the rover", session.halt.sent, [])

    session.mind_the_watchers(200.0 + drive_web.ORPHAN_GRACE_S + 0.1)
    check("but a closed tab does", session.halt.sent, ["stop_driving"])
    # Once, not once per tick: the pump runs ten times a second, and a stop resent
    # ten times a second would bury the transcript meant to explain it.
    session.mind_the_watchers(260.0)
    check("...and only once", session.halt.sent, ["stop_driving"])

    # A rover doing nothing is not stopped for being unwatched. There is nothing to
    # stop, and the line it would write in the transcript would be a lie.
    idle = drive_web.Session(None, 3.0, 480)
    idle.halt = Fake()
    idle.mind_the_watchers(300.0)
    idle.mind_the_watchers(400.0)
    check("an idle rover is not stopped for being alone", idle.halt.sent, [])


def test_finding_the_rover_again() -> None:
    """A console that has lost its rover goes looking, without being asked.

    The button is the thing being tested away. A rover on wifi that has driven
    behind the boiler, or been power-cycled, or come back on another address, used
    to leave the page reading "no daemon answered" until somebody noticed and
    clicked -- which is the wrong thing to require at the moment the stop button has
    stopped working. So the clock and the decision are both here, driven with an
    explicit `now` rather than a sleep.
    """
    try:
        import drive_web
    except Exception as exc:
        SKIP.append(f"finding the rover again ({type(exc).__name__})")
        return

    def recording(session):
        """A session whose `connect` records instead of opening sockets."""
        session.tried = []
        session.connect = lambda: session.tried.append("connect")
        return session

    # --- a link that is up ---------------------------------------------------
    live = recording(drive_web.Session(None, 3.0, 480))
    live.channels = ["a channel"]          # only its emptiness is ever read
    live.answered_at = 100.0
    live.mind_the_link(100.0 + drive_web.LINK_LOST_S / 2)
    check("a rover that is answering is left alone", live.tried, [])

    live.mind_the_link(100.0 + drive_web.LINK_LOST_S + 0.1)
    check("one that has gone quiet is reconnected", live.tried, ["connect"])

    # A move in flight owns the link: the move channel waits longer than this does,
    # and pulling the connections out from under it would throw away the one reply
    # that says what the rover did.
    driving = recording(drive_web.Session(None, 3.0, 480))
    driving.channels = ["a channel"]
    driving.answered_at = 100.0
    driving.busy_since = 100.0
    driving.mind_the_link(200.0)
    check("a move in flight is not interrupted to reconnect", driving.tried, [])

    # A join takes the rover off this network on purpose, and `rejoined` is already
    # scheduled to pick the pieces up.
    joining = recording(drive_web.Session(None, 3.0, 480))
    joining.channels = ["a channel"]
    joining.answered_at = 100.0
    joining.wifi_joining = "upstairs"
    joining.mind_the_link(200.0)
    check("a network join is left to finish", joining.tried, [])

    # --- a link that is down -------------------------------------------------
    down = recording(drive_web.Session(None, 3.0, 480))
    down.find_at = 100.0
    down.find_tries = 1
    down.mind_the_link(100.0 + drive_web.RECONNECT_S / 2)
    check("a search is not repeated the instant it fails", down.tried, [])
    down.mind_the_link(100.0 + drive_web.RECONNECT_S + 0.1)
    check("...and is repeated once the wait is up", down.tried, ["connect"])

    # One at a time. The search runs on a thread and takes seconds on a name that
    # does not resolve; a retry per tick would be ten threads a second.
    flying = recording(drive_web.Session(None, 3.0, 480))
    flying.find_at = 100.0
    flying.find_outstanding = True
    flying.mind_the_link(500.0)
    check("a search already running is not started again", flying.tried, [])

    # Backing off, and stopping backing off. A rover switched off for the evening
    # should not be dialled every two seconds all night, and one switched off for a
    # moment should not take a minute to be noticed.
    waits = [recording(drive_web.Session(None, 3.0, 480)) for _ in range(3)]
    for tries, session in zip((0, 1, 50), waits):
        session.find_tries = tries
    check("the first wait is the short one",
          waits[0].retry_in(), drive_web.RECONNECT_S)
    check("...and so is the wait after one failure",
          waits[1].retry_in(), drive_web.RECONNECT_S)
    check("...and it never grows past the ceiling",
          waits[2].retry_in(), drive_web.RECONNECT_MAX_S)

    # --- what the log and the link line say ----------------------------------
    talking = drive_web.Session("rpi.local:8769", 3.0, 480)
    talking.connected = lambda address: None          # no sockets in a selftest
    for _ in range(3):
        talking.handle(drive_web.Reply(
            "__found__", {}, {"ok": False, "address": None}, 0.0))
    said = [line["text"] for line in talking.log]
    check("a rover that is not there is reported once, not once a try",
          sum("no rover daemon answered" in text for text in said), 1)
    check("...and the line says it is still looking",
          "keep looking" in " ".join(said), True)
    check("...as does the link", "looking again" in talking.link_text, True)
    check("...and the tries were counted", talking.find_tries, 3)

    talking.handle(drive_web.Reply(
        "__found__", {}, {"ok": True, "address": "rpi.local:8769"}, 0.0))
    check("coming back is worth a line", any("answered again" in line["text"]
                                             for line in talking.log), True)
    check("...and the count starts over", talking.find_tries, 0)


def test_a_browser_leaving() -> None:
    """A closed tab is not an error, and everything else still is.

    `socketserver` prints a full traceback for anything that reaches it out of a
    handler, and a browser closing a kept-alive connection reaches it as one --
    `ConnectionAbortedError [WinError 10053]` from the read of the next request
    line. Every reload printed twenty lines about it. That is worth a test rather
    than a comment because the fix is a suppression, and a suppression that grows
    to cover a real fault is how a console stops reporting the thing it is for.
    """
    try:
        import drive_web
    except Exception as exc:
        SKIP.append(f"a browser leaving ({type(exc).__name__})")
        return

    def printed(error):
        """What the server would write to stderr while `error` is being handled."""
        caught = io.StringIO()
        was, sys.stderr = sys.stderr, caught
        try:
            try:
                raise error
            except type(error):
                # An instance without its __init__, so no socket is bound to ask
                # the question of -- the whole decision is which exception is in
                # flight, and `super()` inside it needs a real instance to reach
                # the printing it falls back to.
                server = drive_web.Console.__new__(drive_web.Console)
                server.handle_error(None, ("127.0.0.1", 1))
        except Exception as exc:        # the real handler's own failure, if any
            caught.write(f"handle_error raised {type(exc).__name__}")
        finally:
            sys.stderr = was
        return caught.getvalue()

    for error in (ConnectionAbortedError(10053, "aborted"),
                  ConnectionResetError(10054, "reset"),
                  BrokenPipeError(32, "broken pipe"),
                  TimeoutError("the handler's idle timeout")):
        check(f"{type(error).__name__} is a tab closing, not an error",
              printed(error), "")

    # And the other half, which is the half that matters: a genuine fault in a
    # handler still lands in the window somebody is watching.
    shouted = printed(ValueError("the map arrived as a duck"))
    check("a real fault is still printed", "ValueError" in shouted, True)
    check("...with the traceback that says where it came from",
          "Traceback" in shouted, True)


def test_one_console_at_a_time() -> None:
    """A second drive console must not start, on any port.

    Two consoles are not two windows onto one rover, they are two clients of it:
    each polls three times a second and each asks for a map that costs the Pi's
    single core two and a half seconds to draw. Measured with three attached, the
    daemon sat at 48% of the core drawing maps for windows nobody was looking at.
    Worse on Windows, where `SO_REUSEADDR` means *share* rather than *reclaim*, so
    the second one binds the same port happily and the browser is served its page by
    one console while its buttons post to the other -- which reads as a rover that
    has stopped listening and a map from some earlier session.
    """
    try:
        import drive_web
    except Exception as exc:
        SKIP.append(f"one console at a time ({type(exc).__name__})")
        return

    import tempfile

    path = os.path.join(tempfile.mkdtemp(prefix="rover-lock-"), "console.lock")
    first, second = drive_web.OnlyOne(path), drive_web.OnlyOne(path)
    try:
        check("the first console gets the lock", first.claim(), "")
        refused = second.claim()
        check("...and the second is refused", bool(refused), True)
        check("...and told which process to close",
              str(os.getpid()) in refused, True)

        # A lock the kernel holds, not a file somebody has to remember to delete:
        # the console that matters here is the one that died without tidying up, and
        # a stale lock nobody can clear is a console nobody can run.
        first.release()
        check("once the first goes, the next one starts", second.claim(), "")
    finally:
        first.release()
        second.release()

    # The port guard is the other half, and it is a property of the class rather
    # than of a running server: on Windows the default would let two consoles share
    # one port without either of them finding out.
    check("the server refuses to share its port",
          drive_web.Console.allow_reuse_address, False)


def test_pictures_are_not_replayed() -> None:
    """Two consoles must never publish a picture at the same URL.

    They did, and it was the worst-looking bug in this thing. Each map is served at
    `/map.png?gen=N` with N counting from 1 and a year of `immutable` on it, and N
    starts again at 1 in every new process -- so the second console handed a browser
    exactly the URLs the first had already filled its cache with, in the same order.
    The browser never asked about them again and drew the earlier run's pictures back
    frame by frame, over a live rover, the same run every time. Restarting did not
    help and neither did rebooting: the pictures were on disk in the browser profile.

    Reproduced by pointing one console at the mock rover and the next at the real one
    and logging what the server was asked for: the second console served a different
    picture at that URL and the browser fetched it zero times. So the guard is that
    the name of a picture belongs to the run that drew it.
    """
    try:
        import drive_web
    except Exception as exc:
        SKIP.append(f"pictures are not replayed ({type(exc).__name__})")
        return

    one = drive_web.Session(None, 3.0, 480)
    two = drive_web.Session(None, 3.0, 480)

    check("a console with no picture yet publishes no name", one.tag(0), "")
    check("...which is what the page reads as nothing to show", bool(one.tag(0)),
          False)
    check("the first picture of a run is named", bool(one.tag(1)), True)
    check("...and pictures within one run differ", one.tag(1) != one.tag(2), True)

    # The whole point: same counter, different run, different URL.
    check("two consoles do not name their first picture the same",
          one.tag(1) != two.tag(1), True)
    check("...nor their tenth", one.tag(10) != two.tag(10), True)

    # And the header that made it permanent is only honest once that holds.
    check("a picture is still cacheable for a year",
          "immutable" in io.open(drive_web.__file__, encoding="utf-8").read(), True)


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
    test_prompts()
    test_frames()
    test_speaker()
    test_echo_guard()
    test_pointing_the_camera()
    test_move_commentary()
    test_choosing_a_network()
    test_signal_verdict()
    test_map_size_for_a_panel()
    test_web_console()
    test_stopping_an_unwatched_rover()
    test_finding_the_rover_again()
    test_a_browser_leaving()
    test_one_console_at_a_time()
    test_pictures_are_not_replayed()
    test_talk_session()

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
