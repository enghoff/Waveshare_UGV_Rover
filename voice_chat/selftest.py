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
            server._generate = lambda history, tools=(): iter(pieces)
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


def main() -> int:
    test_sentences()
    test_tool_sniffer()
    test_trim()
    test_rover_client()
    test_endpointer()

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
