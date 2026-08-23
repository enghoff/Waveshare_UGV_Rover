"""MEDIA-side checks: sentence splitting, sniffer, trim, vision."""
from __future__ import annotations

import io

from test_harness import FAIL, PASS, SKIP, check


def test_sentences() -> None:
    """The splitter decides what Kokoro is handed, one clause at a time."""
    try:
        from server import _sentences
    except ImportError as exc:
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
    except ImportError as exc:
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
    except ImportError as exc:
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
    except ImportError as exc:
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

