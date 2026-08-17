"""Catching a tool call in a stream of text, and noting when it became catchable.

This is the same job `_ToolSniffer` does in [voice_chat/server.py], for the same
reason -- a tool call is text, and anything downstream will happily read it out
loud, brace by brace -- with one addition that only matters here.

The addition is the clock. Step 0's second question is whether a tool call
surfaces in the text stream *early enough to intercept* before a speech decoder
starts saying it, and that is not a yes/no about the text: it is a margin,
measured in whatever the model emits in between. So the sniffer records where the
marker appeared -- how many characters and how many chunks into the reply, and
how long after the first token -- and a caller that is also collecting audio can
compare that against when the first audio chunk arrived.

Two shapes are watched for, because which one arrives depends on the tokenizer
rather than the model: a `<tool_call>` marker anywhere, or a reply that opens with
a brace, which is what a decoder built with `skip_special_tokens=True` leaves
behind. Spoken English does neither.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field

TOOL_OPEN = "<tool_call>"
TOOL_CLOSE = "</tool_call>"


@dataclass
class Sniffer:
    """Feed it decoded text; it separates prose from calls and times the split."""

    prose: str = ""
    tail: str = ""                       # the call, and anything after it
    _pending: str = ""                   # a part-written marker, held back
    _opened: bool = False
    started: float | None = None
    marker_at_char: int | None = None    # characters of reply before the marker
    marker_at_chunk: int | None = None   # chunks of reply before the marker
    marker_at_time: float | None = None  # seconds after the first token
    chunks: int = 0
    seen: int = 0                        # characters fed, including the marker

    def feed(self, piece: str) -> str:
        """One chunk of decoded text in, the part of it that is safe to speak out."""
        now = time.monotonic()
        if self.started is None:
            self.started = now
        self.chunks += 1

        if self.tail:
            self.tail += piece
            self.seen += len(piece)
            return ""

        text = self._pending + piece
        self._pending = ""

        if not self._opened:
            head = text.lstrip()
            if not head:
                self._pending = text
                return ""
            if head.startswith("{"):
                self._mark(now, 0)
                self.tail = head
                return ""
            self._opened = True

        cut = text.find(TOOL_OPEN)
        if cut >= 0:
            self._mark(now, cut)
            self.tail = text[cut:]
            self.prose += text[:cut]
            self.seen += len(piece)
            return text[:cut]

        # Hold back a trailing fragment that could still become the marker: the
        # stream arrives in sub-word pieces, and "<tool" then "_call>" is an
        # ordinary way for one to turn up.
        for n in range(min(len(TOOL_OPEN) - 1, len(text)), 0, -1):
            if text.endswith(TOOL_OPEN[:n]):
                self._pending = text[-n:]
                out = text[:-n]
                self.prose += out
                self.seen += len(piece)
                return out

        self.prose += text
        self.seen += len(piece)
        return text

    def _mark(self, now: float, offset: int) -> None:
        if self.marker_at_char is not None:
            return
        self.marker_at_char = len(self.prose) + offset
        self.marker_at_chunk = self.chunks
        self.marker_at_time = now - (self.started or now)

    def flush(self) -> str:
        """Whatever was held back and turned out to be ordinary text."""
        text, self._pending = self._pending, ""
        self.prose += text
        return text

    @property
    def calls(self) -> list[dict]:
        return parse_calls(self.tail)


def parse_calls(text: str) -> list[dict]:
    """Every well-formed call in a swallowed block.

    The deployed service honours only the first, because on a real rover a second
    call is a second physical act. Here they are all returned, because a model
    that asks for two tools when one was requested has told us something and the
    scoring wants to see it rather than have it hidden.
    """
    calls = []
    body = text.strip()
    if body.startswith("{") and TOOL_OPEN not in body:
        pieces = [body]
    else:
        pieces = []
        rest = body
        while (start := rest.find(TOOL_OPEN)) >= 0:
            rest = rest[start + len(TOOL_OPEN):]
            end = rest.find(TOOL_CLOSE)
            pieces.append(rest[:end] if end >= 0 else rest)
            if end < 0:
                break
            rest = rest[end + len(TOOL_CLOSE):]

    for piece in pieces:
        call = _one(piece)
        if call:
            calls.append(call)
    return calls


def _one(body: str) -> dict | None:
    body = body.strip()
    # A model that keeps writing after the closing brace is common enough that
    # decoding the first complete object is worth more than being strict.
    try:
        call = json.loads(body)
    except ValueError:
        try:
            call, _ = json.JSONDecoder().raw_decode(body)
        except ValueError:
            return None
    if not isinstance(call, dict):
        return None
    name = call.get("name")
    arguments = call.get("arguments", {})
    if isinstance(arguments, str):  # some templates emit args as a JSON string
        try:
            arguments = json.loads(arguments)
        except ValueError:
            return None
    if not isinstance(name, str) or not isinstance(arguments, dict):
        return None
    return {"name": name, "arguments": arguments}


if __name__ == "__main__":
    # The shapes that actually turn up, including the sub-word split that this
    # exists to survive.
    cases = [
        ["I'll get that. ", "<tool", "_call>", '{"name": "set_lights", ', '"arguments": {"level": 255}}', "</tool_call>"],
        ['{"name": "look", "arguments": {}}'],
        ["Sure thing. ", '<tool_call>{"name": "set_lights", "arguments": "{\\"level\\": 0}"}</tool_call>'],
        ["My name is Rover, and I do not have one otherwise."],
    ]
    for pieces in cases:
        sniffer = Sniffer()
        spoken = "".join(sniffer.feed(p) for p in pieces) + sniffer.flush()
        print(f"spoken={spoken!r:<50} calls={sniffer.calls} at char {sniffer.marker_at_char}")
