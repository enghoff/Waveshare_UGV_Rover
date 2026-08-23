"""Sentence splitting and tool-call sniffing for the voice service."""
from __future__ import annotations

import json
import os
import re
from typing import Any, Iterator

# A sentence ends at .?!… or a newline, but only when the next character is
# whitespace or end-of-string -- otherwise "3.5 metres" and "192.168.1.4" split
# mid-number and Kokoro reads the fragments with falling intonation.
_SENTENCE_END = re.compile(r"(?<=[.!?…\n])(?=\s|$)")
# Do not hand Kokoro a two-word fragment just because it ended in a period; the
# prosody of a very short clause spoken alone is noticeably wrong. Below this,
# keep buffering.
_MIN_SENTENCE = int(os.environ.get("VOICE_MIN_SENTENCE", "12"))

# A clause ends at , ; or : followed by whitespace. Requiring the whitespace is
# what keeps "1,234" and "at 3:30" whole, the same trick _SENTENCE_END uses.
_CLAUSE_END = re.compile(r"(?<=[,;:])(?=\s)")
# Only the *first* chunk of a reply is allowed to break at a clause, and only
# above this length. The reasoning is asymmetric on purpose: the first chunk is
# the only one that gates first-audio, because every later one is spoken while
# the model is still decoding and is already waiting on the speaker, not the
# card. So splitting later chunks early buys no latency and costs prosody --
# Kokoro reads a comma-terminated fragment with the wrong intonation, and doing
# that throughout a reply is audible. Doing it once, at the head, is not.
_MIN_FIRST = int(os.environ.get("VOICE_MIN_FIRST", "24"))
FIRST_CLAUSE = os.environ.get("VOICE_FIRST_CLAUSE", "1") not in ("0", "false", "")


def _split_once(buf: str, first: bool) -> tuple[str | None, str]:
    """Take one speakable chunk off the front of `buf`, or report there is none.

    Returns `(head, rest)`, with `head` None when nothing may be spoken yet.
    """
    parts = _SENTENCE_END.split(buf, maxsplit=1)
    if len(parts) == 2 and len(parts[0].strip()) >= _MIN_SENTENCE:
        return parts[0].strip(), parts[1]
    # A sentence end that was too short to speak alone falls through to here
    # rather than ending the search: on the first chunk a later comma may still
    # give something long enough, and "Yes. I can see a chair," is both speakable
    # and half a second earlier than waiting for the sentence after it.
    if first and FIRST_CLAUSE:
        parts = _CLAUSE_END.split(buf, maxsplit=1)
        if len(parts) == 2 and len(parts[0].strip()) >= _MIN_FIRST:
            return parts[0].strip(), parts[1]
    return None, buf


def _sentences(stream: Iterator[str]) -> Iterator[str]:
    """Regroup a token stream into speakable sentences as they complete."""
    buf = ""
    first = True
    for piece in stream:
        buf += piece
        while True:
            head, buf = _split_once(buf, first)
            if head is None:
                break
            yield head
            first = False
    if buf.strip():
        yield buf.strip()


_TOOL_OPEN = "<tool_call>"
_TOOL_CLOSE = "</tool_call>"


class _ToolSniffer:
    """Passes prose through; swallows a tool call and everything after it.

    This sits between the token stream and the sentence splitter for one
    reason: a tool call is *text*, and the splitter would hand it to Kokoro,
    which would read the JSON out loud, brace by brace. Nothing may be spoken
    until it is known not to be a call.

    It watches for two shapes, because which one arrives depends on the
    tokenizer rather than on the model. Qwen wraps a call in `<tool_call>`
    markers, but those markers are added tokens, and a streamer built with
    `skip_special_tokens=True` may well eat them before this ever sees them --
    leaving a bare JSON object as the whole reply. So: a `<tool_call>` marker
    anywhere, *or* a reply that opens with a brace. Spoken English does neither.
    """

    def __init__(self) -> None:
        self.tail = ""  # the call, and anything the model wrote after it
        self._pending = ""  # a part-written marker, held back until it can be judged
        self._opened = False  # has the reply's first real character been seen?

    def feed(self, piece: str) -> str:
        """One chunk of decoded text in, the part of it that is prose out."""
        if self.tail:
            self.tail += piece
            return ""
        text = self._pending + piece
        self._pending = ""

        if not self._opened:
            head = text.lstrip()
            if not head:
                self._pending = text  # nothing but whitespace so far
                return ""
            if head.startswith("{"):
                self.tail = head
                return ""
            self._opened = True

        cut = text.find(_TOOL_OPEN)
        if cut >= 0:
            self.tail = text[cut:]
            return text[:cut]
        # Hold back a trailing fragment that could still become the marker --
        # the stream arrives in sub-word pieces, so "<tool" and "_call>" is a
        # perfectly ordinary way for one to turn up.
        for n in range(min(len(_TOOL_OPEN) - 1, len(text)), 0, -1):
            if text.endswith(_TOOL_OPEN[:n]):
                self._pending = text[-n:]
                return text[:-n]
        return text

    def flush(self) -> str:
        """Whatever was held back and turned out to be ordinary text."""
        text, self._pending = self._pending, ""
        return text


def _parse_tool_call(text: str) -> dict[str, Any] | None:
    """The first call in a swallowed block, or None if it will not parse."""
    body = text.strip()
    if body.startswith(_TOOL_OPEN):
        body = body[len(_TOOL_OPEN):]
    body = body.split(_TOOL_CLOSE, 1)[0].strip()
    # A second call in the same reply is dropped rather than queued: the tools
    # here are cheap and idempotent, and honouring only the first keeps the turn
    # to one round trip.
    body = body.split(_TOOL_OPEN, 1)[0].strip()
    try:
        call = json.loads(body)
    except ValueError:
        return None
    if not isinstance(call, dict):
        return None
    name = call.get("name")
    arguments = call.get("arguments", {})
    # Some templates emit the arguments as a JSON *string* rather than an
    # object. Both are common enough to accept.
    if isinstance(arguments, str):
        try:
            arguments = json.loads(arguments)
        except ValueError:
            return None
    if not isinstance(name, str) or not isinstance(arguments, dict):
        return None
    return {"name": name, "arguments": arguments}

