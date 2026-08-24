#!/usr/bin/env python3
"""WebSocket framing, hand-rolled, because the console is `http.server`.

The drive console is a `ThreadingHTTPServer` from the standard library and its
one dependency rule has held since it replaced a tkinter window: the page, the
stream and the two pictures are all stdlib. A microphone breaks that, because
audio wants a socket that stays open and pushes in both directions, and the
event stream this console already has only goes one way.

**The alternative was a second port, and that costs a second certificate
exception.** A browser's trust decision is per origin, and an origin is scheme,
host *and port* -- so an audio socket on 8774 would be a second warning to click
through, on a page whose whole reason for existing is that the first one bought a
microphone. Speaking WebSocket on the port the console is already on avoids that
entirely, and what it costs is this file.

Only what that needs, which is less than the protocol has: text and binary
messages, close, ping and pong. No extensions, no compression, and no
continuation frames going *out* -- audio is sent as whole frames because a whole
frame is what it is. Continuations coming *in* are read, because a browser is
entitled to send them and the page's own audio blocks are small enough that it
never will.

    key = accept(request_key)          # the one header the handshake turns on
    op, data = read_message(rfile)     # blocking, on the connection's thread
    send(wfile, lock, TEXT, b"...")

`read_message` returns control frames alongside data ones rather than handling
them here: a ping has to be answered on the writing side, which has a lock on it,
and hiding that inside the reader would mean this file knowing about both.
"""
from __future__ import annotations

import base64
import hashlib
import os
import struct

#: RFC 6455's constant, appended to the client's key before hashing. It exists so
#: that a cache or a proxy cannot be talked into completing a handshake by
#: replaying something that merely looks like one.
GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

CONT, TEXT, BINARY = 0x0, 0x1, 0x2
CLOSE, PING, PONG = 0x8, 0x9, 0xA

#: Anything larger than this from a browser is a bug or an attack, and either way
#: the connection is better closed than trusted. Audio arrives in blocks of a few
#: thousand bytes; the largest legitimate message here is a JSON status line.
MAX_MESSAGE = 1 << 20


class ProtocolError(Exception):
    """The peer sent something this cannot be expected to carry on from."""


def accept(key: str) -> str:
    """The `Sec-WebSocket-Accept` value for a client's `Sec-WebSocket-Key`."""
    digest = hashlib.sha1((key + GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def frame(opcode: int, payload: bytes = b"", mask: bool = False) -> bytes:
    """One complete frame, FIN set.

    Masking is the client's job and never the server's, so `mask` is here for the
    tests rather than for the console -- a server that masks its frames is one a
    browser closes on with a protocol error.
    """
    length = len(payload)
    head = bytes([0x80 | opcode])
    flag = 0x80 if mask else 0x00
    if length < 126:
        head += bytes([flag | length])
    elif length < (1 << 16):
        head += bytes([flag | 126]) + struct.pack(">H", length)
    else:
        head += bytes([flag | 127]) + struct.pack(">Q", length)
    if not mask:
        return head + payload
    key = os.urandom(4)
    return head + key + _xor(payload, key)


def _xor(payload: bytes, key: bytes) -> bytes:
    return bytes(byte ^ key[i % 4] for i, byte in enumerate(payload))


def send(wfile, lock, opcode: int, payload: bytes = b"") -> None:
    """Write one frame, under the lock that keeps two threads from interleaving.

    Both halves of the conversation write here: the browser's own thread answers
    pings, and the session's thread pushes the model's audio down the same
    socket. Two writers on one socket without a lock is not a race that shows up
    as an error -- it shows up as a frame with another frame inside it, and the
    browser closing the connection without saying why.
    """
    with lock:
        wfile.write(frame(opcode, payload))
        wfile.flush()


def _exactly(rfile, count: int) -> bytes:
    """`count` bytes or an end of file, never a short read.

    `rfile.read(n)` on a socket is allowed to come back with less than asked for,
    and a frame header read short is a frame header read wrong -- which then
    desynchronises every frame after it. This is the whole reason the reads here
    are not just `rfile.read`.
    """
    out = b""
    while len(out) < count:
        block = rfile.read(count - len(out))
        if not block:
            raise ConnectionError("the socket closed mid-frame")
        out += block
    return out


def read_frame(rfile, from_client: bool = True) -> tuple[bool, int, bytes]:
    """One frame: whether it is final, its opcode, and its unmasked payload."""
    first, second = _exactly(rfile, 2)
    fin = bool(first & 0x80)
    if first & 0x70:
        raise ProtocolError("a reserved bit was set, and no extension was agreed")
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack(">H", _exactly(rfile, 2))[0]
    elif length == 127:
        length = struct.unpack(">Q", _exactly(rfile, 8))[0]
    if length > MAX_MESSAGE:
        raise ProtocolError(f"a {length} byte frame is larger than anything here sends")
    # A browser always masks and the standard says a server must close on one
    # that does not; a server never masks and a client must close on one that
    # does. Worth enforcing both ways rather than tolerating: masking that is on
    # the wrong side means whatever is at the other end is not what it says it
    # is. `from_client` is which side of that this read is on -- the console
    # reads from browsers and leaves it true, and the tests read the console's
    # own answers back and set it false.
    key = _exactly(rfile, 4) if masked else b""
    payload = _exactly(rfile, length) if length else b""
    if from_client and not masked:
        raise ProtocolError("a client frame arrived unmasked")
    if not from_client and masked:
        raise ProtocolError("a server frame arrived masked")
    return fin, opcode, _xor(payload, key) if masked else payload


def read_message(rfile, from_client: bool = True) -> tuple[int, bytes]:
    """One whole message, reassembling continuations; control frames pass straight
    through, because they are allowed to arrive in the middle of one and the
    caller is the half that can answer them."""
    fin, opcode, payload = read_frame(rfile, from_client)
    if opcode in (CLOSE, PING, PONG):
        return opcode, payload
    if opcode == CONT:
        raise ProtocolError("a continuation arrived with nothing to continue")
    while not fin:
        fin, part, more = read_frame(rfile, from_client)
        if part in (CLOSE, PING, PONG):
            # A control frame inside a fragmented message is legal. Nothing here
            # sends fragments, so rather than hold half a message the connection
            # is ended -- which is what a close in the middle would mean anyway.
            return part, more
        if part != CONT:
            raise ProtocolError("a new message started inside another one")
        payload += more
        if len(payload) > MAX_MESSAGE:
            raise ProtocolError("a fragmented message grew past the limit")
    return opcode, payload


def close_frame(code: int = 1000, reason: str = "") -> bytes:
    """A close payload. The reason is truncated to what a control frame may hold.

    123 bytes, not 125, because the code takes the first two -- and the rover has
    met the other side of this rule already: Alibaba's realtime service once
    refused a session with a 130-byte reason, and every conformant client
    discarded the text and raised a protocol error instead, so the actual message
    ("the free tier of the model has been exhausted") was invisible. A limit
    worth respecting from this side.
    """
    return struct.pack(">H", code) + reason.encode("utf-8")[:123]
