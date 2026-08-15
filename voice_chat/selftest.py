"""Offline checks for the parts of the voice stack that need no GPU and no mic.

Two halves, deliberately runnable independently, because they have different
dependencies and live on different machines:

    python voice_chat/selftest.py          # client half, needs only numpy
    ssh root@media /opt/voice_chat/.venv/bin/python /opt/voice_chat/selftest.py

Whichever half cannot import its dependencies is reported as skipped rather than
failing, so each machine runs the part that is actually deployed on it.

What is covered is the logic that decides *when* to speak and *what* to hand the
synthesiser -- the two places where a bug is silent rather than loud. The models
themselves are not covered here; they need the card.
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


def test_wsclient() -> None:
    """The hand-rolled WebSocket framing -- wrong here and the server just hangs up."""
    import threading

    try:
        from wsclient import BINARY, WebSocket, _xor
    except Exception as exc:
        SKIP.append(f"websocket framing ({type(exc).__name__})")
        return

    mask = b"\xde\xad\xbe\xef"
    check("mask is its own inverse", _xor(_xor(b"hello rover", mask), mask), b"hello rover")
    # The big-integer trick is only there for speed; it has to agree exactly with
    # the obvious loop, including on payloads that start with a zero byte -- that
    # is where a length-losing int conversion would show up.
    payload = bytes(range(256)) * 3
    check(
        "big-int mask matches a byte loop",
        _xor(payload, mask),
        bytes(c ^ mask[i % 4] for i, c in enumerate(payload)),
    )
    check("leading zeros survive masking", len(_xor(b"\x00\x00\x00\x01", mask)), 4)
    check("empty payload masks to empty", _xor(b"", mask), b"")

    class FakeSocket:
        def __init__(self) -> None:
            self.sent = b""

        def sendall(self, data: bytes) -> None:
            self.sent += data

        def recv(self, _n: int) -> bytes:
            raise AssertionError("the reader wanted more than the writer wrote")

    def roundtrip(size: int):
        """Frame it, then parse the bytes back, without a socket in between."""
        ws = object.__new__(WebSocket)
        ws.sock = FakeSocket()
        ws._send_lock = threading.Lock()
        ws._buf = b""
        ws.closed = False
        body = bytes(i % 251 for i in range(size))
        ws._send_frame(BINARY, body)
        raw = ws.sock.sent
        ws._buf = raw
        return raw, ws._read_frame(), body

    # The three length encodings: 7-bit, 16-bit and 64-bit. An off-by-one in the
    # extended-length branches is invisible until a long utterance is sent.
    for size in (10, 1000, 70000):
        raw, (fin, opcode, got), body = roundtrip(size)
        check(f"frame round trip, {size} bytes", (fin, opcode, got), (True, BINARY, body))
        # An unmasked client frame is a protocol violation; the server closes
        # the connection rather than complaining, which looks like a network fault.
        check(f"client frame is masked, {size} bytes", bool(raw[1] & 0x80), True)


def main() -> int:
    test_sentences()
    test_endpointer()
    test_wsclient()

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
