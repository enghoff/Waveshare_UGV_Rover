"""The audio socket: what the browser heard, and starting again.

Audio crosses the rover's wifi in both directions and the conversation is
stateful, so a second conversation has to start at once rather than waiting for
the first to finish dying.
"""
from __future__ import annotations

import contextlib
import io
import os
import re
import sys
import tempfile

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import SKIP, check


def test_the_audio_socket() -> None:
    """The framing under the microphone, with no browser and no model.

    This is a protocol implemented by hand -- see [wsframe.py](wsframe.py) for
    why it is not a library -- and the two rules worth pinning are the ones whose
    failure is silent. A length that crosses one of the header's size boundaries
    comes out as a frame that reads fine and is one byte long; masking applied on
    the wrong side is a connection the browser closes without saying anything.
    Neither shows up as an exception in the place that caused it.
    """
    import wsframe

    # RFC 6455's own example, so this is checked against the standard rather than
    # against itself.
    check("the handshake matches the standard's worked example",
          wsframe.accept("dGhlIHNhbXBsZSBub25jZQ=="),
          "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=")

    for size in (0, 1, 125, 126, 127, 65535, 65536, 70000):
        raw = bytes(range(256)) * (size // 256) + bytes(size % 256)
        wire = wsframe.frame(wsframe.BINARY, raw, mask=True)
        opcode, back = wsframe.read_message(io.BytesIO(wire))
        check(f"a {size} byte frame survives the wire", (opcode, back),
              (wsframe.BINARY, raw))

    # The server's own frames are unmasked, and a client reading them says so.
    wire = wsframe.frame(wsframe.TEXT, b"hello")
    check("a server frame reads back on the client side",
          wsframe.read_message(io.BytesIO(wire), from_client=False),
          (wsframe.TEXT, b"hello"))

    for name, wire, from_client in (
            ("an unmasked frame from a client is refused",
             wsframe.frame(wsframe.TEXT, b"x"), True),
            ("a masked frame from a server is refused",
             wsframe.frame(wsframe.TEXT, b"x", mask=True), False)):
        try:
            wsframe.read_message(io.BytesIO(wire), from_client=from_client)
            check(name, "accepted", "refused")
        except wsframe.ProtocolError:
            check(name, "refused", "refused")

    # Fragments, because a browser may send them even though this never does.
    first = wsframe.frame(wsframe.BINARY, b"one", mask=True)
    first = bytes([first[0] & 0x7F]) + first[1:]        # clear FIN
    rest = wsframe.frame(wsframe.CONT, b"-two", mask=True)
    check("a fragmented message is put back together",
          wsframe.read_message(io.BytesIO(first + rest)),
          (wsframe.BINARY, b"one-two"))

    # A close reason longer than a control frame may carry. The rover has been on
    # the receiving end of this one: Alibaba's service once refused a session with
    # a reason too long to be legal, and every conformant client discarded it, so
    # the actual message was invisible.
    payload = wsframe.close_frame(1000, "x" * 400)
    check("a close reason is cut to what a control frame may hold",
          len(payload) <= 125, True)


def test_talking_needs_no_console_token() -> None:
    """Both doors into one conversation are open to the console's LAN."""
    try:
        import http.client
        import json
        import socket
        import threading

        import drive_web
        import wsframe
    except Exception as error:                         # noqa: BLE001
        SKIP.append(f"the microphone endpoints ({type(error).__name__}: {error})")
        return

    class FakeOmni:
        def __init__(self) -> None:
            self.started = 0
            self.attached = threading.Event()
            self.detached = threading.Event()

        def turn_on(self) -> str:
            self.started += 1
            return ""

        def attach(self, wire) -> None:
            self.attached.set()

        def detach(self, wire) -> None:
            self.detached.set()

        def on_audio(self, data: bytes) -> None:
            pass

    previous = drive_web.Handler.session, drive_web.Handler.omni
    session = drive_web.Session(None, 3.0, 480)
    model = FakeOmni()
    drive_web.Handler.session = session
    drive_web.Handler.omni = model
    server = drive_web.Console(("127.0.0.1", 0), drive_web.Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    port = server.server_address[1]

    control = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    audio = None
    try:
        body = json.dumps({"omni": True})
        control.request("POST", "/do", body=body,
                        headers={"Content-Type": "application/json",
                                 "Content-Length": str(len(body))})
        reply = control.getresponse()
        answer = json.loads(reply.read())
        check("talking starts without a console token", answer.get("ok"), True)
        check("...and reaches the hosted session", model.started, 1)

        audio = socket.create_connection(("127.0.0.1", port), timeout=5)
        audio.sendall(
            b"GET /audio HTTP/1.1\r\n"
            b"Host: 127.0.0.1\r\n"
            b"Upgrade: websocket\r\n"
            b"Connection: Upgrade\r\n"
            b"Sec-WebSocket-Version: 13\r\n"
            b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n")
        headers = b""
        while b"\r\n\r\n" not in headers:
            headers += audio.recv(4096)
        check("the audio socket opens without a token query",
              headers.startswith(b"HTTP/1.1 101"), True)
        check("...and is attached to the hosted session",
              model.attached.wait(1), True)
        audio.sendall(wsframe.frame(wsframe.CLOSE, b"", mask=True))
        check("closing it detaches the microphone", model.detached.wait(1), True)
    finally:
        control.close()
        if audio is not None:
            audio.close()
        server.shutdown()
        server.server_close()
        session.close()
        drive_web.Handler.session, drive_web.Handler.omni = previous


def test_what_the_browser_heard() -> None:
    """The playback accounting an interruption depends on.

    When somebody talks over the rover, the model believes it said the whole
    reply and the only correction available is the number of milliseconds that
    were actually audible. On a desk that number comes off a sound card; here it
    comes back over the wifi from a browser, which means it can be late, stale, or
    about a reply that has already been replaced -- and each of those has a wrong
    answer that sounds like a fault rather than looking like a bug.
    """
    try:
        import numpy as np

        from omni_bridge import BrowserSpeaker
    except Exception as error:                         # noqa: BLE001
        SKIP.append(f"the browser speaker ({type(error).__name__}: {error})")
        return

    sent: list[bytes] = []
    control: list[dict] = []
    speaker = BrowserSpeaker(sent.append, control.append)

    speaker.begin()
    check("a reply starts by telling the page so", control[0]["t"], "begin")
    # A second of audio at the rate the service speaks.
    speaker.write(np.zeros(24000, dtype=np.float32))
    check("what was sent is a second of PCM16", len(sent[0]), 48000)

    speaker.note_played(control[0]["gen"], 400)
    check("what the page says it played is what is reported",
          speaker.played_ms(), 400)
    speaker.note_played(control[0]["gen"], 5000)
    check("...but never more than was actually sent", speaker.played_ms(), 1000)

    # A report about the previous reply, arriving after the next one began.
    speaker.begin()
    speaker.write(np.zeros(2400, dtype=np.float32))
    speaker.note_played(control[0]["gen"], 900)
    check("a report about a finished reply is ignored", speaker.played_ms(), 0)

    speaker.note_played(control[-1]["gen"], 40)
    dropped = speaker.flush()
    check("an interruption tells the page to drop what is queued",
          control[-1]["t"], "flush")
    check("...and says how much of the reply went unheard",
          round(dropped, 3), 0.06)
    check("...after which what was queued is what was heard",
          speaker.played_ms(), 40)


def test_a_second_conversation_starts_at_once() -> None:
    """Pressing start again straight after a refresh must reach the model.

    Reproduced on the rover on 2026-08-27. Refreshing the console ends the
    conversation on purpose -- the page says so on `pagehide`, so a tab closed at
    bedtime cannot quietly spend the account's free quota -- and pressing start
    again a few seconds later failed with `[Errno 98] Address already in use`.
    The port that was in use is not the model's. It is the little loopback
    receiver the daemon posts `look`'s pictures to, which a conversation built
    before it dialled anything, so the failure landed before a single word
    reached the model and read as a rover that had stopped answering.

    What holds the port is the daemon: it keeps one connection to that receiver
    and is never told the conversation has ended, so an open connection on the
    port outlives the receiver that accepted it and the next `bind` is refused.
    A rebind is refused whether or not `SO_REUSEADDR` is set -- that forgives a
    port left in TIME_WAIT and this one is not -- so the fix is to stop
    rebinding, and the receiver now lives as long as the console does.
    """
    try:
        import http.client
        import json
        import socket

        import omni_bridge
        import talk_frames
    except Exception as error:                         # noqa: BLE001
        SKIP.append(f"a second conversation ({type(error).__name__}: {error})")
        return

    # A JPEG only in the sense the receiver checks for.
    jpeg = b"\xff\xd8\xff\xe0\x00\x10JFIF" + b"\x00" * 32 + b"\xff\xd9"

    def free_port() -> int:
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        return port

    def post(connection) -> dict:
        """One picture, over a connection the caller keeps -- as the daemon does."""
        connection.request("POST", "/frame", body=jpeg,
                           headers={"Content-Type": "image/jpeg",
                                    "Content-Length": str(len(jpeg))})
        return json.loads(connection.getresponse().read())

    # 1. The trap, walked into exactly as a conversation used to walk into it.
    port = free_port()
    receiver = talk_frames.Frames(port=port, host="127.0.0.1")
    receiver.serve_in_background()
    daemon = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    try:
        check("the daemon files a picture with the receiver",
              post(daemon).get("ok"), True)
        # The conversation ends. The daemon is not told and does not let go.
        receiver.shutdown()
        receiver.server_close()
        try:
            talk_frames.Frames(port=port, host="127.0.0.1").server_close()
            refused = ""
        except OSError as error:
            refused = str(error)
        if sys.platform.startswith("linux"):
            check("...and the next conversation cannot have the port back",
                  "in use" in refused, True)
        else:
            SKIP.append("the port a finished receiver leaves behind "
                        f"(this kernel hands it straight back: {refused or 'ok'})")
    finally:
        daemon.close()

    # 2. And the console no longer asks for it back, because it never let go.
    port = free_port()
    said: list[str] = []
    omni = omni_bridge.Omni("127.0.0.1:1", lambda text, err=False: said.append(text),
                            frame_port=port)
    daemon = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    try:
        first = omni._frame_server()
        picture = post(daemon).get("image")
        check("a conversation has somewhere for the pictures to go",
              bool(picture), True)

        # The conversation ends -- a refresh, the button, or the idle watch --
        # and the next one starts while the daemon's connection is still open.
        second = omni._frame_server()
        check("the next conversation is served by the same receiver",
              second is first, True)
        check("...so the daemon's kept-open connection is still good",
              post(daemon).get("ok"), True)
        check("...and a picture from the conversation before is gone",
              first.take(picture), None)
    finally:
        daemon.close()
        omni.close()

    check("closing the console is what finally gives the port up",
          omni._frames, None)


def test_the_conversation_is_written_down() -> None:
    """What the model was asked, what it called, and what it was told back.

    The fault this exists for: the rover refuses to go somewhere, says so out
    loud, and by the time anybody asks why, the sentence it was handed has faded
    off the console and is in no file on the board. The daemon logs nothing per
    tool call and the protocol trace is all-or-nothing, so this file is the only
    place the refusal is kept.
    """
    import omni_bridge

    with tempfile.TemporaryDirectory() as work:
        path = os.path.join(work, "omni.log")
        said: list[str] = []
        console = omni_bridge.Omni("127.0.0.1:1",
                                   lambda text, err=False: said.append(text),
                                   log_path=path)
        check("a console nobody has spoken to leaves no file behind",
              os.path.exists(path), False)

        # Through `Notes`, because that is the object `Session` is handed: this
        # checks the whole road from what the conversation reports to what lands
        # on the disk, not just the last step of it.
        notes = omni_bridge.Notes(console._note)
        notes.say("you: go to the sofa")
        notes.say('  [go_to_thing{"description": "the sofa"} -> {"ok": false, '
                  '"error": "there is no route to there that the rover fits '
                  'through"}]')
        notes.say("  error: the session went away", err=True)
        console.close()

        lines = open(path, encoding="utf-8").read().splitlines()
        check("every line the console showed is written down", len(lines), 3)
        check("...and the console still heard all three", len(said), 3)
        check("...each with the day and the second on it",
              all(re.match(r"^\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d ", line)
                  for line in lines), True)
        # The whole answer and not a summary of it: the error string *is* the
        # thing somebody came to this file to read.
        check("...the tool call keeps its name, its argument and its answer",
              ("go_to_thing" in lines[1] and '"the sofa"' in lines[1]
               and "no route to there that the rover fits through" in lines[1]),
              True)
        check("...and a line the console showed in red is marked as one",
              lines[2].split(" ", 1)[1].startswith("! "), True)

    # Rolled rather than grown, so an afternoon of talking cannot be the thing
    # that fills the board's disk.
    with tempfile.TemporaryDirectory() as work:
        path = os.path.join(work, "omni.log")
        log = omni_bridge.Transcript(path, roll_at=200, keep=1)
        for turn in range(40):
            log.write("you: %s" % ("say that again " * 3 + str(turn)))
        log.close()
        check("a log past its size is moved aside", os.path.exists(path + ".1"),
              True)
        check("...and only the one older file is kept",
              os.path.exists(path + ".2"), False)
        check("...with the newest lines in the live file",
              "39" in open(path, encoding="utf-8").read(), True)

    # A disk that will not take it is not a reason to stop talking to the model.
    # The complaint is caught rather than let out, both to keep this runner's
    # output clean and because the sentence is the thing being checked: a
    # transcript that stopped without a word would read as a conversation that
    # never happened.
    nowhere = os.path.join(tempfile.gettempdir(), "no-such-dir-for-omni", "x.log")
    log = omni_bridge.Transcript(nowhere)
    complaint = io.StringIO()
    with contextlib.redirect_stderr(complaint):
        log.write("you: hello")
        log.write("you: still here")
    check("a log that cannot be opened gives up and names the file",
          nowhere in complaint.getvalue(), True)
    check("...and says so once, not once a line",
          complaint.getvalue().count("no longer being written"), 1)
    check("...while the conversation carries on without it",
          os.path.exists(nowhere), False)


TESTS = (
    test_the_audio_socket,
    test_talking_needs_no_console_token,
    test_what_the_browser_heard,
    test_a_second_conversation_starts_at_once,
    test_the_conversation_is_written_down,
)
