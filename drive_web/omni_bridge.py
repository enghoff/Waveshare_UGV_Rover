#!/usr/bin/env python3
"""The omni conversation, run by the rover instead of by a desk.

[session.py](../voice_chat/session.py) holds a live session with Alibaba's realtime
model and drives the rover's tools from it, and everything measured about that --
which prompt sentence flash cannot be given, how a picture has to travel as a
turn of its own, what a barge-in has to tell the service it actually played --
lives in its `Session`. **This file does not fork any of that.** It supplies the
three things `Session` expects -- a speaker, an indicator and a frame
server -- backed by a browser instead of by a sound card, and lets the rest run
unchanged. A fork would drift from the file the measurements were taken against,
and the measurements are most of that file's value.

    console (drive_web.py, https on 8771)
      |  wss://<rover>:8771/audio        <- one browser, microphone and speaker
      |
    Omni  ---- asyncio thread ---->  wss://dashscope-intl...  (the model)
      |                                     |
      |  Frames on 127.0.0.1:8774           +-- tool calls
      |  <- the daemon posts look's JPEG    |
      +---------------------------------> 127.0.0.1:8769 (the daemon)

**Why the rover holds this and not the desk.** Every tool call and every picture
was crossing the LAN twice -- rover to desk to the model -- and `look` was
routing a 35 kB frame through whichever machine happened to have the microphone.
Here the daemon is on loopback and so is the frame server, so a picture travels
from the camera to the model without touching the house wifi at all. What crosses
the wifi instead is the audio, at 32 kB a second each way, which is a twentieth of
one picture a second.

**It is created on demand and closed again.** The account behind the key is
free-quota-only -- see the note in `secrets/` -- and a session that stayed open
because the rover was switched on would find that ceiling with nobody talking to
it. So there is no session until somebody presses the microphone button, and it
is torn down when the last browser goes away, when the button is pressed again,
or when nothing has been attached for IDLE_STOP_S.

**One microphone at a time.** Two browsers pushing audio into one context is two
people talking over each other into the same sentence, and the model has no way
to tell them apart. The newest connection wins and the older one is told it has
lost the microphone, which is at least legible; sharing it silently would not be.
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
import threading
import time
from typing import Any

import _paths  # noqa: F401 - session.py and its neighbours

import numpy as np

import rover_tools
import session as omni
from session import IN_RATE, OUT_RATE, _to_pcm16
from talk_frames import Frames

#: What the page has to send and what it will be sent, so that the rates live in
#: one place and the browser is told them rather than told to assume. The
#: microphone is 16 kHz because that is what the service takes; the reply comes
#: back at 24 kHz because that is what it speaks.
MIC_RATE = IN_RATE
PLAY_RATE = OUT_RATE

#: Where the console keeps what a deploy must not overwrite and must never carry
#: back into the repository. `make_cert.sh` writes its certificate here for the
#: same reason.
UGV = os.path.join(os.path.expanduser("~"), ".ugv")
KEY_PATH = os.path.join(UGV, "alibaba.key")

#: The frame server, loopback only. 8769 is the daemon, 8770 the depth camera,
#: 8771 this console, 8772 the board bridge and 8773 the nav bridge.
#:
#: **Plain HTTP on purpose, and it is not a hole.** The console speaks TLS
#: because a browser will not open a microphone otherwise; the daemon posting a
#: picture is not a browser, it is a process on this same board, and pointing it
#: at an https endpoint would mean it either following a redirect it does not
#: follow or verifying a certificate it has no reason to hold. Loopback and
#: plain is the honest shape: nothing off this board can reach it.
FRAME_PORT = 8774
FRAME_HOST = "127.0.0.1"

#: How long the session survives with no browser holding the microphone. Long
#: enough to reload a page or lose wifi for a moment, short enough that a tab
#: closed at bedtime is not still holding a conversation open at midnight.
IDLE_STOP_S = 120.0

#: What the browser's playback cursor is allowed to be behind by before it is
#: treated as stale. It is reported roughly five times a second, so this is
#: several reports missed rather than one.
PLAYED_STALE_S = 2.0

#: Where the conversation is written down: beside the console's own log, in the
#: directory this file is deployed into. Not under ~/.ugv because it is neither a
#: secret nor state -- it is the same lines the console shows, kept.
LOG_NAME = "omni.log"

#: Roll at a megabyte and keep one older file, so the record survives a session
#: that talked all afternoon without the board's disk being the thing that
#: notices. A spoken exchange with a tool call in it is a few hundred bytes.
LOG_BYTES = 1024 * 1024
LOG_KEEP = 1

#: The longest line worth writing whole. Every tool a model is offered answers in
#: words -- `look` and `show_map` send their pictures by another road and hand
#: back a name -- so nothing should come near this, and a line that does is a
#: result that has grown a payload rather than one worth keeping in full.
LOG_LINE_MAX = 4000


def api_key() -> str:
    """The DashScope key, from ~/.ugv/ rather than from the repository's secrets/.

    Deliberately outside ~/ugv. A deploy lands on ~/ugv and `rsync --delete` runs
    against parts of it, so a key there is one `scp -r` away from being copied
    back to the workstation and one careless commit away from being public.
    """
    from_env = os.environ.get("DASHSCOPE_API_KEY", "").strip()
    if from_env:
        return from_env
    with open(KEY_PATH, encoding="utf-8") as handle:
        key = handle.read().strip()
    if not key:
        raise OSError(f"{KEY_PATH} is empty")
    return key


class BrowserSpeaker:
    """The speaker interface `Session` calls, with the sound card a network away.

    `Session` asks a speaker four things: start a reply, take this audio, how
    many milliseconds of it were actually heard, and throw away what has not been
    heard yet. Three of those are local questions on a desk and none of them is
    here, because the audio is played in a browser on the other side of the wifi.

    **The one that matters is `played_ms`, and it is why this is not a queue.**
    When somebody interrupts, the model believes it said the whole reply, and the
    only correction available is to tell it how much was audible. Get that number
    from what was *sent* and every interruption teaches the model it said more
    than anyone heard, which is a conversation that drifts out of agreement with
    the room and never comes back. So the browser reports its own playback cursor
    and this reports what the browser said, clamped to what was sent, and treats
    a report older than PLAYED_STALE_S as no report at all.
    """

    rate = OUT_RATE

    def __init__(self, send_audio, send_control) -> None:
        self._send_audio = send_audio
        self._send_control = send_control
        self._lock = threading.Lock()
        self._queued_ms = 0.0     # audio handed to the browser for this reply
        self._played_ms = 0.0     # what it says it has played of it
        self._played_at = 0.0
        self._generation = 0      # bumped per reply, so a late report is ignored

    # --- what Session calls ---------------------------------------------------

    def begin(self) -> None:
        with self._lock:
            self._generation += 1
            self._queued_ms = 0.0
            self._played_ms = 0.0
            self._played_at = time.monotonic()
            generation = self._generation
        self._send_control({"t": "begin", "gen": generation, "rate": OUT_RATE})

    def write(self, audio: np.ndarray) -> None:
        pcm = _to_pcm16(audio)
        with self._lock:
            self._queued_ms += len(pcm) / 2 * 1000.0 / OUT_RATE
        self._send_audio(pcm)

    def played_ms(self) -> int:
        with self._lock:
            if time.monotonic() - self._played_at > PLAYED_STALE_S:
                # No word from the browser recently. Everything sent is the
                # honest guess then: it is the answer that over-states what was
                # heard, and over-stating means the model believes it said
                # something nobody heard -- which is recoverable by asking it to
                # repeat. Under-stating makes it repeat what was already heard,
                # which sounds like a fault.
                return int(self._queued_ms)
            return int(min(self._played_ms, self._queued_ms))

    def flush(self) -> float:
        """Drop what has not been played, and say how many seconds went."""
        self._send_control({"t": "flush"})
        with self._lock:
            dropped = max(0.0, self._queued_ms - self._played_ms) / 1000.0
            self._queued_ms = self._played_ms
        return dropped

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._played_ms + 50 < self._queued_ms

    # --- what the browser tells us -------------------------------------------

    def note_played(self, generation: int, played_ms: float) -> None:
        with self._lock:
            if generation != self._generation:
                return            # a report about a reply that has been replaced
            self._played_ms = max(self._played_ms, float(played_ms))
            self._played_at = time.monotonic()


class Transcript:
    """The same lines the console shows, appended to a file with a clock on them.

    **The console shows this conversation and keeps none of it.** A notice is one
    line that replaces the last and fades, which is the right shape for somebody
    watching the rover and the wrong one for somebody asked afterwards why it
    refused. Nothing else holds the answer either: the daemon writes nothing per
    tool call, and the protocol trace behind `QWEN_REALTIME_TRACE` is every frame
    of the websocket or none of them, decided before the session starts.

    So a model that says it cannot get to the sofa was quoting a refusal some
    tool handed it, and until this file that sentence had been shown once and
    written down nowhere. What lands here is what was heard, what was said, and
    each tool call with the whole of its answer, in the order they happened.

    Opened on the first line rather than at start-up, so a console nobody has
    spoken to leaves no file behind -- which is also what keeps it out of a test
    that builds an `Omni` and never says anything through it.
    """

    def __init__(self, path: str, roll_at: int = LOG_BYTES,
                 keep: int = LOG_KEEP) -> None:
        self.path = path
        self.broken = ""            # why it stopped writing, if it has
        self._roll_at = roll_at
        self._keep = keep
        self._lock = threading.Lock()
        self._handle = None

    def write(self, text: str, err: bool = False) -> None:
        """One line, stamped. Never raises: a log is not worth a conversation."""
        line = text.strip()
        if not line:
            return
        if len(line) > LOG_LINE_MAX:
            line = line[:LOG_LINE_MAX] + " ...[%d more]" % (len(line) - LOG_LINE_MAX)
        stamp = time.strftime("%Y-%m-%dT%H:%M:%S")
        try:
            with self._lock:
                if self.broken:
                    return
                self._roll()
                if self._handle is None:
                    self._handle = open(self.path, "a", buffering=1,
                                        encoding="utf-8")
                self._handle.write("%s %s%s\n" % (stamp, "! " if err else "", line))
        except OSError as error:
            # Said once, on the console's own stderr, which is the log the
            # supervisor keeps. Going quiet without a word would leave a missing
            # transcript looking like a conversation that never happened.
            self.broken = str(error)
            self._handle = None
            print(f"note: the conversation is no longer being written to "
                  f"{self.path}: {error}", file=sys.stderr, flush=True)

    def _roll(self) -> None:
        """Move the file aside once it is big enough. Called holding the lock."""
        if self._handle is None or self._handle.tell() < self._roll_at:
            return
        self._handle.close()
        self._handle = None
        for older in range(self._keep, 0, -1):
            source = self.path if older == 1 else "%s.%d" % (self.path, older - 1)
            if os.path.exists(source):
                os.replace(source, "%s.%d" % (self.path, older))

    def close(self) -> None:
        with self._lock:
            handle, self._handle = self._handle, None
        if handle is not None:
            handle.close()


class Notes:
    """The indicator interface `Session` calls, writing into the console's transcript.

    The desk client draws a spinner and prints lines under it. Here the same
    lines are what the page shows, so `set` is a state the page renders and `say`
    is a line in the log it already scrolls -- the same log the driving tools
    write to, deliberately, so that "you: turn left" and the turn itself appear
    in one place and in the order they happened.
    """

    def __init__(self, note) -> None:
        self._note = note
        self.state = "idle"

    def set(self, state: str) -> None:
        self.state = state

    def clear(self) -> None:
        self.state = "idle"

    def say(self, text: str, err: bool = False) -> None:
        self._note(text.strip(), err=err)


class Omni:
    """The session, its asyncio thread, and the one browser attached to it.

    Every public method here is called from a console thread -- an HTTP handler,
    or the pump -- and everything it touches inside the loop is reached with
    `call_soon_threadsafe`. That is the whole of the threading story: one loop,
    one lock, and no shared mutable state that both sides write.
    """

    def __init__(self, rover_address: str, note, model: str | None = None,
                 frame_port: int = FRAME_PORT, log_path: str | None = None) -> None:
        self.rover_address = rover_address
        self.model = model or omni.MODEL
        self.frame_port = frame_port
        self._say = note
        # Beside this file, which on the rover is ~/ugv/drive_web/ -- the same
        # directory the supervisor's own log is in. A caller with somewhere else
        # in mind says so; a test that wants no file says so with a temporary one.
        self.log = Transcript(log_path or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), LOG_NAME))
        self._lock = threading.Lock()

        self.state = "off"           # off | starting | live | closing | error
        self.error = ""
        self.since = 0.0
        self.heard = ""              # the last thing the person said
        self.said = ""               # ...and the last thing the rover said
        self.tool_names: list[str] = []

        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._stop: asyncio.Event | None = None
        self._audio: asyncio.Queue | None = None
        self._session: Any = None
        self._speaker: BrowserSpeaker | None = None
        self._frames: Frames | None = None
        self._wire: Any = None
        self._detached_at = time.monotonic()

    def _note(self, text: str, err: bool = False) -> None:
        """Say one line to the console, and write the same line down.

        Everything the conversation reports comes through here -- what `Session`
        tells its indicator, and this class's own lines about the microphone --
        so this is the one place that has to know the record exists. Called from
        the asyncio thread and from console threads both; `Transcript` holds the
        lock that makes that safe.
        """
        self.log.write(text, err)
        self._say(text, err=err)

    # --- what the console asks -------------------------------------------------

    def status(self) -> dict[str, Any]:
        with self._lock:
            live = self.state == "live"
            return {
                "state": self.state,
                "error": self.error,
                "model": self.model,
                "listening": bool(self._wire) and live,
                "seconds": round(time.monotonic() - self.since, 1) if self.since else 0,
                "heard": self.heard,
                "said": self.said,
                "tools": len(self.tool_names),
            }

    def turn_on(self) -> str:
        """Start a session. Returns "" or a sentence about why it did not."""
        with self._lock:
            if self.state in ("starting", "live"):
                return ""
            if self._thread is not None and self._thread.is_alive():
                return "the last session is still closing; try again in a moment"
            try:
                key = api_key()
            except OSError as error:
                self.state, self.error = "error", (
                    f"no API key on this rover: {error}. Put the DashScope key in "
                    f"{KEY_PATH}, one line, mode 600.")
                return self.error
            self.state, self.error, self.since = "starting", "", time.monotonic()
            # The grace before an unattended session closes itself runs from
            # here, not from whenever a browser last let go. Without this the
            # console's own start time is the clock, so a session opened an hour
            # later is already past its idle limit and closes on the watchdog's
            # first tick -- which reads exactly like the service hanging up.
            self._detached_at = time.monotonic()
            self._thread = threading.Thread(target=self._run, args=(key,),
                                            daemon=True, name="omni")
            self._thread.start()
        self._note("microphone: connecting to the model")
        return ""

    def turn_off(self) -> None:
        loop, stop = self._loop, self._stop
        if loop is not None and stop is not None:
            loop.call_soon_threadsafe(stop.set)
        with self._lock:
            if self.state in ("starting", "live"):
                self.state = "closing"

    def close(self) -> None:
        self.turn_off()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=5.0)
        with self._lock:
            frames, self._frames = self._frames, None
        if frames is not None:
            frames.shutdown()
            frames.server_close()
        # Last, and not in turn_off: the record spans the whole life of the
        # console, not one press of the microphone button.
        self.log.close()

    # --- what the audio socket does -------------------------------------------

    def attach(self, wire) -> None:
        """A browser has opened its microphone. The newest one wins."""
        with self._lock:
            old, self._wire = self._wire, wire
        if old is not None and old is not wire:
            old.evict("another browser took the microphone")

    def detach(self, wire) -> None:
        with self._lock:
            if self._wire is wire:
                self._wire = None
                self._detached_at = time.monotonic()

    def on_audio(self, pcm: bytes) -> None:
        """One block of microphone audio, PCM16 mono at IN_RATE, from the page."""
        loop, audio = self._loop, self._audio
        if loop is None or audio is None:
            return
        try:
            loop.call_soon_threadsafe(audio.put_nowait, pcm)
        except RuntimeError:
            pass                      # the loop is on its way out

    def on_played(self, generation: int, played_ms: float) -> None:
        speaker = self._speaker
        if speaker is not None:
            speaker.note_played(generation, played_ms)

    # --- the session itself ----------------------------------------------------

    def _frame_server(self) -> Frames:
        """The loopback receiver `look` posts its pictures to, bound once and kept.

        **It outlives the conversation on purpose, and that is a fix rather than
        an oversight.** It used to be built at the top of every conversation and
        torn down at the end of it, which read as tidy and was not: the daemon
        holds one connection to this receiver and is never told a conversation
        has ended, so the port stayed spoken for by that still-open connection
        after the receiver had gone. Refreshing the console ends the conversation
        deliberately, and pressing start again a few seconds later then died on
        `[Errno 98] Address already in use` -- before the model had been dialled
        at all, so what a person saw was a rover that had stopped answering.
        Reproduced on the rover on 2026-08-27 and pinned by
        `test_a_second_conversation_starts_at_once`.

        Binding once means nothing is ever rebound, and it suits the daemon
        better than the old arrangement did: its one kept-open connection stays
        good from one conversation to the next instead of having to be remade.
        What each new conversation gets is an empty receiver, not a new one.
        """
        with self._lock:
            frames = self._frames
        if frames is not None:
            frames.forget()
            return frames
        frames = Frames(port=self.frame_port, host=FRAME_HOST)
        frames.serve_in_background()
        with self._lock:
            self._frames = frames
        return frames

    def _send_audio(self, pcm: bytes) -> None:
        wire = self._wire
        if wire is not None:
            wire.audio(pcm)

    def _send_control(self, payload: dict) -> None:
        wire = self._wire
        if wire is not None:
            wire.control(payload)

    def _run(self, key: str) -> None:
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._converse(key))
        except (Exception, SystemExit) as error:       # noqa: BLE001 - reported, not raised
            with self._lock:
                self.state, self.error = "error", f"{type(error).__name__}: {error}"
            self._note(f"microphone: {self.error}", err=True)
        finally:
            try:
                loop.run_until_complete(loop.shutdown_asyncgens())
            finally:
                loop.close()
            self._loop = None
            self._stop = None
            self._audio = None
            self._session = None
            self._speaker = None
            with self._lock:
                if self.state != "error":
                    self.state, self.since = "off", 0.0
            self._note("microphone: the session is closed")

    async def _converse(self, key: str) -> None:
        # Named before the `try`, because the `finally` names it and the first
        # thing that can fail is the connection that has not happened yet.
        session = None
        self._stop = asyncio.Event()
        self._audio = asyncio.Queue()
        speaker = BrowserSpeaker(self._send_audio, self._send_control)
        self._speaker = speaker
        notes = Notes(self._note)

        # The frame server first, so that `set_vision` below has somewhere real to
        # point. It is this process, on loopback, and it is the same one every
        # conversation uses -- see `_frame_server` for why building a fresh one
        # here was what stopped a second conversation from ever starting.
        frames = self._frame_server()

        rover = rover_tools.RoverClient(self.rover_address)
        try:
            answer = await asyncio.to_thread(
                rover.call, "set_vision",
                {"address": f"{FRAME_HOST}:{self.frame_port}"})
            if not answer.get("ok"):
                self._note(f"microphone: the daemon would not take set_vision "
                           f"({answer.get('error')}), so look has nowhere to post",
                           err=True)

            async with await omni._open(omni.ENDPOINT, key, self.model) as ws:
                session = omni.Session(ws, rover, frames, speaker, notes,
                                       duplex=True, model=self.model)
                self._session = session

                async def receive() -> None:
                    async for message in ws:
                        await session.handle(json.loads(message))

                reader = asyncio.create_task(receive())
                tools = await asyncio.to_thread(rover.tools)
                vision = any(t.get("function", {}).get("name") == "look"
                             for t in tools)
                await session.configure(tools, vision=vision)
                with self._lock:
                    self.state = "live"
                    self.tool_names = [t.get("function", {}).get("name", "")
                                       for t in tools]
                self._note(f"microphone: live, {len(tools)} tools, "
                           f"{'vision' if vision else 'no vision'}")

                pump = asyncio.create_task(self._pump(session))
                idle = asyncio.create_task(self._idle_watch())
                done, pending = await asyncio.wait(
                    [reader, pump, idle, asyncio.create_task(self._stop.wait())],
                    return_when=asyncio.FIRST_COMPLETED)
                for task in pending:
                    task.cancel()
                await session.drain()
                # Whichever finished first may have finished by raising, and a
                # socket closed by the service is the interesting case: it is how
                # an exhausted free tier presents. Re-raise it so _run reports it.
                for task in done:
                    if not task.cancelled() and task.exception() is not None:
                        raise task.exception()
        finally:
            self._session = None
            self._mirror(session=None)

    async def _pump(self, session) -> None:
        """Microphone blocks into the session, in the order they arrived.

        `hold_mic` is the one thing that stops it, and it is set while a picture
        is being handed over: a frame goes up as a turn of its own and anything
        said across that moment would join it. Dropping rather than buffering is
        deliberate -- audio held back and released a second late is a person
        being answered a second late, for ever, and the words lost here are the
        ones spoken into a camera shutter.
        """
        assert self._audio is not None
        while True:
            pcm = await self._audio.get()
            if session.hold_mic:
                continue
            await session.push(pcm)
            self._mirror(session)

    def _mirror(self, session) -> None:
        """Copy the last thing said each way out of the session, for the page."""
        with self._lock:
            if session is None:
                return
            if session.transcripts:
                self.heard = session.transcripts[-1]
            if session.said:
                self.said = session.said[-1]

    async def _idle_watch(self) -> None:
        """Close the session when no browser has held the microphone for a while,
        and when the service's own two-hour ceiling is close enough to matter."""
        while True:
            await asyncio.sleep(5.0)
            with self._lock:
                attached = self._wire is not None
                gone = time.monotonic() - self._detached_at
                age = time.monotonic() - self.since if self.since else 0.0
            if not attached and gone > IDLE_STOP_S:
                self._note(f"microphone: nothing has been listening for "
                           f"{IDLE_STOP_S:.0f}s, so the session is closing")
                return
            if age > omni.SESSION_LIMIT_S - 60:
                self._note("microphone: this service caps a session at two hours "
                           "and it is there; closing, press the button to start "
                           "another")
                return
