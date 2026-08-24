"""The Alibaba realtime session the drive console's microphone runs.

The protocol lives here -- prompt, tools, barge-in, how a picture has to travel
as a turn of its own -- so [omni_bridge.py](../drive_web/omni_bridge.py) does not
fork it. That file supplies the three things :class:`Session` expects from a
desk (a speaker, an indicator, a frame server) backed by a browser instead of a
sound card.

Nothing here performs a tool. [rover_daemon.py](../rover_daemon/rover_daemon.py)
owns the hardware, [rover_tools.py](rover_tools.py) asks it what it can do, and
this passes calls along. Audio never touches a sound card in this process.

`look` does not hand its photograph back through the conversation; it posts the
JPEG to a loopback :class:`~talk_frames.Frames` server and this forwards it into
the session when the tool result names it. That destination used to be a constant
baked into the daemon; a client now says it on every connection with `set_vision`.
"""
from __future__ import annotations

import asyncio
import base64
import json
import os
import sys
from typing import Any

import numpy as np

try:
    import websockets
except ImportError:
    websockets = None  # type: ignore[assignment]

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import prompts
import rover_tools
from talk_frames import Frames, _jpeg_size

# 16 kHz is what the service takes; 24 kHz is what it speaks.
IN_RATE = 16000
OUT_RATE = 24000

OPEN_TIMEOUT_S = 15.0

# The international endpoint. The mainland host refuses this account's key with a
# 401, which reads like a bad key and is a wrong region -- see the note in
# `secrets/` and in the omni bench's cloud backend.
ENDPOINT = os.environ.get(
    "QWEN_REALTIME_URL",
    "wss://dashscope-intl.aliyuncs.com/api-ws/v1/realtime")
MODEL = os.environ.get(
    "QWEN_REALTIME_MODEL", "qwen3.5-omni-plus-realtime-2026-03-15")
# The dated snapshot, not the undated alias. As of 2026-08-17 the alias
# `qwen3.5-omni-plus-realtime` is refused outright: its free tier is exhausted
# and the console marks Stop-on-Exhaust as "Not Supported" for that name, which
# is Alibaba's way of saying the model has no free quota left to gate. The socket
# opens, `session.created` arrives, and the service then closes with 1007 and
# "The free tier of the model has been exhausted. If you wish to continue access
# the model on a paid basis, please disable the 'use free tier only' mode in the
# management console." Worth knowing how that presents, because it does not
# present as a billing error: the reason travels in a close frame longer than the
# 125 bytes the RFC allows a control frame, so `websockets` refuses the frame and
# raises a protocol error about its length while throwing the text away. It took
# a hand-rolled socket to read it.
#
# A dated snapshot is a different model with its own quota. Console as of
# 2026-08-19: `qwen3.5-omni-plus-realtime-2026-03-15` still has 1,000,000/1,000,000
# free until 2026-11-15, and Stop-on-Exhaust is already off. That is why it is
# the default. The alias is still
# `QWEN_REALTIME_MODEL=qwen3.5-omni-plus-realtime` once that row can bill.
# Flash is `QWEN_REALTIME_MODEL=qwen3.5-omni-flash-realtime`.
#
# Read what follows as the price of being here, not as a reason to stay. Plus
# costs about three times flash and the two are indistinguishable in
# docs/omni-step0.md -- both 90/90 typed and spoken, both 30/30 on the five extra
# tools. That was the *chat completions* pair. Their realtime namesakes are not
# the same models and do not behave alike, measured here against the mock rover,
# three samples a phrase:
#
#     first turn of a session      flash-realtime   plus-realtime
#     "Could you dim the lights a bit?"      0/3          3/3
#     "Can you look to your left?"           0/3          3/3
#     "Start tracking people."               0/3          3/3
#     "Switch the lights on."                1/3          3/3
#     "What do you see?"                  12/18          6/6
#
# Flash fails in the two ways this repository has spent the most words on. It
# announces without acting -- "I'll pan the camera to my left now", no call -- and
# it *writes the tool call into its own speech*, which comes out as the rover
# saying "<set_lights> <level>255</level> </set_lights> I've switched the lights
# on." out loud while doing nothing. Neither is caught here: the sniffer that
# catches the second one lives in server.py and watches a text stream, and this
# protocol replaces that stream with a control channel, so a model declining to
# use the channel looks like a model with nothing to say.
#
# One sentence of the prompt causes all of it, and see `instructions` below for
# which and for the measurement. With that sentence removed flash calls cleanly,
# which changes what the table above means: it is a measurement of flash under a
# prompt tuned for a different model, not of flash.


def _b64(raw: bytes) -> str:
    return base64.b64encode(raw).decode("ascii")


def _to_pcm16(audio: np.ndarray) -> bytes:
    """Float samples as the 16-bit little-endian PCM the service reads."""
    return (np.clip(audio, -1.0, 1.0) * 32767).astype("<i2").tobytes()


def _from_pcm16(raw: bytes) -> np.ndarray:
    return np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0

# The sentence, verbatim from server.py's TOOL_PROMPT. Removed on the way to
# flash-realtime and left alone everywhere else.
_READ_ALOUD = (" Then say what you did in one short sentence, without reading the"
               " tool call or its result out loud.")


def instructions(*, vision: bool, model: str) -> str:
    """The tuned prompt, minus the one sentence flash-realtime cannot be given.

    Three samples a cell, fourteen schemas, typed input so the microphone is out
    of it, counting first-turn calls for "Switch the lights on.":

        base prompt only                                    3/3
        + the tool prompt's first two sentences              3/3
        + the "you have done something only if" clause       3/3
        + the "do not say 'I will'" clause                   3/3
        + the vision paragraph                               3/3
        the whole prompt as server.py writes it              0/3
        the whole prompt minus this one sentence             3/3

    So it is not prompt length, not the tool count on its own, and not the
    clauses that were tuned to stop a model announcing instead of acting: it is
    the sentence that says not to read the tool call out loud, which is what
    makes flash read the tool call out loud. Naming the thing you do not want in
    a prompt is a way of asking for it, and this is the cleanest example of that
    the repository has.

    It stays in server.py because it earns its place there -- it is what stops
    the local model's speech decoder reciting result JSON, which step 0 caught it
    doing -- and the local path does not have a control channel to lose. Plus is
    given the sentence too, having been measured with it and being fine.

    This does not make flash equal to plus. Against the live daemon's fifteen
    schemas it fixes the lights, the camera, driving and looking, and it does not
    fix the tracking family: "Start tracking people." still calls 1/3 and "Follow
    me." 0/3, both by announcing in the past tense. That one is crowding rather
    than prompting -- it is 3/3 against the nine base tools and decays as the list
    grows -- and README.md has the numbers and the two wordings that failed to fix
    it.
    """
    prompt = prompts.system_prompt(vision=vision)
    if "flash" in model and _READ_ALOUD in prompt:
        return prompt.replace(_READ_ALOUD, "")
    return prompt

# Jennifer: the service's own description is "premium, cinematic-quality American
# English female voice". Not the service's default, which is Tina and which
# speaks English with a marked accent -- fine for a demonstration and wrong for a
# rover that is meant to sound like it is talking to you rather than reading to
# you. Three others were tried and all are accepted by this model:
#
#     Jennifer   American English, female   (the default here)
#     Aiden      American English, male, friendly and casual
#     Mione      British English, female, mature and intellectual
#     Tina       the service default, accented
#
# Anything else needs checking before it is put here, and checking it is cheap:
# the voice list differs between the realtime models, and a name from the wrong
# one does not fall back -- it is refused at the first `session.update` with
# "Voice 'x' is not supported" and the socket closes. `Cherry` is documented and
# refused by this model, which is how that was found out.
VOICE = os.environ.get("QWEN_REALTIME_VOICE", "Jennifer")

# 0.2 for the same reason the local service uses it: whether the model *acts*
# turned out to be a sampled decision, and at 0.7 half the tool calls that should
# have happened did not. See the table above TEMPERATURE in server.py.
TEMPERATURE = float(os.environ.get("QWEN_REALTIME_TEMPERATURE", "0.2"))

TRACE = os.environ.get("QWEN_REALTIME_TRACE", "") not in ("", "0", "false", "False")

# A session is capped at two hours by the service, and it says so by closing the
# socket. The console watches this and closes a little early so a conversation
# ending is a line in the log rather than a stack trace.
SESSION_LIMIT_S = 120 * 60

# A picture cannot be put into an empty input buffer. The rule is the service's
# and it is not negotiable -- "you must send audio data at least once before you
# send image data" -- and a buffer that has been committed counts as empty, which
# is exactly the state a tool result arrives in. So a frame travels as a user
# turn of its own, led by a fifth of a second of silence whose only job is to
# satisfy that rule. Measured rather than assumed: appending the image first
# fails with "Error append image before append audio", and the model then
# describes a room it was never shown, in confident detail.
PICTURE_LEAD_MS = 200

# Turn-taking, when the service is doing it. semantic_vad is the one that knows
# the difference between somebody saying "mm-hm" and somebody taking the floor,
# which is most of what makes barge-in usable rather than merely present.
#
# `interrupt_response` is the service's half of a barge-in; `Session.interrupt`
# is the other half, and they do different jobs. The service stops generating.
# Only the client can stop the speaker playing what it has already received.
DUPLEX_TURNS = {"type": "semantic_vad", "create_response": True,
                "interrupt_response": True}


class Session:
    """One conversation with the hosted model, and the rover behind it.

    Everything that crosses the socket goes through here, so the protocol is in
    one place and the two loops around it -- one pushing the microphone, one
    draining the model -- stay short enough to read.
    """

    def __init__(self, ws, rover: rover_tools.RoverClient | None,
                 frames: Frames | None, speaker: Any | None,
                 indicator: Any, duplex: bool, model: str,
                 quiet: bool = False) -> None:
        self.ws = ws
        self.rover = rover
        self.frames = frames
        self.speaker = speaker
        self.indicator = indicator
        self.duplex = duplex
        self.model = model
        self.quiet = quiet

        self.tools: list[dict] = []
        self.tool_names: list[str] = []
        self.responding = False
        self.calls: list[tuple[str, dict]] = []      # what the model asked for
        self.results: list[tuple[str, dict]] = []    # ...and what came back
        self.transcripts: list[str] = []
        self.said: list[str] = []
        self.usage: dict[str, Any] = {}
        self.errors: list[str] = []
        self.closed = False

        self._pending: list[asyncio.Task] = []
        self._finishing: asyncio.Task | None = None
        # Replies asked for and not yet begun. A tool call means two responses to
        # one question -- the call, then the sentence saying what was done -- and
        # between the first ending and the second starting the session looks
        # briefly idle while being nothing of the sort. Counting the gap is what
        # keeps the next question from being asked into the middle of this
        # answer, which is not a race that fails loudly: the conversation simply
        # runs one turn behind itself from then on.
        self._asked = 0
        # The assistant message currently being spoken, which is what a truncate
        # has to name. Learned from the audio deltas rather than announced.
        self._item_id: str | None = None
        # Raised when the item a commit produced actually appears in the
        # conversation. Waited on rather than assumed, because a
        # `response.create` that arrives before then is discarded without a word
        # -- see _show_picture. Note that this is later than the commit's own
        # acknowledgement, which is not late enough.
        self._landed = asyncio.Event()
        self._awaiting: str | None = None
        # Raised when the service acknowledges a session change, so a change can
        # be waited for rather than hoped for.
        self._configured = asyncio.Event()
        # Set while a picture is being pushed with server-side turn detection
        # switched off. The microphone loop reads it and stops uploading, so the
        # frame's turn is the frame and not whatever was said over it.
        self.hold_mic = False
        self._session: dict[str, Any] = {}
        self._announced = False  # the tool list is said once, not on every update
        # Undocumented for this service, and the thing that keeps a barge-in
        # honest, so it is tried once and given up on if refused. See _truncate.
        self._truncate_ok = True

    # --- sending ------------------------------------------------------------

    async def send(self, event: dict) -> None:
        if TRACE and event.get("type") != "input_audio_buffer.append":
            brief = {k: v for k, v in event.items() if k != "image"}
            print(f"    -> {json.dumps(brief)[:220]}", file=sys.stderr, flush=True)
        await self.ws.send(json.dumps(event))

    async def configure(self, tools: list[dict], *, vision: bool) -> None:
        """The one message that sets up everything: prompt, voice, tools, turns.

        The prompt and the ten schemas are *sent* once here rather than with
        every question, which is the shape the step-0 sweep wanted -- it found
        1,450 of a request's roughly 1,470 input tokens were these. They are
        still counted on every response, though: the usage this service reports
        puts `cached_tokens` at 0 or 128 against 1,900-2,400 input tokens a turn.
        So this saves the upload and not the bill.
        """
        self.tools = tools
        self.tool_names = [t["function"]["name"] for t in tools]
        session: dict[str, Any] = {
            "modalities": ["text", "audio"],
            "voice": VOICE,
            "input_audio_format": "pcm",
            "output_audio_format": "pcm",
            "instructions": instructions(vision=vision, model=self.model),
            "temperature": TEMPERATURE,
            # Turn-taking is either the service's job or this client's, never
            # both. See DUPLEX_TURNS for which and why.
            "turn_detection": (dict(DUPLEX_TURNS) if self.duplex else None),
        }
        if tools:
            # Straight from the daemon, in the shape the daemon already uses:
            # {"type": "function", "function": {...}}, which is what this API
            # takes too. Nothing is rewritten on the way through, so a schema
            # improved on the rover is in force here on the next connection.
            session["tools"] = tools
        self._session = session
        self._configured.clear()
        await self.send({"type": "session.update", "session": session})
        # And nothing is said until the service confirms it took all that. This
        # is not politeness: a turn committed while the update is still being
        # applied is answered by a model that has the prompt and not the tools,
        # and what that looks like is the model *writing* a tool call into its
        # own speech -- "<set_lights> <parameter=level> 128 </parameter>", read
        # aloud, with nothing called. It looks exactly like a model that cannot
        # use tools, and it is a client that asked too early.
        try:
            await asyncio.wait_for(self._configured.wait(), 15)
        except asyncio.TimeoutError:
            self.indicator.say("  [the session was never confirmed; carrying on]",
                               err=True)

    async def _turn_detection(self, detection: dict | None) -> None:
        """Change who decides when a turn ended, and wait to be told it took.

        Only used to switch the service's turn detection off for as long as a
        picture takes and back on afterwards -- see :meth:`_show_picture`. The
        whole session is resent rather than the one field, because this API's
        `session.update` replaces what it is given.
        """
        self._session["turn_detection"] = detection
        self._configured.clear()
        await self.send({"type": "session.update", "session": self._session})
        try:
            await asyncio.wait_for(self._configured.wait(), 10)
        except asyncio.TimeoutError:
            self.indicator.say("  [the session change was never acknowledged]", err=True)

    async def push(self, pcm: bytes) -> None:
        await self.send({"type": "input_audio_buffer.append", "audio": _b64(pcm)})

    async def commit(self) -> None:
        await self.send({"type": "input_audio_buffer.commit"})
        await self.ask()

    async def ask(self) -> None:
        """Ask for a reply, and remember that one is owed."""
        self._asked += 1
        await self.send({"type": "response.create"})

    @property
    def idle(self) -> bool:
        """Nothing is being said, asked for, or waited on at the rover."""
        return (not self.responding and self._asked == 0 and not self._pending
                and (self._finishing is None or self._finishing.done()))

    async def discard(self) -> None:
        await self.send({"type": "input_audio_buffer.clear"})

    async def interrupt(self) -> None:
        """Stop the reply, drop what has not been heard, and say how far it got.

        Three things, and all three are needed. Cancelling stops the model
        generating; flushing stops the speaker playing what already arrived; and
        the truncate is what stops the model believing it said the part nobody
        heard. Without the third, the rover is interrupted and then answers the
        next question as though the interrupted sentence had landed.
        """
        if not self.responding:
            return
        played = self.speaker.played_ms() if self.speaker else 0
        dropped = self.speaker.flush() if self.speaker else 0.0
        await self.send({"type": "response.cancel"})
        await self._truncate(played)
        self.responding = False
        if not self.quiet:
            self.indicator.say(f"  [interrupted after {played / 1000:.1f}s"
                               f", {dropped:.1f}s unheard]")

    async def _truncate(self, played_ms: int) -> None:
        """Tell the model how much of its reply was audible, if it will listen.

        `conversation.item.truncate` is in the OpenAI Realtime protocol this one
        is modelled on, and is not in this service's published client-event list.
        It may work anyway. Trying costs one message and one error event, and the
        error is caught and the attempt abandoned, so a service that does not
        have it degrades to a conversation whose model over-remembers rather than
        to a conversation that stops.
        """
        if not self._truncate_ok or not self._item_id:
            return
        await self.send({"type": "conversation.item.truncate",
                         "item_id": self._item_id,
                         "content_index": 0,
                         "audio_end_ms": played_ms})

    # --- receiving ----------------------------------------------------------

    async def handle(self, event: dict) -> None:
        kind = event.get("type", "")
        if TRACE:
            # Every event, minus the base64. Set QWEN_REALTIME_TRACE=1 when the
            # conversation stalls: this protocol fails by going quiet rather than
            # by complaining, and the difference between "no reply was asked for"
            # and "a reply was asked for and never came" is only visible here.
            brief = {k: v for k, v in event.items()
                     if k not in ("delta", "audio", "image", "event_id")}
            print(f"    <- {json.dumps(brief)[:220]}", file=sys.stderr, flush=True)

        if kind == "session.updated":
            self._configured.set()
            accepted = [t.get("function", {}).get("name")
                        for t in (event.get("session", {}).get("tools") or [])]
            if self.tool_names and not self.quiet and not self._announced:
                self._announced = True
                self.indicator.say(
                    f"tools: {', '.join(n for n in accepted if n) or 'none accepted'}")

        elif kind == "input_audio_buffer.speech_started":
            # Only meaningful with server-side turn detection. It is also the
            # barge-in signal: the service heard somebody start talking, and if
            # the rover is mid-sentence that somebody is interrupting it.
            if self.speaker is not None and self.responding:
                await self.interrupt()
            self.indicator.set("hearing")

        elif kind == "input_audio_buffer.committed":
            self._awaiting = event.get("item_id")

        elif kind == "conversation.item.input_audio_transcription.completed":
            if self._awaiting and event.get("item_id") == self._awaiting:
                self._awaiting = None
                self._landed.set()
                return  # the silence carrying a picture is not something anybody said
            text = (event.get("transcript") or "").strip()
            if text:
                self.transcripts.append(text)
                self.indicator.say(f"you: {text}")

        elif kind == "response.created":
            self.responding = True
            self._asked = max(self._asked - 1, 0)
            if self.speaker is not None:
                self.speaker.begin()
            self.indicator.set("thinking")

        elif kind == "response.audio.delta":
            self._item_id = event.get("item_id") or self._item_id
            if self.speaker is not None:
                self.speaker.write(_from_pcm16(base64.b64decode(event["delta"])))
                self.indicator.set("speaking")

        elif kind == "response.audio_transcript.done":
            text = (event.get("transcript") or "").strip()
            if text:
                self.said.append(text)
                self.indicator.say(f"bot: {text}")

        elif kind == "response.function_call_arguments.done":
            # Started here rather than after `response.done`, because a call is a
            # physical act -- `look` starts the camera, and that is seconds -- and
            # those seconds are better spent while the response is winding up
            # than after it. The result is not sent until the response is over,
            # which is the protocol's rule, not a choice.
            self._pending.append(asyncio.create_task(self._perform(event)))

        elif kind == "response.done":
            self.responding = False
            usage = (event.get("response") or {}).get("usage") or {}
            if usage:
                self.usage = usage
            if self._pending:
                self._finishing = asyncio.create_task(self._finish())
            else:
                self.indicator.set("listening")

        elif kind == "error":
            error = event.get("error") or {}
            message = error.get("message") or json.dumps(error)
            if "truncate" in message.lower() or error.get("param") == "item_id":
                # The optimistic attempt in _truncate. Say it once and stop.
                if self._truncate_ok:
                    self._truncate_ok = False
                    self.indicator.say(
                        "  [this service does not take conversation.item.truncate;"
                        " an interrupted reply stays whole in its memory]", err=True)
                return
            self.errors.append(message)
            self.indicator.say(f"  error: {message}", err=True)

    # --- tools --------------------------------------------------------------

    async def _perform(self, event: dict) -> tuple[str, dict, dict]:
        """Run one tool call on the rover. Never raises -- a failure is a result."""
        name = event.get("name") or ""
        call_id = event.get("call_id") or ""
        raw = (event.get("arguments") or "").strip()  # the service pads it with a space
        try:
            arguments = json.loads(raw) if raw else {}
        except ValueError:
            arguments = {}
        if not isinstance(arguments, dict):
            arguments = {}
        self.calls.append((name, arguments))

        if self.rover is None:
            result: dict = {"ok": False, "error": "no rover attached"}
        else:
            # On a thread: the daemon is on the other side of a LAN and a camera,
            # and this coroutine shares an event loop with the speaker.
            result = await asyncio.to_thread(self.rover.call, name, arguments)
        self.results.append((name, result))
        if not self.quiet:
            self.indicator.say(f"  [{name}{json.dumps(arguments)} -> {json.dumps(result)}]")
        return name, call_id, result

    async def _finish(self) -> None:
        """Hand back every result the last response asked for, then ask for a reply."""
        pending, self._pending = self._pending, []
        for task in pending:
            try:
                _name, call_id, result = await task
            except Exception as error:  # a crashed tool is still an answer
                self.indicator.say(f"  tool failed: {type(error).__name__}: {error}",
                                   err=True)
                continue
            # Resolved before the result is sent, not after, because a picture
            # that cannot be found changes what the result *says*. See _picture.
            jpeg, result = self._picture(result)
            await self.send({"type": "conversation.item.create",
                             "item": {"type": "function_call_output",
                                      "call_id": call_id,
                                      "output": json.dumps(result)}})
            if jpeg is not None:
                await self._show_picture(jpeg)
        await self.ask()

    def _picture(self, result: dict) -> tuple[bytes | None, dict]:
        """The frame a result names, and the result the model should be given.

        A `look` answers with nothing but a name, because the picture went to the
        model's host by the other road. If that name is not one this client is
        holding, the honest thing is not a warning on the terminal -- it is to
        stop the result saying `"ok": true`. Left alone, the model is told the
        photograph succeeded, is shown no photograph, and describes the room
        anyway: a wooden table, a white mug, a small green plant, none of which
        were ever in front of the rover. A tool that failed has to read as one.

        It happens for a dull reason worth naming. Two of these clients can hold
        the same port on Windows (see :class:`Frames`), so the rover's picture
        goes to whichever the operating system feels like, and the other one is
        left with a name and no frame.
        """
        if not isinstance(result, dict):
            return None, result
        name = result.get("image")
        if not isinstance(name, str):
            return None, result
        if self.frames is None:
            return None, dict(result, ok=False, image=None,
                              error="the picture had nowhere to be sent, so there "
                                    "is nothing to look at")
        jpeg = self.frames.take(name)
        if jpeg is None:
            self.indicator.say(f"  [{name} went to another frame server; "
                               f"is a second client running?]", err=True)
            return None, dict(result, ok=False, image=None,
                              error="the picture was taken but never arrived here, "
                                    "so there is nothing to look at")
        return jpeg, result

    async def _show_picture(self, jpeg: bytes) -> None:
        """Put one frame into the session, as a turn of its own.

        It goes up as a user turn rather than as part of the tool result, because
        that is the only shape this service accepts -- see PICTURE_LEAD_MS. The
        silence in front of the frame is not padding for timing, it is the price
        of admission, and without it the picture is refused and the model answers
        the question from imagination without ever saying it could not see.
        """
        lead = np.zeros(IN_RATE * PICTURE_LEAD_MS // 1000, dtype=np.float32)
        # A picture needs a turn committed by hand, and a commit by hand is
        # rejected outright while the service is deciding turns for itself --
        # "Internal service error: null", on both VAD types. So duplex mode hands
        # turn-taking back for about a second. The microphone is held shut for
        # that second rather than being allowed to talk into the frame's turn,
        # which is the one real cost of this: a question asked in the moment
        # after the shutter is not heard.
        if self.duplex:
            self.hold_mic = True
            await self._turn_detection(None)
        try:
            self._landed.clear()
            self._awaiting = None
            await self.push(_to_pcm16(lead))
            await self.send({"type": "input_image_buffer.append", "image": _b64(jpeg)})
            await self.send({"type": "input_audio_buffer.commit"})
        # And then wait for that turn to *finish*, which means waiting for the
        # transcription of the silence. A `response.create` sent while the turn
        # is still in progress is dropped without a word -- no error, no reply, a
        # conversation that stops with the rover having taken a photograph and
        # said nothing about it. Neither the commit's acknowledgement nor the
        # item's creation is late enough; both arrive while it is in progress.
            try:
                await asyncio.wait_for(self._landed.wait(), 10)
            except asyncio.TimeoutError:
                self.indicator.say("  [the picture's turn never finished]", err=True)
        finally:
            if self.duplex:
                await self._turn_detection(dict(DUPLEX_TURNS))
                self.hold_mic = False
        width, height = _jpeg_size(jpeg)
        if not self.quiet:
            self.indicator.say(f"  [sent {len(jpeg)} bytes of JPEG"
                               f"{f', {width}x{height}' if width else ''}]")

    async def drain(self) -> None:
        """Wait for any tool round trip still in flight, so a caller can stop cleanly."""
        if self._finishing is not None:
            await asyncio.shield(self._finishing)


async def _open(url: str, key: str, model: str):
    """Connect, or explain what went wrong in terms somebody can act on."""
    if websockets is None:
        raise SystemExit("websockets is not installed for this interpreter")
    uri = f"{url}?model={model}"
    headers = {"Authorization": f"Bearer {key}"}
    try:
        # websockets renamed this in 14.0 and the deployed machines are not all
        # on the same version.
        # close_timeout is short on purpose. Leaving is nearly always Ctrl-C, and
        # the library's ten-second default is ten seconds of a terminal that
        # looks hung while it waits politely for a close frame from a service
        # that has nothing more to say.
        try:
            return await websockets.connect(uri, additional_headers=headers,
                                            max_size=None, open_timeout=OPEN_TIMEOUT_S,
                                            close_timeout=3)
        except TypeError:
            return await websockets.connect(uri, extra_headers=headers,
                                            max_size=None, open_timeout=OPEN_TIMEOUT_S,
                                            close_timeout=3)
    except websockets.InvalidStatus as error:
        status = error.response.status_code
        if status in (401, 403):
            raise SystemExit(
                f"the service refused the key (HTTP {status}).\n"
                "  Check secrets/alibaba.key, and check the region: the mainland\n"
                "  host answers 401 for a key issued in Singapore. This is using\n"
                f"  {url}") from None
        raise SystemExit(f"{url} answered HTTP {status} rather than upgrading.") from None
    except (TimeoutError, asyncio.TimeoutError):
        raise SystemExit(f"{url} did not answer within {OPEN_TIMEOUT_S:.0f}s.") from None
    except OSError as error:
        raise SystemExit(f"cannot reach {url}: {error}") from None
