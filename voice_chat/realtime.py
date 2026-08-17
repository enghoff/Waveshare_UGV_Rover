"""The rover's voice with the model in Singapore and everything else on this desk.

[talk.py](talk.py) sends a finished utterance to a service on MEDIA, which
transcribes it, runs a 4B model, calls the rover's tools and synthesises a reply.
This does the same conversation against Alibaba's hosted omni model instead, over
the Realtime WebSocket protocol, and the difference is that there is no MEDIA in
it at all -- no Whisper, no local weights, no Kokoro, and no GPU.

    python voice_chat/realtime.py                    # full duplex; wear headphones
    python voice_chat/realtime.py --half-duplex      # push to talk, no barge-in
    python voice_chat/realtime.py --rover rpi.local:8769

The split is the one this repository already has. Audio capture and playback stay
on whatever desk has the microphone because they have to; the tools stay on the
rover because that is where the board and the camera are; and only the model is
remote. Nothing here performs a tool -- [rover_daemon.py](../rover_daemon/rover_daemon.py)
owns the hardware, [rover_tools.py](rover_tools.py) asks it what it can do, and
this passes calls along, exactly as `talk.py` does. What changed is the far end of
the socket.

Three things are worth knowing before running it.

**It is full duplex, so wear headphones.** Barge-in needs the microphone open
while the reply is playing, and an open microphone in a room with speakers hears
the rover's own voice, decides it is being interrupted, and stops itself
mid-sentence forever. Alibaba's own documentation says to wear headphones for
this reason. There is a crude suppressor for when you will not -- see
:class:`Ears` -- which is not acoustic echo cancellation and does not pretend to
be. `--half-duplex` is the other way: this client decides when a turn ended, the
microphone is shut while the rover talks, and silence never crosses the network.
That last property is worth something, since server-side turn detection can only
find the end of a turn in silence it was actually sent.

**The picture finds its own way home.** `look` does not hand its photograph back
through the conversation; it posts the JPEG straight to the model's host, which
is what keeps a 35kB frame off a desk that only has a microphone on it. That host
used to be MEDIA and is now this machine, so this serves the same `/frame`
contract itself (:class:`Frames`) and tells the rover where to find it on every
connection (:func:`point_camera_here`). Nothing has to be remembered at the
rover's end, which is the point: the address was a constant once, and when the
model moved the pictures kept going to where the model used to be.

**Face tracking still needs MEDIA**, and this does not change that. `look` is
served from here, but the face *detector* the tracking loop talks to is a
separate service on the GPU box; with it away, `start_tracking` now says so
rather than reporting success and holding still.

Credentials come from `secrets/alibaba.key` or `$DASHSCOPE_API_KEY`, and the key
is never printed. Dependencies are the same thin set as `talk.py` -- sounddevice,
numpy, websockets -- plus nothing.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import http.server
import json
import os
import queue
import struct
import sys
import threading
import time
import wave
from pathlib import Path
from typing import Any

try:
    import numpy as np
    import sounddevice as sd
    import websockets
except ImportError as _missing:
    # Nearly always a shell without the virtualenv on it rather than a machine
    # without the package: `python` outside the venv is whichever interpreter is
    # first on PATH, and on Windows that is usually the Store one, which has none
    # of this. Said as a sentence when run directly; re-raised when imported, so
    # that selftest.py can still skip the parts that need a sound card.
    if __name__ == "__main__":
        import sys as _sys
        print(f"{_missing.name} is not installed for {_sys.executable}.\n"
              "  This is almost always the wrong interpreter rather than a missing\n"
              "  package -- activate the virtualenv first:\n"
              "    .venv\\Scripts\\Activate.ps1        # PowerShell\n"
              "  or run it with that interpreter directly:\n"
              "    .venv\\Scripts\\python.exe voice_chat\\realtime.py\n"
              "  If it really is missing: pip install -r voice_chat/client-requirements.txt",
              file=_sys.stderr)
        raise SystemExit(1) from None
    raise

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import prompts
import rover_tools
from endpointing import BLOCK, BLOCK_MS, IN_RATE, Endpointer
from talk import Indicator  # the terminal-encoding gotcha is solved once, there

ROOT = Path(__file__).resolve().parent.parent

# The international endpoint. The mainland host refuses this account's key with a
# 401, which reads like a bad key and is a wrong region -- see the note in
# `secrets/` and in the omni bench's cloud backend.
ENDPOINT = os.environ.get(
    "QWEN_REALTIME_URL",
    "wss://dashscope-intl.aliyuncs.com/api-ws/v1/realtime")
MODEL = os.environ.get("QWEN_REALTIME_MODEL", "qwen3.5-omni-plus-realtime")
# Plus, despite costing about three times flash and despite the two being
# indistinguishable in docs/omni-step0.md -- both 90/90 typed and spoken, both
# 30/30 on the five extra tools. That was the *chat completions* pair. Their
# realtime namesakes are not the same models and do not behave alike, measured
# here against the mock rover, three samples a phrase:
#
#     first turn of a session      flash-realtime   plus-realtime
#     "Could you dim the lights a bit?"      0/3          3/3
#     "Can you look to your left?"           0/3          3/3
#     "Start tracking people."               0/3          3/3
#     "Switch the lights on."                1/3          3/3
#     "What do you see?"                  12/18          6/6
#
# Flash fails in the two ways this repository has spent the most words on. It
# announces without acting -- "I'll pan the camera to my left now", no call --
# and it *writes the tool call into its own speech*, which comes out as the rover
# saying "<set_lights> <parameter=level> 128 </parameter> </function>" out loud
# while doing nothing. Both are recoverable in principle, by the sniffer in
# server.py and by re-tuning the schemas; neither is worth doing when the model
# beside it is simply right. Flash is still one --model away for a sweep that
# wants to measure the difference properly.

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
OUT_RATE = 24000  # what the model synthesises at; input stays at 16k
# 0.2 for the same reason the local service uses it: whether the model *acts*
# turned out to be a sampled decision, and at 0.7 half the tool calls that should
# have happened did not. See the table above TEMPERATURE in server.py.
TEMPERATURE = float(os.environ.get("QWEN_REALTIME_TEMPERATURE", "0.2"))

TRACE = os.environ.get("QWEN_REALTIME_TRACE", "") not in ("", "0", "false", "False")

# A session is capped at two hours by the service, and it says so by closing the
# socket. Warned about a little early so a conversation ending is a sentence on
# the terminal rather than a stack trace.
SESSION_LIMIT_S = 120 * 60
SESSION_WARN_S = SESSION_LIMIT_S - 300

# How much audio goes in one message. A WebSocket frame per 20ms block is a lot
# of frames for a conversation that lasts minutes; 100ms is still five updates a
# second, which is far below anything a person can hear as latency.
UPLOAD_MS = 100
UPLOAD_BLOCKS = max(1, UPLOAD_MS // BLOCK_MS)

# A frame is held only as long as the turn that asked for it needs, and by name
# rather than "the latest one" -- the camera is on a gimbal that sweeps while
# tracking runs, so last turn's picture is of somewhere the rover is no longer
# pointing. Both numbers are the voice service's.
FRAME_TTL_S = 60.0
MAX_FRAMES = 4
# The service takes JPEG up to 256KB once base64'd. The rover's camera is 640x480
# and a frame is ~35KB, so this is a ceiling for anything else that posts.
MAX_FRAME_BYTES = 180 * 1024

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


def point_camera_here(rover: rover_tools.RoverClient | None,
                      frames: Frames | None) -> None:
    """Tell the rover where to post its pictures: here.

    The rover does not hand `look`'s photograph back through the conversation.
    It posts the JPEG straight to the model's host, which is what keeps a 35kB
    frame off a desk that only has a microphone on it. That destination used to
    be a constant baked into however the daemon was started, and a constant is
    the wrong shape for it -- when the model moved off MEDIA the pictures kept
    going to MEDIA, and `look` failed with "No route to host" while every other
    tool on the rover worked perfectly.

    So it is said out loud on every connection, using the address this machine's
    own socket to the daemon is bound to. That is right by construction: the
    kernel already picked the interface that reaches the rover, and it picks a
    different one once the rover is off its dock.
    """
    if rover is None:
        return
    if frames is None:
        rover.call("set_vision", {"address": None})
        return
    here = rover.local_address()
    if here is None:
        print("  cannot tell which address the rover sees this machine on;\n"
              "  'look' will only work if the daemon was already pointed here",
              file=sys.stderr)
        return
    where = f"{here}:{frames.server_address[1]}"
    answer = rover.call("set_vision", {"address": where})
    if answer.get("ok"):
        print(f"pictures: the rover will post to {answer.get('vision', where)}")
        return
    # An older daemon has no such call. Say exactly what to do about it rather
    # than leaving `look` to fail later with a routing error, which is what this
    # whole function exists to stop happening.
    print(f"  this rover daemon does not take set_vision ({answer.get('error')}).\n"
          f"  'look' will post wherever it was started pointing. To fix it:\n"
          f"    scp rover_daemon/rover_daemon.py rpi:~/ugv/ && ssh rpi 'sudo systemctl "
          f"restart rover-daemon'\n"
          f"  or restart the daemon with: --vision {where}", file=sys.stderr)


def api_key() -> str:
    """The DashScope key, from the environment or from `secrets/`, never printed."""
    from_env = os.environ.get("DASHSCOPE_API_KEY", "").strip()
    if from_env:
        return from_env
    path = ROOT / "secrets" / "alibaba.key"
    if not path.exists():
        raise SystemExit(
            f"no API key: set $DASHSCOPE_API_KEY or put one line in {path}\n"
            "  (secrets/ is gitignored)")
    key = path.read_text(encoding="utf-8").strip()
    if not key:
        raise SystemExit(f"{path} is empty")
    return key


def _b64(raw: bytes) -> str:
    return base64.b64encode(raw).decode("ascii")


def _to_pcm16(audio: np.ndarray) -> bytes:
    """Float samples as the 16kHz mono s16le the service reads."""
    return (np.clip(audio, -1.0, 1.0) * 32767).astype("<i2").tobytes()


def _from_pcm16(raw: bytes) -> np.ndarray:
    return np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0


def _jpeg_size(data: bytes) -> tuple[int | None, int | None]:
    """Width and height out of a JPEG's frame header, without decoding it.

    Only so the log line can say what was posted. Deliberately not PIL: this
    client's whole dependency list is three packages, and reading two big-endian
    shorts out of a marker is not worth a fourth.
    """
    index = 2
    while index + 9 < len(data):
        if data[index] != 0xFF:
            index += 1
            continue
        marker = data[index + 1]
        # SOF0..SOF15, skipping the four that are not start-of-frame markers.
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            height, width = struct.unpack(">HH", data[index + 5:index + 9])
            return width, height
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            index += 2
            continue
        length = struct.unpack(">H", data[index + 2:index + 4])[0]
        index += 2 + length
    return None, None


class Speaker:
    """Playback that can be thrown away mid-sentence.

    `sounddevice`'s blocking `write` is the obvious way to play a stream and the
    wrong one here, because interrupting means dropping audio that has been
    received and not yet heard -- and a blocking write has already committed it.
    So this fills from a buffer under a callback, and :meth:`flush` is a barge-in:
    everything not yet handed to the card stops existing.

    It also counts what was actually played, which is not bookkeeping for its own
    sake. When a reply is cut off, the model's idea of what it said is the whole
    reply, and the only way to correct that is to tell it how many milliseconds
    of it were audible.
    """

    def __init__(self, device: int | None = None, rate: int = OUT_RATE) -> None:
        self.rate = rate
        self.device = device
        self._buffer = np.zeros(0, dtype=np.float32)
        self._lock = threading.Lock()
        self._queued = 0    # samples ever accepted for the current response
        self._level = 0.0   # RMS of the last block handed to the card
        # The card is not opened until `start`, so that the part worth testing --
        # what is buffered, what is thrown away, and how much of a reply was
        # audible before it was cut off -- can be tested on a machine with no
        # sound at all. Same reason endpointing.py touches no audio device.
        self.stream: sd.OutputStream | None = None

    def _fill(self, out, frames, _time, _status) -> None:
        with self._lock:
            take = min(frames, len(self._buffer))
            out[:take, 0] = self._buffer[:take]
            out[take:, 0] = 0.0
            block = self._buffer[:take]
            self._buffer = self._buffer[take:]
        self._level = float(np.sqrt(np.mean(block ** 2))) if take else 0.0

    def start(self) -> None:
        self.stream = sd.OutputStream(
            samplerate=self.rate, channels=1, dtype="float32",
            device=self.device, callback=self._fill)
        self.stream.start()

    def close(self) -> None:
        if self.stream is None:
            return
        # Dropped and aborted rather than stopped. `stop` waits for what is
        # queued to finish playing, and the usual reason to be closing is that
        # somebody pressed Ctrl-C in the middle of a sentence -- which is a
        # request to stop talking, not to finish the thought first.
        self.flush()
        try:
            self.stream.abort()
            self.stream.close()
        except Exception:
            pass
        self.stream = None

    def write(self, audio: np.ndarray) -> None:
        with self._lock:
            self._buffer = np.concatenate([self._buffer, audio])
            self._queued += len(audio)

    def begin(self) -> None:
        """A new response is starting; what was played before it does not count."""
        with self._lock:
            self._queued = len(self._buffer)

    @property
    def busy(self) -> bool:
        with self._lock:
            return len(self._buffer) > 0

    @property
    def level(self) -> float:
        """RMS of what is coming out of the speaker right now."""
        return self._level

    def remaining_s(self) -> float:
        with self._lock:
            return len(self._buffer) / self.rate

    def played_ms(self) -> int:
        """Milliseconds of the current response that reached the card."""
        with self._lock:
            return int(max(self._queued - len(self._buffer), 0) * 1000 / self.rate)

    def flush(self) -> float:
        """Drop everything unheard. Returns the seconds of audio thrown away."""
        with self._lock:
            unheard = len(self._buffer)
            # Forgotten, not merely dropped. `played_ms` is queued-minus-waiting,
            # so throwing audio away without also forgetting it was queued makes
            # a barge-in report that the whole reply was heard -- which is the
            # one thing this number exists to prevent it saying.
            self._queued -= unheard
            self._buffer = np.zeros(0, dtype=np.float32)
        return unheard / self.rate


class Frames(http.server.ThreadingHTTPServer):
    """`POST /frame` on this machine, because the service that used to serve it is gone.

    The rover's `look` does not send the picture through the conversation. It
    posts the JPEG straight to the model's host and returns nothing but the name
    it was filed under, which keeps a 35KB frame off the desk that only has a
    microphone on it. That road led to MEDIA, and on this path MEDIA does not
    exist -- so the desk becomes the host, holds the frame under a name, and
    forwards it into the session when the tool result comes back naming it.

    The contract is the voice service's, unchanged, so the daemon needs no edit:

        POST /frame   body: one JPEG
          -> {"ok": true, "image": "frame-7", "w": 640, "h": 480}

    **Threading here is not about throughput.** The rover posts over one
    kept-open connection, deliberately, and a plain `HTTPServer` handles requests
    one at a time inside `serve_forever` -- so after a single picture it is
    parked inside that connection's handler, blocked on a request line that will
    not arrive until the next `look`. Nothing else can be accepted, and
    `shutdown()` never returns, because the loop it is waiting on is the one that
    is blocked. What that looks like from outside is a conversation that ends
    fine until somebody asks the rover what it can see, after which Ctrl-C hangs
    the terminal.
    """

    # Not reusable, deliberately, and this is the one place where the usual
    # advice is backwards. On Windows SO_REUSEADDR does not mean "reclaim a port
    # left in TIME_WAIT", it means *share*: a second process binds the same port
    # happily and which of the two a given connection reaches is anyone's guess.
    # A leftover client from an earlier run therefore steals the rover's
    # pictures, and the running one is handed a frame name it is not holding.
    # Refusing to start is the better failure, and `main` prints it.
    allow_reuse_address = False
    daemon_threads = True  # inherited, and load-bearing: see above

    def __init__(self, port: int = 8767, host: str = "0.0.0.0") -> None:
        super().__init__((host, port), _FrameHandler)
        self._frames: dict[str, tuple[bytes, float]] = {}
        self._seq = 0
        self._lock = threading.Lock()
        self.posted = 0

    def stash(self, jpeg: bytes) -> str:
        with self._lock:
            now = time.monotonic()
            for name, (_data, at) in list(self._frames.items()):
                if now - at > FRAME_TTL_S:
                    del self._frames[name]
            while len(self._frames) >= MAX_FRAMES:
                del self._frames[min(self._frames, key=lambda n: self._frames[n][1])]
            self._seq += 1
            self.posted += 1
            name = f"frame-{self._seq}"
            self._frames[name] = (jpeg, now)
            return name

    def take(self, name: str) -> bytes | None:
        """The frame under this name, removed. One picture answers one question."""
        with self._lock:
            found = self._frames.pop(name, None)
        return found[0] if found else None

    def serve_in_background(self) -> None:
        threading.Thread(target=self.serve_forever, daemon=True).start()


class _FrameHandler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    # An idle kept-open connection costs a parked thread, and a rover that is
    # restarted a few times over an afternoon leaves one behind each time. On
    # timeout the handler simply closes the connection, and the rover's next
    # picture reconnects -- which its VisionLink already expects, since a stale
    # keep-alive is a retry there rather than a lost frame.
    timeout = 300

    def log_message(self, *_args) -> None:
        pass  # the conversation owns the terminal

    def _reply(self, status: int, payload: dict) -> None:
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:
        if self.path.rstrip("/") not in ("/frame", ""):
            self._reply(404, {"ok": False, "error": "only /frame"})
            return
        length = int(self.headers.get("Content-Length") or 0)
        data = self.rfile.read(length) if length else b""
        if not data.startswith(b"\xff\xd8"):
            self._reply(400, {"ok": False, "error": "not a JPEG"})
            return
        if len(data) > MAX_FRAME_BYTES:
            self._reply(413, {"ok": False,
                              "error": f"{len(data)} bytes is over the "
                                       f"{MAX_FRAME_BYTES} the model accepts"})
            return
        name = self.server.stash(data)
        width, height = _jpeg_size(data)
        self._reply(200, {"ok": True, "image": name, "w": width, "h": height,
                          "bytes": len(data)})

    def do_GET(self) -> None:
        if self.path.rstrip("/") == "/health":
            self._reply(200, {"ok": True, "frames": self.server.posted})
        else:
            self._reply(404, {"ok": False, "error": "only /frame and /health"})


class Ears:
    """Whether a microphone block should be uploaded while the rover is talking.

    This is a suppressor, not an echo canceller. It has no model of the room and
    no reference alignment; it compares how loud the microphone is against how
    loud the speaker is right now, and passes the microphone only when it is
    clearly louder. That is enough to stop a rover interrupting itself at
    conversational volume, and it is not enough for a loud room, a close speaker,
    or a reply the user talks over quietly.

    The correct fix is headphones, which Alibaba's own documentation recommends
    for this exact failure, or an AEC -- and an AEC is a real dependency and a
    reference signal and a delay estimate, which is a different piece of work
    than this one. Until then the honest thing is to say which of the two you are
    getting: `--no-echo-guard` gives you none of this and assumes headphones.
    """

    def __init__(self, speaker: Speaker | None, factor: float, on: bool = True) -> None:
        self.speaker = speaker
        self.factor = factor
        self.on = on

    def hears(self, rms: float) -> bool:
        if not self.on or self.speaker is None:
            return True
        playing = self.speaker.level
        if playing <= 0.0:
            return True
        return rms > playing * self.factor


class Session:
    """One conversation with the hosted model, and the rover behind it.

    Everything that crosses the socket goes through here, so the protocol is in
    one place and the two loops around it -- one pushing the microphone, one
    draining the model -- stay short enough to read.
    """

    def __init__(self, ws, rover: rover_tools.RoverClient | None,
                 frames: Frames | None, speaker: Speaker | None,
                 indicator: Indicator, duplex: bool, model: str,
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
            "instructions": prompts.system_prompt(vision=vision),
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
                                            max_size=None, open_timeout=15,
                                            close_timeout=3)
        except TypeError:
            return await websockets.connect(uri, extra_headers=headers,
                                            max_size=None, open_timeout=15,
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
        raise SystemExit(f"{url} did not answer within 15s.") from None
    except OSError as error:
        raise SystemExit(f"cannot reach {url}: {error}") from None


async def converse(url: str, key: str, model: str, device: int | None,
                   out_device: int | None, rover: rover_tools.RoverClient | None,
                   frames: Frames | None, duplex: bool, echo_guard: bool,
                   echo_factor: float) -> None:
    blocks: queue.Queue[np.ndarray] = queue.Queue()
    indicator = Indicator()

    def on_audio(indata, _frames, _time, status) -> None:
        if status:
            print(f"\n  (input {status})", file=sys.stderr)
        blocks.put(indata[:, 0].copy())

    print(f"connecting to {model} ...", flush=True)
    async with await _open(url, key, model) as ws:
        speaker = Speaker(out_device)
        speaker.start()
        session = Session(ws, rover, frames, speaker, indicator, duplex, model)

        async def receive() -> None:
            async for message in ws:
                await session.handle(json.loads(message))

        # Started before anything is configured, not after. Everything this
        # client waits to be told -- that the session took, that a turn landed --
        # is told through here, so a setup that runs before the reader does is a
        # setup that waits for events nobody is listening for.
        reader = asyncio.create_task(receive())

        # Asked for afresh on every connection rather than cached: the daemon is
        # the authority on what this rover can do, and it may have been restarted
        # with more tools since the last time anybody looked.
        tools = await asyncio.to_thread(rover.tools) if rover is not None else []
        vision = any(t.get("function", {}).get("name") == "look" for t in tools)
        await session.configure(tools, vision=vision)

        print(f"connected. {'full duplex -- wear headphones' if duplex else 'push to talk'}"
              f", Ctrl-C to quit.\n")

        stream = sd.InputStream(samplerate=IN_RATE, blocksize=BLOCK, channels=1,
                                dtype="float32", device=device, callback=on_audio)
        ears = Ears(speaker, echo_factor, echo_guard and duplex)

        async def microphone() -> None:
            endpointer = Endpointer()
            outgoing: list[np.ndarray] = []
            speaking = False
            started = time.monotonic()
            warned = False

            while True:
                try:
                    block = blocks.get_nowait()
                except queue.Empty:
                    await asyncio.sleep(0.005)
                    continue

                if not warned and time.monotonic() - started > SESSION_WARN_S:
                    warned = True
                    indicator.say("  [this session is capped at two hours and is "
                                  "nearly there; restart to keep talking]")

                rms = float(np.sqrt(np.mean(block ** 2)))

                if duplex:
                    # Everything goes, because the service is the one deciding
                    # when a turn ended and it cannot decide that from audio it
                    # was never sent. Two things are held back: whatever the echo
                    # guard judges to be the rover's own voice, and everything at
                    # all while a picture is being handed over -- that second one
                    # is a turn of its own and anything said into it joins it.
                    if session.hold_mic:
                        outgoing = []
                        indicator.set("thinking")
                        continue
                    if ears.hears(rms):
                        outgoing.append(block)
                    if len(outgoing) >= UPLOAD_BLOCKS:
                        await session.push(_to_pcm16(np.concatenate(outgoing)))
                        outgoing = []
                    if not session.responding and not speaker.busy:
                        indicator.set("listening")
                    continue

                # Half duplex: deaf on purpose while the reply plays, because an
                # open microphone here endpoints the rover's own voice.
                if session.responding or speaker.busy:
                    indicator.set("speaking" if speaker.busy else "thinking")
                    endpointer.reset()
                    continue

                utterance = endpointer.push(block, rms)
                if endpointer.speaking and not speaking:
                    # The rising edge. `voiced` already holds the preroll, which
                    # is the 300ms without which "start" arrives as "art".
                    speaking = True
                    await session.push(_to_pcm16(np.concatenate(endpointer.voiced)))
                    indicator.set("hearing")
                elif endpointer.speaking:
                    outgoing.append(block)
                    if len(outgoing) >= UPLOAD_BLOCKS:
                        await session.push(_to_pcm16(np.concatenate(outgoing)))
                        outgoing = []

                if speaking and not endpointer.speaking:
                    # The falling edge, which is either a turn or a cough. The
                    # endpointer answers with the utterance for the first and
                    # nothing for the second, and a cough that has already been
                    # uploaded has to be taken back or it becomes the next turn.
                    speaking = False
                    if outgoing:
                        await session.push(_to_pcm16(np.concatenate(outgoing)))
                        outgoing = []
                    if utterance is None:
                        await session.discard()
                        indicator.set("listening")
                    else:
                        await session.commit()
                        indicator.set("thinking")
                elif not speaking:
                    indicator.set("listening")

        with stream, indicator:
            indicator.set("listening")
            try:
                await asyncio.gather(reader, microphone())
            finally:
                reader.cancel()
                speaker.close()


async def smoke(url: str, key: str, model: str, wavs: list[str],
                rover: rover_tools.RoverClient | None, frames: Frames | None,
                out: str | None, duplex: bool = False) -> int:
    """The whole path with a file where the microphone goes, and no speaker.

    This is what makes the client testable at all. A conversation needs a room, a
    microphone and somebody to talk into it; this needs a WAV, and it exercises
    the same session setup, the same schemas, the same tool dispatch and the same
    picture forwarding. What it cannot test is the two things that only exist in
    a room -- echo and interruption.
    """
    indicator = Indicator()
    async with await _open(url, key, model) as ws:
        session = Session(ws, rover, frames, None, indicator, duplex, model)
        heard: list[np.ndarray] = []

        async def receive() -> None:
            async for message in ws:
                event = json.loads(message)
                if event.get("type") == "response.audio.delta":
                    heard.append(_from_pcm16(base64.b64decode(event["delta"])))
                await session.handle(event)

        # Before configuring, for the reason given in `converse`.
        receiver = asyncio.create_task(receive())

        tools = await asyncio.to_thread(rover.tools) if rover is not None else []
        vision = any(t.get("function", {}).get("name") == "look" for t in tools)
        await session.configure(tools, vision=vision)
        print(f"{model}: {len(tools)} tools, "
              f"{len(prompts.system_prompt(vision=vision))} chars of prompt"
              f"{', server-side turns' if duplex else ''}")

        try:
            for path in wavs:
                with wave.open(path, "rb") as source:
                    if source.getframerate() != IN_RATE or source.getnchannels() != 1:
                        print(f"  {path}: want 16kHz mono, got "
                              f"{source.getframerate()}Hz "
                              f"{source.getnchannels()}ch", file=sys.stderr)
                        return 1
                    raw = source.readframes(source.getnframes())
                print(f"\n> {Path(path).name} ({len(raw) / 2 / IN_RATE:.1f}s)")
                # In the same 100ms pieces the microphone would send, so the
                # service sees the shape of a real turn and not one long blob.
                step = IN_RATE * UPLOAD_MS // 1000 * 2
                for start in range(0, len(raw), step):
                    await session.push(raw[start:start + step])
                if duplex:
                    # Nothing is committed by hand here: the service is watching
                    # for the end of the turn, and the only way to show it one is
                    # to send the silence that follows. Two seconds, against the
                    # 800ms it waits for by default.
                    quiet = _to_pcm16(np.zeros(IN_RATE * UPLOAD_MS // 1000,
                                               dtype=np.float32))
                    for _ in range(2000 // UPLOAD_MS):
                        await session.push(quiet)
                        await asyncio.sleep(0.02)
                else:
                    await session.commit()

                # A turn is over when nothing is being said, nothing is owed and
                # nothing is out at the rover -- see `Session.idle`. Stopping at
                # the first `response.done` instead is the bug this replaced: a
                # tool call answers in two responses, and cutting between them
                # sends the next question into the middle of the first answer,
                # after which every reply is one turn late and still plausible.
                deadline = time.monotonic() + 60
                await asyncio.sleep(2.0 if duplex else 0.4)
                while time.monotonic() < deadline:
                    await session.drain()
                    if session.idle:
                        break
                    await asyncio.sleep(0.1)
                else:
                    print("  (gave up waiting for the turn to end)", file=sys.stderr)
        finally:
            receiver.cancel()

        if out and heard:
            audio = np.concatenate(heard)
            with wave.open(out, "wb") as sink:
                sink.setnchannels(1)
                sink.setsampwidth(2)
                sink.setframerate(OUT_RATE)
                sink.writeframes(_to_pcm16(audio))
            print(f"\n{len(audio) / OUT_RATE:.1f}s of reply audio -> {out}")

        print(f"\ncalls: {[name for name, _ in session.calls] or 'none'}")
        if session.usage:
            print(f"usage: {json.dumps(session.usage)}")
        if session.errors:
            print(f"errors: {session.errors}", file=sys.stderr)
            return 1
    return 0


def main() -> int:
    # The model speaks typographic English -- curly apostrophes, dashes -- and a
    # redirected stdout on Windows is cp1252, which turns "don't" into "don?t" on
    # a good day and raises UnicodeEncodeError on a bad one, mid-conversation.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass

    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--url", default=ENDPOINT)
    parser.add_argument("--model", default=MODEL)
    parser.add_argument("--rover", default="auto", metavar="HOST:PORT",
                        help='the rover daemon; "auto" looks for it, "none" for no tools')
    # Duplex is the default. `--duplex` is kept as a no-op because it was the way
    # to ask for this and there is no reason for that to start being an error.
    parser.add_argument("--duplex", action="store_true",
                        help="the default; kept so the old spelling still works")
    parser.add_argument("--half-duplex", action="store_true",
                        help="no barge-in: this client decides when a turn ended, "
                             "the microphone is shut while the rover talks, and "
                             "silence never crosses the network")
    parser.add_argument("--no-echo-guard", action="store_true",
                        help="with --duplex, do not suppress the microphone while "
                             "the rover is talking (assumes headphones)")
    parser.add_argument("--echo-factor", type=float, default=2.5,
                        help="how much louder than the speaker the microphone must "
                             "be to count as somebody interrupting")
    parser.add_argument("--frame-port", type=int, default=8767,
                        help="where this client answers the daemon's /frame POSTs")
    parser.add_argument("--no-frames", action="store_true",
                        help="do not serve /frame; 'look' will fail")
    parser.add_argument("--input-device", type=int, default=None)
    parser.add_argument("--output-device", type=int, default=None)
    parser.add_argument("--list-devices", action="store_true")
    parser.add_argument("--smoke", nargs="+", metavar="WAV",
                        help="no microphone: send these 16kHz mono WAVs as turns "
                             "and print what happened")
    parser.add_argument("--out", metavar="WAV", default=None,
                        help="with --smoke, write the reply audio here")
    args = parser.parse_args()

    if args.list_devices:
        print(sd.query_devices())
        return 0

    key = api_key()

    rover = None
    if args.rover != "none":
        # Probed rather than assumed, and searched for rather than probed at one
        # address: the rover answers on wlan0 or eth0 depending on whether it is
        # plugged in, and picking the wrong one looks exactly like a rover that
        # is not there.
        if args.rover == "auto":
            rover = rover_tools.discover()
        else:
            rover = rover_tools.RoverClient(args.rover)
            if not rover.probe():
                rover.close()
                rover = None
        if rover is not None:
            print(f"rover daemon at {rover.describe()}")
        else:
            where = ("any of " + ", ".join(rover_tools.DEFAULT_CANDIDATES)
                     if args.rover == "auto" else args.rover)
            print(f"  no rover daemon on {where}; no tools.\n"
                  f"  Start one with: python voice_chat/mock_rover.py --vision",
                  file=sys.stderr)

    frames = None
    if not args.no_frames:
        try:
            frames = Frames(args.frame_port)
            frames.serve_in_background()
            print(f"pictures accepted on http://0.0.0.0:{args.frame_port}/frame")
        except OSError as error:
            print(f"  cannot serve /frame on {args.frame_port} ({error}); "
                  f"'look' will fail", file=sys.stderr)

    # Said before the tool list is asked for, because it changes the tool list:
    # `look` exists only while there is somewhere for a picture to go.
    point_camera_here(rover, frames)

    duplex = not args.half_duplex

    try:
        if args.smoke:
            return asyncio.run(smoke(args.url, key, args.model, args.smoke,
                                     rover, frames, args.out, duplex))
        asyncio.run(converse(args.url, key, args.model, args.input_device,
                             args.output_device, rover, frames, duplex,
                             not args.no_echo_guard, args.echo_factor))
    except KeyboardInterrupt:
        print("\nbye")
    except websockets.ConnectionClosed as error:
        print(f"\nthe service closed the connection ({error}).\n"
              "  A session is capped at two hours; if it ran that long, this is why.",
              file=sys.stderr)
        return 1
    finally:
        if frames is not None:
            frames.shutdown()
            frames.server_close()  # or the port stays claimed until the shell dies
        if rover is not None:
            rover.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
