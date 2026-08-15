"""Voice chat from the rover: Bluetooth headset in, Bluetooth speaker out.

Same conversation as [talk.py](talk.py) and the same endpointer, but everything
about the audio devices is different, so it is a separate client rather than a
flag on that one.

    mic  -- WI-XB400 over HFP/mSBC (16kHz, exactly what Whisper wants)
    out  -- JBL Flip over A2DP
    both -- PipeWire on the Pi, models on MEDIA over the LAN

Why it drives PipeWire through pipes instead of using sounddevice: the Pi has no
PortAudio and no ALSA-to-PipeWire bridge, and installing either needs an apt
password we do not have from a script. It would not help much anyway -- the two
Bluetooth devices exist only as PipeWire nodes, so PortAudio would be talking to
raw ALSA and would not see them. `pw-record` and `pw-play` are already on the
box, take the exact sample format the protocol wants, and are addressed by node
name, which survives a reconnect where a node id does not. This is the same
shape as the camera in `face_tracking/track_face_pi.py`, which drives `v4l2-ctl`
for the same reason.

    python3 talk_pi.py                       # defaults are the two devices above
    python3 talk_pi.py --list                # what PipeWire currently has
    python3 talk_pi.py --mic auto --speaker auto     # whatever wpctl says is default

Ctrl-C to quit.
"""

from __future__ import annotations

import argparse
import ctypes
import fcntl
import json
import queue
import signal
import subprocess
import sys
import threading
import time

import numpy as np

from endpointing import BLOCK, IN_RATE, Endpointer
from wsclient import WebSocket, WebSocketError

DEFAULT_URL = "ws://192.168.1.3:8767/ws"

# Addressed by node name, not id: ids are handed out afresh every time a device
# reconnects, and these two do that whenever they are switched off.
DEFAULT_MIC = "bluez_input.30_53_C1_A4_66_86.0"  # WI-XB400 headset mic
DEFAULT_SPEAKER = "bluez_output.20_18_5B_7C_2E_44.1"  # JBL Flip

BYTES_PER_BLOCK = BLOCK * 2  # s16le mono
# Read the microphone in bulk. pw-record hands over ~100ms at a time, so this
# adds no latency -- it only stops us waking up once per 20ms block, which is
# what was chopping up playback. See Recorder._read.
READ_BYTES = BYTES_PER_BLOCK * 8

# pw-play's own buffer. Larger than a desktop would want, because an ARMv6 that
# is also encoding SBC has scheduling hiccups, and an underrun on A2DP is an
# audible click rather than a dropped sample.
PLAY_LATENCY_MS = 200
# Queue-to-heard delay once the stream is running: pw-play's buffer plus the
# JBL's own A2DP latency. Measured on the rover by putting a burst through the
# JBL and finding it in the headset mic -- 151ms, with margin for jitter.
PLAY_LATENCY_S = 0.45
# The same measurement for a pw-play that has never played anything: 2057ms,
# nearly all of it PipeWire negotiating the A2DP stream and the JBL filling its
# buffer. Paying that on the first reply is what made the assistant answer
# itself -- the mute expired while its last sentence was still coming out. The
# speaker is now built at startup so this is normally spent before anyone
# speaks, but a device that reconnects mid-conversation pays it again.
COLD_LATENCY_S = 2.5
# Silence written at startup, so the stream is established rather than merely
# opened by the time there is anything to say.
PRIME_S = 0.3
# Room echo and the speaker ringing down, on top of whichever of those applies.
MUTE_TAIL_S = 0.35
# How much room to give the pipe feeding pw-play. 1MB is the default ceiling in
# /proc/sys/fs/pipe-max-size and holds ~21s at 24kHz, comfortably more than any
# one reply -- see widen_pipe() for why this is the fix rather than a tweak.
F_SETPIPE_SZ = 1031
PIPE_BYTES = 1024 * 1024
# How much of an utterance goes in one WebSocket frame. Small enough that the
# masking XOR is a short burst rather than a long one -- see turn().
SEND_CHUNK = 32 * 1024
# What the service synthesises at. Confirmed by the "start" event on every turn;
# only needed up front so the speaker can be warmed before the first one.
OUT_RATE = 24000

# How long to wait for a Bluetooth device's PipeWire node to turn up. Generous:
# bluetoothd reports a successful connection several seconds before WirePlumber
# has finished building the card, and the headset is often switched on last.
NODE_WAIT_S = 30.0

# The rover's wifi does drop. Reconnect rather than ending the conversation.
RETRY_S = 2.0
# A turn is STT plus a 4B model plus TTS; warm it is under two seconds, but a
# cold or contended card is slower. Longer than this and something has gone.
TURN_TIMEOUT_S = 90.0
# Silence longer than this while nothing is happening means the link is dead
# even though the socket has not noticed.
IDLE_TIMEOUT_S = 5.0
# ...but the very first block takes longer than that to arrive: pw-record has to
# start and the headset's SCO link has to come up, which together run past 5s and
# were being reported as "no audio from the microphone" on every startup.
FIRST_BLOCK_TIMEOUT_S = 20.0


class Stopping(Exception):
    """SIGTERM arrived; unwind and let the audio processes go."""


def die_with_parent() -> None:
    """Ask the kernel to SIGTERM this child when its parent goes.

    Without it, a killed run leaves pw-record holding the SCO link open and the
    headset is unusable until it is found and killed by hand -- the same failure
    the camera had with v4l2-ctl.
    """
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGTERM, 0, 0, 0)
    except Exception:  # non-Linux, or no libc; the atexit path still covers most cases
        pass


def stop_on_sigterm() -> None:
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(Stopping()))


def end(proc: subprocess.Popen) -> None:
    """Terminate, then insist. A stuck pw-play has been seen to ignore SIGTERM,
    and one left holding the microphone makes the headset unusable until it is
    hunted down by hand."""
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        proc.kill()


def widen_pipe(handle) -> int:
    """Let the kernel hold a whole reply, rather than 64KB of one.

    Belt and braces, not the fix -- the choppiness was Recorder._read waking up
    50 times a second, see there. But a default pipe is 64KB, which is only 1.4s
    at 24kHz, so a reply still has to be topped up mid-playback and every top-up
    depends on this process being scheduled promptly. Widening it removes that
    dependency for nothing.

    Capped by /proc/sys/fs/pipe-max-size (1MB by default, ~21s of audio) and
    needs no privileges up to that.
    """
    try:
        return fcntl.fcntl(handle.fileno(), F_SETPIPE_SZ, PIPE_BYTES)
    except (OSError, AttributeError):
        return 0  # not Linux, or the cap is lower; playback still works


def nodes() -> list[tuple[str, str]]:
    """(media.class, node.name) for every audio node PipeWire currently has."""
    try:
        dump = subprocess.run(
            ["pw-dump"], capture_output=True, timeout=15, check=True
        ).stdout
    except (OSError, subprocess.SubprocessError) as error:
        raise SystemExit(f"cannot talk to PipeWire: {error}")
    found = []
    for obj in json.loads(dump):
        props = (obj.get("info") or {}).get("props") or {}
        media = props.get("media.class", "")
        if obj.get("type", "").endswith("Node") and media.startswith("Audio"):
            found.append((media, props.get("node.name", "?")))
    return sorted(found)


def require(name: str, want: str, wait_s: float = NODE_WAIT_S) -> str | None:
    """Wait for a node to exist before we build a pipeline around it.

    Waiting rather than checking once, because a Bluetooth device is usually
    switched on *after* the client is started, and because WirePlumber takes
    several seconds to build the card even once bluetoothd says the connection
    succeeded -- long enough that an immediate check loses the race.

    "auto" means let PipeWire pick its default, which is what `wpctl
    set-default` controls; that is returned as None, the absence of a --target.
    """
    if name == "auto":
        return None
    deadline = time.monotonic() + wait_s
    announced = False
    while True:
        available = nodes()
        if any(node == name for _, node in available):
            return name
        if time.monotonic() >= deadline:
            listing = "\n".join(f"    {media:<22} {node}" for media, node in available)
            raise SystemExit(
                f"no PipeWire {want} called {name!r} after {wait_s:.0f}s.\n"
                f"  Is the device switched on? bluetoothctl connect ...\n"
                f"  What PipeWire has right now:\n{listing}"
            )
        if not announced:
            print(f"waiting for {want} {name} ...")
            announced = True
        time.sleep(1.0)


class Recorder:
    """pw-record in a pipe, delivered as fixed 20ms blocks.

    A thread rather than the main loop, so that a scheduling hiccup shows up as
    a burst of blocks -- which the endpointer counts correctly -- instead of a
    gap that a wall clock would misread as the end of a sentence.
    """

    def __init__(self, target: str | None) -> None:
        command = [
            "pw-record", "--raw",
            "--rate", str(IN_RATE),
            "--channels", "1",
            "--format", "s16",
        ]
        if target:
            command += ["--target", target]
        command.append("-")
        self.proc = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            preexec_fn=die_with_parent,
        )
        self.blocks: queue.Queue[tuple[np.ndarray, np.ndarray]] = queue.Queue()
        self.stderr: list[str] = []
        self.delivered = False
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._watch, daemon=True).start()

    def _read(self) -> None:
        """Take whatever pw-record has, and convert it in one go.

        Asking for one 20ms block at a time forced 50 wakeups a second and 50
        sets of small numpy allocations, and that -- not CPU load, not the
        Bluetooth link -- is what chopped up playback: measured against a
        `cat`-fed tone, this loop alone put 36 holes into 15s of audio (11.7%),
        while an idle pw-play stream on the same sink put in none. pw-record
        already delivers in ~100ms chunks, so reading big costs no latency; it
        just stops waking up for each block separately.
        """
        pending = b""
        while True:
            chunk = self.proc.stdout.read(READ_BYTES)
            if not chunk:
                break
            pending += chunk
            count = len(pending) // BYTES_PER_BLOCK
            if not count:
                continue
            usable = pending[:count * BYTES_PER_BLOCK]
            pending = pending[count * BYTES_PER_BLOCK:]
            samples = (np.frombuffer(usable, dtype="<i2").astype(np.float32)
                       / 32768.0).reshape(count, BLOCK)
            # One RMS for the whole batch rather than one per block; the
            # endpointer takes it precomputed so it does no numpy of its own.
            levels = np.sqrt((samples ** 2).mean(axis=1))
            self.delivered = True
            # The whole batch as one item. Queueing block by block woke the
            # consumer 50 times a second, which on its own still cost 8.7% of
            # playback even once this thread had stopped doing that.
            self.blocks.put((samples, levels))

    def _watch(self) -> None:
        for line in self.proc.stderr:
            text = line.decode("utf-8", "replace").strip()
            # PipeWire complains about RTKit on every start here; it is a
            # priority it did not get, not a fault, and saying so is noise.
            if text and "RTKit" not in text and "rt.c" not in text:
                self.stderr.append(text)

    @property
    def alive(self) -> bool:
        return self.proc.poll() is None

    def why(self) -> str:
        return "; ".join(self.stderr[-3:]) or f"exited {self.proc.returncode}"

    def flush(self) -> None:
        while True:
            try:
                self.blocks.get_nowait()
            except queue.Empty:
                return

    def stop(self) -> None:
        end(self.proc)


class Speaker:
    """pw-play in a pipe, fed from a thread so the socket is never held up.

    Also keeps the estimate of when the noise stops, which is what the mic is
    muted against.
    """

    def __init__(self, target: str | None, rate: int) -> None:
        command = [
            "pw-play", "--raw",
            "--rate", str(rate),
            "--channels", "1",
            "--format", "s16",
            "--latency", f"{PLAY_LATENCY_MS}ms",
        ]
        if target:
            command += ["--target", target]
        command.append("-")
        self.proc = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            bufsize=0,
            preexec_fn=die_with_parent,
        )
        widen_pipe(self.proc.stdin)
        self.rate = rate
        self.queue: queue.Queue[bytes | None] = queue.Queue()
        self.plays_until = 0.0
        self.broken = False
        self.created = time.monotonic()
        threading.Thread(target=self._pump, daemon=True).start()
        self.queue.put(b"\x00\x00" * int(rate * PRIME_S))

    @property
    def warm(self) -> bool:
        """Has the stream had long enough to actually be running?"""
        return time.monotonic() - self.created >= COLD_LATENCY_S

    def _pump(self) -> None:
        while True:
            chunk = self.queue.get()
            if chunk is None:
                return
            try:
                # bufsize=0 makes stdin a raw FileIO, whose write() is one
                # write(2) and may return a short count -- a signal arriving
                # mid-write is enough. Dropping the remainder would truncate a
                # sentence silently, so loop until it is all gone.
                view = memoryview(chunk)
                while view:
                    written = self.proc.stdin.write(view)
                    if written is None:
                        # Raw non-blocking write that would have blocked. Not an
                        # error and not a short write -- retry the same buffer.
                        time.sleep(0.005)
                        continue
                    if written == 0:
                        raise BrokenPipeError("pw-play accepted nothing")
                    view = view[written:]
            except (BrokenPipeError, ValueError, OSError) as error:
                # Losing this thread silently drops the rest of the reply, which
                # looks like the speaker cutting out mid-sentence. Say so.
                print(f"  playback stopped: {error!r}", file=sys.stderr)
                self.broken = True
                return

    def play(self, pcm: bytes) -> None:
        """Queue one sentence and push the "still talking" estimate out."""
        now = time.monotonic()
        seconds = len(pcm) / 2 / self.rate
        latency = PLAY_LATENCY_S if self.warm else COLD_LATENCY_S
        self.plays_until = max(self.plays_until, now + latency) + seconds
        self.queue.put(pcm)

    def stop(self) -> None:
        self.queue.put(None)
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        end(self.proc)


class Link:
    """The socket plus a thread that is always reading it.

    The reason it cannot just be read during a turn: uvicorn pings every 20s and
    hangs up if nothing comes back, and a conversation spends most of its life
    idle between turns. Nothing was answering those pings, so the service closed
    the connection and the next utterance died with `Broken pipe`. `recv` deals
    with pings itself, so a thread sitting in it is all that is needed.
    """

    def __init__(self, url: str) -> None:
        self.ws = WebSocket(url)
        self.inbox: queue.Queue[tuple[bool, bytes] | None] = queue.Queue()
        self.error: Exception | None = None
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        try:
            while True:
                message = self.ws.recv()
                if message is None:
                    raise WebSocketError("the service closed the connection")
                self.inbox.put(message)
        except Exception as error:
            self.error = error
            self.inbox.put(None)

    @property
    def alive(self) -> bool:
        return self.error is None

    def get(self, timeout: float) -> tuple[bool, bytes]:
        try:
            message = self.inbox.get(timeout=timeout)
        except queue.Empty:
            raise WebSocketError(f"no reply from the service in {timeout:.0f}s")
        if message is None:
            raise self.error or WebSocketError("connection closed")
        return message

    def send_bytes(self, data: bytes) -> None:
        self.ws.send_bytes(data)

    def send_text(self, text: str) -> None:
        self.ws.send_text(text)

    def close(self) -> None:
        self.ws.close()


def turn(link: Link, utterance: np.ndarray, state: dict) -> None:
    """Send one utterance and play the reply, printing the conversation."""
    pcm = (np.clip(utterance, -1.0, 1.0) * 32767).astype("<i2").tobytes()
    # In pieces, not one frame. Masking is a single big-integer XOR, so a
    # 15-second utterance is a ~500KB integer operation that holds the GIL for
    # hundreds of milliseconds on an ARMv6 -- long enough to punch holes in
    # whatever the speaker is playing. The service appends binary frames to a
    # buffer until {"type":"end"}, so splitting the send changes nothing it sees.
    for start in range(0, len(pcm), SEND_CHUNK):
        link.send_bytes(pcm[start:start + SEND_CHUNK])
    link.send_text(json.dumps({"type": "end"}))

    while True:
        is_binary, payload = link.get(TURN_TIMEOUT_S)

        if is_binary:
            speaker = state.get("speaker")
            if speaker is not None and not speaker.broken:
                speaker.play(payload)
                state["muted_until"] = speaker.plays_until + MUTE_TAIL_S
            continue

        event = json.loads(payload)
        kind = event.get("type")
        if kind == "stt":
            print("  (nothing heard)" if event.get("empty") else f"you: {event['text']}")
        elif kind == "start":
            speaker = state.get("speaker")
            # Normally already built and warm. Rebuilt only if pw-play died --
            # switching the JBL off and on again takes its node with it -- or if
            # the service is synthesising at a rate we did not expect.
            stale = (
                speaker is None
                or speaker.broken
                or speaker.proc.poll() is not None
                or speaker.rate != event["rate"]
            )
            if stale:
                if speaker is not None:
                    speaker.stop()
                state["speaker"] = Speaker(state["speaker_target"], event["rate"])
        elif kind == "text":
            print(f"bot: {event['text']}")
        elif kind == "error":
            print(f"  error: {event['message']}", file=sys.stderr)
            return
        elif kind == "done":
            stats = event.get("stats") or {}
            if stats.get("first_audio_ms"):
                print(
                    f"  [stt {stats['stt_ms']}ms, "
                    f"first audio {stats['first_audio_ms']}ms, "
                    f"total {stats['total_ms']}ms]\n"
                )
            state["muted_until"] = max(
                state.get("muted_until", 0.0), time.monotonic() + 0.25
            )
            return


def converse(url: str, recorder: Recorder, state: dict) -> None:
    """One connection's worth of conversation. Returns when the link goes."""
    link = Link(url)
    try:
        print(f"connected to {url} -- just talk, Ctrl-C to quit\n")
        endpointer = Endpointer()
        muted = False

        while True:
            if not recorder.alive:
                raise SystemExit(f"the microphone stopped: {recorder.why()}")
            if not link.alive:
                raise link.error or WebSocketError("connection closed")
            try:
                blocks, levels = recorder.blocks.get(
                    timeout=IDLE_TIMEOUT_S if recorder.delivered else FIRST_BLOCK_TIMEOUT_S
                )
            except queue.Empty:
                raise WebSocketError("no audio from the microphone")

            for index in range(len(levels)):
                if time.monotonic() < state.get("muted_until", 0.0):
                    muted = True  # drop it; this is the assistant's own voice
                    continue
                if muted:
                    # Everything captured while it was speaking is stale by now,
                    # including the rest of this batch, and the endpointer may be
                    # half way into an "utterance" that is really the reply
                    # leaking back in.
                    recorder.flush()
                    endpointer.reset()
                    muted = False
                    break

                utterance = endpointer.push(blocks[index], float(levels[index]))
                if utterance is not None:
                    turn(link, utterance, state)
    finally:
        link.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--mic", default=DEFAULT_MIC, help='node name, or "auto"')
    parser.add_argument("--speaker", default=DEFAULT_SPEAKER, help='node name, or "auto"')
    parser.add_argument("--list", action="store_true", help="show PipeWire nodes and exit")
    args = parser.parse_args()

    if args.list:
        for media, node in nodes():
            print(f"  {media:<22} {node}")
        return

    stop_on_sigterm()
    mic = require(args.mic, "source")
    target = require(args.speaker, "sink")
    # Built now rather than on the first reply: establishing an A2DP stream costs
    # two seconds, and paid mid-turn it lands inside the window the microphone is
    # supposed to be muted for.
    state = {"speaker_target": target, "speaker": Speaker(target, OUT_RATE),
             "muted_until": 0.0}

    recorder = Recorder(mic)
    try:
        while True:
            try:
                converse(args.url, recorder, state)
            except (OSError, WebSocketError) as error:
                print(f"  link lost ({error}); retrying in {RETRY_S:.0f}s",
                      file=sys.stderr)
                # muted_until is deliberately left alone: the JBL is still
                # speaking whatever was already queued, and clearing it here let
                # the reconnected mic hear the tail of the reply and answer it.
                time.sleep(RETRY_S)
    except (KeyboardInterrupt, Stopping):
        print("\nbye")
    finally:
        recorder.stop()
        if state.get("speaker") is not None:
            state["speaker"].stop()


if __name__ == "__main__":
    main()
