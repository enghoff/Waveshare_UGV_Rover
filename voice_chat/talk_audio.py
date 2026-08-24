"""Microphone, speaker, and the terminal status line for talk.py."""
from __future__ import annotations

import base64
import sys
import threading
from typing import Any

import numpy as np

# Optional, for the same reason it is optional in talk.py: this module is
# imported on the rover, which has no sound card, purely for `Speaker`'s
# interface and the two PCM converters. Everything that needs the library is
# behind `start`, which says so.
try:
    import sounddevice as sd
except ImportError:
    sd = None

from endpointing import IN_RATE

OUT_RATE = 24000

# The dot is only used where the terminal can encode it: a redirected stdout on
# Windows is cp1252, and U+25CF would raise UnicodeEncodeError on the first
# update -- an indicator that kills the conversation it is decorating.
try:
    "●○".encode(sys.stdout.encoding or "ascii")
    STATUS = {
        "listening": "● listening",
        "hearing": "● hearing you",
        "thinking": "○ thinking",
        "speaking": "○ speaking",
    }
except (UnicodeEncodeError, LookupError):
    STATUS = {
        "listening": "[ listening ]",
        "hearing": "[ hearing you ]",
        "thinking": "[ thinking ]",
        "speaking": "[ speaking ]",
    }
STATUS_WIDTH = max(len(text) for text in STATUS.values())


class Indicator:
    """One line at the bottom of the terminal saying whether the mic is open.

    Rewritten in place and rubbed out before anything else prints, so the
    transcript above it stays a transcript. Everything the conversation says goes
    through `say` for that reason.

    Silent when stdout is not a terminal: `\\r` is just a character in a log file,
    and a status that changes forty times a second would be most of the log.
    """

    def __init__(self) -> None:
        self.on = sys.stdout.isatty()
        self.shown = ""

    def set(self, state: str) -> None:
        text = STATUS[state]
        if not self.on or text == self.shown:
            return
        sys.stdout.write("\r" + text.ljust(STATUS_WIDTH))
        sys.stdout.flush()
        self.shown = text

    def clear(self) -> None:
        if self.on and self.shown:
            sys.stdout.write("\r" + " " * STATUS_WIDTH + "\r")
            sys.stdout.flush()
        # Forgotten rather than remembered, so the next `set` redraws it below
        # whatever was just printed instead of deciding nothing has changed.
        self.shown = ""

    def say(self, text: str, err: bool = False) -> None:
        self.clear()
        print(text, file=sys.stderr if err else sys.stdout, flush=True)

    def __enter__(self) -> "Indicator":
        return self

    def __exit__(self, *_exc) -> None:
        self.clear()

def _b64(raw: bytes) -> str:
    return base64.b64encode(raw).decode("ascii")


def _to_pcm16(audio: np.ndarray) -> bytes:
    """Float samples as the 16kHz mono s16le the service reads."""
    return (np.clip(audio, -1.0, 1.0) * 32767).astype("<i2").tobytes()


def _from_pcm16(raw: bytes) -> np.ndarray:
    return np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0



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
        if sd is None:
            raise RuntimeError(
                "there is no sounddevice on this machine, so there is no sound "
                "card to open. A Speaker that is never started still counts what "
                "it was given, which is what the tests use and what the rover's "
                "browser-backed speaker replaces.")
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
